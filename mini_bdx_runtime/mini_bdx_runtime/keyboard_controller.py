import time
from threading import Thread
from queue import Queue

import numpy as np

try:
    import pygame
    PYGAME_AVAILABLE = True
except Exception:
    PYGAME_AVAILABLE = False

from mini_bdx_runtime.buttons import Buttons

X_RANGE = [-0.15, 0.15]
Y_RANGE = [-0.2, 0.2]
YAW_RANGE = [-1.0, 1.0]

# rads
NECK_PITCH_RANGE = [-0.34, 1.1]
HEAD_PITCH_RANGE = [-0.78, 0.3]
HEAD_YAW_RANGE = [-0.5, 0.5]
HEAD_ROLL_RANGE = [-0.5, 0.5]


class KeyboardController:
    """Keyboard-based controller mirroring XBoxController interface.
    - Arrow keys: body movement (x, y, yaw)
    - H/J/K/L: head yaw (-/+) and pitch (+/-)
    - U/O: head roll (-/+)
    - A/S/D/F: A/B/X/Y buttons
    - Q/W: LB/RB
    - Z/X: dpad up/down
    - Left/Right Shift: triggers (L/R) as analog 0..1 while held
    Toggle head-control mode with Y (same as Xbox Y)
    """

    def __init__(self, command_freq, only_head_control=False):
        self.command_freq = command_freq
        self.head_control_mode = only_head_control
        self.only_head_control = only_head_control

        self.last_commands = [0.0] * 7
        self.last_left_trigger = 0.0
        self.last_right_trigger = 0.0

        self.buttons = Buttons()
        self.cmd_queue = Queue(maxsize=1)

        if PYGAME_AVAILABLE:
            pygame.init()
            pygame.display.set_mode((200, 100))

        Thread(target=self.commands_worker, daemon=True).start()

    def commands_worker(self):
        while True:
            self.cmd_queue.put(self.get_commands())
            time.sleep(1 / self.command_freq)

    def _get_keystate(self):
        if not PYGAME_AVAILABLE:
            return None
        pygame.event.pump()
        return pygame.key.get_pressed()

    def get_commands(self):
        last = self.last_commands
        left_trigger = self.last_left_trigger
        right_trigger = self.last_right_trigger

        keys = self._get_keystate() if PYGAME_AVAILABLE else None

        move_x = move_y = yaw = 0.0
        head_yaw = head_pitch = head_roll = 0.0

        if keys:
            # Movement: arrow keys
            if keys[pygame.K_UP]:
                move_x += abs(X_RANGE[1])
            if keys[pygame.K_DOWN]:
                move_x -= abs(X_RANGE[0])
            if keys[pygame.K_RIGHT]:
                move_y += abs(Y_RANGE[1])
            if keys[pygame.K_LEFT]:
                move_y -= abs(Y_RANGE[0])

            if keys[pygame.K_PERIOD] or keys[pygame.K_SEMICOLON]:
                yaw += abs(YAW_RANGE[1])
            if keys[pygame.K_COMMA] or keys[pygame.K_l]:
                yaw -= abs(YAW_RANGE[0])

            # Head: H/J/K/L for yaw/pitch, U/O for roll
            if keys[pygame.K_h]:
                head_yaw -= abs(HEAD_YAW_RANGE[0])
            if keys[pygame.K_l]:
                head_yaw += abs(HEAD_YAW_RANGE[1])
            if keys[pygame.K_k]:
                head_pitch += abs(HEAD_PITCH_RANGE[1])
            if keys[pygame.K_j]:
                head_pitch -= abs(HEAD_PITCH_RANGE[0])
            if keys[pygame.K_u]:
                head_roll -= abs(HEAD_ROLL_RANGE[0])
            if keys[pygame.K_o]:
                head_roll += abs(HEAD_ROLL_RANGE[1])

            # Buttons: A/S/D/F ~ A/B/X/Y
            A_pressed = keys[pygame.K_a]
            B_pressed = keys[pygame.K_s]
            X_pressed = keys[pygame.K_d]
            Y_pressed = keys[pygame.K_f]

            # Bumpers: Q/W
            LB_pressed = keys[pygame.K_q]
            RB_pressed = keys[pygame.K_w]

            # D-pad up/down: Z/X
            dpad_up = keys[pygame.K_z]
            dpad_down = keys[pygame.K_x]

            # Triggers: Left/Right Shift as analog 1.0 when held
            left_trigger = 1.0 if keys[pygame.K_LSHIFT] else 0.0
            right_trigger = 1.0 if keys[pygame.K_RSHIFT] else 0.0

            # Toggle head control with Y (align with Xbox Y)
            if Y_pressed and not self.only_head_control:
                self.head_control_mode = not self.head_control_mode

            self.buttons.update(
                A_pressed,
                B_pressed,
                X_pressed,
                Y_pressed,
                LB_pressed,
                RB_pressed,
                dpad_up,
                dpad_down,
            )
        else:
            # No pygame: keep everything idle
            self.buttons.update(False, False, False, False, False, False, False, False)

        if not self.head_control_mode:
            last[0] = move_x
            last[1] = move_y
            last[2] = yaw
        else:
            last[0] = 0.0
            last[1] = 0.0
            last[2] = 0.0
            last[3] = 0.0
            last[4] = head_pitch
            last[5] = head_yaw
            last[6] = head_roll

        return (
            np.around(last, 3),
            self.buttons.A.is_pressed,
            self.buttons.B.is_pressed,
            self.buttons.X.is_pressed,
            self.buttons.Y.is_pressed,
            self.buttons.LB.is_pressed,
            self.buttons.RB.is_pressed,
            left_trigger,
            right_trigger,
            1 if self.buttons.dpad_up.is_pressed else (-1 if self.buttons.dpad_down.is_pressed else 0),
        )

    def get_last_command(self):
        A_pressed = False
        B_pressed = False
        X_pressed = False
        Y_pressed = False
        LB_pressed = False
        RB_pressed = False
        up_down = 0
        try:
            (
                self.last_commands,
                A_pressed,
                B_pressed,
                X_pressed,
                Y_pressed,
                LB_pressed,
                RB_pressed,
                self.last_left_trigger,
                self.last_right_trigger,
                up_down,
            ) = self.cmd_queue.get(False)
        except Exception:
            pass

        self.buttons.update(
            A_pressed,
            B_pressed,
            X_pressed,
            Y_pressed,
            LB_pressed,
            RB_pressed,
            up_down == 1,
            up_down == -1,
        )

        return (
            self.last_commands,
            self.buttons,
            self.last_left_trigger,
            self.last_right_trigger,
        )


if __name__ == "__main__":
    kc = KeyboardController(20)
    while True:
        print(kc.get_last_command())
        time.sleep(0.05)
