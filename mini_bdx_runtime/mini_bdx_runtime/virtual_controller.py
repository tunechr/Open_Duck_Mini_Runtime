import time
import math
from threading import Thread
from queue import Queue

try:
    import numpy as np
    NUMPY_AVAILABLE = True
except ImportError:
    NUMPY_AVAILABLE = False
    np = None
    print("Warning: numpy not available, using basic math")

try:
    import pygame
    PYGAME_AVAILABLE = True
except ImportError:
    PYGAME_AVAILABLE = False
    print("Warning: pygame not available, using basic timing")

try:
    from mini_bdx_runtime.buttons import Buttons
except ImportError:
    # Fallback if buttons module not available
    class Buttons:
        def __init__(self):
            pass
        def update(self, *args):
            pass


X_RANGE = [-0.15, 0.15]
Y_RANGE = [-0.2, 0.2]
YAW_RANGE = [-1.0, 1.0]

# rads
NECK_PITCH_RANGE = [-0.34, 1.1]
HEAD_PITCH_RANGE = [-0.78, 0.3]
HEAD_YAW_RANGE = [-0.5, 0.5]
HEAD_ROLL_RANGE = [-0.5, 0.5]


class VirtualController:
    """
    Virtual controller. Idle by default (no movement, no head motion, no button/trigger events).
    Enable behaviors via flags when instantiating.
    """

    def __init__(
        self,
        command_freq,
        only_head_control=False,
        look_around_interval=10.0,
        movement_interval=30.0,
        enable_movement=False,
        enable_head_movement=False,
        simulate_buttons=False,
        simulate_triggers=False,
    ):
        self.command_freq = command_freq
        self.head_control_mode = only_head_control
        self.only_head_control = only_head_control
        self.look_around_interval = look_around_interval
        self.movement_interval = movement_interval
        self.enable_movement = enable_movement
        self.enable_head_movement = enable_head_movement
        self.simulate_buttons = simulate_buttons
        self.simulate_triggers = simulate_triggers

        self.last_commands = [0.0] * 7
        self.last_left_trigger = 0.0
        self.last_right_trigger = 0.0

        if PYGAME_AVAILABLE:
            pygame.init()

        print(
            f"Loaded Virtual Controller - Head move: {self.enable_head_movement}, Body move: {self.enable_movement}"
        )

        self.cmd_queue = Queue(maxsize=1)

        # Virtual button states
        self.cross_pressed = False
        self.circle_pressed = False
        self.square_pressed = False
        self.triangle_pressed = False
        self.L1_pressed = False
        self.R1_pressed = False

        self.buttons = Buttons()

        # Behavior state variables
        self.start_time = time.time()
        self.last_head_movement_time = 0.0
        self.last_body_movement_time = 0.0
        self.current_head_target = [0.0, 0.0, 0.0]  # [yaw, pitch, roll]
        self.current_body_target = [0.0, 0.0, 0.0]  # [x, y, yaw]

        # Patterns
        self.head_patterns = [
            [0.3, 0.0, 0.0],
            [-0.3, 0.0, 0.0],
            [0.0, 0.2, 0.0],
            [0.0, -0.2, 0.0],
            [0.2, 0.1, 0.0],
            [-0.2, 0.1, 0.0],
            [0.0, 0.0, 0.0],
        ]

        self.body_patterns = [
            [0.05, 0.0, 0.0],
            [0.0, 0.05, 0.0],
            [0.0, -0.05, 0.0],
            [-0.02, 0.0, 0.0],
            [0.0, 0.0, 0.3],
            [0.0, 0.0, -0.3],
            [0.0, 0.0, 0.0],
        ]

        Thread(target=self.commands_worker, daemon=True).start()

    def commands_worker(self):
        while True:
            self.cmd_queue.put(self.get_commands())
            time.sleep(1 / self.command_freq)

    def get_commands(self):
        current_time = time.time()
        elapsed_time = current_time - self.start_time

        # Head movement
        if self.enable_head_movement and (
            current_time - self.last_head_movement_time >= self.look_around_interval
        ):
            self.last_head_movement_time = current_time
            idx = int(elapsed_time / self.look_around_interval) % len(self.head_patterns)
            self.current_head_target = self.head_patterns[idx].copy()
            print(f"Virtual Controller: Head movement to {self.current_head_target}")

        # Body movement
        if (
            self.enable_movement
            and not self.only_head_control
            and current_time - self.last_body_movement_time >= self.movement_interval
        ):
            self.last_body_movement_time = current_time
            idx = int(elapsed_time / self.movement_interval) % len(self.body_patterns)
            self.current_body_target = self.body_patterns[idx].copy()
            print(f"Virtual Controller: Body movement to {self.current_body_target}")

        # Smooth towards targets
        last = self.last_commands.copy()
        if not self.head_control_mode and self.enable_movement:
            alpha = 0.1
            last[0] = self._smooth_interpolate(last[0], self.current_body_target[0], alpha)
            last[1] = self._smooth_interpolate(last[1], self.current_body_target[1], alpha)
            last[2] = self._smooth_interpolate(last[2], self.current_body_target[2], alpha)
        else:
            last[0] = 0.0
            last[1] = 0.0
            last[2] = 0.0
            last[3] = 0.0
            if self.enable_head_movement:
                alpha = 0.05
                last[4] = self._smooth_interpolate(last[4], self.current_head_target[1], alpha)
                last[5] = self._smooth_interpolate(last[5], self.current_head_target[0], alpha)
                last[6] = self._smooth_interpolate(last[6], self.current_head_target[2], alpha)
            else:
                last[4] = 0.0
                last[5] = 0.0
                last[6] = 0.0

        # Button/trigger simulation
        if self.simulate_buttons:
            self._simulate_button_events(elapsed_time)

        if self.simulate_triggers:
            left_trigger = abs(math.sin(elapsed_time * 0.1)) * 0.3
            right_trigger = abs(math.cos(elapsed_time * 0.1 + 1.57)) * 0.3
        else:
            left_trigger = 0.0
            right_trigger = 0.0

        return (
            np.around(last, 3) if np else [round(x, 3) for x in last],
            self.cross_pressed,
            self.circle_pressed,
            self.square_pressed,
            self.triangle_pressed,
            self.L1_pressed,
            self.R1_pressed,
            left_trigger,
            right_trigger,
            0,
        )

    def _smooth_interpolate(self, current, target, alpha):
        return current + alpha * (target - current)

    def _simulate_button_events(self, elapsed_time):
        self.cross_pressed = False
        self.circle_pressed = False
        self.square_pressed = False
        self.triangle_pressed = False
        self.L1_pressed = False
        self.R1_pressed = False

        if int(elapsed_time) % 60 == 0 and (elapsed_time % 1.0) < (1.0 / self.command_freq):
            if not self.only_head_control:
                self.triangle_pressed = True
                self.head_control_mode = not self.head_control_mode
                print(f"Virtual Controller: Toggled head control mode to {self.head_control_mode}")

        if int(elapsed_time) % 45 == 0 and (elapsed_time % 1.0) < (1.0 / self.command_freq):
            self.circle_pressed = True
            print("Virtual Controller: Playing random sound")

    def get_last_command(self):
        cross_pressed = False
        circle_pressed = False
        square_pressed = False
        triangle_pressed = False
        L1_pressed = False
        R1_pressed = False
        up_down = 0

        try:
            (
                self.last_commands,
                cross_pressed,
                circle_pressed,
                square_pressed,
                triangle_pressed,
                L1_pressed,
                R1_pressed,
                self.last_left_trigger,
                self.last_right_trigger,
                up_down,
            ) = self.cmd_queue.get(False)
        except Exception:
            pass

        self.buttons.update(
            cross_pressed,
            circle_pressed,
            square_pressed,
            triangle_pressed,
            L1_pressed,
            R1_pressed,
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
    # Test with different configurations
    print("Testing Virtual Controller...")
    
    # Test 1: Head-only mode (looking around every 5 seconds)
    print("\n=== Test 1: Head-only mode ===")
    controller = VirtualController(
        command_freq=20, 
        only_head_control=True,
        look_around_interval=5.0
    )
    
    for i in range(20):
        commands, buttons, left_trigger, right_trigger = controller.get_last_command()
        print(f"Commands: {commands}, Triggers: L={left_trigger:.2f}, R={right_trigger:.2f}")
        time.sleep(1.0)
    
    print("\n=== Test 2: Full movement mode ===")
    # Test 2: Full movement mode
    controller2 = VirtualController(
        command_freq=20,
        only_head_control=False,
        look_around_interval=3.0,
        movement_interval=8.0,
        enable_movement=False
    )
    
    for i in range(15):
        commands, buttons, left_trigger, right_trigger = controller2.get_last_command()
        print(f"Commands: {commands}, Mode: {'Head' if controller2.head_control_mode else 'Body'}")
        time.sleep(1.0)
