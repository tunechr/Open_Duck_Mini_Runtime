# Open Duck Mini Runtime

## Raspberry Pi zero 2W setup

### Install Raspberry Pi OS

Download Raspberry Pi OS Lite (64-bit) from here : https://www.raspberrypi.com/software/operating-systems/

Follow the instructions here to install the OS on the SD card : https://www.raspberrypi.com/documentation/computers/getting-started.html

With the Raspberry Pi Imager, you can pre-configure session, wifi and ssh. Do it like below :

![imager_setup](https://github.com/user-attachments/assets/7a4987b2-de83-41dd-ab7f-585259685f16)

> Tip: I configure the rasp to connect to my phone's hotspot, this way I can connect to it from anywhere.

### Setup SSH (If not setup during the installation)

When first booting on the rasp, you will need to connect a screen and a keyboard. The first thing you should do is connect to a wifi network and enable SSH.

To do so, you can follow this guide : https://www.raspberrypi.com/documentation/computers/configuration.html#setting-up-wifi

Then, you can connect to your rasp using SSH without having to plug a screen and a keyboard.

### Update the system and install necessary stuff

```bash
sudo apt update
sudo apt upgrade
sudo apt install git
sudo apt install python3-pip
sudo apt install python3-virtualenvwrapper
(optional) sudo apt install python3-picamzero

```

Add this to the end of the `.bashrc`:

```bash
export WORKON_HOME=$HOME/.virtualenvs
export PROJECT_HOME=$HOME/Devel
source /usr/share/virtualenvwrapper/virtualenvwrapper.sh
```

### Enable I2C

`sudo raspi-config` -> `Interface Options` -> `I2C`

TODO set 400KHz ?

### Set the usbserial latency timer

```bash
cd  /etc/udev/rules.d/
sudo touch 99-usb-serial.rules
sudo nano 99-usb-serial.rules
# copy the following line in the file
SUBSYSTEM=="usb-serial", DRIVER=="ftdi_sio", ATTR{latency_timer}="1"
```

### Set the udev rules for the motor control board

TODO


### Setup xbox one controller over bluetooth

Turn your xbox one controller on and set it in pairing mode by long pressing the sync button on the top of the controller.

Run the following commands on the rasp :

```bash
bluetoothctl
scan on
```

Wait for the controller to appear in the list, then run :

```bash
pair <controller_mac_address>
trust <controller_mac_address>
connect <controller_mac_address>
```

10:18:49:98:41:14


The led on the controller should stop blinking and stay on.

You can test that it's working by running

```bash
python3 mini_bdx_runtime/mini_bdx_runtime/xbox_controller.py
```

## Speaker wiring and configuration
Follow this tutorial

> For now, don't activate `/dev/zero` when they ask

https://learn.adafruit.com/adafruit-max98357-i2s-class-d-mono-amp?view=all


## Install the runtime

### Make a virtual environment and activate it

```bash
mkvirtualenv -p python3 open-duck-mini-runtime
workon open-duck-mini-runtime
```

Clone this repository on your rasp, cd into the repo, then :

```bash
git clone https://github.com/apirrone/Open_Duck_Mini_Runtime
cd Open_Duck_Mini_Runtime
git checkout v2
pip install -e .
```

In Raspberry Pi 5, you need to perform the following operations

```bash
pip uninstall -y RPi.GPIO
pip install lgpio
```


## Test the IMU

```bash
python3 mini_bdx_runtime/mini_bdx_runtime/raw_imu.py
```

You can also run `python3 scripts/imu_server.py` on the robot and `python3 scripts/imu_client.py --ip <robot_ip>` on your computer to check that the frame is oriented correctly. 

> To find the ip address of the robot, run `ifconfig` on the robot

## Test motors

This will allow you to verify all your motors are connected and configured.

```bash
python3 scripts/check_motors.py
```

## Make your duck_config.json

Copy `example_config.json` in the home directory of your duck and rename it `duck_config.json`.

`cp example_config.json ~/duck_config.json`

In this file, you can configure some stuff, like registering if you installed the expression features, installed the imu upside down or and other stuff. You also write the joints offsets of your duck here

## Find the joints offsets

This script will guide you through finding the joints offsets of your robot that you can then write in your `duck_config.json`

> This procedure won't be necessary in the future as we will be flashing the offsets directly in each motor's eeprom.

```bash
cd scripts/
python find_soft_offsets.py
```

## Run the walk !

Download the [latest policy checkpoint ](https://github.com/apirrone/Open_Duck_Mini/blob/v2/BEST_WALK_ONNX_2.onnx) and copy it to your duck.

`cd scripts/`

`python v2_rl_walk_mujoco.py --onnx_model_path <path_to>/BEST_WALK_ONNX_2.onnx`



```
- The commands are : 
- A to pause/unpause
- X to turn on/off the projector
- B to play a random sound
- Y to turn on/off head control (very experimental, I don't recommend trying that, it can break your duck's head)
- left and right triggers to control the left and right antennas
- LB (new!) press and hold to increase the walking frequency, kind of a sprint mode 🙂
```

## Troubleshooting I2C
A useful way to check for I2C issues.
Scanning for I2C devices using  i2cdetect.

```
sudo apt-get install i2c-tools
i2cdetect -y 1
```

On modern Raspberry Pi OS releases, you do not need to run the command with sudo. The -y disables interactive mode, so it just goes ahead and scans. The 1 specifies the I2C bus.



https://learn.adafruit.com/scanning-i2c-addresses/raspberry-pi

## Camera Test
Error : ModuleNotFoundError: No module named 'libcamera'

reinstall: 

sudo apt-get install --reinstall libcamera-apps
```

## Controllers (Xbox, PS5, Keyboard, Virtual)

You can drive the robot using different controllers. Selection can be done via CLI or `duck_config.json`.

- Supported types: `xbox`, `ps5` (aka `playstation5`/`dualsense`), `keyboard`, `virtual`, or `auto` (auto-detect; defaults to virtual if none).
- Configure in JSON: set `controller_type` and optionally `auto_detect_controller`.
- CLI override: `--controller-type <type>`.

Keyboard Controller (when using `--controller-type keyboard`):

- Movement: arrows (Up/Down = forward/back; Left/Right = lateral)
- Yaw: `,` or `L` = left, `.` or `;` = right
- Head: `H/L` yaw (-/+), `J/K` pitch (+/-), `U/O` roll (-/+)
- Buttons: `A/S/D/F` map to Xbox `A/B/X/Y`
- Bumpers: `Q/W` map to `LB/RB`
- D-Pad: `Z/X` up/down
- Triggers: Left/Right Shift as analog 1.0 while held

Virtual Controller:
- Safe idle by default (no movement/head/inputs)
Optional behaviors via `virtual_controller_settings` in config:

- `look_around_interval`, `movement_interval`
- `enable_movement`, `enable_head_movement`, `simulate_buttons`, `simulate_triggers`

## IMU: client/server and BNO08x/BNO085 support

Run the IMU server on the robot:

```bash
python3 scripts/imu_server.py --imu-type <bno055|icm20948|bno08x|bno085> --upside-down --calibrate --use-mag
```

Notes:

- `--use-mag` applies to BNO08x only (uses Rotation Vector with magnetometer; omit for Game Rotation Vector).
- `--upside-down` flips axes for inverted mounting.
- `--calibrate` triggers the device’s calibration flow.

Connect from your computer with the IMU client:

```bash
python3 scripts/imu_client.py --ip <robot_ip> [--freq 30] [--console-only] [--force-qt] [--no-gui] [--raw-mode]
```

Viewer fallback order:

1) FramesViewer (if installed) → 2) PyQtGraph 3D axes viewer → 3) Console (Euler + quaternion)

## Config file (`duck_config.json`)

Start from `example_config.json` and copy it to your home directory on the robot:

```bash
cp example_config.json ~/duck_config.json
```

Key fields:

- `start_paused` (bool)
- `imu_upside_down` (bool)
- `phase_frequency_factor_offset` (float)
- `expression_features` (eyes, projector, antennas, speaker, microphone, camera)
- `joints_offsets` (per-joint offsets)
- Controller selection:
	- `controller_type`: `xbox` | `ps5` | `keyboard` | `virtual` | `auto`
	- `auto_detect_controller`: true/false
- Virtual controller tuning (optional):
	- `virtual_controller_settings`: `{ "look_around_interval": 10.0, "movement_interval": 30.0, "enable_movement": false, "enable_head_movement": false, "simulate_buttons": false, "simulate_triggers": false }`

## Run the walk with a controller

Download the ONNX policy (see link above), then run:

```bash
cd scripts
python3 v2_rl_walk_mujoco.py --onnx_model_path <path>/BEST_WALK_ONNX_2.onnx --controller-type auto
```

Examples:

- Force keyboard: `--controller-type keyboard`
- Force virtual (idle): `--controller-type virtual`
- Force Xbox/PS5: `--controller-type xbox` or `--controller-type ps5`

During run (Xbox mapping):

- A: pause/unpause
- X: projector toggle
- B: play random sound
- Y: head control toggle (experimental)
- LB (hold): sprint (increase walking frequency)
- Left/Right triggers: control antennas (if installed)
