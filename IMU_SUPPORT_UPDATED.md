# IMU Support Documentation

This project now supports two types of IMU sensors across two different modules:

## IMU Modules

### 1. `raw_imu.py` - Raw Sensor Data
- **Purpose**: Raw gyroscope and accelerometer readings
- **Output**: `{"gyro": [x,y,z], "accelero": [x,y,z]}`
- **Use case**: Motion detection, sensor debugging, custom fusion algorithms

### 2. `imu.py` - Orientation/Quaternion Data  
- **Purpose**: Processed orientation data (quaternions, euler angles)
- **Output**: Quaternion `[x,y,z,w]` or euler angles
- **Use case**: Robot balance control, attitude estimation, pose tracking

## Supported IMU Sensors

### 1. BNO055 (Original)
- **Library**: `adafruit-circuitpython-bno055`
- **Features**: Built-in sensor fusion, automatic calibration
- **Default**: This is the default IMU type
- **Available in**: Both `raw_imu.py` and `imu.py`

### 2. ICM20948 (New)
- **Library**: `adafruit-circuitpython-icm20x`
- **Features**: 9-axis motion tracking, manual calibration
- **Available in**: Both `raw_imu.py` and `imu.py`
- **Orientation**: Uses complementary filter for quaternion estimation

## Usage Examples

### Raw Sensor Data (`raw_imu.py`)

```bash
# BNO055 raw data (default)
python mini_bdx_runtime/mini_bdx_runtime/raw_imu.py

# ICM20948 raw data
python mini_bdx_runtime/mini_bdx_runtime/raw_imu.py --imu-type icm20948

# With upside-down orientation
python mini_bdx_runtime/mini_bdx_runtime/raw_imu.py --imu-type icm20948 --upside-down
```

### Orientation Data (`imu.py`)

```bash
# BNO055 quaternions (default)
python mini_bdx_runtime/mini_bdx_runtime/imu.py

# ICM20948 quaternions
python mini_bdx_runtime/mini_bdx_runtime/imu.py --imu-type icm20948

# Euler angles output
python mini_bdx_runtime/mini_bdx_runtime/imu.py --imu-type icm20948 --output euler

# With upside-down orientation
python mini_bdx_runtime/mini_bdx_runtime/imu.py --imu-type icm20948 --upside-down
```

### Running Calibration

```bash
# BNO055 calibration (both modules)
python mini_bdx_runtime/mini_bdx_runtime/raw_imu.py --calibrate
python mini_bdx_runtime/mini_bdx_runtime/imu.py --calibrate

# ICM20948 calibration (both modules)
python mini_bdx_runtime/mini_bdx_runtime/raw_imu.py --imu-type icm20948 --calibrate
python mini_bdx_runtime/mini_bdx_runtime/imu.py --imu-type icm20948 --calibrate
```

## Command Line Arguments

### Both modules support:
- `--imu-type {bno055,icm20948}`: Choose the IMU type (default: bno055)
- `--upside-down`: Set if IMU is mounted upside down
- `--calibrate`: Run calibration routine

### Additional for `imu.py`:
- `--output {quat,euler}`: Output format - quaternion or euler angles (default: quat)

## Integration in Other Scripts

### Raw Sensor Data

```python
from mini_bdx_runtime.mini_bdx_runtime.raw_imu import Imu, ImuICM20948

# Choose your IMU type for raw data
imu = ImuICM20948(sampling_freq=50, upside_down=False)  # or Imu(...)

# Get raw sensor data
data = imu.get_data()
print(f"Gyro: {data['gyro']}")        # [x, y, z] rad/s
print(f"Accelerometer: {data['accelero']}")  # [x, y, z] m/s²
```

### Orientation Data

```python
from mini_bdx_runtime.mini_bdx_runtime.imu import Imu, ImuICM20948

# Choose your IMU type for orientation
imu = ImuICM20948(sampling_freq=50, upside_down=False)  # or Imu(...)

# Get orientation data
quat = imu.get_data()                    # Quaternion [x, y, z, w]
euler = imu.get_data(euler=True)         # Euler angles [roll, pitch, yaw] in radians
rotation_matrix = imu.get_data(mat=True) # 3x3 rotation matrix
```

## Calibration

### BNO055 Calibration
- Uses the built-in calibration system
- Automatically saves calibration data to `imu_calib_data.pkl`
- Follow the on-screen instructions during calibration

### ICM20948 Calibration
- Uses a simple offset-based calibration
- Keep the IMU stationary during the 5-second calibration period
- Automatically saves calibration data to `icm20948_calib_data.pkl`

## ICM20948 Orientation Estimation

The ICM20948 implementation in `imu.py` uses a **complementary filter** for orientation estimation:

- **Accelerometer**: Provides roll/pitch estimation (gravity reference)
- **Gyroscope**: Provides angular velocity integration
- **Filter**: Combines both with `alpha=0.98` (98% gyro, 2% accel)
- **No magnetometer fusion**: Yaw drifts over time (simple implementation)

For applications requiring absolute yaw reference, consider using the BNO055 which has built-in magnetometer fusion.

## Axis Remapping

Both IMU implementations support axis remapping for different mounting orientations:
- **upside_down=True**: Remaps and negates all axes for inverted mounting
- **upside_down=False**: Standard orientation with minimal remapping

The axis remapping ensures consistent coordinate system regardless of physical mounting.
