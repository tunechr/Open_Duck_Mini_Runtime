import adafruit_bno055
import adafruit_icm20x
import board
import busio
import numpy as np
import pickle
import os
import argparse

# import serial

from queue import Queue
from threading import Thread
import time
from scipy.spatial.transform import Rotation as R


# TODO filter spikes
class ImuICM20948:
    def __init__(
        self, sampling_freq, user_pitch_bias=0, calibrate=False, upside_down=True
    ):
        self.sampling_freq = sampling_freq
        self.user_pitch_bias = user_pitch_bias
        self.nominal_pitch_bias = 0
        self.calibrate = calibrate

        i2c = busio.I2C(board.SCL, board.SDA)
        self.imu = adafruit_icm20x.ICM20948(i2c)

        self.pitch_bias = self.nominal_pitch_bias + self.user_pitch_bias

        # ICM20948 doesn't have built-in quaternion fusion like BNO055
        # We'll implement a simple complementary filter for orientation estimation
        if self.calibrate:
            print("Calibrating ICM20948...")
            self._calibrate_icm20948()
            
        # Load calibration data if available
        if os.path.exists("icm20948_calib_data.pkl"):
            calib_data = pickle.load(open("icm20948_calib_data.pkl", "rb"))
            self.gyro_offsets = calib_data.get("gyro_offsets", [0, 0, 0])
            self.accel_offsets = calib_data.get("accel_offsets", [0, 0, 0])
            print("Loaded ICM20948 calibration data")
        else:
            self.gyro_offsets = [0, 0, 0]
            self.accel_offsets = [0, 0, 0]
            print("icm20948_calib_data.pkl not found")
            print("ICM20948 is running uncalibrated")

        # Apply axis remapping for upside down orientation
        self.upside_down = upside_down
        
        # Initialize orientation estimate
        self.orientation_quat = np.array([0, 0, 0, 1])  # [x, y, z, w] - identity quaternion
        self.last_time = time.time()

        self.last_imu_data = [0, 0, 0, 1]  # quaternion [x, y, z, w]
        self.imu_queue = Queue(maxsize=1)
        Thread(target=self.imu_worker, daemon=True).start()

    def _calibrate_icm20948(self):
        """Simple calibration for ICM20948 - calculates offsets while stationary"""
        print("Keep the IMU stationary for calibration...")
        gyro_samples = []
        accel_samples = []
        
        # Collect samples for 5 seconds
        start_time = time.time()
        while time.time() - start_time < 5:
            try:
                gyro_samples.append(list(self.imu.gyro))
                accel_samples.append(list(self.imu.acceleration))
                time.sleep(0.01)
            except Exception as e:
                print(f"Calibration error: {e}")
                continue
        
        # Calculate offsets
        gyro_samples = np.array(gyro_samples)
        accel_samples = np.array(accel_samples)
        
        self.gyro_offsets = np.mean(gyro_samples, axis=0).tolist()
        # For accelerometer, we expect Z to be ~9.8 (gravity), X and Y to be ~0
        accel_means = np.mean(accel_samples, axis=0)
        self.accel_offsets = [accel_means[0], accel_means[1], accel_means[2] - 9.8]
        
        # Save calibration data
        calib_data = {
            "gyro_offsets": self.gyro_offsets,
            "accel_offsets": self.accel_offsets
        }
        pickle.dump(calib_data, open("icm20948_calib_data.pkl", "wb"))
        print("ICM20948 calibration completed and saved")
        print(f"Gyro offsets: {self.gyro_offsets}")
        print(f"Accel offsets: {self.accel_offsets}")
        
        if self.calibrate:
            exit()

    def _apply_axis_remap(self, gyro, accel):
        """Apply axis remapping similar to BNO055"""
        if self.upside_down:
            # Remap axes: Y->X, X->Y, Z->Z, and negate all
            gyro_remapped = [-gyro[1], -gyro[0], -gyro[2]]
            accel_remapped = [-accel[1], -accel[0], -accel[2]]
        else:
            # Remap axes: Y->X, X->Y, Z->Z, negate X, keep Y and Z positive
            gyro_remapped = [-gyro[1], gyro[0], gyro[2]]
            accel_remapped = [-accel[1], accel[0], accel[2]]
        
        return gyro_remapped, accel_remapped

    def _complementary_filter(self, gyro, accel, dt, alpha=0.98):
        """Simple complementary filter for orientation estimation"""
        # Normalize accelerometer data
        accel_norm = np.linalg.norm(accel)
        if accel_norm > 0:
            accel = accel / accel_norm
        else:
            return self.orientation_quat  # Return previous orientation if invalid accel data
        
        # Calculate accelerometer-based roll and pitch
        roll_accel = np.arctan2(accel[1], accel[2])
        pitch_accel = np.arctan2(-accel[0], np.sqrt(accel[1]**2 + accel[2]**2))
        
        # Get current orientation as euler angles
        current_euler = R.from_quat(self.orientation_quat).as_euler('xyz')
        
        # Integrate gyroscope data
        roll_gyro = current_euler[0] + gyro[0] * dt
        pitch_gyro = current_euler[1] + gyro[1] * dt
        yaw_gyro = current_euler[2] + gyro[2] * dt
        
        # Apply complementary filter
        roll = alpha * roll_gyro + (1 - alpha) * roll_accel
        pitch = alpha * pitch_gyro + (1 - alpha) * pitch_accel
        yaw = yaw_gyro  # No magnetometer correction for yaw in this simple implementation
        
        # Apply pitch bias
        pitch -= np.deg2rad(self.pitch_bias)
        
        # Convert back to quaternion
        orientation_quat = R.from_euler('xyz', [roll, pitch, yaw]).as_quat()
        
        return orientation_quat

    def convert_axes(self, euler):
        euler = [np.pi + euler[1], euler[0], euler[2]]
        return euler

    def imu_worker(self):
        while True:
            s = time.time()
            current_time = time.time()
            dt = current_time - self.last_time
            self.last_time = current_time
            
            try:
                gyro_raw = self.imu.gyro
                accel_raw = self.imu.acceleration
                
                if gyro_raw is None or accel_raw is None:
                    continue
                    
                # Apply calibration offsets
                gyro = [g - offset for g, offset in zip(gyro_raw, self.gyro_offsets)]
                accel = [a - offset for a, offset in zip(accel_raw, self.accel_offsets)]
                
                # Apply axis remapping
                gyro, accel = self._apply_axis_remap(gyro, accel)
                
                # Update orientation using complementary filter
                self.orientation_quat = self._complementary_filter(gyro, accel, dt)

            except Exception as e:
                print("[ICM20948 IMU]:", e)
                continue

            self.imu_queue.put(self.orientation_quat.copy())
            took = time.time() - s
            time.sleep(max(0, 1 / self.sampling_freq - took))

    def get_data(self, euler=False, mat=False):
        try:
            self.last_imu_data = self.imu_queue.get(False)  # non blocking
        except Exception:
            pass

        try:
            if not euler and not mat:
                return self.last_imu_data
            elif euler:
                return R.from_quat(self.last_imu_data).as_euler("xyz")
            elif mat:
                return R.from_quat(self.last_imu_data).as_matrix()

        except Exception as e:
            print("[ICM20948 IMU]: ", e)
            return None


class Imu:
    def __init__(
        self, sampling_freq, user_pitch_bias=0, calibrate=False, upside_down=True
    ):
        self.sampling_freq = sampling_freq
        self.user_pitch_bias = user_pitch_bias
        self.nominal_pitch_bias = 0
        self.calibrate = calibrate

        # self.uart = serial.Serial("/dev/ttyS0", baudrate=9600)
        # self.imu = adafruit_bno055.BNO055_UART(self.uart)

        i2c = busio.I2C(board.SCL, board.SDA)
        self.imu = adafruit_bno055.BNO055_I2C(i2c)

        self.imu.mode = adafruit_bno055.IMUPLUS_MODE
        # self.imu.mode = adafruit_bno055.ACCGYRO_MODE
        # self.imu.mode = adafruit_bno055.GYRONLY_MODE
        # self.imu.mode = adafruit_bno055.NDOF_MODE
        # self.imu.mode = adafruit_bno055.NDOF_FMC_OFF_MODE

        if upside_down:
            self.imu.axis_remap = (
                adafruit_bno055.AXIS_REMAP_Y,
                adafruit_bno055.AXIS_REMAP_X,
                adafruit_bno055.AXIS_REMAP_Z,
                adafruit_bno055.AXIS_REMAP_NEGATIVE,
                adafruit_bno055.AXIS_REMAP_NEGATIVE,
                adafruit_bno055.AXIS_REMAP_NEGATIVE,
            )
        else:
            self.imu.axis_remap = (
                adafruit_bno055.AXIS_REMAP_Y,
                adafruit_bno055.AXIS_REMAP_X,
                adafruit_bno055.AXIS_REMAP_Z,
                adafruit_bno055.AXIS_REMAP_NEGATIVE,
                adafruit_bno055.AXIS_REMAP_POSITIVE,
                adafruit_bno055.AXIS_REMAP_POSITIVE,
            )

        self.pitch_bias = self.nominal_pitch_bias + self.user_pitch_bias

        if self.calibrate:
            self.imu.mode = adafruit_bno055.NDOF_MODE
            calibrated = self.imu.calibrated
            while not calibrated:
                print("Calibration status: ", self.imu.calibration_status)
                print("Calibrated : ", self.imu.calibrated)
                calibrated = self.imu.calibrated
                time.sleep(0.1)
            print("CALIBRATION DONE")
            offsets_accelerometer = self.imu.offsets_accelerometer
            offsets_gyroscope = self.imu.offsets_gyroscope
            offsets_magnetometer = self.imu.offsets_magnetometer

            imu_calib_data = {
                "offsets_accelerometer": offsets_accelerometer,
                "offsets_gyroscope": offsets_gyroscope,
                "offsets_magnetometer": offsets_magnetometer,
            }
            for k, v in imu_calib_data.items():
                print(k, v)

            pickle.dump(imu_calib_data, open("imu_calib_data.pkl", "wb"))

            print("Saved", "imu_calib_data.pkl")
            exit()

        if os.path.exists("imu_calib_data.pkl"):
            imu_calib_data = pickle.load(open("imu_calib_data.pkl", "rb"))
            self.imu.mode = adafruit_bno055.CONFIG_MODE
            time.sleep(0.1)
            self.imu.offsets_accelerometer = imu_calib_data["offsets_accelerometer"]
            self.imu.offsets_gyroscope = imu_calib_data["offsets_gyroscope"]
            self.imu.offsets_magnetometer = imu_calib_data["offsets_magnetometer"]
            self.imu.mode = adafruit_bno055.IMUPLUS_MODE
            time.sleep(0.1)
        else:
            print("imu_calib_data.pkl not found")
            print("Imu is running uncalibrated")

        self.last_imu_data = [0, 0, 0, 0]
        self.imu_queue = Queue(maxsize=1)
        Thread(target=self.imu_worker, daemon=True).start()

    def convert_axes(self, euler):
        euler = [np.pi + euler[1], euler[0], euler[2]]
        return euler

    def imu_worker(self):
        while True:
            s = time.time()
            try:
                # imu returns scalar first
                raw_orientation = np.array(self.imu.quaternion).copy()  # quat
                euler = (
                    R.from_quat(raw_orientation, scalar_first=True)
                    .as_euler("xyz")
                    .copy()
                )
            except Exception as e:
                print("[IMU]:", e)
                continue

            # Converting to correct axes
            # euler = self.convert_axes(euler)
            euler[1] -= np.deg2rad(self.pitch_bias)
            # euler[2] = 0  # ignoring yaw

            # gives scalar last, which is what isaac wants
            final_orientation_quat = R.from_euler("xyz", euler).as_quat()

            self.imu_queue.put(final_orientation_quat.copy())
            took = time.time() - s
            time.sleep(max(0, 1 / self.sampling_freq - took))

    def get_data(self, euler=False, mat=False):
        try:
            self.last_imu_data = self.imu_queue.get(False)  # non blocking
        except Exception:
            pass

        try:
            if not euler and not mat:
                return self.last_imu_data
            elif euler:
                return R.from_quat(self.last_imu_data).as_euler("xyz")
            elif mat:
                return R.from_quat(self.last_imu_data).as_matrix()

        except Exception as e:
            print("[IMU]: ", e)
            return None


if __name__ == "__main__":    
    parser = argparse.ArgumentParser(description='IMU Orientation Reader')
    parser.add_argument('--imu-type', choices=['bno055', 'icm20948'], default='bno055',
                       help='Type of IMU to use (default: bno055)')
    parser.add_argument('--upside-down', action='store_true', default=False,
                       help='Set if IMU is mounted upside down')
    parser.add_argument('--calibrate', action='store_true', default=False,
                       help='Run calibration routine')
    parser.add_argument('--output', choices=['quat', 'euler'], default='quat',
                       help='Output format: quaternion or euler angles')
    
    args = parser.parse_args()
    
    print(f"Using {args.imu_type.upper()} IMU")
    
    if args.imu_type == 'icm20948':
        imu = ImuICM20948(50, upside_down=args.upside_down, calibrate=args.calibrate)
    else:
        imu = Imu(50, upside_down=args.upside_down, calibrate=args.calibrate)
    
    while True:
        if args.output == 'euler':
            data = imu.get_data(euler=True)
            if data is not None:
                print("euler (rad)", np.around(data, 3))
                print("euler (deg)", np.around(np.rad2deg(data), 1))
        else:
            data = imu.get_data()
            if data is not None:
                print("quaternion [x,y,z,w]", np.around(data, 3))
        print("---")
        time.sleep(1 / 25)
