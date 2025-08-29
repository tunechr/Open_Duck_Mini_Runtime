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
            # New: magnetometer bias (hard-iron). Soft-iron omitted for brevity.
            self.mag_offsets = calib_data.get("mag_offsets", [0, 0, 0])
            print("Loaded ICM20948 calibration data")
        else:
            self.gyro_offsets = [0, 0, 0]
            self.accel_offsets = [0, 0, 0]
            self.mag_offsets = [0, 0, 0]
            print("icm20948_calib_data.pkl not found")
            print("ICM20948 is running uncalibrated")

        # Apply axis remapping for upside down orientation
        self.upside_down = upside_down
        
        # Initialize orientation estimate
        self.orientation_quat = np.array([0, 0, 0, 1])  # [x, y, z, w]
        self.last_time = time.time()

        # Track yaw separately for blending
        self._yaw_gyro = 0.0

        self.last_imu_data = [0, 0, 0, 1]  # quaternion [x, y, z, w]
        self.imu_queue = Queue(maxsize=1)
        Thread(target=self.imu_worker, daemon=True).start()

    def _calibrate_icm20948(self):
        """Collect gyro, accel offsets (stationary) and mag hard-iron bias (slowly rotate)."""
        print("Keep the IMU stationary for 5s for gyro/accel calibration...")
        gyro_samples, accel_samples = [], []
        start_time = time.time()
        while time.time() - start_time < 5:
            try:
                gyro_samples.append(list(self.imu.gyro))
                accel_samples.append(list(self.imu.acceleration))
                time.sleep(0.01)
            except Exception as e:
                print(f"Calibration error: {e}")

        gyro_samples = np.array(gyro_samples) if len(gyro_samples) else np.zeros((1, 3))
        accel_samples = np.array(accel_samples) if len(accel_samples) else np.zeros((1, 3))

        self.gyro_offsets = np.mean(gyro_samples, axis=0).tolist()
        accel_means = np.mean(accel_samples, axis=0)
        self.accel_offsets = [float(accel_means[0]), float(accel_means[1]), float(accel_means[2] - 9.80665)]

        print("Now rotate the IMU slowly through all orientations for 20s for magnetometer bias...")
        mag_samples = []
        start_time = time.time()
        while time.time() - start_time < 20:
            try:
                m = self.imu.magnetic  # microtesla
                if m is not None:
                    mag_samples.append(list(m))
                time.sleep(0.02)
            except Exception as e:
                print(f"Mag calibration error: {e}")

        if len(mag_samples) >= 50:
            mags = np.array(mag_samples)
            # Simple hard-iron bias: (max + min)/2 per axis
            mag_min = mags.min(axis=0)
            mag_max = mags.max(axis=0)
            self.mag_offsets = ((mag_max + mag_min) / 2.0).tolist()
        else:
            self.mag_offsets = [0.0, 0.0, 0.0]
            print("Not enough magnetometer samples; skipping mag bias. You may recalibrate later.")

        calib_data = {
            "gyro_offsets": self.gyro_offsets,
            "accel_offsets": self.accel_offsets,
            "mag_offsets": self.mag_offsets,
        }
        pickle.dump(calib_data, open("icm20948_calib_data.pkl", "wb"))
        print("ICM20948 calibration completed and saved")
        print(f"Gyro offsets: {self.gyro_offsets}")
        print(f"Accel offsets: {self.accel_offsets}")
        print(f"Mag offsets:   {self.mag_offsets}")

        if self.calibrate:
            exit()

    def _apply_axis_remap(self, gyro, accel, mag=None):
        """Apply axis remapping similar to BNO055 remap above."""
        if self.upside_down:
            gyro_remapped = [-gyro[1], -gyro[0], -gyro[2]]
            accel_remapped = [-accel[1], -accel[0], -accel[2]]
            mag_remapped = None if mag is None else [-mag[1], -mag[0], -mag[2]]
        else:
            gyro_remapped = [-gyro[1], gyro[0], gyro[2]]
            accel_remapped = [-accel[1], accel[0], accel[2]]
            mag_remapped = None if mag is None else [-mag[1], mag[0], mag[2]]
        return gyro_remapped, accel_remapped, mag_remapped

    def _tilt_compensated_yaw(self, accel, mag):
        """Compute tilt-compensated yaw (heading) from accel (for roll/pitch) and mag."""
        # Normalize
        a = np.array(accel, dtype=float)
        m = np.array(mag, dtype=float)
        an = np.linalg.norm(a); mn = np.linalg.norm(m)
        if an < 1e-6 or mn < 1e-6:
            return None
        a /= an; m /= mn
        # Roll, pitch from accel
        roll = np.arctan2(a[1], a[2])
        pitch = np.arctan2(-a[0], np.sqrt(a[1] ** 2 + a[2] ** 2))
        # Tilt-compensation (NED-like)
        mx, my, mz = m
        sinr, cosr = np.sin(roll), np.cos(roll)
        sinp, cosp = np.sin(pitch), np.cos(pitch)
        # Rotate mag into horizontal plane
        hx = mx * cosp + mz * sinp
        hy = mx * sinr * sinp + my * cosr - mz * sinr * cosp
        yaw = np.arctan2(-hy, hx)  # sign chosen to match BNO heading sense; adjust if needed
        return float(yaw)

    def _wrap_angle(self, a):
        return (a + np.pi) % (2 * np.pi) - np.pi

    def _normalize_quat(self, q):
        q = np.array(q, dtype=float)
        n = np.linalg.norm(q)
        if n > 0:
            q /= n
        # Stabilize sign so w >= 0 for consistency with BNO usage
        if q[3] < 0:
            q = -q
        return q

    def _complementary_filter(self, gyro, accel, mag, dt, alpha=0.98, beta=0.98):
        """
        alpha: accel vs gyro for roll/pitch
        beta:  gyro vs magnetometer for yaw (1.0 => rely on gyro, 0.0 => rely on mag)
        """
        # Normalize accel
        accel = np.array(accel, dtype=float)
        an = np.linalg.norm(accel)
        if an > 0:
            accel /= an
        else:
            return self.orientation_quat

        # Accel-based roll & pitch
        roll_accel = np.arctan2(accel[1], accel[2])
        pitch_accel = np.arctan2(-accel[0], np.sqrt(accel[1] ** 2 + accel[2] ** 2))

        # Current euler from last orientation
        current_euler = R.from_quat(self.orientation_quat).as_euler('xyz')
        # Gyro integrate
        roll_gyro = current_euler[0] + gyro[0] * dt
        pitch_gyro = current_euler[1] + gyro[1] * dt
        yaw_gyro = current_euler[2] + gyro[2] * dt

        # Blend roll/pitch
        roll = alpha * roll_gyro + (1 - alpha) * roll_accel
        pitch = alpha * pitch_gyro + (1 - alpha) * pitch_accel

        # Magnetometer yaw if available
        yaw = yaw_gyro
        if mag is not None:
            yaw_mag = self._tilt_compensated_yaw(accel, mag)
            if yaw_mag is not None:
                # Blend with wrapping
                dy = self._wrap_angle(yaw_gyro - yaw_mag)
                yaw = self._wrap_angle(yaw_mag + beta * dy)

        # Pitch bias to match BNO convention
        pitch -= np.deg2rad(self.pitch_bias)

        q = R.from_euler('xyz', [roll, pitch, yaw]).as_quat()
        return self._normalize_quat(q)

    def imu_worker(self):
        while True:
            s = time.time()
            current_time = time.time()
            dt = current_time - self.last_time
            self.last_time = current_time
            try:
                gyro_raw = self.imu.gyro            # rad/s (Adafruit)
                accel_raw = self.imu.acceleration   # m/s^2
                mag_raw = self.imu.magnetic         # microtesla
                if gyro_raw is None or accel_raw is None:
                    time.sleep(1 / self.sampling_freq)
                    continue

                # Offsets
                gyro = [g - off for g, off in zip(gyro_raw, self.gyro_offsets)]
                accel = [a - off for a, off in zip(accel_raw, self.accel_offsets)]
                mag = None
                if mag_raw is not None:
                    mag = [m - off for m, off in zip(mag_raw, self.mag_offsets)]

                # Axis remap to match BNO
                gyro, accel, mag = self._apply_axis_remap(gyro, accel, mag)

                # Update orientation
                self.orientation_quat = self._complementary_filter(gyro, accel, mag, dt)

            except Exception as e:
                print("[ICM20948 IMU]:", e)
                # continue

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


class ImuBNO08x:
    """
    BNO08x/BNO085 IMU using CircuitPython driver. Outputs quaternion [x,y,z,w].
    Notes:
    - Sensor fusion is done on-chip. We enable Game Rotation Vector by default (gyro+accel),
      which is stable and not impacted by magnetic disturbances. If you want magnetometer-based
      absolute yaw, set use_mag=True.
    - Calibration is handled internally by the sensor; move through different orientations.
    """
    def __init__(
        self,
        sampling_freq,
        user_pitch_bias=0,
        calibrate=False,
        upside_down=False,
        use_mag=False,
    ):
        self.sampling_freq = sampling_freq
        self.user_pitch_bias = user_pitch_bias
        self.nominal_pitch_bias = 0
        self.calibrate = calibrate
        self.upside_down = upside_down
        self.use_mag = use_mag

        # Lazy import to avoid hard dependency for users not using BNO08x
        try:
            from adafruit_bno08x.i2c import BNO08X_I2C
            from adafruit_bno08x import (
                BNO_REPORT_GAME_ROTATION_VECTOR,
                BNO_REPORT_ROTATION_VECTOR,
            )
        except Exception as e:
            raise RuntimeError(
                "BNO08x driver not available. Install 'adafruit-circuitpython-bno08x' and 'adafruit-blinka'."
            ) from e

        i2c = busio.I2C(board.SCL, board.SDA)
        self.imu = BNO08X_I2C(i2c)

        # Enable desired fusion output
        if self.use_mag:
            self.imu.enable_feature(BNO_REPORT_ROTATION_VECTOR)
        else:
            self.imu.enable_feature(BNO_REPORT_GAME_ROTATION_VECTOR)

        self.pitch_bias = self.nominal_pitch_bias + self.user_pitch_bias

        if self.calibrate:
            print("BNO08x calibrates internally; move sensor slowly through orientations.")
            # No explicit save/load API in this driver; rely on continuous self-calibration

        self.last_imu_data = [0, 0, 0, 1]
        self.imu_queue = Queue(maxsize=1)
        Thread(target=self.imu_worker, daemon=True).start()

    def _apply_upside_down(self, q):
        """Apply a simple upside-down correction by rotating 180° about X.
        Adjust if your physical mount differs.
        """
        if not self.upside_down:
            return q
        # 180 deg about X axis
        q_ud = R.from_euler("x", np.pi).as_quat()  # [x,y,z,w]
        return (R.from_quat(q) * R.from_quat(q_ud)).as_quat()

    def imu_worker(self):
        while True:
            s = time.time()
            try:
                # Returns (i, j, k, real) => (x, y, z, w)
                quat = self.imu.quaternion
                if quat is None:
                    time.sleep(1 / self.sampling_freq)
                    continue
                x, y, z, w = quat
                q = np.array([x, y, z, w], dtype=float)
                # Normalize and enforce scalar w >= 0 for consistency
                n = np.linalg.norm(q)
                if n > 0:
                    q = q / n
                if q[3] < 0:
                    q = -q

                # Optional upside-down mapping
                q = self._apply_upside_down(q)

                # Apply pitch bias by converting to euler, subtract bias, then back
                e = R.from_quat(q).as_euler("xyz")
                e[1] -= np.deg2rad(self.pitch_bias)
                q = R.from_euler("xyz", e).as_quat()

                self.imu_queue.put(q.copy())
            except Exception as e:
                print("[BNO08x IMU]:", e)

            took = time.time() - s
            time.sleep(max(0, 1 / self.sampling_freq - took))

    def get_data(self, euler=False, mat=False):
        try:
            self.last_imu_data = self.imu_queue.get(False)
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
            print("[BNO08x IMU]: ", e)
            return None

if __name__ == "__main__":    
    parser = argparse.ArgumentParser(description='IMU Orientation Reader')
    parser.add_argument('--imu-type', choices=['bno055', 'icm20948', 'bno08x', 'bno085'], default='bno055',
                       help='Type of IMU to use (default: bno055)')
    parser.add_argument('--upside-down', action='store_true', default=False,
                       help='Set if IMU is mounted upside down')
    parser.add_argument('--calibrate', action='store_true', default=False,
                       help='Run calibration routine')
    parser.add_argument('--use-mag', action='store_true', default=False,
                        help='For BNO08x only: use magnetometer yaw (Rotation Vector) instead of Game Rotation Vector')
    parser.add_argument('--output', choices=['quat', 'euler'], default='quat',
                       help='Output format: quaternion or euler angles')
    
    args = parser.parse_args()
    
    print(f"Using {args.imu_type.upper()} IMU")
    
    if args.imu_type == 'icm20948':
        imu = ImuICM20948(50, upside_down=args.upside_down, calibrate=args.calibrate)
    elif args.imu_type in ('bno08x', 'bno085'):
        imu = ImuBNO08x(50, upside_down=args.upside_down, calibrate=args.calibrate, use_mag=args.use_mag)
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
