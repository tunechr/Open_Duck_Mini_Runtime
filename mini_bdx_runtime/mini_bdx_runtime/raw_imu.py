import adafruit_bno055
import adafruit_icm20x
import board
import busio
import numpy as np
import os
import pickle

from queue import Queue
from threading import Thread
import time


# TODO filter spikes
class ImuICM20948:
    def __init__(
        self, sampling_freq, user_pitch_bias=0, calibrate=False, upside_down=True
    ):
        self.sampling_freq = sampling_freq
        self.calibrate = calibrate

        i2c = busio.I2C(board.SCL, board.SDA)
        self.imu = adafruit_icm20x.ICM20948(i2c)

        # ICM20948 doesn't have the same calibration system as BNO055
        # We'll implement a simple offset-based calibration
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
        
        self.x_offset = 0

        self.last_imu_data = {
            "gyro": [0, 0, 0],
            "accelero": [0, 0, 0],
        }
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

    def tare_x(self):
        print("Taring x ...")
        x_values = []
        num_values = 100
        ok = False
        while not ok:
            try:
                accel = self.imu.acceleration
                if accel is not None:
                    x_values.append(accel[0])
            except Exception:
                continue

            x_values = x_values[-num_values:]

            if len(x_values) == num_values:
                mean = np.mean(x_values)
                std = np.std(x_values)
                if std < 0.05:
                    ok = True
                    self.x_offset = mean
                    print("Tare x done")
                else:
                    print(std)

            time.sleep(0.01)

    def imu_worker(self):
        while True:
            s = time.time()
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
                
                # Apply x offset (tare)
                accel[0] -= self.x_offset

                data = {
                    "gyro": np.array(gyro),
                    "accelero": np.array(accel),
                }

            except Exception as e:
                print("[ICM20948 IMU]:", e)
                continue

            self.imu_queue.put(data)
            took = time.time() - s
            time.sleep(max(0, 1 / self.sampling_freq - took))

    def get_data(self):
        try:
            self.last_imu_data = self.imu_queue.get(False)  # non blocking
        except Exception:
            pass

        return self.last_imu_data


class Imu:
    def __init__(
        self, sampling_freq, user_pitch_bias=0, calibrate=False, upside_down=True
    ):
        self.sampling_freq = sampling_freq
        self.calibrate = calibrate

        i2c = busio.I2C(board.SCL, board.SDA)
        self.imu = adafruit_bno055.BNO055_I2C(i2c)

        # self.imu.mode = adafruit_bno055.IMUPLUS_MODE
        # self.imu.mode = adafruit_bno055.ACCGYRO_MODE
        # self.imu.mode = adafruit_bno055.GYRONLY_MODE
        self.imu.mode = adafruit_bno055.NDOF_MODE
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
            self.imu.mode = adafruit_bno055.NDOF_MODE
            time.sleep(0.1)
        else:
            print("imu_calib_data.pkl not found")
            print("Imu is running uncalibrated")

        self.x_offset = 0

        # self.tare_x()

        self.last_imu_data = [0, 0, 0, 0]
        self.last_imu_data = {
            "gyro": [0, 0, 0],
            "accelero": [0, 0, 0],
        }
        self.imu_queue = Queue(maxsize=1)
        Thread(target=self.imu_worker, daemon=True).start()

    def tare_x(self):
        print("Taring x ...")
        x_values = []
        num_values = 100
        ok = False
        while not ok:
            x_values.append(np.array(self.imu.acceleration)[0])

            x_values = x_values[-num_values:]

            if len(x_values) == num_values:
                mean = np.mean(x_values)
                std = np.std(x_values)
                if std < 0.05:
                    ok = True
                    self.x_offset = mean
                    print("Tare x done")
                else:
                    print(std)

            time.sleep(0.01)

    def imu_worker(self):
        while True:
            s = time.time()
            try:
                gyro = np.array(self.imu.gyro).copy()
                accelero = np.array(self.imu.acceleration).copy()
            except Exception as e:
                print("[IMU]:", e)
                continue

            if gyro is None or accelero is None:
                continue

            if gyro.any() is None or accelero.any() is None:
                continue

            accelero[0] -= self.x_offset

            data = {
                "gyro": gyro,
                "accelero": accelero,
            }

            self.imu_queue.put(data)
            took = time.time() - s
            time.sleep(max(0, 1 / self.sampling_freq - took))

    def get_data(self):
        try:
            self.last_imu_data = self.imu_queue.get(False)  # non blocking
        except Exception:
            pass

        return self.last_imu_data


if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(description='IMU Data Reader')
    parser.add_argument('--imu-type', choices=['bno055', 'icm20948'], default='bno055',
                       help='Type of IMU to use (default: bno055)')
    parser.add_argument('--upside-down', action='store_true', default=False,
                       help='Set if IMU is mounted upside down')
    parser.add_argument('--calibrate', action='store_true', default=False,
                       help='Run calibration routine')
    
    args = parser.parse_args()
    
    print(f"Using {args.imu_type.upper()} IMU")
    
    if args.imu_type == 'icm20948':
        imu = ImuICM20948(50, upside_down=args.upside_down, calibrate=args.calibrate)
    else:
        imu = Imu(50, upside_down=args.upside_down, calibrate=args.calibrate)
    
    while True:
        data = imu.get_data()
        # print(data)
        print("gyro", np.around(data["gyro"], 3))
        print("accelero", np.around(data["accelero"], 3))
        print("---")
        time.sleep(1 / 25)
