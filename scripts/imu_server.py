import socket
import time
import pickle
import sys
import os

# Add the parent directory to the path to import mini_bdx_runtime
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from mini_bdx_runtime.mini_bdx_runtime.imu import Imu, ImuICM20948
from threading import Thread
import time

import argparse


class IMUServer:
    def __init__(self, imu=None, imu_type='bno055', pitch_bias=0, upside_down=False, calibrate=False):
        self.host = "0.0.0.0"
        self.port = 1234

        self.server_socket = socket.socket()
        self.server_socket.setsockopt(
            socket.SOL_SOCKET, socket.SO_REUSEADDR, 1
        )  # enable address reuse

        self.server_socket.bind((self.host, self.port))

        if imu is None:
            if imu_type == 'icm20948':
                self.imu = ImuICM20948(50, user_pitch_bias=pitch_bias, upside_down=upside_down, calibrate=calibrate)
            else:
                self.imu = Imu(50, user_pitch_bias=pitch_bias, upside_down=upside_down, calibrate=calibrate)
        else:
            self.imu = imu
        self.stop = False

        Thread(target=self.run, daemon=True).start()

    def run(self):
        while not self.stop:
            self.server_socket.listen(1)
            conn, address = self.server_socket.accept()  # accept new connection
            print("Connection from: " + str(address))
            try:
                while True:
                    data = self.imu.get_data()
                    data = pickle.dumps(data)
                    conn.send(data)  # send data to the client
                    time.sleep(1 / 30)
            except:
                pass

        self.server_socket.close()
        print("thread closed")
        time.sleep(1)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--pitch_bias", type=float, default=0, help="deg")
    parser.add_argument('--imu-type', choices=['bno055', 'icm20948'], default='bno055',
                       help='Type of IMU to use (default: bno055)')
    parser.add_argument('--upside-down', action='store_true', default=False,
                       help='Set if IMU is mounted upside down')
    parser.add_argument('--calibrate', action='store_true', default=False,
                       help='Run calibration routine')
    args = parser.parse_args()
    
    print(f"Starting IMU server with {args.imu_type.upper()} IMU")
    
    imu_server = IMUServer(
        imu_type=args.imu_type,
        pitch_bias=args.pitch_bias,
        upside_down=args.upside_down,
        calibrate=args.calibrate
    )
    try:
        while True:
            time.sleep(0.01)
    except KeyboardInterrupt:
        print("Closing server")
        imu_server.stop = True

    time.sleep(2)
