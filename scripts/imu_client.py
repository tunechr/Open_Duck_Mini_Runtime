import socket
import time
import numpy as np
import pickle
from queue import Queue
from threading import Thread
from scipy.spatial.transform import Rotation as R
from FramesViewer.viewer import Viewer
import argparse


class IMUClient:
    def __init__(self, host, port=1234, freq=30):
        self.host = host
        self.port = port
        self.freq = freq
        self.client_socket = socket.socket()
        self.connected = False
        
        print(f"Connecting to IMU server at {host}:{port}...")
        while not self.connected:
            try:
                self.client_socket.connect((self.host, self.port))
                self.connected = True
                print("Connected successfully!")
            except Exception as e:
                print(f"Connection failed: {e}")
                time.sleep(0.5)
        self.imu_queue = Queue(maxsize=1)
        self.last_imu = [0, 0, 0, 0]

        Thread(target=self.imu_worker, daemon=True).start()

    def imu_worker(self):
        while True:
            try:
                data = self.client_socket.recv(1024)  # receive response
                if len(data) == 0:
                    print("Connection lost")
                    break
                data = pickle.loads(data)
                self.imu_queue.put(data)
            except Exception as e:
                print(f"IMU data receive error: {e}")

            time.sleep(1 / self.freq)

    def get_imu(self):
        try:
            self.last_imu = self.imu_queue.get(False)
        except:
            pass

        return self.last_imu


if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", type=str, required=True, help="IP address of the robot")
    parser.add_argument("--port", type=int, default=1234, help="Port number (default: 1234)")
    parser.add_argument("--freq", type=int, default=30, help="Update frequency (default: 30)")
    args = parser.parse_args()

    client = IMUClient(args.ip, port=args.port, freq=args.freq)

    fv = Viewer()
    fv.start()
    pose = np.eye(4)
    pose[:3, 3] = [0.1, 0.1, 0.1]
    try:
        while True:
            quat = client.get_imu()
            try:
                rot_mat = R.from_quat(quat).as_matrix()
                pose[:3, :3] = rot_mat
                fv.pushFrame(pose, "pose")
            except Exception as e:
                print("error", e)
                pass
            time.sleep(1 / 30)
    except KeyboardInterrupt:
        pass
