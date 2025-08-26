import time
import numpy as np
from scipy.spatial.transform import Rotation as R

from mini_bdx_runtime.mini_bdx_runtime.imu import Imu, ImuICM20948

def quat_to_euler_deg(q):
    q = np.array(q, dtype=float)
    q = q / (np.linalg.norm(q) + 1e-12)
    if q[3] < 0:
        q = -q
    return np.rad2deg(R.from_quat(q).as_euler('xyz'))

def main():
    # Run both at 50 Hz, same upside_down setting
    bno = Imu(50, upside_down=False, calibrate=False)
    icm = ImuICM20948(50, upside_down=False, calibrate=False)

    print("Comparing BNO055 vs ICM20948 (Euler xyz degrees). Ctrl+C to stop.")
    time.sleep(1.0)
    while True:
        q_bno = bno.get_data()         # [x,y,z,w]
        q_icm = icm.get_data()
        if q_bno is None or q_icm is None:
            time.sleep(0.02)
            continue
        e_bno = quat_to_euler_deg(q_bno)
        e_icm = quat_to_euler_deg(q_icm)
        delta = e_icm - e_bno
        # Wrap deltas to [-180, 180]
        delta = (delta + 180.0) % 360.0 - 180.0

        print(
            f"BNO  [R,P,Y]: {e_bno.round(1)} | "
            f"ICM  [R,P,Y]: {e_icm.round(1)} | "
            f"Δ [R,P,Y]: {delta.round(1)}"
        )
        time.sleep(0.05)

if __name__ == "__main__":
    main()