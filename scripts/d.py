import pyglet
from pyglet.gl import *
import numpy as np
from scipy.spatial.transform import Rotation as R
import time

# ---- replace with your real client ----
class DummyClient:
    def get_imu(self):
        # Slowly rotating demo quaternion (z-y-x)
        t = time.time()
        r = R.from_euler('xyz', [0, 0, (t*30)%360], degrees=True)
        return r.as_quat()  # [x,y,z,w]
client = DummyClient()
# ---------------------------------------

window = pyglet.window.Window(800, 600, "IMU Orientation (pyglet 1.5.x)", resizable=True)
rotation_matrix = (GLfloat * 16)(*np.eye(4, dtype=np.float32).T.flatten())

@window.event
def on_draw():
    window.clear()
    glEnable(GL_DEPTH_TEST)

    # Projection
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    aspect = window.width / float(max(window.height, 1))
    gluPerspective(45.0, aspect, 0.1, 100.0)

    # View
    glMatrixMode(GL_MODELVIEW)
    glLoadIdentity()
    gluLookAt(0, 0, 1.5, 0, 0, 0, 0, 1, 0)

    # Model (apply orientation)
    glMultMatrixf(rotation_matrix)

    # Draw XYZ axes
    glLineWidth(3.0)
    glBegin(GL_LINES)
    # X (red)
    glColor3f(1, 0, 0); glVertex3f(0, 0, 0); glVertex3f(0.5, 0, 0)
    # Y (green)
    glColor3f(0, 1, 0); glVertex3f(0, 0, 0); glVertex3f(0, 0.5, 0)
    # Z (blue)
    glColor3f(0, 0, 1); glVertex3f(0, 0, 0); glVertex3f(0, 0, 0.5)
    glEnd()

def update(dt):
    global rotation_matrix
    quat = client.get_imu()  # expects [x,y,z,w]
    rot = R.from_quat(quat).as_matrix()
    pose = np.eye(4, dtype=np.float32)
    pose[:3, :3] = rot
    # optional translation:
    # pose[:3, 3] = [0.1, 0.1, 0.1]
    rotation_matrix[:] = (GLfloat * 16)(*pose.T.flatten())

pyglet.clock.schedule_interval(update, 1/30.0)
pyglet.app.run()
