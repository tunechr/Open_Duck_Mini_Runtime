import socket
import time
import numpy as np
import pickle
from queue import Queue
from threading import Thread
from scipy.spatial.transform import Rotation as R
import argparse

# -------------------- PyQtGraph Fallback Viewer --------------------
def run_pyqtgraph_viewer(client, freq_hz=30):
    """
    Blocking PyQtGraph 3D viewer.
    - Shows a 3D scene with colored axes that rotate with the latest IMU quaternion.
    - Displays real-time quaternion and Euler angle values as text.
    - Updates at the specified frequency (default 30Hz).
    - Pulls data from `client.get_imu()` on a QTimer.
    """
    try:
        # Prefer PyQt5; if you use PySide6, swap imports accordingly
        from pyqtgraph.Qt import QtCore, QtWidgets, QtGui
        import pyqtgraph.opengl as gl
    except Exception as e:
        raise RuntimeError(f"PyQtGraph/Qt not available: {e}")

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    # Create main widget with layout
    main_widget = QtWidgets.QWidget()
    main_widget.setWindowTitle("IMU Orientation (PyQtGraph)")
    main_widget.setMinimumSize(900, 700)
    
    layout = QtWidgets.QVBoxLayout(main_widget)
    
    # Add text display for values
    text_widget = QtWidgets.QLabel()
    text_widget.setAlignment(QtCore.Qt.AlignLeft)
    text_widget.setFont(QtGui.QFont("Courier", 12))
    text_widget.setStyleSheet("QLabel { background-color: #f0f0f0; padding: 10px; }")
    text_widget.setMinimumHeight(100)
    layout.addWidget(text_widget)

    # Create 3D view
    view = gl.GLViewWidget()
    view.setCameraPosition(distance=2.0)
    view.setMinimumSize(800, 500)
    layout.addWidget(view)
    
    main_widget.show()

    # Create a simple axis representation using lines
    # X-axis (red), Y-axis (green), Z-axis (blue)
    x_axis = gl.GLLinePlotItem(pos=np.array([[0, 0, 0], [0.5, 0, 0]]), color=(1, 0, 0, 1), width=4)
    y_axis = gl.GLLinePlotItem(pos=np.array([[0, 0, 0], [0, 0.5, 0]]), color=(0, 1, 0, 1), width=4)
    z_axis = gl.GLLinePlotItem(pos=np.array([[0, 0, 0], [0, 0, 0.5]]), color=(0, 0, 1, 1), width=4)
    
    view.addItem(x_axis)
    view.addItem(y_axis)
    view.addItem(z_axis)
    
    # Add grid for reference
    grid = gl.GLGridItem()
    grid.scale(0.1, 0.1, 0.1)
    view.addItem(grid)

    # We’ll update the axis transform each tick
    timer = QtCore.QTimer()
    timer.setInterval(int(1000 / max(1, freq_hz)))

    def update():
        if not client.is_connected():
            timer.stop()
            main_widget.close()
            return
        quat = client.get_imu()  # [x,y,z,w]
        try:
            # Calculate rotation matrix
            rot = R.from_quat(quat).as_matrix()
            
            # Calculate Euler angles
            euler = R.from_quat(quat).as_euler('xyz', degrees=True)
            
            # Update text display
            text_content = f"""Quaternion: [{quat[0]:.3f}, {quat[1]:.3f}, {quat[2]:.3f}, {quat[3]:.3f}]
Euler (XYZ): Roll={euler[0]:.1f}°, Pitch={euler[1]:.1f}°, Yaw={euler[2]:.1f}°

X-axis: Red    Y-axis: Green    Z-axis: Blue"""
            text_widget.setText(text_content)
            
            # Update axis positions based on rotation
            x_pos = np.array([[0, 0, 0], rot[:, 0] * 0.5])
            y_pos = np.array([[0, 0, 0], rot[:, 1] * 0.5])  
            z_pos = np.array([[0, 0, 0], rot[:, 2] * 0.5])
            
            x_axis.setData(pos=x_pos)
            y_axis.setData(pos=y_pos)
            z_axis.setData(pos=z_pos)
            
        except Exception as e:
            print(f"Visualization error: {e}")

    timer.timeout.connect(update)
    timer.start()

    # Start the Qt event loop (blocking until window closes)
    app.exec_()
# ------------------------------------------------------------------


class IMUClient:
    def __init__(self, host, port=1234, freq=30, debug=False):
        self.host = host
        self.port = port
        self.freq = freq
        self.debug = debug
        self.client_socket = None
        self.connected = False
        self.connection_attempts = 0
        self.max_connection_attempts = 10
        self.should_run = True  # Flag to control the worker thread

        self.imu_queue = Queue(maxsize=1)
        self.last_imu = [0, 0, 0, 1]  # Default quaternion [x, y, z, w]
        
        self.connect()
        Thread(target=self.imu_worker, daemon=True).start()

    def connect(self):
        """Attempt to connect to the IMU server"""
        print(f"Connecting to IMU server at {self.host}:{self.port}...")
        self.connection_attempts = 0
        
        while not self.connected and self.connection_attempts < self.max_connection_attempts:
            try:
                if self.client_socket:
                    try:
                        self.client_socket.close()
                    except:
                        pass
                
                self.client_socket = socket.socket()
                self.client_socket.settimeout(5.0)  # 5 second timeout
                self.client_socket.connect((self.host, self.port))
                self.connected = True
                print("Connected successfully!")
                return True
            except Exception as e:
                self.connection_attempts += 1
                print(f"Connection attempt {self.connection_attempts} failed: {e}")
                if self.connection_attempts < self.max_connection_attempts:
                    print("Retrying in 1 second...")
                    time.sleep(1)
                else:
                    print("Max connection attempts reached.")
                    return False
        return False

    def reconnect(self):
        """Attempt to reconnect to the server"""
        print("Attempting to reconnect...")
        self.connected = False
        time.sleep(2)  # Wait before reconnecting
        return self.connect()

    def imu_worker(self):
        buffer = b''  # Buffer to accumulate incomplete data
        consecutive_errors = 0
        max_consecutive_errors = 10
        reconnect_attempts = 0
        max_reconnect_attempts = 3
        
        while self.should_run:
            if not self.connected:
                if reconnect_attempts < max_reconnect_attempts:
                    if self.reconnect():
                        buffer = b''  # Clear buffer on reconnect
                        consecutive_errors = 0
                        reconnect_attempts = 0
                    else:
                        reconnect_attempts += 1
                        time.sleep(5)  # Wait longer between reconnect attempts
                else:
                    print("Max reconnection attempts reached. Stopping...")
                    break
                continue
            
            try:
                data = self.client_socket.recv(1024)
                if len(data) == 0:
                    print("Connection lost - server closed")
                    self.connected = False
                    continue
                
                if self.debug:
                    print(f"Received {len(data)} bytes: {data[:50]}...")  # Show first 50 bytes
                
                buffer += data
                
                # Try to deserialize data from buffer
                while buffer:  # Process all complete objects in buffer
                    try:
                        # Use BytesIO to create a stream from buffer
                        import io
                        stream = io.BytesIO(buffer)
                        decoded_data = pickle.load(stream)  # Load one object
                        
                        # Successfully loaded one object, remove it from buffer
                        consumed_bytes = stream.tell()
                        buffer = buffer[consumed_bytes:]
                        consecutive_errors = 0  # Reset error counter
                        
                        if self.debug:
                            print(f"Successfully parsed pickle object, consumed {consumed_bytes} bytes, {len(buffer)} bytes remaining")
                        
                        # Overwrite queue with newest
                        try:
                            self.imu_queue.put_nowait(decoded_data)
                        except:
                            try:
                                self.imu_queue.get_nowait()
                                self.imu_queue.put_nowait(decoded_data)
                            except:
                                pass
                                
                    except (pickle.UnpicklingError, EOFError) as e:
                        # Data might be incomplete, stop processing and wait for more data
                        if self.debug:
                            print(f"Pickle error: {e}, buffer size: {len(buffer)}")
                        break  # Exit the while loop, keep current buffer
                    except Exception as e:
                        print(f"Unexpected error parsing pickle: {e}")
                        # Skip some bytes and try again
                        if len(buffer) > 1:
                            buffer = buffer[1:]
                        else:
                            buffer = b''
                        break
                
                # Prevent buffer from growing too large
                if len(buffer) > 4096:
                    if self.debug:
                        print(f"Buffer contents (first 100 bytes): {buffer[:100]}")
                    print(f"Buffer too large, clearing: {len(buffer)} bytes")
                    # Try to find a valid pickle header and start from there
                    pickle_start = buffer.find(b'\x80\x04')  # Look for pickle protocol 4 header
                    if pickle_start > 0:
                        buffer = buffer[pickle_start:]
                        if self.debug:
                            print(f"Found pickle header at position {pickle_start}, keeping {len(buffer)} bytes")
                    else:
                        buffer = b''  # No valid header found, clear everything
                    
            except socket.timeout:
                # Timeout is normal, just continue
                continue
            except socket.error as e:
                print(f"Socket error: {e}")
                self.connected = False
                continue
            except Exception as e:
                consecutive_errors += 1
                print(f"IMU data receive error ({consecutive_errors}/{max_consecutive_errors}): {e}")
                
                # Only disconnect after multiple consecutive errors
                if consecutive_errors >= max_consecutive_errors:
                    print("Too many consecutive errors, will attempt reconnection...")
                    self.connected = False
                    buffer = b''
                    continue
                else:
                    # Clear buffer and continue trying
                    buffer = b''
                    time.sleep(0.1)  # Brief pause before retry

            time.sleep(1 / self.freq)

    def get_imu(self):
        try:
            self.last_imu = self.imu_queue.get_nowait()
        except:
            pass
        return self.last_imu

    def is_connected(self):
        return self.connected

    def raw_inspect_mode(self):
        """Raw data inspection mode - shows what's actually being received"""
        print("Raw data inspection mode - showing received data...")
        buffer = b''
        packet_count = 0
        
        while self.should_run and self.connected:
            try:
                data = self.client_socket.recv(1024)
                if len(data) == 0:
                    print("Connection lost - server closed")
                    break
                
                packet_count += 1
                print(f"\nPacket {packet_count}: {len(data)} bytes")
                print(f"Raw bytes: {data}")
                print(f"As string (errors ignored): {data.decode('utf-8', errors='ignore')}")
                print(f"Hex: {data.hex()}")
                
                # Try to find patterns
                if b'quat' in data.lower() or b'euler' in data.lower():
                    print("  -> Contains 'quat' or 'euler' keywords")
                if data.startswith(b'\x80'):
                    print("  -> Starts with pickle protocol marker")
                if b'\n' in data:
                    print("  -> Contains newlines (might be text-based)")
                
                time.sleep(0.5)  # Slow down for readability
                
            except Exception as e:
                print(f"Error in raw inspection: {e}")
                break

    def close(self):
        print("Closing IMU client...")
        self.should_run = False
        self.connected = False
        try:
            if self.client_socket:
                self.client_socket.close()
        except:
            pass


def try_import_framesviewer():
    try:
        from FramesViewer.viewer import Viewer
        return Viewer
    except Exception as e:
        print(f"FramesViewer not available: {e}")
        return None


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='IMU Client for visualizing orientation data')
    parser.add_argument("--ip", type=str, required=True, help="IP address of the IMU server")
    parser.add_argument("--port", type=int, default=1234, help="Port number (default: 1234)")
    parser.add_argument("--freq", type=int, default=30, help="Update frequency (default: 30)")
    parser.add_argument("--console-only", action='store_true', default=False,
                       help="Run in console mode without 3D visualization")
    parser.add_argument("--force-qt", action='store_true', default=False,
                       help="Force PyQtGraph fallback even if FramesViewer is present")
    parser.add_argument("--no-gui", action='store_true', default=False,
                       help="Skip all GUI attempts and go straight to console mode")
    parser.add_argument("--debug", action='store_true', default=False,
                       help="Enable debug output for data inspection")
    parser.add_argument("--raw-mode", action='store_true', default=False,
                       help="Inspect raw data from server without trying to parse it")
    args = parser.parse_args()

    try:
        client = IMUClient(args.ip, port=args.port, freq=args.freq, debug=args.debug)
        if not client.connected:
            print("Failed to establish initial connection")
            exit(1)
    except Exception as e:
        print(f"Failed to create client: {e}")
        exit(1)

    # Raw inspection mode
    if args.raw_mode:
        try:
            client.raw_inspect_mode()
        except KeyboardInterrupt:
            print("\nStopping raw inspection...")
        finally:
            client.close()
        exit(0)

    visualization_success = False
    
    if not args.console_only and not args.no_gui:
        FV = try_import_framesviewer()
        if args.force_qt:
            # user explicitly wants PyQtGraph, ignore FramesViewer
            FV = None

        # Try FramesViewer first (if available and not forced to use Qt)
        if FV is not None:
            try:
                print("Starting 3D visualization (FramesViewer)...")
                fv = FV()
                fv.start()
                pose = np.eye(4)
                pose[:3, 3] = [0.1, 0.1, 0.1]
                print("3D viewer started successfully. Press Ctrl+C to stop.")
                visualization_success = True
                while client.is_connected():
                    quat = client.get_imu()
                    try:
                        rot_mat = R.from_quat(quat).as_matrix()
                        pose[:3, :3] = rot_mat
                        fv.pushFrame(pose, "imu_orientation")
                    except Exception as e:
                        print(f"Visualization error: {e}")
                    time.sleep(1 / float(max(args.freq, 1)))
            except KeyboardInterrupt:
                print("\nStopping viewer...")
                visualization_success = True  # User interrupted, not an error
            except Exception as e:
                print(f"3D visualization failed: {e}")
                print("Falling back to PyQtGraph...")
                visualization_success = False

        # If FramesViewer failed or wasn't available, try PyQtGraph
        if not visualization_success:
            try:
                print("Starting 3D visualization (PyQtGraph)...")
                run_pyqtgraph_viewer(client, freq_hz=args.freq)
                print("3D viewer closed.")
                visualization_success = True
            except Exception as e:
                print(f"PyQtGraph visualization failed: {e}")
                visualization_success = False

    # Console mode - run if no visualization succeeded or if explicitly requested
    if not visualization_success or args.console_only:
        print("Running in console mode. Displaying quaternion and euler angles.")
        print("Press Ctrl+C to stop.")
        try:
            while client.is_connected():
                quat = client.get_imu()
                try:
                    euler = R.from_quat(quat).as_euler('xyz', degrees=True)
                    print(f"Quaternion: [{quat[0]:.3f}, {quat[1]:.3f}, {quat[2]:.3f}, {quat[3]:.3f}]")
                    print(f"Euler (XYZ): Roll={euler[0]:.1f}°, Pitch={euler[1]:.1f}°, Yaw={euler[2]:.1f}°")
                    print("-" * 60)
                except Exception as e:
                    print(f"Data processing error: {e}")
                time.sleep(1)
        except KeyboardInterrupt:
            print("\nStopping client...")
        finally:
            client.close()

    print("IMU client terminated.")
