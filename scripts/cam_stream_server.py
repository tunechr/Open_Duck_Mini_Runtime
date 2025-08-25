from picamzero import Camera
import cv2
from flask import Flask, render_template, Response
import threading
import time
from io import BytesIO

app = Flask(__name__)

class CameraStreamer:
    def __init__(self):
        print("Initializing camera...")
        self.camera = Camera()
        print("Camera initialized")
        self.frame = None
        self.lock = threading.Lock()
        
        # Start camera capture thread
        self.capture_thread = threading.Thread(target=self._capture_frames, daemon=True)
        self.capture_thread.start()
    
    def _capture_frames(self):
        while True:
            try:
                # Capture frame
                frame = self.camera.capture_array()
                frame = cv2.resize(frame, (640, 480))  # Resize for web streaming
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                
                with self.lock:
                    self.frame = frame
                    
                time.sleep(0.1)  # ~10 FPS
            except Exception as e:
                print(f"Camera capture error: {e}")
                time.sleep(1)
    
    def get_frame(self):
        with self.lock:
            if self.frame is not None:
                # Encode frame as JPEG
                _, buffer = cv2.imencode('.jpg', self.frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
                return buffer.tobytes()
        return None

# Global camera instance
camera_streamer = CameraStreamer()

@app.route('/')
def index():
    return render_template('camera.html')

@app.route('/video_feed')
def video_feed():
    def generate():
        while True:
            frame = camera_streamer.get_frame()
            if frame:
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
            time.sleep(0.1)
    
    return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/snapshot')
def snapshot():
    frame = camera_streamer.get_frame()
    if frame:
        return Response(frame, mimetype='image/jpeg')
    return "No image available", 404

if __name__ == '__main__':
    print("Starting camera web server...")
    print("Access the stream at: http://<your-robot-ip>:5000")
    app.run(host='0.0.0.0', port=5000, debug=False, threaded=True)