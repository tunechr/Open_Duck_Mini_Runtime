from picamzero import Camera
import cv2
from http.server import HTTPServer, BaseHTTPRequestHandler
import threading
import time
import json

class CameraHandler(BaseHTTPRequestHandler):
    camera = None
    current_frame = None
    lock = threading.Lock()
    
    def do_GET(self):
        if self.path == '/':
            self.send_response(200)
            self.send_header('Content-type', 'text/html')
            self.end_headers()
            
            html = """
            <!DOCTYPE html>
            <html>
            <head><title>Robot Camera</title></head>
            <body style="text-align: center; font-family: Arial;">
                <h1>🤖 Robot Camera Stream</h1>
                <img id="camera" src="/stream" style="max-width: 80%; border: 2px solid #ddd;">
                <br><br>
                <button onclick="location.reload()">Refresh</button>
                <script>
                    setInterval(() => {
                        document.getElementById('camera').src = '/stream?t=' + Date.now();
                    }, 500);
                </script>
            </body>
            </html>
            """
            self.wfile.write(html.encode())
            
        elif self.path.startswith('/stream'):
            with CameraHandler.lock:
                if CameraHandler.current_frame is not None:
                    self.send_response(200)
                    self.send_header('Content-type', 'image/jpeg')
                    self.send_header('Cache-Control', 'no-cache')
                    self.end_headers()
                    
                    _, buffer = cv2.imencode('.jpg', CameraHandler.current_frame, 
                                           [cv2.IMWRITE_JPEG_QUALITY, 80])
                    self.wfile.write(buffer.tobytes())
                else:
                    self.send_response(404)
                    self.end_headers()
        else:
            self.send_response(404)
            self.end_headers()

def capture_frames():
    print("Initializing camera...")
    camera = Camera()
    print("Camera initialized")
    
    while True:
        try:
            frame = camera.capture_array()
            frame = cv2.resize(frame, (640, 480))
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            
            with CameraHandler.lock:
                CameraHandler.current_frame = frame
                
            time.sleep(0.2)  # 5 FPS
        except Exception as e:
            print(f"Camera error: {e}")
            time.sleep(1)

if __name__ == '__main__':
    # Start camera capture thread
    capture_thread = threading.Thread(target=capture_frames, daemon=True)
    capture_thread.start()
    
    # Start web server
    server = HTTPServer(('0.0.0.0', 8000), CameraHandler)
    print("Camera server running at: http://<your-robot-ip>:8000")
    print("Press Ctrl+C to stop")
    
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nShutting down server...")
        server.shutdown()