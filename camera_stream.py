import io
import logging
import socketserver
from threading import Condition
from http import server
from picamera2 import Picamera2
from picamera2.outputs import FileOutput
from picamera2.encoders import MJPEGEncoder
from libcamera import Transform  # 🔧 ADD THIS IMPORT

PAGE = """\
<html>
<head><title>Robot POV</title></head>
<body style="background-color:black; color:white; text-align:center; font-family:sans-serif;">
    <h1>Mecanum Robot - Live Feed</h1>
    <img src="stream.mjpg" width="640" height="480" style="border: 3px solid #33cc33;" />
    <p style="color: #33cc33;">Streaming Activ - Picamera2 Hardware Encoder</p>
</body>
</html>
"""

class StreamingOutput(io.BufferedIOBase):
    def __init__(self):
        self.frame = None
        self.condition = Condition()

    def write(self, buf):
        if buf.startswith(b'\xff\xd8'):  # JPEG start marker
            with self.condition:
                self.frame = buf
                self.condition.notify_all()
        return len(buf)

    def readable(self):
        return False

    def writable(self):
        return True

class StreamingHandler(server.BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/':
            self.send_response(301)
            self.send_header('Location', '/index.html')
            self.end_headers()
        elif self.path == '/index.html':
            content = PAGE.encode('utf-8')
            self.send_response(200)
            self.send_header('Content-Type', 'text/html')
            self.send_header('Content-Length', len(content))
            self.end_headers()
            self.wfile.write(content)
        elif self.path == '/stream.mjpg':
            self.send_response(200)
            self.send_header('Age', 0)
            self.send_header('Cache-Control', 'no-cache, private')
            self.send_header('Pragma', 'no-cache')
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=FRAME')
            self.end_headers()
            try:
                while True:
                    with output.condition:
                        output.condition.wait()
                        frame = output.frame
                    
                    if frame:
                        self.wfile.write(b'--FRAME\r\n')
                        self.wfile.write(b'Content-Type: image/jpeg\r\n')
                        self.wfile.write(f'Content-Length: {len(frame)}\r\n'.encode())
                        self.wfile.write(b'\r\n')
                        self.wfile.write(frame)
                        self.wfile.write(b'\r\n')
            except Exception as e:
                logging.info(f'Client deconectat: {e}')
        else:
            self.send_error(404)

class StreamingServer(socketserver.ThreadingMixIn, server.HTTPServer):
    allow_reuse_address = True
    daemon_threads = True

# --- INITIALIZE CAMERA ---
output = StreamingOutput()
picam2 = Picamera2()

config = picam2.create_video_configuration(
    main={"format": 'YUV420', "size": (640, 480)},
    transform=Transform(hflip=1, vflip=1)  # 🔧 ROTATE 180 DEGREES
)
picam2.configure(config)
    
try:
    encoder = MJPEGEncoder()
    output_file = FileOutput(output)
    
    picam2.start_recording(encoder, output_file)
    print("Succes! MJPEG recording started without errors.")
except Exception as e:
    print(f"Eroare la pornire inregistrare: {e}")
    import traceback
    traceback.print_exc()

try:
    address = ('', 8000)
    server_inst = StreamingServer(address, StreamingHandler)
    print("Serverul ruleaza! http://192.168.137.212:8000")
    server_inst.serve_forever()
except KeyboardInterrupt:
    print("\nOprire manuală detectată...")
finally:
    print("Închidere resurse cameră...")
    try:
        picam2.stop_recording()
        picam2.stop()
        picam2.close()
    except:
        pass