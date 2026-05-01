#!/usr/bin/env python3
import io
import cv2
import logging
import socketserver
import time
from threading import Condition, Thread
from http import server
from picamera2 import Picamera2
from libcamera import Transform
from ultralytics import YOLO

FRAME_SIZE = (640, 360)
YOLO_IMGSZ = 320
DETECT_EVERY_N_FRAMES = 3
CONF_THRESHOLD = 0.35
JPEG_QUALITY = 70

PAGE = """\
<html>
<head><title>Robot YOLO POV</title></head>
<body style="background:black; color:white; text-align:center; font-family:sans-serif;">
<h1>Mecanum Robot - YOLO Live Feed</h1>
<img src="stream.mjpg" width="640" height="480" style="border:3px solid #33cc33;" />
<p>YOLO object detection active</p>
</body>
</html>
"""

class StreamingOutput:
    def __init__(self):
        self.frame = None
        self.condition = Condition()

    def update(self, jpeg):
        with self.condition:
            self.frame = jpeg
            self.condition.notify_all()

output = StreamingOutput()

class StreamingHandler(server.BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == "/":
            self.send_response(301)
            self.send_header("Location", "/index.html")
            self.end_headers()

        elif self.path == "/index.html":
            content = PAGE.encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/html")
            self.send_header("Content-Length", len(content))
            self.end_headers()
            self.wfile.write(content)

        elif self.path == "/stream.mjpg":
            self.send_response(200)
            self.send_header("Cache-Control", "no-cache")
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=FRAME")
            self.end_headers()

            try:
                while True:
                    with output.condition:
                        output.condition.wait()
                        frame = output.frame

                    self.wfile.write(b"--FRAME\r\n")
                    self.wfile.write(b"Content-Type: image/jpeg\r\n")
                    self.wfile.write(f"Content-Length: {len(frame)}\r\n\r\n".encode())
                    self.wfile.write(frame)
                    self.wfile.write(b"\r\n")

            except Exception as e:
                logging.info(f"Client disconnected: {e}")

        else:
            self.send_error(404)

class StreamingServer(socketserver.ThreadingMixIn, server.HTTPServer):
    allow_reuse_address = True
    daemon_threads = True


def draw_cached_detections(frame, detections):
    for det in detections:
        x1, y1, x2, y2, conf, label = det
        cv2.rectangle(frame, (x1, y1), (x2, y2), (51, 204, 51), 2)
        text = f"{label} {conf:.2f}"
        (text_w, text_h), baseline = cv2.getTextSize(
            text,
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            1,
        )
        top_y = max(y1, text_h + baseline + 2)
        cv2.rectangle(
            frame,
            (x1, top_y - text_h - baseline - 2),
            (x1 + text_w + 4, top_y),
            (51, 204, 51),
            -1,
        )
        cv2.putText(
            frame,
            text,
            (x1 + 2, top_y - baseline - 1),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 0, 0),
            1,
            cv2.LINE_AA,
        )


def extract_detections(result, class_names):
    detections = []
    boxes = result.boxes
    if boxes is None:
        return detections

    xyxy_list = boxes.xyxy.tolist()
    conf_list = boxes.conf.tolist()
    cls_list = boxes.cls.tolist()

    for xyxy, conf, cls_id in zip(xyxy_list, conf_list, cls_list):
        x1, y1, x2, y2 = [int(v) for v in xyxy]
        label = class_names.get(int(cls_id), str(int(cls_id)))
        detections.append((x1, y1, x2, y2, float(conf), label))

    return detections

def detection_loop():
    model = YOLO("yolo11n.pt")
    class_names = model.names if isinstance(model.names, dict) else {}

    picam2 = Picamera2()
    config = picam2.create_video_configuration(
        main={"format": "RGB888", "size": FRAME_SIZE},
        transform=Transform(hflip=1, vflip=1)
    )
    picam2.configure(config)
    picam2.start()

    frame_count = 0
    last_detections = []
    encode_params = [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY]

    while True:
        frame = picam2.capture_array()
        frame_count += 1

        if frame_count % DETECT_EVERY_N_FRAMES == 0:
            # Removing the class filter allows the model to return all COCO classes,
            # so smaller objects like remotes, bottles, and phones can be detected again.
            # A larger imgsz gives the detector more detail to work with for small objects,
            # which improves accuracy at the cost of more compute and lower FPS.
            # A lower confidence threshold surfaces weaker detections that may be useful,
            # but it can also introduce more false positives.
            results = model.predict(
                frame,
                imgsz=YOLO_IMGSZ,
                conf=CONF_THRESHOLD,
                verbose=False,
            )
            last_detections = extract_detections(results[0], class_names)

        draw_cached_detections(frame, last_detections)

        ok, jpeg = cv2.imencode(".jpg", frame, encode_params)
        if ok:
            output.update(jpeg.tobytes())
        else:
            time.sleep(0.001)

Thread(target=detection_loop, daemon=True).start()

address = ("", 8000)
server_inst = StreamingServer(address, StreamingHandler)
print("YOLO stream running on http://192.168.137.212:8000")
server_inst.serve_forever()