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
import RPi.GPIO as GPIO
import smbus

FRAME_SIZE = (640, 360)
YOLO_IMGSZ = 320
DETECT_EVERY_N_FRAMES = 3
CONF_THRESHOLD = 0.35
JPEG_QUALITY = 70

# =========================================================
# GPIO SETUP FOR ULTRASONIC SENSORS
# =========================================================
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

TRIG = 16
ECHO_FRONT = 26
ECHO_LEFT = 14
ECHO_RIGHT = 8

GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO_FRONT, GPIO.IN)
GPIO.setup(ECHO_LEFT, GPIO.IN)
GPIO.setup(ECHO_RIGHT, GPIO.IN)
GPIO.output(TRIG, False)

# =========================================================
# MPU6050 I2C SETUP
# =========================================================
MPU6050_ADDR = 0x68
PWR_MGMT_1 = 0x6B
GYRO_XOUT_H = 0x43
GYRO_YOUT_H = 0x45
GYRO_ZOUT_H = 0x47

try:
    i2c_bus = smbus.SMBus(1)
    i2c_bus.write_byte_data(MPU6050_ADDR, PWR_MGMT_1, 0)
    mpu6050_available = True
except Exception as e:
    logging.warning(f"MPU6050 not available: {e}")
    mpu6050_available = False

# Current sensor readings
sensor_readings = {"front": -1, "left": -1, "right": -1, "gx": 0, "gy": 0, "gz": 0}

PAGE = """\
<html>
<head><title>Robot YOLO POV</title></head>
<body style="background:black; color:white; text-align:center; font-family:sans-serif;">
<h1>Mecanum Robot - YOLO Live Feed</h1>
<img src="stream.mjpg" width="640" height="480" style="border:3px solid #33cc33;" />
<p>YOLO object detection + Ultrasonic Sensors active</p>
</body>
</html>
"""

# =========================================================
# DISTANCE READING FUNCTION
# =========================================================

def read_distance(echo_pin):
    """Read distance from ultrasonic sensor"""
    try:
        GPIO.output(TRIG, True)
        time.sleep(0.00001)
        GPIO.output(TRIG, False)

        pulse_start = time.time()
        timeout = pulse_start

        while GPIO.input(echo_pin) == 0:
            pulse_start = time.time()
            if pulse_start - timeout > 0.03:
                return -1

        pulse_end = time.time()

        while GPIO.input(echo_pin) == 1:
            pulse_end = time.time()
            if pulse_end - pulse_start > 0.03:
                return -1

        pulse_duration = pulse_end - pulse_start
        distance = pulse_duration * 17150
        return round(distance, 1)
    except Exception as e:
        logging.warning(f"Sensor read error: {e}")
        return -1

def read_word(reg):
    """Read 16-bit word from MPU6050"""
    try:
        high = i2c_bus.read_byte_data(MPU6050_ADDR, reg)
        low = i2c_bus.read_byte_data(MPU6050_ADDR, reg + 1)
        value = (high << 8) + low
        if value >= 0x8000:
            value = -((65535 - value) + 1)
        return value
    except Exception as e:
        logging.warning(f"MPU6050 read error: {e}")
        return 0

def get_gyro():
    """Read gyroscope values from MPU6050 (degrees/sec)"""
    if not mpu6050_available:
        return 0, 0, 0
    try:
        gx = read_word(GYRO_XOUT_H) / 131.0
        gy = read_word(GYRO_YOUT_H) / 131.0
        gz = read_word(GYRO_ZOUT_H) / 131.0
        return round(gx, 2), round(gy, 2), round(gz, 2)
    except Exception as e:
        logging.warning(f"Gyro read error: {e}")
        return 0, 0, 0

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


def draw_cached_detections(frame, detections, sensor_data=None):
    # Draw YOLO detections
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

    # Draw sensor readings at bottom of frame
    if sensor_data:
        front = sensor_data.get("front", -1)
        left = sensor_data.get("left", -1)
        right = sensor_data.get("right", -1)
        gx = sensor_data.get("gx", 0)
        gy = sensor_data.get("gy", 0)
        gz = sensor_data.get("gz", 0)

        sensor_text = f"Front: {front:.1f}cm | Left: {left:.1f}cm | Right: {right:.1f}cm"
        gyro_text = f"Gyro - X: {gx:6.1f}°/s  Y: {gy:6.1f}°/s  Z: {gz:6.1f}°/s"

        # Semi-transparent background for sensor info (ultrasonic)
        (text_w, text_h), baseline = cv2.getTextSize(
            sensor_text,
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            2,
        )

        y_pos = frame.shape[0] - 10
        cv2.rectangle(
            frame,
            (5, y_pos - text_h - baseline - 8),
            (15 + text_w, y_pos + 2),
            (0, 0, 255),
            -1,
        )

        cv2.putText(
            frame,
            sensor_text,
            (10, y_pos - baseline - 2),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )

        # Draw gyroscope info above sensor info
        (text_w_gyro, text_h_gyro), baseline_gyro = cv2.getTextSize(
            gyro_text,
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            1,
        )

        y_pos_gyro = y_pos - text_h - baseline - 15
        cv2.rectangle(
            frame,
            (5, y_pos_gyro - text_h_gyro - baseline_gyro - 8),
            (15 + text_w_gyro, y_pos_gyro + 2),
            (255, 100, 0),
            -1,
        )

        cv2.putText(
            frame,
            gyro_text,
            (10, y_pos_gyro - baseline_gyro - 2),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 255, 255),
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

        # Read sensor data every 5 frames
        if frame_count % 5 == 0:
            sensor_readings["front"] = read_distance(ECHO_FRONT)
            sensor_readings["left"] = read_distance(ECHO_LEFT)
            sensor_readings["right"] = read_distance(ECHO_RIGHT)
            sensor_readings["gx"], sensor_readings["gy"], sensor_readings["gz"] = get_gyro()

        draw_cached_detections(frame, last_detections, sensor_readings)

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