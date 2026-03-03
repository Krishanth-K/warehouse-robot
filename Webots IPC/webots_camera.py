import socket
import cv2
import numpy as np
from ultralytics import YOLO

HOST = "127.0.0.1"
PORT = 5005

model = YOLO("yolov8n-pose.pt")
model.fuse()
qr = cv2.QRCodeDetector()

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1) 
sock.bind((HOST, PORT))
sock.listen(1)

print("Vision Worker ready...")

try:
    while True:
        print("Waiting for controller...")
        conn, addr = sock.accept()
        print("Controller connected:", addr)

        frame_count = 0

        try:
            while True:
                size_data = conn.recv(4)
                if not size_data:
                    print("Controller disconnected")
                    break

                size = int.from_bytes(size_data, "big")

                buf = b""
                while len(buf) < size:
                    packet = conn.recv(size - len(buf))
                    if not packet:
                        raise ConnectionResetError("Socket closed while receiving")
                    buf += packet

                img = np.frombuffer(buf, np.uint8)
                image = cv2.imdecode(img, cv2.IMREAD_COLOR)

                frame_count += 1
                if frame_count % 30 == 0:
                    print("Frames received:", frame_count)

                yolo_img = cv2.resize(image, None, fx=1.5, fy=1.5, interpolation=cv2.INTER_LINEAR)
                results = model(yolo_img, verbose=False)

                human_found = False
                for r in results:
                    if r.keypoints is not None and len(r.keypoints) > 0:
                        human_found = True
                        break

                image_qr = cv2.resize(image, None, fx=3.0, fy=3.0, interpolation=cv2.INTER_CUBIC)
                gray = cv2.cvtColor(image_qr, cv2.COLOR_BGR2GRAY)

                thresh = cv2.adaptiveThreshold(
                    gray, 255,
                    cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                    cv2.THRESH_BINARY, 31, 5
                )

                kernel = np.array([[0,-1,0],
                                   [-1,5,-1],
                                   [0,-1,0]])
                sharpened = cv2.filter2D(thresh, -1, kernel)

                data, bbox, _ = qr.detectAndDecode(sharpened)

                qr_data = data.strip() if data else "NONE"

                if data:
                    print("QR FOUND:", qr_data)

                if human_found:
                    reply = f"HUMAN;QR={qr_data}"
                else:
                    reply = f"NOHUMAN;QR={qr_data}"

                reply_bytes = reply.encode()
                conn.sendall(len(reply_bytes).to_bytes(4, "big"))
                conn.sendall(reply_bytes)

        except Exception as e:
            print("Connection lost:", e)

        finally:
            try:
                conn.shutdown(socket.SHUT_RDWR)
            except:
                pass  

            conn.close()
            print("Connection closed, waiting for reconnect...\n")

except KeyboardInterrupt:
    print("Worker shutting down...")

finally:
    sock.close()
