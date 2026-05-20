from gz.transport13 import Node
import cv2
import numpy as np
from gz.msgs10.image_pb2 import Image
from pyzbar.pyzbar import decode

topic = "/world/iris_runway/model/iris_with_gimbal/model/gimbal/link/pitch_link/sensor/camera/image"

node = Node()

def callback(msg):
    img = np.frombuffer(msg.data, dtype=np.uint8)

    frame = img.reshape((msg.height, msg.width, 3))

    qr_codes = decode(frame)

    for qr in qr_codes:
        x, y, w, h = qr.rect

        cv2.rectangle(frame, (x, y), (x+w, y+h), (0,255,0), 2)

        text = qr.data.decode("utf-8")

        cv2.putText(frame, text, (x, y-10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (0,255,0), 2)

        print("QR Detected:", text)

    cv2.imshow("Drone Camera", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        exit()

node.subscribe(Image, topic, callback)

print("QR Detection Started...")

while True:
    pass
