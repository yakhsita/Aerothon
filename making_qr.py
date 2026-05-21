import cv2
from pyzbar.pyzbar import decode

img = cv2.imread("/home/yakhsita-hansdah/qr_marker/materials/textures/qr.png")

qr = decode(img)

print(qr)
