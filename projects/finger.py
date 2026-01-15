import cv2 as cv
import numpy as np
import serial
import time
cam = cv.VideoCapture(0)
arduino = serial.Serial('COM3', 9600)
time.sleep(2)

while True:
    ret, frame = cam.read()
    y = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    lower = np.array([0,48,80])
    upper = np.array([20,255,255])
    mask = cv.inRange(y, lower, upper)
    mask = cv.GaussianBlur(mask, (5, 5), 0)
    kernel = np.ones((5,5), np.uint8)
    mask = cv.morphologyEx(mask, cv.MORPH_OPEN, kernel)
    mask[:100, :] = 0
    cv.imshow('mask',mask)
    contours, hierarchy = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    if len(contours) == 0:
        continue
    hand_contour = max(contours, key= cv.contourArea)
    cv.drawContours(frame, [hand_contour], -1, (0,255,0),2)
    cv.imshow('frame', frame)
    hull = cv.convexHull(hand_contour)
    cv.drawContours(frame, [hull], -1, (255,0,0), 2)
    cv.imshow('g', frame)
    hullind = cv.convexHull(hand_contour, returnPoints=False)
    defects = cv.convexityDefects(hand_contour, hullind)
    num = 0
    for i in range(defects.shape[0]):
        s, e, f,d = defects[i,0]
        start = tuple(hand_contour[s][0])
        end = tuple(hand_contour[e][0])
        far = tuple(hand_contour[f][0])
        a = np.linalg.norm(np.array(far) - np.array(start))
        b = np.linalg.norm(np.array(far) - np.array(end))
        c = np.linalg.norm(np.array(start) - np.array(end))

        angle = np.arccos((a**2 + b**2 - c**2) / (2 * a * b))

        if angle <= np.pi / 2 and d > 10000:
            num += 1
    num += 1  
    cv.putText(frame, 'fingers' + str(num), (100,100), cv.FONT_HERSHEY_SIMPLEX,1.5, (0,255,0), 3)
    cv.imshow('frick', frame)
    arduino.write(str(num).encode())
    if cv.waitKey(20) & 0xFF==ord('q'):
        break
cv.destroyAllWindows()
  