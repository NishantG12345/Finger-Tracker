import serial 
import time

arduino = serial.Serial('COM3', 9600)
time.sleep(2)

while True: 
    cmd = input('type a brightness from  0 to 255 or q')
    if cmd == 'q':
        break
    if int(cmd) >= 0 and int(cmd) <= 255: 
        arduino.write(f'B{cmd}\n'.encode())
        time.sleep(0.1)
arduino.close()