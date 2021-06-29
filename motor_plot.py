import serial
import time
import matplotlib.pyplot as plt

ser= serial.Serial("/dev/ttyUSB0", 115200, timeout=1)
time.sleep(2)

data = []

while True:
    line= ser.readline()
    if line:
        string= line.decode()
        print(string)
        data.append(string)
ser.close()