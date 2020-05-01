import serial
import matplotlib as plt
from time import *


def serial_connect(port, baud):
    try:
        ser = serial.Serial(port, baud)
        print("opened port " + ser.name + '\n')
        # give Arduino time to reset
        sleep(2)
        # flush input buffer, discarding all contents
        ser.reset_input_buffer()
        return ser
    except serial.SerialException:
        raise IOError("problem connecting to " + port)


if __name__ == '__main__':
    portName = 'COM4'
    fileName = 'angleData.txt'
    sp = serial_connect(portName, 115200)
    accelData = []
    gyroData = []
    compData = []
    dataPoint = 0
    pointsNeeded = 15000
    with open(fileName, 'w') as f:
        f.writelines('Angle Data\n')
    while dataPoint < pointsNeeded:
        while sp.inWaiting() == 0:
            pass

        read_string = sp.readline().decode('utf-8')
        # print(read_string)  # debug
        data = read_string.split(',')
        compData.append(data[0])
        accelData.append(data[1])
        gyroData.append(data[2])
        with open(fileName, 'a') as f:
            f.writelines(read_string)
        dataPoint += 1

