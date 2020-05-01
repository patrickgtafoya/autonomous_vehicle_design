import vpython as vp
import serial
from time import sleep
from math import *


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
    port = serial_connect(portName, 115200)

    vp.scene.range = 1.25
    ax = vp.arrow(shaftwidth=0.1, color=vp.color.white)
    ay = vp.arrow(shaftwidth=0.1, color=vp.color.green)
    az = vp.arrow(shaftwidth=0.1, color=vp.color.cyan)
    ax.pos = vp.vector(0, 0, 0)
    ay.pos = vp.vector(0, 0, 0)
    az.pos = vp.vector(0, 0, 0)
    ax.axis = vp.vector(1, 0, 0)
    ay.axis = vp.vector(0, 0, -1)
    az.axis = vp.vector(0, 1, 0)
    while True:
        while port.inWaiting() == 0:
            pass
        read_string = port.readline().decode("utf-8")
        data_array = read_string.split(',')

        roll = (float(data_array[4]) * pi/180)
        pitch = (float(data_array[5]) * pi/180)
        yaw = (float(data_array[6]) * pi/180)

        ax.axis = vp.norm(vp.vector(1, -pitch, yaw))
        ay.axis = vp.norm(vp.vector(yaw, roll, -1))
        az.axis = vp.norm(vp.vector(pitch, 1, roll))
        # az.axis = vp.norm(vp.vector(pitch, -roll, 1))

