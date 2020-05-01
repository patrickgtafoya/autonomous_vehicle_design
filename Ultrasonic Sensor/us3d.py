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
    vp.scene.range = 10
    us = vp.box(length=5.0, height=0.1, width=2.0, color=vp.vector(0, 0, 1), pos=vp.vector(0, -4, 0))
    t = vp.cylinder(radius=0.75, length=0.5, axis=vp.vector(0, 1, 0), pos=vp.vector(1, -4, 0))
    r = vp.cylinder(radius=0.75, length=0.5, axis=vp.vector(0, 1, 0), pos=vp.vector(-1, -4, 0))
    beam = vp.cone(radius=1, length=0.1, axis=vp.vector(0, -1, 0), pos=vp.vector(0, -2.4, 0),
                   color=vp.vector(1, 0, 1), opacity=0.5)
    while True:
        while port.inWaiting() == 0:
            pass
        read_string = port.readline().decode("utf-8")
        data_array = read_string.split(',')
        distanceF = float(data_array[0])
        print(distanceF)
        center = distanceF - 2.4
        rad = distanceF*tan(pi / 6.0)
        if distanceF > 40:
            vp.scene.range = 40
            us.pos.y = -39
            t.pos.y = -39
            r.pos.y = -39
            center = center - 36
        elif distanceF > 30:
            vp.scene.range = 30
            us.pos.y = -29
            t.pos.y = -29
            r.pos.y = -29
            center = center - 26
        elif distanceF > 20:
            vp.scene.range = 20
            us.pos.y = -19
            t.pos.y = -19
            r.pos.y = -19
            center = center - 16
        else:
            vp.scene.range = 10
            us.pos.y = -9
            t.pos.y = -9
            r.pos.y = -9
            center = center - 7
        beam.axis = vp.vector(0, -distanceF, 0)
        beam.pos.y = center
        beam.radius = rad
