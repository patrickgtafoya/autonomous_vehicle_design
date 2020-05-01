import serial
from time import sleep
from matplotlib import pyplot as plt
from matplotlib import animation


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


def init():
    # graph_roll.set_data([], [])
    for item in graph_roll:
        item[0].set_data([], [])
    for item in graph_pitch:
        item[0].set_data([], [])
    graph_yaw.set_data([], [])
    return graph_roll[0][0], graph_roll[1][0], graph_roll[2][0], graph_pitch[0][0], graph_pitch[1][0], graph_pitch[2][0], graph_yaw


def animate(i):
    global t, acc_roll, acc_pitch, gyro_roll, gyro_pitch, comp_roll, comp_pitch, yaw
    while ser.inWaiting() == 0:
        pass

    read_string = ser.readline().decode("utf-8")
    data_array = read_string.split(',')
    # print(read_string)
    acc_roll.append(float(data_array[0]))
    acc_pitch.append(float(data_array[1]))
    gyro_roll.append(float(data_array[2]))
    gyro_pitch.append(float(data_array[3]))
    comp_roll.append(float(data_array[4]))
    comp_pitch.append(float(data_array[5]))
    yaw.append(float(data_array[6]))

    acc_roll.pop(0)
    acc_pitch.pop(0)
    gyro_roll.pop(0)
    gyro_pitch.pop(0)
    comp_roll.pop(0)
    comp_pitch.pop(0)
    yaw.pop(0)

    graph_roll[0][0].set_data(t, acc_roll)
    graph_roll[1][0].set_data(t, gyro_roll)
    graph_roll[2][0].set_data(t, comp_roll)

    graph_pitch[0][0].set_data(t, acc_pitch)
    graph_pitch[1][0].set_data(t, gyro_pitch)
    graph_pitch[2][0].set_data(t, comp_pitch)

    graph_yaw.set_data(t, yaw)

    return graph_roll[0][0], graph_roll[1][0], graph_roll[2][0], graph_pitch[0][0], graph_pitch[1][0], graph_pitch[2][0], graph_yaw


if __name__ == '__main__':

    portName = "COM4"
    ser = serial_connect(portName, 115200)
    sleep(2)  # give Arduino time to reset

    # flush input buffer, discarding all contents
    ser.reset_input_buffer()

    numPoints = 201  # number of data points
    fig = plt.figure(figsize=(12, 6))  # create figure window
    # add sub plots
    roll = fig.add_subplot(3, 1, 1)
    roll.set_title('Roll')
    pitch = fig.add_subplot(3, 1, 2)
    pitch.set_title('Pitch')
    yaw = fig.add_subplot(3, 1, 3)
    yaw.set_title('Yaw')

    # get Line2D object for each sub plot

    graph_roll = roll.plot([], [], 'b', label='Accel'), roll.plot([], [], 'r', label='Gyro'), roll.plot([], [], 'g', label='Comp')
    graph_pitch = pitch.plot([], [], 'b', label='Accel'), pitch.plot([], [], 'r', label='Gyro'), pitch.plot([], [], 'g', label='Comp')
    graph_yaw, = yaw.plot([], [], 'g', label='Comp')
    axes = [roll, pitch, yaw]

    # print(type(graph_roll[0][0]))  # debug
    # print(graph_roll[0][0])  # debug

    # specify axes limits, labels, and legend location
    for ax in axes:
        ax.set_xlim(0, numPoints - 1)
        ax.set_ylim(-90, 90)
        ax.set_ylabel('Orientation [Degrees]')
        ax.legend(loc='upper right')
        ax.grid(True)

    # graph_roll.set_title('Real-time sensor data')
    # graph_yaw.set_xlabel('Data points')

    t = list(range(0, numPoints))
    acc_roll = []
    acc_pitch = []
    gyro_roll = []
    gyro_pitch = []
    comp_roll = []
    comp_pitch = []
    yaw = []

    for i in range(0, numPoints):
        acc_roll.append(0)
        acc_pitch.append(0)
        gyro_roll.append(0)
        gyro_pitch.append(0)
        comp_roll.append(0)
        comp_pitch.append(0)
        yaw.append(0)

    delay = 20
    anim = animation.FuncAnimation(fig, animate, init_func=init,
                                   interval=delay, blit=True)

    plt.show()