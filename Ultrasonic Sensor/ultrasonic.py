import serial
from time import sleep
from matplotlib import pyplot as pyp


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


def send_ack(port, signal):
    port.write(signal)


def look_for_start(port):
    wait = True
    while wait:
        while port.inWaiting() == 0:
            pass
        read_string = port.readline().decode('utf-8').strip()
        if read_string == 'start':
            # print('Start Received')  # debug
            port.reset_input_buffer()
            sleep(2)
            send_ack(port, 1)
            wait = False


def receive_data(port, rl, fl):
    receive = True
    while receive:
        while port.inWaiting() == 0:
            pass
        read_string = port.readline().decode('utf-8').strip()
        if read_string == 'start':
            # print('String is: start')  # debug
            pass
        elif read_string == 'stop':
            # print('String is: stop')  # debug
            receive = False
        else:
            try:
                data = read_string.split(',')
                filtered_data.append(float(data[0]))
                raw_data.append(float(data[1]))
            except TypeError:
                pass


def plot_data(rl, fl):
    t = []
    for i in range(len(rl)):
        t.append(i)

    pyp.plot(t, rl, 'r', label='Raw')
    pyp.plot(t, fl, 'g', label='Filtered')
    pyp.legend()
    pyp.title('Ultrasonic Sensor Data')
    pyp.xlabel('Sample Count')
    pyp.ylabel('Distance [cm]')
    pyp.savefig('rawVsFilteredPlot.png')
    pyp.show()


def accuracy_precision(rl, fl, expected, file, title, buffer_size):
    from statistics import stdev, median, mean, mode
    # remove filtered values before buffer is filled
    for i in range(buffer_size):
        fl.pop(0)
        rl.pop(0)
    maximum = [max(rl), max(fl)]
    minimum = [min(rl), min(fl)]
    mean = [mean(rl), mean(fl)]
    median = [median(rl), median(fl)]
    mode = [mode(rl), mode(fl)]
    error = [abs((expected - mean[0]) / expected)*100, abs((expected - mean[1]) / expected)*100]
    precision = [stdev(rl), stdev(fl)]

    with open(file, 'w') as f:
        f.write(title + '\n\n')
        f.write('Raw Data\n')
        f.write('\tMaximum: ' + str(maximum[0]) + '\n')
        f.write('\tMinimum: ' + str(minimum[0]) + '\n')
        f.write('\tMean: ' + str(mean[0]) + '\n')
        f.write('\tMedian: ' + str(median[0]) + '\n')
        f.write('\tMode: ' + str(mode[0]) + '\n')
        f.write('\tExpected: ' + str(expected) + '\n')
        f.write('\tError: ' + str(error[0]) + '%\n')
        f.write('\tStandard Deviation: ' + str(precision[0]) + '\n\n')

        f.write('Filtered Data\n')
        f.write('\tMaximum: ' + str(maximum[1]) + '\n')
        f.write('\tMinimum: ' + str(minimum[1]) + '\n')
        f.write('\tMean: ' + str(mean[1]) + '\n')
        f.write('\tMedian: ' + str(median[1]) + '\n')
        f.write('\tMode: ' + str(mode[1]) + '\n')
        f.write('\tExpected: ' + str(expected) + '\n')
        f.write('\tError: ' + str(error[1]) + '%\n')
        f.write('\tStandard Deviation: ' + str(precision[1]) + '\n\n')
        f.close()


if __name__ == '__main__':
    portName = 'COM4'
    raw_data = []
    filtered_data = []
    sp = serial_connect(portName, 115200)
    look_for_start(sp)
    receive_data(sp, raw_data, filtered_data)
    send_ack(sp, 2)
    plot_data(raw_data, filtered_data)
    accuracy_precision(raw_data, filtered_data, 30.0, 'accuracyAndPrecision.txt', 'Accuracy and Precision', 5)
