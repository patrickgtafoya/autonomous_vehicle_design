import biasOffsetCalibration as b
from copy import deepcopy


def state_two(port, dict_1, dict_2, dict_3, k, sig):
    next_state = False
    # iteration count to write to table file
    index = 1
    while not next_state:
        # receive data, looking for 7 bytes
        if port.inWaiting() > 6:
            data = receive_data(port)
            # print(str(index) + '\t' + bin(data[0]))  # debug
            next_state = b.look_for_stop(data[4], port, sig)
            write_data(data, dict_1, dict_2, dict_3, k)
            index += 1


def state_three(dict_1, dict_2, dict_3, keys):
    # lists to store drift for each frequency at 1/10, 1/2 and full second
    # [0]: 1/10 sec, [1] 1/2 sec, [2] 1 sec
    drift_1000hz = []
    drift_200hz = []
    drift_100hz = []
    # 1khz
    drift_1000hz.append(calculate_orientation(dict_1, keys, div=10))
    drift_1000hz.append(calculate_orientation(dict_1, keys, div=2))
    drift_1000hz.append(calculate_orientation(dict_1, keys))
    # 200hz
    drift_200hz.append(calculate_orientation(dict_2, keys, div=10))
    drift_200hz.append(calculate_orientation(dict_2, keys, div=2))
    drift_200hz.append(calculate_orientation(dict_2, keys))
    # 100hz
    drift_100hz.append(calculate_orientation(dict_3, keys, div=10))
    drift_100hz.append(calculate_orientation(dict_3, keys, div=2))
    drift_100hz.append(calculate_orientation(dict_3, keys))
    return [drift_1000hz, drift_200hz, drift_100hz]


def receive_data(port):
    # looking for 7 bytes of data
    read_bytes = port.read(7)
    identifier = int.from_bytes(read_bytes[0:1], byteorder='big', signed=True)
    gx = float(int.from_bytes(read_bytes[1:3], byteorder='big', signed=True)) / 10000.0
    gy = float(int.from_bytes(read_bytes[3:5], byteorder='big', signed=True)) / 10000.0
    gz = float(int.from_bytes(read_bytes[5:], byteorder='big', signed=True)) / 10000.0
    stop_check = int.from_bytes(read_bytes, byteorder='big', signed=False)
    return identifier, gx, gy, gz, stop_check


def write_data(data, dict_1000, dict_200, dict_100, keys):
    # check if 1000Hz data
    if data[0] & 0x01:
        dict_1000[keys[0]].append(data[1])
        dict_1000[keys[1]].append(data[2])
        dict_1000[keys[2]].append(data[3])
    # check if 200Hz data
    if data[0] & 0x02:
        dict_200[keys[0]].append(data[1])
        dict_200[keys[1]].append(data[2])
        dict_200[keys[2]].append(data[3])
    # check if 100Hz data
    if data[0] & 0x04:
        dict_100[keys[0]].append(data[1])
        dict_100[keys[1]].append(data[2])
        dict_100[keys[2]].append(data[3])


def calculate_orientation(hz_dict, keys, div=1):
    # length of data
    delta = len(hz_dict[keys[0]])
    # slice index value
    sl = int(delta / div)
    x_data = hz_dict[keys[0]][0:sl]
    y_data = hz_dict[keys[1]][0:sl]
    z_data = hz_dict[keys[2]][0:sl]
    # change in time
    delta_t = 1.0 / delta
    x_orientation = 0
    y_orientation = 0
    z_orientation = 0
    # calculate drift
    for index in range(1, len(x_data)):
        x_orientation += (x_data[index] + x_data[index-1]) * (delta_t / 2.0)
        y_orientation += (y_data[index] + y_data[index-1]) * (delta_t / 2.0)
        z_orientation += (z_data[index] + z_data[index-1]) * (delta_t / 2.0)
    # print('X:\t' + str(x_orientation))  # debug
    # print('Y:\t' + str(y_orientation))  # debug
    # print('Z:\t' + str(z_orientation))  # debug
    return x_orientation, y_orientation, z_orientation


if __name__ == "__main__":
    portName = "COM4"
    file = 'measuredError.txt'
    title = 'Measured Error After Calibration'
    stop = 0
    start = 0xFF
    baud_rate = 115200
    # lists for reading data
    key_list = ['Gyro X', 'Gyro Y', 'Gyro Z']
    hz_100 = {key_list[0]: [], key_list[1]: [], key_list[2]: []}
    hz_200 = deepcopy(hz_100)
    hz_1000 = deepcopy(hz_100)
    ser = b.initialize_serial(portName, baud_rate)
    b.state_one(ser, file, title, start)
    state_two(ser, hz_1000, hz_200, hz_100, key_list, stop)
    drift = state_three(hz_1000, hz_200, hz_100, key_list)
    # write drift calculations to file
    with open('driftCalculations.txt', 'w') as f:
        f.write('Drift Calculations\n')
        f.write('\n1 kHz:\n')
        f.write('\t1/10 sec:\n')
        f.write('\t\tX:\t' + str(drift[0][0][0]) + '\n')
        f.write('\t\tY:\t' + str(drift[0][0][1]) + '\n')
        f.write('\t\tZ:\t' + str(drift[0][0][2]) + '\n')
        f.write('\t1/2 sec:\n')
        f.write('\t\tX:\t' + str(drift[0][1][0]) + '\n')
        f.write('\t\tY:\t' + str(drift[0][1][1]) + '\n')
        f.write('\t\tZ:\t' + str(drift[0][1][2]) + '\n')
        f.write('\t1 sec:\n')
        f.write('\t\tX:\t' + str(drift[0][2][0]) + '\n')
        f.write('\t\tY:\t' + str(drift[0][2][1]) + '\n')
        f.write('\t\tZ:\t' + str(drift[0][2][2]) + '\n')

        f.write('Drift Calculations\n')
        f.write('\n200 Hz:\n')
        f.write('\t1/10 sec:\n')
        f.write('\t\tX:\t' + str(drift[1][0][0]) + '\n')
        f.write('\t\tY:\t' + str(drift[1][0][1]) + '\n')
        f.write('\t\tZ:\t' + str(drift[1][0][2]) + '\n')
        f.write('\t1/2 sec:\n')
        f.write('\t\tX:\t' + str(drift[1][1][0]) + '\n')
        f.write('\t\tY:\t' + str(drift[1][1][1]) + '\n')
        f.write('\t\tZ:\t' + str(drift[1][1][2]) + '\n')
        f.write('\t1 sec:\n')
        f.write('\t\tX:\t' + str(drift[1][2][0]) + '\n')
        f.write('\t\tY:\t' + str(drift[1][2][1]) + '\n')
        f.write('\t\tZ:\t' + str(drift[1][2][2]) + '\n')

        f.write('Drift Calculations\n')
        f.write('\n100 Hz:\n')
        f.write('\t1/10 sec:\n')
        f.write('\t\tX:\t' + str(drift[2][0][0]) + '\n')
        f.write('\t\tY:\t' + str(drift[2][0][1]) + '\n')
        f.write('\t\tZ:\t' + str(drift[2][0][2]) + '\n')
        f.write('\t1/2 sec:\n')
        f.write('\t\tX:\t' + str(drift[2][1][0]) + '\n')
        f.write('\t\tY:\t' + str(drift[2][1][1]) + '\n')
        f.write('\t\tZ:\t' + str(drift[2][1][2]) + '\n')
        f.write('\t1 sec:\n')
        f.write('\t\tX:\t' + str(drift[2][2][0]) + '\n')
        f.write('\t\tY:\t' + str(drift[2][2][1]) + '\n')
        f.write('\t\tZ:\t' + str(drift[2][2][2]) + '\n')

        f.close()



