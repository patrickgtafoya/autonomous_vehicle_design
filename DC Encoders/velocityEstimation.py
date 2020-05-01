import serial
from time import sleep
from copy import deepcopy
from pandas import DataFrame
from math import pi
from matplotlib import pyplot as plt


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


def calculate_angular_speed(count, time, count_per_turn):
    return (count / time) * (1 / count_per_turn) * 360.0


def calculate_wheel_speed(angular, wheel_diameter):
    circumference = pi*wheel_diameter
    return angular * (circumference / 360.0)


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


def receive_data(port, data_dict):
    key1 = ''
    key2 = ''
    receive = True
    while receive:
        while port.inWaiting() == 0:
            pass
        read_string = port.readline().decode('utf-8').strip()
        # print(read_string)  # debug
        if read_string == 'start':
            # print('String is: start')  # debug
            pass
        elif read_string == 'stop':
            # print('String is: stop')  # debug
            receive = False
        elif read_string == 'forward' or read_string == 'reverse':
            key1 = read_string
            # print('key1 is: ' + read_string)  # debug
        elif read_string == 'quarter' or read_string == 'half' or read_string == 'three-quarter':
            key2 = read_string
            # print('key2 is: ' + read_string)  # debug
        else:
            try:
                data = read_string.split(',')
                # print(key1 + ' ' + key2)  # debug
                data_dict[key1][key2]['right'].append(int(data[0]))
                data_dict[key1][key2]['left'].append(int(data[1]))
                data_dict[key1][key2]['time'].append(int(data[2]))
                data_dict[key1][key2]['volts'].append(float(data[3]))
                # print(data_dict)  # debug
            except TypeError:
                pass


if __name__ == '__main__':
    portName = 'COM4'
    sp = serial_connect(portName, 115200)
    # constants
    encoder_per_turn = 780.0
    wheel_diameter = 0.07
    # dict for data
    param_list = ['right', 'left', 'time', 'volts']
    speed_list = ['quarter', 'half', 'three-quarter']
    direct_list = ['forward', 'reverse']
    param = {param_list[0]: [], param_list[1]: [], param_list[2]: [], param_list[3]: []}
    speed = {speed_list[0]: deepcopy(param), speed_list[1]: deepcopy(param), speed_list[2]: deepcopy(param)}
    motor_test = {direct_list[0]: deepcopy(speed), direct_list[1]: deepcopy(speed)}
    # look for start signal
    look_for_start(sp)
    # receive data
    receive_data(sp, motor_test)
    send_ack(sp, 2)
    sp.close()
    print(motor_test)  # debug
    # table
    table_index = ['1/4 Speed Right', '1/4 Speed Left', '1/2 Speed Right', '1/2 Speed Left', '3/4 Speed Right', '3/4 Speed Left']
    table_column = ['Samples', 'Encoder Count', 'Time[ms]', 'Shaft Speed[deg/s]', 'Wheel Speed[m/s]', 'Battery Voltage[V]']
    forward_table = DataFrame(index=table_index, columns=table_column)
    reverse_table = DataFrame(index=table_index, columns=table_column)
    # sample count
    forward_table[table_column[0]][table_index[0]] = len(motor_test['forward']['quarter']['left'])
    forward_table[table_column[0]][table_index[1]] = len(motor_test['forward']['quarter']['right'])
    forward_table[table_column[0]][table_index[2]] = len(motor_test['forward']['half']['left'])
    forward_table[table_column[0]][table_index[3]] = len(motor_test['forward']['half']['right'])
    forward_table[table_column[0]][table_index[4]] = len(motor_test['forward']['three-quarter']['left'])
    forward_table[table_column[0]][table_index[5]] = len(motor_test['forward']['three-quarter']['right'])
    reverse_table[table_column[0]][table_index[0]] = len(motor_test['reverse']['quarter']['left'])
    reverse_table[table_column[0]][table_index[1]] = len(motor_test['reverse']['quarter']['right'])
    reverse_table[table_column[0]][table_index[2]] = len(motor_test['reverse']['half']['left'])
    reverse_table[table_column[0]][table_index[3]] = len(motor_test['reverse']['half']['right'])
    reverse_table[table_column[0]][table_index[4]] = len(motor_test['reverse']['three-quarter']['left'])
    reverse_table[table_column[0]][table_index[5]] = len(motor_test['reverse']['three-quarter']['right'])
    # encoder count
    forward_table[table_column[1]][table_index[0]] = sum(motor_test['forward']['quarter']['left'])
    forward_table[table_column[1]][table_index[1]] = sum(motor_test['forward']['quarter']['right'])
    forward_table[table_column[1]][table_index[2]] = sum(motor_test['forward']['half']['left'])
    forward_table[table_column[1]][table_index[3]] = sum(motor_test['forward']['half']['right'])
    forward_table[table_column[1]][table_index[4]] = sum(motor_test['forward']['three-quarter']['left'])
    forward_table[table_column[1]][table_index[5]] = sum(motor_test['forward']['three-quarter']['right'])
    reverse_table[table_column[1]][table_index[0]] = sum(motor_test['reverse']['quarter']['left'])
    reverse_table[table_column[1]][table_index[1]] = sum(motor_test['reverse']['quarter']['right'])
    reverse_table[table_column[1]][table_index[2]] = sum(motor_test['reverse']['half']['left'])
    reverse_table[table_column[1]][table_index[3]] = sum(motor_test['reverse']['half']['right'])
    reverse_table[table_column[1]][table_index[4]] = sum(motor_test['reverse']['three-quarter']['left'])
    reverse_table[table_column[1]][table_index[5]] = sum(motor_test['reverse']['three-quarter']['right'])
    # total time
    forward_table[table_column[2]][table_index[0]] = sum(motor_test['forward']['quarter']['time'])
    forward_table[table_column[2]][table_index[1]] = sum(motor_test['forward']['quarter']['time'])
    forward_table[table_column[2]][table_index[2]] = sum(motor_test['forward']['half']['time'])
    forward_table[table_column[2]][table_index[3]] = sum(motor_test['forward']['half']['time'])
    forward_table[table_column[2]][table_index[4]] = sum(motor_test['forward']['three-quarter']['time'])
    forward_table[table_column[2]][table_index[5]] = sum(motor_test['forward']['three-quarter']['time'])
    reverse_table[table_column[2]][table_index[0]] = sum(motor_test['reverse']['quarter']['time'])
    reverse_table[table_column[2]][table_index[1]] = sum(motor_test['reverse']['quarter']['time'])
    reverse_table[table_column[2]][table_index[2]] = sum(motor_test['reverse']['half']['time'])
    reverse_table[table_column[2]][table_index[3]] = sum(motor_test['reverse']['half']['time'])
    reverse_table[table_column[2]][table_index[4]] = sum(motor_test['reverse']['three-quarter']['time'])
    reverse_table[table_column[2]][table_index[5]] = sum(motor_test['reverse']['three-quarter']['time'])
    # shaft speed
    for i in range(len(table_index)):
        forward_table[table_column[3]][table_index[i]] = calculate_angular_speed(
            forward_table[table_column[1]][table_index[i]], (forward_table[table_column[2]][table_index[i]] / 1000.0),
            encoder_per_turn)
        reverse_table[table_column[3]][table_index[i]] = calculate_angular_speed(
            reverse_table[table_column[1]][table_index[i]], (reverse_table[table_column[2]][table_index[i]] / 1000.0),
            encoder_per_turn)
    # wheel speed
    for i in range(len(table_index)):
        forward_table[table_column[4]][table_index[i]] = calculate_wheel_speed(
            forward_table[table_column[3]][table_index[i]], wheel_diameter)
        reverse_table[table_column[4]][table_index[i]] = calculate_wheel_speed(
            reverse_table[table_column[3]][table_index[i]], wheel_diameter)
    # battery level
    for s in speed_list:
        for i in range(len(table_index)):
            forward_table[table_column[5]][table_index[i]] = sum(motor_test['forward'][s]['volts']) / len(
                motor_test['forward'][s]['volts'])
            reverse_table[table_column[5]][table_index[i]] = sum(motor_test['reverse'][s]['volts']) / len(
                motor_test['reverse'][s]['volts'])
    # write table
    with open('forwardTable.txt', 'w') as f:
        f.write(forward_table.to_string())
        f.close()
    with open('reverseTable.txt', 'w') as f:
        f.write(reverse_table.to_string())
        f.close()
