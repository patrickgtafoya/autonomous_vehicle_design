import serial
from time import sleep
from matplotlib import pyplot as pyp
import pandas as pd

"""
    state_one()
        return: void
        param:  serial port object
                name of file to write to
            
            waits for start signal from nano
            sends acknowledgment upon receiving
            creates file and writes first line
"""


def state_one(port, file_name, file_title, signal):
    next_state = False
    # wait for start signal
    while not next_state:
        next_state = look_for_start(signal, port)
    send_ack(port)
    # create file to write tables
    with open(file_name, 'w') as f:
        f.write(file_title + '\n\n')


"""
    state_two()
        return: void
        param:  serial port object, pandas data frame
            
            reads data through serial port into a pandas data frame
            writes data frame to table in .txt file
            continues until stop signal received
            creates plots of data
"""


def state_two(port, data_frame, md, od, signal, file_name):
    next_state = False
    # index for mean or offset received. Mean is odd, offset is even
    index = 1
    # iteration count to write to table file
    iterate = 1
    while not next_state:
        # receive data
        if port.inWaiting() > 11:
            data = receive_calibration_data(port)
            # check for stop signal, 6th element in data
            next_state = look_for_stop(data[6], port, signal)
            if index % 2:
                # print('Iteration ' + str(iterate))
                md[iterate] = data[0:6]
                store_mean_data(data_frame, data)
                index += 1
            else:
                print('Iteration ' + str(iterate))
                store_offset_data(data_frame, data)
                print(data_frame)
                od.append(data[0:6])
                write_table_to_file(data_frame, iterate, file_name)
                index += 1
                iterate += 1


"""
    initialize_serial()
        return: serial port object
        param:  name of serial port for connection
        
            attempts connection to specified serial port at given baud rate
            returns serial port object upon success
            flags error if unable to connect
"""


def initialize_serial(port_name, baud):
    # initialize Serial Connection
    try:
        port = serial.Serial(port_name, baudrate=baud)
        print("Connected to port: " + port_name)  # debug
    # raise exception if unable to connect
    except serial.SerialException:
        raise IOError("Unable to connect to port: " + port_name)  # debug
    # ensure port is open
    sleep(500 / 1000)
    # flush input buffer
    port.reset_input_buffer()
    return port


"""
    receive_calibration_data()
        return: tuple, size 7, of data read from serial port
        param:  serial port object
        
            reads 12 bytes of data from serial port
            data sent big endian signed
            acceleration xyz, then gyro xyz all 2 bytes in size
            stores in tuple in same order as received
            unsigned full 12 bytes stored in last position of tuple to check for stop signal
"""


def receive_calibration_data(port):
    # looking for 12 bytes of data
    read_bytes = port.read(12)
    ax = int.from_bytes(read_bytes[0:2], byteorder='big', signed=True)
    ay = int.from_bytes(read_bytes[2:4], byteorder='big', signed=True)
    az = int.from_bytes(read_bytes[4:6], byteorder='big', signed=True)
    gx = int.from_bytes(read_bytes[6:8], byteorder='big', signed=True)
    gy = int.from_bytes(read_bytes[8:10], byteorder='big', signed=True)
    gz = int.from_bytes(read_bytes[10:], byteorder='big', signed=True)
    stop_check = int.from_bytes(read_bytes, byteorder='big', signed=False)
    return ax, ay, az, gx, gy, gz, stop_check


"""
    look_for_start()
        return: bool true of false
        param:  start signal value, serial port object
        
            reads serial port when data available
            if data matches given start signal value, returns true
            if not, returns false
"""


def look_for_start(signal, port):
    # signal will be two bytes, big endian
    if port.inWaiting():
        read_byte = port.read()
        sig = int.from_bytes(read_byte, byteorder='big', signed=False)
        # compare to start signal
        if sig == signal:
            print("Start Received")  # debug
            return True
        else:
            return False

"""
    send_ack()
        return: void
        param:  serial port object
        
            sends acknowledgement to nano over serial port
"""


def send_ack(port):
    # send ACK
    try:
        port.write(1)
        # print("ACK sent")  # debug
    except serial.SerialException:
        raise IOError("ACK not sent")


"""
    store_mean_data()
        return: void
        param:  pandas data frame, data to write
        
            takes data and writes to data frame with given mean value
            indices and columns
"""


def store_mean_data(data_frame, data):
    data_frame['X']['Acceleration Mean'] = data[0]
    data_frame['Y']['Acceleration Mean'] = data[1]
    data_frame['Z']['Acceleration Mean'] = data[2]
    data_frame['X']['Gyro Mean'] = data[3]
    data_frame['Y']['Gyro Mean'] = data[4]
    data_frame['Z']['Gyro Mean'] = data[5]


"""
    store_offset_data()
        return: void
        param:  pandas data frame, data to write

            takes data and writes to data frame with given offset value
            indices and columns
"""


def store_offset_data(data_frame, data):
    data_frame['X']['Acceleration Offset'] = data[0]
    data_frame['Y']['Acceleration Offset'] = data[1]
    data_frame['Z']['Acceleration Offset'] = data[2]
    data_frame['X']['Gyro Offset'] = data[3]
    data_frame['Y']['Gyro Offset'] = data[4]
    data_frame['Z']['Gyro Offset'] = data[5]


"""
    write_table_to_file()
        return: void
        param:  data to write, index of data
        
            appends each new data reading to offsetCalibration.txt
"""


def write_table_to_file(data, index, file_name):
    with open(file_name, 'a') as f:
        f.write('Iteration ' + str(index) + '\n' + str(data) + '\n\n')


def plot_calibration_data(calibration_dict):
    # empty lists to store values
    key_list = []
    acceleration_mean_x = []
    acceleration_mean_y = []
    acceleration_mean_z = []
    gyro_mean_x = []
    gyro_mean_y = []
    gyro_mean_z = []
    # assign values
    for key in calibration_dict:
        key_list.append(key)
        acceleration_mean_x.append(calibration_dict[key][0])
        acceleration_mean_y.append(calibration_dict[key][1])
        acceleration_mean_z.append(calibration_dict[key][2])
        gyro_mean_x.append(calibration_dict[key][3])
        gyro_mean_y.append(calibration_dict[key][4])
        gyro_mean_z.append(calibration_dict[key][5])
    # plot data
    pyp.subplot(2, 1, 1)
    pyp.plot(key_list, acceleration_mean_x, color='b', label='x')
    pyp.plot(key_list, acceleration_mean_y, color='g', label='y')
    pyp.plot(key_list, acceleration_mean_z, color='r', label='z')
    pyp.legend()
    pyp.title('Acceleration Mean Value')

    pyp.subplot(2, 1, 2)
    pyp.plot(key_list, gyro_mean_x, color='b', label='x')
    pyp.plot(key_list, gyro_mean_y, color='g', label='y')
    pyp.plot(key_list, gyro_mean_z, color='r', label='z')
    pyp.tight_layout()
    pyp.legend()
    pyp.title('Gyroscope Mean Value')
    # save and show plots
    pyp.savefig('offsetCalibration.png')
    pyp.show()


if __name__ == "__main__":
    portName = "COM4"
    file = 'offsetCalibration.txt'
    title = 'Offset Calibration Data'
    stop = 0
    start = 0xFF
    baud_rate = 115200

    ser = initialize_serial(portName, baud_rate)
    index_list = ['Acceleration Mean', 'Gyro Mean', 'Acceleration Offset', 'Gyro Offset']
    columns_list = ['X', 'Y', 'Z']
    calibration_data = pd.DataFrame(index=index_list, columns=columns_list)
    mean_dict = {}
    offset_list = []
    state_one(ser, file, title, start)
    state_two(ser, calibration_data, mean_dict, offset_list, stop, file)
    final_offsets = offset_list[-2]
    # print(final_offsets)  # debug
    with open('offsetCalibration.txt', 'a') as f:
        f.write('\nFinal Offsets:\n')
        f.write('AX: ' + str(final_offsets[0]) + '\n')
        f.write('AY: ' + str(final_offsets[1]) + '\n')
        f.write('AZ: ' + str(final_offsets[2]) + '\n')
        f.write('GX: ' + str(final_offsets[3]) + '\n')
        f.write('GY: ' + str(final_offsets[4]) + '\n')
        f.write('GZ: ' + str(final_offsets[5]) + '\n')
    # print(mean_dict)  # debug
    plot_calibration_data(mean_dict)


