import pandas as pd

import biasOffsetCalibration as b


def state_two(port, data_frame, md, signal, i, c, file_name, scaled):
    next_state = False
    # iteration count to write to table file
    index = 1
    while not next_state:
        # receive data
        if port.inWaiting() > 11:
            data = b.receive_calibration_data(port)
            # print(data)  # debug
            if scaled:
                # scale data back down and convert to float
                scaled_data = tuple(scale_data(data[0:-1]))
                md.append(scaled_data)
                store_data(data_frame, scaled_data, i, c)
                b.write_table_to_file(data_frame, index, file_name)
            else:
                md.append(data[0:6])
                store_data(data_frame, data, i, c)
                b.write_table_to_file(data_frame, index, file_name)
            # check for stop signal, 6th element in data
            next_state = b.look_for_stop(data[6], port, signal)
            print('Iteration ' + str(index))
            print(data_frame)
            print()
            index += 1


"""
    scale_data(data)
        generator yields scaled version of input elements
"""


def scale_data(data):
    for element in data:
        element = float(element) / 10000.0
        yield element


def store_data(data_frame, data, index, cols):
    data_frame[cols[0]][index[0]] = data[0]
    data_frame[cols[1]][index[0]] = data[1]
    data_frame[cols[2]][index[0]] = data[2]
    data_frame[cols[3]][index[0]] = data[3]
    data_frame[cols[4]][index[0]] = data[4]
    data_frame[cols[5]][index[0]] = data[5]


if __name__ == "__main__":
    portName = "COM4"
    file_s = 'measuredErrorScaled.txt'
    file_r = 'measuredErrorRaw.txt'
    title = 'Measured Error After Calibration'
    stop = 0
    start = 0xFF
    baud_rate = 115200

    data_list = []
    ser = b.initialize_serial(portName, baud_rate)
    columns_list = ['AX', 'AY', 'AZ', 'GX', 'GY', 'GZ']
    index_list = ['Measurement']
    error_data = pd.DataFrame(index=index_list, columns=columns_list)

    b.state_one(ser, file_s, title, start)
    state_two(ser, error_data, data_list, stop, index_list, columns_list, file_s, scaled=True)



