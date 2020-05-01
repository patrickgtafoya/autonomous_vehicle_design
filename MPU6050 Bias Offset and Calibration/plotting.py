from matplotlib import pyplot as pyp


def get_calculations(data):
    from statistics import stdev
    d_max = max(data)
    d_min = min(data)
    d_mean = sum(data) / len(data)
    d_dev = stdev(data)
    return d_max, d_min, d_mean, d_dev


def store_calc(data, file_name, title):
    with open(file_name, 'a') as f:
        f.write('\n' + title + '\n')
        f.write('Max: ' + str(data[0]) + '\n')
        f.write('Min: ' + str(data[1]) + '\n')
        f.write('Mean: ' + str(data[2]) + '\n')
        f.write('Standard Deviation: ' + str(data[3]) + '\n')
        f.close()


file_r = 'measuredErrorRaw.txt'
file_s = 'measuredErrorScaled.txt'
write_file_r = 'errorCalculationsRaw.txt'
write_file_s = 'errorCalculationsScaled.txt'

scaled = True

if scaled:
    file = file_s
    write_file = write_file_s
    figure_title = ['calibratedAcceleration.png', 'calibratedGyro.png']
    axis_unit = ['[g]', '[deg/sec]']
else:
    file = file_r
    write_file = write_file_r
    figure_title = ['calibratedRawAcceleration.png', 'calibratedRawGyro.png']
    axis_unit = ['[LSB]', '[LSB]']

data_list = []
with open(file, 'r') as f:
    d = f.readlines()
    d = (str(d[2:]).replace('[', '').replace(']', '').split('\\n'))
    for element in d:
        data_list.append(element.strip(",' "))
    # print(data_list)  # debug

# trim any non-measurement lines
# measurements are line 3 of 4 for every iteration
data_list = data_list[2::4]
data_list = data_list[:-1]

# convert measurements fo type float
for i in range(len(data_list)):
    # delete label
    data_list[i] = str(data_list[i].replace('Measurement', '')).split()
    # convert elements
    for j in range(len(data_list[i])):
        if scaled:
            data_list[i][j] = float(data_list[i][j])
        else:
            # print(data_list[i][j])
            data_list[i][j] = int(data_list[i][j])

# lists for plotting
ax = []
ay = []
az = []
gx = []
gy = []
gz = []
# assign data accordingly
for element in data_list:
    ax.append(element[0])
    ay.append(element[1])
    az.append(element[2])
    gx.append(element[3])
    gy.append(element[4])
    gz.append(element[5])

# plot acceleration
pyp.subplot(3, 1, 1)
pyp.plot(ax, color='b', label='x')
pyp.title('X Acceleration')
pyp.xlabel('[Iteration]')
pyp.ylabel(axis_unit[0])
pyp.subplot(3, 1, 2)
pyp.plot(ay, color='g', label='y')
pyp.title('Y Acceleration')
pyp.xlabel('[Iteration]')
pyp.ylabel(axis_unit[0])
pyp.subplot(3, 1, 3)
pyp.plot(az, color='r', label='z')
pyp.title('Z Acceleration')
pyp.xlabel('[Iteration]')
pyp.ylabel(axis_unit[0])
pyp.tight_layout()
pyp.savefig(figure_title[0])
pyp.show()

# plot gyro
pyp.subplot(3, 1, 1)
pyp.plot(gx, color='b', label='x')
pyp.title('X Gyroscope')
pyp.xlabel('[Iteration]')
pyp.ylabel(axis_unit[1])
pyp.subplot(3, 1, 2)
pyp.plot(gy, color='g', label='y')
pyp.title('Y Gyroscope')
pyp.xlabel('[Iteration]')
pyp.ylabel(axis_unit[1])
pyp.subplot(3, 1, 3)
pyp.plot(gz, color='r', label='z')
pyp.title('Z Gyroscope')
pyp.xlabel('[Iteration]')
pyp.ylabel(axis_unit[1])
pyp.tight_layout()
pyp.savefig(figure_title[1])
pyp.show()

# calculations
ax_calc = get_calculations(ax)
ay_calc = get_calculations(ay)
az_calc = get_calculations(az)
gx_calc = get_calculations(gx)
gy_calc = get_calculations(gy)
gz_calc = get_calculations(gz)


store_calc(ax_calc, write_file, 'AX Calculations')
store_calc(ay_calc, write_file, 'AY Calculations')
store_calc(az_calc, write_file, 'AZ Calculations')
store_calc(gx_calc, write_file, 'GX Calculations')
store_calc(gy_calc, write_file, 'GY Calculations')
store_calc(gz_calc, write_file, 'GZ Calculations')



