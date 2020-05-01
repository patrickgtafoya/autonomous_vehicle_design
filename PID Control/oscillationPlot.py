import matplotlib.pyplot as plt

fileName = 'angleData.txt'
allData = []
compData = []
accelData = []
gyroData = []

with open(fileName, 'r') as f:
    data = str(f.readlines()[1:-1]).strip('[').strip(']').split('\\n')
    # print(data)

    for index in range(1, len(data)):
        if index % 2 == 0:
            allData.append(data[index][4:].split(','))
    for sublist in allData:
        for index in range(len(sublist)):
            if index % 3 == 0:
                compData.append(float(sublist[index]))
            elif index % 3 == 1:
                accelData.append(float(sublist[index]))
            elif index % 3 == 2:
                gyroData.append(float(sublist[index]))
t = []
for index in range(len(compData)):
    t.append(index)

plt.subplot(311)
plt.plot(t, compData, 'r', label='Filtered')
plt.title('Oscillation Over Time')
plt.legend()
plt.subplot(312)
plt.plot(t, accelData, 'g', label='Acceleration')
plt.ylabel('Orientation [degrees]')
plt.legend()
plt.subplot(313)
plt.plot(t, gyroData, 'b', label='Gyro')
plt.xlabel('Sample [4ms]')
plt.legend()
plt.savefig('oscillationVsTime.png')
plt.show()

