import serial
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import threading
import queue



ser = serial.Serial("COM5", 115200)
# prev_angle = -1
# full_view = {}

dataQue = queue.Queue(360)
queueLock = threading.Lock()
threads = []

class LidarData(threading.Thread):
    def __init__(self, threadID, name, counter):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        self.counter = counter


    def run(self):
        print("Starting:", self.name)

        data = ser.readline().rstrip()

        # Convert data to list of floats
        try:
            data = data.decode()
        except UnicodeDecodeError:
            return

        try:
            data = list(map(float, data.split(',')))
        except ValueError:
            return

        # Collect data into a full map of 360 degrees
        angle = round(data[0])
        dist = data[1]

        if angle <= 360:
            dataQue.put((angle, dist))


class Plotter(threading.Thread):
    def __init__(self, threadID, name, counter):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        self.counter = counter

        self.all_data = {}

    def run(self):
        # Read new values from que into all_data
        queueLock.acquire()
        for i in range(dataQue.qsize()):
            new_data = dataQue.get()
            self.all_data[new_data[0]] = new_data[1]
        queueLock.release()

        print(self.all_data)


# Create threads
# Create new threads
Lidar = LidarData(1, "Lidar", 1)
MatLab = Plotter(2, "MatLab", 2)

# Start new Threads
Lidar.start()
MatLab.start()

# Add threads to thread list
threads.append(Lidar)
threads.append(MatLab)

# Wait for all threads to complete
for t in threads:
   t.join()
print ("Exiting Main Thread")


































#
#
#
# # Matlab Boilerplate
# # fig = plt.figure()
# # ax = fig.subplot(111)
#
#
#
# def update(data):
#     # Convert dict to xs and ys
#     print(data)
#     # line.set_xdata()
#     # angs = []
#     # rs = []
#     # for key, value in data.items():
#     #     angs.append(key)
#     #     rs.append(value)
#
#     # Plot the data
#     # theta = list(full_view.keys())
#     # r = list(full_view.values())
#     #
#     # ax.plot(theta, r, 'ro')
#     #
#     # plt.show()
#
#     # ani = animation.FuncAnimation(fig, update, frames=[(key, value) for key, value in full_view.items()])
#
# # full_view = {}
# # fig = plt.figure()
# # ax = fig.add_subplot(1,1,1)
# #
# #
#
# while True:
#     data = ser.readline().rstrip()
#
#     # Convert data to list of floats
#     try:
#         data = data.decode()
#     except UnicodeDecodeError:
#         continue
#
#     try:
#         data = list(map(float, data.split(',')))
#     except ValueError:
#         continue
#
#     # Collect data into a full map of 360 degrees
#     angle = data[0]
#     dist = data[1]
#
#
#     if angle <= 360:
#         full_view[angle] = dist
#
#     print(full_view)
# #     theta = list(full_view.keys())
# #     r = list(full_view.values())
# #
# #     ax.clear()
# #     ax.plot(theta, r, 'ro')
# #
# #     print(theta, ':', r)
# #
# # while True:
# #     a = animation.FuncAnimation(fig, update, repeat=False)
#     plt.show()