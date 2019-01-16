import serial
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import threading
import queue


plt.ion()
ser = serial.Serial("COM5", 115200)
# prev_angle = -1
# full_view = {}

dataQue = queue.Queue(360)
queueLock = threading.Lock()
threads = []

class LidarData(threading.Thread):
    def __init__(self, threadID, name):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name


    def run(self):
        print("Starting:", self.name)
        while True:
            data = ser.readline().rstrip()

            # Convert data to list of floats
            try:
                data = data.decode()
            except UnicodeDecodeError:
                continue

            try:
                data = list(map(float, data.split(',')))
            except ValueError:
                continue

            # Collect data into a full map of 360 degrees
            try:
                # if data[3] < 10:
                #     continue

                angle = round(data[0])
                dist = data[1]

            except IndexError:
                continue

            if angle <= 360:
                queueLock.acquire()
                dataQue.put((angle, dist))
                queueLock.release()


class Plotter(threading.Thread):
    def __init__(self, threadID, name):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        self.min_x = 0
        self.max_x = 360
        self.IMIN = 0
        self.IMAX = 50

        self.all_data = {}

    def update_line(self, line):
        # Read fresh data
        queueLock.acquire()
        for i in range(dataQue.qsize()):
            new_data = dataQue.get()
            # convert degrees to radians
            self.all_data[new_data[0]] = new_data[1]

        queueLock.release()

        # Flatten all_data
        scan = [tuple([key] + list(value)) for key, value in self.all_data.items()]
        print(scan)

        offsets = np.array([(np.radians(meas[0]), meas[1]) for meas in scan])
        line.set_offsets(offsets)
        intens = np.array([meas[2] for meas in scan])
        line.set_array(intens)
        return line,


    def run(self):
        fig = plt.figure()
        ax = plt.subplot(111, projection='polar')
        line = ax.scatter([0, 0], [0, 0], s=5, c=[self.IMIN, self.IMAX],
                          cmap=plt.cm.Greys_r, lw=0)
        ax.set_rmax(4000)
        ax.grid(True)

        while True:
            # Update line
            self.update_line(line)
        plt.show()
        print("Stopping Lidar")


# Create threads
# Create new threads
Lidar = LidarData(1, "Lidar")
MatLab = Plotter(2, "MatLab")

# Add threads to thread list
threads.append(Lidar)
threads.append(MatLab)

# Start new Threads
Lidar.start()
MatLab.start()

