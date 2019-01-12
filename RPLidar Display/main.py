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

        self.all_data = {}

    def run(self):
        # Setup
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='polar')
        self.line, = self.ax.plot([], [])
        self.ax.set_ylim(0, 2000)
        self.ax.set_rticks([500, 1000, 1500, 2000])
        self.fig.patch.set_facecolor('k')
        self.ax.set_facecolor('k')
        self.ax.tick_params(labelcolor='w')

        #Autoscale on unknown axis and known lims on the other
        # self.ax.set_autoscaley_on(True)

        while True:
            # Read new values from que into all_data
            if not dataQue.empty():
                queueLock.acquire()
                for i in range(dataQue.qsize()):
                    new_data = dataQue.get()
                    # convert degrees to radians
                    angle = new_data[0] * (np.pi / 180)
                    self.all_data[angle] = new_data[1]

                queueLock.release()

                rdata = []
                tdata = []

                for key, value in self.all_data.items():
                    rdata.append(value)
                    tdata.append(key)

                # Update data (with the new _and_ the old points)
                self.ax.scatter(tdata, rdata, s=1)
                # Need both of these in order to rescale
                self.ax.relim()
                self.ax.autoscale_view()
                # We need to draw *and* flush
                self.fig.canvas.draw()
                self.fig.canvas.flush_events()


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

