import matplotlib.pyplot as plt
import numpy as np
import matplotlib.animation as animation
import serial
import threading
import queue

ser = serial.Serial("COM5", 115200)

dataQue = queue.Queue()
queueLock = threading.Lock()
threads = []
all_data = {0: (0, 0, 0)}

DMAX = 1000
IMIN = 0
IMAX = 50


def clear(self):
    '''
    Clears all items from the queue.
    '''

    with self.mutex:
        unfinished = self.unfinished_tasks - len(self.queue)
        if unfinished <= 0:
            if unfinished < 0:
                raise ValueError('task_done() called too many times')
            self.all_tasks_done.notify_all()
        self.unfinished_tasks = unfinished
        self.queue.clear()
        self.not_full.notify_all()

class RPLidar(threading.Thread):
    def __init__(self, threadID, name):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name

    def run(self):
        while True:
            data = ser.readline().rstrip()

            # Convert data to list of floats
            try:
                data = data.decode()
                data = tuple(map(float, data.split(',')))

                if queueLock.acquire(timeout=0.1):
                    # all_data[round(data[0])] = data[1:]
                    dataQue.put(data)
                    queueLock.release()
                else:
                    print("failed to acquire lock for collection")
                    continue

            except (ValueError, UnicodeDecodeError):
                print("Exception Raised!")
                continue

            # print("Putting data in queue")
            # queueLock.acquire()
            # # try:
            # #     dataQue.get()
            # # except queue.Empty:
            # #     pass
            # try:
            #     dataQue.put(all_data)
            # except:
            #     print("Exception raised!")
            #     pass
            # finally:
            #     queueLock.release()
            #     print("Operation successful")


class Plotter:
    def __init__(self, threadID, name):
        self.threadID = threadID
        self.name = name

    def update_line(self, num, iterator, line):
        global all_data
        while True:
            if queueLock.acquire(timeout=1):
                for i in range(dataQue.qsize()):
                    new_data = dataQue.get()

                    all_data[round(new_data[0])] = new_data[1:]
                # scan = all_data.copy()
                # Clear queue
                clear(dataQue)
                queueLock.release()

            else:
                print("failed to acquire lock for plot")
                continue

            # Flatten data into array
            scan = [tuple([key] + list(value)) for key, value in all_data.items()]

            try:
                offsets = np.array([(np.radians(meas[0]), meas[1]) for meas in scan])
                line.set_offsets(offsets)
                intens = np.array([meas[3] for meas in scan])
                line.set_array(intens)
                return line,
            except IndexError:
                pass

            del scan

    def run(self):
        fig = plt.figure()
        ax = plt.subplot(111, projection='polar')
        line = ax.scatter([0, 0], [0, 0], s=5, c=[IMIN, IMAX],
                               cmap=plt.cm.Greys_r, lw=0)
        ax.set_rmax(DMAX)
        ax.grid(True)

        iterable = []
        ani = animation.FuncAnimation(fig, self.update_line,
            fargs=(iterable, line), interval=50 , blit=True)
        plt.show()


if __name__ == '__main__':
    # Create new threads
    Lidar = RPLidar(1, "Lidar")
    MatLab = Plotter(2, "MatLab")

    # Add threads to thread list
    threads.append(Lidar)
    threads.append(MatLab)

    # Start new Threads
    Lidar.start()
    MatLab.run()

    print("Threads started")
    