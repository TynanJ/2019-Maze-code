import serial
import matplotlib.pyplot as plt
import threading
import queue
import time

all_data = {}
read_times = []
full_map_times = []
ser = serial.Serial("COM4", 115200)
print("Starting scan")

start_time_map = time.time()
while True:
    start_time_read = time.time()
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

    read_times.append(time.time() - start_time_read)

    # Collect data into a full map of 360 degrees
    try:
        angle = round(data[0])
        dist = data[1]
    except IndexError:
        continue

    if angle <= 360:
        all_data[angle] = dist

    if len(all_data.keys()) == 360:
        full_map_times.append(time.time() - start_time_map)
        print("Full scan complete")
        print("Average Time to complete: {}".format(sum(full_map_times) / len(full_map_times)))
        print("Average time per scan: {}".format(sum(read_times) / len(read_times)))
        print('\n=================================================\n')


        all_data = {}
        start_time_map = time.time()

