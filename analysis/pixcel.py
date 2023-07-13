#!/usr/bin/env python
# -*- coding: utf-8 -*-
import csv
import matplotlib.pyplot as plt
import math
import numpy as np

# ファイル名
filename = "/home/wanglab/catkin_wsTrim/src/trim_apriltag/data/2023-07-08/bpm60/saved_data.csv"

# 初期化
times = []
x_positions = []
y_positions = []

# データの読み込み
with open(filename, 'r') as f:
    reader = csv.reader(f)
    for row in reader:
       x_positions.append(float(row[0]))
       y_positions.append(float(row[1]))
       times.append(float(row[2]))

# Velocity and acceleration calculation
x_velocities = np.diff(x_positions) / np.diff(times)
y_velocities = np.diff(y_positions) / np.diff(times)

x_accelerations = np.diff(x_velocities) / np.diff(times[:-1])  # We lose one element due to diff
y_accelerations = np.diff(y_velocities) / np.diff(times[:-1])  # We lose one element due to diff

# Moving average function
def moving_average(data, window_size):
    return [np.mean(data[i-window_size+1:i+1]) for i in range(window_size-1, len(data))]

# Smoothing
window_size = 5  # Choose a suitable window size for your data
x_velocities_smooth = moving_average(x_velocities, window_size)
y_velocities_smooth = moving_average(y_velocities, window_size)
x_accelerations_smooth = moving_average(x_accelerations, window_size)
y_accelerations_smooth = moving_average(y_accelerations, window_size)

# データのプロット
plt.figure(figsize=(12, 8))

plt.subplot(3, 1, 1)
plt.plot(times, x_positions, marker='o', linestyle='-', color='b')
plt.title('u position')
plt.xlabel('Time [s]')
plt.ylabel('u position [pix]')

plt.subplot(3, 1, 2)
plt.plot(times[window_size-1:-1], x_velocities_smooth, marker='o', linestyle='-', color='b')
plt.title('u velocity')
plt.xlabel('Time [s]')
plt.ylabel('u velocity [pix/s]')

plt.subplot(3, 1, 3)
plt.plot(times[window_size-1:-2], x_accelerations_smooth, marker='o', linestyle='-', color='b')
plt.title('u acceleration')
plt.xlabel('Time [s]')
plt.ylabel('u acceleration [pix/s^2]')

plt.tight_layout()

plt.figure(figsize=(12, 8))

plt.subplot(3, 1, 1)
plt.plot(times, y_positions, marker='o', linestyle='-', color='r')
plt.title('v position')
plt.xlabel('Time [s]')
plt.ylabel('v position [pix]')

plt.subplot(3, 1, 2)
plt.plot(times[window_size-1:-1], y_velocities_smooth, marker='o', linestyle='-', color='r')
plt.title('v velocity')
plt.xlabel('Time [s]')
plt.ylabel('v velocity [pix/s]')

plt.subplot(3, 1, 3)
plt.plot(times[window_size-1:-2], y_accelerations_smooth, marker='o', linestyle='-', color='r')
plt.title('v acceleration')
plt.xlabel('Time [s]')
plt.ylabel('v acceleration [pix/s^2]')

plt.tight_layout()
plt.show()

