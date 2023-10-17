#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Float64
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import time
import csv
import os  # ディレクトリの作成のためのライブラリをインポート

class ErrorGraphYaw:
	def __init__(self):
		rospy.init_node('error_graph_yaw_plotter')
		rospy.on_shutdown(self.cleanup)

		# サブスクライバの定義
		rospy.Subscriber('/error_yaw', Float64, self.error_yaw_callback)

		# error値と時間を保存するリスト
		self.error_yaw_values = []
		self.timestamps = []

		# グラフの初期設定
		self.fig, self.ax = plt.subplots()
		self.line_yaw, = self.ax.plot([], [], 'r-', label="Yaw Error")
		self.ax.axhline(0, color='black',linewidth=0.5)  # 0の位置に直線を追加
		self.ax.legend()

		# 軸のラベルとタイトルの設定
		self.ax.set_xlabel('Time[s]')
		self.ax.set_ylabel('Error [pix]')
		self.ax.set_title('Error vs Time with PID:')
		self.start_time = time.time()

	def error_yaw_callback(self, msg):
		# error値をリストに追加
		self.error_yaw_values.append(msg.data)
		# 現在の時間を追加
		self.timestamps.append(time.time() - self.start_time)
		# グラフの更新
		self.update_graph()

	def update_graph(self):
		self.line_yaw.set_data(self.timestamps, self.error_yaw_values)
		self.ax.relim()
		self.ax.autoscale_view()

	def cleanup(self):
		self.save_graph()
		self.save_csv()

	def save_graph(self):
		date_folder = time.strftime("%Y%m%d")  # 今日の日付を取得
		image_folder_path = os.path.join('/home/wanglab/catkin_wsTrim/src/trim_apriltag/image', date_folder)
		if not os.path.exists(image_folder_path):
			os.makedirs(image_folder_path)  # 日付のフォルダがない場合、新しく作成

		self.P = rospy.get_param("/yaw_P")
		self.I = rospy.get_param("/yaw_I")
		self.D = rospy.get_param("/yaw_D")

		self.ax.set_title('Error vs Time with PID: P={}, I={}, D={}'.format(self.P, self.I, self.D))
		file_name = os.path.join(image_folder_path, 'error_yaw_graph_P{}_I{}_D{}.png'.format(self.P, self.I, self.D))
		self.fig.savefig(file_name)

	def save_csv(self):
		date_folder = time.strftime("%Y%m%d")
		data_folder_path = os.path.join('/home/wanglab/catkin_wsTrim/src/trim_apriltag/data', date_folder)
		if not os.path.exists(data_folder_path):
			os.makedirs(data_folder_path)  # 日付のフォルダがない場合、新しく作成

		file_name_csv = os.path.join(data_folder_path, 'error_yaw_P{}_I{}_D{}.csv'.format(self.P, self.I, self.D))
		with open(file_name_csv, 'w') as f:
			writer = csv.writer(f)
			writer.writerow(["Time", "Error"])  # データの名称
			for t, e in zip(self.timestamps, self.error_yaw_values):
				writer.writerow([t, e])

if __name__ == "__main__":
	egy = ErrorGraphYaw()
	rospy.spin()
