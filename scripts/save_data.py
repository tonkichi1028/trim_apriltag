#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from apriltag_ros.msg import AprilTagDetectionArray, AprilTagDetectionPositionArray
import csv
import os
from datetime import datetime

class TopicSubscriber(object):
    def __init__(self):
        self.subscriber_detections = rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self.callback_detections)
        self.subscriber_position = rospy.Subscriber('/tag_position', AprilTagDetectionPositionArray, self.callback_position)
        self.data_to_save_detections = []
        self.data_to_save_positions = []
        self.prev_time_detections = rospy.get_time()
        self.prev_time_position = rospy.get_time()

    def callback_detections(self, msg):
        curr_time = rospy.get_time()
        hz_detections = 1.0 / (curr_time - self.prev_time_detections)
        self.prev_time_detections = curr_time
        x = msg.detections[0].pose.pose.pose.position.x
        y = msg.detections[0].pose.pose.pose.position.y
        z = msg.detections[0].pose.pose.pose.position.z
        self.data_to_save_detections.append([curr_time, hz_detections, x, y, z])
        rospy.loginfo("Data saved: x=%f, y=%f, z=%f, Frequency: %f Hz", x, y, z, hz_detections)

    def callback_position(self, msg):
        curr_time = rospy.get_time()
        hz_position = 1.0 / (curr_time - self.prev_time_position)
        self.prev_time_position = curr_time
        u = msg.detect_positions[0].x
        v = msg.detect_positions[0].y
        self.data_to_save_positions.append([curr_time, hz_position, u, v])
        rospy.loginfo("Data saved: u=%f, v=%f, Frequency: %f Hz", u, v, hz_position)

    def save_data(self):
        data_folder = "/home/wanglab/catkin_wsTrim/src/trim_apriltag/data/"
        date_folder = data_folder + datetime.now().strftime("%Y-%m-%d")

        if not os.path.exists(date_folder):
            os.makedirs(date_folder)

        existing_folders = [f for f in os.listdir(date_folder) if os.path.isdir(os.path.join(date_folder, f))]
        experiment_folder = os.path.join(date_folder, 'experiment_{}'.format(len(existing_folders) + 1))

        if not os.path.exists(experiment_folder):
            os.makedirs(experiment_folder)

        filename_detections = os.path.join(experiment_folder, "saved_data_detections.csv")
        with open(filename_detections, "w") as f:
            writer = csv.writer(f)
            writer.writerow(['timestamp', 'hz_detections', 'x', 'y', 'z'])
            writer.writerows(self.data_to_save_detections)

        filename_positions = os.path.join(experiment_folder, "saved_data_positions.csv")
        with open(filename_positions, "w") as f:
            writer = csv.writer(f)
            writer.writerow(['timestamp', 'hz_positions', 'u', 'v'])
            writer.writerows(self.data_to_save_positions)

if __name__ == "__main__":
    rospy.init_node('topic_subscriber_node')
    ts = TopicSubscriber()
    rospy.on_shutdown(ts.save_data)
    rospy.spin()
