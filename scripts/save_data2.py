#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from apriltag_ros.msg import AprilTagDetectionArray, AprilTagDetectionPositionArray
from sensor_msgs.msg import CameraInfo
import csv
import os
from datetime import datetime

class TopicSubscriber(object):
    def __init__(self):
        self.subscriber_detections1 = rospy.Subscriber('/tag_detections1', AprilTagDetectionArray, self.callback_detections1)
        self.subscriber_detections2 = rospy.Subscriber('/tag_detections2', AprilTagDetectionArray, self.callback_detections2)
        self.subscriber_position1 = rospy.Subscriber('/tag_position1', AprilTagDetectionPositionArray, self.callback_position1)
        self.subscriber_position2 = rospy.Subscriber('/tag_position2', AprilTagDetectionPositionArray, self.callback_position2)
        self.subscriber_camera_info1 = rospy.Subscriber('/masking_info1', CameraInfo, self.callback_camera_info1)
        self.subscriber_camera_info2 = rospy.Subscriber('/masking_info2', CameraInfo, self.callback_camera_info2)

        self.data_to_save_detections1 = []
        self.data_to_save_detections2 = []
        self.data_to_save_positions1 = []
        self.data_to_save_positions2 = []
        self.data_to_save_camera_info1 = []
        self.data_to_save_camera_info2 = []
        self.prev_time_detections1 = rospy.get_time()
        self.prev_time_detections2 = rospy.get_time()
        self.prev_time_position1 = rospy.get_time()
        self.prev_time_position2 = rospy.get_time()
        self.prev_time_camera_info1 = rospy.get_time()
        self.prev_time_camera_info2 = rospy.get_time()

    def callback_camera_info1(self, msg):
        curr_time = rospy.get_time()
        hz_camera_info = 1.0 / (curr_time - self.prev_time_camera_info1)
        self.prev_time_camera_info1 = curr_time
        x_offset = msg.roi.x_offset
        y_offset = msg.roi.y_offset
        self.data_to_save_camera_info1.append([curr_time, hz_camera_info, x_offset, y_offset])

    def callback_camera_info2(self, msg):
        curr_time = rospy.get_time()
        hz_camera_info = 1.0 / (curr_time - self.prev_time_camera_info2)
        self.prev_time_camera_info2 = curr_time
        x_offset = msg.roi.x_offset
        y_offset = msg.roi.y_offset
        self.data_to_save_camera_info2.append([curr_time, hz_camera_info, x_offset, y_offset])

    def callback_detections1(self, msg):
        curr_time = rospy.get_time()
        hz_detections = 1.0 / (curr_time - self.prev_time_detections1)
        self.prev_time_detections1 = curr_time
        x = msg.detections[0].pose.pose.pose.position.x
        y = msg.detections[0].pose.pose.pose.position.y
        z = msg.detections[0].pose.pose.pose.position.z
        self.data_to_save_detections1.append([curr_time, hz_detections, x, y, z])

    def callback_detections2(self, msg):
        curr_time = rospy.get_time()
        hz_detections = 1.0 / (curr_time - self.prev_time_detections2)
        self.prev_time_detections2 = curr_time
        x = msg.detections[0].pose.pose.pose.position.x
        y = msg.detections[0].pose.pose.pose.position.y
        z = msg.detections[0].pose.pose.pose.position.z
        self.data_to_save_detections2.append([curr_time, hz_detections, x, y, z])

    def callback_position1(self, msg):
        curr_time = rospy.get_time()
        hz_position = 1.0 / (curr_time - self.prev_time_position1)
        self.prev_time_position1 = curr_time
        u = msg.detect_positions[0].x
        v = msg.detect_positions[0].y
        self.data_to_save_positions1.append([curr_time, hz_position, u, v])

    def callback_position2(self, msg):
        curr_time = rospy.get_time()
        hz_position = 1.0 / (curr_time - self.prev_time_position2)
        self.prev_time_position2 = curr_time
        u = msg.detect_positions[0].x
        v = msg.detect_positions[0].y
        self.data_to_save_positions2.append([curr_time, hz_position, u, v])

    def save_data(self):
        data_folder = "/home/wanglab/catkin_wsTrim/src/trim_apriltag/data/"
        date_folder = data_folder + datetime.now().strftime("%Y-%m-%d")

        if not os.path.exists(date_folder):
            os.makedirs(date_folder)

        existing_folders = [f for f in os.listdir(date_folder) if os.path.isdir(os.path.join(date_folder, f))]
        experiment_folder = os.path.join(date_folder, 'experiment_{}'.format(len(existing_folders) + 1))

        if not os.path.exists(experiment_folder):
            os.makedirs(experiment_folder)

        self.save_data_for_camera("1", self.data_to_save_detections1, self.data_to_save_positions1, self.data_to_save_camera_info1, experiment_folder)
        self.save_data_for_camera("2", self.data_to_save_detections2, self.data_to_save_positions2, self.data_to_save_camera_info2, experiment_folder)

    def save_data_for_camera(self, camera_id, data_to_save_detections, data_to_save_positions, data_to_save_camera_info, experiment_folder):
        filename_detections = os.path.join(experiment_folder, "saved_data_detections{}.csv".format(camera_id))
        with open(filename_detections, "w") as f:
            writer = csv.writer(f)
            writer.writerow(['timestamp', 'hz_detections', 'x', 'y', 'z'])
            writer.writerows(data_to_save_detections)

        filename_positions = os.path.join(experiment_folder, "saved_data_positions{}.csv".format(camera_id))
        with open(filename_positions, "w") as f:
            writer = csv.writer(f)
            writer.writerow(['timestamp', 'hz_positions', 'u', 'v'])
            writer.writerows(data_to_save_positions)

        filename_camera_info = os.path.join(experiment_folder, "saved_data_camera_info{}.csv".format(camera_id))
        with open(filename_camera_info, "w") as f:
            writer = csv.writer(f)
            writer.writerow(['timestamp', 'hz_camera_info', 'x_offset', 'y_offset'])
            writer.writerows(data_to_save_camera_info)

if __name__ == "__main__":
    rospy.init_node('topic_subscriber_node')
    ts = TopicSubscriber()
    rospy.on_shutdown(ts.save_data)
    rospy.spin()
