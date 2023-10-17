#!/usr/bin/env python
import rospy
from apriltag_ros.msg import AprilTagDetectionArray
from apriltag_ros.msg import AprilTagDetectionPositionArray
import csv
import os
from datetime import datetime

class TopicSubscriber(object):
    def __init__(self):
        self.data_detections = None
        self.data_position = None
        self.subscriber_detections = rospy.Subscriber("/tag_detections", TagDetections, self.callback_detections)
        self.subscriber_position = rospy.Subscriber("/tag_position", TagPosition, self.callback_position)
        self.data_to_save = []
        self.prev_time_detections = rospy.get_time()
        self.prev_time_position = rospy.get_time()

    def callback_detections(self, msg):
        curr_time = rospy.get_time()
        hz_detections = 1.0 / (curr_time - self.prev_time_detections)
        self.prev_time_detections = curr_time
        self.data_detections = msg.data
        self.data_to_save.append([msg.data, hz_detections, 'tag_detections'])
        rospy.loginfo("Data saved: %s, Frequency: %f Hz", self.data_detections, hz_detections)

    def callback_position(self, msg):
        curr_time = rospy.get_time()
        hz_position = 1.0 / (curr_time - self.prev_time_position)
        self.prev_time_position = curr_time
        self.data_position = msg.data
        self.data_to_save.append([msg.data, hz_position, 'tag_position'])
        rospy.loginfo("Data saved: %s, Frequency: %f Hz", self.data_position, hz_position)

    def save_data(self):
        data_folder = "/home/wanglab/catkin_wsTrim/src/trim_apriltag/data/"
        date_folder = data_folder + datetime.now().strftime("%Y-%m-%d")

        if not os.path.exists(date_folder):
            os.makedirs(date_folder)

        existing_folders = [f for f in os.listdir(date_folder) if os.path.isdir(os.path.join(date_folder, f))]
        experiment_folder = os.path.join(date_folder, 'experiment_{}'.format(len(existing_folders) + 1))

        if not os.path.exists(experiment_folder):
            os.makedirs(experiment_folder)

        filename = os.path.join(experiment_folder, "saved_data.csv")
        with open(filename, "w") as f:
            writer = csv.writer(f)
            writer.writerows(self.data_to_save)

if __name__ == "__main__":
    rospy.init_node('topic_subscriber_node')
    ts = TopicSubscriber()
    rospy.on_shutdown(ts.save_data)
    rospy.spin()
