#!/usr/bin/env python
# -*- coding: utf-8 -*-
# PWM : 35.743 ~ 59.179 -- 61.133 ~ 86.523

import rospy
import cv2
import message_filters
import numpy as np
# msg
from sensor_msgs.msg import Image, CameraInfo
from apriltag_ros.msg import AprilTagDetectionArray
from apriltag_ros.msg import AprilTagDetectionPositionArray

from cv_bridge import CvBridge, CvBridgeError



class tracking_apriltag(object):
	def __init__(self):
		# ROS
		rospy.init_node("tracking_apriltag")
		rospy.on_shutdown(self.cleanup)

		self.bridge = CvBridge()
		self.image_pub = rospy.Publisher("/masking_image", Image, queue_size=1)
		self.info_pub = rospy.Publisher("/masking_info", CameraInfo, queue_size=1)
		self.tag_det = rospy.Subscriber('/tag_detections',AprilTagDetectionArray,self.tag_camera_callback)
		self.tag_pos = rospy.Subscriber('/tag_position',AprilTagDetectionPositionArray,self.tag_image_callback)
		sub1 = message_filters.Subscriber('/usb_cam/image_raw', Image)
		sub2 = message_filters.Subscriber('/usb_cam/camera_info', CameraInfo)
		ts = message_filters.ApproximateTimeSynchronizer([sub1,sub2], 1, 0.5)
		ts.registerCallback(self.image_callback)

		# image size
		self.image_size = [1280,720]

		# Tag_camera
		self.Position_now_camera = [0, 0, 0]
		self.Position_now_camera2 = [0, 0, 0]  # For the second tag

		# Tag_image
		self.Position_now_image = [0, 0]
		self.Position_now_image2 = [0, 0]  # For the second tag

		# flag Tag
		self.flag_image = 0
		self.flag_detection = 0

		# flaf image prosess
		self.flag_trim = 1

		# Time
		self.time_start = 0
		self.time = 0

		# trim_position
		self.trim0_u0 = 0
		self.trim0_u1 = 1280
		self.trim0_v0 = 0
		self.trim0_v1 = 720


	def image_callback(self, ros_image, camera_info):

		if self.flag_detection == 1:
			input_image = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
			output_image, camera_info = self.image_process(input_image, camera_info)
		else:
			output_image = ros_image

		now = rospy.Time.now()
		output_image.header.stamp = now
		camera_info.header.stamp = now
		self.image_pub.publish(output_image)
		self.info_pub.publish(camera_info)



	def image_process(self, input_image, camera_info):

		if self.flag_trim == 1:
			trim_image = self.Trim(input_image)
			if self.flag_trim == 1:
				camera_info = self.ROI_process(camera_info)
				output_image = self.bridge.cv2_to_imgmsg(np.array(trim_image), "bgr8")
			else:
				output_image = self.bridge.cv2_to_imgmsg(np.array(input_image), "bgr8")
		else:
			output_image = self.bridge.cv2_to_imgmsg(np.array(input_image), "bgr8")

		return output_image, camera_info



	# Trimming Process
	def Trim(self,input_image):
		self.Wide_Trim()
		trim_image = input_image[self.trim0_v0:self.trim0_v1, self.trim0_u0:self.trim0_u1]

		return trim_image



	def Wide_Trim(self):
		center_u1 = self.Position_now_image[0]
		center_v1 = self.Position_now_image[1]
		center_u2 = self.Position_now_image2[0]  # For the second tag
		center_v2 = self.Position_now_image2[1]  # For the second tag


		f = 1581
		z = self.Position_now_camera.z
		z2 = self.Position_now_camera2.z  # For the second tag

		Length_Tag_world = 0.043

		Length_Tag_image1 = f * (Length_Tag_world / z)
		Length_Tag_image2 = f * (Length_Tag_world / z2)  # For the second tag

		alpha = 1.0


		# Calculate the coordinates of the corners of the trimming area for each tag
		u0_1 = int(center_u1 - Length_Tag_image1 * alpha)
		u1_1 = int(center_u1 + Length_Tag_image1 * alpha)
		v0_1 = int(center_v1 - Length_Tag_image1 * alpha)
		v1_1 = int(center_v1 + Length_Tag_image1 * alpha)

		u0_2 = int(center_u2 - Length_Tag_image2 * alpha)
		u1_2 = int(center_u2 + Length_Tag_image2 * alpha)
		v0_2 = int(center_v2 - Length_Tag_image2 * alpha)
		v1_2 = int(center_v2 + Length_Tag_image2 * alpha)

		# Use the minimum u0 and v0, and the maximum u1 and v1
		self.trim0_u0 = min(u0_1, u0_2)
		self.trim0_u1 = max(u1_1, u1_2)
		self.trim0_v0 = min(v0_1, v0_2)
		self.trim0_v1 = max(v1_1, v1_2)

		if self.trim0_u0 < 0:
			self.trim0_u0 = 0
		if self.trim0_v0 < 0:
			self.trim0_v0 = 0


	def ROI_process(self, camera_info):
		width = self.trim0_u1 - self.trim0_u0
		height = self.trim0_v1 - self.trim0_v0
		# Modify the RegionOfInterest in the CameraInfo message
		camera_info.roi.x_offset = self.trim0_u0
		camera_info.roi.y_offset = self.trim0_v0
		camera_info.roi.width = width
		camera_info.roi.height = height
		camera_info.roi.do_rectify = True

		return camera_info



	def tag_camera_callback(self,data_camera):
		if len(data_camera.detections) == 1:
			self.Position_now_camera = data_camera.detections[0].pose.pose.pose.position
			self.Position_now_camera2 = data_camera.detections[0].pose.pose.pose.position  # Use the same data for the second tag
			self.flag_detection = 1
		elif len(data_camera.detections) >= 2:
			self.Position_now_camera = data_camera.detections[0].pose.pose.pose.position
			self.Position_now_camera2 = data_camera.detections[1].pose.pose.pose.position  # Save the second tag's data
			self.flag_detection = 1
		else:
			self.flag_detection = 0



	def tag_image_callback(self, data_image):
		if len(data_image.detect_positions) == 1:
				self.Position_now_image[0] = data_image.detect_positions[0].x
				self.Position_now_image[1] = data_image.detect_positions[0].y
				self.Position_now_image2[0] = data_image.detect_positions[0].x  # Use the same data for the second tag
				self.Position_now_image2[1] = data_image.detect_positions[0].y  # Use the same data for the second tag
		elif len(data_image.detect_positions) >= 2:
				self.Position_now_image[0] = data_image.detect_positions[0].x
				self.Position_now_image[1] = data_image.detect_positions[0].y
				self.Position_now_image2[0] = data_image.detect_positions[1].x  # Save the second tag's data
				self.Position_now_image2[1] = data_image.detect_positions[1].y  # Save the second tag's data
		else:
			# init
			self.trim0_u0 = 0
			self.trim0_u1 = 1280
			self.trim0_v0 = 0
			self.trim0_v1 = 720



	def cleanup(self):
		cv2.destroyAllWindows()



if __name__ == "__main__":
	ts = tracking_apriltag()
	rospy.spin()
