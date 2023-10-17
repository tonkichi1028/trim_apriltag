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
		self.Position_now_camera1 = [0, 0, 0]
		self.Position_now_camera2 = [0, 0, 0]  # For the second tag

		# Tag_image
		self.Position_now_image1 = [0, 0]
		self.Position_now_image2 = [0, 0]  # For the second tag

		# flag Tag
		self.flag_detection1 = 0
		self.flag_detection2 = 0


		# flaf image prosess
		self.flag_trim = 1

		# Time
		self.time_start = 0
		self.time = 0

		# trim_position
		self.trim1_u0 = 0
		self.trim1_u1 = 1280
		self.trim1_v0 = 0
		self.trim1_v1 = 720

		self.trim2_u0 = 0
		self.trim2_u1 = 1280
		self.trim2_v0 = 0
		self.trim2_v1 = 720


	def image_callback(self, ros_image, camera_info):
		if self.flag_detection1 == 1 or self.flag_detection2 == 1:
			input_image = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
			output_image1, output_image2, camera_info1, camera_info2 = self.image_process(input_image, camera_info)
		else:
			output_image1 = ros_image
			output_image2 = ros_image  # Add this line to handle the case where no trimming is performed
			camera_info1 = camera_info  # Use the original CameraInfo message for the first image
			camera_info2 = camera_info  # Add this line to handle the case where no trimming is performed

		now = rospy.Time.now()
		output_image1.header.stamp = now
		camera_info1.header.stamp = now
		self.image_pub.publish(output_image1)
		self.info_pub.publish(camera_info1)

		#if output_image2 is not None and camera_info2 is not None:  # Add this condition to handle the case where no trimming is performed
		output_image2.header.stamp = now
		camera_info2.header.stamp = now
		#self.image_pub.publish(output_image2)  # Add this line to publish the second image
		#self.info_pub.publish(camera_info2)  # Add this line to publish the second CameraInfo message




	def image_process(self, input_image, camera_info):
		#self.flag_detection = 0
		if self.flag_detection1 == 1:
			trim_image1 = self.Trim(input_image, 1)  # Modify this line to return the trimmed image for the first tag
			camera_info1 = self.ROI_process(camera_info, 1)  # Process the CameraInfo for the first tag
			trim_image1 = self.draw_center_point(trim_image1)
			output_image1 = self.bridge.cv2_to_imgmsg(np.array(trim_image1), "bgr8")
		else:
			output_image1 = self.bridge.cv2_to_imgmsg(np.array(input_image), "bgr8")
			camera_info1 = camera_info  # Use the original CameraInfo message for the first image
			trim_image1 = self.draw_center_point(trim_image1)


		#self.flag_detection2 = 0
		if self.flag_detection2 == 1:
			trim_image2 = self.Trim(input_image, 2)  # Add this line to return the trimmed image for the second tag
			camera_info2 = self.ROI_process(camera_info, 2)  # Add this line to process the CameraInfo for the second tag
			trim_image2 = self.draw_center_point(trim_image2)
			output_image2 = self.bridge.cv2_to_imgmsg(np.array(trim_image2), "bgr8")
		else:
			output_image2 = self.bridge.cv2_to_imgmsg(np.array(input_image), "bgr8")
			camera_info2 = camera_info  # Use the original CameraInfo message for the second image

		return output_image1, output_image2, camera_info1, camera_info2


	# Trimming Process
	def Trim(self, input_image, tag_number):
		self.Wide_Trim(tag_number)  # Pass the tag number to the Wide_Trim function
		if tag_number == 1:
			trim_image = input_image[self.trim1_v0:self.trim1_v1, self.trim1_u0:self.trim1_u1]  # Trim the first tag
		else:  # tag_number == 2
			trim_image = input_image[self.trim2_v0:self.trim2_v1, self.trim2_u0:self.trim2_u1]  # Trim the second tag

		trim_image = self.draw_center_point(trim_image)

		return trim_image  # Return the trimmed image
	# Trimming Process


	def draw_center_point(self, image):
		# Calculate the center coordinates
		center_coordinates = (image.shape[1] // 2, image.shape[0] // 2)

		# Define the parameters for the circle
		radius = 5
		color = (0, 0, 255)  # BGR format for red
		thickness = -1  # Fill the circle

		# Draw the circle
		image = cv2.circle(image, center_coordinates, radius, color, thickness)

		return image



	def Wide_Trim(self, tag_number):
		if tag_number == 1:
			center_u1 = self.Position_now_image1[0]
			center_v1 = self.Position_now_image1[1]
			z1 = self.Position_now_camera1.z
		else:  # tag_number == 2
			center_u2 = self.Position_now_image2[0]
			center_v2 = self.Position_now_image2[1]
			z2 = self.Position_now_camera2.z

		f = 1581
		Length_Tag_world = 0.043
		if tag_number == 1:
			Length_Tag_image1 = f * (Length_Tag_world / z1)
		else:  # tag_number == 2
			Length_Tag_image2 = f * (Length_Tag_world / z2)
		alpha = 2

		# Calculate the coordinates of the corners of the trimming area for the tag
		if tag_number == 1:
			trim_u0_1 = int(center_u1 - Length_Tag_image1 * alpha)
			trim_u1_1 = int(center_u1 + Length_Tag_image1 * alpha)
			trim_v0_1 = int(center_v1 - Length_Tag_image1 * alpha)
			trim_v1_1 = int(center_v1 + Length_Tag_image1 * alpha)
		else:  # tag_number == 2
			trim_u0_2 = int(center_u2 - Length_Tag_image2 * alpha)
			trim_u1_2 = int(center_u2 + Length_Tag_image2 * alpha)
			trim_v0_2 = int(center_v2 - Length_Tag_image2 * alpha)
			trim_v1_2 = int(center_v2 + Length_Tag_image2 * alpha)

		if tag_number == 1:
			if trim_u0_1 < 0:
				trim_u0_1 = 0
			if trim_v0_1 < 0:
				trim_v0_1 = 0
		else:  # tag_number == 2
			if trim_u0_2 < 0:
				trim_u0_2 = 0
			if trim_v0_2 < 0:
				trim_v0_2 = 0

		# Update the trimming coordinates for the tag
		if tag_number == 1:
			self.trim1_u0 = trim_u0_1
			self.trim1_u1 = trim_u1_1
			self.trim1_v0 = trim_v0_1
			self.trim1_v1 = trim_v1_1
		else:  # tag_number == 2
			self.trim2_u0 = trim_u0_2
			self.trim2_u1 = trim_u1_2
			self.trim2_v0 = trim_v0_2
			self.trim2_v1 = trim_v1_2

	def ROI_process(self, camera_info, tag_number):
		if tag_number == 1:
			width1 = self.trim1_u1 - self.trim1_u0
			height1 = self.trim1_v1 - self.trim1_v0
			x_offset1 = self.trim1_u0
			y_offset1 = self.trim1_v0
		else:  # tag_number == 2
			width2 = self.trim2_u1 - self.trim2_u0
			height2 = self.trim2_v1 - self.trim2_v0
			x_offset2 = self.trim2_u0
			y_offset2 = self.trim2_v0

		# Modify the RegionOfInterest in the CameraInfo message
		if tag_number == 1:
			camera_info.roi.x_offset = x_offset1
			camera_info.roi.y_offset = y_offset1
			camera_info.roi.width = width1
			camera_info.roi.height = height1
		else:  # tag_number == 2
			camera_info.roi.x_offset = x_offset2
			camera_info.roi.y_offset = y_offset2
			camera_info.roi.width = width2
			camera_info.roi.height = height2
		camera_info.roi.do_rectify = True

		return camera_info

	def tag_camera_callback(self,data_camera):
		if len(data_camera.detections) == 1:
			if data_camera.detections[0].id[0] == 0:
				self.Position_now_camera1 = data_camera.detections[0].pose.pose.pose.position
				self.flag_detection1 = 1
				self.flag_detection2 = 0
			else:  # data_camera.detections[0].id[0] == 1
				self.Position_now_camera2 = data_camera.detections[0].pose.pose.pose.position
				self.flag_detection1 = 0
				self.flag_detection2 = 1

		elif len(data_camera.detections) == 2:
			if data_camera.detections[0].id[0] == 0:
				self.Position_now_camera1 = data_camera.detections[0].pose.pose.pose.position
				self.Position_now_camera2 = data_camera.detections[1].pose.pose.pose.position
			else:  # data_camera.detections[0].id[0] == 1
				self.Position_now_camera1 = data_camera.detections[1].pose.pose.pose.position
				self.Position_now_camera2 = data_camera.detections[0].pose.pose.pose.position
			self.flag_detection1 = 1
			self.flag_detection2 = 1

		else:
			self.flag_detection1 = 0
			self.flag_detection2 = 0


	def tag_image_callback(self, data_image):
		if len(data_image.detect_positions) == 1:
			if data_image.detect_positions[0].id == 0:
				self.Position_now_image1[0] = data_image.detect_positions[0].x
				self.Position_now_image1[1] = data_image.detect_positions[0].y
				# Don't reset the second tag's data
			else:  # data_image.detect_positions[0].id == 1
				self.Position_now_image2[0] = data_image.detect_positions[0].x
				self.Position_now_image2[1] = data_image.detect_positions[0].y
				# Don't reset the first tag's data

		elif len(data_image.detect_positions) >= 2:
			if data_image.detect_positions[0].id == 0:
				self.Position_now_image1[0] = data_image.detect_positions[0].x
				self.Position_now_image1[1] = data_image.detect_positions[0].y
				self.Position_now_image2[0] = data_image.detect_positions[1].x  # Save the second tag's data
				self.Position_now_image2[1] = data_image.detect_positions[1].y  # Save the second tag's data
			else:  # data_image.detect_positions[0].id == 1
				self.Position_now_image1[0] = data_image.detect_positions[1].x
				self.Position_now_image1[1] = data_image.detect_positions[1].y
				self.Position_now_image2[0] = data_image.detect_positions[0].x  # Save the second tag's data
				self.Position_now_image2[1] = data_image.detect_positions[0].y  # Save the second tag's data

		else:
			# init
			self.trim0_u0 = 0
			self.trim0_u1 = 1280
			self.trim0_v0 = 0
			self.trim0_v1 = 720
			self.trim1_u0 = 0  # Add this line for the second tag
			self.trim1_u1 = 1280  # Add this line for the second tag
			self.trim1_v0 = 0  # Add this line for the second tag
			self.trim1_v1 = 720  # Add this line for the second tag


	def cleanup(self):
		cv2.destroyAllWindows()



if __name__ == "__main__":
    try:
        ts = tracking_apriltag()
        rospy.spin()
    except Exception as e:
        print("An error occurred: {e}")
