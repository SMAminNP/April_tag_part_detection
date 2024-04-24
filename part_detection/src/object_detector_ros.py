#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import numpy as np
from ultralytics import YOLO
from part_detection.msg import ObjectCoordinates
import cv2

class ObjectDetectorROS:
    def __init__(self):
        self.bridge = CvBridge()
        self.model_path = '/home/amin/YOLO/runs/detect/train/weights/best.pt'  # Update with your model path
        self.threshold = 0.5
        self.model = YOLO(self.model_path)

        self.color_image = None
        self.depth_image = None
        self.camera_info = None

        rospy.init_node('object_detector_ros')
        rospy.Subscriber('/rs_camera_1/color/image_raw', Image, self.color_image_callback) # Update with your color image topic
        rospy.Subscriber('/rs_camera_1/depth/image_rect_raw', Image, self.depth_image_callback) # Update with your depth image topic
        rospy.Subscriber('/rs_camera_1/depth/camera_info', CameraInfo, self.camera_info_callback) # Update with your camera info topic

        self.object_coordinates_pub = rospy.Publisher('/object_coordinates', ObjectCoordinates, queue_size=10)

    def color_image_callback(self, msg):
        self.color_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def depth_image_callback(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

    def camera_info_callback(self, msg):
        self.camera_info = msg

    def run_detection(self):
        rate = rospy.Rate(10)  # Adjust the rate according to your needs

        while not rospy.is_shutdown():
            if self.color_image is not None and self.depth_image is not None and self.camera_info is not None:
                # Run YOLO detection on the color image
                results = self.model(self.color_image)[0]

                object_coordinates = []

                for result in results.boxes.data.tolist():
                    x1, y1, x2, y2, score, class_id = result

                    if score > self.threshold:
                        # Calculate object center coordinates in pixel space
                        x_center = int((x1 + x2) / 2)
                        y_center = int((y1 + y2) / 2)

                        # Get depth value at object center
                        depth = self.depth_image[y_center, x_center] * self.camera_info.K[0] * 0.00005  # Convert depth to meters

                        # Calculate world coordinates using camera intrinsic parameters
                        x_world = (x_center - self.camera_info.K[2]) * depth / self.camera_info.K[0]
                        y_world = (y_center - self.camera_info.K[5]) * depth / self.camera_info.K[4]
                        z_world = depth

                        object_coordinates.extend([x_world, y_world, z_world])

                        rospy.loginfo("Object detected: %s at depth: %.2f meters, world coordinates: (%.2f, %.2f, %.2f)",
                                      results.names[int(class_id)], depth, x_world, y_world, z_world)

                # Publish object coordinates
                object_coordinates_msg = ObjectCoordinates()
                object_coordinates_msg.coordinates = object_coordinates
                self.object_coordinates_pub.publish(object_coordinates_msg)

                cv2.imshow("Color Image", self.color_image)
                cv2.imshow("Depth Image", self.depth_image)
                cv2.waitKey(1)

            rate.sleep()

if __name__ == '__main__':
    try:
        detector = ObjectDetectorROS()
        detector.run_detection()
    except rospy.ROSInterruptException:
        pass
