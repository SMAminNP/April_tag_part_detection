#!/usr/bin/env python3

import rospy
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
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
        rospy.Subscriber('/rs_camera_1/color/image_rect', Image, self.color_image_callback) # Update with your color image topic
        rospy.Subscriber('/rs_camera_1/depth/registered/image_rect_raw', Image, self.depth_image_callback) # Update with your depth image topic
        rospy.Subscriber('/rs_camera_1/depth/camera_info', CameraInfo, self.camera_info_callback) # Update with your camera info topic

        self.object_coordinates_pub = rospy.Publisher('/object_coordinates', ObjectCoordinates, queue_size=10)
        self.annotated_image_pub = rospy.Publisher('/annotated_image', Image, queue_size=10)

    def color_image_callback(self, msg):
        self.color_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        annotated_image = self.detect_objects_and_annotate()
        annotated_image_msg = self.bridge.cv2_to_imgmsg(annotated_image, encoding="bgr8")
        self.annotated_image_pub.publish(annotated_image_msg)
        self.run_detection()

    def depth_image_callback(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

    def camera_info_callback(self, msg):
        self.camera_info = msg

    def detect_objects_and_annotate(self):
        annotated_image = self.color_image.copy()

        if self.color_image is not None and self.camera_info is not None:
            # Run YOLO detection on the color image
            results = self.model(self.color_image)[0]

            for result in results.boxes.tolist():
                x1, y1, x2, y2 = map(int, result[:4])  # Extract box coordinates
                score = float(result[4])
                class_id = int(result[5])

                if score > self.threshold:
                    # Draw bounding box
                    cv2.rectangle(annotated_image, (x1, y1), (x2, y2), (0, 255, 0), 2)

                    # Put label
                    label = f"{results.names[class_id]}: {score:.2f}"
                    cv2.putText(annotated_image, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        return annotated_image

    def run_detection(self):
        if self.color_image is not None and self.depth_image is not None and self.camera_info is not None:
            # Run YOLO detection on the color image
            results = self.model(self.color_image)[0]

            object_coordinates = []

            for result in results.boxes.data.tolist():
                x1, y1, x2, y2 = map(int, result[:4])  # Extract box coordinates
                score = float(result[4])
                class_id = int(result[5])

                if score > self.threshold:
                    # Calculate object center coordinates in pixel space
                    x_center = (x1 + x2) // 2
                    y_center = (y1 + y2) // 2

                    # Get depth value at object center
                    depth = self.depth_image[y_center, x_center] * 0.013  # Convert depth to meters

                    # Calculate world coordinates using camera intrinsic parameters
                    x_world = (x_center - self.camera_info.K[2]) * depth / self.camera_info.K[0]
                    y_world = (y_center - self.camera_info.K[5]) * depth / self.camera_info.K[4]
                    z_world = depth 

                    # Scale the coordinates based on the actual size of the screw
                    x_world *= actual_screw_length / (x2 - x1)
                    y_world *= actual_screw_length / (y2 - y1)

                    object_coordinates.extend([x_world, y_world, z_world])

                    rospy.loginfo("Object detected: %s at depth: %.2f meters, world coordinates: (%.2f, %.2f, %.2f)",
                                  results.names[class_id], depth, x_world, y_world, z_world)

            # Publish object coordinates
            object_coordinates_msg = ObjectCoordinates()
            object_coordinates_msg.coordinates = object_coordinates
            self.object_coordinates_pub.publish(object_coordinates_msg)

if __name__ == '__main__':
    try:
        actual_screw_length = 0.02 
        detector = ObjectDetectorROS()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
