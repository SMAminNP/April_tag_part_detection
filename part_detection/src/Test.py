import os
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from ultralytics import YOLO
from part_detection.msg import ObjectCoordinates
import pyrealsense2 as rs

class ObjectDetectorROS:
    def __init__(self):
        self.bridge = CvBridge()
        self.model_path = '/home/amin/YOLO/runs/detect/train/weights/best.pt'  # Update with your model path
        self.threshold = 0.5
        self.model = YOLO(self.model_path)

        self.color_image = None
        self.depth_image = None
        self.camera_info = None

        ctx = rs.context()
        serials = []
        devices = ctx.query_devices()
        for dev in devices:
            dev.hardware_reset()

        if len(ctx.devices) > 0:
            for dev in ctx.devices:
                rospy.loginfo('Found device: %s %s', dev.get_info(rs.camera_info.name), dev.get_info(rs.camera_info.serial_number))
                serials.append(dev.get_info(rs.camera_info.serial_number))
        else:
            rospy.loginfo("No Intel Device connected")

        self.pipelines = []
        self.windows = []

        for serial in serials:
            pipe = rs.pipeline(ctx)
            cfg = rs.config()
            cfg.enable_device(serial)
            cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
            cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
            pipe.start(cfg)
            self.pipelines.append(pipe)

            window_name = f"Camera {serial}"
            cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
            self.windows.append(window_name)

        rospy.init_node('object_detector_ros')
        rospy.Subscriber('/rs_camera_1/color/image_rect', Image, self.color_image_callback) # Update with your color image topic
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
                
                results = self.model(self.color_image)[0]

                object_coordinates = []

                for result in results.boxes.data.tolist():
                    x1, y1, x2, y2, score, class_id = result

                    if score > self.threshold:
                        # Calculate object center coordinates in pixel space
                        x_center = int((x1 + x2) / 2)
                        y_center = int((y1 + y2) / 2)

                        # Get depth value at object center
                        depth = self.depth_image[y_center, x_center] * 0.001  # Convert depth to meters

                        # Convert color image to grayscale
                        grayscale_image = cv2.cvtColor(self.color_image, cv2.COLOR_BGR2GRAY)

                        # Convert grayscale image to three-channel by stacking the single channel
                        rgb_image = np.stack((grayscale_image,) * 3, axis=-1)

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

                # Display images
                for color_image, window_name in zip(self.color_image, self.windows):
                    cv2.imshow(window_name, color_image)

            rate.sleep()

if __name__ == '__main__':
    try:
        detector = ObjectDetectorROS()
        detector.run_detection()
    except rospy.ROSInterruptException:
        pass
