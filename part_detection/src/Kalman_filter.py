#!/usr/bin/env python3

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from part_detection.msg import ObjectCoordinates
from filterpy.kalman import KalmanFilter
import numpy as np

class ObjectCoordinateTransformer:
    def __init__(self):
        self.tf_broadcaster = tf2_ros.StaticTransformBroadcaster()

        rospy.init_node('TF_publisher')
        rospy.Subscriber('/object_coordinates', ObjectCoordinates, self.object_coordinates_callback)

        # Initialize Kalman filter
        self.kf = KalmanFilter(dim_x=3, dim_z=3)
        self.kf.F = np.eye(3)  # Identity matrix for the state transition model
        self.kf.H = np.eye(3)  # Identity matrix for the observation model
        self.kf.Q = np.eye(3) * 0.1  # Process noise covariance
        self.kf.R = np.eye(3) * 0.1  # Measurement noise covariance
        self.kf.x = np.zeros(3)  # Initial state estimate
        self.kf.P = np.eye(3)  # Initial state covariance

    def object_coordinates_callback(self, msg):
        object_coordinates = msg.coordinates

        for i in range(0, len(object_coordinates), 3):
            x, y, z = object_coordinates[i:i+3]

            # Update Kalman filter with observed coordinates
            self.kf.predict()
            self.kf.update(np.array([x, y, z]))

            # Get smoothed estimate of object coordinates
            smoothed_estimate = self.kf.x

            # Create a static transform message
            transform_msg = TransformStamped()
            transform_msg.header.stamp = rospy.Time.now()
            transform_msg.header.frame_id = "rs_camera_1_color_optical_frame"  # Change this frame ID according to your setup
            transform_msg.child_frame_id = "Screw" + str(i//3)  # Unique ID for each object
            transform_msg.transform.translation.x = smoothed_estimate[0]
            transform_msg.transform.translation.y = smoothed_estimate[1]
            transform_msg.transform.translation.z = smoothed_estimate[2]
            transform_msg.transform.rotation.w = 1.0  # Set a unit quaternion rotation

            # Publish the static transform
            self.tf_broadcaster.sendTransform(transform_msg)

            rospy.loginfo("Published static transform: object_%d -> camera_link", i//3)

if __name__ == '__main__':
    try:
        transformer = ObjectCoordinateTransformer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
