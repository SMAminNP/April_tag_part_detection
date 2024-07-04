#!/usr/bin/env python3

import rospy
from part_detection.msg import ObjectCoordinates
from filterpy.kalman import UnscentedKalmanFilter, MerweScaledSigmaPoints
import numpy as np

class ObjectCoordinateTransformer:
    def __init__(self):
        rospy.init_node('Coordinate_Publisher')
        rospy.Subscriber('/object_coordinates', ObjectCoordinates, self.object_coordinates_callback)
        self.coordinate_publisher = rospy.Publisher('/smoothed_object_coordinates', ObjectCoordinates, queue_size=10)

        # Initialize Unscented Kalman Filter
        self.points = MerweScaledSigmaPoints(n=3, alpha=0.1, beta=2., kappa=1.)
        self.ukf = UnscentedKalmanFilter(dim_x=3, dim_z=3, dt=1.0, hx=self.hx, fx=self.fx, points=self.points)

        # Initialize state variables
        self.ukf.x = np.zeros(3)
        # Initialize state covariance
        self.ukf.P = np.eye(3) * 0.1
        # Initialize measurement noise covariance
        self.ukf.R = np.eye(3) * 0.1
        # Initialize process noise covariance
        self.ukf.Q = np.eye(3) * 0.01

    def object_coordinates_callback(self, msg):
        object_coordinates = msg.coordinates

        smoothed_coordinates = []

        for i in range(0, len(object_coordinates), 3):
            x, y, z = object_coordinates[i:i+3]

            # Update UKF with observed coordinates
            self.ukf.predict()
            self.ukf.update(np.array([x, y, z]))

            # Get smoothed estimate of object coordinates
            smoothed_estimate = self.ukf.x

            smoothed_coordinates.extend(smoothed_estimate)

        # Publish the smoothed object coordinates
        smoothed_coordinates_msg = ObjectCoordinates()
        smoothed_coordinates_msg.coordinates = smoothed_coordinates
        self.coordinate_publisher.publish(smoothed_coordinates_msg)

        rospy.loginfo("Published smoothed object coordinates")

    def hx(self, x):
        return x

    def fx(self, x, dt):
        return x

if __name__ == '__main__':
    try:
        transformer = ObjectCoordinateTransformer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
