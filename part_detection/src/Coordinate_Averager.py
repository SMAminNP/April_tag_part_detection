#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import TransformStamped
from part_detection.msg import ObjectCoordinates
import numpy as np
import time
from collections import Counter

class CoordinateAverager:
    def __init__(self):
        self.coordinates = []
        self.pub = rospy.Publisher('/average_object_coordinates', ObjectCoordinates, queue_size=10)
        rospy.Subscriber('/smoothed_object_coordinates', ObjectCoordinates, self.coordinates_callback)

    def coordinates_callback(self, msg):
        self.coordinates = []  # Clear previously stored coordinates
        self.coordinates.append(msg.coordinates)

    def run(self):
        rate = rospy.Rate(10)  # Adjust the rate according to your needs
        while not rospy.is_shutdown():
            rospy.loginfo("Collecting coordinates for 5 seconds...")
            start_time = time.time()
            while time.time() - start_time < 5:
                pass

            if self.coordinates:
                average_coordinates = self.calculate_average_coordinates()
                if average_coordinates is not None:
                    self.publish_tf(average_coordinates)

            rospy.loginfo("Waiting for 5 seconds before collecting new coordinates...")
            rospy.sleep(5)

    def calculate_average_coordinates(self):
        if not self.coordinates:
            return None

        # Count the occurrences of each length
        lengths_counter = Counter(len(coord) for coord in self.coordinates)

        # Find the most common length
        most_common_length = lengths_counter.most_common(1)[0][0]

        # Filter coordinates arrays to only include arrays with the most common length
        filtered_coordinates = [coord for coord in self.coordinates if len(coord) == most_common_length]

        # Calculate the average of the filtered coordinates
        average_coordinates = np.mean(filtered_coordinates, axis=0)

        return average_coordinates

    def publish_tf(self, coordinates):
        msg = ObjectCoordinates()
        msg.coordinates = coordinates
        self.pub.publish(msg)

if __name__ == '__main__':
    rospy.init_node('coordinate_averager')
    averager = CoordinateAverager()
    averager.run()
