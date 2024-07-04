#!/usr/bin/env python3

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from part_detection.msg import ObjectCoordinates

class ObjectCoordinateTransformer:
    def __init__(self):
        self.tf_broadcaster = tf2_ros.StaticTransformBroadcaster()
        self.transform_dict = {}  # Dictionary to store transform messages

        rospy.init_node('TF_publisher')
        rospy.Subscriber('/average_object_coordinates', ObjectCoordinates, self.object_coordinates_callback)

    def object_coordinates_callback(self, msg):
        object_coordinates = msg.coordinates

        # Delete old transforms
        for child_frame_id in self.transform_dict:
            transform_msg = TransformStamped()
            transform_msg.header.stamp = rospy.Time.now()
            transform_msg.header.frame_id = "rs_camera_1_color_optical_frame"  # Change this frame ID according to your setup
            transform_msg.child_frame_id = child_frame_id
            transform_msg.transform.translation.x = 0
            transform_msg.transform.translation.y = 0
            transform_msg.transform.translation.z = 0
            transform_msg.transform.rotation.w = 1.0  # Set a unit quaternion rotation
            transform_msg.header.stamp = rospy.Time(0)  # Set a very old timestamp to effectively delete the transform

            self.tf_broadcaster.sendTransform(transform_msg)

        self.transform_dict = {}  # Clear the dictionary

        # Publish new transforms
        for i in range(0, len(object_coordinates), 3):
            x, y, z = object_coordinates[i:i+3]

            transform_msg = TransformStamped()
            transform_msg.header.stamp = rospy.Time.now()
            transform_msg.header.frame_id = "rs_camera_1_color_optical_frame"  # Change this frame ID according to your setup
            transform_msg.child_frame_id = "Screw" + str(i//3)  # Unique ID for each object
            transform_msg.transform.translation.x = x
            transform_msg.transform.translation.y = y
            transform_msg.transform.translation.z = z
            transform_msg.transform.rotation.w = 1.0  # Set a unit quaternion rotation

            self.tf_broadcaster.sendTransform(transform_msg)

            self.transform_dict[transform_msg.child_frame_id] = True

            rospy.loginfo("Published static transform: %s -> %s", transform_msg.child_frame_id, transform_msg.header.frame_id)

if __name__ == '__main__':
    try:
        transformer = ObjectCoordinateTransformer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
