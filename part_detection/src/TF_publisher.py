#!/usr/bin/env python3

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from part_detection.msg import ObjectCoordinates

class ObjectCoordinateTransformer:
    def __init__(self):
        self.tf_broadcaster = tf2_ros.StaticTransformBroadcaster()

        rospy.init_node('TF_publisher')
        rospy.Subscriber('/object_coordinates', ObjectCoordinates, self.object_coordinates_callback)

    def object_coordinates_callback(self, msg):
        object_coordinates = msg.coordinates

        for i in range(0, len(object_coordinates), 3):
            x, y, z = object_coordinates[i:i+3]

            transform_msg = TransformStamped()
            transform_msg.header.stamp = rospy.Time.now()
            transform_msg.header.frame_id = "rs_camera_1_color_optical_frame"  # Change this frame ID according to your setup
            transform_msg.child_frame_id = "object" + str(i//3)  # Unique ID for each object
            transform_msg.transform.translation.x = x
            transform_msg.transform.translation.y = y
            transform_msg.transform.translation.z = z
            transform_msg.transform.rotation.w = 1.0  # Set a unit quaternion rotation

            self.tf_broadcaster.sendTransform(transform_msg)

            rospy.loginfo("Published static transform: object_%d -> camera_link", i//3)

if __name__ == '__main__':
    try:
        transformer = ObjectCoordinateTransformer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
