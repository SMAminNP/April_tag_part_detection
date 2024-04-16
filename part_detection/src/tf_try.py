#!/usr/bin/env python3
import rospy
import tf2_ros
import geometry_msgs.msg

def main():
    rospy.init_node('static_tf2_broadcaster')

    # Create a TF2 static broadcaster
    broadcaster = tf2_ros.StaticTransformBroadcaster()

    # Create a static transformation message
    static_transformStamped = geometry_msgs.msg.TransformStamped()
    static_transformStamped.header.stamp = rospy.Time.now()
    static_transformStamped.header.frame_id = "tag_10"
    static_transformStamped.child_frame_id = "part_1"

    # Set the translation
    static_transformStamped.transform.translation.x = 0.1
    static_transformStamped.transform.translation.y = 0.0
    static_transformStamped.transform.translation.z = 0.2

    # Set the rotation
    static_transformStamped.transform.rotation.x = 0.0
    static_transformStamped.transform.rotation.y = 0.0
    static_transformStamped.transform.rotation.z = 0.0
    static_transformStamped.transform.rotation.w = 1.0

    # Publish the static transformation
    broadcaster.sendTransform(static_transformStamped)

    rospy.spin()

if __name__ == '__main__':
    main()
