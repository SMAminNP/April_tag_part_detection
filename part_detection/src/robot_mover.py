#!/usr/bin/env python3

import rospy
from rtde_control import RTDEControlInterface as RTDEControl
from part_detection.msg import ObjectCoordinates

class RobotMover:
    def __init__(self):
        rospy.init_node('robot_mover')
        self.rtde_c = RTDEControl("10.1.1.2", RTDEControl.FLAG_CUSTOM_SCRIPT)
        rospy.Subscriber('/average_object_coordinates', ObjectCoordinates, self.object_coordinates_callback)
        rospy.spin()

    def object_coordinates_callback(self, msg):
        object_coordinates = msg.coordinates

        
        if len(object_coordinates) >= 3:
            x, y, z = object_coordinates[:3]
            target_pose = [-x, -y, -z, 0, 0, 0]  
            speed = 0.1
            acceleration = 0.1

            rospy.loginfo(f"Moving to coordinates: {target_pose}")
            self.rtde_c.moveL(target_pose, speed, acceleration)

if __name__ == '__main__':
    try:
        RobotMover()
    except rospy.ROSInterruptException:
        pass
