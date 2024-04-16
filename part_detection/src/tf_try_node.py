#!/usr/bin/env python3
import rospy
from part_detection.tf_try import TfTry

if __name__ == "__main__":
    rospy.init_node("tf_example")
    tfExample = TfExamples()
    rospy.spin()
