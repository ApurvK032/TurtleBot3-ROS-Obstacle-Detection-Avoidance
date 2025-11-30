#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray

def scan_callback(msg):
    
    ranges_msg = Float32MultiArray()
    ranges_msg.data = msg.ranges
    ranges_pub.publish(ranges_msg)

if __name__ == '__main__':
    rospy.init_node('obstacle_detection')
    ranges_pub = rospy.Publisher('/ranges', Float32MultiArray, queue_size=10)
    rospy.Subscriber('/scan', LaserScan, scan_callback)
    rospy.spin()
