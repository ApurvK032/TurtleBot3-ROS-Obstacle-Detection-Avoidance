#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class ObstacleAvoidance:
    def __init__(self):
        rospy.init_node('obstacle_avoidance')
        
        
        self.teleop_sub = rospy.Subscriber('/teleop_cmd_vel', Twist, self.teleop_callback)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        
       
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        self.obstacle_detected = False
        self.teleop_cmd = Twist()
        
    def scan_callback(self, msg):

        ranges = list(msg.ranges)
        num_readings = len(ranges)
        

        angle_range = 15 
        front_indices = int((angle_range / 360.0) * num_readings)
        
      
        front_ranges = ranges[0:front_indices] + ranges[-front_indices:]

        valid_ranges = [r for r in front_ranges if not (r == float('inf') or r == 0.0 or r < 0.01)]
        
        if valid_ranges:
            min_distance = min(valid_ranges)
            self.obstacle_detected = min_distance < 0.5
            if self.obstacle_detected:
                rospy.loginfo(f"OBSTACLE AHEAD! Distance: {min_distance:.2f}m")
        else:
            self.obstacle_detected = False
            
    def teleop_callback(self, msg):
        self.teleop_cmd = msg
        
    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            twist = Twist()
            
            if self.obstacle_detected:
                
                twist.linear.x = 0.0
                twist.angular.z = -0.5 
            else:
                twist = self.teleop_cmd
                
            self.cmd_pub.publish(twist)
            rate.sleep()

if __name__ == '__main__':
    try:
        avoider = ObstacleAvoidance()
        avoider.run()
    except rospy.ROSInterruptException:
        pass
