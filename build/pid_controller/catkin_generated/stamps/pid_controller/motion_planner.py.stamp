#!/usr/bin/env python3

import rospy
import math
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry

class MotionPlanner:
    def __init__(self):
        rospy.init_node('motion_planner', anonymous=True)
        
        # Initialize publisher for reference pose
        self.reference_pub = rospy.Publisher('/reference_pose', Float32MultiArray, queue_size=10)
        
        # Subscribe to robot's odometry
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        
        # Initialize robot pose variables
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        # Initialize target pose variables
        self.x_ref = 0.0
        self.y_ref = 0.0
        self.theta_ref = 0.0
        self.mode = 0
        
        # Initialize state
        self.target_set = False
        self.target_reached = True
        
        # Main loop
        self.rate = rospy.Rate(10)  # 10 Hz
        self.run()
        
    def odom_callback(self, msg):
        # Extract current pose from odometry
        pose = msg.pose.pose
        self.x = pose.position.x
        self.y = pose.position.y
        
        # Convert quaternion to Euler angle (just the yaw/z-axis rotation)
        quat = pose.orientation
        # Simplified quaternion to Euler conversion for 2D rotation
        self.theta = 2 * math.atan2(quat.z, quat.w)
        
        # Check if target has been reached
        if self.target_set and not self.target_reached:
            distance_error = math.sqrt((self.x_ref - self.x)**2 + (self.y_ref - self.y)**2)
            angle_error = abs(self.normalize_angle(self.theta_ref - self.theta))
            
            if distance_error <= 0.1 and angle_error <= 0.1:
                self.target_reached = True
                rospy.loginfo(f"Target reached! Final pose: x={self.x:.2f}, y={self.y:.2f}, theta={self.theta:.2f}")
                rospy.loginfo(f"Error: position={distance_error:.4f}, angle={angle_error:.4f}")
                
    def normalize_angle(self, angle):
        # Normalize angle to [-π, π]
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
        
    def get_user_input(self):
        # Get target pose from user
        try:
            x_input = float(input("Enter target x position: "))
            y_input = float(input("Enter target y position: "))
            theta_input = float(input("Enter target orientation (radians): "))
            mode_input = int(input("Enter mode (0: sequential, 1: simultaneous): "))
            
            # Validate input
            if mode_input not in [0, 1]:
                rospy.logwarn("Invalid mode. Using default mode: 0")
                mode_input = 0
                
            return x_input, y_input, theta_input, mode_input
        except ValueError:
            rospy.logerr("Invalid input. Please enter numeric values.")
            return None
        
    def publish_reference(self):
        # Create and publish reference pose message
        msg = Float32MultiArray()
        msg.data = [self.x_ref, self.y_ref, self.theta_ref, self.mode]
        self.reference_pub.publish(msg)
        
        rospy.loginfo(f"Published reference: x={self.x_ref}, y={self.y_ref}, theta={self.theta_ref}, mode={self.mode}")
        
    def run(self):
        while not rospy.is_shutdown():
            if self.target_reached:
                rospy.loginfo("Enter a new target pose:")
                
                input_result = self.get_user_input()
                if input_result is not None:
                    self.x_ref, self.y_ref, self.theta_ref, self.mode = input_result
                    self.target_set = True
                    self.target_reached = False
                    
                    # Publish the new reference
                    self.publish_reference()
            
            self.rate.sleep()

if __name__ == '__main__':
    try:
        planner = MotionPlanner()
    except rospy.ROSInterruptException:
        pass