#!/usr/bin/env python3

import rospy
import math
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray

class PIDController:
    def __init__(self):
        rospy.init_node('pid_controller', anonymous=True)
        
        # Initialize publishers and subscribers
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.reference_sub = rospy.Subscriber('/reference_pose', Float32MultiArray, self.reference_callback)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        
        # Initialize PID parameters for angular velocity
        self.angular_kp = 0.8
        self.angular_ki = 0.0
        self.angular_kd = 0.2
        self.angular_error_sum = 0.0
        self.angular_error_prev = 0.0
        
        # Initialize PID parameters for linear velocity
        self.linear_kp = 0.3
        self.linear_ki = 0.0
        self.linear_kd = 0.1
        self.linear_error_sum = 0.0
        self.linear_error_prev = 0.0
        
        # Initialize pose variables
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        # Initialize reference pose variables
        self.x_ref = 0.0
        self.y_ref = 0.0
        self.theta_ref = 0.0
        self.mode = 0
        
        # Initialize state variables
        self.state = 0  # 0: initial, 1: turning to target, 2: moving to target, 3: turning to final orientation
        
        # Control loop
        self.rate = rospy.Rate(10)  # 10 Hz
        self.run()
        
    def reference_callback(self, msg):
        # Extract reference pose from message
        data = msg.data
        self.x_ref = data[0]
        self.y_ref = data[1]
        self.theta_ref = data[2]
        self.mode = int(data[3])
        
        # Reset state for new reference
        if self.mode == 0:
            self.state = 1  # Start by turning to target
        else:
            self.state = 0  # Simultaneous control
            
        # Reset PID errors
        self.angular_error_sum = 0.0
        self.angular_error_prev = 0.0
        self.linear_error_sum = 0.0
        self.linear_error_prev = 0.0
        
        rospy.loginfo(f"New reference: x={self.x_ref}, y={self.y_ref}, theta={self.theta_ref}, mode={self.mode}")
        
    def odom_callback(self, msg):
        # Extract current pose from odometry
        pose = msg.pose.pose
        self.x = pose.position.x
        self.y = pose.position.y
        
        # Convert quaternion to Euler angle (just the yaw/z-axis rotation)
        quat = pose.orientation
        # Simplified quaternion to Euler conversion for 2D rotation
        self.theta = 2 * math.atan2(quat.z, quat.w)
        
    def calculate_target_angle(self):
        # Calculate angle to target position
        dx = self.x_ref - self.x
        dy = self.y_ref - self.y
        return math.atan2(dy, dx)
        
    def calculate_distance_error(self):
        # Calculate distance to target position
        dx = self.x_ref - self.x
        dy = self.y_ref - self.y
        return math.sqrt(dx*dx + dy*dy)
        
    def calculate_angular_pid(self, error):
        # PID control for angular velocity
        self.angular_error_sum += error
        angular_error_diff = error - self.angular_error_prev
        self.angular_error_prev = error
        
        # Anti-windup: Limit the integral term
        if abs(self.angular_error_sum) > 1.0:
            self.angular_error_sum = 1.0 if self.angular_error_sum > 0 else -1.0
        
        # Calculate PID output
        p_term = self.angular_kp * error
        i_term = self.angular_ki * self.angular_error_sum
        d_term = self.angular_kd * angular_error_diff
        
        return p_term + i_term + d_term
        
    def calculate_linear_pid(self, error):
        # PID control for linear velocity
        self.linear_error_sum += error
        linear_error_diff = error - self.linear_error_prev
        self.linear_error_prev = error
        
        # Anti-windup: Limit the integral term
        if abs(self.linear_error_sum) > 1.0:
            self.linear_error_sum = 1.0 if self.linear_error_sum > 0 else -1.0
        
        # Calculate PID output
        p_term = self.linear_kp * error
        i_term = self.linear_ki * self.linear_error_sum
        d_term = self.linear_kd * linear_error_diff
        
        # Limit the maximum output
        output = p_term + i_term + d_term
        if output > 0.5:  # Maximum forward speed
            output = 0.5
            
        return output
        
    def normalize_angle(self, angle):
        # Normalize angle to [-π, π]
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
        
    def run(self):
        while not rospy.is_shutdown():
            # Create Twist message for velocity control
            vel_msg = Twist()
            
            if self.mode == 0:  # Sequential control
                # State machine for sequential control
                if self.state == 1:  # Turning to face target
                    target_angle = self.calculate_target_angle()
                    angle_error = self.normalize_angle(target_angle - self.theta)
                    
                    if abs(angle_error) < 0.05:  # Close enough to target angle
                        self.state = 2  # Move to target
                        self.angular_error_sum = 0.0
                        rospy.loginfo("State 1 -> 2: Now moving to target")
                    else:
                        vel_msg.angular.z = self.calculate_angular_pid(angle_error)
                        
                elif self.state == 2:  # Moving to target
                    distance_error = self.calculate_distance_error()
                    
                    if distance_error < 0.1:  # Close enough to target position
                        self.state = 3  # Turn to final orientation
                        self.linear_error_sum = 0.0
                        rospy.loginfo("State 2 -> 3: Now turning to final orientation")
                    else:
                        # Keep facing the target while moving
                        target_angle = self.calculate_target_angle()
                        angle_error = self.normalize_angle(target_angle - self.theta)
                        
                        vel_msg.linear.x = self.calculate_linear_pid(distance_error)
                        vel_msg.angular.z = self.calculate_angular_pid(angle_error) * 0.5
                        
                elif self.state == 3:  # Turning to final orientation
                    angle_error = self.normalize_angle(self.theta_ref - self.theta)
                    
                    if abs(angle_error) < 0.05:  # Close enough to target orientation
                        self.state = 0  # Task complete
                        rospy.loginfo("State 3 -> 0: Target reached")
                    else:
                        vel_msg.angular.z = self.calculate_angular_pid(angle_error)
                        
            else:  # Simultaneous control (mode = 1)
                # Calculate errors
                distance_error = self.calculate_distance_error()
                
                if distance_error > 0.1:  # Still moving to position
                    # Calculate angle to target for position control
                    target_angle = self.calculate_target_angle()
                    angle_error = self.normalize_angle(target_angle - self.theta)
                    
                    # Calculate and set velocities
                    vel_msg.linear.x = self.calculate_linear_pid(distance_error)
                    vel_msg.angular.z = self.calculate_angular_pid(angle_error)
                else:
                    # We've reached the position, now turn to final orientation
                    angle_error = self.normalize_angle(self.theta_ref - self.theta)
                    
                    if abs(angle_error) < 0.05:  # Close to final orientation
                        rospy.loginfo("Target reached")
                    else:
                        vel_msg.angular.z = self.calculate_angular_pid(angle_error)
            
            # Publish velocity command
            self.cmd_vel_pub.publish(vel_msg)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        controller = PIDController()
    except rospy.ROSInterruptException:
        pass