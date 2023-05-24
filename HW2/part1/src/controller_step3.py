#!/usr/bin/python3

import rospy
import tf
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
class Controller:
    
    def __init__(self) -> None:
        
        rospy.init_node("controller" , anonymous=False)
        
        self.odom_subscriber = rospy.Subscriber("/odom" , Odometry , callback=self.odom_callback)
        self.cmd_publisher = rospy.Publisher('/cmd_vel' , Twist , queue_size=2)

        self.p_linear = 0.2
        self.i_linear = 0.02
        self.d_linear = 0.2

        self.p_angular = 2
        self.i_angular = 0
        self.d_angular = 20

        self.epsilon = 0.1
        self.dt = 0.005
        self.distance_from_wall = 0.8
        self.linear_speed = 0.3

        self.odom_yaw = 0
        self.odom_x=0
        self.odom_y=0




        
    
   
    def odom_callback(self, msg: Odometry):

        
        orientation = msg.pose.pose.orientation
        # convert quaternion to odom
        roll, pitch, yaw = tf.transformations.euler_from_quaternion((
            orientation.x ,orientation.y ,orientation.z ,orientation.w
        )) 
        
        self.odom_yaw = yaw
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y



    



    def error_from_left(self):
        laser_data = rospy.wait_for_message("/scan" , LaserScan)
        rng = laser_data.ranges[:100]
        d = min(rng)
        error =  d - self.distance_from_wall
        if error < 20.0 :
            return error
        else:
            return 20.0
    

    
    def error_from_front(self):
        laser_data = rospy.wait_for_message("/scan" , LaserScan)
        rng = laser_data.ranges[0:90] + laser_data.ranges[270:360]
        d = min(rng)
        return d - self.distance_from_wall


    def control_toward_wall(self):
        sum_linear_error = 0
        prev_linear_error = 0
        while True:
            linear_error = self.error_from_front()
            if linear_error < self.epsilon:
                return

            sum_linear_error += (linear_error * self.dt)
            twist = Twist()
            # PID for linear speed
            P = self.p_linear * linear_error
            I = self.i_linear * sum_linear_error
            D = self.d_linear * (linear_error - prev_linear_error)
            twist.linear.x =  P + I + D

            prev_linear_error = linear_error

            self.cmd_publisher.publish(twist)
            rospy.sleep(self.dt)





    def control_wallFollow(self):

        sum_angular_error = 0
        prev_angular_error = 0
        while True:
            angular_error = self.error_from_left() 
            sum_angular_error += (angular_error * self.dt)
            twist = Twist()
            twist.linear.x =  self.linear_speed  # constant speed during wall following

            # PID for angular speed
            P = self.p_angular * angular_error
            I = self.i_angular * sum_angular_error
            D = self.d_angular * (angular_error - prev_angular_error) 
            twist.angular.z =  P + I + D


            prev_angular_error = angular_error

            self.cmd_publisher.publish(twist)
            rospy.sleep(self.dt)
        


    
    def run(self):
        rospy.loginfo("run started")
        self.control_toward_wall()
        self.control_wallFollow()

                     


if __name__ == "__main__":
    controller = Controller()
    
    controller.run()