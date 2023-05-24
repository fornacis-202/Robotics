#!/usr/bin/python3

import rospy
import tf
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

class Controller:
    
    def __init__(self) -> None:
        
        rospy.init_node("controller" , anonymous=False)
        
        self.odom_subscriber = rospy.Subscriber("/odom" , Odometry , callback=self.odom_callback)
        self.cmd_publisher = rospy.Publisher('/cmd_vel' , Twist , queue_size=2)
        
        # getting specified parameters
        self.p_linear = 0.1
        self.i_linear = 0.007
        self.d_linear = 0.1

        self.p_angular = 6.0
        self.i_angular = 0.0001
        self.d_angular = 0.6

        self.epsilon = 0.1
        self.dt = 0.005

        self.odom_yaw = 0
        self.odom_x=0
        self.odom_y=0
        self.goal_x=10
        self.goal_y=0



        
    
   
    def odom_callback(self, msg: Odometry):

        
        orientation = msg.pose.pose.orientation
        # convert quaternion to odom
        roll, pitch, yaw = tf.transformations.euler_from_quaternion((
            orientation.x ,orientation.y ,orientation.z ,orientation.w
        )) 
        
        self.odom_yaw = yaw
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y


    def linear_error(self):

        dis = (abs(self.odom_x-self.goal_x)**2 + abs(self.odom_y-self.goal_y)**2)**0.5
        return dis
    



    def angular_error(self):
        x = self.goal_x-self.odom_x
        y = self.goal_y-self.odom_y
        goal_yaw = math.atan2(y,x)


        if self.odom_yaw > 0:
            sign = -1 if (self.odom_yaw - math.pi < goal_yaw < self.odom_yaw) else +1
        else:
            sign = +1 if (self.odom_yaw + math.pi > goal_yaw > self.odom_yaw) else -1

        return sign * (math.pi - abs(abs(self.odom_yaw - goal_yaw) - math.pi))
    


    def stop(self):
        twist = Twist()
        self.cmd_publisher.publish(twist)
        rospy.sleep(200)


    def control_tward_goal_pose(self):
        twist = Twist()
        sum_angular_error = 0
        sum_linear_error = 0
        prev_angular_error = 0
        prev_linear_error = 0
        while True:
            linear_error = self.linear_error()
            angular_error = self.angular_error() 

            if linear_error < self.epsilon:
                return

            sum_angular_error += (angular_error * self.dt)
            sum_linear_error += (linear_error * self.dt)

            # PID for linear speed
            P = self.p_linear * linear_error
            I = self.i_linear * sum_linear_error
            D = self.d_linear * (linear_error - prev_linear_error)
            twist.linear.x =  P + I + D


            # PID for angular speed
            P = self.p_angular * angular_error
            I = self.i_angular * sum_angular_error
            D = self.d_angular * (angular_error - prev_angular_error) 
            twist.angular.z =  P + I + D


            prev_angular_error = angular_error
            prev_linear_error = linear_error

            self.cmd_publisher.publish(twist)
            rospy.sleep(self.dt)
        


    
    def run(self):
        rospy.loginfo("run started")
        self.control_tward_goal_pose()  # initaialzed as (10,0) 
        self.stop()
            
            


                


if __name__ == "__main__":
    controller = Controller()
    controller.run()