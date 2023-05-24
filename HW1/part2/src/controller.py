#!/usr/bin/python3

import rospy
import tf
from part2.msg import ClosestObstacle
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

class Controller:
    
    def __init__(self) -> None:
        
        rospy.init_node("controller" , anonymous=False)
        
        self.odom_subscriber = rospy.Subscriber("/odom" , Odometry , callback=self.odom_callback)
        self.sensir_subsciber = rospy.Subscriber("/sensor" , ClosestObstacle , callback=self.sensor_callback)
        self.cmd_publisher = rospy.Publisher('/cmd_vel' , Twist , queue_size=10)
        
        # getting specified parameters
        self.linear_speed = rospy.get_param("linear_speed") # m/s

        self.angular_speed = 0.4
        self.epsilon_linear = 0.1
        self.epsilon_angular = 0.02
        self.odom_yaw = 0
        self.odom_x=0
        self.odom_y=0
        self.min_distance = 10.0
        self.min_direction  = 1.0

        
        # defining the states of our robot
        self.GO, self.ROTATE = 0, 1
        self.state = self.GO
         
        
    
    # heading of the robot 
    def odom_callback(self, msg: Odometry):

        
        orientation = msg.pose.pose.orientation
        # convert quaternion to odom
        roll, pitch, yaw = tf.transformations.euler_from_quaternion((
            orientation.x ,orientation.y ,orientation.z ,orientation.w
        )) 
        
        self.odom_yaw = yaw
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y


    
    
    def sensor_callback(self, msg: ClosestObstacle):
        self.min_distance = msg.distance
        self.min_direction = msg.direction

        


    def calculate_goal_angle(self):
        goal_angle = (self.odom_yaw + self.min_direction + math.pi) % (2* math.pi) 
        if goal_angle > math.pi:
            goal_angle -= 2* math.pi

        return goal_angle


    
    def run(self):
        rospy.loginfo("run started")

        while not rospy.is_shutdown():
            
            # check whether state is changed or not
            if self.state == self.GO:
                twist = Twist()
                twist.linear.x = self.linear_speed
                while self.min_distance > 2.0 or (self.min_direction > (math.pi*0.8) or self.min_direction < (math.pi* -0.8)):
                    rospy.loginfo("distance %f  ",self.min_distance)
                    self.cmd_publisher.publish(twist)
                    
                self.cmd_publisher.publish(Twist())
                rospy.sleep(3)
                self.state=self.ROTATE

            
            if self.state == self.ROTATE:
                rospy.loginfo("in rotation")
                goal_yaw = self.calculate_goal_angle()
                rospy.loginfo(goal_yaw)
                rospy.loginfo(self.odom_yaw)

                if (goal_yaw - self.odom_yaw <= math.pi and goal_yaw - self.odom_yaw >0) or goal_yaw - self.odom_yaw <= -math.pi >0:
                    twist = Twist()
                    twist.angular.z = self.angular_speed
                    rospy.loginfo(twist.angular.z)

                elif goal_yaw - self.odom_yaw == 0:
                    twist = Twist()
                else :
                    twist = Twist()
                    twist.angular.z = -self.angular_speed
                    rospy.loginfo(twist.angular.z)

                
                while(abs(self.odom_yaw-goal_yaw) > self.epsilon_angular):
                    self.cmd_publisher.publish(twist)

                rospy.loginfo("after while")
                self.cmd_publisher.publish(Twist())
                rospy.sleep(3)
                self.state=self.GO
            
            


                


if __name__ == "__main__":
    controller = Controller()
    
    controller.run()