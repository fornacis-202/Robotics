#!/usr/bin/python3

import rospy
import tf
from part1.srv import GetNextDestination
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

class Controller:
    
    def __init__(self) -> None:
        
        rospy.init_node("controller" , anonymous=False)
        
        self.odom_subscriber = rospy.Subscriber("/odom" , Odometry , callback=self.odom_callback)
        self.cmd_publisher = rospy.Publisher('/cmd_vel' , Twist , queue_size=10)
        
        # getting specified parameters
        self.linear_speed = rospy.get_param("linear_speed") # m/s

        self.angular_speed = 0.2
        self.epsilon_linear = 0.1
        self.epsilon_angular = 0.01
        self.odom_yaw = 0
        self.odom_x=0
        self.odom_y=0
        self.goal_x=5
        self.goal_y=5

        
        # defining the states of our robot
        self.GO, self.ROTATE = 0, 1
        self.state = self.ROTATE
         
        
    
   
    def odom_callback(self, msg: Odometry):

        
        orientation = msg.pose.pose.orientation
        # convert quaternion to odom
        roll, pitch, yaw = tf.transformations.euler_from_quaternion((
            orientation.x ,orientation.y ,orientation.z ,orientation.w
        )) 
        
        self.odom_yaw = yaw
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y


    def distance_calculator(self, x1 ,y1 ,x2 ,y2):
        dis = (abs(x1-x2)**2 + abs(y1-y2)**2)**0.5
        return dis
    
    
    def recieve_goal_pose(self):
        rospy.wait_for_service('/GetNextDestination')
        resp = rospy.ServiceProxy('/GetNextDestination', GetNextDestination)(self.odom_x,self.odom_y)
        self.goal_x = resp.next_x
        self.goal_y = resp.next_y


    def calculate_goal_angle(self):
        x = self.goal_x-self.odom_x
        y = self.goal_y-self.odom_y
        angle = math.atan2(y,x)

        return angle


    
    def run(self):
        rospy.loginfo("run started")

        count  = 0
        error_sum = 0.0
        while not rospy.is_shutdown():
            
            # check whether state is changed or not
            if self.state == self.GO:
                count +=1
                twist = Twist()
                twist.linear.x = self.linear_speed
                min = 40.0
                current = 50.0 
                while((current := self.distance_calculator(self.odom_x,self.odom_y,self.goal_x,self.goal_y)) - min < self.epsilon_linear):
                    rospy.loginfo("distance %f  ",current)
                    if current < min:
                        min = current

                    self.cmd_publisher.publish(twist)
                    
                self.cmd_publisher.publish(Twist())
                rospy.sleep(2)
                error_sum += self.distance_calculator(self.odom_x,self.odom_y,self.goal_x,self.goal_y)
                self.state=self.ROTATE

            if count == 5:
                self.cmd_publisher.publish(Twist())
                error = error_sum/count
                rospy.loginfo("average error: %f" , error)
                rospy.sleep(2)
                break
            
            if self.state == self.ROTATE:
                self.recieve_goal_pose()
                goal_yaw = self.calculate_goal_angle()

                if (goal_yaw - self.odom_yaw <= math.pi and goal_yaw - self.odom_yaw >0) or goal_yaw - self.odom_yaw <= -math.pi:
                    twist = Twist()
                    twist.angular.z = self.angular_speed

                elif goal_yaw - self.odom_yaw == 0:
                    twist = Twist()
                else :
                    twist = Twist()
                    twist.angular.z = -self.angular_speed

                
                while(abs(self.odom_yaw-goal_yaw) > self.epsilon_angular):
                    self.cmd_publisher.publish(twist)

                self.cmd_publisher.publish(Twist())
                rospy.sleep(2)
                self.state=self.GO
            
            


                


if __name__ == "__main__":
    controller = Controller()
    
    controller.run()