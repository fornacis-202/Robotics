#!/usr/bin/python3

import rospy
import tf
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import numpy as np
from part1.srv import GetNextDestination

class Controller:
    
    def __init__(self) -> None:
        
        rospy.init_node("controller" , anonymous=False)
        
        self.odom_subscriber = rospy.Subscriber("/odom" , Odometry , callback=self.odom_callback)
        self.cmd_publisher = rospy.Publisher('/cmd_vel' , Twist , queue_size=2)
        
        # getting specified parameters
        self.p_linear = 0.02
        self.i_linear = 0.005
        self.d_linear = 0.1

        self.p_angular = 8.0
        self.i_angular = 0
        self.d_angular = 4.0

        self.epsilon = 0.1
        self.dt = 0.005

        self.odom_yaw = 0
        self.odom_x=0
        self.odom_y=0
        self.goal_x=10
        self.goal_y=0


    def shape_rectangle(self):
        X1 = np.linspace(-3, 3 , 100)
        Y1 = np.array([2]*100)

        Y2 = np.linspace(2, -2 , 100)
        X2 = np.array([3]*100)

        X3 = np.linspace(3, -3 , 100)
        Y3 = np.array([-2]*100)

        Y4 = np.linspace(-2, 2 , 100)
        X4 = np.array([-3]*100)

        self.X = np.concatenate([X1,X2,X3,X4])
        self.Y = np.concatenate([Y1,Y2,Y3,Y4])


    def shape_star(self):
        X1 = np.linspace(0, 3 , 100)
        Y1 = - (7/3) * X1  + 12

        X2 = np.linspace(3, 10 , 100)
        Y2 = np.array([5]*100)

        X3 = np.linspace(10, 4 , 100)
        Y3 = (5/6) * X3  - (10/3)

        X4 = np.linspace(4, 7 , 100)
        Y4 = -(3) * X4  + 12

        X5 = np.linspace(7, 0 , 100)
        Y5 = -(4/7) * X5  - 5

        X6 = np.linspace(0, -7 , 100)
        Y6 = (4/7) * X6  - 5

        X7 = np.linspace(-7, -4 , 100)
        Y7 = 3 * X7  + 12

        X8 = np.linspace(-4, -10 , 100)
        Y8 = -(5/6) * X8  - (10/3)

        X9 = np.linspace(-10, -3 , 100)
        Y9 = np.array([5]*100)

        X10 = np.linspace(-3, 0 , 100)
        Y10 = (7/3) * X10  + 12

        self.X = np.concatenate([X1,X2,X3,X4,X5, X6, X7, X8, X9, X10])
        self.Y = np.concatenate([Y1,Y2,Y3,Y4,Y5, Y6, Y7, Y8, Y9, Y10])



    def shape_spiral(self):
        a = 0.17
        k = math.tan(a)
        self.X , self.Y = [] , []

        for i in range(150):
            t = i / 20 * math.pi
            dx = a * math.exp(k * t) * math.cos(t)
            dy = a * math.exp(k * t) * math.sin(t)
            self.X.append(dx)
            self.Y.append(dy) 



    def set_closest_as_goal(self):
        min = 1000
        min_x = 0
        min_y = 0
        min_i=0

        for i in range(len(self.X)):
            distance = self.calculate_distance(self.odom_x, self.odom_y, self.X[i], self.Y[i])
            if distance < min:
                min = distance
                min_x = self.X[i]
                min_y = self.Y[i]
                min_i = i

        self.goal_x = min_x
        self.goal_y = min_y
        self.first_index = min_i
        self.current_index = min_i



    def set_next_goal(self):
        self.current_index = (self.current_index + 1) % len(self.X)
        if self.current_index == self.first_index:
            return False
        
        self.goal_x = self.X[self.current_index]
        self.goal_y = self.Y[self.current_index]
        return True



    def calculate_distance(self, x1 ,y1 ,x2 ,y2):
        dis = (abs(x1-x2)**2 + abs(y1-y2)**2)**0.5
        return dis

    
   
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
        rospy.sleep(1)




    def control_tward_goal_pose(self):

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
            twist = Twist()
            if angular_error < math.pi/2 and angular_error > -math.pi/2:  # if the goal is behind the robot, the robot shuold not move forward.
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
        self.shape_spiral()
        self.set_closest_as_goal()
        self.control_tward_goal_pose()  
        # changing control parametrs when moving across short distances
        self.p_linear = 0.5
        self.i_linear = 0.1
        self.d_linear = 0.2
        while self.set_next_goal() is True:
            self.control_tward_goal_pose()  
        self.stop()

            
            


                


if __name__ == "__main__":
    controller = Controller()
    
    controller.run()