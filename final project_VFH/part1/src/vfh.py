#!/usr/bin/python3

import tf
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math
import numpy as np

class VFH_Controller():

    def __init__(self):

        rospy.init_node('vfh_node', anonymous=False)



        self.trajectories = [(3.5,4) , (1.5, 1.5) , (5,5.5) , (13,7), (-7,13)]
        self.target_x = self.trajectories[0][0]
        self.target_y = self.trajectories[0][1]
        

        self.angular_speed = 0.1
        self.angular_epsilon = 0.1
        self.max_linear_spead = 0.07
        self.linear_epsilon = 0.05
        self.linear_lenght = 1

        self.sector_size = 5
        self.a = 1
        self.b = 0.27
        self.threshold = 3.2
        self.s_max = 4
        self.angular_error = 0


        self.p_angular = 0.7
        self.i_angular = 0
        self.d_angular = 1.0

        self.epsilon = 0.8
        self.dt = 0.005

        self.odom_yaw = 0
        self.odom_x=0
        self.odom_y=0


        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
        self.laser_subscriber = rospy.Subscriber("/scan" , LaserScan , callback=self.laser_callback)
        self.odom_subscriber = rospy.Subscriber("/odom" , Odometry, callback=self.odom_callback)


    def laser_callback(self, msg : LaserScan):
        self.laser_scan = msg
        self.global_path_planning()



    def odom_callback(self, msg: Odometry):
      
        orientation = msg.pose.pose.orientation
        # convert quaternion to odom
        roll, pitch, yaw = tf.transformations.euler_from_quaternion((
            orientation.x ,orientation.y ,orientation.z ,orientation.w
        )) 
        
        self.odom_yaw = yaw
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y


    
    def smoothing_histogram(self,sectors):

        smoothed = []
        for i in range(self.number_of_sector):
            sum_h = 0
            for j in range(-3,4):
                
                if j == 3 or j == -3:
                    jj = 1
                elif j == 2 or j == -2:
                    jj =2
                elif j == 1 or j == -1:
                    jj = 3
                else:
                    jj = 3
                
                if i+j >= self.number_of_sector:
                    j = j * -1
                    i  = self.number_of_sector - i - 1

                sum_h += sectors[i+j] * jj

            sum_h = sum_h/15
            smoothed.append(sum_h)
        
        return smoothed

    def thresholding(self ,sectors):

        thresholded = []
        for i in range(self.number_of_sector):
            
            if sectors[i] < self.threshold and (0 <= i <= 18 or 54 <= i <= 72 ):
                thresholded.append(i)
        
        return thresholded



    def find_target_sector(self):

        yaw = self.odom_yaw
        current_x = self.odom_x
        current_y = self.odom_y
        angle = math.atan2(self.target_x - current_y, self.target_y - current_x)

        if angle < 0:
            angle += 2 * math.pi

        dif = angle - yaw

        if dif < 0:
            dif += 2 * math.pi
        
        target_index = int(math.degrees(dif) / self.sector_size) 

        return target_index % self.number_of_sector
        

    def calculate_Histogram(self):
       histogram = []

       self.number_of_sector = int(len(self.laser_scan.ranges)/self.sector_size)
       
       for i in range(self.number_of_sector):
           
           tmp_histogram = 0
           for j in range(i*self.sector_size,(i+1)*self.sector_size):
               magnitude = self.a - self.b * min(3.6,self.laser_scan.ranges[j])
               tmp_histogram += magnitude
           
           histogram.append(tmp_histogram)
    
       
       return self.smoothing_histogram(histogram)
    #    return histogram


    def vallye_clstering(self ,selected_sectors):

        valleys = []
        tmp_valley=[]
        # print(selected_sectors)
        for i in range(len(selected_sectors)):

            j = i - 1

            if i == 0 :
                tmp_valley.append(selected_sectors[i])
                continue
            
            if selected_sectors[i] - selected_sectors[j] > 1:
                valleys.append(tmp_valley)
                tmp_valley = []

            tmp_valley.append(selected_sectors[i])

        valleys.append(tmp_valley)
        tmp_valley = []

        if valleys[0][0] == 0 and  valleys[-1][-1] == (self.number_of_sector -1):
            tmp_valley = valleys.pop(0)
            for i in tmp_valley:
                valleys[-1].append(i)
            

        print(valleys)
        return valleys

    def select_valley(self,selected_sectors,target_sector):

        my_min= 999
        my_index = 0
        # my_index2 = 0

        if target_sector in selected_sectors:
            return target_sector
        
        valleys = self.vallye_clstering(selected_sectors)
        # print(valleys)

        
        for i in range(len(valleys)):

            for j in range(len(valleys[i])):
                
                distances = abs(valleys[i][j]  - target_sector)

                if distances > 36:
                    distances = 72 - distances

                # print(str(valleys[i][j]) +' '+str(distances))

                if distances < my_min:
                    my_min = distances
                    my_index = i
                    
                    # my_index2 = j

        
        closest_valley = valleys[my_index]
        # print(valleys[my_index][my_index2])
        
        if len(closest_valley) <= self.s_max:
            return closest_valley[int(len(closest_valley)/2)]
        else :
            return closest_valley[my_index+int(self.s_max/2)]
        

    def linear_error(self):
        dis = (abs(self.odom_x-self.target_x)**2 + abs(self.odom_y-self.target_y)**2)**0.5

        return dis    
    

    def controller(self):
        prev_angular_error = 0
        for trajectory in self.trajectories:
            self.target_x = trajectory[0]
            self.target_y = trajectory[1]
            while True:
                angular_error = self.angular_error 
                linear_error = self.linear_error()

                if linear_error < self.epsilon:
                    break

                twist = Twist()
                if angular_error < math.pi/2 and angular_error > -math.pi/2:  # if the goal is behind the robot, the robot shuold not move forward.
                    magnitude = math.pi/2 - abs(angular_error)
                    twist.linear.x = magnitude * self.max_linear_spead
                else:
                    twist.linear.x = 0

                # PID for angular speed
                P = self.p_angular * angular_error
                D = self.d_angular * (angular_error - prev_angular_error) 
                twist.angular.z =  P + D


                prev_angular_error = angular_error

                self.cmd_vel.publish(twist)
                rospy.sleep(self.dt)
        


    def global_path_planning(self):

        sectors = self.calculate_Histogram()
        
        print(sectors[-5:])
        print(sectors[:5])
        
        target_sector = self.find_target_sector()
        selected_sectors = self.thresholding(sectors)

        if sectors[target_sector] < self.threshold:
            best_sector = target_sector
        else:
            best_sector = self.select_valley(selected_sectors, target_sector)

        if best_sector > 36:
            best_sector -= 72

        print(best_sector)
        self.angular_error = math.radians(best_sector * 5)


        



if __name__ == '__main__':
       
    vfh = VFH_Controller()

    while not rospy.is_shutdown():

        vfh.controller()
        # rospy.sleep(10)


    
    