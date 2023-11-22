#!/usr/bin/python3
# -*- coding: utf-8 -*-

################################################################################
# Copyright 2018 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
################################################################################

# Author: Leon Jung, Gilbert, Ashe Kim
 
import rospy
import numpy as np
from std_msgs.msg import Float64, UInt8 
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from turtlebot3_autorace_msgs.msg import MovingParam

class ControlLane():
    def __init__(self):
        self.sub_lane = rospy.Subscriber('/control/lane', Float64, self.cbFollowLane, queue_size = 1)
        self.sub_max_vel = rospy.Subscriber('/control/max_vel', Float64, self.cbGetMaxVel, queue_size = 1)
        self.pub_cmd_vel = rospy.Publisher('/control/cmd_vel', Twist, queue_size = 1)
        self.pub_moving = rospy.Publisher('/control/moving/state', MovingParam, queue_size= 1)
        self.sub_scan_obstacle = rospy.Subscriber('/scan', LaserScan, self.cbScanObstacle, queue_size=1)
        self.sub_moving_completed = rospy.Subscriber('/control/moving/complete', UInt8, self.cbMovingComplete, queue_size = 1)

        self.lastError = 0
        self.MAX_VEL = 0.1
        self.is_obstacle_detected = False
        self.is_moving_complete = False
        self.construction_in_action = False

        rospy.on_shutdown(self.fnShutDown)

    def cbMovingComplete(self, data):
        self.is_moving_complete = True

    def cbGetMaxVel(self, max_vel_msg):
        self.MAX_VEL = max_vel_msg.data

    def cbFollowLane(self, desired_center):
        if self.construction_in_action is False:
            center = desired_center.data

            error = center - 500

            Kp = 0.0025
            Kd = 0.007

            angular_z = Kp * error + Kd * (error - self.lastError)
            self.lastError = error
            
            twist = Twist()
            # twist.linear.x = 0.05        
            twist.linear.x = min(self.MAX_VEL * ((1 - abs(error) / 500) ** 2.2), 0.05)
            twist.linear.y = 0
            twist.linear.z = 0
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = -max(angular_z, -2.0) if angular_z < 0 else -min(angular_z, 2.0)
            self.pub_cmd_vel.publish(twist)

    def construction_action(self):
        self.construction_in_action = True
        rospy.loginfo("avoid obstacle")
        rospy.loginfo("go left")
        msg_moving = MovingParam()
        msg_moving.moving_type=2
        msg_moving.moving_value_angular=90
        msg_moving.moving_value_linear=0.0
        self.pub_moving.publish(msg_moving)
        while True:
            if self.is_moving_complete == True:
                break
        self.is_moving_complete = False

        rospy.sleep(1)

        rospy.loginfo("go straight")
        msg_moving.moving_type= 4
        msg_moving.moving_value_angular=0.0
        msg_moving.moving_value_linear=0.25
        self.pub_moving.publish(msg_moving)
        while True:
            if self.is_moving_complete == True:
                break
        self.is_moving_complete = False

        rospy.sleep(1)

        rospy.loginfo("go right")
        msg_moving.moving_type=3
        msg_moving.moving_value_angular=90
        msg_moving.moving_value_linear=0.0
        self.pub_moving.publish(msg_moving)
        while True:
            if self.is_moving_complete == True:
                break
        self.is_moving_complete = False

        rospy.sleep(1)

        rospy.loginfo("go straight")
        msg_moving.moving_type= 4
        msg_moving.moving_value_angular=0.0
        msg_moving.moving_value_linear=0.5
        self.pub_moving.publish(msg_moving)
        while True:
            if self.is_moving_complete == True:
                break
        self.is_moving_complete = False

        rospy.sleep(1)

        rospy.loginfo("go right")
        msg_moving.moving_type=3
        msg_moving.moving_value_angular=90
        msg_moving.moving_value_linear=0.0
        self.pub_moving.publish(msg_moving)
        while True:
            if self.is_moving_complete == True:
                break
        self.is_moving_complete = False

        rospy.sleep(1)

        rospy.loginfo("go straight")
        msg_moving.moving_type= 4
        msg_moving.moving_value_angular=0.0
        msg_moving.moving_value_linear=0.4
        self.pub_moving.publish(msg_moving)
        while True:
            if self.is_moving_complete == True:
                break
        self.is_moving_complete = False

        rospy.sleep(1)

        rospy.loginfo("go left")
        msg_moving.moving_type=2
        msg_moving.moving_value_angular=90
        msg_moving.moving_value_linear=0.0
        self.pub_moving.publish(msg_moving)
        while True:
            if self.is_moving_complete == True:
                break
        self.is_moving_complete = False

        rospy.sleep(1)

        rospy.loginfo("go straight")
        msg_moving.moving_type=4
        msg_moving.moving_value_angular=0.0
        msg_moving.moving_value_linear=0.4
        self.pub_moving.publish(msg_moving)
        while True:
            if self.is_moving_complete == True:
                break
        self.is_moving_complete = False

        rospy.sleep(1)

        rospy.loginfo("go left")
        msg_moving.moving_type=2
        msg_moving.moving_value_angular=80
        msg_moving.moving_value_linear=0.0
        self.pub_moving.publish(msg_moving)
        while True:
            if self.is_moving_complete == True:
                break
        self.is_moving_complete = False

        rospy.sleep(1)

        rospy.loginfo("go straight")
        msg_moving.moving_type=4
        msg_moving.moving_value_angular=0.0
        msg_moving.moving_value_linear=0.5
        self.pub_moving.publish(msg_moving)
        while True:
            if self.is_moving_complete == True:
                break         
        self.is_moving_complete = False
        self.construction_in_action = False
        rospy.loginfo("construction finished")


    def cbScanObstacle(self, scan):
        angle_scan = 10
        scan_start = 0 - angle_scan
        scan_end = 0 + angle_scan
        threshold_distance = 0.2
        is_obstacle_detected = False

        for i in range(scan_start, scan_end):
            if scan.ranges[i] < threshold_distance and scan.ranges[i] > 0.01:
                is_obstacle_detected = True

        self.is_obstacle_detected = is_obstacle_detected




    def fnShutDown(self):
        rospy.loginfo("Shutting down. cmd_vel will be 0")

        twist = Twist()
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        self.pub_cmd_vel.publish(twist) 

    def main(self):
        while(True):
            if self.is_obstacle_detected and  not self.construction_in_action:
                self.construction_action()
        # rospy.spin()

if __name__ == '__main__':
    rospy.init_node('control_lane')
    node = ControlLane()
    node.main()
