#!/usr/bin/python3

import rospy
from math import radians ,pi
from part2.msg import ClosestObstacle
from sensor_msgs.msg import LaserScan


class Sensor:
    
    def __init__(self):
        rospy.init_node("sensor_node", anonymous=True)
        self.pub = rospy.Publisher("/sensor",ClosestObstacle,queue_size=10)
        self.sub = rospy.Subscriber("/scan" , LaserScan , callback=self.laser_callback)

    def laser_callback(self, msg:LaserScan):
        min_distance = 999.0
        min_degree = 0
        for i in range(360):
            if msg.ranges[i] < min_distance:
                min_degree = i
                min_distance = msg.ranges[i]
            
        r = radians(min_degree)
        if r > pi:
            r = r - 2*pi
        
        sensor_msg = ClosestObstacle()
        sensor_msg.distance = min_distance
        sensor_msg.direction = r
        self.pub.publish(sensor_msg)
        
        

        
    def run(self):
        rospy.spin()

if __name__ =="__main__":
    sensor = Sensor()
    sensor.run()