#!/usr/bin/python3

import rospy
import random

from part1.srv import GetNextDestination, GetNextDestinationResponse
def gnd_handler(req):
    current_x = req.current_x
    current_y = req.current_y
    next_x = random.uniform(-10,10)
    y_bound = (25 - (abs(next_x-current_x))**2)**0.5 
    if abs(next_x-current_x) > 5:
        y_bound=0
    y_upper = current_y + y_bound
    y_lower = current_y - y_bound
    if y_lower < -10 :
        next_y = random.uniform(y_upper , 10)
    elif y_upper > 10:
        next_y = random.uniform(-10 , y_lower)
    else:
        next_y = random.choice([ random.uniform(-10 , y_lower) , random.uniform(y_upper , 10)])

    response = GetNextDestinationResponse()
    response.next_x = next_x
    response.next_y = next_y
    return response




def listener():
    rospy.init_node('mission_node', anonymous=True)
    s = rospy.Service('/GetNextDestination', GetNextDestination, gnd_handler)
    rospy.spin()

if __name__ == "__main__":
    listener()