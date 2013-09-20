#! /usr/bin/env python

import roslib; roslib.load_manifest('autopnp_scenario')
import rospy
import cv
import numpy as np

import actionlib

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import autopnp_scenario.msg

def random_Location( InputImg , RoomNumber, RoomMinX, RoomMaxX, RoomMinY, RoomMaxY , UnsuccessTime ):  
    
    client = actionlib.SimpleActionClient( 'Find_Random_Location', autopnp_scenario.msg.RandomLocationAction )
    
    rospy.loginfo("Waiting for the Random Location Action server to start.")
    
    client.wait_for_server()    
    
    rospy.loginfo("Random Location Action server started, sending goal.")
       
    goal = autopnp_scenario.msg.RandomLocationGoal( input_img = InputImg,
                                                     room_number = RoomNumber,
                                                     room_min_x = RoomMinX,
                                                     room_max_x = RoomMaxX,
                                                     room_min_y = RoomMinY,
                                                     room_max_y = RoomMaxY,
                                                     Unsuccess_time = UnsuccessTime)
    
    client.send_goal(goal)
    
    finished_before_timeout = client.wait_for_result()
    if finished_before_timeout:
        state = client.get_state()
        if state is 3:
            state = 'SUCCEEDED'
            rospy.loginfo("Random Location Action finished: %s " % state)
        else:
            rospy.loginfo("Random Location Action finished: %s " % state)

    else:
        rospy.loginfo("Random Location Action did not finish before the time out.")
        
    return client.get_result()