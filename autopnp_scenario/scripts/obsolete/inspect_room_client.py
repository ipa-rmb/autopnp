#! /usr/bin/env python

import roslib; roslib.load_manifest('autopnp_scenario')
import rospy
import cv
import numpy as np

import actionlib

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import autopnp_scenario.msg

def inspect_Room( InputImg, RoomNumber, RoomCenterX, RoomCenterY, RoomMinX, RoomMaxX, RoomMinY, RoomMaxY , mapResolution , mapOriginX , mapOriginY ):  
    
    client = actionlib.SimpleActionClient( 'Inspect_Room', autopnp_scenario.msg.InspectRoomAction )
    
    rospy.loginfo("Waiting for the Inspect Room Action server to start.")
    
    client.wait_for_server()    
    
    rospy.loginfo("Inspect Room Action server started, sending goal.")
       
    goal = autopnp_scenario.msg.InspectRoomGoal( input_img = InputImg,
                                                  room_number = RoomNumber, 
                                                  room_center_x = RoomCenterX, 
                                                  room_center_y = RoomCenterY,
                                                  room_min_x = RoomMinX,
                                                  room_max_x = RoomMaxX,
                                                  room_min_y = RoomMinY,
                                                  room_max_y = RoomMaxY,
                                                  map_resolution = mapResolution,
                                                  Map_Origin_x = mapOriginX,
                                                  Map_Origin_y = mapOriginY )
    
    client.send_goal(goal)
    
    finished_before_timeout = client.wait_for_result()
    if finished_before_timeout:
        state = client.get_state()
        if state is 3:
            state = 'SUCCEEDED'
            rospy.loginfo("Action finished: %s " % state)
        else:
            rospy.loginfo("Action finished: %s " % state)

    else:
        rospy.loginfo("Action did not finish before the time out.")
        
    return client.get_result()