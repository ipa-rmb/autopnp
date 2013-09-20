#! /usr/bin/env python

import roslib; roslib.load_manifest('autopnp_scenario')
import rospy

import actionlib
import autopnp_scenario.msg

def to_goal( InputImg, CenterX, CenterY , mapResolution , mapOriginX , mapOriginY ):  
    
    client = actionlib.SimpleActionClient( 'go_to_location', autopnp_scenario.msg.ToLocationAction )
    
    rospy.loginfo("Waiting for the Go To Goal Action server to start.")
    
    client.wait_for_server()    
    
    rospy.loginfo("Go To Goal Action server started, sending goal.")
       
    goal = autopnp_scenario.msg.ToLocationGoal(input_img = InputImg,
                                                CenterPositionX = CenterX,
                                                CenterPositionY = CenterY, 
                                                map_resolution = mapResolution, 
                                                Map_Origin_x = mapOriginX, 
                                                Map_Origin_y = mapOriginY )
    
    client.send_goal(goal)
    
    finished_before_timeout = client.wait_for_result()
    if finished_before_timeout:
        state = client.get_state()
        if state is 3:
            state = 'SUCCEEDED'
            rospy.loginfo("Go To Goal Action finished: %s " % state)
        else:
            rospy.loginfo(" Go To GoalAction finished: %s " % state)

    else:
        rospy.loginfo("Go To Goal Action did not finish before the time out.")
        
    return client.get_result()