#! /usr/bin/env python
#################################################################
##\file
## \note
#   Copyright (c) 2013 \n
#   Fraunhofer Institute for Manufacturing Engineering
#   and Automation (IPA) \n\n
#
#################################################################
#
# \note
#   Project name: care-o-bot
# \note
#   ROS stack name: autopnp
# \note
#   ROS package name: autopnp_scenario
#
# \author: Mohammad Muinul Islam(email-> mohammad.islam@ipa.fraunhofer.de)
#
# \author
# Supervised by: Richard Bormann(email:richard.bormann@ipa.fraunhofer.de) 
# 
# \date Date of creation: August 2013
#
# \brief
# 1.The analyze map client has a class name MapSegmentationActionClient which work as a 
# action client to communicate/send the goal definition to map segmentation 
# action server.
# 
# 2.The MapSegmentationActionClient class also provides necessary function for the state machine 
# in exploration smach
#
#################################################################
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#     - Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer. \n
#     - Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution. \n
#     - Neither the name of the Fraunhofer Institute for Manufacturing
#       Engineering and Automation (IPA) nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission. \n
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License LGPL as 
# published by the Free Software Foundation, either version 3 of the 
# License, or (at your option) any later version.
# 
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU Lesser General Public License LGPL for more details.
# 
# You should have received a copy of the GNU Lesser General Public 
# License LGPL along with this program. 
# If not, see <http://www.gnu.org/licenses/>.
#
#################################################################

import roslib; roslib.load_manifest('autopnp_scenario')
import rospy
import cv
import numpy as np
import actionlib
from nav_msgs.msg import OccupancyGrid 
from cv_bridge import CvBridge
import autopnp_scenario.msg

class MapSegmentationActionClient():
    def __init__(self):   
        #Subscribe to the map topic to get the navigation map     
        rospy.Subscriber('/map', OccupancyGrid, self.update_map)        
        self.map_ = None
        self.map_origin_ = None
        self.map_resolution_ = None
        self.map_origin_x_ = None
        self.map_origin_y_ = None
    
    #receive data and creates a cv map for the goal of map segmentation action server          
    def update_map(self, map_msg):        
        self.map_resolution_ = map_msg.info.resolution        
        self.map_origin_x_ = map_msg.info.origin.position.x
        self.map_origin_y_ = map_msg.info.origin.position.y        
        self.map_origin_ = ( map_msg.info.origin.position.x , map_msg.info.origin.position.y )
        #accessible areas are white color and have a value of 255
        self.map_ = 255*np.ones(( map_msg.info.height , map_msg.info.width) , np.uint8 )        
        i = 0
        #obstacles are black color and have a value of 0
        for v in range(0,map_msg.info.height):
            for u in range(0,map_msg.info.width):
                if map_msg.data[i] != 0:
                    self.map_[v][u] = 0
                i += 1                
        
    def map_segmentation_action_client_(self):                      
        mat = cv.fromarray(self.map_)
#         cv.ShowImage( "map_image", mat )
#         cv.WaitKey()   
        #creates a action client object     
        client = actionlib.SimpleActionClient( 'segment_map', autopnp_scenario.msg.MapSegmentationAction )               
        cv_image = CvBridge()
        #filling the goal msg format for map segmentation action server        
        goal = autopnp_scenario.msg.MapSegmentationGoal( input_map = cv_image.cv_to_imgmsg( mat , "mono8"), 
                                                    map_resolution = self.map_resolution_, 
                                                    map_origin_x = self.map_origin_x_ , 
                                                    map_origin_y = self.map_origin_y_,
                                                    return_format_in_pixel = True )        
        rospy.loginfo("waiting for the map segmentation action server to start.....")
        client.wait_for_server()        
        rospy.loginfo("map segmentation action server started, sending goal.....")        
        client.send_goal(goal)        
        finished_before_timeout = client.wait_for_result(rospy.Duration(30.0))        
        if finished_before_timeout:
            state = client.get_state()
            if state is 3:                
                state = 'SUCCEEDED'
                rospy.loginfo("action finished: %s " % state)                
            else:
                rospy.loginfo("action finished: %s " % state)    
        else:
            rospy.loginfo("Action did not finish before the time out.")                
        return client.get_result()


#Only need, if you want to test the map segmentation action server from Client 
if __name__ == '__main__':    
    rospy.init_node('test_segment_map')    
    test = MapSegmentationActionClient()    
    rospy.sleep(10)    
    test.map_segmentation_action_client_()    
    rospy.spin()
