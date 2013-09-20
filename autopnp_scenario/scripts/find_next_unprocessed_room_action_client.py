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
# 1. The find next unprocessed room action client provides a goal
#    definition to find next unprocessed room action server.
# 
# 2. The find next unprocessed room action server process the goal 
#    definition and find the next unprocessed room.
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
import actionlib
import autopnp_scenario.msg

# The function creates a goal definition for the find next unprocessed room action server    
def find_next_unprocessed_room( input_img, x_coordinate_of_room_center, y_coordinate_of_room_center , map_resolution , map_origin_x , map_origin_y ):      
    client = actionlib.SimpleActionClient( 'find_next_unprocessed_room', autopnp_scenario.msg.FindNextUnprocessedRoomAction )
    rospy.loginfo("Waiting for the find next unprocessed room action server to start......")
    client.wait_for_server()    
    rospy.loginfo("find next unprocessed room action server started, sending goal.....")          
    goal = autopnp_scenario.msg.FindNextUnprocessedRoomGoal( input_map = input_img, 
                                                             room_center_x = x_coordinate_of_room_center, 
                                                             room_center_y = y_coordinate_of_room_center,
                                                             map_resolution = map_resolution,
                                                             map_origin_x = map_origin_x,
                                                             map_origin_y = map_origin_y )
    
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