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
# 1. The random location finder action client provides a goal
#    definition to random location finder action server.
# 
# 2. The random location finder action server process the goal 
#    definition and find the next random goal position to move the 
#    robot inside of the room.
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

def random_location_finder_client( input_map_ , image_resolution_ , image_origin_x_ , image_origin_y_ , room_number_, room_min_x_, room_max_x_, room_min_y_, room_max_y_ , unsuccess_times_ ):     
    client = actionlib.SimpleActionClient( 'random_location_finder', autopnp_scenario.msg.RandomLocationFinderAction )    
    rospy.loginfo("waiting for the random location finder action server to start.....")    
    client.wait_for_server()        
    rospy.loginfo("random location finder action server started, sending goal.....")       
    goal = autopnp_scenario.msg.RandomLocationFinderGoal(input_img = input_map_,
                                                         map_resolution = image_resolution_,
                                                         map_origin_x = image_origin_x_,
                                                         map_origin_y = image_origin_y_, 
                                                         room_number = room_number_,
                                                         room_min_x = room_min_x_,
                                                         room_max_x = room_max_x_,
                                                         room_min_y = room_min_y_,
                                                         room_max_y = room_max_y_,
                                                         unsuccess_counter = unsuccess_times_)
    
    client.send_goal(goal)    
    finished_before_timeout = client.wait_for_result()
    if finished_before_timeout:
        state = client.get_state()
        if state is 3:
            state = 'SUCCEEDED'
            rospy.loginfo("random location finder action finished: %s " % state)
        else:
            rospy.loginfo("random location finder action finished: %s " % state)
    else:
        rospy.loginfo("random location finder action did not finish before the time out.")
    return client.get_result()

