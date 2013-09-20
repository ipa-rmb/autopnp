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
# exploration smach is a smach representation of Exploration Algorithm
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
import smach
import smach_ros

from map_segmentation_action_client import MapSegmentationActionClient
from find_next_unprocessed_room_action_client import find_next_unprocessed_room
from go_to_room_location_action_client import go_to_room_location
from random_location_finder_action_client import random_location_finder_client
from inspect_room_action_client import inspect_room

# The AnalyzeMap class defines a state machine of smach which basically 
# call the map segmentation action client object and execute the function
# of action client to communicate with action server
class AnalyzeMap(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['list_of_rooms'], output_keys=['analyze_map_data_img_',
                                                                            'analyze_map_data_map_resolution_',
                                                                            'analyze_map_data_map_origin_x_',
                                                                            'analyze_map_data_map_origin_y_',
                                                                            'analyze_map_data_room_center_x_', 
                                                                            'analyze_map_data_room_center_y_',
                                                                            'analyze_map_data_room_min_x_',
                                                                            'analyze_map_data_room_max_x_',
                                                                            'analyze_map_data_room_min_y_',
                                                                            'analyze_map_data_room_max_y_'])

    def execute(self, userdata):
        
        map_segmentation_action_client_object_ = MapSegmentationActionClient()
        
        rospy.sleep(1)
        rospy.loginfo('Executing state analyze map.....')
                        
        map_segmentation_action_server_result_ = map_segmentation_action_client_object_.map_segmentation_action_client_()   
        
        userdata.analyze_map_data_img_ = map_segmentation_action_server_result_.output_map
        userdata.analyze_map_data_map_resolution_ = map_segmentation_action_server_result_.map_resolution        
        userdata.analyze_map_data_map_origin_x_ = map_segmentation_action_server_result_.map_origin_x
        userdata.analyze_map_data_map_origin_y_ = map_segmentation_action_server_result_.map_origin_y        
        userdata.analyze_map_data_room_center_x_ = map_segmentation_action_server_result_.room_center_x_in_pixel
        userdata.analyze_map_data_room_center_y_ = map_segmentation_action_server_result_.room_center_y_in_pixel
        userdata.analyze_map_data_room_min_x_ = map_segmentation_action_server_result_.room_min_x_in_pixel
        userdata.analyze_map_data_room_max_x_ = map_segmentation_action_server_result_.room_max_x_in_pixel
        userdata.analyze_map_data_room_min_y_ = map_segmentation_action_server_result_.room_min_y_in_pixel        
        userdata.analyze_map_data_room_max_y_ = map_segmentation_action_server_result_.room_max_y_in_pixel
        
        return 'list_of_rooms'  
       
# The NextUnprocessedRoom class defines a state machine of smach which basically 
# call the find next unprocessed room action client object and execute the function
# of action client to communicate with action server
class NextUnprocessedRoom(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['location','no_rooms','arrived'],input_keys=['find_next_unprocessed_room_data_img_',
                                                                                          'analyze_map_data_map_resolution_',
                                                                                          'analyze_map_data_map_origin_x_',
                                                                                          'analyze_map_data_map_origin_y_',
                                                                                          'analyze_map_data_room_center_x_',
                                                                                          'analyze_map_data_room_center_y_',
                                                                                          'find_next_unprocessed_room_number_in',
                                                                                          'find_next_unprocessed_room_loop_counter_in_'],                             
                                                                             output_keys=['find_next_unprocessed_room_number_out_',
                                                                                          'find_next_unprocessed_room_loop_counter_out_',
                                                                                          'find_next_unprocessed_room_center_x_',
                                                                                          'find_next_unprocessed_room_center_y_'])
     
        
    def execute(self, userdata ):                       
        #rospy.sleep(10)
        rospy.loginfo('Executing state next unprocessed room.....')        
        if userdata.find_next_unprocessed_room_loop_counter_in_ <= len(userdata.analyze_map_data_room_center_x_):
            find_next_unprocessed_room_action_server_result_ = find_next_unprocessed_room( userdata.find_next_unprocessed_room_data_img_,
                                                                                           userdata.analyze_map_data_room_center_x_,
                                                                                           userdata.analyze_map_data_room_center_y_,
                                                                                           userdata.analyze_map_data_map_resolution_,
                                                                                           userdata.analyze_map_data_map_origin_x_,
                                                                                           userdata.analyze_map_data_map_origin_y_ )
                    
            #rospy.sleep(10)        
            userdata.find_next_unprocessed_room_number_out_ = find_next_unprocessed_room_action_server_result_.room_number            
            rospy.loginfo('Current room No: %d'%userdata.find_next_unprocessed_room_loop_counter_in_)  
            userdata.find_next_unprocessed_room_loop_counter_out_ = userdata.find_next_unprocessed_room_loop_counter_in_ + 1  
            userdata.find_next_unprocessed_room_center_x_ = find_next_unprocessed_room_action_server_result_.center_position_x
            userdata.find_next_unprocessed_room_center_y_ = find_next_unprocessed_room_action_server_result_.center_position_y                                           
            return 'location'
        else:
            return 'no_rooms'

     
# The GoToRoomLocation class defines a state machine of smach which basically 
# call the go to room location action client object and execute the function
# of action client to communicate with action server     
class GoToRoomLocation(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['successful','unsuccessful'],input_keys=['go_to_room_location_data_img_',
                                                                                     'analyze_map_data_map_resolution_',
                                                                                     'analyze_map_data_map_origin_x_',
                                                                                     'analyze_map_data_map_origin_y_',
                                                                                     'find_next_unprocessed_room_center_x_',
                                                                                     'find_next_unprocessed_room_center_y_',
                                                                                     'random_location_finder_random_location_x_',
                                                                                     'random_location_finder_random_location_y_',
                                                                                     'go_to_room_location_loop_counter_in_'],
                                                                        output_keys=['go_to_room_location_loop_counter_out_'])        
        
    def execute(self, userdata):        
        #rospy.sleep(10)
        rospy.loginfo('Executing state go to room location')                
        if userdata.go_to_room_location_loop_counter_in_ == 0 :
            go_to_room_location_action_server_result_ = go_to_room_location(userdata.go_to_room_location_data_img_,
                                                                            userdata.find_next_unprocessed_room_center_x_ , 
                                                                            userdata.find_next_unprocessed_room_center_y_,
                                                                            userdata.analyze_map_data_map_resolution_,
                                                                            userdata.analyze_map_data_map_origin_x_,
                                                                            userdata.analyze_map_data_map_origin_y_ )
            
        else:
             go_to_room_location_action_server_result_ = go_to_room_location(userdata.go_to_room_location_data_img_,
                                                                             userdata.random_location_finder_random_location_x_,
                                                                             userdata.random_location_finder_random_location_y_,
                                                                             userdata.analyze_map_data_map_resolution_,
                                                                             userdata.analyze_map_data_map_origin_x_,
                                                                             userdata.analyze_map_data_map_origin_y_)
                    
        if go_to_room_location_action_server_result_.output_flag == 'True':        
            return 'successful'        
        else:
            userdata.go_to_room_location_loop_counter_out_ = userdata.go_to_room_location_loop_counter_in_ + 1
            return 'unsuccessful' 
    
    
# The FindRandomLocation class defines a state machine of smach which basically 
# call the random location finder action client object and execute the function
# of action client to communicate with action server        
class FindRandomLocation(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['re_locate','unsuccessful_five_times'],input_keys=['random_location_finder_data_img_in_',
                                                                                               'analyze_map_data_map_resolution_',
                                                                                               'analyze_map_data_map_origin_x_',
                                                                                               'analyze_map_data_map_origin_y_',
                                                                                               'random_location_finder_room_number_',
                                                                                               'analyze_map_data_room_min_x_',
                                                                                               'analyze_map_data_room_max_x_',
                                                                                               'analyze_map_data_room_min_y_',
                                                                                               'analyze_map_data_room_max_y_',
                                                                                               'random_location_finder_counter_in_'],
                                                                                  output_keys=['random_location_finder_random_location_x_',
                                                                                               'random_location_finder_random_location_y_',
                                                                                               'random_location_finder_counter_out_',
                                                                                               'random_location_finder_data_img_out_'])        
        
    def execute(self, userdata):
        
        #rospy.sleep(10)
        rospy.loginfo('Executing state find random location')        
        
        random_location_finder_action_server_result_ = random_location_finder_client(userdata.random_location_finder_data_img_in_,
                                                                                     userdata.analyze_map_data_map_resolution_,
                                                                                     userdata.analyze_map_data_map_origin_x_,
                                                                                     userdata.analyze_map_data_map_origin_y_ ,
                                                                                     userdata.random_location_finder_room_number_,
                                                                                     userdata.analyze_map_data_room_min_x_,
                                                                                     userdata.analyze_map_data_room_max_x_,
                                                                                     userdata.analyze_map_data_room_min_y_,
                                                                                     userdata.analyze_map_data_room_max_y_,
                                                                                     userdata.random_location_finder_counter_in_)
        
        userdata.random_location_finder_random_location_x_ = random_location_finder_action_server_result_.random_location_x
        userdata.random_location_finder_random_location_y_ = random_location_finder_action_server_result_.random_location_y        
        if userdata.random_location_finder_counter_in_ < 6:
            rospy.loginfo("unsuccessful rate: %d",userdata.random_location_finder_counter_in_)
            return 're_locate'        
        else:
            rospy.loginfo("unsuccessful rate: %d",userdata.random_location_finder_counter_in_)
            userdata.random_location_finder_counter_out_ = 0 ;
            userdata.random_location_finder_data_img_out_ = random_location_finder_action_server_result_.output_img
            return 'unsuccessful_five_times'
   

# The InspectRoom class defines a state machine of smach which basically 
# call the inspect room action client object and execute the function
# of action client to communicate with action server   
class InspectRoom(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['finished'],input_keys=['inspect_room_data_img_in_',
                                                                     'inspect_room_room_number_',
                                                                     'analyze_map_data_room_center_x_',
                                                                     'analyze_map_data_room_center_y_',
                                                                     'analyze_map_data_room_min_x_',
                                                                     'analyze_map_data_room_max_x_',
                                                                     'analyze_map_data_room_min_y_',
                                                                     'analyze_map_data_room_max_y_',
                                                                     'analyze_map_data_map_resolution_',
                                                                     'analyze_map_data_map_origin_x_',
                                                                     'analyze_map_data_map_origin_y_'],
                                                        output_keys=['inspect_room_img_out_'])         
        
    def execute(self, userdata):        
        #rospy.sleep(10)
        rospy.loginfo('Executing state inspect_room')        
        inspect_room_action_server_result_= inspect_room( userdata.inspect_room_data_img_in_,
                                                          userdata.inspect_room_room_number_,
                                                          userdata.analyze_map_data_room_center_x_,
                                                          userdata.analyze_map_data_room_center_y_,
                                                          userdata.analyze_map_data_room_min_x_,
                                                          userdata.analyze_map_data_room_max_x_,
                                                          userdata.analyze_map_data_room_min_y_,
                                                          userdata.analyze_map_data_room_max_y_,
                                                          userdata.analyze_map_data_map_resolution_,
                                                          userdata.analyze_map_data_map_origin_x_,
                                                          userdata.analyze_map_data_map_origin_y_)
        #rospy.sleep(10)        
        userdata.inspect_room_img_out_ = inspect_room_action_server_result_.output_img        
        return 'finished'          



def main():
    rospy.init_node('smach_exploration')
    sm_top = smach.StateMachine(outcomes=['finish'])    
    with sm_top:
        smach.StateMachine.add('ANALYZE_MAP', AnalyzeMap(),transitions={'list_of_rooms':'UNPROCESSED_ROOM'},
                   remapping={'analyze_map_data_img_':'sm_img'})               
        sm_sub = smach.StateMachine(outcomes=['arrived','no_more_rooms_left'],input_keys=['sm_img',
                                                                                        'analyze_map_data_map_resolution_',
                                                                                        'analyze_map_data_map_origin_x_',
                                                                                        'analyze_map_data_map_origin_y_',
                                                                                        'analyze_map_data_room_center_x_', 
                                                                                        'analyze_map_data_room_center_y_',
                                                                                        'analyze_map_data_room_min_x_',
                                                                                        'analyze_map_data_room_max_x_',
                                                                                        'analyze_map_data_room_min_y_',
                                                                                        'analyze_map_data_room_max_y_'],
                                                                           output_keys=['sm_RoomNo'])
        sm_sub.userdata.sm_counter = 1
        sm_sub.userdata.sm_location_counter = 0       
        with sm_sub:                 
            smach.StateMachine.add('FIND_NEXT_ROOM', NextUnprocessedRoom(),transitions={'location':'GO_TO_LOCATION','no_rooms':'no_more_rooms_left'},
                   remapping={'find_next_unprocessed_room_data_img_':'sm_img',
                              'find_next_unprocessed_room_number_in':'sm_RoomNo',
                              'find_next_unprocessed_room_number_out_':'sm_RoomNo',
                              'find_next_unprocessed_room_loop_counter_in_':'sm_counter',
                              'find_next_unprocessed_room_loop_counter_out_':'sm_counter'})
            
            smach.StateMachine.add('GO_TO_LOCATION', GoToRoomLocation(),transitions={'successful':'arrived','unsuccessful':'RANDOM_LOCATION_FINDER'},
                   remapping={'go_to_room_location_data_img_':'sm_img',
                              'go_to_room_location_loop_counter_in_':'sm_location_counter',
                              'go_to_room_location_loop_counter_out_':'sm_location_counter'})  
            
            smach.StateMachine.add('RANDOM_LOCATION_FINDER', FindRandomLocation(),transitions={'re_locate':'GO_TO_LOCATION','unsuccessful_five_times':'FIND_NEXT_ROOM'},
                   remapping={'random_location_finder_data_img_in_':'sm_img',
                              'random_location_finder_data_img_out_':'sm_img',
                              'random_location_finder_counter_in_':'sm_location_counter',
                              'random_location_finder_counter_out_':'sm_location_counter',
                              'random_location_finder_room_number_':'sm_RoomNo'}) 
        
        smach.StateMachine.add('UNPROCESSED_ROOM', sm_sub ,transitions={'arrived':'INSPECT_ROOM','no_more_rooms_left':'finish'})      
        
        smach.StateMachine.add('INSPECT_ROOM', InspectRoom(),transitions={'finished':'UNPROCESSED_ROOM'},
                   remapping={'inspect_room_data_img_in_':'sm_img',
                              'inspect_room_img_out_':'sm_img',
                              'inspect_room_room_number_':'sm_RoomNo'})
        
    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('server_name', sm_top, '/SM_ROOT')
    sis.start()
    
    # Execute SMACH plan
    outcome = sm_top.execute()
    
    #rospy.spin()    
    sis.stop()

if __name__ == '__main__':
    main()
    
    
    
    
    