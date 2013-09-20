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
# smach representation of Autopnp project
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

from autopnp_scenario.srv import *
from autopnp_dirt_detection.srv import *

from ApproachPerimeter import *

from simple_script_server import simple_script_server
sss = simple_script_server()


#-------------------------------------------------------- Exploration Algorithm ---------------------------------------------------------------------------------------


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
                                                                                          'find_next_unprocessed_room_number_in_',
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
    
    
#----------------------------------------------------------------------------------------------------------------------------------------------------------------


#-------------------------------------------------------- Move Tool Wagon ---------------------------------------------------------------------------------------
    
        
        
class VerifyToolCarLocation(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['no_need_to_move_it','tool_wagon_needs_to_be_moved'])
             
    def execute(self, userdata ):                               
        rospy.loginfo('Executing state Verify_Tool_Car_Location')                                                
        return 'no_need_to_move_it'   
    
    
    
class MoveBaseToLastToolWaggonLocation(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Move_tool_wagon_1'])
             
    def execute(self, userdata ):                               
        rospy.loginfo('Executing state Move_Base_To_Last_Tool_Waggon_Location')                                                
        return 'Move_tool_wagon_1'    

     

class DetectToolWaggon(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Move_tool_wagon_2'])
             
    def execute(self, userdata ):                               
        rospy.loginfo('Executing state Detect_Tool_Waggon')                                                
        return 'Move_tool_wagon_2'
    
    
    
class GraspHandle(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Move_tool_wagon_3'])
             
    def execute(self, userdata ):                               
        rospy.loginfo('Executing state Grasp_Handle')                                                
        return 'Move_tool_wagon_3' 


class GoToNextToolWaggonLocation(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Move_tool_wagon_4'])
             
    def execute(self, userdata ):                               
        rospy.loginfo('Executing state Go_To_Next_Tool_Waggon_Location')                                                
        return 'Move_tool_wagon_4' 
    
    
    
class ReleaseGrasp(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Move_tool_wagon_5'])
             
    def execute(self, userdata ):                               
        rospy.loginfo('Executing state Release_Grasp')                                                
        return 'Move_tool_wagon_5'     


     
#----------------------------------------------------------------------------------------------------------------------------------------------------------------


#-------------------------------------------------------- Dirt Detection ----------------------------------------------------------------------------------------
        
        
        
class DirtDetectionOn(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['dd_On'])
             
    def execute(self, userdata ):         
#         rospy.sleep(2)                              
        rospy.loginfo('Executing state Dirt_Detection_On')
        rospy.wait_for_service('/dirt_detection/activate_dirt_detection') 
        try:
            req = rospy.ServiceProxy('/dirt_detection/activate_dirt_detection',ActivateDirtDetection)
            resp = req()
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        return 'dd_On' 
    
    
# The TrashBinDetectionOn class defines a state machine of smach which basically 
# use the ActivateTrashBinDetection service to activate Trash Bin Detection
class TrashBinDetectionOn(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['TBD_On'])
             
    def execute(self, userdata ):  
#         rospy.sleep(2)                              
        rospy.loginfo('Executing state Trash_Bin_Detection_On') 
        rospy.wait_for_service('activate_trash_bin_detection_service') 
        try:
            req = rospy.ServiceProxy('activate_trash_bin_detection_service',ActivateTrashBinDetection)
            resp = req()
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e                                               
        return 'TBD_On'               
     
    
    
class DirtDetectionOff(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['dd_Off'])
             
    def execute(self, userdata ):
#         rospy.sleep(2)                                
        rospy.loginfo('Executing state Dirt_Detection_Off')   
        rospy.wait_for_service('/dirt_detection/deactivate_dirt_detection') 
        try:
            req = rospy.ServiceProxy('/dirt_detection/deactivate_dirt_detection',DeactivateDirtDetection)
            resp = req()
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e                                             
        return 'dd_Off'  

  
# The TrashBinDetectionOff class defines a state machine of smach which basically 
# use the DeactivateTrashBinDetection service to deactivate Trash Bin Detection
class TrashBinDetectionOff(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['trash_can_found','trash_can_not_found'],output_keys=['detected_waste_bin_poses_'])
             
    def execute(self, userdata ): 
#         rospy.sleep(2)                               
        rospy.loginfo('Executing state Trash_Bin_Detection_Off')     
        rospy.wait_for_service('deactivate_trash_bin_detection_service') 
        try:
            req = rospy.ServiceProxy('deactivate_trash_bin_detection_service',DeactivateTrashBinDetection)
            resp = req()            
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
                        
        userdata.detected_waste_bin_poses_ = resp.detected_trash_bin_poses.detections
        
        if len(resp.detected_trash_bin_poses.detections)==0:                                    
            return 'trash_can_not_found'   
        else:
            return 'trash_can_found'
    
    
# The GoToNextUnprocessedWasteBin class defines a state machine of smach which basically 
# give the goal position to go to the next unprocessed trash bin location   
class GoToNextUnprocessedWasteBin(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['go_to_trash_location','All_the_trash_bin_is_cleared'],input_keys=['go_to_next_unprocessed_waste_bin_in_',
                                                                                                                'number_of_unprocessed_trash_bin_in_'],
                                                                                                    output_keys= ['go_to_next_unprocessed_waste_bin_out_',
                                                                                                                  'number_of_unprocessed_trash_bin_out_'])
              
    def execute(self, userdata ): 
#         rospy.sleep(2)                               
        rospy.loginfo('Executing state Go_To_Next_Unprocessed_Waste_Bin')
        if (len(userdata.go_to_next_unprocessed_waste_bin_in_)==0 or
            userdata.number_of_unprocessed_trash_bin_in_ == len(userdata.go_to_next_unprocessed_waste_bin_in_)):
            rospy.loginfo('Total Number of Trash Bin: %d',len(userdata.go_to_next_unprocessed_waste_bin_in_))
            return 'All_the_trash_bin_is_cleared'
        else:
            rospy.loginfo('Total Number of Trash Bin: %d',len(userdata.go_to_next_unprocessed_waste_bin_in_))
            rospy.loginfo('Current Trash Bin Number: %d',userdata.number_of_unprocessed_trash_bin_in_)
            userdata.go_to_next_unprocessed_waste_bin_out_ = userdata.go_to_next_unprocessed_waste_bin_in_[userdata.number_of_unprocessed_trash_bin_in_]
            userdata.number_of_unprocessed_trash_bin_out_ = userdata.number_of_unprocessed_trash_bin_in_ + 1
            return 'go_to_trash_location'                    
    
        

#----------------------------------------------------------------------------------------------------------------------------------------------------------------


#-------------------------------------------------------- Clear Waste Bin ---------------------------------------------------------------------------------------


# Here you can use the 'trash_bin_pose_' input key to move the robot 
# to desire trash bin position     
class MoveToTrashBinLocation(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['MTTBL_success'],input_keys=['trash_bin_pose_'],
                             output_keys=['center', 'radius', 'rotational_sampling_step', 'new_computation_flag', 'invalidate_other_poses_radius', 'goal_pose_selection_strategy'])
             
    def execute(self, userdata ):                              
        rospy.loginfo('Executing state Move_To_Trash_Bin_Location') 
        #try:
            #sm = ApproachPerimeter()
        center = Pose2D()
        center.x = userdata.trash_bin_pose_.pose.pose.position.x 
        center.y = userdata.trash_bin_pose_.pose.pose.position.y
        center.theta = 0
        userdata.center = center
        userdata.radius = 0.8
        userdata.rotational_sampling_step = 10.0/180.0*math.pi
        userdata.new_computation_flag = True
        userdata.invalidate_other_poses_radius = 1.0 #in meters, radius the current goal covers
        userdata.goal_pose_selection_strategy = 'closest_to_target_gaze_direction'  #'closest_to_target_gaze_direction', 'closest_to_robot'             
            # introspection -> smach_viewer
#             sis = smach_ros.IntrospectionServer('map_accessibility_analysis_introspection', sm, '/MAP_ACCESSIBILITY_ANALYSIS')             
#             sis.start()
#             sm.execute()
#             sis.stop()
#         except:
#             print('EXCEPTION THROWN')
#             print('Aborting cleanly')
#             os._exit(1)                                              
        return 'MTTBL_success'  
        
    
    
class GraspTrashBin(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['GTB_success'])
             
    def execute(self, userdata ):                             
        rospy.loginfo('Executing state Grasp_Trash_Bin')  
        rospy.loginfo('setting robot head and torso position')
        handle_head = sss.move("head","back",False)
        handle_head.wait()
        handle_torso = sss.move("torso","back",False)
        handle_torso.wait()        
        rospy.wait_for_service('detect_trash_bin_again_service')
        try:
            while 1:
                req = rospy.ServiceProxy('detect_trash_bin_again_service', DetectFiducials)
                resp = req('tag_0')
#                 print'\nseq: ',resp.waste_bin_location.header.seq 
#                 print'frame id: ',resp.waste_bin_location.header.frame_id
#                 print'sec: ',resp.waste_bin_location.header.stamp.secs
#                 print'nsec: ',resp.waste_bin_location.header.stamp.nsecs
#                 print'position.x: ',resp.waste_bin_location.pose.position.x    
#                 print'position.y: ',resp.waste_bin_location.pose.position.y  
#                 print'position.z: ',resp.waste_bin_location.pose.position.z  
#                 print'orientation.x: ',resp.waste_bin_location.pose.orientation.x  
#                 print'orientation.y: ',resp.waste_bin_location.pose.orientation.y  
#                 print'orientation.z: ',resp.waste_bin_location.pose.orientation.z  
#                 print'orientation.w: ',resp.waste_bin_location.pose.orientation.x   
                if resp.waste_bin_location.header.seq != 0:
                    break
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e  
        
        handle_torso = sss.move("torso","home",False)
        handle_torso.wait()                                              
        return 'GTB_success'        
    
    
             
class MoveToToolWagon(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['MTTW_success'])
             
    def execute(self, userdata ):  
#         rospy.sleep(2)                              
        rospy.loginfo('Executing state Move_To_Tool_Wagon')                                                
        return 'MTTW_success'
    
    
    
class ClearTrashBinIntoToolWagon(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['CTBITW_done'])
             
    def execute(self, userdata ):  
#         rospy.sleep(2)                              
        rospy.loginfo('Executing state Clear_Trash_Bin_Into_Tool_Wagon')                                                
        return 'CTBITW_done'   
    
    
    
class MoveToTrashBinPickingLocation(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['MTTBPL_done'])
             
    def execute(self, userdata ):  
#         rospy.sleep(2)                              
        rospy.loginfo('Executing state Move_To_Trash_Bin_Picking_Location')                                                
        return 'MTTBPL_done'
    
    
    
class ReleaseTrashBin(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['RTB_finished'])
             
    def execute(self, userdata ):  
#         rospy.sleep(2)                              
        rospy.loginfo('Executing state Release_Trash_Bin')                                                
        return 'RTB_finished'  
    
 
#----------------------------------------------------------------------------------------------------------------------------------------------------------------
 
 
#--------------------------------------------------------Change Tool Hand --> Vacuum Cleaner---------------------------------------------------------------------
 
    
    
class GoToToolWagonLocation(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['GTTWL_done'])
             
    def execute(self, userdata ):  
#         rospy.sleep(2)                              
        rospy.loginfo('Executing state Go_To_Tool_Wagon_Location')                                                
        return 'GTTWL_done'
    
    
    
class DetectSlotForCurrentTool(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['slot_pose'])
             
    def execute(self, userdata ):  
#         rospy.sleep(2)                              
        rospy.loginfo('Executing state Detect_Slot_For_Current_Tool')                                                
        return 'slot_pose'
    
    
    
class MoveArmToSlot(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['MATS_done'])
             
    def execute(self, userdata ):  
#         rospy.sleep(2)                              
        rospy.loginfo('Executing state Move_Arm_To_Slot')                                                
        return 'MATS_done'
    
    
    
class ReleaseToolChanger(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['RTC_done'])
             
    def execute(self, userdata ):  
#         rospy.sleep(2)                              
        rospy.loginfo('Executing state Release_Tool_Changer')                                                
        return 'RTC_done'
    
    
    
class LiftArm(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['LA_done'])
             
    def execute(self, userdata ):  
#         rospy.sleep(2)                              
        rospy.loginfo('Executing state Lift-Arm')                                                
        return 'LA_done'
    
    
    
class DetectSlotForNewDevice(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['DSFND_slot_pose'])
             
    def execute(self, userdata ):  
#         rospy.sleep(2)                              
        rospy.loginfo('Executing state Detect_Slot_For_New_Device')                                                
        return 'DSFND_slot_pose'
    
    
    
class MoveArmToSlot2(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['MATS2_done'])
             
    def execute(self, userdata ):  
#         rospy.sleep(2)                              
        rospy.loginfo('Executing state Move_Arm_To_Slot_2')                                                
        return 'MATS2_done'
      
    
    
class CloseToolChanger(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['CTC_done'])
             
    def execute(self, userdata ):  
#         rospy.sleep(2)                              
        rospy.loginfo('Executing state Close_Tool_Changer')                                                
        return 'CTC_done'
    
    
    
class MoveArmToStandardLocation(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['MATSL_done'])
             
    def execute(self, userdata ):  
#         rospy.sleep(2)                              
        rospy.loginfo('Executing state Move_Arm_To_Standard_Location')                                                
        return 'MATSL_done'



#----------------------------------------------------------------------------------------------------------------------------------------------------------------


#-------------------------------------------------------- Get Dirt Map ------------------------------------------------------------------------------------------



class GetDirtMap(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['list_of_dirt_location'])
             
    def execute(self, userdata ):  
#         rospy.sleep(2)                              
        rospy.loginfo('Executing state Get_Dirt_Map')   
        rospy.wait_for_service('/dirt_detection/get_dirt_map') 
        try:
            rospy.ServiceProxy('/dirt_detection/get_dirt_map',GetDirtMap)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e                                             
        return 'list_of_dirt_location'
    
    
    
#----------------------------------------------------------------------------------------------------------------------------------------------------------------


#-------------------------------------------------------- Go To Next Unprocessed dirt Location ------------------------------------------------------------------


class SelectNextUnprocssedDirtSpot(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['SNUDS_location','no_dirt_spots_left'])
             
    def execute(self, userdata ):  
#         rospy.sleep(2)                              
        rospy.loginfo('Executing state Select_Next_Unprocssed_Dirt_Spot')                                                
        return 'no_dirt_spots_left'
    
    
    
class Move_Location_Perimeter_60cm(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['MLP60_arrived_dirt_location','MLP60_unsuccessful'])
             
    def execute(self, userdata ):  
#         rospy.sleep(2)                              
        rospy.loginfo('Executing state Move_Location_Perimeter_60cm')                                                
        return 'MLP60_arrived_dirt_location'
    
    
    
#----------------------------------------------------------------------------------------------------------------------------------------------------------------


#-------------------------------------------------------- Clean -------------------------------------------------------------------------------------------------



class Clean(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['clean_done'])
             
    def execute(self, userdata ):  
#         rospy.sleep(2)                              
        rospy.loginfo('Executing state Clean')                                                
        return 'clean_done'
    
    
    
#----------------------------------------------------------------------------------------------------------------------------------------------------------------


#-------------------------------------------------------- Go To Inspect Location --------------------------------------------------------------------------------



class Move_Location_Perimeter_180cm(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['MLP180_arrived_dirt_location'])
             
    def execute(self, userdata ):  
#         rospy.sleep(2)                              
        rospy.loginfo('Executing state Move_Location_Perimeter_180cm')                                                
        return 'MLP180_arrived_dirt_location'
    
    
    
#----------------------------------------------------------------------------------------------------------------------------------------------------------------


#-------------------------------------------------------- Verify Cleaning Success -------------------------------------------------------------------------------



class verifyCleaningProcess(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['VCP_done'])
             
    def execute(self, userdata ):  
#         rospy.sleep(2)                              
        rospy.loginfo('Executing state verify_Cleaning_Process')     
        rospy.wait_for_service('/dirt_detection/validate_cleaning_result') 
        try:
            rospy.ServiceProxy('/dirt_detection/validate_cleaning_result',ValidateCleaningResult)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e                                              
        return 'VCP_done'
    
    
    
#----------------------------------------------------------------------------------------------------------------------------------------------------------------


#-------------------------------------------------------- Process Cleaning Verification Results -----------------------------------------------------------------



class ProcessCleaningVerificationResults(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['PCVR_finish'])
             
    def execute(self, userdata ):  
#         rospy.sleep(2)                              
        rospy.loginfo('Executing state Process_Cleaning_Verification_Results')                                                
        return 'PCVR_finish'
    
    
    
#----------------------------------------------------------------------------------------------------------------------------------------------------------------
 
 
 
def main():
    
    rospy.init_node('exploration_detection_cleaning')

    sm_top_exploration = smach.StateMachine(outcomes=['finish', 'failed'])  
    sm_top_exploration.userdata.sm_trash_bin_counter = 0  

    with sm_top_exploration:
        smach.StateMachine.add('ANALYZE_MAP', AnalyzeMap(),transitions={'list_of_rooms':'UNPROCESSED_ROOM'},
                   remapping={'analyze_map_data_img_':'sm_img'})                           
               
        sm_sub_go_to_next_unproccessed_room = smach.StateMachine(outcomes=['arrived','no_more_rooms_left'],input_keys=[ 'sm_img',
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
        
        sm_sub_go_to_next_unproccessed_room.userdata.sm_counter = 1
        sm_sub_go_to_next_unproccessed_room.userdata.sm_location_counter = 0                                
        
        with sm_sub_go_to_next_unproccessed_room:                 
            smach.StateMachine.add('FIND_NEXT_ROOM', NextUnprocessedRoom(),transitions={'location':'VERIFY_TOOL_CAR_LOCATION','no_rooms':'no_more_rooms_left'},
                   remapping={'find_next_unprocessed_room_data_img_':'sm_img',
                              'find_next_unprocessed_room_number_in':'sm_RoomNo',
                              'find_next_unprocessed_room_number_out_':'sm_RoomNo',
                              'find_next_unprocessed_room_loop_counter_in_':'sm_counter',
                              'find_next_unprocessed_room_loop_counter_out_':'sm_counter'})
            
            smach.StateMachine.add('VERIFY_TOOL_CAR_LOCATION', VerifyToolCarLocation(),transitions={'no_need_to_move_it':'GO_TO_LOCATION','tool_wagon_needs_to_be_moved':'MOVE_TOOL_WAGON'}) 
            
            
            sm_sub_move_tool_wagon = smach.StateMachine(outcomes=['wagon_location'])
                                
            with sm_sub_move_tool_wagon:
                smach.StateMachine.add('MOVE_BASE_TO_LAST_TOOL_WAGGON_LOCATION', MoveBaseToLastToolWaggonLocation() ,transitions={'Move_tool_wagon_1':'DETECT_TOOL_WAGGON'})
                
                smach.StateMachine.add('DETECT_TOOL_WAGGON', DetectToolWaggon() ,transitions={'Move_tool_wagon_2':'GRASP_HANDLE'})
                
                smach.StateMachine.add('GRASP_HANDLE', GraspHandle() ,transitions={'Move_tool_wagon_3':'GO_TO_NEXT_TOOL_WAGGON_LOCATION'})
                
                smach.StateMachine.add('GO_TO_NEXT_TOOL_WAGGON_LOCATION', GoToNextToolWaggonLocation() ,transitions={'Move_tool_wagon_4':'RELEASE_GRASP'})
                
                smach.StateMachine.add('RELEASE_GRASP', ReleaseGrasp() ,transitions={'Move_tool_wagon_5':'wagon_location'})
                
            smach.StateMachine.add('MOVE_TOOL_WAGON', sm_sub_move_tool_wagon ,transitions={'wagon_location':'GO_TO_LOCATION'})
            
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
        
        smach.StateMachine.add('UNPROCESSED_ROOM', sm_sub_go_to_next_unproccessed_room ,transitions={'arrived':'DIRT_DETECTION_ON','no_more_rooms_left':'CHANGE_TOOL_HAND'})
        
        smach.StateMachine.add('DIRT_DETECTION_ON', DirtDetectionOn() ,transitions={'dd_On':'TRASH_BIN_DETECTION_ON'})
        
        smach.StateMachine.add('TRASH_BIN_DETECTION_ON', TrashBinDetectionOn() ,transitions={'TBD_On':'INSPECT_ROOM'})        
        
        smach.StateMachine.add('INSPECT_ROOM', InspectRoom(),transitions={'finished':'DIRT_DETECTION_OFF'},
                   remapping={'inspect_room_data_img_in_':'sm_img',
                              'inspect_room_img_out_':'sm_img',
                              'inspect_room_room_number_':'sm_RoomNo'})
        
        smach.StateMachine.add('DIRT_DETECTION_OFF', DirtDetectionOff() ,transitions={'dd_Off':'TRASH_BIN_DETECTION_OFF'})
        
        smach.StateMachine.add('TRASH_BIN_DETECTION_OFF', TrashBinDetectionOff() ,transitions={'trash_can_found':'GO_TO_NEXT_UNPROCESSED_WASTE_BIN','trash_can_not_found':'UNPROCESSED_ROOM'},
                   remapping={'detected_waste_bin_poses_':'trash_detection_poses'})

        smach.StateMachine.add('GO_TO_NEXT_UNPROCESSED_WASTE_BIN', GoToNextUnprocessedWasteBin() ,transitions={'go_to_trash_location':'CLEAR_WASTE_BIN','All_the_trash_bin_is_cleared':'UNPROCESSED_ROOM'},
           remapping={'go_to_next_unprocessed_waste_bin_in_':'trash_detection_poses',
                      'go_to_next_unprocessed_waste_bin_out_':'detection_pose',
                      'number_of_unprocessed_trash_bin_in_':'sm_trash_bin_counter',
                      'number_of_unprocessed_trash_bin_out_':'sm_trash_bin_counter'})

        
        sm_sub_clear_waste_bin = smach.StateMachine(outcomes=['CWB_done', 'failed'],input_keys=['detection_pose'])
              
        with sm_sub_clear_waste_bin:
            smach.StateMachine.add('MOVE_TO_TRASH_BIN_LOCATION',MoveToTrashBinLocation() ,transitions={'MTTBL_success':'APPROACH_PERIMETER'},
                    remapping = {'trash_bin_pose_':'detection_pose'})
            
            smach.StateMachine.add('APPROACH_PERIMETER',ApproachPerimeter(),transitions={'reached':'GRASP_TRASH_BIN', 'not_reached':'failed', 'failed':'failed'},
                    remapping = {'trash_bin_pose_':'detection_pose'})
            
            smach.StateMachine.add('GRASP_TRASH_BIN',GraspTrashBin() ,transitions={'GTB_success':'MOVE_TO_TOOL_WAGON'})
            
            smach.StateMachine.add('MOVE_TO_TOOL_WAGON',MoveToToolWagon() ,transitions={'MTTW_success':'CLEAR_TRASH_BIN_INTO_TOOL_WAGON'})
            
            smach.StateMachine.add('CLEAR_TRASH_BIN_INTO_TOOL_WAGON',ClearTrashBinIntoToolWagon() ,transitions={'CTBITW_done':'MOVE_TO_TRASH_BIN_PICKING_LOCATION'})
            
            smach.StateMachine.add('MOVE_TO_TRASH_BIN_PICKING_LOCATION',MoveToTrashBinPickingLocation() ,transitions={'MTTBPL_done':'RELEASE_TRASH_BIN'})
            
            smach.StateMachine.add('RELEASE_TRASH_BIN',ReleaseTrashBin() ,transitions={'RTB_finished':'CWB_done'})
        
        smach.StateMachine.add('CLEAR_WASTE_BIN', sm_sub_clear_waste_bin ,transitions={'CWB_done':'GO_TO_NEXT_UNPROCESSED_WASTE_BIN'})
        
        
        
        sm_sub_change_tool_hand = smach.StateMachine(outcomes=['CTH_done'])
        
        with sm_sub_change_tool_hand:
            smach.StateMachine.add('GO_TO_TOOL_WAGON_LOCATION',GoToToolWagonLocation() ,transitions={'GTTWL_done':'DETECT_SLOT_FOR_CURRENT_TOOL'})
            
            smach.StateMachine.add('DETECT_SLOT_FOR_CURRENT_TOOL',DetectSlotForCurrentTool() ,transitions={'slot_pose':'MOVE_ARM_TO_SLOT'})
            
            smach.StateMachine.add('MOVE_ARM_TO_SLOT',MoveArmToSlot() ,transitions={'MATS_done':'RELEASE_TOOL_CHANGER'})
            
            smach.StateMachine.add('RELEASE_TOOL_CHANGER',ReleaseToolChanger() ,transitions={'RTC_done':'LIFT_ARM'})
            
            smach.StateMachine.add('LIFT_ARM',LiftArm() ,transitions={'LA_done':'DETECT_SLOT_FOR_NEW_DEVICE'})
            
            smach.StateMachine.add('DETECT_SLOT_FOR_NEW_DEVICE',DetectSlotForNewDevice() ,transitions={'DSFND_slot_pose':'MOVE_ARM_TO_SLOT'})
            
            smach.StateMachine.add('MOVE_ARM_TO_SLOT_2',MoveArmToSlot2() ,transitions={'MATS2_done':'CLOSE_TOOL_CHANGER'})
            
            smach.StateMachine.add('CLOSE_TOOL_CHANGER',CloseToolChanger() ,transitions={'CTC_done':'MOVE_ARM_TO_STANDARD_LOCATION'})
            
            smach.StateMachine.add('MOVE_ARM_TO_STANDARD_LOCATION',MoveArmToStandardLocation() ,transitions={'MATSL_done':'CTH_done'})
            
        smach.StateMachine.add('CHANGE_TOOL_HAND', sm_sub_change_tool_hand ,transitions={'CTH_done':'GET_DIRT_MAP'})
        
        
        
        smach.StateMachine.add('GET_DIRT_MAP', GetDirtMap() ,transitions={'list_of_dirt_location':'GO_TO_NEXT_UNPROCESSED_DIRT_LOCATION'})
        
        
        
        sm_sub_go_to_next_unprocessed_dirt_location = smach.StateMachine(outcomes=['no_dirt_spots_left','arrived_dirt_location'])
        
        with sm_sub_go_to_next_unprocessed_dirt_location:
            smach.StateMachine.add('SELECT_NEXT_UNPROCESSED_DIRT_SPOT', SelectNextUnprocssedDirtSpot() ,transitions={'SNUDS_location':'MOVE_TO_LOCATION_PERIMETER_60CM','no_dirt_spots_left':'no_dirt_spots_left'})
        
            smach.StateMachine.add('MOVE_TO_LOCATION_PERIMETER_60CM', Move_Location_Perimeter_60cm() ,transitions={'MLP60_arrived_dirt_location':'arrived_dirt_location','MLP60_unsuccessful':'SELECT_NEXT_UNPROCESSED_DIRT_SPOT'})
            
        smach.StateMachine.add('GO_TO_NEXT_UNPROCESSED_DIRT_LOCATION', sm_sub_go_to_next_unprocessed_dirt_location ,transitions={'arrived_dirt_location':'CLEAN','no_dirt_spots_left':'PROCESS_CLEANING_VERIFICATION_RESULTS'})
        
        
        
        smach.StateMachine.add('CLEAN', Clean() ,transitions={'clean_done':'GO_TO_INSPECT_LOCATION'})
        
        
        
        sm_sub_go_to_inspect_location = smach.StateMachine(outcomes=['GTIL_arrived_dirt_location'])
        
        with sm_sub_go_to_inspect_location:
            smach.StateMachine.add('MOVE_TO_LOCATION_PERIMETER_180CM', Move_Location_Perimeter_180cm() ,transitions={'MLP180_arrived_dirt_location':'GTIL_arrived_dirt_location'})
            
        smach.StateMachine.add('GO_TO_INSPECT_LOCATION', sm_sub_go_to_inspect_location ,transitions={'GTIL_arrived_dirt_location':'VERIFY_CLEANING_SUCCESS'})
        
        
        
        smach.StateMachine.add('VERIFY_CLEANING_SUCCESS', verifyCleaningProcess() ,transitions={'VCP_done':'GO_TO_NEXT_UNPROCESSED_DIRT_LOCATION'})
        
        
        
        smach.StateMachine.add('PROCESS_CLEANING_VERIFICATION_RESULTS', ProcessCleaningVerificationResults() ,transitions={'PCVR_finish':'finish'})        
        
        
        
    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('server_name', sm_top_exploration, '/SM_ROOT')
    sis.start()

    # Execute SMACH plan
    outcome = sm_top_exploration.execute()
    
    #rospy.spin()
    
    sis.stop()


if __name__ == '__main__':
    main()
    
    
    
    
    