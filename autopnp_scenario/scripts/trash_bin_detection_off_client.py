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
# \date Date of creation: September 2013
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
from autopnp_scenario.srv import *

if __name__ == "__main__":
    rospy.wait_for_service('deactivate_trash_bin_detection_service')
    try:
        req = rospy.ServiceProxy('deactivate_trash_bin_detection_service', DeactivateTrashBinDetection)
        resp = req()
        for loop_counter in range(len(resp.detected_trash_bin_poses.detections)):
            print'\nseq: ',resp.detected_trash_bin_poses.detections[loop_counter].header.seq 
            print'frame id: ',resp.detected_trash_bin_poses.detections[loop_counter].header.frame_id
            print'sec: ',resp.detected_trash_bin_poses.detections[loop_counter].header.stamp.secs
            print'nsec: ',resp.detected_trash_bin_poses.detections[loop_counter].header.stamp.nsecs
            print'position.x: ',resp.detected_trash_bin_poses.detections[loop_counter].pose.pose.position.x    
            print'position.y: ',resp.detected_trash_bin_poses.detections[loop_counter].pose.pose.position.y  
            print'position.z: ',resp.detected_trash_bin_poses.detections[loop_counter].pose.pose.position.z  
            print'orientation.x: ',resp.detected_trash_bin_poses.detections[loop_counter].pose.pose.orientation.x  
            print'orientation.y: ',resp.detected_trash_bin_poses.detections[loop_counter].pose.pose.orientation.y  
            print'orientation.z: ',resp.detected_trash_bin_poses.detections[loop_counter].pose.pose.orientation.z  
            print'orientation.w: ',resp.detected_trash_bin_poses.detections[loop_counter].pose.pose.orientation.x     
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        