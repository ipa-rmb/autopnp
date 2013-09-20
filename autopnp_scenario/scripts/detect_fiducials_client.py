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

NODE = 'detect_fiducials_service_client'
import roslib; roslib.load_manifest('autopnp_scenario')
import rospy
from autopnp_scenario.srv import *
from simple_script_server import simple_script_server
sss = simple_script_server()

if __name__ == "__main__":
    # init node
    rospy.init_node(NODE)
    print "==> node %s started" %NODE
    rospy.sleep(0.5)

    # set arm parameters
    print "--> setting robot head and torso position"
    handle_head = sss.move("head","back",False)
    handle_head.wait()
    handle_torso = sss.move("torso","back",False)
    handle_torso.wait()
    
    rospy.wait_for_service('detect_trash_bin_again_service')
    try:
        while 1:
            req = rospy.ServiceProxy('detect_trash_bin_again_service', DetectFiducials)
            resp = req('tag_0')
            print'\nseq: ',resp.waste_bin_location.header.seq 
            print'frame id: ',resp.waste_bin_location.header.frame_id
            print'sec: ',resp.waste_bin_location.header.stamp.secs
            print'nsec: ',resp.waste_bin_location.header.stamp.nsecs
            print'position.x: ',resp.waste_bin_location.pose.position.x    
            print'position.y: ',resp.waste_bin_location.pose.position.y  
            print'position.z: ',resp.waste_bin_location.pose.position.z  
            print'orientation.x: ',resp.waste_bin_location.pose.orientation.x  
            print'orientation.y: ',resp.waste_bin_location.pose.orientation.y  
            print'orientation.z: ',resp.waste_bin_location.pose.orientation.z  
            print'orientation.w: ',resp.waste_bin_location.pose.orientation.x   
            if resp.waste_bin_location.pose.position.x != 0:
                break
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e