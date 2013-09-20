#!/usr/bin/env python

PKG = 'autopnp_scenario'
NODE = 'set_up_arm_for_cleaning_mode'

import roslib; roslib.load_manifest(PKG)
import rospy

from cob_trajectory_controller.srv import SetFloat

from simple_script_server import simple_script_server
sss = simple_script_server()


def main():
	# init node
	rospy.init_node(NODE)
	print "==> node %s started" %NODE
	rospy.sleep(0.5)

	# set arm parameters
	print "--> setting parameters"
	rospy.set_param('/script_server/arm/wave_in', [[1.5, 0.25, 0.0, -1.0, 0.0, 1.5, 0.0]])
	rospy.set_param('/script_server/arm/wave_out', [[1.5, 0.0, 0.0, -0.75, 0.0, 1.0, 0.0]])
	sss.move("arm", "wave_in")
	#sss.move("arm", "waveout")


if __name__ == "__main__":
	main()

