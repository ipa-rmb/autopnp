cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_find_ros_package(actionlib_msgs)
include(${actionlib_msgs_PACKAGE_PATH}/cmake/actionbuild.cmake)
genaction()

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})


INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/ros/include
					${PROJECT_SOURCE_DIR}/common/include)
					
#rosbuild_add_executable(send_Goal_nav ros/src/send_Goal_nav.cpp)
#rosbuild_add_executable(Exploration_Algorithm ros/src/Exploration_Algorithm.cpp)
#rosbuild_add_executable(myBWlabel ros/src/myBWlabel.cpp)
#rosbuild_add_executable(tf_data ros/src/tf_data.cpp)
#rosbuild_add_executable(help ros/src/help.cpp)
#rosbuild_add_executable(test_test ros/src/test.cpp)

			#Exploration Algorithm Executable
#rosbuild_add_executable(Exploration_Algorithm_2nd_version ros/src/Exploration_Algorithm_2nd_version.cpp)

			#Exploration Algorithm Library executable
#rosbuild_add_library(exploration_algorithm_action_server_library ros/src/exploration_algorithm_action_server_library.cpp)
#rosbuild_link_boost(exploration_algorithm_action_server_library thread)

			#Analyze Map Server and Client
#rosbuild_add_executable(analyze_map_server ros/src/analyze_map_server.cpp)

#target_link_libraries(analyze_map_server exploration_algorithm_action_server_library)

			#Next Room Server and Client
#rosbuild_add_executable(next_room_server ros/src/next_room_server.cpp)

#target_link_libraries(next_room_server exploration_algorithm_action_server_library)

#rosbuild_add_executable(next_room_client ros/src/next_room_client.cpp)

#rosbuild_link_boost(next_room_client thread)


			#Inspect Room Server and Client
#rosbuild_add_executable(inspect_room_server ros/src/inspect_room_server.cpp)

#target_link_libraries(inspect_room_server exploration_algorithm_action_server_library)

#rosbuild_add_executable(inspect_room_client ros/src/inspect_room_client.cpp)

#rosbuild_link_boost(inspect_room_client thread)


			#to location Server and Client
#rosbuild_add_executable(to_location_server ros/src/to_location_server.cpp)

#target_link_libraries(to_location_server exploration_algorithm_action_server_library)

#rosbuild_add_executable(to_location_client ros/src/to_location_client.cpp)

#rosbuild_link_boost(to_location_client thread)


			#Random Location Server and Client
#rosbuild_add_executable(random_location_server ros/src/random_location_server.cpp)

#target_link_libraries(random_location_server exploration_algorithm_action_server_library)

#rosbuild_add_executable(random_location_client ros/src/random_location_client.cpp)

#rosbuild_link_boost(random_location_client thread)


			#move_base_location_on_perimeter executable
#rosbuild_add_executable(move_base_location_on_perimeter_cpp ros/src/move_base_location_on_perimeter.cpp)
#rosbuild_add_library(move_base_location_on_perimeter ros/src/move_base_location_on_perimeter.cpp)
#rosbuild_link_boost(move_base_location_on_perimeter thread)		

			#Get Obstacle free point server and client executable
#rosbuild_add_executable(get_obstacle_free_point_server ros/src/get_obstacle_free_point_server.cpp)	
#target_link_libraries(get_obstacle_free_point_server move_base_location_on_perimeter)	

#rosbuild_add_executable(get_obstacle_free_point_client ros/src/get_obstacle_free_point_client.cpp)	
#target_link_libraries(get_obstacle_free_point_client move_base_location_on_perimeter)				


			#Move Arm Client
#rosbuild_add_executable(Move_Arm_Client ros/src/Move_Arm_Client.cpp)

			#Inflation Data Test CPP	
#rosbuild_add_executable(Inflation_Data_test ros/src/Inflation_Data_test.cpp)

			#Check Point accessibility
#rosbuild_add_executable(map_point_accessibility_check_server ros/src/map_point_accessibility_check_server.cpp)
#rosbuild_add_executable(map_point_accessibility_check_client ros/src/map_point_accessibility_check_client.cpp)

			#map segmentation action server and client
rosbuild_add_executable(map_segmentation_action_server ros/src/map_segmentation_action_server.cpp)

rosbuild_add_executable(map_segmentation_action_client ros/src/map_segmentation_action_client.cpp)

rosbuild_link_boost(map_segmentation_action_client thread)

			#find next room action server
rosbuild_add_executable(find_next_unprocessed_room_action_server ros/src/find_next_unprocessed_room_action_server.cpp)		

			#go to room location action server
rosbuild_add_executable(go_to_room_location_action_server ros/src/go_to_room_location_action_server.cpp)		

			#Random Location Finder Server
rosbuild_add_executable(random_location_finder_action_server ros/src/random_location_finder_action_server.cpp)	

			#Inspect Room Server			
rosbuild_add_executable(inspect_room_action_server ros/src/inspect_room_action_server.cpp)

			#Trash Bin detection Server			
rosbuild_add_executable(trash_bin_detection_service_server ros/src/trash_bin_detection_service_server.cpp)

			#Activate Trash Bin Detection Client 			
rosbuild_add_executable(activate_trash_bin_detection_client ros/src/activate_trash_bin_detection_client.cpp)

			#Deactivate Trash Bin Detection Client 		
rosbuild_add_executable(deactivate_trash_bin_detection_client ros/src/deactivate_trash_bin_detection_client.cpp)

			#detect fiducials service server		
rosbuild_add_executable(detect_fiducials_service_server ros/src/detect_fiducials_service_server.cpp)

