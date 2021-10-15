#!/usr/bin/env python

import py_trees as pt, py_trees_ros as ptr, rospy
from behaviours_student import *
from reactive_sequence import RSequence

import numpy as np

#####################################################################################################
#                                                                                                   #
#                                           NEW IMPLEMENTATION                                      #
#                                                                                                   #
#####################################################################################################

class BehaviourTree(ptr.trees.BehaviourTree):

	def __init__(self):

		# We can actually import them from the launch file
		linear = 1.0
		angular = -np.pi
		cube_pose = rospy.get_param(rospy.get_name() + '/cube_pose')


		rospy.loginfo("Initialising behaviour tree")


		# Execution nodes

		# Raise head
		exec_moveheadup = movehead('up')
		# exec_moveheadup = moveheadup()
		# Tuck arm (home)
		exec_tuckarm = tuckarm()
		# Check if localized
		exec_checklocalization = checklocalization()
		# Localization
		exec_localization = localization()
		# Check if cube has been picked
		exec_checkpickcube =checkpickcube()
		# Go to pick pose
		exec_movepick = move('pick')
		# Lower head
		exec_moveheaddown = movehead('down')
		# exec_moveheaddown = moveheaddown()
		# Pick cube
		exec_pickcube = pickcube()
		# Check if cube has been placed
		exec_checkplacecube = checkplacecube()
		# Go to place pose
		exec_moveplace = move('place')
		# Place cube
		exec_placecube = placecube()
		# Clear costmap
		exec_clearcostmaps = clearcostmaps()
		
		# Check gripper
		exec_checkgripper = checkgripper()

		# Reinit cube
		exec_reinitcube = reinitcube()

		# Detect the cube after it has been placed
		detect_placed_cube = detectplacedcube('place')

		detect_picked_cube = detectplacedcube('pick')

		# check cube
		exec_check_cube = checkcube()

		exec_cube_state = cubestate()
		
		exec_setcubestate = setcubestate()

		exec_checkdetectedplacecube = checkdetectedplacecube()
		# exec_movearm = movearm()









		# # Control nodes



		# Control nodes
		
		# Place cube
		check_place_seq = pt.composites.Sequence(name = 'check place sequence', children = [exec_checkplacecube, detect_placed_cube])
		place_process_seq = pt.composites.Sequence(name = 'Place Sequence'			, children = [exec_checkdetectedplacecube, exec_moveplace,exec_placecube,exec_moveheaddown ])
		place_sel = pt.composites.Selector(name = 'Place Selector'			, children = [check_place_seq, place_process_seq])
		# Pick cube
		cw_sel = pt.composites.Selector(name="Turn CW", children=[counter(11, "?"), go("Rotate!", 0, -0.5)])
		rein_seq = pt.composites.Sequence(neme = 'Rein cube Sequence', children = [exec_reinitcube,exec_moveheadup,cw_sel,exec_tuckarm, exec_setcubestate])
		detectcube_headup_sel = pt.composites.Selector(name = 'detect cube, move head up selector', children = [detect_picked_cube, exec_moveheadup])
		grip_cube_seq = pt.composites.Sequence(name = 'gripper cube sequence', children = [exec_checkgripper, exec_cube_state])
		cube_sel = pt.composites.Selector(name = 'random!!'				, children = [grip_cube_seq, rein_seq])
		check_pick_grip_cube_seq = pt.composites.Sequence(name = 'Check pick, gripper, cube sequence!', children = [cube_sel, exec_checkpickcube])
		pick_process_seq = pt.composites.Sequence(name = 'Pick Sequence'			, children = [exec_movepick, exec_moveheaddown, exec_pickcube, detectcube_headup_sel])
		pick_sel = pt.composites.Selector(name = 'Pick Selector'			, children = [check_pick_grip_cube_seq, pick_process_seq])

		# Localization
		localization_seq  = pt.composites.Sequence(name = 'Localization Sequence'  , children = [exec_moveheadup, exec_tuckarm, exec_clearcostmaps, exec_localization])
		localization_sel = pt.composites.Selector(name = 'Localization Selector'  , children = [exec_checklocalization, localization_seq])

		# Behaviour tree		
		tree   = RSequence(name = '"Main sequence', children = [localization_sel, pick_sel, place_sel])
		super(BehaviourTree, self).__init__(tree)

		# execute the behaviour tree
		rospy.sleep(5)
		self.setup(timeout=10000)
		while not rospy.is_shutdown(): self.tick_tock(1)




		'''
		# Use this for C grade

		# We can actually import them from the launch file
		linear = 1.0
		angular = -np.pi
		cube_pose = rospy.get_param(rospy.get_name() + '/cube_pose')


		rospy.loginfo("Initialising behaviour tree")

		# lower head
		b0 = movehead("down")

		# Tuck arm
		b1 = tuckarm()

		# b2 = pickcube(cube_pose) # Use this for the E grade
		b2 = pickcube()

		# Rotate robot
		b3 = pt.composites.Selector(
			name="Rotate", 
			children=[counter(12, "Rotated?"), go("Rotate!", 0, angular)]
		)

		# Move to table 2
		b4 = pt.composites.Selector(
			name="Go to table 2 fallback", 
			children=[counter(10, "At table2?"), go("Go to table2!", linear, 0)]
		)

		b5 = placecube()

		tree = RSequence(name="Main sequence", children=[b0, b1, b2, b3, b4, b5])
		super(BehaviourTree, self).__init__(tree)

		# execute the behaviour tree
		rospy.sleep(5)
		self.setup(timeout=10000)
		while not rospy.is_shutdown(): self.tick_tock(1)
		'''
		

if __name__ == "__main__":


	rospy.init_node('main_state_machine')
	try:
		BehaviourTree()
	except rospy.ROSInterruptException:
		pass

	rospy.spin()