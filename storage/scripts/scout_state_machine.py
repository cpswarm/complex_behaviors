#!/usr/bin/env python

import rospy
import smach
import smach_ros
import mavros_msgs
import std_srvs.srv

from cpswarm_msgs.msg import *
from swarmros.msg import *


# define state Idle
class Idle(smach.State):

	def __init__(self):
		smach.State.__init__(self, outcomes=['succeeded'])

	def execute(self, userdata):
		rospy.loginfo('Executing state Idle')
		rospy.sleep(10.0)
		return 'succeeded'


def main():
	rospy.init_node('state_machine_node')
	
	# Create a TOP level SMACH state machine
	top_sm = smach.StateMachine(['succeeded', 'preempted', 'aborted'])
	
	# Open the container
	with top_sm:

		#  ===================================== ScoutThreads =====================================
		# Callback for custom outcomes from ScoutThreads
		def out_cb(outcome_map):
			if outcome_map['AbortEventMonitoring'] == 'invalid':
				rospy.loginfo('Returning missionAbort Event')
				return 'missionAbort'

			return 'aborted'

		# Create a Concurrence container
		scoutthreads_concurrence = smach.Concurrence(
			outcomes=['missionAbort', 'aborted'],
			default_outcome='missionAbort',
			child_termination_cb=lambda so: True,
			outcome_cb=out_cb)
	
		# Open the container
		with scoutthreads_concurrence:

			# ===================================== ScoutBehavior =====================================
			# Create a State Machine container
			scoutbehavior_sm = smach.StateMachine(
				outcomes=['succeeded', 'preempted', 'aborted'])

			# Open the container
			with scoutbehavior_sm:

				# ADD Idle to ScoutBehavior #
				smach.StateMachine.add('Idle',
					Idle(),
					transitions={'succeeded':'Search'})

				# ADD Search to ScoutBehavior #
				smach.StateMachine.add('Search',
					smach_ros.SimpleActionState('ugv_coverage',
						SearchAction,
						result_slots=['target_id', 'target_pose']),
					transitions={'succeeded':'AssignBox'},
					remapping={'target_id':'box_id', 'target_pose':'box_position'})

				# ADD Task Allocation to ScoutBehavior #
				smach.StateMachine.add('AssignBox',
					smach_ros.SimpleActionState('cmd/task_allocation_auction',
						AssignBoxAction,
						goal_slots=['task_id', 'task_pose'],
						result_slots=['task_id', 'winner', 'task_pose']),
					transitions={'succeeded':'Search', 'aborted':'AssignBox'},
					remapping={'task_id':'box_id', 'task_pose':'box_position'})

			#  ===================================== ScoutBehavior END =====================================

			# ADD ScoutBehavior to ScoutThreads #
			smach.Concurrence.add('ScoutBehavior', scoutbehavior_sm)

			# ADD AbortEventMonitoring to ScoutThreads #
			smach.Concurrence.add('AbortEventMonitoring',
				smach_ros.MonitorState('bridge/events/mission_abort',
					SimpleEvent,
					cond_cb=lambda ud, msg: False))
		#  ===================================== ScoutThreads END =====================================

		# ADD ScoutThreads to TOP state #
		smach.StateMachine.add('ScoutThreads',
			scoutthreads_concurrence,
			transitions={'missionAbort':'MissionAbort'})

		# ===================================== MissionAbort =====================================
		# Create a State Machine container
		missionabort_sm = smach.StateMachine(
			outcomes=['succeeded', 'preempted', 'aborted'])
		#  ===================================== MissionAbort END =====================================

		# ADD MissionAbort to TOP state #
		smach.StateMachine.add('MissionAbort',
			missionabort_sm,
			transitions={'succeeded':'ScoutThreads'})
	
	# Create and start the introspection server (uncomment if needed)
	# sis = smach_ros.IntrospectionServer('smach_server', top_sm, '/SM_TOP')
	# sis.start()
	
	# Execute SMACH plan
	outcome = top_sm.execute()

	# Wait for ctrl-c to stop the application
	rospy.spin()
	# sis.stop()

	
if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
