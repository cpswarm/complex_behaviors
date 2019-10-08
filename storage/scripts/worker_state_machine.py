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

		#  ===================================== WorkerThreads =====================================
		# Callback for custom outcomes from WorkerThreads
		def out_cb(outcome_map):
			if outcome_map['AbortEventMonitoring'] == 'invalid':
				rospy.loginfo('Returning missionAbort Event')
				return 'missionAbort'

			return 'aborted'

		# Create a Concurrence container
		workerthreads_concurrence = smach.Concurrence(
			outcomes=['missionAbort', 'aborted'],
			default_outcome='missionAbort',
			child_termination_cb=lambda so: True,
			outcome_cb=out_cb)
	
		# Open the container
		with workerthreads_concurrence:

			# ===================================== ScoutBehavior =====================================
			# Create a State Machine container
			workerbehavior_sm = smach.StateMachine(
				outcomes=['succeeded', 'preempted', 'aborted'])

			# Open the container
			with workerbehavior_sm:

				# ADD Idle to ScoutBehavior #
				smach.StateMachine.add('Idle',
					Idle(),
					transitions={'succeeded':'Coverage'})

				# ADD Coverage to ScoutBehavior #
				smach.StateMachine.add('Coverage',
					smach_ros.SimpleActionState('ugv_coverage',
						CoverageAction,
						result_slots=['target_id', 'target_pose']),
					transitions={'succeeded':'AssignBox'},
					remapping={'target_id':'target_id', 'target_pose':'target_pose'})

				# ADD Task Allocation to ScoutBehavior #
				smach.StateMachine.add('AssignBox',
					smach_ros.SimpleActionState('cmd/task_allocation_auction',
						AssignBoxAction,
						goal_slots=['task_id', 'task_pose'],
						result_slots=['task_id', 'winner', 'task_pose']),
					transitions={'succeeded':'Coverage'},
					remapping={'task_id':'target_id', 'task_pose':'target_pose'})

			#  ===================================== ScoutBehavior END =====================================

			# ADD ScoutBehavior to WorkerThreads #
			smach.Concurrence.add('ScoutBehavior', workerbehavior_sm)

			# ADD AbortEventMonitoring to WorkerThreads #
			smach.Concurrence.add('AbortEventMonitoring',
				smach_ros.MonitorState('bridge/events/mission_abort',
					SimpleEvent,
					cond_cb=lambda ud, msg: False))
		#  ===================================== WorkerThreads END =====================================

		# ADD WorkerThreads to TOP state #
		smach.StateMachine.add('WorkerThreads',
			workerthreads_concurrence,
			transitions={'missionAbort':'MissionAbort'})

		# ===================================== MissionAbort =====================================
		# Create a State Machine container
		missionabort_sm = smach.StateMachine(
			outcomes=['succeeded', 'preempted', 'aborted'])
		#  ===================================== MissionAbort END =====================================

		# ADD MissionAbort to TOP state #
		smach.StateMachine.add('MissionAbort',
			missionabort_sm,
			transitions={'succeeded':'WorkerThreads'})
	
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
