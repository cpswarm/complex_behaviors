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

		#  ===================================== StorageThreads =====================================
		# Callback for custom outcomes from StorageThreads
		def out_cb(outcome_map):
			if outcome_map['AbortEventMonitoring'] == 'invalid':
				rospy.loginfo('Returning missionAbort Event')
				return 'missionAbort'

			return 'aborted'

		# Create a Concurrence container
		storagethreads_concurrence = smach.Concurrence(
			outcomes=['missionAbort', 'aborted'],
			default_outcome='missionAbort',
			child_termination_cb=lambda so: True,
			outcome_cb=out_cb)
	
		# Open the container
		with storagethreads_concurrence:

			# ===================================== StorageBehavior =====================================
			# Create a State Machine container
			storagebehavior_sm = smach.StateMachine(
				outcomes=['succeeded', 'preempted', 'aborted'])

			# Open the container
			with storagebehavior_sm:

				# ADD Idle to StorageBehavior #
				smach.StateMachine.add('Idle',
					Idle(),
					transitions={'succeeded':'Coverage'})

				# ADD Coverage to StorageBehavior #
				smach.StateMachine.add('Coverage',
					smach_ros.SimpleActionState('ugv_coverage',
						CoverageAction,
						result_slots=['target_id', 'target_pose']),
					transitions={'succeeded':'Idle'},
					remapping={'target_id':'target_id', 'target_pose':'target_pose'})

			#  ===================================== StorageBehavior END =====================================

			# ADD StorageBehavior to StorageThreads #
			smach.Concurrence.add('StorageBehavior', storagebehavior_sm)

			# ADD AbortEventMonitoring to StorageThreads #
			smach.Concurrence.add('AbortEventMonitoring',
				smach_ros.MonitorState('bridge/events/mission_abort',
					SimpleEvent,
					cond_cb=lambda ud, msg: False))
		#  ===================================== StorageThreads END =====================================

		# ADD StorageThreads to TOP state #
		smach.StateMachine.add('StorageThreads',
			storagethreads_concurrence,
			transitions={'missionAbort':'MissionAbort'})

		# ===================================== MissionAbort =====================================
		# Create a State Machine container
		missionabort_sm = smach.StateMachine(
			outcomes=['succeeded', 'preempted', 'aborted'])
		#  ===================================== MissionAbort END =====================================

		# ADD MissionAbort to TOP state #
		smach.StateMachine.add('MissionAbort',
			missionabort_sm,
			transitions={'succeeded':'StorageThreads'})
	
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
