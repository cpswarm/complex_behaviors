#!/usr/bin/env python

import rospy
import smach
import smach_ros
import mavros_msgs
import std_srvs.srv

from cpswarm_msgs.msg import *
from swarmros.msg import *
from uav_mavros_takeoff.msg import *


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

		#  ===================================== SarThreads =====================================
		# Callback for custom outcomes from SarThreads
		def out_cb(outcome_map):
			if outcome_map['AbortEventMonitoring'] == 'invalid':
				rospy.loginfo('Returning missionAbort Event')
				return 'missionAbort'

			return 'aborted'

		# Create a Concurrence container
		sarthreads_concurrence = smach.Concurrence(
			outcomes=['missionAbort', 'aborted'],
			default_outcome='missionAbort',
			child_termination_cb=lambda so: True,
			outcome_cb=out_cb)
	
		# Open the container
		with sarthreads_concurrence:

			# ===================================== SarBehavior =====================================
			# Create a State Machine container
			sarbehavior_sm = smach.StateMachine(
				outcomes=['succeeded', 'preempted', 'aborted'])

			# Open the container
			with sarbehavior_sm:

				# ADD Idle to SarBehavior #
				smach.StateMachine.add('Idle',
					Idle(),
					transitions={'succeeded':'TakeOff'})

				# ADD TakeOff to SarBehavior #
				smach.StateMachine.add('TakeOff',
					smach_ros.SimpleActionState('cmd/takeoff',
						TakeOffAction,
						goal=TakeOffGoal(1.5)),
					transitions={'succeeded':'Coverage'})

				# ADD Coverage to SarBehavior #
				smach.StateMachine.add('Coverage',
					smach_ros.SimpleActionState('uav_coverage',
						CoverageAction,
						result_slots=['target_id', 'target_pose']),
					transitions={'succeeded':'SelectRover'},
					remapping={'target_id':'target_id', 'target_pose':'target_pose'})

				# ADD SelectRover to SarBehavior #
				smach.StateMachine.add('SelectRover',
					smach_ros.SimpleActionState('cmd/task_allocation_auction',
						TaskAllocationAction,
						goal_slots=['task_id', 'task_pose'],
						result_slots=['task_id', 'winner', 'task_pose']),
					transitions={'succeeded':'Tracking', 'aborted':'SelectRover'},
					remapping={'task_id':'target_id', 'task_pose':'target_pose'})

				# ADD Tracking to SarBehavior #
				smach.StateMachine.add('Tracking',
					smach_ros.SimpleActionState('uav_tracking',
						TrackingAction,
						goal_slots=['target']),
					transitions={'succeeded':'Coverage', 'aborted':'LocalCoverage'},
					remapping={'target':'target_id'})
				
				# ADD LocalCoverage to SarBehavior #
				smach.StateMachine.add('LocalCoverage',
					smach_ros.SimpleActionState('uav_local_coverage',
						CoverageAction,
						result_slots=['target_id', 'target_pose']),
					transitions={'aborted':'Coverage', 'succeeded':'SelectRover'},
					remapping={'target_id':'target_id', 'target_pose':'pose'})

			#  ===================================== SarBehavior END =====================================

			# ADD SarBehavior to SarThreads #
			smach.Concurrence.add('SarBehavior', sarbehavior_sm)

			# ADD AbortEventMonitoring to SarThreads #
			smach.Concurrence.add('AbortEventMonitoring',
				smach_ros.MonitorState('bridge/events/mission_abort',
					SimpleEvent,
					cond_cb=lambda ud, msg: False))
		#  ===================================== SarThreads END =====================================

		# ADD SarThreads to TOP state #
		smach.StateMachine.add('SarThreads',
			sarthreads_concurrence,
			transitions={'missionAbort':'MissionAbort'})

		# ===================================== MissionAbort =====================================
		# Create a State Machine container
		missionabort_sm = smach.StateMachine(
			outcomes=['succeeded', 'preempted', 'aborted'])

		# Open the container
		with missionabort_sm:

			# ADD Land to MissionAbort #
			smach.StateMachine.add('Land',
				smach_ros.ServiceState('cmd/land',
					std_srvs.srv.Empty),
				transitions={})
		#  ===================================== MissionAbort END =====================================

		# ADD MissionAbort to TOP state #
		smach.StateMachine.add('MissionAbort',
			missionabort_sm,
			transitions={'succeeded':'SarThreads'})
	
	# Create and start the introspection server (uncomment if needed)
	sis = smach_ros.IntrospectionServer('smach_server', top_sm, '/SM_TOP')
	sis.start()
	
	# Execute SMACH plan
	outcome = top_sm.execute()

	# Wait for ctrl-c to stop the application
	rospy.spin()
	sis.stop()

	
if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
