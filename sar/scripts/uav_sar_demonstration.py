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
	    smach.State.__init__(self, outcomes=['succeeded', 'preempted'])

	def execute(self, userdata):
		rospy.loginfo('Executing state Idle')
		while True:
			# Check for preempt
			if self.preempt_requested():
				rospy.loginfo("Idle state has been preempted")
				self.service_preempt()
				return 'preempted'
			rospy.sleep(1.0)
			
		return 'succeeded'


def main():
	rospy.init_node('state_machine_node')
	
	if not rospy.has_param('~altitude'):
		rospy.logerr('Altitude not specified, cannot perform simulation!')
		return

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

				#  ===================================== IdleThreads =====================================
				# Callback for custom outcomes from IdleThreads
				def out_cb(outcome_map):
					if outcome_map['IdleEventMonitoring'] == 'invalid':
						rospy.loginfo('Returning launch Event')
						return 'launch'

					return 'aborted'

				# Create a Concurrence container
				idlethreads_concurrence = smach.Concurrence(
					outcomes=['launch', 'aborted'],
					default_outcome='launch',
					child_termination_cb=lambda so: True,
					outcome_cb=out_cb)
	
				# Open the container
				with idlethreads_concurrence:

					# ADD Idle to IdleThreads #
					smach.Concurrence.add('Idle',
						Idle()
					)

					# ADD IdleEventMonitoring to IdleThreads #
					smach.Concurrence.add('IdleEventMonitoring',
						smach_ros.MonitorState('bridge/events/launch',
							SimpleEvent,
							cond_cb=lambda ud, msg: False))
				#  ===================================== IdleThreads END =====================================

				# ADD IdleThreads to SarBehavior #
				smach.StateMachine.add('IdleThreads',
					idlethreads_concurrence,
					transitions={'launch':'Takeoff'})

				# ADD Takeoff to SarBehavior #
				smach.StateMachine.add('Takeoff',
					smach_ros.SimpleActionState('cmd/takeoff',
						TakeoffAction,
						goal=TakeoffGoal(rospy.get_param('~altitude'))),
					transitions={'succeeded':'IdleThreads2'})

				#  ===================================== IdleThreads2 =====================================
				# Callback for custom outcomes from IdleThreads2
				def out_cb(outcome_map):
					if outcome_map['IdleEventMonitoring2'] == 'invalid':
						rospy.loginfo('Returning missionStart Event')
						return 'missionStart'

					return 'aborted'

				# Create a Concurrence container
				idlethreads2_concurrence = smach.Concurrence(
					outcomes=['missionStart', 'aborted'],
					default_outcome='missionStart',
					child_termination_cb=lambda so: True,
					outcome_cb=out_cb)
	
				# Open the container
				with idlethreads2_concurrence:

					# ADD Idle2 to IdleThreads2 #
					smach.Concurrence.add('Idle2',
						Idle()
					)

					# ADD IdleEventMonitoring2 to IdleThreads2 #
					smach.Concurrence.add('IdleEventMonitoring2',
						smach_ros.MonitorState('bridge/events/mission_start',
							SimpleEvent,
							cond_cb=lambda ud, msg: False))
				#  ===================================== IdleThreads2 END =====================================

				# ADD IdleThreads2 to SarBehavior #
				smach.StateMachine.add('IdleThreads2',
					idlethreads2_concurrence,
					transitions={'missionStart':'Coverage'})

				# ADD Coverage to SarBehavior #
				smach.StateMachine.add('Coverage',
					smach_ros.SimpleActionState('uav_coverage',
						CoverageAction,
						goal=CoverageGoal(rospy.get_param('~altitude')),
						result_slots=['target_id', 'target_pose']),
					transitions={'succeeded':'SelectRover'},
					remapping={'target_id':'target_id', 'target_pose':'target_pose'})

				#  ===================================== SelectRoverThreads =====================================
				# Callback for custom outcomes
				def selectrover_outcb(outcome_map):
					if outcome_map['LostEventMonitoring'] == 'invalid':
						rospy.loginfo('Returning target_lost Event')
						return 'target_lost'
					return outcome_map['TaskAllocation']

				# Create a Concurrence container
				selectrover_concurrence = smach.Concurrence(
					input_keys=['target_id','target_pose'],
					outcomes=['succeeded', 'aborted', 'target_lost'],
					default_outcome='aborted',
					child_termination_cb=lambda so: True,
					outcome_cb=selectrover_outcb)

				# Open the container
				with selectrover_concurrence:
					# ADD TaskAllocation to SelectRoverThreads #
					smach.Concurrence.add('TaskAllocation',
						smach_ros.SimpleActionState('cmd/task_allocation_auction',
							TaskAllocationAction,
							goal_slots=['task_id', 'task_pose'],
							result_slots=['task_id', 'winner', 'task_pose']),
						remapping={'task_id':'target_id', 'task_pose':'target_pose'})

					# ADD LostEventMonitoring to SelectRoverThreads #
					smach.Concurrence.add('LostEventMonitoring',
						smach_ros.MonitorState('target_lost',
							TargetPositionEvent,
							cond_cb=lambda ud, msg: False))
				#  ===================================== SelectRoverThreads END =====================================

				# ADD SelectRoverThreads to SarBehavior #
				smach.StateMachine.add('SelectRover',
					selectrover_concurrence,
					transitions={'succeeded':'Tracking', 'aborted':'SelectRover', 'target_lost':'LocalCoverage'})

				def tracking_goal_cb(userdata, goal):
					tracking_goal = TrackingGoal()
					tracking_goal.target = userdata.target
					tracking_goal.altitude = rospy.get_param('~altitude')
					return tracking_goal

				# ADD Tracking to SarBehavior #
				smach.StateMachine.add('Tracking',
					smach_ros.SimpleActionState('uav_tracking',
						TrackingAction,
						goal_cb=tracking_goal_cb),
					transitions={'succeeded':'Coverage', 'aborted':'Coverage'},
					remapping={'target':'target_id'})

				# ADD LocalCoverage to SarBehavior #
# 				smach.StateMachine.add('LocalCoverage',
# 					smach_ros.SimpleActionState('uav_local_coverage',
# 						CoverageAction,
# 						goal=CoverageGoal(rospy.get_param('~altitude')),
# 						result_slots=['target_id', 'target_pose']),
# 					transitions={'aborted':'Coverage', 'succeeded':'SelectRover'},
# 					remapping={'target_id':'target_id', 'target_pose':'pose'})
				
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

			# ADD Idle to MissionAbort #
			smach.StateMachine.add('EndMission',
				Idle(),
				transitions={})
		#  ===================================== MissionAbort END =====================================

		# ADD MissionAbort to TOP state #
		smach.StateMachine.add('MissionAbort',
			missionabort_sm,
			transitions={'succeeded':'succeeded'})
	
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
