#!/usr/bin/env python

import rospy
import smach
import smach_ros
import mavros_msgs
import std_srvs.srv

from cpswarm_msgs.msg import *
from swarmros.msg import *
from uav_mavros_takeoff.msg import *
from move_base_msgs.msg import *
from geometry_msgs.msg import PoseStamped


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
	
class GoHome(smach.State):

	def __init__(self):
	    smach.State.__init__(self, outcomes=['succeeded'], input_keys=['home'], output_keys=['task_pose'])

	def execute(self, userdata):
		pub = rospy.Publisher('pos_controller/goal_position', PoseStamped, queue_size=1)
		
		target_pose = PoseStamped()
		target_pose = userdata.home
		target_pose.pose.position.z = rospy.get_param('~altitude')
	 	rospy.sleep(0.5)
	 	pub.publish(target_pose)
	 	rospy.loginfo('Move Home at [%.2f, %.2f]', target_pose.pose.position.x, target_pose.pose.position.y)
			
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
			elif outcome_map['StopEventMonitoring'] == 'invalid':
				rospy.loginfo('Returning missionStop Event')
				return 'missionStop'

			return 'aborted'

		# Create a Concurrence container
		sarthreads_concurrence = smach.Concurrence(
			outcomes=['missionAbort', 'missionStop', 'aborted'],
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
					transitions={'launch':'TakeOff'})

				# ADD TakeOff to SarBehavior #
				smach.StateMachine.add('TakeOff',
					smach_ros.SimpleActionState('cmd/takeoff',
						TakeOffAction,
						goal=TakeOffGoal(rospy.get_param('~altitude'))),
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

				# ADD SelectRover to SarBehavior #
				smach.StateMachine.add('SelectRover',
					smach_ros.SimpleActionState('cmd/task_allocation_auction',
						TaskAllocationAction,
						goal_slots=['task_id', 'task_pose'],
						result_slots=['task_id', 'winner', 'task_pose']),
					transitions={'succeeded':'Tracking', 'aborted':'SelectRover'},
					remapping={'task_id':'target_id', 'task_pose':'target_pose'})

				def tracking_goal_cb(userdata, goal):
					tracking_goal = TrackingGoal()
					tracking_goal.target = userdata.target
					tracking_goal.altitude = rospy.get_param('~altitude')
					return tracking_goal

				# ADD Tracking to SarBehavior #
				smach.StateMachine.add('Tracking',
					smach_ros.SimpleActionState('uav_tracking',
						TrackingAction,
						goal_cb=tracking_goal_cb,
						input_keys=['target']),
					transitions={'succeeded':'Coverage', 'aborted':'Coverage'},
					remapping={'target':'target_id'})

				# ADD LocalCoverage to SarBehavior #
				# smach.StateMachine.add('LocalCoverage',
				# 	smach_ros.SimpleActionState('uav_local_coverage',
				# 		CoverageAction,
				# 		goal=CoverageGoal(rospy.get_param('~altitude')),
				# 		result_slots=['target_id', 'target_pose']),
				# 	transitions={'aborted':'Coverage', 'succeeded':'SelectRover'},
				# 	remapping={'target_id':'target_id', 'target_pose':'pose'})
				
			#  ===================================== SarBehavior END =====================================

			# ADD SarBehavior to SarThreads #
			smach.Concurrence.add('SarBehavior', sarbehavior_sm)

			# ADD AbortEventMonitoring to SarThreads #
			smach.Concurrence.add('AbortEventMonitoring',
				smach_ros.MonitorState('bridge/events/mission_abort',
					SimpleEvent,
					cond_cb=lambda ud, msg: False))
			
			# ADD StopEventMonitoring to SarThreads #
			smach.Concurrence.add('StopEventMonitoring',
				smach_ros.MonitorState('bridge/events/mission_stop',
					SimpleEvent,
					cond_cb=lambda ud, msg: False))	
		#  ===================================== SarThreads END =====================================

		# ADD SarThreads to TOP state #
		smach.StateMachine.add('SarThreads',
			sarthreads_concurrence,
			transitions={'missionAbort':'MissionAbort', 'missionStop':'StopThreads'})

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
		
		#  ===================================== StopThreads =====================================
		
		# Callback for custom outcomes from StopThreads
		def out_cb(outcome_map):
			if outcome_map['AbortEventMonitoring2'] == 'invalid':
				rospy.loginfo('Returning missionAbort2 Event')
				return 'missionAbort'

			return 'aborted'

		# Create a Concurrence container
		stopthreads_concurrence = smach.Concurrence(
			outcomes=['missionAbort', 'aborted'],
			default_outcome='missionAbort',
			child_termination_cb=lambda so: True,
			outcome_cb=out_cb)
	
		# Open the container
		with stopthreads_concurrence:
			
			# ===================================== StopBehavior =====================================
			# Create a State Machine container
			stopbehavior_sm = smach.StateMachine(
				outcomes=['succeeded', 'preempted', 'aborted'])
			
			stopbehavior_sm.userdata.task_counter = 0
			
			# Open the container
			with stopbehavior_sm:
				
				def task_organize_goal_cb(userdata, goal):
					organize_goal = TaskOrganizationGoal()
					home_x = rospy.get_param('~home_x')
					home_y = rospy.get_param('~home_y')
					organize_goal.task_id = userdata.task_counter
					organize_goal.task_pose.pose.position.x = home_x[userdata.task_counter]
					organize_goal.task_pose.pose.position.y = home_y[userdata.task_counter]
					organize_goal.task_pose.pose.position.z = rospy.get_param('~altitude')
					organize_goal.task_pose.pose.orientation.w = 1
					userdata.task_counter += 1
					return organize_goal
				
				# ADD TaskOrganize to StopBehavior #
				smach.StateMachine.add('TaskOrganize',
					smach_ros.SimpleActionState('cmd/task_organize',
						TaskOrganizationAction,
						goal_cb=task_organize_goal_cb,
						input_keys=['task_counter'],
						output_keys=['task_counter'],
						result_slots=['task_id', 'task_pose']),
					transitions={'succeeded':'GoHome', 'aborted':'IdleThreads3'},
					remapping={'task_id':'task_id', 'task_pose':'task_pose'})
				
# 				def go_home_cb(ud, goal):
# 					goal = MoveBaseGoal()
# 					goal.target_pose = ud.home
# 					return goal
                
                # ADD MoveHome to StopBehavior #   
				smach.StateMachine.add('GoHome',
                    GoHome(),
					transitions={'succeeded':'IdleThreads4'},
					remapping={'home':'task_pose'})
				
				#  ===================================== IdleThreads3 =====================================
				# Callback for custom outcomes from IdleThreads3
				def out_cb(outcome_map):
					if outcome_map['IdleEventMonitoring3'] == 'invalid':
						rospy.loginfo('Returning taskOrganize Event')
						return 'taskOrganize'

					return 'aborted'

				# Create a Concurrence container
				idlethreads3_concurrence = smach.Concurrence(
					outcomes=['taskOrganize', 'aborted'],
					default_outcome='taskOrganize',
					child_termination_cb=lambda so: True,
					outcome_cb=out_cb)
	
				# Open the container
				with idlethreads3_concurrence:

					# ADD Idle3 to IdleThreads3 #
					smach.Concurrence.add('Idle3',
						Idle()
					)

					# ADD IdleEventMonitoring3 to IdleThreads3 #
					smach.Concurrence.add('IdleEventMonitoring3',
						smach_ros.MonitorState('bridge/events/land',
							SimpleEvent,
							cond_cb=lambda ud, msg: False))
				#  ===================================== IdleThreads3 END =====================================

				# ADD IdleThreads3 to StopBehavior #
				smach.StateMachine.add('IdleThreads3',
					idlethreads3_concurrence,
					transitions={'taskOrganize':'TaskOrganize'})
				
				#  ===================================== IdleThreads4 =====================================
				# Callback for custom outcomes from IdleThreads4
				def out_cb(outcome_map):
					if outcome_map['IdleEventMonitoring4'] == 'invalid':
						rospy.loginfo('Returning land Event')
						return 'land'

					return 'aborted'

				# Create a Concurrence container
				idlethreads4_concurrence = smach.Concurrence(
					outcomes=['land', 'aborted'],
					default_outcome='land',
					child_termination_cb=lambda so: True,
					outcome_cb=out_cb)
	
				# Open the container
				with idlethreads4_concurrence:

					# ADD Idle4 to IdleThreads4 #
					smach.Concurrence.add('Idle4',
						Idle()
					)

					# ADD IdleEventMonitoring4 to IdleThreads4 #
					smach.Concurrence.add('IdleEventMonitoring4',
						smach_ros.MonitorState('bridge/events/land',
							SimpleEvent,
							cond_cb=lambda ud, msg: False))
				#  ===================================== IdleThreads4 END ====================================
				
				# ADD IdleThreads4 to StopBehavior #
				smach.StateMachine.add('IdleThreads4',
					idlethreads4_concurrence,
					transitions={'land':'Land'})
				
				# ADD Land to StopBehavior #
				smach.StateMachine.add('Land',
					smach_ros.ServiceState('cmd/land',
						std_srvs.srv.Empty),
					transitions={'succeeded':'succeeded'})
				
			#  ===================================== StopBehavior END =====================================
			
			# ADD SarBehavior to StopThreads #
			smach.Concurrence.add('StopBehavior', stopbehavior_sm)
			
			# ADD AbortEventMonitoring2 to StopThreads #
			smach.Concurrence.add('AbortEventMonitoring2',
				smach_ros.MonitorState('bridge/events/mission_abort',
					SimpleEvent,
					cond_cb=lambda ud, msg: False))
			
		#  ===================================== StopThreads END =====================================
		
		# ADD SarThreads to TOP state #
		smach.StateMachine.add('StopThreads',
			stopthreads_concurrence,
			transitions={'missionAbort':'MissionAbort'})
	
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
