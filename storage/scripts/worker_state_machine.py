#!/usr/bin/env python

import rospy
import smach
import smach_ros
import geometry_msgs
import move_base_msgs

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

	# Create a TOP level SMACH state machine
	top_sm = smach.StateMachine(['succeeded', 'preempted', 'aborted'])

	# Open the container
	with top_sm:

		#  ===================================== WorkerThreads =====================================
		# Callback for custom outcomes from WorkerThreads
		def out_cb(outcome_map):
			if outcome_map['GoHomeEventMonitoring'] == 'invalid':
				rospy.loginfo('Returning goHome Event')
				return 'goHome'

			return 'aborted'

		# Create a Concurrence container
		workerthreads_concurrence = smach.Concurrence(
			outcomes=['goHome', 'aborted'],
			default_outcome='goHome',
			child_termination_cb=lambda so: True,
			outcome_cb=out_cb)

		# Open the container
		with workerthreads_concurrence:

			# ===================================== WorkerBehavior =====================================
			# Create a State Machine container
			workerbehavior_sm = smach.StateMachine(
				outcomes=['succeeded', 'preempted', 'aborted'])

			# Open the container
			with workerbehavior_sm:

				#  ===================================== IdleThreads =====================================
				# Callback for custom outcomes from IdleThreads
				def out_cb(outcome_map):
					if outcome_map['IdleEventMonitoring'] == 'invalid':
						rospy.loginfo('Returning targetFound Event')
						return 'targetFound'

					return 'aborted'

				# Create a Concurrence container
				idlethreads_concurrence = smach.Concurrence(
					outcomes=['targetFound', 'aborted'],
					default_outcome='targetFound',
					child_termination_cb=lambda so: True,
					outcome_cb=out_cb,
					output_keys=['box_id', 'scout', 'box_position'])

				# Open the container
				with idlethreads_concurrence:

					# ADD Idle to IdleThreads #
					smach.Concurrence.add('Idle',
						Idle()
					)

					def monitor_cb(ud, msg):
						rospy.loginfo('Executing monitor_cb')
						ud.box_id = msg.id
						ud.scout = msg.swarmio.node
						ud.box_position = msg.pose
						return False

					# ADD IdleEventMonitoring to IdleThreads #
					smach.Concurrence.add('IdleEventMonitoring',
						smach_ros.MonitorState('bridge/events/target_found',
							TargetPositionEvent,
							cond_cb=monitor_cb,
							output_keys=['box_id', 'scout', 'box_position']))
				#  ===================================== IdleThreads END =====================================

				# ADD IdleThreads to WorkerBehavior #
				smach.StateMachine.add('IdleThreads',
					idlethreads_concurrence,
					transitions={'boxFound':'AssignBox'})

				# ADD AssignBox to WorkerBehavior #
				smach.StateMachine.add('AssignBox',
					smach_ros.SimpleActionState('cmd/task_allocation_bid',
						TaskAllocationAction,
						goal_slots=['task_id', 'task_pose', 'auctioneer']),
						result_slots=['task_pose']),
					transitions={'succeeded':'MoveToBox', 'aborted':'IdleThreads'}
                    remapping={'task_id':'box_id', 'task_pose':'box_position', 'scout':'auctioneer'})

				# ADD MoveToBox to WorkerBehavior #
				smach.StateMachine.add('MoveToBox',
					smach_ros.SimpleActionState('cmd/moveto',
						move_base_msgs.MoveBaseGoal,
						goal_slots=['target_pose']),
					transitions={'succeeded':'MoveToDest', 'aborted':'GoHome'}
                    remapping={'target_pose':'box_position'})

				# ADD MoveToDest to WorkerBehavior #
                def move_dest_cb(ud, goal):
                    goal = geometry_msgs.Pose()
                    goal.position.x = 4 # TODO: should be parameter!
                    goal.position.y = 0
                    rospy.loginfo('Deliver Box to %.2f, %.2f', goal.position.x, goal.position.y)
                    return goal

				smach.StateMachine.add('MoveToDest',
					smach_ros.SimpleActionState('cmd/moveto',
						move_base_msgs.MoveBaseGoal,
                        goal_cb=move_dest_cb,
						goal_slots=['target_pose']),
					transitions={'succeeded':'GoHome', 'aborted':'GoHome'}
                    remapping={'target_pose':'dest_position'})

			#  ===================================== WorkerBehavior END =====================================

			# ADD WorkerBehavior to WorkerThreads #
			smach.Concurrence.add('WorkerBehavior', workerbehavior_sm)

			# ADD GoHomeEventMonitoring to WorkerThreads #
			smach.Concurrence.add('GoHomeEventMonitoring',
				smach_ros.MonitorState('bridge/events/go_home',
					SimpleEvent,
					cond_cb=lambda ud, msg: False))
		#  ===================================== WorkerThreads END =====================================

		# ADD WorkerThreads to TOP state #
		smach.StateMachine.add('WorkerThreads',
			workerthreads_concurrence,
			transitions={'goHome':'GoHome'})

		def move_home_cb(ud, goal):
			goal = geometry_msgs.Pose()
			goal.position.x = 0 # TODO: should be parameter!
			goal.position.y = -4
			rospy.loginfo('Going HOME: %.2f, %.2f', goal.position.x, goal.position.y)
			return goal

		# ADD GoHome to TOP state #
		smach.StateMachine.add('GoHome',
			smach_ros.SimpleActionState('cmd/moveto',
				move_base_msgs.MoveBaseGoal,
				goal_cb=move_home_cb,
                goal_slots=['target_pose']),
			transitions={'boxFound':'AssignBox'})

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
