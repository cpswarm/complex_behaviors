#!/usr/bin/env python

import rospy
import smach
import smach_ros

from actionlib.simple_action_client import SimpleActionClient, GoalStatus
from move_base_msgs.msg import *
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
                        rospy.loginfo('Returning boxFound Event')
                        return 'boxFound'

                    return 'aborted'

                # Create a Concurrence container
                idlethreads_concurrence = smach.Concurrence(
                    outcomes=['boxFound', 'aborted'],
                    default_outcome='boxFound',
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
                        goal_slots=['task_id', 'task_pose', 'auctioneer'],
                        result_slots=['task_pose']),
                    transitions={'succeeded':'MoveToBox', 'aborted':'IdleThreads'},
                    remapping={'task_id':'box_id', 'task_pose':'box_position', 'auctioneer':'scout'})

                # ADD MoveToBox to WorkerBehavior #
                smach.StateMachine.add('MoveToBox',
                    smach_ros.SimpleActionState('cmd/moveto',
                        MoveBaseAction,
                        goal_slots=['target_pose']),
                    transitions={'succeeded':'MoveToDest', 'aborted':'MoveToHome'},
                    remapping={'target_pose':'box_position'})

                # ADD MoveToDest to WorkerBehavior #
                def move_dest_cb(ud, goal):
                    goal = MoveBaseGoal()
                    goal.target_pose.pose.position.x = 4 # TODO: should be parameter!
                    goal.target_pose.pose.position.y = 0
                    goal.target_pose.pose.orientation.w = 1;
                    rospy.loginfo('Deliver Box to %.2f, %.2f', goal.target_pose.pose.position.x, goal.target_pose.pose.position.y)
                    return goal

                smach.StateMachine.add('MoveToDest',
                    smach_ros.SimpleActionState('cmd/moveto',
                        MoveBaseAction,
                        goal_cb=move_dest_cb),
                    transitions={'succeeded':'PublishState', 'aborted':'MoveToHome'})

                # ADD PublishState to WorkerBehavior #
                smach.StateMachine.add('PublishState',
                    smach_ros.SimpleActionState('cmd/target_done',
                        TargetAction,
                        goal_slots=['id', 'pose']),
                    transitions={'succeeded':'MoveToHome', 'aborted':'MoveToHome'},
                    remapping={'id':'box_id', 'pose':'box_position'})

                # ADD MoveToHome to WorkerBehavior #
                def move_home_cb(ud, goal):
                    goal = MoveBaseGoal()
                    goal.target_pose.pose.position.x = 0 # TODO: should be parameter!
                    goal.target_pose.pose.position.y = -4
                    goal.target_pose.pose.orientation.w = 1;
                    rospy.loginfo('Going home %.2f, %.2f', goal.target_pose.pose.position.x, goal.target_pose.pose.position.y)
                    return goal

                smach.StateMachine.add('MoveToHome',
                    smach_ros.SimpleActionState('cmd/moveto',
                        MoveBaseAction,
                        goal_cb=move_home_cb),
                    transitions={'succeeded':'IdleThreads', 'aborted':'IdleThreads'})

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
            goal = MoveBaseGoal()
            goal.target_pose.pose.position.x = 0 # TODO: should be parameter!
            goal.target_pose.pose.position.y = -4
            goal.target_pose.pose.orientation.w = 1;
            rospy.loginfo('Going HOME: %.2f, %.2f', goal.target_pose.pose.position.x, goal.target_pose.pose.position.y)
            return goal

        # ADD GoHome to TOP state #
        smach.StateMachine.add('GoHome',
            smach_ros.SimpleActionState('cmd/moveto',
                MoveBaseAction,
                goal_cb=move_home_cb),
            transitions={})

    # Create and start the introspection server (uncomment if needed)
    #sis = smach_ros.IntrospectionServer('smach_server', top_sm, '/SM_TOP')
    #sis.start()

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
