#!/usr/bin/env python3
import signal

import rospy
import smach
import smach_ros
import math

# from state_machine.poylgon import PolygonSM
# from state_machine.toCenter import ToCenterSM
from state_machine.module.control_module import CtrlModuleSM
from state_machine.module.move import MoveTogetherSM

def signal_handler(signum, frame):
    res = input("\n[Triangle] Ctrl-c was pressed. Do you want to exit? (y/n) ")
    if res =='y':
        exit(1)

signal.signal(signal.SIGINT, signal_handler)

class Triangle:
    def __init__(self):

        rospy.init_node("triangle_node")

        self.sm = smach.StateMachine(outcomes=["end"])
        self.sm.set_initial_state(['RANDOM_MOVE'])
        
        self.sm.userdata.robot_list = ['tb3_0', 'tb3_1', 'tb3_2']

        with self.sm:
            smach.StateMachine.add('CTRL_MODULE', CtrlModuleSM(direction="forward"),
                                   transitions={'complete': 'RANDOM_MOVE'})
            smach.StateMachine.add('RANDOM_MOVE', MoveTogetherSM(),
                                   transitions={'arrive': 'end'}),
            

if __name__ == "__main__":

    triangle = Triangle()

    sis = smach_ros.IntrospectionServer('triangle_node', triangle.sm, '/triangle')
    sis. start()

    outcome = triangle.sm.execute()
    rospy.loginfo("[Triangle] final state is %s. done.", outcome)
    
    sis.stop()
