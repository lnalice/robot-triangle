#!/usr/bin/env python3
import signal

import rospy
import smach
import smach_ros
import math

from state_machine.poylgon import PolygonSM
from state_machine.toCenter import ToCenterSM
from state_machine.module.control_module import CtrlModuleSM

LIN_SECONDS =  7 # 직진 초
LIN_VEL = 0.15 # 직진 속도
LIN_VEL_CENTER = 0.15 / math.sqrt(3) # 중심으로 이동할 때의 속도

NUM_OF_SIDE = 3 # 변의 개수
DEGREE = 180 * (NUM_OF_SIDE -2) / NUM_OF_SIDE # 다각형의 각도: 180×(n-2)÷n
DEGREE_ROTATE = 180 - DEGREE # 로봇 회전 각도
DEGREE_TO_CENTER = DEGREE / 2 # 중심으로 이동시 로봇 회전 각도

ANG_SECONDS = 5
ANG_VEL_ROTATE = 2 * math.pi * (DEGREE_ROTATE / 360) / ANG_SECONDS
ANG_VEL_CENTER = 2 * math.pi * (DEGREE_TO_CENTER / 360) / ANG_SECONDS

def signal_handler(signum, frame):
    res = input("\n[Triangle] Ctrl-c was pressed. Do you want to exit? (y/n) ")
    if res =='y':
        exit(1)

signal.signal(signal.SIGINT, signal_handler)

class Triangle:
    def __init__(self):

        rospy.init_node("triangle_node")

        self.sm = smach.StateMachine(outcomes=["end"])
        self.sm.set_initial_state(['POLYGON'])
        
        self.sm.userdata.robot_list = ['tb3_0', 'tb3_1', 'tb3_2']
        self.sm.userdata.num_of_side = NUM_OF_SIDE
        self.sm.userdata.rotateAngZ = ANG_VEL_ROTATE
        self.sm.userdata.centerAngZ = ANG_VEL_CENTER
        self.sm.userdata.linX = LIN_VEL
        self.sm.userdata.lin_seconds = LIN_SECONDS
        self.sm.userdata.ang_seconds = ANG_SECONDS

        with self.sm:
            smach.StateMachine.add('POLYGON', PolygonSM(),
                                   transitions={'complete': 'GO_TO_CENTER'}),
            smach.StateMachine.add('GO_TO_CENTER', ToCenterSM(),
                                   transitions={'complete': 'CTRL_MODULE'}),
            smach.StateMachine.add('CTRL_MODULE', CtrlModuleSM(direction="forward"),
                                   transitions={'complete': 'end'})

if __name__ == "__main__":

    triangle = Triangle()

    sis = smach_ros.IntrospectionServer('triangle_node', triangle.sm, '/triangle')
    sis. start()

    outcome = triangle.sm.execute()
    rospy.loginfo("[Triangle] final state is %s. done.", outcome)
    
    sis.stop()