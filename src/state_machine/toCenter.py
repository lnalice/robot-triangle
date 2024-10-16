#!/usr/bin/env python3
import smach

from state_machine.module.move import MoveTogetherSM

class ToCenterSM(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=["complete"],
                                    input_keys=['linX', 'rotateAngZ', 'centerAngZ', 'lin_seconds', 'ang_seconds', 'robot_list', 'num_of_side'],
                                    output_keys=['robot_list'])
        
        with self:
            smach.StateMachine.add('ANGLE_ADJUSTMENT', MoveTogetherSM(vel="ang", goal="center"),
                                   transitions={'arrive': 'MOVE'})
            smach.StateMachine.add('MOVE', MoveTogetherSM(vel="lin", goal="polygon"),
                                   transitions={'arrive': 'complete'}),
