#!/usr/bin/env python3
import rospy
import smach

from state_machine.module.move import MoveTogetherSM

class CountState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['complete', 'in_progress'],
                             input_keys=['num_of_side'],
                             output_keys=['num_of_side'])
        
    def execute(self, user_data):

        num_of_side = user_data.num_of_side
        user_data.num_of_side = num_of_side - 1

        rospy.logwarn(f"[Counter] {user_data.num_of_side} times left.")

        if user_data.num_of_side <= 0:
            return 'complete'
        
        return 'in_progress'
        
class CounterSM(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=['in_progress', 'complete'],
                                    input_keys=['num_of_side'],
                                    output_keys=['num_of_side'])
                
        with self:
            self.add('COUNT', CountState(),
                     transitions={'in_progress': 'in_progress',
                                  'complete': 'complete'})
            

class PolygonSM(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=["complete"],
                                    input_keys=['linX', 'rotateAngZ', 'centerAngZ', 'lin_seconds', 'ang_seconds', 'robot_list', 'num_of_side'],
                                    output_keys=['robot_list', 'num_of_side'])
        
        with self:
            smach.StateMachine.add('MOVE', MoveTogetherSM(vel="lin", goal="polygon"),
                                   transitions={'arrive': 'ANGLE_ADJUSTMENT'}),
            smach.StateMachine.add('ANGLE_ADJUSTMENT', MoveTogetherSM(vel="ang", goal="polygon"),
                                   transitions={'arrive': 'COUNT'})
            smach.StateMachine.add('COUNT', CounterSM(),
                                   transitions={'in_progress': 'MOVE',
                                                'complete': 'complete'})