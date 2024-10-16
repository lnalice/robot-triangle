import rospy
import smach
import smach_ros
import smach_ros.monitor_state
from std_msgs.msg import String

from collections import deque
import math

from helper.moveFlow import getMoveFlow, getLinFlowOfPolygon

class MoveRequest(smach.State):
    def __init__(self, vel:String, goal:String):
        smach.State.__init__(self, outcomes=["done", "none"],
                                    input_keys=['linX', 'rotateAngZ', 'centerAngZ', 'lin_seconds', 'ang_seconds', 'robot_list'],
                                    output_keys=['robot_list'])

        self.move_pub = rospy.Publisher('/scene_manager/move_req', String, queue_size=1)
        self.vel = vel
        self.goal = goal

    def execute(self, user_data):
        move_flow = deque()
        linX = user_data.linX
        rotateAngZ = user_data.rotateAngZ
        centerAngZ = user_data.centerAngZ
        lin_seconds = user_data.lin_seconds
        ang_seconds = user_data.ang_seconds

        if self.vel == "lin" and self.goal == "polygon": # 다각형 직진
            move_flow = getLinFlowOfPolygon(['tb3_0', 'tb3_1', 'tb3_2'])
            
        elif self.vel == "ang" and self.goal == "polygon": # 다각형 회전
            move_flow = getMoveFlow(0, rotateAngZ, ang_seconds, ['tb3_0', 'tb3_1', 'tb3_2'])

        elif self.vel == "lin" and self.goal == "center":
            move_flow = getMoveFlow(linX / math.sqrt(3), 0, lin_seconds, ['tb3_0', 'tb3_1', 'tb3_2'])
        
        elif self.vel == "ang" and self.goal == "center": 
            move_flow = getMoveFlow(0, centerAngZ / 2, ang_seconds, ['tb3_0', 'tb3_1', 'tb3_2'])

        user_data.robot_list = []

        if len(move_flow) == 0:
            return 'none'
        
        while move_flow:
            rospy.sleep(0.3)
            
            goal_data = move_flow.popleft() 
            
            self.move_pub.publish(goal_data)

            rospy.loginfo("[MoveTogether] move_req is published now.")
            rospy.loginfo("[MoveTogether] data published now: %s", goal_data)

            user_data.robot_list.append(goal_data.split()[0])

        rospy.loginfo("[MoveTogether] robot_list is updated now (%s)", str(user_data.robot_list))

        return 'done'

class OnTheMove(smach_ros.MonitorState):
    def __init__(self):
        smach_ros.MonitorState.__init__(self, '/scene_manager/move_res', String, self.check_leftover,
                                        input_keys=['robot_list'],
                                        output_keys=['robot_list'])
        
    def check_leftover(self, user_data, res_msg):
        result = str(res_msg.data).split()

        if result[0] in user_data.robot_list:
            user_data.robot_list.remove(result[0])

            rospy.loginfo(f"robot %s arrived", result[0])
            rospy.loginfo("[MoveTogether] robot_list is updated now (%s)", str(user_data.robot_list))
        
        if len(user_data.robot_list) > 0:
            return True
        
        return False

class MoveTogetherSM(smach.StateMachine):
    def __init__(self, vel:String, goal:String):
        smach.StateMachine.__init__(self, outcomes=["arrive"],
                                    input_keys=['linX', 'rotateAngZ', 'centerAngZ', 'lin_seconds', 'ang_seconds', 'robot_list'],
                                    output_keys=['robot_list'])
        
        with self:
            self.add('MOVE_REQUEST', MoveRequest(vel, goal),
                     transitions={'done': 'ON_THE_MOVE',
                                  'none': "arrive"})
            self.add('ON_THE_MOVE', OnTheMove(),
                     transitions={'invalid': 'arrive',
                                'valid': 'ON_THE_MOVE',
                                'preempted':'ON_THE_MOVE'})            