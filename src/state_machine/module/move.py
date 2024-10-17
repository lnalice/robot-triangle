import rospy
import smach
import smach_ros
import smach_ros.monitor_state
from std_msgs.msg import String

from collections import deque
import math

from helper.moveFlow import getMoveFlow, getLinFlowOfPolygon, getSceneDataFlow, getSceneDataByRobotID

take_cnt:dict= {"tb3_0": 3, "tb3_1": 3, "tb3_2": 3}

class MoveRequest(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["done", "none"],
                                    input_keys=['robot_list'],
                                    output_keys=['robot_list'])

        self.move_pub = rospy.Publisher('/scene_manager/move_req', String, queue_size=1)

    def execute(self, user_data):
        move_flow = deque()
        request_robot_list = ['tb3_0', 'tb3_1', 'tb3_2']

        move_flow = getSceneDataFlow("take_3")
        
        user_data.robot_list = []

        if len(move_flow) == 0:
            return 'none'
        
        while move_flow:
            rospy.sleep(0.2)
            
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

        self.move_pub = rospy.Publisher('/scene_manager/move_req', String, queue_size=1)
        
    def check_leftover(self, user_data, res_msg):
        result = str(res_msg.data).split()

        arrived_robot = result[0]

        if arrived_robot in user_data.robot_list:
            rospy.loginfo(f"robot %s arrived", result[0])
            take_cnt[arrived_robot] -= 1
            if take_cnt[arrived_robot] > 0:
                goal_data = getSceneDataByRobotID("take_" + str(take_cnt[arrived_robot]), arrived_robot)
                self.move_pub.publish(goal_data)
                rospy.loginfo("[MoveTogether] I would publish data 'take_%s' for %s", str(take_cnt), arrived_robot)
                rospy.loginfo("[MoveTogether] data published now: %s", goal_data)
            else:
                user_data.robot_list.remove(result[0])
                rospy.loginfo("[MoveTogether] robot_list is updated now (%s)", str(user_data.robot_list))
        
        if len(user_data.robot_list) > 0:
            return True
        
        return False

class MoveTogetherSM(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=["arrive"],
                                    input_keys=['robot_list'],
                                    output_keys=['robot_list'])
        
        with self:
            self.add('MOVE_REQUEST', MoveRequest(),
                     transitions={'done': 'ON_THE_MOVE',
                                  'none': "arrive"})
            self.add('ON_THE_MOVE', OnTheMove(),
                     transitions={'invalid': 'arrive',
                                'valid': 'ON_THE_MOVE',
                                'preempted':'ON_THE_MOVE'})            
