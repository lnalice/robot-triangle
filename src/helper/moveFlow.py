from collections import deque
import json
import rospy

polygon_rel_loc = 'data/polygon.json'
test_rel_loc = 'data/test_data.json'
base_path = os.path.abspath(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))
polygon_loc = os.path.join(base_path, polygon_rel_loc) 
test_loc = os.path.join(base_path, test_rel_loc)

def getMoveFlow(linX:float, angZ: float, seconds: float, robot_list:list) -> deque:
    
    move_flow = deque()

    for robot in robot_list:
         task = f"{robot} {seconds} {linX} 0 0 0 0 {angZ} 0"
         move_flow.append(task)

    return move_flow

def getLinFlowOfPolygon(robot_list: list) -> deque:
    
    move_flow = deque()
            
    # read json file
    with open(polygon_loc) as json_file:
            json_data = json.load(json_file)

            for robot_id in robot_list:
                robot_goal = json_data[robot_id]
                
                sec = robot_goal["seconds"]
                lin_vel = robot_goal["lin_vel"]
                ang_vel = robot_goal["ang_vel"]
                delay_sec = robot_goal["move_delay"]

                task = "%s %d %f %f %f %f %f %f %d" %(robot_id, sec, lin_vel["x"], lin_vel["y"], lin_vel["z"], 
                                                ang_vel["x"], ang_vel["y"], ang_vel["z"], delay_sec)
                move_flow.append(task)

    return move_flow

def getSceneDataFlow(take_id: String, robot_id: String) -> deque:
    
    move_flow = deque()
            
    # read json file
    with open(test_loc) as json_file:
            json_data = json.load(json_file)

            for robot_id in robot_list:
                robot_goal = json_data[robot_id]
                
                sec = robot_goal["seconds"]
                lin_vel = robot_goal["lin_vel"]
                ang_vel = robot_goal["ang_vel"]
                delay_sec = robot_goal["move_delay"]

                task = "%s %d %f %f %f %f %f %f %d" %(robot_id, sec, lin_vel["x"], lin_vel["y"], lin_vel["z"], 
                                                ang_vel["x"], ang_vel["y"], ang_vel["z"], delay_sec)
                move_flow.append(task)

    return move_flow

def getSceneDataByRobotID(take_id: String, robot_id: String) -> deque:

     task = ""
    # read json file
    with open(test_loc) as json_file:
            json_data = json.load(json_file)
            
            for robot_goal in json_data[take_id]:
               if robot_goal["id"] != robot_id:
                    continue
               
                sec = robot_goal["seconds"]
                lin_vel = robot_goal["lin_vel"]
                ang_vel = robot_goal["ang_vel"]
                delay_sec = robot_goal["move_delay"]

                task = "%s %d %f %f %f %f %f %f %d" %(robot_id, sec, lin_vel["x"], lin_vel["y"], lin_vel["z"], 
                                                ang_vel["x"], ang_vel["y"], ang_vel["z"], delay_sec)
                

    return task