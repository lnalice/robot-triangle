import os
from collections import deque
import json

json_vel_rel_loc = 'data/ctrl_module.json'
base_path = os.path.abspath(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))  # upper path
json_vel_loc = os.path.join(base_path, json_vel_rel_loc) 

def getCtrlFlow(isOpposite:bool, robot_list:list) -> deque:
    
    ctrl_flow = deque()

    with open(json_vel_loc) as json_file:
          json_data = json.load(json_file)
          degZ = json_data["degZ"]
          degX = json_data["degX"]

          for robotID in robot_list:
               task = ""
               if isOpposite:
                    task = "%s %f %f %d" %(robotID, -float(degZ), -float(degX), 0)
               else:
                    task = "%s %f %f %d" %(robotID, degZ, degX, 0)

               ctrl_flow.append(task)

    return ctrl_flow