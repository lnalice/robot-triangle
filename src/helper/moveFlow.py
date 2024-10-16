from collections import deque

def getMoveFlow(linX:float, angZ: float, seconds: float, robot_list:list) -> deque:
    
    move_flow = deque()

    for robot in robot_list:
         task = f"{robot} {seconds} {linX} 0 0 0 0 {angZ} 0"
         move_flow.append(task)

    return move_flow