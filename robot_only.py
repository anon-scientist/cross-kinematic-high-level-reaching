"""
CRoSS benchmark suite, HLR  benchmark. Example for standalone use of provided
robot manager, without RL framework
"""

from gazebo_sim.simulation.PandaRobot import PandaRobot

end=False

class RobotAction():
    def __init__(self,label,xyz_translation,speed=1.0):
        self.label = label
        self.x = xyz_translation[0] * speed
        self.y = xyz_translation[1] * speed
        self.z = xyz_translation[2] * speed
    def xyz_translation(self):
        return [self.x,self.y,self.z]

class State():
    def __init__(self, start):
        self.reset(start)
    def reset(self, start=None):
        if start:
            self.start = list(start)
        self.previous = list(self.start)
        self.current = list(self.start)
    def move(self,direction):
        self.previous = list(self.current)
        self.current[0] += direction[0]
        self.current[1] += direction[1]
        self.current[2] += direction[2]
    def pretend_to_move(self,direction):
        out = list(self.current)
        out[0] += direction[0]
        out[1] += direction[1]
        out[2] += direction[2]
        return out
    def move_to(self,position):
        self.previous = list(self.current)
        self.current = list(position)
    def revert(self):
        self.current = list(self.previous)

def perform_action(indx):
    last_move_illegal = True
    action = robot.actions[indx] # select action
    move_to = state.pretend_to_move(action.xyz_translation())
    joint_actions = robot.compute_inverse_kinematics(move_to)
    if joint_actions:
        state.move_to(robot.compute_forward_kinematic(joint_actions))
        last_move_illegal = False
    else:
        approx_joint_actions = robot.compute_inverse_kinematic_approx(move_to)
        if approx_joint_actions:
            state.move_to(robot.compute_forward_kinematic(approx_joint_actions))
            last_move_illegal = False
    return last_move_illegal
    
action_speed = 0.1
actions = []
actions.append(RobotAction("left",[-1.0,0.0,0.0],action_speed)) # 0
actions.append(RobotAction("right",[1.0,0.0,0.0],action_speed)) # 1
actions.append(RobotAction("forwards",[0.0,0.0,-1.0],action_speed)) # 2
actions.append(RobotAction("backwards",[0.0,0.0,1.0],action_speed)) # 3
actions.append(RobotAction("down",[0.0,-1.0,0.0],action_speed)) # 4
actions.append(RobotAction("up",[0.0,1.0,0.0],action_speed)) # 5

robot = PandaRobot(actions)

state = State([0.0,0.0,0.0])

while not end:
    userin = input(f"Move with [WASDRF]. Current Pos: {state.current}: ")
    illegal = False
    if userin == "w" or userin == "W":
        illegal = perform_action(3)
    elif userin == "a" or userin == "A":
        illegal = perform_action(0)
    elif userin == "s" or userin == "S":
        illegal = perform_action(2)
    elif userin == "d" or userin == "D":
        illegal = perform_action(1)
    elif userin == "r" or userin == "R":
        illegal =perform_action(5)
    elif userin == "f" or userin == "F":
        illegal = perform_action(4)
    elif userin == "e" or userin == "exit" or userin == "q" or userin == "quit":
        end = True
        break;
    else:
        print("Illegal Input. To exit write 'e' or 'exit'")
        continue
    if illegal:
        print("This move is illegal and cannot be performed. Try other actions.")