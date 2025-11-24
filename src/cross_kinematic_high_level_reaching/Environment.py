"""
$-task high-level reaching scenario
"""
import numpy as np
from gazebo_sim.simulation.Environment import GenericEnvironment
from gazebo_sim.simulation.PandaRobot import PandaRobot as Robot
from cl_experiment.parsing import Kwarg_Parser
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

class RobotAction():
    def __init__(self,label,xyz_translation,speed=1.0):
        self.label = label
        self.x = xyz_translation[0] * speed
        self.y = xyz_translation[1] * speed
        self.z = xyz_translation[2] * speed
    def xyz_translation(self):
        return [self.x,self.y,self.z]

class Task():
    def __init__(self, name, goals):
        self.name = name
        self.goals = goals

    def get_milestone_amount(self):
        return len(self.goals)

class RobotArmEnvironment():
    def __init__(self,**kwargs) -> None:
        self.config = self.parse_args(**kwargs)

        self.observation_shape = [6]
        self.training_duration = self.config.training_duration
        self.evaluation_duration = self.config.evaluation_duration
        self.max_steps_per_episode = self.config.max_steps_per_episode
        self.task_list = self.config.task_list
        self.reward_skew = self.config.reward_skew

        # Possible Tasks
        self.tasks = {}
        self.tasks["hammer"] = Task("hammer", [[0.5, -0.1, 0.1]])
        self.tasks["push_wall"] = Task("push_wall", [[0.4, 0.3, 0.1]])
        self.tasks["faucet_close"] = Task("faucet_close", [[0.8, 0.0, 0.4]])
        self.tasks["push_back"] = Task("push_back", [[0.4, 0.0, 0.1]])
        self.tasks["stick_pull"] = Task("stick_pull", [[0.5, 0.3, 0.6]])
        self.tasks["handle_press_side"] = Task("handle_press_side", [[0.5, 0.5, 0.5]])
        self.tasks["push"] = Task("push", [[0.7, 0.0, 0.1]])
        self.tasks["shelf_place"] = Task("shelf_place", [[0.7, 0.0, 0.8]])
        self.tasks["window_close"] = Task("window_close", [[0.5, -0.4, 0.7]])
        self.tasks["peg_unplug_side"] = Task("peg_unplug_side", [[0.5, 0.4, 0.4]])

        ## Action space of the Robot
        action_speed = self.config.action_speed
        actions = []
        actions.append(RobotAction("left",[-1.0,0.0,0.0],action_speed))
        actions.append(RobotAction("right",[1.0,0.0,0.0],action_speed))
        actions.append(RobotAction("forwards",[0.0,0.0,-1.0],action_speed))
        actions.append(RobotAction("backwards",[0.0,0.0,1.0],action_speed))
        actions.append(RobotAction("down",[0.0,-1.0,0.0],action_speed))
        actions.append(RobotAction("up",[0.0,1.0,0.0],action_speed))

        self.robot = Robot(actions)
        self.nr_actions = len(self.robot.actions)
        self.goal_discrepency_threshold = self.config.goal_discrepency_threshold

        self.step_count = 0
        self.task_index = 0

        self.task_milestone = 0
        self.last_move_illegal = False

        self.state = State([0.0,0.0,0.0])

        self.info = {
           'input_dims': self.observation_shape,
           'number_actions': len(actions),
           'terminate_cond': 'unassigned',
        }
    
    def get_current_status(self):
        return (self.info['object'][0], self.info['terminate_cond'])
    
    def get_nr_of_tasks(self):
        return len(self.task_list)

    def get_input_dims(self):
        return self.observation_shape
    
    def get_observation(self):
        return np.array([self.state.current[0],self.state.current[1],self.state.current[2],self.tasks[self.task_id].goals[self.task_milestone][0],self.tasks[self.task_id].goals[self.task_milestone][1],self.tasks[self.task_id].goals[self.task_milestone][2]],dtype=np.float32)

    def switch(self, task_index: int) -> None:
        self.task_id = self.task_list[task_index]
        self.reset()

    def reset(self):
        self.current_name = self.task_id
        self.info['object'] = (self.current_name,)
        self.step_count = 0
        self.task_milestone = 0
        self.state.reset()

        state = self.get_observation()

        _, _, _ = self.compute_reward(state)
        return (state, self.info)

    def step(self, action_index: int):
        self.perform_action(action_index=action_index)
        self.step_count += 1

        state = self.get_observation()

        ## compute reward
        reward, terminated, truncated = self.compute_reward(state) ;
        if self.step_count > self.max_steps_per_episode: # should not be necessary but is a precaution
            terminated = True
        return state,reward,terminated,truncated, self.info ;

    def perform_action(self, action_index:int)->None:
        """ high level action execution """
        self.last_move_illegal = True

        action = self.robot.actions[action_index] # select action
        move_to = self.state.pretend_to_move(action.xyz_translation())
        joint_actions = self.robot.compute_inverse_kinematics(move_to)
        if joint_actions:
            self.state.move_to(self.robot.compute_forward_kinematic(joint_actions))
            self.last_move_illegal = False
        else:
            approx_joint_actions = self.robot.compute_inverse_kinematic_approx(move_to)
            if approx_joint_actions:
                self.state.move_to(self.robot.compute_forward_kinematic(approx_joint_actions))
                self.last_move_illegal = False

    def compute_reward(self, state):
        truncated = False
        terminated = False
        
        # Get relevant vectors
        current = state[:3]
        target = state[3:]

        # Calculate target distance from start:
        magnitude = np.linalg.norm(np.array(target) - self.state.start)

        # Euclidean Distance
        dist = np.linalg.norm(current - target)
        normalized_distance = dist/magnitude

        reward = self.skew_reward(1 - normalized_distance,self.reward_skew)

        if self.step_count >= self.max_steps_per_episode:
            terminated = True
            self.info['terminate_cond'] = f"COND: MAX STEPS REACHED, CLOSENESS:{normalized_distance:.3f}"
            return reward, truncated, terminated ;
        if self.last_move_illegal:  # INVALID MOVE
            self.info['terminate_cond'] = "COND: ILLEGAL MOVE"
            return -2.0, truncated, terminated
        if dist < self.goal_discrepency_threshold:
            truncated = True
            self.info['terminate_cond'] = "COND: GOAL REACHED"
            reward = 10.0
            return reward, truncated, terminated

        self.info['terminate_cond'] = "COND: Normal" ;
        return reward, truncated, terminated ;

    def skew_reward(self, reward, base=2):
        return np.power(reward,base)

    def parse_args(self, **kwargs):
        parser = Kwarg_Parser(**kwargs)
        # ------------------------------------ LEARNER
        parser.add_argument('--goal_discrepency_threshold', type=float,default=0.15, help='The distance between goal and hand that is considered touching.')
        parser.add_argument('--action_speed', type=float, default=0.1, help='Defines the distance the hand can move per action in predetermined directions.')
        parser.add_argument('--reward_skew', type=float, default=1.0, help="Allows for the distance reward function to skew the reward closer to 1 or 0 with a pow function parameter. This value is the power.")
        cfg,unparsed = parser.parse_known_args() ; 

        # parse superclass params
        old_cfg = GenericEnvironment.parse_args(self, **kwargs) ;
        
        for attr in dir(old_cfg):
          # exclude magic methods and private fields
          if len(attr) > 2 and (attr[0] != "_" and attr[1] != "_"):
            setattr(cfg, attr,getattr(old_cfg, attr)) ;
        return cfg ;

    def close(self):
        pass
