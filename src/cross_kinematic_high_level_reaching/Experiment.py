from datetime import datetime

# ICRL imports
from gazebo_sim.agent import RLAgent ;

from gazebo_sim.learner import DQNLearner
from cross_kinematic_high_level_reaching.Environment import RobotArmEnvironment
from cl_experiment.parsing import Command_Line_Parser, Kwarg_Parser


def main():
    print(f'Begin execution at: {datetime.now()}')
    args_dict = Command_Line_Parser().parse_args()
    p = Kwarg_Parser(**args_dict)
    p.add_argument("--algorithm", type=str, required=True) ;
    config, unparsed = p.parse_known_args() ;
    
    env = RobotArmEnvironment(**args_dict)

    # instantiate learner
    if config.algorithm == "DQN":
        learner = DQNLearner(n_actions=len(env.robot.actions),
                                obs_space=env.get_input_dims(),
                                config=None, **args_dict) ;
    
    elif config.algorithm == "AR":
        print("NOT IMPLEMENTED IN THIS SCENARIO")
    
    elif config.algorithm == "X":
        print("NOT IMPLEMENTED IN THIS SCENARIO")

    # instantiate agent
    agent = RLAgent(env, learner, **args_dict)

    # execute experiment
    agent.go()
    print(f'Finish execution at: {datetime.now()}')
    agent.mop_up(); # Terminates debug thread so program can exit

if __name__== "__main__":
    main()