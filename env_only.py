"""
CRoSS benchmark suite, HLR  benchmark Example for standalone use of provided
robot manager, without RL framework
"""

from src.cross_kinematic_high_level_reaching.Environment import RobotArmEnvironment

end=False

args_dict = {"task_list":["hammer","push"],
             "action_speed":"0.1",
             "reward_skew":"1.0",
             "max_steps_per_episode":"30",
             "goal_discrepency_threshold":"0.1"}

env = RobotArmEnvironment(**args_dict)

print("Switching to Task 0: ", env.task_list[0])
env.switch(0) ;

iter = 0
terminated = False
truncated = False
while not end:
    if terminated or truncated: env.reset()
    action_index = 2 # For the possible actions look in Environment.py
    obs, reward, terminated, truncated, info = env.step(action_index)
    print(f"--- Step {iter}\nAction {action_index}\nState: {obs}\nReward: {reward}\n")

    if iter >= 30:
        end=True
        break
    iter+=1