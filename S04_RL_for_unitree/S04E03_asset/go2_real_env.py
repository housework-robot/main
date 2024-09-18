import numpy as np
import json
import torch

import gymnasium as gym
from gymnasium import spaces

class Go2RealEnv(gym.Env):
    fake_obs_dict = {
        'shape': [48, 1], 
        'base_lin_vel': [[-0.5578, -0.8615, -0.0395]], 
        'base_ang_vel': [[-0.5672,  0.1676, -0.0858]], 
        'projected_gravity': [[ 0.0587,  0.0334, -1.0118]], 
        'velocity_commands': [[-0.5736, -0.8130,  0.0198]], 
        'joint_pos': [[-0.1076,  0.2249,  0.1174, -0.1215,  0.4330, -0.4039, -0.0043, -0.2249, -0.3630, -0.2957, -0.2046, -0.2444]], 
        'joint_vel': [[-3.6872,  0.8345,  0.9783,  3.4316,  0.9197,  0.7776, -0.0279, -1.5330, -2.3675,  0.7889, -1.4047, -3.2064]], 
        'actions': [[-1.2007,  0.1980,  0.1681,  0.8183,  1.4110, -0.2133,  0.4731, -1.4501, -1.1923, -0.0632, -0.5582,  0.3421]]
    }

    fake_obs_tensor = torch.tensor([[-0.5578, -0.8615, -0.0395, -0.5672,  0.1676, -0.0858, 0.0587,  0.0334, -1.0118, -0.5736, -0.8130,  0.0198, -0.1076,  0.2249,  0.1174, -0.1215,  0.4330, -0.4039, -0.0043, -0.2249, -0.3630, -0.2957, -0.2046, -0.2444, -3.6872,  0.8345,  0.9783,  3.4316,  0.9197,  0.7776, -0.0279, -1.5330, -2.3675,  0.7889, -1.4047, -3.2064, -1.2007,  0.1980,  0.1681,  0.8183,  1.4110, -0.2133,  0.4731, -1.4501, -1.1923, -0.0632, -0.5582,  0.3421]])


    def __init__(self, rl_cfg_file: str):
        print(f"\n >> Go2RealEnv() \n")
        self.num_envs = 1
        self.num_actions = 12

        # Observations are dictionaries with the agent's and the target's location.
        # Each location is encoded as an element of {0, ..., `size`}^2, i.e. MultiDiscrete([size, size]).
        """
        【Kan】observation_space: 'Dict('policy': Box(-inf, inf, (1, 48), float32))'
        【Kan】action_space: 'Box(-inf, inf, (1, 12), float32)' 
        """
        self.observation_space = spaces.Dict(
            {
                "policy": spaces.Box(-1.0 * np.inf, np.inf, shape=(1,48), dtype=float)
            }
        )
        self.action_space = spaces.Box(-1.0 * np.inf, np.inf, shape=(1,12), dtype=float)

        self.rl_cfg = {}
        with open(rl_cfg_file) as fi:
            self.rl_cfg = json.load(fi)
            print(f"\n 【Kan】rl_cfg: {self.rl_cfg} \n\n")


    def get_observations(self):
        return self.fake_obs_tensor, {"observations": self.fake_obs_dict}
    

    def reset(self, seed=None, options=None):
        obs, info = self.get_observations()
        return obs, info

      
    def step(self, action):
        reward = None
        terminated = None
        return self.fake_obs_tensor, reward, terminated, False, self.fake_obs_dict        
    

    def close(self):
        print(f"\n[INFO] go2_real_env is closed.\n")
      
