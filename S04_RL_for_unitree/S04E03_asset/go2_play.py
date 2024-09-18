import torch
import json
import gymnasium as gym
from unitree.go2_real_env import Go2RealEnv
from rsl_rl.runners import OnPolicyRunner

go2_env = Go2RealEnv("unitree/rsl_rl_ppo_cfg.json")
print(f"\n【Kan】observation_space: '{go2_env.observation_space}'")
print(f"【Kan】action_space: '{go2_env.action_space}' \n")

print(f"\n 【Kan】go2_env.get_observations(): {go2_env.get_observations()} \n")
go2_ppo_runner = OnPolicyRunner(go2_env, go2_env.rl_cfg, log_dir=None, device="cuda:0")

resume_path = "/home/robot/IsaacLab/logs/rsl_rl/unitree_go2_flat/2024-09-01_16-32-19/model_299.pt"
print(f"[INFO]: Loading model checkpoint from resume_path: {resume_path}")
go2_ppo_runner.load(resume_path)  
# go2_policy = go2_ppo_runner.get_inference_policy("cuda:0")
go2_policy = go2_ppo_runner.get_inference_policy("cpu")
print(f"Successfully load the model with checkpoint.")

# reset environment
obs, _ = go2_env.get_observations()
timestep = 0
# simulate environment
while timestep < 10:
   # run everything in inference mode
   with torch.inference_mode():
      # agent stepping
      go2_actions = go2_policy(obs)
      print(f" 【Kan】go2_actions: {go2_actions} \n\n")

      # env stepping
      obs, _, _, _, _ = go2_env.step(go2_actions)
      print(f"【Kan】obs: {obs} \n")
      timestep += 1

# close the simulator
go2_env.close()
