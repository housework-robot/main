import torch
import numpy as np
import os
import pickle
import argparse
import matplotlib.pyplot as plt
from copy import deepcopy
from tqdm import tqdm
from einops import rearrange
import random

from constants import DT
from constants import PUPPET_GRIPPER_JOINT_OPEN
from utils import load_data # data functions
from utils import sample_box_pose, sample_insertion_pose # robot functions
from utils import compute_dict_mean, set_seed, detach_dict # helper functions
# from policy import ACTPolicy, CNNMLPPolicy
from visualize_episodes import save_videos

from sim_env import BOX_POSE
import pprint

import IPython
e = IPython.embed

from lerobot.common.policies.factory import make_policy
from lerobot.common.policies.policy_protocol import Policy
from lerobot.common.policies.utils import get_device_from_parameters
from lerobot.common.utils.io_utils import write_video
from lerobot.common.utils.utils import get_safe_torch_device, init_hydra_config, init_logging, set_global_seed

def make_optimizer(policy_class, policy):
    if policy_class.upper() == 'ACT':
        optimizer = policy.configure_optimizers()
    else:
        raise NotImplementedError
    return optimizer

def get_image(ts, camera_names):
    curr_images = []
    for cam_name in camera_names:
        curr_image = rearrange(ts.observation['images'][cam_name], 'h w c -> c h w')
        curr_images.append(curr_image)
    curr_image = np.stack(curr_images, axis=0)
    curr_image = torch.from_numpy(curr_image / 255.0).float().cuda().unsqueeze(0)
    return curr_image

def eval_bc(config, ckpt_name, save_episode=True):
    # Generate a random integer between 0 to 9
    rand_seed = random.randrange(9)
    set_seed(rand_seed)
    
    # command line parameters
    ckpt_dir = config['ckpt_dir']
    batch_size_val = config['batch_size']
    num_episodes = config['num_episodes']

    policy_config = init_hydra_config(
        f"{ckpt_dir}/config.yaml", 
        None
    )

    policy_class = policy_config.policy.name

    policy = make_policy(
        hydra_cfg=policy_config, 
        pretrained_policy_name_or_path=str(ckpt_dir)
    )      

    print(f"[imitate_lerobot] type(policy): {type(policy)} \n")      

    policy.cuda()
    policy.eval()

    # load simulation environment
    from sim_env import make_sim_env
    task_name = policy_config.env.task_name
    print(f"[imitate_lerobot] task_name: {task_name} \n")
    env = make_sim_env(task_name)
    env_max_reward = env.task.max_reward

    from constants import SIM_TASK_CONFIGS
    task_config = SIM_TASK_CONFIGS[task_name]
    camera_names = task_config['camera_names']
    camera_name = camera_names[0]

    query_frequency = policy_config.policy.chunk_size
    
    max_timesteps = task_config['episode_len']
    max_timesteps = int(max_timesteps * 1) # may increase for real-world tasks

    num_rollouts = 50
    episode_returns = []
    highest_rewards = []
    for rollout_id in range(num_rollouts):
        rollout_id += 0
        ### set task
        if 'sim_transfer_cube' in task_name:
            BOX_POSE[0] = sample_box_pose() # used in sim reset
        elif 'sim_insertion' in task_name:
            BOX_POSE[0] = np.concatenate(sample_insertion_pose()) # used in sim reset

        ts = env.reset()

        ### onscreen render
        ax = plt.subplot()
        plt_img = ax.imshow(env._physics.render(height=480, width=640, camera_id=camera_name))
        plt.ion()

        image_list = [] # for visualization
        qpos_list = []
        target_qpos_list = []
        rewards = []
      
        with torch.inference_mode():
            for t in range(max_timesteps):
                ### update onscreen render and wait for DT
                image = env._physics.render(height=480, width=640, camera_id=camera_name)
                plt_img.set_data(image)
                plt.pause(DT)

                ### process previous timestep to get qpos and image_list
                obs = ts.observation

                if 'images' in obs:
                    image_list.append(obs['images'])
                
                qpos_numpy = np.array(obs['qpos'])
                qpos = torch.from_numpy(qpos_numpy).float().cuda().unsqueeze(0)
                curr_image = get_image(ts, camera_names)

                ### query policy
                if policy_class.upper() == "ACT":
                    if t % query_frequency == 0:
                        print(f"\n[{t}] Policy inputs:\n")

                        """
                        print(f"   'qpos': shape: {qpos.shape}, dtype: {qpos.dtype}")
                        print(qpos)
                        print(f"   'curr_image': shape {curr_image.shape}, dtype: {curr_image.dtype}")
                        print(curr_image)                        
                        """

                        top_image = torch.squeeze(curr_image, dim=1)
                        obs = {
                            'observation.images.top': top_image,
                            'observation.state': qpos 
                        }
                        print(obs)
                        print(f" obs['observation.images.top'].shape: {obs['observation.images.top'].shape} \n")
                        print(f" obs['observation.state'].shape: {obs['observation.state'].shape} \n")

                        all_actions = policy.select_action(obs)

                        print(f"\n[{t}] Policy outputs:\n")
                        print(f"   'all_actions': shape: {all_actions.shape}, dtype: {all_actions.dtype}")
                        print(all_actions)
                        print(f"\n")

                        target_qpos = all_actions.squeeze(0).cpu().numpy()
                        print(f"[imitate_lerobot] target_qpos: {target_qpos} \n")

                else:
                    raise NotImplementedError

                ### step the environment
                print(f"[{t}] target_qpos: {target_qpos} \n")
                print(f"  target_qpos.shape: {target_qpos.shape} \n")
                ts = env.step(target_qpos)

                print(f"[{t}] ts: {ts} \n")
                print(f"[{t}] ts.step_type: {ts.step_type}, ts.reward: {ts.reward}, ts.discount: {ts.discount} \n")
                print(f"[{t}] ts.observation.keys(): {ts.observation.keys()} \n")

                ### for visualization
                qpos_list.append(qpos_numpy)
                target_qpos_list.append(target_qpos)
                rewards.append(ts.reward)

            plt.close()

        rewards = np.array(rewards)
        episode_return = np.sum(rewards[rewards!=None])
        episode_returns.append(episode_return)
        episode_highest_reward = np.max(rewards)
        highest_rewards.append(episode_highest_reward)
        print(f'Rollout {rollout_id}\n{episode_return=}, {episode_highest_reward=}, {env_max_reward=}, Success: {episode_highest_reward==env_max_reward}')

        if save_episode:
            save_videos(image_list, DT, video_path=os.path.join(ckpt_dir, f'video{rollout_id}.mp4'))

    success_rate = np.mean(np.array(highest_rewards) == env_max_reward)
    avg_return = np.mean(episode_returns)
    summary_str = f'\nSuccess rate: {success_rate}\nAverage return: {avg_return}\n\n'

    for r in range(env_max_reward+1):
        more_or_equal_r = (np.array(highest_rewards) >= r).sum()
        more_or_equal_r_rate = more_or_equal_r / num_rollouts
        summary_str += f'Reward >= {r}: {more_or_equal_r}/{num_rollouts} = {more_or_equal_r_rate*100}%\n'

    print(summary_str)

    # save success rate to txt
    result_file_name = 'result_' + ckpt_name.split('.')[0] + '.txt'
    with open(os.path.join(ckpt_dir, result_file_name), 'w') as f:
        f.write(summary_str)
        f.write(repr(episode_returns))
        f.write('\n\n')
        f.write(repr(highest_rewards))

    return success_rate, avg_return


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--ckpt_dir', action='store', type=str, help='ckpt_dir', required=True)  
    parser.add_argument('--batch_size', action='store', type=int, help='batch_size', required=True)
    parser.add_argument('--num_episodes', action='store', type=int, help='num_episodes', required=True)

    config = vars(parser.parse_args())
    ckpt_name = f'policy_best.ckpt'

    success_rate, avg_return = eval_bc(config, ckpt_name, save_episode=True)
    result = [ckpt_name, success_rate, avg_return]
    print(f"Aloha with Lerobot's result: {result} \n")
