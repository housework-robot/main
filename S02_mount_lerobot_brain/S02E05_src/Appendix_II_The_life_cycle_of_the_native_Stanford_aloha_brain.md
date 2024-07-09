
# Appendix II. The life cycle of the native Stanford aloha brain

## 1. The input and output of native Aloha

Following the user guide from the official [Stanford Aloha ACT GitHub](https://github.com/tonyzhaozh/act?tab=readme-ov-file#simulated-experiments) website, we execute the following command to experience how to use the actions output by the ACT model.

~~~
# (aloha) robot@robot-test:~/act-main$ python3 imitate_episodes.py \
    --task_name sim_insertion_scripted \
    --ckpt_dir ./ckpt \
    --policy_class ACT \
    --kl_weight 10  \
    --chunk_size 100 \
    --hidden_dim 512 \ 
    --batch_size 8  \ 
    --dim_feedforward 3200 \ 
    --num_epochs 2000 \ 
    --lr 1e-5 \
    --seed 0 \
    --eval \
    --onscreen_render
~~~


We modified the source code of `imitate_episodes.py`, print out the iput and output of `ACT` model, as well as their data formats. 

~~~
# /home/robot/act-main/imitate_episodes.py

def eval_bc(config, ckpt_name, save_episode=True):
    policy_class = config['policy_class']
    policy_config = config['policy_config']
    max_timesteps = config['episode_len']
    task_name = config['task_name']
    ...
    # load environment
    if real_robot:
        ...
        env = make_real_env(init_node=True)
        env_max_reward = 0
    else:
        from sim_env import make_sim_env
        env = make_sim_env(task_name)
        env_max_reward = env.task.max_reward

    query_frequency = policy_config['num_queries']        
    num_rollouts = 50
    for rollout_id in range(num_rollouts):
        rollout_id += 0
        
        ts = env.reset()    
        with torch.inference_mode():
            for t in range(max_timesteps):
                ### process previous timestep to get qpos and image_list
                obs = ts.observation        
                                
                qpos_numpy = np.array(obs['qpos'])
                qpos = pre_process(qpos_numpy)
                curr_image = get_image(ts, camera_names)
                
                ### query policy
                if config['policy_class'] == "ACT":
                    if t % query_frequency == 0:
                        print(f"\n[{t}] Policy inputs:\n")
                        print(f"   'qpos': shape: {qpos.shape}, dtype: {qpos.dtype} \n")
                        print(qpos)
                        print(f"   'curr_image': shape {curr_image.shape}, dtype: {curr_image.dtype}")
                        print(curr_image)

                        all_actions = policy(qpos, curr_image)

                        print(f"\n[{t}] Policy outputs:\n")
                        print(f"   'all_actions': shape: {all_actions.shape}, dtype: {all_actions.dtype} \n")
                        print(all_actions)
                        print(f"\n") 
                        
                ### post-process actions
                raw_action = raw_action.squeeze(0).cpu().numpy()
                action = post_process(raw_action)
                target_qpos = action

                ### step the environment
                ts = env.step(target_qpos)               
~~~


The following is the execution result,

~~~
[0] Policy inputs:

   'qpos': shape: torch.Size([1, 14]), dtype: torch.float32
   
tensor([[-0.2335, -0.8679,  1.2146,  0.2200, -4.4009, -0.2200, -2.4127,  0.0991,
         -0.8481,  1.1700, -0.0797, -4.2769,  0.0921, -1.4997]],
       device='cuda:0')
       
   'curr_image': shape torch.Size([1, 1, 3, 480, 640]), dtype: torch.float32
   
tensor([[[[[0., 0., 0.,  ..., 0., 0., 0.],
           [0., 0., 0.,  ..., 0., 0., 0.],
           ...,
           [0., 0., 0.,  ..., 0., 0., 0.]]]]], device='cuda:0')

[0] Policy outputs:

   'all_actions': shape: torch.Size([1, 100, 14]), dtype: torch.float32
   
tensor([[[-0.2376, -2.3184,  1.5074,  ...,  0.0986,  0.0097, -0.5107],
         [-0.2230, -2.3310,  1.5066,  ...,  0.0763,  0.0217, -0.5318],
         ...,
         [ 0.6262, -0.1672,  0.6320,  ...,  0.5909,  0.5287,  1.6766]]],
       device='cuda:0')

~~~



## 2. The creation of native Aloha brain

Read the source code of [imitate_episodes.py](https://github.com/tonyzhaozh/act/blob/main/imitate_episodes.py), in order to understand the creation process of Action Chunking with Transformers (ACT) model. 

~~~
# /home/robot/act-main/imitate_episodes.py

from policy import ACTPolicy

def make_policy(policy_class, policy_config):
    if policy_class == 'ACT':
        policy = ACTPolicy(policy_config)
    ...
    return policy
   
    
def eval_bc(config, ckpt_name, save_episode=True):
    ckpt_dir = config['ckpt_dir']
    policy_class = config['policy_class']
    policy_config = config['policy_config']
    max_timesteps = config['episode_len']
    task_name = config['task_name']

    # load policy and stats
    ckpt_path = os.path.join(ckpt_dir, ckpt_name)
    policy = make_policy(policy_class, policy_config)
    loading_status = policy.load_state_dict(torch.load(ckpt_path))
 
def main(args):
    ckpt_dir = args['ckpt_dir']
    policy_class = args['policy_class']
    task_name = args['task_name']
    batch_size_train = args['batch_size']
    batch_size_val = args['batch_size']
    num_epochs = args['num_epochs']      
    
    # fixed parameters
    state_dim = 14
    lr_backbone = 1e-5
    backbone = 'resnet18'
    if policy_class == 'ACT':
        enc_layers = 4
        dec_layers = 7
        nheads = 8

        policy_config = {'lr': args['lr'],
                         'num_queries': args['chunk_size'],
                         'kl_weight': args['kl_weight'],
                         'hidden_dim': args['hidden_dim'],
                         'dim_feedforward': args['dim_feedforward'],
                         'lr_backbone': lr_backbone,
                         'backbone': backbone,
                         'enc_layers': enc_layers,
                         'dec_layers': dec_layers,
                         'nheads': nheads,
                         'camera_names': camera_names,
                         }     
                         
    config = {
        'num_epochs': num_epochs,
        'ckpt_dir': ckpt_dir,
        'episode_len': episode_len,
        'state_dim': state_dim,
        'lr': args['lr'],
        'policy_class': policy_class,
        'onscreen_render': onscreen_render,
        'policy_config': policy_config,
        'task_name': task_name,
        'seed': args['seed'],
        'temporal_agg': args['temporal_agg'],
        'camera_names': camera_names,
        'real_robot': not is_sim
    }
    
    if is_eval:
        ckpt_names = [f'policy_best.ckpt']
        results = []
        for ckpt_name in ckpt_names:
            success_rate, avg_return = eval_bc(config, ckpt_name, save_episode=True)
            ...   
        exit()                                      
~~~


Actually it is quite simple to create ACT model.

First, set up the model parameters `policy_config`, then it is time to create an instance of `ACTPolicy`. 

~~~
from policy import ACTPolicy
policy = ACTPolicy(policy_config)
~~~




## 3. The usage of native Aloha brain

### 3.1 the input and output of native Aloha's env

Read the source code of the Stanford aloha system, and print out the input and output, both their data formats and data content, of `env.step()`. 

~~~
# /home/robot/act-main/imitate_episodes.py

def eval_bc(config, ckpt_name, save_episode=True):
    ckpt_dir = config['ckpt_dir']
    real_robot = config['real_robot']
    policy_class = config['policy_class']
    policy_config = config['policy_config']
    max_timesteps = config['episode_len']
    task_name = config['task_name']

    num_rollouts = 50
    for rollout_id in range(num_rollouts):
        rollout_id += 0

        ts = env.reset()
        with torch.inference_mode():
            for t in range(max_timesteps):       
                ### query policy
                if config['policy_class'] == "ACT":
                    if t % query_frequency == 0:
                        all_actions = policy(qpos, curr_image)
                ...
                 
                ### post-process actions
                raw_action = raw_action.squeeze(0).cpu().numpy()
                action = post_process(raw_action)
                target_qpos = action

                ### step the environment
                print(f"[{t}] target_qpos: {target_qpos} \n")
                print(f"  target_qpos.shape: {target_qpos.shape} \n")
                ts = env.step(target_qpos)
                print(f"[{t}] ts: {ts} \n")            
~~~


Following is the execution result, 

~~~
# Env input:

[0] target_qpos: [-9.3384460e-04 -1.7078998e+00  1.2308732e+00  4.6417303e-03
  5.6922406e-01 -1.3676301e-02  1.6363129e-01 -1.0724664e-03
 -1.7091711e+00  1.2568194e+00  2.2574503e-02  5.7522881e-01
 -3.0301604e-02  1.3787243e-01] 

  target_qpos.shape: (14,) 

# Env output:

 'ts': is an instance of TimeStep which consists of 4 attributes
    
  1. step_type: a Tensor or array of StepType enum values.
     The first TimeStep in a sequence will equal StepType.FIRST. The final TimeStep will equal StepType.LAST. All other TimeSteps in a sequence will equal `StepType.MID.
  2. reward: a Tensor or array of reward values.
  3. discount: a discount value in the range [0, 1].
  4. observation: a NumPy array, or a nested dict, list or tuple of arrays.
     
e.g. ts.step_type: 1, ts.reward: 0, ts.discount: 1.0 
     ts.observation.keys(): odict_keys(['qpos', 'qvel', 'env_state', 'images']) 

[0] ts: TimeStep(step_type=<StepType.MID: 1>, 
     reward=0, 
     discount=1.0, 
     observation=OrderedDict([
       ('qpos', array([-2.45689374e-04, -1.54264617e+00,  1.17960026e+00,  2.31387960e-02,
        5.53493980e-01, -1.52162980e-02,  1.01577523e-01, -2.76312779e-04,
       -1.54770313e+00,  1.19695550e+00,  3.43526111e-02,  5.27472879e-01,
       -3.15232887e-02,  9.83908518e-02])), ('qvel', array([-1.70046630e-02, -5.15807537e+01,  2.43335777e+00,  2.12908711e+00,
        3.28384074e+01, -5.25420991e-01, -8.81957642e-02, -1.90028883e-02,
       -5.23950068e+01,  4.07555151e+00,  2.70369948e+00,  3.14341535e+01,
       -1.18558891e+00, -2.91378338e-01])), 
       ('env_state', array([ 0.16535896,  0.42300139,  0.0478858,  1.,  0.,
        0.,  0., -0.15178086,  0.57449491,  0.0478858,
        1.,  0.,  0.,  0.])), 
       ('images', 
          {'top': array([[
              [0, 0, 0],
              [0, 0, 0],
              [0, 0, 0],
              ...
             ]], dtype=uint8), 
           'angle': array([[
              [0, 0, 0],
              [0, 0, 0],
              [0, 0, 0],
              ...
             ]], dtype=uint8), 
           'vis': array([[
              [ 0,  0,  0],
              [ 0,  0,  0],
              [ 0,  0,  0],
              ...
             ]], dtype=uint8)
          }
        )
       ])
     ) 
~~~  

### 3.2 the creation process of native Aloha env

~~~
# /home/robot/act-main/imitate_episodes.py

def eval_bc(config, ckpt_name, save_episode=True):
    ckpt_dir = config['ckpt_dir']
    real_robot = config['real_robot']
    policy_class = config['policy_class']
    policy_config = config['policy_config']
    max_timesteps = config['episode_len']
    task_name = config['task_name']
    
    # load environment
    if real_robot:
        ...
        env = make_real_env(init_node=True)
        env_max_reward = 0
    else:
        from sim_env import make_sim_env
        env = make_sim_env(task_name)
        env_max_reward = env.task.max_reward 
~~~

~~~
# /home/robot/act-main/sim_env.py

from dm_control import mujoco
from dm_control.rl import control
from dm_control.suite import base

def make_sim_env(task_name):
    """
    Environment for simulated robot bi-manual manipulation, with joint position control
    Action space:      [left_arm_qpos (6),             # absolute joint position
                        left_gripper_positions (1),    # normalized gripper position (0: close, 1: open)
                        right_arm_qpos (6),            # absolute joint position
                        right_gripper_positions (1),]  # normalized gripper position (0: close, 1: open)

    Observation space: {"qpos": Concat[ left_arm_qpos (6),         # absolute joint position
                                        left_gripper_position (1),  # normalized gripper position (0: close, 1: open)
                                        right_arm_qpos (6),         # absolute joint position
                                        right_gripper_qpos (1)]     # normalized gripper position (0: close, 1: open)
                        "qvel": Concat[ left_arm_qvel (6),         # absolute joint velocity (rad)
                                        left_gripper_velocity (1),  # normalized gripper velocity (pos: opening, neg: closing)
                                        right_arm_qvel (6),         # absolute joint velocity (rad)
                                        right_gripper_qvel (1)]     # normalized gripper velocity (pos: opening, neg: closing)
                        "images": {"main": (480x640x3)}        # h, w, c, dtype='uint8'
    """
    if 'sim_transfer_cube' in task_name:
        xml_path = os.path.join(XML_DIR, f'bimanual_viperx_transfer_cube.xml')
        physics = mujoco.Physics.from_xml_path(xml_path)
        task = TransferCubeTask(random=False)
        env = control.Environment(physics, task, time_limit=20, control_timestep=DT,
                                  n_sub_steps=None, flat_observation=False)
                                  
    elif 'sim_insertion' in task_name:
        xml_path = os.path.join(XML_DIR, f'bimanual_viperx_insertion.xml')
        physics = mujoco.Physics.from_xml_path(xml_path)
        task = InsertionTask(random=False)
        env = control.Environment(physics, task, time_limit=20, control_timestep=DT,
                                  n_sub_steps=None, flat_observation=False)

    return env    
~~~

