
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


