# 1. Connecting the robot brain with the body

In [the previous article](https://github.com/housework-robot/main/blob/main/S02_mount_lerobot_brain/S02E02_dataformat_lerobot.md), we analyzed the file format of LeRobot training dataset to understand the input and output data formats of the LeRobot robot brain.

The theme of this article is to analyze the input and output data formats of the Stanford aloha bimanual robot. As a case study, the objective is to show how to analyze the data format of a native robot body. 

LeRobot's partner, [Dora-rs](https://github.com/dora-rs/dora), has reimplemented the software system for the Stanford aloha robot with [dora-lerobot](https://github.com/dora-rs/dora-lerobot), a brand new robot system framework whose performance is 17 times faster than ROS2. 

However, this article analyzes the data format of the native software system of Stanford aloha robot, not dora-lerobot.

Our motivation for doing this is,

1. Although the dora-lerobot body system has significant advantages over the Stanford aloha native body system, there are still many robot bodies that have not yet been upgraded to the dora-lerobot system,

2. To connect the LeRobot robot brain to various robot bodies, it is necessary to convert the output data of the robot brain, into input data that the native robot body can accept,

    Meanwhile, the output data of the robot body must be converted into the input data that LeRobot can accept.

3. Connecting the LeRobot robot brain to the Stanford aloha robot body is just a case study. By this practice, we will get familiar with the operation of mounting LeRobot brain to any native robot body.


# 2. Stanford aloha native dataset

## 2.1 Data structure of the native real training Dataset

On [the GitHub homepage](https://mobile-aloha.github.io/) of the Stanford mobile aloha project, there is a link to some training datasets they collected. These training datasets were obtained through manual operation of the Aloha robot bimanual robot.

![alt text](image-1.png)

Additionally, if for the time being we do not focus on the mobile base of aloha robot, but only focus on the motion of its dual arms, then we can find the simulation dataset the Stanford team collected, on [the ACT GitHub homepage](https://github.com/tonyzhaozh/act), which is used for the training of the Action Chunking with Transformers (ACT) motion planning model.

![alt text](image-2.png)

We first downloaded [the aloha_mobile_shrimp_truncated dataset](https://drive.google.com/drive/folders/1FP5eakcxQrsHyiWBRDsMRvUfSxeykiDc) from the real dataset collection, which includes an MP4 video file and several [HDF5](https://docs.hdfgroup.org/hdf5/v1_14/_getting_started.html) files.

We used a Python program shared by an engineer on Stackoverflow that utilizes [the h5py library](https://docs.h5py.org/en/stable/quick.html) to open the HDF5 files.

~~~
import h5py

data_dir = "/Users/dengkan/Downloads"
data_file = f"{data_dir}/aloha_mobile_shrimp_truncated/episode_2.hdf5"

def h5_tree(val, pre=''):
    items = len(val)
    for key, val in val.items():
        items -= 1
        if items == 0:
            # the last item
            if type(val) == h5py._hl.group.Group:
                print(pre + '└── ' + key)
                h5_tree(val, pre+'    ')
            else:
                try:
                    print(pre + '└── ' + key + f': {val.shape} * {val[0].dtype}')
                except TypeError:
                    print(pre + '└── ' + key + ' (scalar)')
        else:
            if type(val) == h5py._hl.group.Group:
                print(pre + '├── ' + key)
                h5_tree(val, pre+'│   ')
            else:
                try:
                    print(pre + '└── ' + key + f': {val.shape} * {val[0].dtype}')
                except TypeError:
                    print(pre + '├── ' + key + ' (scalar)')

with h5py.File(data_file, 'r') as hf:
    print(f'\n{data_file} : \n')
    h5_tree(hf)
    print('\n')                    
~~~

Here are the results of the execution:

~~~
$ python3 peek_hdf5.py

/Users/dengkan/Downloads/aloha_mobile_shrimp_truncated/episode_2.hdf5 :

└── action: (3750, 14) * float32
└── base_action: (3750, 2) * float32
└── compress_len: (3, 4500) * float32
└── observations
    └── effort: (3750, 14) * float32
    ├── images
    │   └── cam_high: (3750, 21167) * uint8
    │   └── cam_left_wrist: (3750, 21167) * uint8
    │   └── cam_right_wrist: (3750, 21167) * uint8
    └── qpos: (3750, 14) * float32
    └── qvel: (3750, 14) * float32
~~~

The result indicates that this training dataset contains the system states at 3,750 moments. A system state includes a motion action, and an action consists of the parameters of 14 joints.


## 2.2 Data structure of the native simulation training dataset

We use the same program to inspect the data structure of the simulation data.

~~~
$ python3 peek_hdf5.py

/Users/dengkan/Downloads/sim_insertion_human/episode_2.hdf5 :

└── action: (500, 14) * float32
└── observations
    ├── images
    │   └── top: (500, 480, 640, 3) * uint8
    └── qpos: (500, 14) * float32
    └── qvel: (500, 14) * float32
~~~

The result shows that this simulation training dataset contains the system states at 500 moments, and a system state includes an action, with an action consists of the parameters for 14 joints.

Additionally, it is interesting to note that in the simulation training dataset, each video frame is represented as a tensor of shape (480, 640, 3); whereas in the real training dataset, each video frame is flattened into a one-dimensional array with 21,167 elements.


# 3. Visualizing the contents of the Stanford aloha native dataset

Download [the ACT code](https://github.com/tonyzhaozh/act) from Github, set up the environment following the user guide, and then view the dataset.

~~~
$ conda activate aloha
$ pwd
/home/robot/act-main

$ python3 visualize_episodes.py --dataset_dir /home/robot/lerobot/dataset/sim_insertion_scripted --episode_idx 0
Saved video to: /home/robot/lerobot/dataset/sim_insertion_scripted/episode_0_video.mp4
Saved qpos plot to: /home/robot/lerobot/dataset/sim_insertion_scripted/episode_0_qpos.png
~~~

[[video]]

![alt text](image-3.png)


# 4. Training the ACT model with the Stanford aloha native training dataset

## 4.1 The training process of the ACT model

Modify [the act-main/constants.py code](https://github.com/tonyzhaozh/act/blob/main/constants.py) to specify the DATA_DIR, and then train the ACT model for [sim_insertion_scripted task](https://github.com/huggingface/lerobot/tree/main/tests/data/lerobot/aloha_sim_insertion_scripted).

~~~
import pathlib

### Task parameters
DATA_DIR = '/home/robot/lerobot/dataset'
SIM_TASK_CONFIGS = {
    ...
    'sim_insertion_scripted': {
        'dataset_dir': DATA_DIR + '/sim_insertion_scripted',
        'num_episodes': 50,
        'episode_len': 400,
        'camera_names': ['top']
    },
    ...
}
...
~~~

Following is the training process:

~~~
$ conda activate aloha
$ pwd
/home/robot/act-main

$ python3 imitate_episodes.py --task_name sim_insertion_scripted --ckpt_dir ./ckpt --policy_class ACT --kl_weight 10 --chunk_size 100 --hidden_dim 512 --batch_size 8 --dim_feedforward 3200 --num_epochs 350  --lr 1e-5 --seed 0

Data from: /home/robot/lerobot/dataset/sim_insertion_scripted

/home/robot/.local/lib/python3.10/site-packages/torchvision/models/_utils.py:208: UserWarning: The parameter 'pretrained' is deprecated since 0.13 and may be removed in the future, please use 'weights' instead.
  warnings.warn(
/home/robot/.local/lib/python3.10/site-packages/torchvision/models/_utils.py:223: UserWarning: Arguments other than a weight enum or `None` for 'weights' are deprecated since 0.13 and may be removed in the future. The current behavior is equivalent to passing `weights=ResNet18_Weights.IMAGENET1K_V1`. You can also use `weights=ResNet18_Weights.DEFAULT` to get the most up-to-date weights.
  warnings.warn(msg)
number of parameters: 83.92M
KL Weight 10
  0%|                                                                                                                                                                                                                   | 0/350 [00:00<?, ?it/s]
Epoch 0
Val loss:   80.88815
l1: 1.113 kl: 7.978 loss: 80.888 
Train loss: 52.35494
l1: 0.823 kl: 5.153 loss: 52.355 
Saved plots to ./ckpt
  0%|▌                                                                                                                                                                                                          | 1/350 [00:02<13:49,  2.38s/it]
Epoch 1
Val loss:   38.44741
l1: 0.673 kl: 3.777 loss: 38.447 
Train loss: 29.08282
l1: 0.676 kl: 2.841 loss: 29.083 
  1%|█▏   

...

Epoch 349
Val loss:   0.33279
l1: 0.155 kl: 0.018 loss: 0.333 
Train loss: 1.55908
l1: 0.120 kl: 0.144 loss: 1.559 
100%|█████████████████████████████████████████████████████████████████████████████████████████████████████████████████████████████████████████████████████████████████████████████████████████████████████████| 350/350 [08:07<00:00,  1.39s/it]
Training finished:
Seed 0, val loss 0.142372 at epoch 310
Saved plots to ./ckpt
Best ckpt, val loss 0.142372 @ epoch310
   
~~~

We finished the training process prematurally with only 350 epochs. The reason is that our objective for the time being, is to analyze the input and output data formats of the ACT model, rather than training out an accurate ACT model.


## 4.2 The training result of the ACT model

~~~
$ pwd
/home/robot/act-main

$ tree ckpt/
ckpt/
├── dataset_stats.pkl
├── policy_best.ckpt
├── policy_epoch_0_seed_0.ckpt
├── policy_epoch_100_seed_0.ckpt
├── policy_epoch_200_seed_0.ckpt
├── policy_epoch_300_seed_0.ckpt
├── policy_epoch_310_seed_0.ckpt
├── policy_last.ckpt
├── train_val_kl_seed_0.png
├── train_val_l1_seed_0.png
└── train_val_loss_seed_0.png

0 directories, 11 files
~~~

![alt text](image-4.png)
![alt text](image-5.png)
![alt text](image-6.png)


# 5. Use the ACT model after training

~~~
$ pwd
/home/robot/act-main

$ python3 imitate_episodes.py --task_name sim_insertion_scripted --ckpt_dir ./ckpt --policy_class ACT --kl_weight 10 --chunk_size 100 --hidden_dim 512 --batch_size 8 --dim_feedforward 3200 --num_epochs 2000  --lr 1e-5 --seed 0 --eval --onscreen_render

/home/robot/.local/lib/python3.10/site-packages/torchvision/models/_utils.py:208: UserWarning: The parameter 'pretrained' is deprecated since 0.13 and may be removed in the future, please use 'weights' instead.
  warnings.warn(
/home/robot/.local/lib/python3.10/site-packages/torchvision/models/_utils.py:223: UserWarning: Arguments other than a weight enum or `None` for 'weights' are deprecated since 0.13 and may be removed in the future. The current behavior is equivalent to passing `weights=ResNet18_Weights.IMAGENET1K_V1`. You can also use `weights=ResNet18_Weights.DEFAULT` to get the most up-to-date weights.
  warnings.warn(msg)
number of parameters: 83.92M
KL Weight 10
<All keys matched successfully>
Loaded: ./ckpt/policy_best.ckpt

Rollout 0
episode_return=306, episode_highest_reward=2, env_max_reward=4, Success: False
Saved video to: ./ckpt/video0.mp4

Rollout 1
episode_return=143, episode_highest_reward=3, env_max_reward=4, Success: False
Saved video to: ./ckpt/video1.mp4
~~~

[[video]]

[[video]]


# 6. Inspecting the data formats and the content of the ACT model's input and output during runtime

Modify [the act-main/imitate_episodes.py code](https://github.com/tonyzhaozh/act/blob/main/imitate_episodes.py) to inspect the data formats and the content of the ACT model's input and output during the execution process.

~~~
# imitate_episodes.py

def eval_bc(config, ckpt_name, save_episode=True):
    ...
    num_rollouts = 50
    for rollout_id in range(num_rollouts):
        rollout_id += 0
        ts = env.reset()        
        ...
        with torch.inference_mode():
            for t in range(max_timesteps):
                ### process previous timestep to get qpos and image_list
                obs = ts.observation

                qpos_numpy = np.array(obs['qpos'])
                qpos = pre_process(qpos_numpy)
                qpos = torch.from_numpy(qpos).float().cuda().unsqueeze(0)
                curr_image = get_image(ts, camera_names)

                ### query policy
                if config['policy_class'] == "ACT":
                    if t % query_frequency == 0:
                        print(f"\n[{t}] Policy inputs:\n")
                        print(f"   'qpos': shape: {qpos.shape}, dtype: {qpos.dtype}")
                        print(qpos)
                        print(f"   'curr_image': shape {curr_image.shape}, dtype: {curr_image.dtype}")
                        print(curr_image)

                        all_actions = policy(qpos, curr_image)

                        print(f"\n[{t}] Policy outputs:\n")
                        print(f"   'all_actions': shape: {all_actions.shape}, dtype: {all_actions.dtype}")
                        print(all_actions)
                        print(f"\n")
            ...
~~~

In the code, there are 5 + 4 `print()`  before and after `all_actions = policy(qpos, curr_image)`. These `print()`s  are the codes we plugged in, in order to inspect the data formats and the content of the inputs and outputs of the ACT model (aka policy) during the runtime.

The results of the `print()`  are as follows:

~~~
$ conda activate aloha
$ cd /home/robot/act-main

$ python3 imitate_episodes.py --task_name sim_insertion_scripted --ckpt_dir ./ckpt --policy_class ACT --kl_weight 10 --chunk_size 100 --hidden_dim 512 --batch_size 8 --dim_feedforward 3200 --num_epochs 2000  --lr 1e-5 --seed 0 --eval --onscreen_render

/home/robot/.local/lib/python3.10/site-packages/torchvision/models/_utils.py:208: UserWarning: The parameter 'pretrained' is deprecated since 0.13 and may be removed in the future, please use 'weights' instead.
  warnings.warn(
/home/robot/.local/lib/python3.10/site-packages/torchvision/models/_utils.py:223: UserWarning: Arguments other than a weight enum or `None` for 'weights' are deprecated since 0.13 and may be removed in the future. The current behavior is equivalent to passing `weights=ResNet18_Weights.IMAGENET1K_V1`. You can also use `weights=ResNet18_Weights.DEFAULT` to get the most up-to-date weights.
  warnings.warn(msg)
number of parameters: 83.92M
KL Weight 10
<All keys matched successfully>
Loaded: ./ckpt/policy_best.ckpt

[0] Policy inputs:

   'qpos': shape: torch.Size([1, 14]), dtype: torch.float32
tensor([[-0.2335, -0.8679,  1.2146,  0.2200, -4.4009, -0.2200, -2.4127,  0.0991,
         -0.8481,  1.1700, -0.0797, -4.2769,  0.0921, -1.4997]],
       device='cuda:0')
   'curr_image': shape torch.Size([1, 1, 3, 480, 640]), dtype: torch.float32
tensor([[[[[0., 0., 0.,  ..., 0., 0., 0.],
           [0., 0., 0.,  ..., 0., 0., 0.],
           ... ]]]], device='cuda:0')


[0] Policy outputs:

   'all_actions': shape: torch.Size([1, 100, 14]), dtype: torch.float32
tensor([[[-0.2376, -2.3184,  1.5074,  ...,  0.0986,  0.0097, -0.5107],
         ...
       ]],
       device='cuda:0')


[100] Policy inputs:

   'qpos': shape: torch.Size([1, 14]), dtype: torch.float32
tensor([[ 0.5370, -0.1795,  0.6244, -0.4296,  0.3457,  0.3452,  1.2962,  0.7356,
         -0.5594,  0.9732, -0.4657,  0.5953,  0.5339,  1.7004]],
       device='cuda:0')
   'curr_image': shape torch.Size([1, 1, 3, 480, 640]), dtype: torch.float32
tensor([[[[[0., 0., 0.,  ..., 0., 0., 0.],
           [0., 0., 0.,  ..., 0., 0., 0.],
           ...  ]]]], device='cuda:0')

[100] Policy outputs:

   'all_actions': shape: torch.Size([1, 100, 14]), dtype: torch.float32
tensor([[[ 0.7762, -0.2658,  0.3375,  ...,  0.6632,  0.5565,  1.6795],
         [ 0.7713, -0.2701,  0.3630,  ...,  0.6525,  0.5994,  1.6529],
         ...
       ]],
       device='cuda:0')
~~~

In the first article of this series ["Why to learn LeRobot"](https://github.com/housework-robot/main/blob/main/S02_mount_lerobot_brain/S02E01_why_to_learn_lerobot.md), we discussed:

> 1. The Robot Brainstem
> 
> LeRobot uses [Gymnasium](https://gymnasium.farama.org/content/basic_usage/) as the pivotal connection between the brain and the body, 
> transmitting the brain's action commands to the body, 
> and then feeding back the body's operational results and the environmental information collected by the body, such as the video captured by the body's camera, to the brain.

In the next article, we will introduce how to use Gym as the brainstem to invoke the native body of Stanford Aloha.
