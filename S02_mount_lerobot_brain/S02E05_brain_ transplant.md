# 1. The robot brain transplant surgery

The subject of this article is the brain transplant surgery on the Stanford Aloha robot.

Specifically, it replaces [the Action Chunking with Transformers (ACT)](https://github.com/tonyzhaozh/act) motion planning model on the native Stanford Aloha system with another version of the ACT model reimplemented by LeRobot.

After then we ran the native Stanford aloha system in a simulation environment to executive an insertion task, to verify that the LeRobot brain works well with the native Stanford aloha body. 

There are two approaches to the brain transplant surgery:

1. Using the native Stanford Aloha system as the basic framework, replace the Aloha native brain with the LeRobot's brain. This article adopts this approach.
   
2. Using LeRobot's brain as the basic framework, mount the Stanford Aloha system as the robot's body.

The difference between the two lies in the fact that when using the first approach, the brainstem connecting the brain and the body is the brainstem implemented by the Stanford Aloha team; whereas when using the second approach, the brainstem is implemented by the LeRobot team.


The Stanford Aloha team and the LeRobot team, implement their robot's brainstems, based on the same principle, the [gym-env](https://github.com/Farama-Foundation/Gymnasium) framework:

1. The Stanford Aloha team has implemented two envs: one for simulation, known as [sim-env](https://github.com/tonyzhaozh/act/blob/main/sim_env.py), and the other for the hardware body, known as [real-env](https://github.com/MarkFzp/mobile-aloha/blob/main/aloha_scripts/real_env.py).

2. The LeRobot team's version of [sim-env](https://github.com/huggingface/lerobot/blob/main/lerobot/common/envs/factory.py) re-implemented Stanford Aloha's [sim-env](https://github.com/tonyzhaozh/act/blob/main/sim_env.py), with a more modular system structure, more standardized APIs, and easier configuration of the system through parameter files.


Why did we choose the first approach instead of the more standardized second approach?

1. The native brainstem of the first approach often provides more comprehensive and detailed support for the functionality of the robot body.

2. The more standardized second approach, in its early stages, often sacrifices some less important functions of the robot body in order to emphasize standardization.

3. As the second approach continues to evolve, it is likely to settle on a hybrid architecture, where the main framework of the brainstem achieves 100% standardization, and it contains some sockets, where some native brainstem components can plug-and-play.


To achieve excellence his work, one must first sharpen his tools. First, let us spend some time to prepare the surgery, after then we proceed with the brain transplant surgery:

1. Read LeRobot's source code, to understand the creation, usage, and data format conversion of LeRobot's brain. This step is unrelated to the Stanford Aloha native system.

2. Read Stanford aloha's source code, to understand the the creation, usage, and data format conversion of Stanford Aloha's brain. This step is unrelated to LeRobot's brain.

3. Perform the head transplant surgery, using the Stanford Aloha native system as the framework, and replace the native Aloha brain with LeRobot's brain.


# 2. LeRobot's brain

We only need to take 5 steps, to create and use the LeRobot brain, in addition to inspecting its data formats.

Here, we only discuss how to operate and the results of the operation.

If you want to understand not only what is done but also why it is done, please refer to Appendix I of this document, "Appendix I. The life cycle of the LeRobot brain".


## 2.1 CLI command

Following the user guide of [the LeRobot github](https://github.com/huggingface/lerobot/blob/main/README.md), when evaluating the performance of a LeRobot after training it, 
in the CLI command, you should input the path and name of the file folder, which contains the checkpoint of the  LeRobot model.

The `config.yaml` configuration file within the checkpoint file folder contains the system parameters we need.

~~~
# (lerobot) robot@robot-test:~/lerobot$ python3 lerobot/scripts/eval.py \
  -p outputs/train/2024-06-23/16-48-49_aloha_act_default/checkpoints/last/pretrained_model \
  eval.n_episodes=1 \
  eval.batch_size=10
~~~


## 2.2 the creation of hydra_cfg

To generate hydra_cfg, which will be useful when creating LeRobot brain (aka `policy`), you need the the `config.yaml` file from the checkpoint file folder as an input parameter,

~~~
hydra_cfg = init_hydra_config(
                str(pretrained_policy_path / "config.yaml"), 
                config_overrides
            )
~~~


## 2.3 the creation of policy

Use `hydra_cfg` as input parameter, to create an instance of `policy`.

~~~
policy = make_policy(
            hydra_cfg=hydra_cfg, 
            pretrained_policy_name_or_path=str(pretrained_policy_path)
        )
~~~


## 2.4 the usage of policy 

~~~
actions = policy.select_action(observations)
~~~

As the output of `policy`, `actions` usually consists of one or more actions. 

The amount of output actions, should be the same as the number of the input observations. 


## 2.5 the usage of env

~~~
observation, reward, terminated, truncated, info = env.step(action)
~~~

When calling `env.step()`, shall we input only one single action, or mutiple ones?

Well, it depends on your `env`, if you create a single instance of `env`, then you input only one single action to `env.step()`. 

Otherwise, if you create an instance of `VectorEnv`, then you have to input multiple actions, and the number of actions should be consist with the size of `VectorEnv`.



# 3. Native Aloha's brain

We only need to take 5 steps, to create and use an instance of the native Aloha brain, in addition to inspecting its data formats.

Here, we only discuss how to operate and the results of the operation.

If you want to understand not only what is done but also why it is done, please refer to Appendix I of this document, "Appendix II. The life cycle of the native Stanford Aloha brain".


## 3.1 CLI command

In the CLI command, we need to specify some system parameters as the input parameters. 

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


## 3.2 the creation of policy_config

Reading the source code of [imitate_episodes.py](https://github.com/tonyzhaozh/act/blob/main/imitate_episodes.py), we need to prepare `policy_config` which consists of some configurations, before creating an instance of `policy`.

~~~
policy_config = {
    'lr': 1e-5,
    'num_queries': 100,
    'kl_weight': 10,
    'hidden_dim': 512,
    'dim_feedforward': 3200,
    'lr_backbone': 1e-5,
    'backbone': 'resnet18',
    'enc_layers': 4,
    'dec_layers': 7,
    'nheads': 8,
    'camera_names': ['top'],
} 
~~~


## 3.3 the creation of policy

When creating an instance of the native Aloha `policy`, we need to use `policy_config` as the input parameter. 

~~~
from policy import ACTPolicy
policy = ACTPolicy(policy_config)
~~~


## 3.4 the usage of policy

~~~
all_actions = policy(qpos, curr_image)
~~~

In the above code, the data formats of `qpos`, `curr_image`, and `all_actions` are, 

~~~
# Policy inputs:

   'qpos': torch.Size([1, 14]), dtype: torch.float32    
   'curr_image': torch.Size([1, 1, 3, 480, 640]), dtype: torch.float32
   # The first 1 means there is one data, 
   # The second 1 means there is only one camera.

# Policy outputs:

   'all_actions': shape: torch.Size([1, 100, 14]), dtype: torch.float32
~~~


## 3.5 the creation of env

~~~
from sim_env import make_sim_env
task_name = 'sim_insertion_scripted'
env = make_sim_env(task_name)
~~~


## 3.6 the usage of env

~~~
ts = env.step(target_qpos)
~~~

In the above code, the data formats of `target_qpos` and `ts` are, 

~~~
# Env input 
    
    'target_qpos': array(14,) 

# Env output

    'ts': is an instance of TimeStep 
 TimeStep(step_type=<StepType.MID: 1>, 
     reward=0, 
     discount=1.0, 
     observation=OrderedDict([
        ('qpos', array(14,), 
        ('qvel', array(14,), 
        ('env_state', array(array(1, 14)) ), 
        ('images', 
           {'top': array(480, 640, 3)), 
            'angle': array(480, 640, 3)), 
            'vis': array(480, 640, 3))
           }
        )
     ])
 ) 
~~~


# 4. Transplant LeRobot brain to Aloha body

Having completed the preoperative preparations, we are now ready to proceed with the brain transplant surgery.

Specifically, in the native Stanford Aloha simulation system, we will replace the `policy` from the original Stanford aloha's implementation of `ACT` (Action Chunking with Transformers) model, with LeRobot's implementation of `ACT` model. 

The process is divided into the following steps:


## 4.1 setup PYTHONPATH

~~~
$ export PYTHONPATH=/home/robot/lerobot:/home/robot/act-main:$PYTHONPATH
$ echo $PYTHONPATH
  /home/robot/lerobot:/home/robot/act-main
~~~


## 4.2 CLI command

We modified Stanford aloha's source code, `imitate_episodes.py`, and named the new code as `imitate_lerobot.py`. The source code of [imitate_lerobot.py]() is provided in this repo. 

Having completed the modification of the source code, we execute a CLI command similar to [the native aloha CLI command](https://github.com/tonyzhaozh/act?tab=readme-ov-file#simulated-experiments). 

~~~
(aloha) robot@robot-test:~/act-main$ $ python3 imitate_lerobot.py \
  --ckpt_dir /home/robot/lerobot/outputs/train/2024-06-23/16-48-49_aloha_act_default/checkpoints/last/pretrained_model/ \
  --batch_size 8 \
  --num_episodes 1000
~~~

Note, `ckpt_dir` should be LeRobot model checkpoint, rather than Aloha native model checkpoint.  



## 4.3 the creation and usage of LeRobot policy

In the source of `imitate_lerobot.py`, we modified the codes to create an instance of `policy`. 

We do not use the native Stanford aloha's `ACT` model anymore, instead, we use LeRobot's implementation of `ACT` model. 

~~~
pretrained_policy_path = 'outputs/train/2024-06-23/16-48-49_aloha_act_default/checkpoints/last/pretrained_model'

hydra_cfg = init_hydra_config(
    str(pretrained_policy_path / "config.yaml"), 
    config_overrides
)

policy = make_policy(
    hydra_cfg=hydra_cfg, 
    pretrained_policy_name_or_path=str(pretrained_policy_path)
)  
         
with torch.inference_mode():
    action = policy.select_action(observation)             
~~~


## 4.4 the creation and usage of native Aloha env

~~~
from sim_env import make_sim_env
env = make_sim_env(task_name)
ts = env.step(target_qpos)
~~~


## 4.5 the conversion of LeRobot data format to Aloha's

Convert the data format of LeRobot's `policy` output, to be aligned with the data format of the native Aloha's `env` input.

1. As mentioned above, the data format of LeRobot `policy` output is, 

~~~
Policy output data format:

   action.shape, torch.Size([10, 14]) 
~~~

Note, the number of rows, 10, is determined by the input parameter in the CLI command, `batch_size=10`. 

The 10 policy outputs will be executed by `env` in sequence according to their serial numbers.

Also note, each policy output is an instruction to the robot body. Usually, the instruction contains the targeted angles of every servo of the robot body. 

Stanford aloha robot body has multiple servos, totally they have 14 degree-of-freedom, therefore, the policy instruction need to provide 14 angles to the aloha robot body. 


2. As mentioned above, the data format of the native Aloha `env` input is, 
   
~~~
Output data format:

   'target_qpos': (14,) 
    
e.g. [-9.3384460e-04 -1.7078998e+00  1.2308732e+00  4.6417303e-03
       5.6922406e-01 -1.3676301e-02  1.6363129e-01 -1.0724664e-03
      -1.7091711e+00  1.2568194e+00  2.2574503e-02  5.7522881e-01
      -3.0301604e-02  1.3787243e-01] 

~~~

Note, the input of the native Aloha env, `target_qpos` is a list, consisting of 14 parameters. 

We do not need to do any conversion of the data formats from the output of LeRobot `policy`, to the input of the native Stanford aloha `env`. 

We only need to ask the native Stanford aloha's `env`, to execute the output of LeRobot `policy`, i.e `actions`, in sequence. 


## 4.6 the conversion of Aloha data format to LeRobot's

Convert the data format of the output of the native Aloha's `env`, to be aligned with the data format of the input of LeRobot's `policy`. 

1. As mentioned above, the data format of Aloha `env`'s output is, 

~~~
# Env output

    'ts': is an instance of TimeStep 
 TimeStep(step_type=<StepType.MID: 1>, 
     reward=0, 
     discount=1.0, 
     observation=OrderedDict([
        ('qpos', array(14,), 
        ('qvel', array(14,), 
        ('env_state', array(array(1, 14)) ), 
        ('images', 
           {'top': array(480, 640, 3)), 
            'angle': array(480, 640, 3)), 
            'vis': array(480, 640, 3))
           }
        )
     ])
 ) 
~~~

2. As mentioned above, the data format of LeRobot `policy`'s input is 

~~~
# Policy input: observation

   observation['observation.images.top'].shape: torch.Size([10, 3, 480, 640])
   observation['observation.state'].shape: torch.Size([10, 14]) 
~~~

We need to take `qpos` and `images.top` out of Aloha `env`'s output, then convert them to the input of LeRobot `policy`.  


## 4.7 the process and result of the exection of Aloha body with LeRobot brain

After meticulous preoperative preparation, the robot brain transplant surgery went relatively smoothly. Below is the execution process of the Aloha body with LeRobot brain for a certain task.

Note that we have not yet carefully verified whether the simulation task was successfully completed.

Our current goal is that the task execution process should be smooth and free of bugs. And obviously, our goal is achieved. 



# 5. Appendix I. The life cycle of the LeRobot brain

In [LeRobot github](https://github.com/huggingface/lerobot/blob/main/README.md%23evaluate-a-pretrained-policy) page, there is a short user guide,

1. The training of the LeRobot brain

~~~
python lerobot/scripts/train.py policy=act \
    env=aloha env.task=AlohaInsertion-v0 \
    dataset_repo_id=lerobot/aloha_sim_insertion_human 
~~~

2. The Usage of the LeRobot brain

~~~
python lerobot/scripts/eval.py -p lerobot/diffusion_pusht \
    eval.n_episodes=10 eval.batch_size=10
~~~

To understand LeRobot system, [our last article](https://github.com/housework-robot/main/blob/main/S02_mount_lerobot_brain/S02E04_train_lerobot_brain.md) "The training of LeRobot robot brain", describes the process that how to train and use the LeRobot brain. 

Based on our experience of training and using LeRobot brain with CLI commands, here we read the source code of LeRobot system, and study the procedure how the program creates and uses the LeRobot brain. 


## 5.1 The input and output of LeRobot

1. Following the instruction of our last article, "[The training of LeRobot robot brain](https://github.com/housework-robot/main/blob/main/S02_mount_lerobot_brain/S02E04_train_lerobot_brain.md)", we used simulation dataset to train LeRobot brain. After the training, we got the checkpoint. 

~~~
$ tree outputs/train/2024-06-23/16-48-49_aloha_act_default/checkpoints/last/pretrained_model

pretrained_model
├── config.json
├── config.yaml
├── model.safetensors
└── README.md

0 directories, 4 files
~~~


2. There are some synonyms of "the robot brain",

    `brain` ≈≈ `model` ≈≈ `policy`

    In the source of [lerobot/scripts/eval.py](), `policy` is equivalent to `brain`. 

    We modified the source code of [lerobot/scripts/eval.py](), especially before and after line 154, we inserted some `print` lines, printed out the data format and content of `observation` and `action`. 

    ~~~
    print(f" [Kan] observation: '{observation}'\n")
    print(f" [Kan] observation['observation.images.top'].shape: {observation['observation.images.top'].shape}")
    print(f" [Kan] observation['observation.state'].shape: {(observation['observation.state'].shape)} \n")

    with torch.inference_mode():
        action = policy.select_action(observation)
        print(f" [Kan] action selected: '{action}'\n")
        print(f" [Kan] action.shape, {action.shape} \n")
    ~~~


3. The input and output of LeRobot brain `policy`, are `observation` and `action` respectively. The following are their data formats and contents, 

~~~
# (lerobot) robot@robot-test:~/lerobot$ python3 lerobot/scripts/eval.py -p outputs/train/2024-06-23/16-48-49_aloha_act_default/checkpoints/last/pretrained_model eval.n_episodes=1 eval.batch_size=10


# Input，observation

 [Kan] observation: 
  '{'observation.images.top': 
tensor([[[[0., 0., 0.,  ..., 0., 0., 0.],
          [0., 0., 0.,  ..., 0., 0., 0.],
          [0., 0., 0.,  ..., 0., 0., 0.],
          ...,
          [0., 0., 0.,  ..., 0., 0., 0.]]]], device='cuda:0'), 

    'observation.state': 
 tensor([[ 7.1613e-03, -1.7131e+00,  1.1841e+00, -2.5924e-02,  5.4231e-01,
          2.4643e-02,  1.1398e-01, -8.8337e-04, -1.7068e+00,  1.1786e+00,
         -2.3806e-04,  5.5058e-01, -7.6557e-03,  1.0513e-01],
        [ 7.8769e-03, -1.7267e+00,  1.1833e+00, -2.7808e-02,  5.0721e-01,
          2.7018e-02,  1.0768e-01, -5.0291e-04, -1.7197e+00,  1.1796e+00,
         -3.6706e-04,  5.1722e-01, -8.3997e-03,  1.0427e-01],
        [ 8.8999e-03, -1.7333e+00,  1.1846e+00, -2.8895e-02,  5.2125e-01,
          3.0681e-02,  1.0819e-01, -5.8283e-04, -1.7263e+00,  1.1803e+00,
          2.3186e-04,  5.3151e-01, -8.0607e-03,  1.0438e-01],
        [ 8.2411e-03, -1.7271e+00,  1.1855e+00, -2.6289e-02,  5.4436e-01,
          2.6184e-02,  1.1222e-01, -9.4044e-04, -1.7143e+00,  1.1781e+00,
          5.6957e-05,  5.5315e-01, -8.5074e-03,  1.0433e-01],
        [ 8.4857e-03, -1.7266e+00,  1.1866e+00, -2.5859e-02,  5.5489e-01,
          2.5932e-02,  1.1484e-01, -7.1643e-04, -1.7169e+00,  1.1786e+00,
          1.0940e-03,  5.6355e-01, -6.7197e-03,  1.0530e-01],
        [ 7.8832e-03, -1.7260e+00,  1.1836e+00, -2.8671e-02,  5.1814e-01,
          3.0354e-02,  1.0786e-01, -6.1902e-04, -1.7226e+00,  1.1796e+00,
         -1.2735e-04,  5.2900e-01, -8.3296e-03,  1.0427e-01],
        [ 7.6585e-03, -1.7215e+00,  1.1851e+00, -2.6093e-02,  5.3062e-01,
          2.5576e-02,  1.0869e-01, -6.9695e-04, -1.7132e+00,  1.1791e+00,
          3.2161e-05,  5.3951e-01, -7.6392e-03,  1.0396e-01],
        [ 8.7841e-03, -1.7280e+00,  1.1852e+00, -2.5406e-02,  5.4712e-01,
          2.5087e-02,  1.1265e-01, -5.7298e-04, -1.7179e+00,  1.1776e+00,
          7.5060e-04,  5.5597e-01, -7.1240e-03,  1.0474e-01],
        [ 8.3793e-03, -1.7249e+00,  1.1828e+00, -2.6629e-02,  5.3378e-01,
          2.6916e-02,  1.0926e-01, -1.0310e-04, -1.7157e+00,  1.1760e+00,
          6.7634e-04,  5.4312e-01, -7.5307e-03,  1.0422e-01],
        [ 7.8232e-03, -1.7210e+00,  1.1827e+00, -2.6510e-02,  5.2974e-01,
          2.6158e-02,  1.0965e-01, -5.9296e-04, -1.7151e+00,  1.1768e+00,
          1.5710e-04,  5.3742e-01, -7.8570e-03,  1.0401e-01]], device='cuda:0')}'

 [Kan] observation['observation.images.top'].shape: torch.Size([10, 3, 480, 640])
 [Kan] observation['observation.state'].shape: torch.Size([10, 14])           


# output, action

 [Kan] action selected: 
 'tensor([[ 8.8620e-03, -1.6702e+00,  1.1822e+00, -2.3200e-02,  5.5219e-01,
          2.3818e-02,  1.2918e-01, -9.3020e-04, -1.6637e+00,  1.1772e+00,
          2.0534e-03,  5.6042e-01, -7.6822e-03,  1.1365e-01],
        [ 9.2443e-03, -1.6861e+00,  1.1813e+00, -2.5128e-02,  5.2075e-01,
          2.6149e-02,  8.5067e-02, -3.5986e-04, -1.6816e+00,  1.1777e+00,
          1.8512e-03,  5.3019e-01, -8.5207e-03,  7.4029e-02],
        [ 1.0428e-02, -1.6959e+00,  1.1821e+00, -2.6302e-02,  5.3178e-01,
          2.9850e-02,  9.2296e-02, -5.7994e-04, -1.6885e+00,  1.1782e+00,
          2.4758e-03,  5.4185e-01, -7.9629e-03,  8.1606e-02],
        [ 9.8164e-03, -1.6841e+00,  1.1837e+00, -2.3704e-02,  5.5438e-01,
          2.5395e-02,  1.2291e-01, -1.0144e-03, -1.6711e+00,  1.1768e+00,
          2.3180e-03,  5.6335e-01, -8.4204e-03,  1.0676e-01],
        [ 1.0480e-02, -1.6846e+00,  1.1844e+00, -2.3715e-02,  5.6315e-01,
          2.5008e-02,  1.3061e-01, -9.2633e-04, -1.6734e+00,  1.1771e+00,
          3.4737e-03,  5.7205e-01, -6.5401e-03,  1.1396e-01],
        [ 9.3254e-03, -1.6867e+00,  1.1814e+00, -2.6119e-02,  5.3024e-01,
          2.9510e-02,  9.3985e-02, -5.4167e-04, -1.6842e+00,  1.1776e+00,
          2.1354e-03,  5.4090e-01, -8.3011e-03,  8.5743e-02],
        [ 9.4159e-03, -1.6785e+00,  1.1831e+00, -2.3335e-02,  5.4101e-01,
          2.4707e-02,  1.1168e-01, -6.5782e-04, -1.6697e+00,  1.1775e+00,
          2.3859e-03,  5.4986e-01, -7.6164e-03,  9.4281e-02], 
        [ 1.0410e-02, -1.6857e+00,  1.1831e+00, -2.3105e-02,  5.5587e-01,
          2.4283e-02,  1.2472e-01, -6.6477e-04, -1.6745e+00,  1.1760e+00,
          3.1848e-03,  5.6483e-01, -6.9666e-03,  1.1071e-01],
        [ 9.9562e-03, -1.6856e+00,  1.1806e+00, -2.4051e-02,  5.4362e-01,
          2.6082e-02,  1.1347e-01, -8.3810e-05, -1.6774e+00,  1.1741e+00,
          3.0972e-03,  5.5283e-01, -7.3712e-03,  1.0193e-01],
        [ 9.3886e-03, -1.6776e+00,  1.1807e+00, -2.3623e-02,  5.4139e-01,
          2.5224e-02,  1.1575e-01, -5.8711e-04, -1.6714e+00,  1.1753e+00,
          2.4901e-03,  5.4904e-01, -7.7936e-03,  1.0027e-01]], device='cuda:0')'
          
[Kan] action.shape, torch.Size([10, 14])           
~~~


## 5.2 The creation of LeRobot brain

To invoke the LeRobot brain within the Stanford Aloha system, we need to understand how to instantiate a LeRobot `policy`. 

1. Set up parameters

Read the source code of [lerobot/scripts/eval.py](https://github.com/huggingface/lerobot/blob/main/lerobot/scripts/eval.py%23L154),

~~~
def main(
    pretrained_policy_path: Path | None = None,
    hydra_cfg_path: str | None = None,
    out_dir: str | None = None,
    config_overrides: list[str] | None = None,
):
    
    if pretrained_policy_path is not None:
        hydra_cfg = init_hydra_config(
            str(pretrained_policy_path / "config.yaml"), 
            config_overrides
        )
    else:
        ...

    if out_dir is None:
        out_dir = f"outputs/eval/{dt.now().strftime('%Y-%m-%d/%H-%M-%S')}_{hydra_cfg.env.name}_{hydra_cfg.policy.name}"
    ...

    logging.info("Making environment.")
    env = make_env(hydra_cfg)

    logging.info("Making policy.")
    if hydra_cfg_path is None:
        # policy = ACTPolicy(hydra_cfg)
        policy = make_policy(
            hydra_cfg=hydra_cfg, 
            pretrained_policy_name_or_path=str(pretrained_policy_path)
        )
    else:
        ...


if __name__ == "__main__":
    init_logging()

    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument(
        "-p",
        "--pretrained-policy-name-or-path",
        help=(
            "Either the repo ID of a model hosted on the Hub or a path to a directory containing weights "
            "saved using `Policy.save_pretrained`. If not provided, the policy is initialized from scratch "
            "(useful for debugging). This argument is mutually exclusive with `--config`."
        ),
    )
    group.add_argument(
        "--config",
        help=(
            "Path to a yaml config you want to use for initializing a policy from scratch (useful for "
            "debugging). This argument is mutually exclusive with `--pretrained-policy-name-or-path` (`-p`)."
        ),
    )
    parser.add_argument("--revision", help="Optionally provide the Hugging Face Hub revision ID.")
    parser.add_argument(
        "--out-dir",
        help=(
            "Where to save the evaluation outputs. If not provided, outputs are saved in "
            "outputs/eval/{timestamp}_{env_name}_{policy_name}"
        ),
    )
    parser.add_argument(
        "overrides",
        nargs="*",
        help="Any key=value arguments to override config values (use dots for.nested=overrides)",
    )
    args = parser.parse_args()
    
    if args.pretrained_policy_name_or_path is None:
        ...
    else:
        try:
            pretrained_policy_path = Path(
                snapshot_download(args.pretrained_policy_name_or_path, revision=args.revision)
            )
        except (HFValidationError, RepositoryNotFoundError) as e:
            ...
            
        main(
            pretrained_policy_path=pretrained_policy_path,
            out_dir=args.out_dir,
            config_overrides=args.overrides,
        )                  
~~~

Comparing the above source code with the CLI command that we executed, 

~~~
# (lerobot) robot@robot-test:~/lerobot$ python3 lerobot/scripts/eval.py \
-p outputs/train/2024-06-23/16-48-49_aloha_act_default/checkpoints/last/pretrained_model \
eval.n_episodes=1 \
eval.batch_size=10
~~~


The values of the input parameters are,

~~~
pretrained_policy_path: "outputs/train/2024-06-23/16-48-49_aloha_act_default/checkpoints/last/pretrained_model",
hydra_cfg_path: None,
out_dir: None，
config_overrides: None
~~~


2. The content of `hydra_cfg`

Following is the source code of the creation of `policy`, note that there is an input parameter `hydra_cfg`, we need to inspect its content, 

~~~
policy = make_policy(
            hydra_cfg=hydra_cfg, 
            pretrained_policy_name_or_path=str(pretrained_policy_path)
        )
~~~


Following is the source code of the creation of `hydra_cfg`, 

~~~
hydra_cfg = init_hydra_config(
                str(pretrained_policy_path / "config.yaml"), 
                config_overrides
            )
~~~

As mentioned above, 

~~~
pretrained_policy_path: "outputs/train/2024-06-23/16-48-49_aloha_act_default/checkpoints/last/pretrained_model",
config_overrides: None
~~~


Therefore, the content of `hydra_cfg` comes from `outputs/train/2024-06-23/16-48-49_aloha_act_default/checkpoints/last/pretrained_model/config.yaml`

~~~
resume: false
device: cuda
use_amp: false
seed: 1000
dataset_repo_id: lerobot/aloha_sim_insertion_scripted
video_backend: pyav
training:
  offline_steps: 10010
  online_steps: 0
  online_steps_between_rollouts: 1
  online_sampling_ratio: 0.5
  online_env_seed: ???
  eval_freq: 10000
  log_freq: 250
  save_checkpoint: true
  save_freq: 10000
  num_workers: 4
  batch_size: 8
  image_transforms:
    ...
eval:
  n_episodes: 50
  batch_size: 50
  use_async_envs: false
wandb:
  ...
fps: 50
env:
  name: aloha
  task: AlohaInsertion-v0
  state_dim: 14
  action_dim: 14
  fps: ${fps}
  episode_length: 400
  gym:
    obs_type: pixels_agent_pos
    render_mode: rgb_array
override_dataset_stats:
  ...
policy:
  name: act
  n_obs_steps: 1
  chunk_size: 100
  n_action_steps: 100
  input_shapes:
    observation.images.top:
    - 3
    - 480
    - 640
    observation.state:
    - ${env.state_dim}
  output_shapes:
    action:
    - ${env.action_dim}  
  ...
  vision_backbone: resnet18
  pretrained_backbone_weights: ResNet18_Weights.IMAGENET1K_V1
  replace_final_stride_with_dilation: false
  pre_norm: false
  dim_model: 512
  n_heads: 8
  dim_feedforward: 3200
  feedforward_activation: relu
  n_encoder_layers: 4
  n_decoder_layers: 1
  use_vae: true
  latent_dim: 32
  n_vae_encoder_layers: 4
  temporal_ensemble_momentum: null
  dropout: 0.1
  kl_weight: 10.0    
~~~

The content of the configuration file `config.yaml` is very comprehensive, covering the system structure of the LeRobot brain (`policy`), as well as the settings for the LeRobot brainstem (`env`), in addition to the control parameters of the training process. 




## 5.3 The usage of LeRobot brain


# 6. Appendix II. The life cycle of the native Stanford aloha brain

## 6.1 The input and output of native Aloha

## 6.2 The creation of native Aloha brain

## 6.3 The usage of native Aloha brain




