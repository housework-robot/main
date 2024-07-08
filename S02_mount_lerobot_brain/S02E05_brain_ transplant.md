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

## 4.1 setup PYTHONPATH

## 4.2 CLI command

## 4.3 the creation and usage of LeRobot policy

## 4.4 the creation and usage of native Aloha env

## 4.5 the conversion of LeRobot data format to Aloha's

## 4.6 the conversion of Aloha data format to LeRobot's

## 4.7 the process and result of the exection of Aloha body with LeRobot brain


# 5. Appendix 1. The life cycle of the LeRobot brain

## 5.1 The input and output of LeRobot

## 5.2 The creation of LeRobot brain

## 5.3 The usage of LeRobot brain


# 6. Appendix 2. The life cycle of the native Stanford aloha brain

## 6.1 The input and output of native Aloha

## 6.2 The creation of native Aloha brain

## 6.3 The usage of native Aloha brain




