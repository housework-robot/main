# 1. Train robot brain

The subject of this article is to train the LeRobot robot brain.

Specifically, it is to train [the Action Chunking with Transformers (ACT)](https://github.com/tonyzhaozh/act) bimanual robot motion planning model developed by the Stanford aloha team, using the simulation training data provided by LeRobot, to complete the `[aloha_sim_insertion_scripted](https://github.com/huggingface/lerobot/tree/main/tests/data/lerobot/aloha_sim_insertion_scripted)` action.

1. LeRobot has optimized and reimplemented the original ACT model code of the Stanford aloha team, making it more structured, easier to understand, and easier to redevelop.

2. LeRobot has also converted the original training dataset collected by the Stanford aloha team into a format that can train the reimplemented ACT model.

The subject of the next article will be how to perform a robot head transplant.

Epecifically, we will mount the LeRobot robot brain to the simulated body of the Stanford aloha robot via the LeRobot robot brainstem.

In the later articles, we will mount the LeRobot robot brain and brainstem to the real hardware body of the Stanford aloha robot.

Aften then, we will mount the LeRobot robot brain and brainstem to the hardware body of commercial robots.

After the head transplant is completed, we will shift our focus to do the heart transplant, embed the Dora-rs robot body operating system into the hardware body of the commercial robots.


# 2. Download LeRobot code and datasets

# 2.1 Download

The LeRobot team has open-sourced [their code and training dataset on Github](https://github.com/huggingface/lerobot).

To download the LeRobot code and data, it is recommended to use the git clone method.

~~~
$ git clone https://github.com/huggingface/lerobot.git
~~~

We don't recommended to do the downloading using the zip method.

It works fine to use zip method to download code.

However, when downloading training dataset using the zip method, it may result in incorrect data content.

# 2.2 Installation

Following the LeRobot user guide, first set up a conda virtual environment.

~~~
$ conda create -n lerobot  
$ conda activate lerobot
(lerobot)$ 
~~~

Then install the python packages.

~~~
(lerobot)$ cd lerobot
(lerobot)$ pip3 install .
(lerobot)$ pip3 install ".[aloha, pusht]"
~~~


# 3. Train aloha-act model

LeRobot offers two methods for training the model:

1. Write your own python program, leverage the packages provided by LeRobot, to complete the training task.

    LeRobot provides [a sample code](https://github.com/huggingface/lerobot/blob/main/examples/3_train_policy.py), and all you need is to change a few lines of the sample code to do your job.

    This method seems simple, but errors occur when running it.

    [A bug report](https://github.com/huggingface/lerobot/issues/289) has already been issued, and we are waiting for a response from the LeRobot team. But no hurry, because we found an easy-to-work solution.

2. Set the parameters yourself, and then run the ready-to-go training code provided by LeRobot to complete the training task.

    LeRobot provides a ready-to-go training code, `[lerobot/scripts/train.py](https://github.com/huggingface/lerobot/blob/main/lerobot/scripts/train.py)`. The only thing you need to do is to set the parameters yourself to do your job.  

    However, directly reading and understanding the source code of `lerobot/scripts/train.py` is a bit challenging. Fortunately, LeRobot provides a user guide for this code.

    We use this method to complete the training of the aloha-act model.

To train the aloha-act model, you only need to perform the following 3 steps.

## 3.1 Set parameters

Modify the related parameters in the configuration file.

Only one configuration file needs modification, which is `[lerobot/configs/policy/act.yaml](https://github.com/huggingface/lerobot/blob/main/lerobot/configs/policy/act.yaml)`.

~~~
# ${lerobot_home}/lerobot/configs/policy/act.yaml
# @package _global_

seed: 1000
dataset_repo_id: lerobot/aloha_sim_insertion_scripted

override_dataset_stats:
  observation.images.top:
    # stats from imagenet, since we use a pretrained vision model
    mean: [[[0.485]], [[0.456]], [[0.406]]]  # (c,1,1)
    std: [[[0.229]], [[0.224]], [[0.225]]]  # (c,1,1)

training:
  offline_steps: 10010
  online_steps: 0
  eval_freq: 10000
  save_freq: 10000
  log_freq: 250
  save_checkpoint: true

  batch_size: 8
  lr: 1e-5
  lr_backbone: 1e-5
  weight_decay: 1e-4
  grad_clip_norm: 10
  online_steps_between_rollouts: 1
  delta_timestamps:
    action: "[i / ${fps} for i in range(${policy.chunk_size})]"
    
# The following part of the YAML is not changed. 
# ...   
~~~

1. Change the `dataset_repo_id` to `lerobot/aloha_sim_insertion_scripted`.

2. Reduce the `training/offline_steps` to 10,010. The reason for this is that our current goal is to successfully complete the training process, not to pursue the accuracy of the model after training, so we shorten the training process `offline_steps`.

3. Also reduce the `eval_freq` and `save_freq` accordingly, both to 10,000, in order to complete the training process more quickly.


## 3.2 Set environmental variables

Only one variable needs to set, `DATA_DIR`,

~~~
(lerobot)$ export DATA_DIR="${lerobot_home}/tests/data"
(lerobot)$ echo $DATA_DIR
/home/robot/lerobot/tests/data
~~~


## 3.3 Train the model

~~~
$ python3 lerobot/scripts/train.py policy=act env=aloha
INFO 2024-06-23 16:48:49 n/logger.py:106 Logs will be saved locally.
INFO 2024-06-23 16:48:49 ts/train.py:287 make_dataset
INFO 2024-06-23 16:48:49 ts/train.py:300 make_env
INFO 2024-06-23 16:48:49 /__init__.py:88 MUJOCO_GL is not set, so an OpenGL backend will be chosen automatically.
INFO 2024-06-23 16:48:49 /__init__.py:96 Successfully imported OpenGL backend: %s
INFO 2024-06-23 16:48:49 /__init__.py:31 MuJoCo library version is: %s
INFO 2024-06-23 16:48:52 ts/train.py:303 make_policy
INFO 2024-06-23 16:48:52 on/logger.py:39 Output dir: outputs/train/2024-06-23/16-48-49_aloha_act_default
INFO 2024-06-23 16:48:52 ts/train.py:324 cfg.env.task='AlohaInsertion-v0'
INFO 2024-06-23 16:48:52 ts/train.py:325 cfg.training.offline_steps=10010 (10K)
INFO 2024-06-23 16:48:52 ts/train.py:326 cfg.training.online_steps=0
INFO 2024-06-23 16:48:52 ts/train.py:327 offline_dataset.num_samples=400 (400)
INFO 2024-06-23 16:48:52 ts/train.py:328 offline_dataset.num_episodes=1
INFO 2024-06-23 16:48:52 ts/train.py:329 num_learnable_params=51613582 (52M)
INFO 2024-06-23 16:48:52 ts/train.py:330 num_total_params=51613672 (52M)
INFO 2024-06-23 16:48:52 ts/train.py:395 Start offline training on a fixed dataset
INFO 2024-06-23 16:48:54 ts/train.py:179 step:0 smpl:8 ep:0 epch:0.02 loss:69.037 grdn:1078.903 lr:1.0e-05 updt_s:1.507 data_s:0.342
INFO 2024-06-23 16:49:32 ts/train.py:179 step:250 smpl:2K ep:5 epch:5.02 loss:2.675 grdn:84.770 lr:1.0e-05 updt_s:0.142 data_s:0.288
...
...
INFO 2024-06-23 17:14:09 ts/train.py:179 step:10K smpl:78K ep:195 epch:195.02 loss:0.053 grdn:10.005 lr:1.0e-05 updt_s:0.144 data_s:0.294
INFO 2024-06-23 17:14:47 ts/train.py:338 Eval policy at step 10000
INFO 2024-06-23 17:20:39 ts/train.py:213 step:10K smpl:80K ep:200 epch:200.02 ∑rwrd:14.880 success:0.0% eval_s:352.029
INFO 2024-06-23 17:20:39 ts/train.py:352 Resume training
INFO 2024-06-23 17:20:39 ts/train.py:358 Checkpoint policy after step 10000
INFO 2024-06-23 17:20:39 ts/train.py:368 Resume training
INFO 2024-06-23 17:20:40 ts/train.py:179 step:10K smpl:80K ep:200 epch:200.02 loss:0.062 grdn:15.750 lr:1.0e-05 updt_s:0.173 data_s:0.321
INFO 2024-06-23 17:20:41 ts/train.py:358 Checkpoint policy after step 10010
INFO 2024-06-23 17:20:42 ts/train.py:368 Resume training
INFO 2024-06-23 17:20:42 ts/train.py:427 End of training
~~~

In the command line instruction:

```bash
$ python3 lerobot/scripts/train.py policy=act env=aloha
```

1. `policy=act` instructs the system to execute the configurations in `${lerobot_home}/lerobot/configs/policy/act.yaml`.
   
   Note that we have made several modifications to the `act.yaml` settings.

2. `env=aloha` instructs the system to execute the configurations in `${lerobot_home}/lerobot/configs/env/aloha.yaml`.

   Note that we do NOT make any modifications to the `aloha.yaml` settings.


## 3.4 Training result

According to the settings in `${lerobot_home}/lerobot/configs/default.yaml`, the training results are stored in the `${lerobot_home}/outputs/train` directory.

~~~
# https://github.com/huggingface/lerobot/blob/main/lerobot/configs/default.yaml
...
hydra:
  run:
    # Set `dir` to where you would like to save all of the run outputs. If you run another training session
    # with the same value for `dir` its contents will be overwritten unless you set `resume` to true.
    dir: outputs/train/${now:%Y-%m-%d}/${now:%H-%M-%S}_${env.name}_${policy.name}_${hydra.job.name}
...  
~~~

Examining the ${lerobot_home}/outputs/train directory, and find that it contains the following contents:

~~~
(lerobot)$ tree outputs/train/2024-06-23/16-48-49_aloha_act_default/
outputs/train/2024-06-23/16-48-49_aloha_act_default/
├── checkpoints
│   ├── 010000
│   │   ├── pretrained_model
│   │   │   ├── config.json
│   │   │   ├── config.yaml
│   │   │   ├── model.safetensors
│   │   │   └── README.md
│   │   └── training_state.pth
│   ├── 010010
│   │   ├── pretrained_model
│   │   │   ├── config.json
│   │   │   ├── config.yaml
│   │   │   ├── model.safetensors
│   │   │   └── README.md
│   │   └── training_state.pth
│   └── last -> /home/robot/lerobot/outputs/train/2024-06-23/16-48-49_aloha_act_default/checkpoints/010010
├── default.log
└── eval
    └── videos_step_010000
        ├── eval_episode_0.mp4
        ├── eval_episode_1.mp4
        ├── eval_episode_2.mp4
        └── eval_episode_3.mp4

8 directories, 15 files
~~~

Open `eval_episode_[0-3].mp4`, and notice that the motions of the Aloha dual-arm robotic arms are smooth.

Although the task of 'insertion' was not successfully completed, our goal was to complete the model training in a shorter amount of time, rather than pursuing the accuracy of the model.

[[video]]


# 4. Use model after training

Following the LeRobot user guide, execute the following command to use the trained motion model:

~~~
(lerobot)$ python3 lerobot/scripts/eval.py -p outputs/train/2024-06-23/16-48-49_aloha_act_default/checkpoints/last/pretrained_model eval.n_episodes=10 eval.batch_size=10

WARNING 2024-06-23 18:07:40 pts/eval.py:636 The provided pretrained_policy_name_or_path is not a valid Hugging Face Hub repo ID. Treating it as a local directory.
INFO 2024-06-23 18:07:40 on/logger.py:39 Output dir: outputs/eval/2024-06-23/18-07-40_aloha_act
INFO 2024-06-23 18:07:40 pts/eval.py:546 Making environment.
INFO 2024-06-23 18:07:40 /__init__.py:88 MUJOCO_GL is not set, so an OpenGL backend will be chosen automatically.
INFO 2024-06-23 18:07:40 /__init__.py:96 Successfully imported OpenGL backend: %s
INFO 2024-06-23 18:07:40 /__init__.py:31 MuJoCo library version is: %s
INFO 2024-06-23 18:07:40 pts/eval.py:549 Making policy.
Loading weights from local directory
Stepping through eval batches: 100%|█████████████████████████████████████████████████████████████████████████████████████████████████████████████████████████| 1/1 [01:30<00:00, 90.35s/it, running_success_rate=0.0%]
{'avg_sum_reward': 27.2, 'avg_max_reward': 0.3, 'pc_success': 0.0, 'eval_s': 93.11764073371887, 'eval_ep_s': 9.311764121055603}                                                                                       
INFO 2024-06-23 18:09:14 pts/eval.py:578 End of eval
~~~

Check the `${lerobot_home}/lerobot/scripts/eval.py` script to find out in which folder the evaluation results are stored,

~~~
# ${lerobot_home}/lerobot/scripts/eval.py
...
def main(
    pretrained_policy_path: Path | None = None,
    hydra_cfg_path: str | None = None,
    out_dir: str | None = None,
    config_overrides: list[str] | None = None,
):
    ...

    if out_dir is None:
        out_dir = f"outputs/eval/{dt.now().strftime('%Y-%m-%d/%H-%M-%S')}_{hydra_cfg.env.name}_{hydra_cfg.policy.name}"
    ...
~~~

Examine the folder where the evaluation results are stored,

~~~
(lerobot)$ tree outputs/eval/2024-06-23/18-07-40_aloha_act/
outputs/eval/2024-06-23/18-07-40_aloha_act/
├── eval_info.json
└── videos
    ├── eval_episode_0.mp4
    ├── eval_episode_1.mp4
    ├── eval_episode_2.mp4
    ├── eval_episode_3.mp4
    ├── eval_episode_4.mp4
    ├── eval_episode_5.mp4
    ├── eval_episode_6.mp4
    ├── eval_episode_7.mp4
    ├── eval_episode_8.mp4
    └── eval_episode_9.mp4

1 directory, 11 files
~~~

Examine the contents of `eval_info.json` to check the list of video clips that record the episodes of robotic arms motions.

As anticipated, none of the episodes successfully completed the task. This is because the training process for the model was too short, resulting in the model's suboptimal accuracy.

~~~
{
  "per_episode": [
    {
      "episode_ix": 0,
      "sum_reward": 0.0,
      "max_reward": 0.0,
      "success": false,
      "seed": 1000
    },
    ...
    {
      "episode_ix": 9,
      "sum_reward": 0.0,
      "max_reward": 0.0,
      "success": false,
      "seed": 1009
    }
  ],
  "aggregated": {
    "avg_sum_reward": 27.2,
    "avg_max_reward": 0.3,
    "pc_success": 0.0,
    "eval_s": 93.11764073371887,
    "eval_ep_s": 9.311764121055603
  },
  "video_paths": [
    "outputs/eval/2024-06-23/18-07-40_aloha_act/videos/eval_episode_0.mp4",
    ...
    "outputs/eval/2024-06-23/18-07-40_aloha_act/videos/eval_episode_9.mp4"
  ]
}  
~~~

Reviewing the video, it confirms again that although the aloha bimanual robot's motions are smooth, meeting our expectation, how as anticipated, the arms did not successfully complete the insertion task.

[[video]]
