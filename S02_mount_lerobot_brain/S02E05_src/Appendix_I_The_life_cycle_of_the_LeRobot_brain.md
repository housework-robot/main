# Appendix I. The life cycle of the LeRobot brain

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


## 1. The input and output of LeRobot

### 1.1 Checkpoint
   
Following the instruction of our last article, "[The training of LeRobot robot brain](https://github.com/housework-robot/main/blob/main/S02_mount_lerobot_brain/S02E04_train_lerobot_brain.md)", we used simulation dataset to train LeRobot brain. After the training, we got the checkpoint. 

~~~
$ tree outputs/train/2024-06-23/16-48-49_aloha_act_default/checkpoints/last/pretrained_model

pretrained_model
├── config.json
├── config.yaml
├── model.safetensors
└── README.md

0 directories, 4 files
~~~


### 1.2 Policy is brain

There are some synonyms of "the robot brain",

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


### 1.3 observation and action

The input and output of LeRobot brain `policy`, are `observation` and `action` respectively. The following are their data formats and contents, 

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


## 2. The creation of LeRobot brain

To invoke the LeRobot brain within the Stanford Aloha system, we need to understand how to instantiate a LeRobot `policy`. 

### 2.1 System parameters

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


### 2.2 The content of `hydra_cfg`

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



## 3. The lifecycle of LeRobot brain

As mentioned above, in the source code, LeRobot brain is actually `policy`, its input is `observation`, and its output is `action`.

`action` is both the output of the LeRobot brain, and the input of the native Aloha brainstem. 

### 3.1 the source code

Let us look into the source code of LeRobot, to understand how LeRobot's `env` uses LeRobot `policy`'s output `action`. 

~~~
# /home/robot/lerobot/lerobot/scripts/eval.py 

def rollout(
    env: gym.vector.VectorEnv,
    policy: Policy,
    seeds: list[int] | None = None,
    return_observations: bool = False,
    render_callback: Callable[[gym.vector.VectorEnv], None] | None = None,
    enable_progbar: bool = False,
) -> dict:
    """
    Run a batched policy rollout once through a batch of environments.

    Note that all environments in the batch are run until the last environment is done. This means some
    data will probably need to be discarded (for environments that aren't the first one to be done).

    The return dictionary contains:
        (optional) "observation": A a dictionary of (batch, sequence + 1, *) tensors mapped to observation
            keys. NOTE the that this has an extra sequence element relative to the other keys in the
            dictionary. This is because an extra observation is included for after the environment is
            terminated or truncated.
        "action": A (batch, sequence, action_dim) tensor of actions applied based on the observations (not
            including the last observations).
        "reward": A (batch, sequence) tensor of rewards received for applying the actions.
        "success": A (batch, sequence) tensor of success conditions (the only time this can be True is upon
            environment termination/truncation).
        "done": A (batch, sequence) tensor of **cumulative** done conditions. For any given batch element,
            the first True is followed by True's all the way till the end. This can be used for masking
            extraneous elements from the sequences above.

    Args:
        env: The batch of environments.
        policy: The policy. Must be a PyTorch nn module.
        seeds: The environments are seeded once at the start of the rollout. If provided, this argument
            specifies the seeds for each of the environments.
        return_observations: Whether to include all observations in the returned rollout data. Observations
            are returned optionally because they typically take more memory to cache. Defaults to False.
        render_callback: Optional rendering callback to be used after the environments are reset, and after
            every step.
        enable_progbar: Enable a progress bar over rollout steps.
    Returns:
        The dictionary described above.
    """
    ...

    # Reset the policy and environments.
    policy.reset()
    observation, info = env.reset(seed=seeds)

    while not np.all(done):
        # Numpy array to tensor and changing dictionary keys to LeRobot policy format.
        observation = preprocess_observation(observation)
        observation = {key: observation[key].to(device, non_blocking=True) for key in observation}
        
        with torch.inference_mode():
            action = policy.select_action(observation)
 
        # Apply the next action.
        observation, reward, terminated, truncated, info = env.step(action)
        
        # Keep track of which environments are done so far.
        done = terminated | truncated | done      
~~~

`Action` represents the control parameters for a robot. For example, the Stanford Aloha has two forearms, each with 7 degrees of freedom, so the `action` for the Stanford Aloha robot is a vector containing 14 elements.


### 3.2 why not one single action?

As mentioned above, the output of the `policy` is not a single `action`, but rather 10 `actions` output at once.

This raises the question: when the LeRobot brainstem `env` receives 10 `actions` as input, does the `env` execute each `action` in sequence, or does it only select only one `action` to execute and ignore the other 9 `actions`?

We read the source code of LeRobot to study the generation process of the brainstem `env`.

In the source code, `env` is an instance of `VectorEnv`,

~~~
# /home/robot/lerobot/lerobot/scripts/eval.py 

def rollout(
    env: gym.vector.VectorEnv,
    ...
~~~


### 3.3 why 10 actions?

Let us inspect the creation process of `env`, to see the size of `VectorEnv`, 

~~~
# /home/robot/lerobot/lerobot/scripts/eval.py 

from lerobot.common.envs.factory import make_env

def main(
    pretrained_policy_path: Path | None = None,
    hydra_cfg_path: str | None = None,
    out_dir: str | None = None,
    config_overrides: list[str] | None = None,
):
    ...
    logging.info("Making environment.")
    env = make_env(hydra_cfg)
~~~

~~~
# /home/robot/lerobot/lerobot/common/envs/factory.py

def make_env(cfg: DictConfig, n_envs: int | None = None) -> gym.vector.VectorEnv:
    """
    Makes a gym vector environment according to the evaluation config.
    n_envs can be used to override eval.batch_size in the configuration. Must be at least 1.
    """

    # cfg.env.name: aloha
    package_name = f"gym_{cfg.env.name}"
    try:
        # package_name: gym_aloha
        # https://github.com/huggingface/gym-aloha
        importlib.import_module(package_name)
    except ModuleNotFoundError as e:
        ...
       
    # cfg.env.task: AlohaInsertion-v0
    gym_handle = f"{package_name}/{cfg.env.task}"
    gym_kwgs = dict(cfg.env.get("gym", {}))

    if cfg.env.get("episode_length"):
        gym_kwgs["max_episode_steps"] = cfg.env.episode_length

    # batched version of the env that returns an observation of shape (b, c)
    # Since, cfg.eval.use_async_envs: false
    # hence, env_cls is gym.vector.SyncVectorEnv

    env_cls = gym.vector.AsyncVectorEnv if cfg.eval.use_async_envs else gym.vector.SyncVectorEnv
    env = env_cls(
        [
            lambda: gym.make(gym_handle, disable_env_checker=True, **gym_kwgs)
            
            # cfg.eval.batch_size: 50
            # but the config is overridden by the command: 
            # $ python3 lerobot/scripts/eval.py -p outputs/train/2024-06-23/16-48-49_aloha_act_default/checkpoints/last/pretrained_model \
            #     eval.n_episodes=1 eval.batch_size=10
            for _ in range(n_envs if n_envs is not None else cfg.eval.batch_size)
        ]
    )

    return env
~~~

Note, 

1. Even through in the config file, `cfg.eval.batch_size` is 50, however, it is overridden by the CLI command, 
    ~~~
    $ python3 lerobot/scripts/eval.py -p outputs/train/2024-06-23/16-48-49_aloha_act_default/checkpoints/last/pretrained_model \
        eval.n_episodes=1 \
        eval.batch_size=10
    ~~~

    Therefore, the size of `batch_size` is 10. 

2. env_cls is an instance of `gym.vector.SyncVectorEnv`,

    and the size of `gym.vector.SyncVectorEnv` is determined by `batch_size`, that is 10.

3. On the website of gym, there is a well-written [tutorial on VectorEnv](https://www.gymlibrary.dev/content/vectorising/).  

