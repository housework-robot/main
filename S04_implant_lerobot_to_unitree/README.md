# Design spec: Implant LeRobot brain to Unitree Go2 robotic dog

## 1. Motivation

Why implant LeRobot as brain to Unitree Go2 robitic dog?

### 1.1 Make the dog smarter

1. endow the dog with AI models,

2. utilize LeRobot's toolkits and APIs off the rack, reducing the cost of learning and development,

3. LeRobot/Huggingface is a community and an ecosystem, which already has a large number of available toolkits, and there will be more in the future.

### 1.2 Become a member of LeRobot/Huggingface community
    
1. It can make it easier for users in the community to use the dog, thereby increasing the dog's market,
    
2. It can enable other systems in the ecosystem to interface with the dog, making the dog a component of a larger, more complex, and more intelligent system.


## 2. Deployments

### 2.1 Deployed on server

We deploy LeRobot on the server, when we want to collect training dataset and train the LeRobot motion model. 

### 2.2 Deployed on dog

We deploy LeRobot on the dog body,  when
  
1. the robotic dog operates independently and uses the LeRobot brain locally, 
    
2. a remote server monitors and records the movements of the robotic dog, 
    
3. if necessary, human operators can intervene with the robotic dog through remote control.   


## 3. Use cases  

The detailed usage, especially how to use the APIs in Python program, refers to [Getting Started with Real-World Robots](https://github.com/huggingface/lerobot/blob/main/examples/7_get_started_with_real_robot.md). 

### 3.1 Recalibrate the dog

In [Unitree's document](https://support.unitree.com/home/en/developer/sports_services), calibration mode is called "damp" mode. 

~~~
$ python lerobot/scripts/control_robot.py calibrate
~~~

### 3.2 Remote control the dog

To control the follower arm by moving the leader arm, this process known as `teleoperate` in [control_robot.py](https://github.com/huggingface/lerobot/blob/main/lerobot/scripts/control_robot.py), 

Different from `teleoperate`, `remote_control` is to control the robotic dog remotely and manually by human operator. 

1. Remote control the unitree robotic dog step by step manually
  
    Use keyboard to manually remote control the robot's motion. 
    To exit with CTRL+C. 

    ~~~
    $ python lerobot/scripts/control_robot.py remote_control
        --manual 
    ~~~


2. Remote control the unitree robotic dog with a pre-defined series of motions.

    The time interval between neighboring motions can be either fixed length or free defined.    
  
    ~~~
    $ python lerobot/scripts/control_robot.py remote_control \
        --plan $USER/unitree_test/motion_plan.yaml
    ~~~

Question: can we remote control the dog with fixed command frequency?


### 3.3 Record the leg movement of the dog

Usually we open 2 terminals, one for remote control, the other for recording. 

1. Record one episode in order to test replay

    ~~~
    python lerobot/scripts/control_robot.py record \
        --fps 30 \
        --root tmp/data \
        --repo-id $USER/unitree_test \
        --num-episodes 1 \
        --run-compute-stats 0
    ~~~


2. Record a full dataset in order to train a policy

    With 2 seconds of warmup, 30 seconds of recording for each episode, and 10 seconds to reset the environment in between episodes.

    ~~~
    python lerobot/scripts/control_robot.py record \
        --fps 30 \
        --root data \
        --repo-id $USER/unitree_test \
        --num-episodes 50 \
        --warmup-time-s 2 \
        --episode-time-s 30 \
        --reset-time-s 10
    ~~~

Question: can we control the frequency of recording, when the fps of the video uploaded by the dog is out of our control?



### 3.4 Visualization

1. Visualize dataset

    ~~~  
    python lerobot/scripts/visualize_dataset.py \
        --root tmp/data \
        --repo-id $USER/unitree_test \
        --episode-index 0
    ~~~

    

2. Replay this test episode

    ~~~
    python lerobot/scripts/control_robot.py replay \
        --fps 30 \
        --root tmp/data \
        --repo-id $USER/unitree_test \
        --episode 0
    ~~~    

Question: Do we need a simulator to display the movement of the dog? Without simulation, we can only display the bare bone data. 


### 3.5 Training a policy model

Train on this dataset with an AI policy:

~~~
DATA_DIR=data python lerobot/scripts/train.py \
    policy=unitree_policy \
    env=unitree_real \
    dataset_repo_id=$USER/unitree_pick_lego \
    hydra.run.dir=outputs/train/unitree_policy
~~~

### 3.6 Using a policy model

You can deploy the LeRobot with the pretrained policy model on the dog, so that the dog can act independently without the help of any remote servers. 

You can also deploy the LeRobot with the pretrained policy model on a remote server, and the policy model remote controls the dog's movement. 

~~~
python lerobot/scripts/control_robot.py record \
    --fps 30 \
    --root data \
    --repo-id $USER/eval_act_unitree_policy \
    --num-episodes 10 \
    --warmup-time-s 2 \
    --episode-time-s 30 \
    --reset-time-s 10
    -p outputs/train/unitree_policy/checkpoints/080000/pretrained_model
~~~

Question: Where to find the motion model for quadruped?


>
> Draft, might be modified intensively, Aug 19 2024.
>

