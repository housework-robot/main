import sys
import time
import datetime
import numpy as np
import json
import pprint

import zmq
# from pyquaternion import Quaternion

from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_
from unitree_sdk2py.go2.robot_state.robot_state_client import RobotStateClient
from unitree_sdk2py.utils.crc import CRC
import unitree_legged_const as go2

class Go2Channel:
    LOW_LEVEL = "LOW_LEVEL"
    HIGH_LEVEL = "HIGH_LEVEL"

    def __init__(self, ether_name: str=""):
        self.ether_name = ether_name
        
        self.go2_sub = None
        self.motion_level = ""
        self.motion_state = None

        # The timestamp when receives the motion state from unitree-go2 for the last time. 
        self.latest_timestamp = time.time()

        # When starting `sub_loop()`, a loop to listen to the motion state from the go2 robotic dog, continuously. 
        # If the time interval from now to `self.latest_timestamp`, is beyond `self.max_time_interval`, the listener loop will quit.
        self.max_time_interval = 6.0

        # A buffer list contains the latest 10 low-level and high-level
        self.state_buffer = StateBuffer() 

        # Cyclic Redundancy Check, to verify message integrity for unitree-go2 low_level command message.
        self.crc = CRC()

        # Zmq req client, send the latest go2 motion state to the zmq rep server. 
        self.zmq_channel = ZmqChannel()

    def init_sub(self, service_mode: str=HIGH_LEVEL):

        if (self.ether_name is not None) and len(self.ether_name) > 1:
            ChannelFactoryInitialize(0, self.ether_name)
        else:
            ChannelFactoryInitialize(0)        

        self.go2_sub = None

        print(f"\n[INFO] service_mode: {service_mode} \n")
        self.motion_level = service_mode

        print(f"\n[INFO] Wait 15 seconds for the unitree-go2 robotic dog to warm up...\n")
        # Switch off 'sport_mode' service to enable low-level motion control.
        # service_switch needs 15 seconds to execute the whole process. 

        if self.motion_level == self.LOW_LEVEL: 
            self.service_switch("sport_mode", False)

            # Create a subscriber to receive the latest robot low-level state every certain seconds 
            self.go2_sub = ChannelSubscriber("rt/lowstate", LowState_)
            print(f"\n[INFO] low_state channel name: '{self.go2_sub.get_channel_name()}'\n")

        elif self.motion_level == self.HIGH_LEVEL:    
            self.service_switch("sport_mode", True)
        
            # Create a subscriber to receive the latest robot high-level state every certain seconds 
            self.go2_sub = ChannelSubscriber("rt/sportmodestate", SportModeState_)
            print(f"\n[INFO] high_state channel name: '{self.go2_sub.get_channel_name()}'\n")
        
        else:
            self.go2_sub = None
            print(f"[ERROR] Failed to set up ChannelSubscriber!! \n")

        if self.go2_sub is not None:    
            self.go2_sub.Init(self.go2_sub_handler, 10)  

    # def go2_sub_handler(self, msg: LowState_ | SportModeState_):
    def go2_sub_handler(self, msg: LowState_ | SportModeState_):
        # print(f"\n[INFO] motion_state: \n{msg}\n")
        self.motion_state = msg

    """
    on_off = True: switch on (1), status (0) 
    on_off = False: switch off (0), status (1)   
    """
    def service_switch(self, service_name: str, on_off: bool):
        # Switch on or off a service, 
        # e.g. switch off 'sport_mode' service to enable low-level motion control.
        self.robot_state = RobotStateClient()
        self.robot_state.Init()

        print(f"\n[INFO] service_list BEFORE switching off {service_name}:")
        _, service_list = self.robot_state.ServiceList()
        # self.print_service_list(service_list)
        self.print_service(service_name, service_list)
        # print(f"[INFO] End of service list. \n")

        self.robot_state.ServiceSwitch(service_name, on_off)
        time.sleep(15.0)

        print(f"\n[INFO] service_list AFTER switching off {service_name}:")
        _, service_list = self.robot_state.ServiceList()
        # self.print_service_list(service_list)
        self.print_service(service_name, service_list)
        # print(f"[INFO] End of service list. \n")           
        

    """
    class ServiceState:
        def __init__(self, name: str = None, status: int = None, protect: bool = None):
            self.name = name
            self.status = status
            self.protect = protect    
    """
    def print_service_list(self, service_list: list):
        print(f"\t When the value of status is 'False', the status is 'off', when 'True', the status is 'on'. So does the protect. ")
        for idx in range(len(service_list)):
            service = service_list[idx]
            print(f"\t [{idx}] name: '{service.name}', status: '{(service.status == 0)}', protect: '{(service.protect == 0)}'")
        print()

    def print_service(self, service_name: str, service_list: list):
        print(f"\t When the value of status is 'False', the status is 'off', when 'True', the status is 'on'. So does the protect. ")
        for idx in range(len(service_list)):
            service = service_list[idx]
            if service_name == service.name:
                print(f"\t Service name: '{service.name}', status: '{(service.status == 0)}', protect: '{(service.protect == 0)}'")
        print()

    """
    # Isaac Sim/Lab: observation terms (order preserved)

    # We change base_lin_vel to root_lin_vel_w, because the unitree hisgh-level motion state contains 'velocity', 
    # which is with respect to the world frame, instead of the body's base frame. 

    # base_lin_vel = ObsTerm(func=mdp.base_lin_vel, noise=Unoise(n_min=-0.1, n_max=0.1))
    root_lin_vel_w = ObsTerm(func=mdp.root_lin_vel_w, noise=Unoise(n_min=-0.1, n_max=0.1))

    # Since the IMU state includes 'gyroscope' angular velocity, which is with respect to body's base frame, 
    # so it is not necessary to do any conversion to the base_ang_vel. 
    base_ang_vel = ObsTerm(func=mdp.base_ang_vel, noise=Unoise(n_min=-0.2, n_max=0.2))

    # We need to project the weight of the dog, 15kg, to the body's base frame. 
    projected_gravity = ObsTerm(
        func=mdp.projected_gravity,
        noise=Unoise(n_min=-0.05, n_max=0.05),
    )

    # We remove velocity_commands for the time being, because for this phase we will not input any commands.
    # velocity_commands = ObsTerm(func=mdp.generated_commands, params={"command_name": "base_velocity"})

    joint_pos = ObsTerm(func=mdp.joint_pos_rel, noise=Unoise(n_min=-0.01, n_max=0.01))
    joint_vel = ObsTerm(func=mdp.joint_vel_rel, noise=Unoise(n_min=-1.5, n_max=1.5))
    actions = ObsTerm(func=mdp.last_action)
    """
    def push(self) -> dict:      
        state_dict = {}

        if self.motion_state is None:
            return state_dict

        """
        # Will move the real2sim convertion to the go2_supervisor on the server side. 

        # base_ang_vel = tensor([[ 3 * float]])
        state_dict["base_ang_vel"] = torch.tensor(self.low_state.imu_state.gyroscope, dtype=torch.float)

        # projected_gravity = tensor([[ 3 * float]])
        # The weight of the unitree go2 dog is 15kg. 
        base_frame = Quaternion(np.array(self.low_state.imu_state.quaternion))
        raw_gravity = np.array([0., 0., -15.0])
        state_dict["projected_count += 1gravity"] = torch.tensor(base_frame.rotate(raw_gravity), dtype=torch.float)

        # joint_pos = tensor([[ 12 * float]])
        joint_qs = [None] * 12
        for joint_idx in range(12):
            joint_qs[joint_idx] = self.low_state.motor_state[joint_idx].q
        state_dict["joint_pos"] = torch.tensor(joint_qs, dtype=torch.float)

        # joint_vel = tensor([[ 12 * float]])
        joint_dqs = [None] * 12
        for joint_idx in range(12):
            joint_dqs[joint_idx] = self.low_state.motor_state[joint_idx].dq        
        state_dict["joint_vel"] = torch.tensor(joint_dqs, dtype=torch.float)

        # actions = tensor([[ 12 * float]])
        state_dict["actions"] = self.latest_action
        """

        try:

            # [4, float] Dog body's orientation quaternion
            state_dict["imu_quaternion"] = self.motion_state.imu_state.quaternion

            # [3, float] Angular velocity information (0->x, 0->y, 0->z)
            state_dict["imu_gyroscope"] = self.motion_state.imu_state.gyroscope

            # [3, float] Acceleration information (0->x, 0->y, 0->z)
            state_dict["imu_accelerometer"] = self.motion_state.imu_state.accelerometer

            # [3, float] Euler angle information: defaults to the radian value 
            # which can be changed to the angle value according to the actual situation, 
            # and can be displayed according to the actual value 
            # (radian value range: -7-+7, displaying 3 decimal places). 
            # (Array: 0-roll (roll angle), 1-pitch (pitch angle), 2-yaw (yaw angle)).
            state_dict["imu_rpy"] = self.motion_state.imu_state.rpy

            if self.motion_level == self.LOW_LEVEL:
                # [12, float] Shutdown feedback position information: 
                # The default value is the radian value, 
                # (which can be changed to the angle value according to the actual situation), 
                # and it can be displayed according to the actual value 
                # (radian value range: -7-+7, displaying 3 decimal places).
                joint_qs = [None] * 12
                for joint_idx in range(12):
                    joint_qs[joint_idx] = self.motion_state.motor_state[joint_idx].q
                state_dict["joint_q"] = joint_qs

                # [12, float] Joint feedback speed
                joint_dqs = [None] * 12
                for joint_idx in range(12):
                    joint_dqs[joint_idx] = self.motion_state.motor_state[joint_idx].dq        
                state_dict["joint_dq"] = joint_dqs      

                # [12, float] Joint feedback acceleration
                joint_ddqs = [None] * 12
                for joint_idx in range(12):
                    joint_ddqs[joint_idx] = self.motion_state.motor_state[joint_idx].ddq        
                state_dict["joint_ddq"] = joint_ddqs  

                # [12, float] Joint feedback torque 
                joint_tau = [None] * 12
                for joint_idx in range(12):
                    joint_tau[joint_idx] = self.motion_state.motor_state[joint_idx].tau_est        
                state_dict["joint_tau"] = joint_tau 

                self.state_buffer.push(state_dict)

            elif self.motion_level == self.HIGH_LEVEL:

                state_dict["gait_type"] = self.motion_state.gait_type
                state_dict["foot_raise_height"] = self.motion_state.foot_raise_height
                state_dict["position"] = self.motion_state.position
                state_dict["body_height"] = self.motion_state.body_height
                state_dict["velocity"] = self.motion_state.velocity
                state_dict["yaw_speed"] = self.motion_state.yaw_speed
                state_dict["range_obstacle"] = self.motion_state.range_obstacle
                state_dict["foot_force"] = self.motion_state.foot_force
                state_dict["foot_position_body"] = self.motion_state.foot_position_body
                state_dict["foot_speed_body"] = self.motion_state.foot_speed_body

                self.state_buffer.push(state_dict)      

            else:  # Both low_level and high_level are not available. 
                print(f"[WARN] motion_level should be either LOW_LEVEL or HIGH_LEVEL")
                state_dict = {""} 
      
        except Exception as e:
            print(f"[WARN] Failed to read motion_state, error '{str(e)}'\n\n")
            state_dict = {""} 
            
        return state_dict

    """
    A loop to listen to the motion state from the go2 robotic dog, continuously. 
    If the time interval from now to the last time that received the motion state of the dog, 
    is beyond a predefined time interval, the listener loop will quit.
    Before the loop is quit, the last action that the loop does is to send a notice to the supervisor server. 
    """
    def sub_loop(self, max_run: int=sys.maxsize):
        self.latest_timestamp = time.time()
        time_interval = 0.0

        count, signal, req, rep = 0, "", {}, {}
        while (count < max_run) and (signal.lower() != "quit"):
            time.sleep(1.0)
            print(f"\n Motion state listener loop [{count}]: ")

            # self.push() returns the latest motion state.
            state_dict = self.push()
            self.motion_state = None
            print(f"\t latest_idx: {self.state_buffer.latest_idx}")
            print(f"\n[INFO] state_dict: ")
            pprint.pprint(state_dict)

            if (state_dict is None) or len(state_dict) == 0:
                now_time = time.time()
                time_interval = now_time - self.latest_timestamp

                if time_interval > self.max_time_interval:               
                    req = {
                        "type": "info", 
                        "info": f"No connection to go2 for {time_interval} seconds",
                        "instruction": "Either quit the zmq server, or do nothing and wait."            
                    }
                    rep = self.zmq_channel.send_req(req)  
                    print(f"[INFO] Inform the supervisor that the connection to the go2 is not available, ", end="")   
                    print(f"after then receive its reply: {rep} \n\n")    

                    self.latest_timestamp = time.time()
                    time_interval = 0.0             

            else:
                self.latest_timestamp = time.time()
                time_interval = 0.0

                req = {
                    "type": "data", 
                    "data": state_dict,
                    "instruction": f"[{count}]"            
                }
                rep = self.zmq_channel.send_req(req)  
                print(f"[INFO] Send the latest motion state to the supervisor, after then receive its reply: {rep}\n\n")          

            if len(rep) > 0 and (rep["type"].lower() == "command"):
                cmd = rep["command"]
                if cmd.lower() == "quit":
                    signal = "quit"
                    print(f"[WARN] Receive `quit` command from the supervisor, will quit in 5 seconds. \n\n\n")
                    time.sleep(5.0)

            count += 1

class StateBuffer:
    def __init__(self, buffer_size: int=10):
        self.buffer_list = [None] * 10
        self.latest_idx = 0
        self.buffer_size = buffer_size

    def push(self, element):
        if element is None:
            print(f"[WARN] Some one tries to pushed a none into the state_buffer. This must be a mistake.")
            return 
        
        if self.latest_idx == (self.buffer_size - 1):
            self.latest_idx = 0
        elif self.buffer_list[self.latest_idx] is None:
            # Usually this case occurs when self.latest_idx == 0 at the very beginning.
            # print(f"[INFO] self.buffer_list[{self.latest_idx}] is none.\n") 
            pass
        else:
            self.latest_idx += 1

        # print(f"\t latest_idx: {self.latest_idx}")
        self.buffer_list[self.latest_idx] = element

    def get_latest(self) -> dict:
        latest_element = self.buffer_list[self.latest_idx]
        return latest_element

    def print(self):
        print(f"\n\n\n[INFO] The state_buffer contains {self.buffer_size} elements.")
        print(f"\t latest_idx is: {self.latest_idx}")

        for idx in range(self.latest_idx, -1, -1):
            element = self.buffer_list[idx]
            print(f"\n\tState buffer [{idx}]: ")
            pprint.pprint(element)
   
        next_idx = self.latest_idx + 1
        if next_idx < self.buffer_size:
            next_element = self.buffer_list[next_idx]
            if next_element is not None:
                for idx in range(self.buffer_size - 1, self.latest_idx, -1):
                    element = self.buffer_list[idx]
                    print(f"\n\tState buffer [{idx}]")
                    pprint.pprint(element)
            else:
                print(f"\n\t[WARN] The {next_idx}'th element of the state_buffer is none. \n")

        print(f"\n[INFO] Above is the states in the buffer from the latest to the earliest.\n")

    def write_to_file(self, filename: str):
        buffer_str = json.dumps(self.buffer_list, indent=4)
        print(f"\n[INFO] Write the `self.buffer_list` into a file `{filename}`\n\n")
        with open(filename, "w") as fo:
            fo.write(buffer_str)

    def read_from_file(self, filename: str) -> list:
        with open(filename, "r") as fi:
            buffer_list = json.load(fi)

        print(f"\n[INFO] Following is the state_buffer's content from a file '{filename}'. \n")
        pprint.pprint(buffer_list)
        print("\n\n")

class ZmqChannel:
    def __init__(self, ip_addr: str="192.168.0.122"):
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.REQ)

        url = f"tcp://{ip_addr}:5555"
        self.socket.connect(url)

    """
    The data format of the request is a dict/json,
    {
        "type": "data|info", 
        "data": <class 'numpy.ndarray'>,
        "instruction": "If necessary, write an instruction to guide the receiver how to process the data."
    }

    The data format of the reply is also a dict/json,
    {
        "type": "command|info"
        "command": <class 'dict'>, # key is the command name, value is the parameters.
        "instruction": "If necessary, write an instruction to guide the sender how to execute the command."
    }
    """
    def send_req(self, req: dict) -> dict:
        self.socket.send_json(req)
        message = self.socket.recv()
        rep = json.loads(message)
        return rep  

def generate_rand_dataset(seed: int=19680801):
    np.random.seed(seed)  # seed the random number generator.
    
    dataset = np.random.randn(3, 40) * 10.0
    """
    dataset = [None] * 3
    for idx in range(3):
        dataset[idx] = [np.random.uniform(-10.0, 10.0) for _ in np.arange(40)]    
    """

    print(f"\n\nfull dataset: \n{dataset} \n\n")
    print(f"\n\nsliced dataset[0:2, 0:10]: \n{dataset[0:2, 0:10]} \n\n")
    return dataset

def testrun_state_buffer():
    state_buffer = StateBuffer()
    for idx in range(29):
        state_buffer.push(idx)
    state_buffer.print()

def testrun_zmq():
    zmq_channel = ZmqChannel()
    test_dataset = generate_rand_dataset()
    print(f" test_dataset, type: {type(test_dataset)}, \n\tshape: {test_dataset.shape}, \n\tlen: {test_dataset.shape[1]}\n")

    test_dataset_dim = test_dataset.shape[0]
    test_dataset_len = test_dataset.shape[1]
    for idx in range(test_dataset_len):
        print(f"[{idx}] {test_dataset[:, idx]}")
        req = {
            "data": test_dataset[:, idx].tolist(),
            "instruction": f"[{idx}]"            
        }
        rep = zmq_channel.send_req(req)  
        print(f"\tReceive server's reply: {rep}\n\n")

def testrun_time_internval():
    early_datetime = datetime.datetime.now()
    print(f"\n\nFrom datetime: {early_datetime} \n")

    early_time = early_datetime.timestamp()
    print(f"From time: {early_time} \n")

    print(f"  sleep for 5 seconds...\n")
    time.sleep(5)

    now_time = time.time()
    print(f"From time: {now_time} \n")
    now_datetime = datetime.datetime.fromtimestamp(now_time)
    print(f"From datetime: {now_datetime} \n")

    time_interval = now_time - early_time
    print(f"From time: interval = '{time_interval}' \n")

def testrun_go2_motion():
    go2channel = Go2Channel("enx207bd51a15b6")

    # go2channel.init_sub(Go2Channel.LOW_LEVEL)
    go2channel.init_sub(Go2Channel.HIGH_LEVEL)
    # go2channel.sub_loop(120)
    go2channel.sub_loop()
    
    # go2channel.state_buffer.print()
    filename = "state_buffer.json"
    go2channel.state_buffer.write_to_file(filename)
    go2channel.state_buffer.read_from_file(filename)

if __name__ == "__main__":
    # testrun_time_internval()
    # testrun_state_buffer()
    # testrun_zmq()

    testrun_go2_motion()
