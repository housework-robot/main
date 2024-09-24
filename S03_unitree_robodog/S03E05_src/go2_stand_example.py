import time
import sys
import math

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelFactoryInitialize
from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_
from unitree_sdk2py.utils.crc import CRC
from unitree_sdk2py.utils.thread import Thread
import unitree_legged_const as go2

crc = CRC()



class Go2Channel:
    def __init__(self, ether_name: str=""):
        self.pub = None
        self.sub = None
        self.low_state = None

        if len(ether_name)>1:
            ChannelFactoryInitialize(0, ether_name)
        else:
            ChannelFactoryInitialize(0)

        # Create a publisher to publish the data defined in UserData class
        self.pub = ChannelPublisher("rt/lowcmd", LowCmd_)
        self.pub.Init()

        # Create a subscriber to receive the latest robot state every certain seconds 
        self.low_state = None
        self.sub = ChannelSubscriber("rt/lowstate", LowState_)
        self.sub.Init(self.LowStateMessageHandler, 10)      

    # Not used, but usable.
    def LowStateHandler(self, msg: LowState_):    
        # print front right hip motor states
        print("FR_0 motor state: ", msg.motor_state[go2.LegID["FR_0"]])
        print("RL_2 motor state: ", msg.motor_state[go2.LegID["RL_2"]])
        print("RL_0 motor state: ", msg.motor_state[go2.LegID["RL_0"]])
        print("IMU state: ", msg.imu_state)
        print("Battery state: voltage: ", msg.power_v, "current: ", msg.power_a)
        print(f"\n")  

    def LowStateMessageHandler(self, msg: LowState_):
        self.low_state = msg

    def ReadSensorData(self):
        """
        /home/robot/unitree/unitree_sdk2_python/unitree_sdk2py/idl/unitree_go/msg/dds_/_MotorState_.py 
        class MotorState_(idl.IdlStruct, typename="unitree_go.msg.dds_.MotorState_"):
            mode: types.uint8
            q: types.float32
            dq: types.float32
            ddq: types.float32
            tau_est: types.float32
            q_raw: types.float32
            dq_raw: types.float32
            ddq_raw: types.float32
            temperature: types.uint8
            lost: types.uint32
            reserve: types.array[types.uint32, 2]

        class IMUState_(idl.IdlStruct, typename="unitree_go.msg.dds_.IMUState_"):
            quaternion: types.array[types.float32, 4]
            gyroscope: types.array[types.float32, 3]
            accelerometer: types.array[types.float32, 3]
            rpy: types.array[types.float32, 3]
            temperature: types.uint8
        """

        print(f"\n\n[INFO] Read sensor data example: ")
        print(f"\tJoint 0 pos: {self.low_state.motor_state[0].q}")
        print(f"\tImu accelerometer: {self.low_state.imu_state}")
        print(f"\tFoot force: {self.low_state.foot_force}")
        print(f"\tThe example is done! \n\n")


    def ReadMotorPosition(self, joint_idx :int=0):
        q = self.low_state.motor_state[joint_idx].q
        return q
    


class Go2Leg:
    def __init__(self):
        self.channel = None
        self.cmd = unitree_go_msg_dds__LowCmd_()
        self._startPos = [0.0] * 12

    @property
    def go2_channel(self):
        return self.channel

    @go2_channel.setter
    def go2_channel(self, channel: Go2Channel):
        self.channel = channel


    def init_cmd(self):
        self.cmd = unitree_go_msg_dds__LowCmd_()
        self.cmd.head[0]=0xFE
        self.cmd.head[1]=0xEF
        self.cmd.level_flag = 0xFF
        self.cmd.gpio = 0
        return self.cmd


    def send_cmd(self):
        self.cmd.crc = crc.Crc(self.cmd)

        #Publish message
        if self.channel.pub.Write(self.cmd):
            print(f"\nPublish success. msg: {self.cmd.motor_cmd} \n\n")
        else:
            print(f"Waitting for subscriber. \n")

    
    def init_state(self):
        self.cmd = self.init_cmd()

        for i in range(20):
            self.cmd.motor_cmd[i].mode = 0x01  # (PMSM) mode
            self.cmd.motor_cmd[i].q= go2.PosStopF
            self.cmd.motor_cmd[i].kp = 0
            self.cmd.motor_cmd[i].dq = go2.VelStopF
            self.cmd.motor_cmd[i].kd = 0
            self.cmd.motor_cmd[i].tau = 0

        self.send_cmd()


    def joint_linear_interpolation(self, init_pos: float, target_pos: float, rate: float):
        p = 0.0
        rate = min(max(rate, 0.0), 1.0)
        p = init_pos * (1.0 - rate) + target_pos * rate
        return p


    def ready_state(self):
        for joint_idx in range(12):
            self._startPos[joint_idx] = self.go2_channel.ReadMotorPosition(joint_idx)

        """
        q_init = [0.0, 0.0, 0.0]
        q_des = [0.0, 0.0, 0.0]
        sim_mid_q = [0.0, 1.2, -2.0]
        k_p = [5.0, 5.0, 5.0]
        k_d = [1.0, 1.0, 1.0]
          
        if self.channel.low_state is None:
            raise Exception("[ERROR] Need to setup the comm channel to Unitree Go2, before setting motion command to it. ")
        
        for rate_count in range(10, 400):
            rate = float(rate_count) / 200.0

            for joint_idx in range(3):
                q_des[joint_idx] = self.joint_linear_interpolation(q_init[joint_idx], sim_mid_q[joint_idx], rate)
            
            for joint_idx in range(3):
                joint_offset = go2.LegID["FR_0"]
                i = joint_offset + joint_idx

                self.cmd.motor_cmd[i].mode = 0x01  # (PMSM) mode
                self.cmd.motor_cmd[i].q= q_des[joint_idx]
                self.cmd.motor_cmd[i].kp = k_p[joint_idx]
                self.cmd.motor_cmd[i].dq = 0.0 # Set to stop angular velocity(rad/s)
                self.cmd.motor_cmd[i].kd = k_d[joint_idx]
                self.cmd.motor_cmd[i].tau = 0.0

                self.send_cmd()                
        """


    def stand_up(self):
        Kp = 60.0
        Kd = 5.0
        time_consume = 0
        rate_count = 0
        sin_count = 0
        motiontime = 0
        dt = 0.002   # 0.001~0.01     

        _targetPos_1 = [0.0, 1.36, -2.65, 0.0, 1.36, -2.65, -0.2, 1.36, -2.65, 0.2, 1.36, -2.65]
        _targetPos_2 = [0.0, 0.67, -1.3, 0.0, 0.67, -1.3, 0.0, 0.67, -1.3, 0.0, 0.67, -1.3]
        _targetPos_3 = [-0.35, 1.36, -2.65, 0.35, 1.36, -2.65, -0.5, 1.36, -2.65, 0.5, 1.36, -2.65]        
    
        _duration_1 = _duration_2 = 500
        _duration_3 = 1000  
        _duration_4 = 900
        _percent_1 = _percent_2 = _percent_3 = _percent_4 = 0.0
        firstRun = True
        done = False


        # Phase 1
        while _percent_1 < 1.0:
            _percent_1 += 1.0 / _duration_1
            _percent_1 = 1.0 if _percent_1 > 1.0 else _percent_1            
            for joint_idx in range(12):
                target_q = (1.0 - _percent_1) * self._startPos[joint_idx] + _percent_1 * _targetPos_1[joint_idx]
                print(f"[INFO] Phase 1. target_q[{joint_idx}]: {target_q}")

                self.cmd.motor_cmd[joint_idx].mode = 0x01  # (PMSM) mode
                self.cmd.motor_cmd[joint_idx].q= target_q
                self.cmd.motor_cmd[joint_idx].kp = Kp
                self.cmd.motor_cmd[joint_idx].dq = 0.0   # Set to stop angular velocity(rad/s)
                self.cmd.motor_cmd[joint_idx].kd = Kd
                self.cmd.motor_cmd[joint_idx].tau = 0.0

                self.send_cmd() 
            print(f"[INFO] Complete phase 1. \n")  


        # Phase 2 
        while (_percent_1 >= 1.0) and (_percent_2 < 1.0):
            _percent_2 += 1.0 / _duration_2
            _percent_2 = 1.0 if _percent_2 > 1.0 else _percent_2

            for joint_idx in range(12):
                target_q = (1.0 - _percent_2) * _targetPos_1[joint_idx] + _percent_2 * _targetPos_2[joint_idx]
                print(f"[INFO] Phase 2. target_q[{joint_idx}]: {target_q}")

                self.cmd.motor_cmd[joint_idx].mode = 0x01  # (PMSM) mode
                self.cmd.motor_cmd[joint_idx].q= target_q
                self.cmd.motor_cmd[joint_idx].kp = Kp
                self.cmd.motor_cmd[joint_idx].dq = 0.0   # Set to stop angular velocity(rad/s)
                self.cmd.motor_cmd[joint_idx].kd = Kd
                self.cmd.motor_cmd[joint_idx].tau = 0.0

                self.send_cmd()   
            print(f"[INFO] Complete phase 2. \n")  


        # Phase 3
        while (_percent_1 >= 1.0) and (_percent_2 >= 1.0) and (_percent_3 < 1.0):
            _percent_3 += 1.0 / _duration_3
            _percent_3 = 1.0 if _percent_3 > 1.0 else _percent_3

            for joint_idx in range(12):
                print(f"[INFO] Phase 3. target_q[{joint_idx}]: {_targetPos_2[joint_idx]}")

                self.cmd.motor_cmd[joint_idx].mode = 0x01  # (PMSM) mode
                self.cmd.motor_cmd[joint_idx].q= _targetPos_2[joint_idx]
                self.cmd.motor_cmd[joint_idx].kp = Kp
                self.cmd.motor_cmd[joint_idx].dq = 0.0   # Set to stop angular velocity(rad/s)
                self.cmd.motor_cmd[joint_idx].kd = Kd
                self.cmd.motor_cmd[joint_idx].tau = 0.0

                self.send_cmd() 
            print(f"[INFO] Complete phase 3. \n")  

     
        # Phase 4
        while (_percent_1 >= 1.0) and (_percent_2 >= 1.0) and (_percent_3 >= 1.0) and (_percent_4 <= 1.0):
            _percent_4 += 1.0 / _duration_4
            _percent_4 = 1.0 if _percent_4 > 1.0 else _percent_4

            for joint_idx in range(12):
                target_q = (1.0 - _percent_4) * _targetPos_2[joint_idx] + _percent_4 * _targetPos_3[joint_idx]
                print(f"[INFO] Phase 4. target_q[{joint_idx}]: {target_q}")

                self.cmd.motor_cmd[joint_idx].mode = 0x01  # (PMSM) mode
                self.cmd.motor_cmd[joint_idx].q= target_q
                self.cmd.motor_cmd[joint_idx].kp = Kp
                self.cmd.motor_cmd[joint_idx].dq = 0.0   # Set to stop angular velocity(rad/s)
                self.cmd.motor_cmd[joint_idx].kd = Kd
                self.cmd.motor_cmd[joint_idx].tau = 0.0

                self.send_cmd()             
            print(f"[INFO] Complete phase 4. \n")  
    

if __name__ == '__main__':

    # 1. Create comm channel
    if len(sys.argv)>1:
        go2_channel = Go2Channel(sys.argv[1])
    else:
        go2_channel = Go2Channel()

    # 2. Deactivate the sport-mode
    # Can unitree-python do this job?

    # 3. Initialize the dog with init state
    go2_leg = Go2Leg()
    go2_leg.go2_channel = go2_channel
    go2_leg.init_state()
    time.sleep(2.0)

    # 4. Read the dog's low-level states
    go2_leg.go2_channel.ReadSensorData()

    # 5. Assign the self._startPos to be the current positions
    go2_leg.ready_state()

    # 6. Stand up demo
    go2_leg.stand_up()

    print(f"[INFO] The standing-up demo will complete in 10 seconds.")
    time.sleep(10.0)
    