import time
import datetime

from unitree_sdk2py.core.channel import (ChannelSubscriber, ChannelFactoryInitialize)
from unitree_sdk2py.idl.unitree_go.msg.dds_ import (LowState_, SportModeState_)
from unitree_sdk2py.go2.robot_state.robot_state_client import RobotStateClient
from unitree_sdk2py.go2.sport.sport_client import SportClient
from unitree_sdk2py.utils.crc import CRC


class Singleton(type):
    _instances = {}
    def __call__(cls, *args, **kwargs):
        if cls not in cls._instances:
            cls._instances[cls] = super(Singleton, cls).__call__(*args, **kwargs)
        else:
            # cls._instances[cls].__init__(*args, **kwargs)
            pass
        
        return cls._instances[cls]


# In Python 2        
# class Go2Channel(object):
#    __metaclass__ = Singleton
# 
# In Python 3       
# class Go2Channel(metaclass=Singleton)
#
class Go2Channel(metaclass=Singleton):
    LOW_LEVEL = "LOW_LEVEL"
    HIGH_LEVEL = "HIGH_LEVEL"

    def __init__(self, ether_name: str=""):
        self.ether_name = ether_name
        print(f"[INFO] Go2 channel, ethernet name: '{self.ether_name}'\n ")

        if (self.ether_name is not None) and len(self.ether_name) > 1:
            ChannelFactoryInitialize(0, self.ether_name)
        else:
            ChannelFactoryInitialize(0)  

        # Cyclic Redundancy Check, to verify message integrity for unitree-go2 low_level command message.
        self.crc = CRC()    

        self.motion_mode = self.HIGH_LEVEL   
        self.go2_listener, self.go2_commander = None, None


    def switch_mode(self, service_mode: str=HIGH_LEVEL):
        print(f"\n[INFO] service_mode: {service_mode} \n")
        self.motion_mode = service_mode

        print(f"\n[INFO] Wait 15 seconds for the unitree-go2 robotic dog to switch service mode ...\n")
        # Switch off 'sport_mode' service to enable low-level motion control.
        # switch_service needs 15 seconds to execute the whole process. 

        if self.motion_mode == self.LOW_LEVEL: 
            self._switch_service("sport_mode", False)

            # Create a subscriber to receive the latest robot low-level state every certain seconds 
            self.go2_listener = ChannelSubscriber("rt/lowstate", LowState_)
            print(f"\n[INFO] low_state channel name: '{self.go2_listener.get_channel_name()}'\n")

        elif self.motion_mode == self.HIGH_LEVEL:    
            self._switch_service("sport_mode", True)
        
            # Create a subscriber to receive the latest robot high-level state every certain seconds 
            self.go2_listener = ChannelSubscriber("rt/sportmodestate", SportModeState_)
            self.go2_listener.Init(self.go2_listener_handler, 10)  
            print(f"\n[INFO] high_state channel name: '{self.go2_listener.get_channel_name()}'\n")

            self.go2_commander = SportClient()
            self.go2_commander.SetTimeout(10.0)
            self.go2_commander.Init()
        
            print(f"\n[INFO] BEfore doing various actions, stand up first to be ready. \n")
            self.go2_commander.StandUp()
            time.sleep(1)

        else:
            self.go2_listener = self.go2_commander = None
            print(f"[ERROR] Failed to set up ChannelSubscriber!! \n")


    """
    on_off = True: switch on (1), status (0) 
    on_off = False: switch off (0), status (1)   
    """
    def _switch_service(self, service_name: str, on_off: bool):
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
   

    # def go2_listener_handler(self, msg: LowState_ | SportModeState_):
    def go2_listener_handler(self, msg):

        # print(f"\n[INFO] go2_listener_handler() received message: \n{msg}\n")
        """
        if (msg is not None) and (self.state_queue is not None):
            now_time = datetime.datetime.now()
            now_str = now_time.strftime("%Y%m%d%H%M")
            vel_pos = {
                "velocity": msg.velocity, 
                "position": msg.position
            }
            print(f"\n[INFO] Put [{self.msg_count}] message: {vel_pos} into state_queue.\n")
            self.state_queue.put(vel_pos)
            self.msg_count += 1
            
        else:
            print(f"\n[WARN] Tried to push a None into the shared queue, that seems to be a mistake.\n")
        """
        
        pass


    def go2_commander_handler(self, msg):
        pass   # To be implemented. 


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


# ------------------------------------

# Python 2        
# class Logger(object):
#    __metaclass__ = Singleton

# Python 3
class Logger(metaclass=Singleton):
    pass



def testrun_singleton():
    num = 5
    eth_name = "enx207bd51a15b6"
    channels = [None] * num
    for idx in range(num):
        channels[idx] = Go2Channel(eth_name)
        print(f"[{idx}] channel: {channels[idx]} \n")

if __name__ == "__main__":
    testrun_singleton()