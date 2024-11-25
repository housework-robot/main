import time
import json
from pySerialTransfer import pySerialTransfer as txfer

class SerialChannel:
    def __init__(self):
        self.dev_tty = "/dev/ttyUSB0"
        self.channel = txfer.SerialTransfer(self.dev_tty, baud=115200)
        
        self.channel.open()
        time.sleep(2)
        print(f"[INFO] SerialChannel is ready. \n")


    def send_json(self, send_data: dict) -> dict:
        status = {}
        if len(send_data) == 0:
            status["ERROR"] = "empty data"
            return status
    
        try:
            send_str = json.dumps(send_data, ensure_ascii=False)
            send_size = self.channel.tx_obj(send_str, start_pos=0) 
            self.channel.send(send_size)

        except Exception as e:
            status["ERROR"] = str(e)    
            return status

        status["SUCCESS"] = ""
        return status


    def receive_json(self) -> dict:
        recv_json = {}
                    
        # A negative value for status indicates an error
        while not self.channel.available():
            if self.channel.status < 0:
                recv_json["ERROR"] = self.channel.status
            time.sleep(0.1)

        if self.channel.available():
            recv_str = self.channel.rx_obj(
                obj_type=str,
                start_pos=0,
                obj_byte_size=self.channel.bytesRead
            )

            recv_json = json.loads(recv_str)
        
        return recv_json
    

def testrun_serial():
    channel = SerialChannel()

    cnt = 0
    send_data = {
        "cnt": cnt, 
        "throttle": 0.11,
        "steer": 0.22
    }
    status = channel.send_json(send_data)

    while True:
        print(f"[{cnt}]")
        receive_data = channel.receive_json()
   
        if "ERROR" in receive_data:
            print(f"receive_data ERROR: {receive_data} \n")
        elif len(receive_data) == 0:
            time.sleep(1)
        else:
            print(f"receive_data: {receive_data}")
        
        send_data["cnt"] = cnt
        send_data["throttle"] = send_data["throttle"] + 0.01
        send_data["steer"] = send_data["steer"] + 0.01

        channel.send_json(send_data)
        print(f"sendStr = '{send_data}' \n ")

        cnt += 1
 


if __name__ == "__main__":
    testrun_serial()