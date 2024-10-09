import sys
import time
import datetime
import zmq
import json
import pprint

import numpy as np
import matplotlib.pyplot as plt

class Matplot:
    def __init__(self, x_max: int=50):
        plt.ion()

        self.x_time_max = x_max
        self.y_range = [
            [[-2.0, 2.0], [-1.5, 1.5]], 
            [[-1.0, 1.0], [-0.5, 0.5]], 
            [[2.0, 12.0], [-0.3, 0.3]]
        ]
  
        self.axis_names = ['X', 'Y', 'Z']
        self.colors = ['#cc000080', '#00990080', '#0000cc80']

        self.fig, self.axs = plt.subplots(
            3, 2,
            figsize=(20, 10), 
            sharex=True, 
            # layout='constrained'
        )

        self.scatter = np.empty(shape=(3, 2), dtype='object')
        self.draw_artists()

        self.fig.suptitle(
            "Accelerometer vs Velocity Data Stream", 
            fontsize=16
        )

        plt.subplots_adjust(
            left=0.1, right=0.9, wspace=0.3,
            top=0.9, hspace=0.3
        )

        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()

    def clear_scatter(self):
        for row_idx in range(3):
            for col_idx in range(2):
                if self.scatter[row_idx, col_idx] is not None:
                    self.scatter[row_idx, col_idx].remove()    

    def draw_artists(self):
        for row_idx in range(3):
            left_subtitle = f"Acceleration on {self.axis_names[row_idx]}-axis"  
            right_subtitle = f"Velocity on {self.axis_names[row_idx]}-axis"
             
            for col_idx in range(2):
                self.axs[row_idx, col_idx].clear()
                
                if col_idx == 0:
                    self.axs[row_idx, col_idx].set_title(left_subtitle)
                    self.axs[row_idx, col_idx].set_ylabel("Acceleration (m/s^2)")
                else:
                    self.axs[row_idx, 1].set_title(right_subtitle) 
                    self.axs[row_idx, col_idx].set_ylabel("Velocity (m/s)")

                self.axs[row_idx, col_idx].set_ylim(
                    self.y_range[row_idx][col_idx][0], 
                    self.y_range[row_idx][col_idx][1]
                )   
                self.axs[row_idx, col_idx].set_xlim(0, self.x_time_max)

                if row_idx == 2:
                    self.axs[row_idx, col_idx].set_xlabel("Time steps")

    """
    The format of go2 dataset is like,
    [[x0, x1, x2, ...], [y0, y1, y2, ...], [z0, z1, z2, ...]]
    """
    def draw_scatter(self, go2_dataset):
        self.clear_scatter()

        for row_idx in range(3):           
            for col_idx in range(2):
                # go2_dataset_len = len(go2_dataset[row_idx])
                go2_dataset_len = go2_dataset.shape[2]
                self.scatter[row_idx, col_idx] = self.axs[row_idx, col_idx].scatter(
                    np.arange(go2_dataset_len),  # time ticks.
                    go2_dataset[row_idx, col_idx],
                    c=[self.colors[row_idx]] * go2_dataset_len,
                    s=40  # dot radius
                )

        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()

class StateBuffer:
    def __init__(self, x_max: int=50):
        self.x_time_max = x_max

        self.latest_idx = 0
        self.state_buffer = np.zeros([3, 2, self.x_time_max], dtype=float)

    def push(self, latest_state: np.ndarray):
        if latest_state is None:
            print(f"[WARN] Some one tries to pushed a none into the state_buffer. This must be a mistake.")
            return 

        if self.latest_idx == 0:
            if np.array_equal(self.state_buffer[:,:,0], np.zeros([3,2], dtype=float)):
                pass  # At the very beginning, the 0'th element is zero, don't update latest_idx.
            else:
                self.latest_idx += 1
        else:
            self.latest_idx += 1

        insert_idx = self.latest_idx
        insert_idx = insert_idx % self.x_time_max
        
        self.state_buffer[:, :, insert_idx] = latest_state
        print(f"  push [{self.latest_idx}]: {latest_state} at index [{insert_idx}] \n")
        # print(f"\t self.state_buffer: {self.state_buffer[:,:]} \n")

    def ordered_state_buffer(self):
        if self.latest_idx < self.x_time_max:
            ordered_array = self.state_buffer[:, :, 0 : (self.latest_idx + 1)]
        else:
            truncated_idx = self.latest_idx % self.x_time_max        
            right_array = self.state_buffer[:, :, 0 : (truncated_idx + 1)]
            left_array = self.state_buffer[:, :, (truncated_idx + 1) : ]
            ordered_array = np.concatenate([left_array, right_array], axis=2)

        return ordered_array

    def print(self):
        print(f"\n\n[INFO] Following is the content of `self.state_buffer`: ")
        print(f"   latest_idx is: {self.latest_idx} \n")

        ordered_array = self.ordered_state_buffer()

        with np.printoptions(precision=5, suppress=True):
            print(ordered_array)
        print(f"\n[INFO] Above is the states in the buffer from the latest to the earliest.\n\n")

class ZmqServer:
    def __init__(self):
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.REP)
        self.socket.bind("tcp://*:5555")   

        x_max = 60
        self.matplotter = Matplot(x_max)
        self.state_buffer = StateBuffer(x_max)
        self.log_filename = "go2_log.jsonl"

        """
        matplotter = Matplot()
        dataset = generate_rand_dataset()

        for idx in range(matplotter.x_time_max):
            dataslice = dataset[0:3, 0:idx]
            matplotter.draw_scatter(dataslice)
            time.sleep(1.5)    
        """

        print(f"\n\n[INFO] The go2_supervisor server is listening ...\n")

    def add_to_log(self, req_str: str=""):
        with open(self.log_filename, "a") as fo:
            fo.write(f"{req_str} \n")

    def receive_loop(self, max_loop: int=sys.maxsize):
        count = 0
        while count < max_loop:
            # Wait for next req from client
            message = self.socket.recv()
            req = json.loads(message)

            req_str = json.dumps(req)
            print(f"Received req: \n{req_str} \n")
            self.add_to_log(req_str)

            # Do some work here
            req_type = req["type"]
            if req_type.lower() == "data":
                accelerometer = req["data"]["imu_accelerometer"]
                velocity = req["data"]["velocity"]
                accel_vel = np.stack((accelerometer, velocity), axis=1)
                self.state_buffer.push(accel_vel)

                ordered_state_buffer = self.state_buffer.ordered_state_buffer()
                self.matplotter.draw_scatter(ordered_state_buffer)
                pprint.pprint(ordered_state_buffer)
                print(f"\n")
                time.sleep(1.0)

                # Send reply back to the client
                self.socket.send_json({"type": "info", "instruction": f"{accel_vel}" }) 

            else:
                pprint.pprint(req)
                # Send reply back to the client
                info = req["info"]
                print(f"[INFO] Received info from the go2: {info}\n")
                print(f"[WARN] Send `quit` command to the go2, aand expect it to quit in 5 seconds. \n\n\n")
                self.socket.send_json({
                    "type": "command", 
                    "command": "quit" 
                }) 

            count += 1 

        print(f"[INFO] loop more than '{max_loop}' times, will quit in a moment. ")               

def generate_rand_dataset(len: int=100, seed: int=19680801):
    np.random.seed(seed)  # seed the random number generator.
    
    dataset = np.random.randn(3, 2, len)
    """
    dataset = [None] * 3
    for idx in range(3):
        dataset[idx] = [np.random.uniform(-10.0, 10.0) for _ in np.arange(40)]    
    """

    # print(f"\n\ndataset: {dataset[0:2, 0:6]} \n\n")
    pprint.pprint(dataset)
    return dataset

def testrun_state_buffer():
    now_time = datetime.datetime.now()
    now_str = now_time.strftime("%Y%m%d%H%M")
    print(f"time: {now_time}, time str: {now_str} \n")

    state_buff = StateBuffer(15)

    latest_state = state_buff.state_buffer[:, :, state_buff.latest_idx]
    print(f"\n latest_idx: {state_buff.latest_idx}, latest_state: {latest_state} ")
    print(f"\n np.zeros([3,2], dtype=float): {np.zeros([3,2], dtype=float)} \n")

    dataset = generate_rand_dataset(50)[:,:,:28]
    print(f"\n dataset.shape: {dataset.shape} \n")

    for idx in range(dataset.shape[2]):
        state_buff.push(dataset[:,:,idx])

    state_buff.print()

def testrun_matplot():
    matplotter = Matplot()
    dataset = generate_rand_dataset(80)
    dataslice = None
    dataset_len = dataset.shape[2]
  
    # for idx in range(matplotter.x_time_max):
    for idx in range(dataset_len):
        if idx < matplotter.x_time_max:
            end_idx = idx + 1
            dataslice = dataset[:, :, :end_idx] 
        else:
            left_start_idx = idx + 1 - matplotter.x_time_max
            left_end_idx = matplotter.x_time_max
            data_left = dataset[:, :, left_start_idx:left_end_idx] 
            right_start_idx = 0
            right_end_idx = idx + 1 - matplotter.x_time_max
            data_right = dataset[:, :, right_start_idx:right_end_idx]
            dataslice = np.concatenate((data_left, data_right), axis=2)

        # dataslice = dataslice * [[1.0, 1.0], [1.0, 1.0], [10.0, 10.0]]
        # dataslice = dataslice + [[0.0, 0.0], [0.0, 0.0], [10.0, 10.0]]
        matplotter.draw_scatter(dataslice)
        time.sleep(0.3)      

    time.sleep(20)

def testrun_zmq():
    zmq_server = ZmqServer()
    now_time = datetime.datetime.now()
    now_str = now_time.strftime("%Y%m%d%H%M")
    zmq_server.log_filename = f"log/go2_log_{now_str}.jsonl"
    zmq_server.receive_loop(120)
    time.sleep(20.0)
 

if __name__ == "__main__":
    # testrun_state_buffer()
    testrun_matplot()
    # testrun_zmq()
