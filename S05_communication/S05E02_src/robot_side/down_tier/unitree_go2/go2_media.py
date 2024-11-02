import cv2
import numpy as np
import os
import sys
import datetime

from unitree_sdk2py.core.channel import (ChannelSubscriber, ChannelFactoryInitialize)
from unitree_sdk2py.go2.video.video_client import VideoClient

from example.robot_side.down_tier.unitree_go2.go2_channel import Go2Channel

class Go2Camera:
    def __init__(self, ether_name: str=""):
        self.channel = Go2Channel(ether_name)

        self.video_client = VideoClient() 
        self.video_client.SetTimeout(3.0)
        self.video_client.Init()
        self.camera_name = "Go2 front camera"
        print(f"[INFO] Camera name: '{self.camera_name}'\n")       

        code, data = self.video_client.GetImageSample()
        if code == 0:
            frame_data = np.frombuffer(bytes(data), dtype=np.uint8)
            frame_image = cv2.imdecode(frame_data, cv2.IMREAD_COLOR)
            print(f"[INFO] frame_image.shape: {frame_image.shape} \n")

            self.video_width = frame_image.shape[1]
            self.video_height = frame_image.shape[0]

            print(f"[INFO] Go2Camera.init(): video_width={self.video_width}, video_height={self.video_height}\n")
        else:
            self.video_width = 0
            self.video_height = 0         
            error_msg = f"Referring to unitree_sdk2py/rpc/internal.py, error code == {code}"
            print(f"[ERROR] Go2Camera.init(): {error_msg}\n")
         

    def receive_video(self):
        code, data = self.video_client.GetImageSample()
        if code == 0:
            # Convert to numpy image
            frame_data = np.frombuffer(bytes(data), dtype=np.uint8)
            frame_image = cv2.imdecode(frame_data, cv2.IMREAD_COLOR)
            return True, frame_image
        else:
            error_msg = f"Referring to unitree_sdk2py/rpc/internal.py, error code == {code}"
            return False, error_msg


    # This is a generator function, which yield video frame continuously.
    def receive_video_continuously(self):
        code, data = self.video_client.GetImageSample()

        # Request normal when code==0eth_name
        while code == 0:
            # Get Image data from Go2 robot
            code, data = self.video_client.GetImageSample()

            # Convert to numpy image
            frame_data = np.frombuffer(bytes(data), dtype=np.uint8)
            frame_image = cv2.imdecode(frame_data, cv2.IMREAD_COLOR)

            # Yield frame_image to outsider users. 
            yield frame_image

            # Display image
            # cv2.imshow(self.camera_name, frame_image)

            # Press ESC to stop
            # if cv2.waitKey(20) == 27:
            #    break  
        
        if code != 0:
            print(f"[ERROR] Get image sample error. code: {code}\n\n")
        else:
            # Capture an image
            log_dir = os.getenv("log_dir")
            timestamp = datetime.now().strftime("%Y%m%d%H%M%S")  # Current time with milliseconds
            image_filename = f"{log_dir}/front_image_{timestamp}.jpg"
            cv2.imwrite(image_filename, frame_image)

        # cv2.destroyWindow(self.camera_name)        
        


# Following functions are for testing purpose. 
# -----------------------------------------------------

def testrun_go2_camera():
    go2_camera = Go2Camera("enx207bd51a15b6")
    go2_camera.receive_video_continuously()

    # Display image
    for frame_image in go2_camera.receive_video_continuously():
        cv2.imshow(go2_camera.camera_name, frame_image)

        # Press ESC to stop
        cv2.waitKey(20)

    cv2.destroyWindow(go2_camera.camera_name)


if __name__ == "__main__":
    testrun_go2_camera()


