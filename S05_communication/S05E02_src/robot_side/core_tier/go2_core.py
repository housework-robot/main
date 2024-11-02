import numpy as np
import os
import sys
import datetime

from example.robot_side.down_tier.unitree_go2.go2_media import Go2Camera
from example.robot_side.up_tier.rtmp_sender import RtmpSender

class Go2Core:
    def __init__(self, ethname: str):
        self.rtmp_sender = RtmpSender()
        self.video_src = Go2Camera(ethname)       

    def push_video_stream(self):
        self.rtmp_sender.set_video_src(self.video_src)
        self.rtmp_sender.run()         


def testrun_go2_rtmp():
    go2_core = Go2Core("enx207bd51a15b6")
    go2_core.push_video_stream()


if __name__ == "__main__":
    testrun_go2_rtmp()