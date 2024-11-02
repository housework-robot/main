import asyncio
import ffmpeg
import cv2
from datetime import datetime


class RtmpSender:
    def __init__(self):
        self.ip_addr = "127.0.0.1"
        self.port = 1935
        self.rtmp_server_url = f"rtmp://{self.ip_addr}:{self.port}/live/livestream"

        self.video_src = None
        self.video_width = 0
        self.video_height = 0


    def set_video_src(self, video_src):
        self.video_src = video_src
        self.video_width = video_src.video_width
        self.video_height = video_src.video_height


    def ffmpeg_pipeline(self, width, height):
        process = (
            ffmpeg
            .input(
                'pipe:', 
                format='rawvideo',
                codec="rawvideo", 
                # format="v4l2",
                # codec="h264", 
                pix_fmt='bgr24', 
                s=f'{self.video_width}x{self.video_height}'
            )
            .output(
                self.rtmp_server_url,
                format="flv",
                # flvflags="no_duration_filesize",
                # codec = "copy", # use same codecs of the original video
                vcodec="libx264",
                acodec="aac",
                # listen=1, # enables HTTP server, if using ffplay to view the video stream, please remove the #.
                pix_fmt="yuv420p",
                preset="veryfast", # preset="ultrafast",
                tune="zerolatency"
            )
            .overwrite_output()
            .run_async(pipe_stdin=True)
        )
        return process


    async def run_rtmp(self):
        streaming_process = self.ffmpeg_pipeline(
            self.video_width, 
            self.video_height
        )

        try:
            while True:
                ret, frame = self.video_src.receive_video()
                if ret:
                    streaming_process.stdin.write(frame.tobytes())

        except Exception as err:
            print(f"\n[ERROR] \n")
            streaming_process.stdin.close()
            streaming_process.wait()
            self.video_src.release()

    def run(self):
        asyncio.run(self.run_rtmp()) 


# This is only for testing purpose.
# To run testing, 
# 1. Open one CLI terminal to run Simple Realtime Server (SRS)
#    docker run --rm -it -p 1935:1935 -p 1985:1985 -p 8080:8080 registry.cn-hangzhou.aliyuncs.com/ossrs/srs:6
# 2. Open another CLI termina to run rtmp_sender.py
#    python3 rtmp_sender.py
# 3. Open a chrome browser, visit http://127.0.0.1:8080/players/rtc_player.html
#    In the webpate, type in the URL: webrtc://127.0.0.1/live/livestream  
#    Notice, the URL doesn't contain the default rtmp port 1935.

class WebcamVideo:
    def __init__(self):
        self.video_src = cv2.VideoCapture(0)
        self.video_width = int(self.video_src.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.video_height = int(self.video_src.get(cv2.CAP_PROP_FRAME_HEIGHT))        

    def receive_video(self):
        ret, frame = self.video_src.read()
        if ret:
            return True, frame
        else:
            error_msg = f"cv2.VideoCapture(0) clapses when read(). Error message: {str(frame)}"
            return False, error_msg


def testrun_rtmp():
    rtmp_sender = RtmpSender()
    video_src = WebcamVideo()
    rtmp_sender.set_video_src(video_src)
    rtmp_sender.run()

if __name__ == "__main__":
    testrun_rtmp()