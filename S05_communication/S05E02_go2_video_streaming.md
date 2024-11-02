# Video Streaming from Unitree Go2 to Web Browsers

&nbsp;
  by Kan Deng, Yuejie Wang, Nov 2, 2024

----------------

## 1. Objectives

The objectives of this article is to push video stream from unitree Go2 robotic dog to a video streaming server, then human viewers can view the video in a browser, in real-time. 

In more details, the dataflow consists of 3 phases. 

1. We develop a python script that pushes the video stream captured by unitree Go2 robotic dog's camera, via RTMP protocol, to a remote video streaming server.

2. We use [Simple Realtime Server (SRS)](https://ossrs.io/lts/en-us/) as the video streaming server, that collects multiple video streams from multiple robots,
   and sends those video streams to multiple browsers, via WebRTC protocol.

3. We also develop a website using VUE framework, so that human viewers can view the video streams in real-time via WebRTC.

The dataflow is illustrated in the following diagram. 

![The dataflow of RTMP to SRS to WebRTC](https://github.com/housework-robot/main/blob/main/S05_communication/S05E02_src/robot_video_stream_wireless.png)

For the convenience of development, we deployment the python script that sends RTMP video stream on a computer, which is connected to the unitree Go2 robotic dog with an ethernet cable, 
rather than deploying it to the Go2 dog.

Hence, in the development mode, the dataflow looks like the following diagram. 

![The dataflow of RTMP to SRS to WebRTC, in development mode](https://github.com/housework-robot/main/blob/main/S05_communication/S05E02_src/robot_video_stream_dev.png)

To implement this video streaming system, we need to accomplish the following tasks, 

1. Get the video stream, that is originally captured by the unitree Go2 robotic dog's camera, using [unitree's python APIs](https://github.com/unitreerobotics/unitree_sdk2_python).

2. Convert the raw video data into H.264, using [ffmpeg-python package](https://github.com/kkroening/ffmpeg-python).

3. Push out the H.264 video stream via RTMP protocol, using [ffmpeg-python package](https://github.com/kkroening/ffmpeg-python).

4. Install a [Simple Realtime Server (SRS)](https://ossrs.io/lts/en-us/).

5. Use VUE framework to construct a website, and use javascript to get video stream from SRS, via [WebRTC protocol](https://www.liveswitch.io/resources/ultimate-guide-to-webrtc).


&nbsp;
## 2. RTMP sender

On the robot side, we developed a system consisting of 3 tiers, up_tier, core_tier and down_tier. 

### 2.1 up_tier

In the up_tier, so far we implemented [`rtmp_sender.py`](https://github.com/housework-robot/main/blob/main/S05_communication/S05E02_src/robot_side/up_tier/rtmp_sender.py) and [`webrtc_send.py`](https://github.com/housework-robot/main/blob/main/S05_communication/S05E02_src/robot_side/up_tier/webrtc_sender.py), that push the video stream from robot to remote video streaming servers. 

For the time being, we use `rtmp_sender.py` to push the video stream. 

~~~
class RtmpSender:
    def __init__(self):
        self.ip_addr = "127.0.0.1"
        self.port = 1935
        self.rtmp_server_url = f"rtmp://{self.ip_addr}:{self.port}/live/livestream"
~~~

Notice, in `rtmp_server_url`, `/live` is mandatory, while `/livestream` is self-defined. For example, you can replace `/live/livestream` with `/live/go2_stream/456/789`. 


But we also implemented `webrtc_send.py` as well as [`webrtc_receiver.py`](https://github.com/housework-robot/main/blob/main/S05_communication/S05E02_src/robot_side/up_tier/webrtc_receiver.py) for testing purpose. 

The scripts in the up_tier are shared by various robots, to provide communication service.

In the future, we will add more shareable services to the up_tier.


### 2.2 down_tier

In the down_tier, so far we implemented [`go2_channel.py`](https://github.com/housework-robot/main/blob/main/S05_communication/S05E02_src/robot_side/down_tier/unitree_go2/go2_channel.py), that initializes the channel to communicate with unitree Go2 robotic dog.

With this communication channel, [`go2_media.py`](https://github.com/housework-robot/main/blob/main/S05_communication/S05E02_src/robot_side/down_tier/unitree_go2/go2_media.py) read video stream from Go2 dog, and decode the stream into [numpy array](https://numpy.org/devdocs/reference/generated/numpy.ndarray.html).

For the time being, `go2_media.py` only reads and decodes video stream. And in the future, we will add more functions to `go2_media.py`, to enable reading and decoding, encoding and writing audio stream.

In addition, we will add more scripts into the down_tier to provide motion control and other functionalities.

Besides unitree's Go2 robotic dog, we will provide services for other robots, and all those specific robot related scripts will be added into the down_tier.
    

### 2.3 core_tier

In the core_tier, so far we implemented [`go2_core.py`](https://github.com/housework-robot/main/blob/main/S05_communication/S05E02_src/robot_side/core_tier/go2_core.py), and in the future we will implement more scripts to serve other robots.

`go2_core.py` behaves as an workflow and dataflow organizer. It reads video stream from `down_tier/unitree_go2/go2_media.py` which is specific for go2 robotic dog, and sends the stream to `up_tier/rtmp_sender.py` which is shareable among various robots.

In addition to organize the video dataflow, `go2_core.py` will integrate other data and organize them into more complex workflow.


&nbsp;
## 3. Simple Realtime Server (SRS)

[Simple Realtime Server (SRS)](https://ossrs.net/lts/en-us/docs/v6/doc/introduction) is one of the top open source video streaming server in the world. 

> SRS supporting RTMP, WebRTC, HLS, HTTP-FLV, SRT, MPEG-DASH, and GB28181.
> 
> SRS media server works with clients like FFmpeg, OBS, VLC, and WebRTC to provide the ability to receive and distribute streams in a typical publish (push) and subscribe (play) server model.
>
> SRS supports widely used internet audio and video protocol conversions, such as converting RTMP or SRT to HLS, HTTP-FLV, or WebRTC.

> SRS is primarily used in the Live streaming and WebRTC fields.
>
> In the live streaming domain, SRS supports typical protocols such as RTMP, HLS, SRT, MPEG-DASH, and HTTP-FLV.
>
> In the WebRTC field, SRS supports protocols like WebRTC, WHIP, and WHEP. SRS facilitates protocol conversion for both Live streaming and WebRTC.

The most convenient way to run SRS is to use docker. 

~~~
$ docker run --rm -it -p 1935:1935 -p 1985:1985 -p 8080:8080 \ registry.cn-hangzhou.aliyuncs.com/ossrs/srs:6
~~~
