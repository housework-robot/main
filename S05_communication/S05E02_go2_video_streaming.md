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

