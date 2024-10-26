# Video streaming with Computer Webcam

# 1. Objectives

The objective of this article is a practice, not a final product. In the practice, we setup the communication channel from a computer's webcam to a website. 

More specifically, it consists of the following components, 

1. Running [ffmpeg](https://www.ffmpeg.org/) in a ubuntu CLI terminal, to push the video stream captured by the ubuntu computer's camera, to the server via RTSP protocol.

2. As an experiment, the server runs on the same ubuntu computer. On the server side, an [EasyDarwin engine](https://www.easydarwin.org/p/easydarwin.html) collects the video stream from multiple robots via RTSP.

3. Also on the server side, a [WebRTC_streamer engine](https://github.com/mpromonet/webrtc-streamer) pulls the video stream from the EasyDarwin engine.

4. The WebRTC_streamer pushes the video stream to a [VLC media player](https://www.videolan.org/vlc/), which runs on the same ubuntu computer, via WebRTC protocol. 

5. The WebRTC_streamer also pushes the video stream to website, developed with VUE,  via WebRTC protocol.

