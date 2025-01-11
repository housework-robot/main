# Anatomy of Mushibot's Wifi WebSockets and HTTP/S connections

## 1. Objectives

One of the application scenarios of Mushibot is robot combat game, indoors or outdoors in the real world. 
Attract people to step out of their homes, head to the malls, or play outdoors.

Usually the human players are divided into two teams. Each human player remotely controls one or multiple robots. 
Each robot cooperates with the robots belonging to the same team, and fights against the other robots belongs to the other team. 

Let us focus on one robot, one human player scenario in this blog, and discuss the communication between the robot and the human player. 

The lifecycle of the communication between a Mushibot and a human player consists of the following steps, 

    1.1 A web server and a streaming server connect to the internet with public IP addresses. 

    1.2 A router running as an access point (AP) provides wifi network and connection to the internet.


    2.1 Assume a Mushibot knows the wifi network's ID and password, 
        it connects to the wifi in the station mode (STA mode), and gets its public IP address.

    2.2 The Mushibot starts up a WebSocket server, ready to communicate with WebSocket clients. 

    2.3 The Mushibot sends a http/s request to the web server, 
        provides its public IP address and registers itself as an available robot.


    3.1 A human player opens a Chrome browser in his computer, connects to the web server, 
        opens the web pages, and finds the available Mushibot.

    3.2 The human player's browser initializes a WebSocket connection with the Mushibot. 
        The human player's browser behaves as a WebSocket client.


    4.1 The Mushibot sends the video stream captured by its camera 
        continuously to the streaming server via RTMP protocol.

    4.2 The streaming server forwards the video stream to the human player's browser via WebRTC protocol. 

    4.3 The human player sends remote control commands continually from his browser to the Mushibot via WebSocket.

    4.4 The Mushibot sends its machine status, like its speed, its leg height, it steering yaw angle, etc, 
        periodically to the human player's browser via WebSocket.


    5.1 When the human player finishes the game, his browser disconnects the WebSocket connection to the Mushibot.

    5.2 When the Mushibot quits the game, it stops the connection to the streaming server,
        and then sends a http/s request to the web server, to deregister itself as a unavailable robot. 


For step 4.1 and 4.2, RTMP and WebRTC for video streaming, we have discussed this topic in our previous blog.
"[Video Streaming from Unitree Go2 to Web Browsers](https://github.com/housework-robot/main/blob/main/S05_communication/S05E02_go2_video_streaming.md)", 

Hence we skip the RTMP and WebRTC topic in this blog, and focus on Wifi, WebSocket and Http/s connections for Arduino-ESP32. 


