# Anatomy of Mushibot's Teleoperation

## 1. Objectives

In this blog, we will discuss and implement teleoperation for Mushibot, a wheel-legged robot. The teleoperation consists of the following steps, 

     1. We construct a http website in a ubuntu computer, using expressjs. 


     2.1 When the mushibot connects to the same wifi network, and gets a IP address.

     2.2 The mushibot starts up a websocket server. 
     

     3.1 The mushibot connects to the website as a http client, 
         and post its IP address to the website. 

     3.2 The website connects to the mushibot as a websocket client. 


     4.1 A human user uses a browser to login to the website, 
         and sends control commands, like "walk to the left", "run forwards", "jump upward". 

     4.2 Once the website receives the remote commands from the human user's browser, 
         it forwards the command to the mushibot via websocket client. 

     4.3 The websocket server running in the mushibot receives the commands, parses them, 
         and then control the motion of the mushibot. 


&nbsp;
## 2. Website with ExpressJS

     
&nbsp;
## 3. Http client in Aruduino C++


&nbsp;
## 4. WebSocket client in JavaScript


&nbsp;
## 5. Parsing payload with ArduinoJson


&nbsp;
## 6. Future work

&nbsp;
## 7. Demo 

