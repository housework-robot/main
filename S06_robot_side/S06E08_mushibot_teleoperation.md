# Anatomy of Mushibot's Teleoperation

## 1. Objectives

In this blog, we will discuss and implement teleoperation for Mushibot, a wheel-legged robot. The teleoperation consists of the following steps, 

     1. We construct a http website in a ubuntu computer, 
        using expressjs and embeddedjs templates. 
     

     2.1 The mushibot connects to the wifi network, and gets a IP address.

     2.2 The mushibot starts up a websocket server. 

     2.3 The mushibot connects to the website as a http client, 
         and posts its IP address to the website. 


     3.1 A human user uses a browser to open the webpage of the website. 
         When the browser opens the webpage, the webpage's javascript connects to the mushibot's as a websocket client.
         
         Notice that the websocket connection links the human user's browser to the remote mushibot directly, 
         rather than indirectly via the website, to prevent the website from being a traffic bottleneck. 
         
     3.2 The human user clicks the control buttons in the webpage,
         and the webpage's javascript sends the control commands, like "Forward", "Left", "Jump" to the mushibot.

     3.2 The websocket server running in the mushibot receives the commands, parses them, 
         and then control the motion of the mushibot. 


&nbsp;
## 2. Website with ExpressJS

### 2.1 ExpressJS web server

### 2.2 EmbeddedJS webpage

### 2.3 Send the mushibot IP to the user




     
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

