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

We construct an experimental http website using expressjs for the web server,
and embeddedjs templates for the webpages.

The webpage contains some buttons to remotely control the mulshibot to go forward or backforward, turn left or right, jump etc. 

The expressjs web server hosts the webpages. 
When a human user uses his browser to visit the website, his browser downloads the webpages and displays them in his browser.

When the user's browser opens the webpages, the browser executes the javascript code in the webpage. 
One task of the javascript is to set up the websocket connection from the browser to the remote mushibot, 

The javascript running in the user's browser is the websocket client,
and the mushibot is the websocket server.


&nbsp;
### 2.1 ExpressJS web server

We used [express-js](https://expressjs.com/en/starter/hello-world.html) 
and [embedded-js templates](https://github.com/mde/ejs) 
to construct an experimental website. 

Here is a fragment of the source code of 
[the express-js web server](/S06E08_src/src/Mushibot20250117/test/teleop_website/app.js). 

~~~
const express = require("express");
const app = express();
app.set('view engine', 'ejs');
// app.use(express.static('public'));

const bodyParser = require("body-parser");

// for parsing application/json
app.use(bodyParser.json());

// Randomly assign an original value
var mushibot_ip = "mushibot.192.168.0.123";

app.get("/", (req, res) => {
    res.render(
        'index', {robot_ip: mushibot_ip}
    );
});

app.post("/post_json", (req, res) => {
    mushibot_ip = (req.body)["robot_ip"];
    console.log(`Robot local IP address: "${mushibot_ip}"`);
    res.send(`Robot local IP address: "${mushibot_ip}"`);
});

const port = 3000;
app.listen(port, () => {
    console.log(`Server is running at http://localhost:${port}`);
});
~~~

1. `app.set('view engine', 'ejs')`

   We used embedded-js templates to create the webpages, rather than the plain HTML file.

2. `app.use(bodyParser.json())`

   We need bodyParser to parse json messages.

3. `app.get("/", (req, res) => { res.render('index', {robot_ip: mushibot_ip}); });`

   The express-js web server will use `index.ejs` made by embedded-js, when the user's browser sends request to GET `/`. 

   The express-js will pass a parameter object to the `index.ejs` webpage,
   the parameter object contains a parameter called `robot_ip`. 

4. `app.post("/post_json", (req, res) => { mushibot_ip = (req.body)["robot_ip"];}`

   The express-js web server is ready to receive http post requests with json payload.

   The json payload contains a variable called `robot_ip`.

   We use a variable `mushibot_ip` to store the value of `robot_ip`.
    

&nbsp;
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

