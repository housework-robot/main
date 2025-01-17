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
[the express-js web server](./S06E08_src/src/Mushibot20250117/test/teleop_website/app.js). 

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

As mentioned in the previous section, the express-js web server passes a parameter object to the embedded-js webpage, 
one parameter is `robot_ip`, 
and the embedded-js webpage can use it by `<%= robot_ip %>`. 

Here is a fragment of [the embedded-js webpage](./S06E08_src/src/Mushibot20250117/test/teleop_website/views/index.ejs#L310). 

~~~
<script>
   window.INITIAL_STATE = {ROBOT_IP: "<%= robot_ip %>"};

   // socket_init is automatically executed when the webpage is loaded.
   function socket_init() {
       console.log("Connecting to mushibot, IP = " + window.INITIAL_STATE.ROBOT_IP);

       // Initialize websocket client
       socket = new WebSocket('ws://' + window.INITIAL_STATE.ROBOT_IP + ':81/'); 
   }

...
</script> 
~~~

1. `window.INITIAL_STATE`
  
   We assigned a variable `INITIAL_STATE` to the global variable `window`,
   to make it convenient for the javascript to use the variable.

2. `window.INITIAL_STATE = {ROBOT_IP: ...}`
  
   We asked `INITIAL_STATE` to contain a object, and in the object there is a key-value pair called `ROBOT_IP`.

3. `window.INITIAL_STATE = {ROBOT_IP: "<%= robot_ip %>"}`
  
   As mentioned above, the express-js web server passes a parameter `robot_ip` to the embedded-js webpage.

   The javascript can use the `robot_ip` parameter by `<%= robot_ip %>`.

   By the way, [the title of the webpage](./S06E08_src/src/Mushibot20250117/test/teleop_website/views/index.ejs#L265)
   also use `robot_ip` parameter.

   ~~~
   <h3> 已经与机器人 ( IP = <%= robot_ip %> ) 连线了 </h3>
   ~~~

4. `new WebSocket('ws://' + window.INITIAL_STATE.ROBOT_IP + ':81/')`

   Javascript used `window.INITIAL_STATE.ROBOT_IP` to initialize a websocket client.

   `window.INITIAL_STATE.ROBOT_IP`'s value is from `<%= robot_ip %>`.

   `<%= robot_ip %>` parameter is passed to the embedded-js webpage by the express-js web server.

   The value of the `<%= robot_ip %>` parameter is received by the the express-js web server from the mushibot.

   The mushibot sends the `robot_ip` by http POST to the express-js web server.

     
&nbsp;
## 3. Http client in Aruduino C++

In [the previous blog](./S06E07_mushibot_wifi_ws_http.md#4-https-client), 
we explained how we implemented [https client](./S06E08_src/src/Mushibot20250117/src/wswifi.cpp#L299) for GET. 

This time, we implemented https client for both GET and POST.

The following source code fragment implements http client for POST, with payload in json. 

~~~
String WsWifi::http_post(String http_url, JsonDocument payload_json) {
    HTTPClient http_client;
    int http_res_code;
    String http_res_str;

    if (WiFi.status() == WL_CONNECTED) {
        http_client.begin(http_url + "/post_json");
        http_client.addHeader("Content-Type", "application/json");

        String payload_str; 
        payload_json["datatype"] = "json";
        serializeJson(payload_json, payload_str);

        http_res_code = http_client.POST(payload_str);

        if (http_res_code == HTTP_CODE_OK) {            
            http_res_str = http_client.getString();
        }
        else {
            Serial.printf("\n[WARN] Cannot access '%s' for http POST, http code is: '%d'.\n", 
                http_url.c_str(), http_res_code);
        }

        http_client.end();
    }

    return http_res_str;
}
~~~

1. `if (WiFi.status() == WL_CONNECTED)`

   Here we assumed that the mushibot has already connected to the wifi.

   [`String WsWifi::connect_wifi(String ssid, String password)`](./S06E08_src/src/Mushibot20250117/src/wswifi.cpp#L132)
    handles the connection to the wifi. 

2. `http_client.begin(http_url + "/post_json")`

   The http client connects to our teleoperation website whose URL is `http_url`.

   Referring to the express-js web server's source code,
   [the route "/post_json"](./S06E08_src/src/Mushibot20250117/test/teleop_website/app.js#L55)
   is dedicated for http POST requests.

3. `http_client.addHeader("Content-Type", "application/json")`

   We must specify in the http request header, that the `Content-Type` is `application/json`.

   In this way, we trigger the express-js web server to ask
   [the "bodyParser" to process](./S06E08_src/src/Mushibot20250117/test/teleop_website/app.js#L13)
   the http POST request's body as a json payload.

4. `serializeJson(payload_json, payload_str)`

   Serialize the json payload `payload_json`, into a String `payload_str`.

5. `http_client.POST(payload_str)`

   Our `http_client` is an instance of
   [`arduino-esp32 HTTPClient`](https://github.com/espressif/arduino-esp32/blob/master/libraries/HTTPClient/src/HTTPClient.h).

   Referring to its source code
   of ["sendRequest"](https://github.com/espressif/arduino-esp32/blob/master/libraries/HTTPClient/src/HTTPClient.cpp#L562),
   both GET and POST, the request payload must be a string.

6. `String http_res_str = http_client.getString()`

   When [our express-js web server](./S06E08_src/src/Mushibot20250117/test/teleop_website/app.js#L58)
   receives and handles the http POST request from the mushibot, it sends back response in string. 



&nbsp;
## 4. WebSocket client in JavaScript


&nbsp;
## 5. Parsing payload with ArduinoJson


&nbsp;
## 6. Future work

&nbsp;
## 7. Demo 

