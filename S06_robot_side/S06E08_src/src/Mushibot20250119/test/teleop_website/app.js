const express = require("express");
const app = express();
app.set('view engine', 'ejs');
// app.use(express.static('public'));

const bodyParser = require("body-parser");
const multer = require("multer");

// for parsing multipart/form-data
const upload = multer();

// for parsing application/json
app.use(bodyParser.json());

// for parsing application/x-www-form-urlencoded
app.use(bodyParser.urlencoded({extended: false}));

// const cors = require('cors');
// app.use(cors({origin: true, credentials: true}));
// app.use((req, res, next) => {
//     res.header('Access-Control-Allow-Origin', '*');
//    next();
// });


var content = "";
var mushibot_ip = "mushibot.192.168.0.123";

app.get("/", (req, res) => {
    res.render(
        'index', {robot_ip: mushibot_ip}
    );
});


app.post("/submit", (req, res) => {
    content = req.body.input;
    console.log(`Input submitted: "${content}"`);
    res.send(`Input submitted: "${content}"`);
});


app.post("/post_text", (req, res) => {
    if ((req.body)["datatype"] == "text") {
        content = (req.body)["content"];
        console.log(`Input submitted: "${content}"`);
        res.send(`Input submitted: "${(req.body)["content"]}"`);
    }
    else {
        res.send(`[ERROR] posted datatype: "${content}"`);
    }

});

app.post("/post_json", (req, res) => {
    mushibot_ip = (req.body)["robot_ip"];
    console.log(`Robot local IP address: "${mushibot_ip}"`);
    res.send(`Robot local IP address: "${mushibot_ip}"`);
    res.redirect('/');
});


app.get("/hello/:name", (req, res) => {
    content = req.params.name
    console.log(`Hello ${content}!`);
    res.send(`Hello ${content}!`);
}); 

app.get("/hello", (req, res) => {
    content = req.query.name
    console.log(`Hello ${content}!`);
    res.send(`Hello ${content}!`);
}); 


const port = 3000;
app.listen(port, () => {
    console.log(`Server is running at http://localhost:${port}`);
});
