//import serialport module
var serialPort = require("serialport");
var Readline = serialPort.parsers.Readline;

//import express 
var express = require('express');
//create express object named app
var app = express();

//instantiate a server on port 3000
var server = app.listen(3000);
var io = require('socket.io')(server);

//set parameters for the serialport
var serialPort = new serialPort("/dev/cu.usbmodem14101", {
    baudRate: 115200
  });


//the readline parser will delimit the data on a newline
var parser = new Readline();
serialPort.pipe(parser);

//expose the local public folder for inluding files js, css etc..
app.use(express.static('public'));

//on a request to / serve index.html
app.get('/', function(req, res) {
    res.sendFile(__dirname + '/index.html');
});

app.get('/start', function(req, res) {
  res.sendFile(__dirname + '/1.html');
});

//send a message when the serialport connects successfully
serialPort.on("open", function() {
    console.log("Communication is on!");
  });
  
//when data is recieved log it to the console
parser.on("data", function(data) {
    console.log("data received: " + data);
    io.sockets.emit('mysocket', data);
});
