// controller.js
const mqtt = require('mqtt')
//const client = mqtt.connect('mqtt://36.92.136.155:31883') //1883
//const client = mqtt.connect('mqtt://test.mosquitto.org:1883') 
const client = mqtt.connect('mqtt://broker.emqx.io:1883')
const fs = require('fs');

var garageState = ''
var connected = false

client.on('connect', () => {
  client.subscribe('ANTARES/DataTest')
})

client.on('message', (topic, message) => {
  if(topic === 'ANTARES/DataTest') {
    connected = (message.toString() === 'true');
    console.log(message.toString())
    fs.appendFile('message.txt', message.toString(), function (err) {
      if (err) throw err;
      console.log('Saved!');
    });
  }
})