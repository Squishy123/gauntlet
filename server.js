const WebSocketServer = require('websocket').server;
const http = require('http');

//http server stuff 
let server = http.createServer((req, res) => {
    //todo to confirm server status
});

server.listen(25565, () => { console.log("Server running!") });

//create websocket server
wsServer = new WebSocketServer({
    httpServer: server
});

wsServer.on('request', (req) => {
    console.log('Client Connected!')

    let connection = req.accept('', req.origin);
    //data send
    connection.on('message', (message) => {
        if (message.type === 'utf8') {
            let parsed = JSON.parse(JSON.stringify(message)).utf8Data;
            if (parsed != "Connected") {
                parsed = JSON.parse(parsed);

                //broadcast readings to all sockets
                wsServer.connections.forEach((connection) => {
                    connection.send(JSON.stringify(parsed));
                });

                //console.log(parsed);
                console.log(`Readings from ${parsed.meta}: ${JSON.stringify(parsed)}`);       
                

                //check if stationary
                //calculate magnitude of acceleration
                accel_mag = Math.sqrt(parsed["Ax"] * parsed["Ax"], parsed["Ay"] * parsed["Ay"], parsed["Az"] * parsed["Az"]);

                //use a butterworth filter(high pass) to remove noise
                //cutoff is the range the accelerometer moves during rest
                let cutoff = 0.5;

                //samplePeriod is the time between each reading
                let samplePeriod = 0.1;

                let highPass = (2 * cutoff) / (1 / samplePeriod);

                //console.log(highPass * accel_mag);

            }
        }
        else if (message.type === 'binary') {
            console.log(message.binaryData.length);
        }
    });
});