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

                console.log(parsed);
                //try {
                //console.log(`Readings from ${parsed.meta}: ${JSON.stringify(parsed)}`);      
                /*} catch(err) {
                    console.log("Error Parsing")
                }*/


                /*
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
                */

                //use complementary filtering method to get accurate orientation angles
                //calculate accelAngle 
                /*
                let accelAngles = getAccelAngles(parsed["Ax"], parsed["Ay"], parsed["Az"]).map((a) => a * 180/Math.PI);

                //grab gyroangles
                let gyroAngles = [parsed["Gx"], parsed["Gy"], parsed["Gz"]].map((a) => a * 180/Math.PI);


                let filteredAngles = accelAngles.map((accelAngle, i) => {
                    return getFilteredAngle(gyroAngles[i], accelAngle, 1, 0.1);
                });

                //console.log(filteredAngles);
                */
                //broadcast readings to all sockets
                wsServer.connections.forEach((connection) => {
                    //connection.send(JSON.stringify(Object.assign({filteredAngles: filteredAngles}, parsed)));
                    connection.send(JSON.stringify(parsed));
                });
            }
        }
        else if (message.type === 'binary') {
            console.log(message.binaryData.length);
        }
    });
});

function getAccelAngles(ax, ay, az) {
    //inclination angle relative to x-axis
    let alpha = Math.atan(ax / Math.sqrt(ay * ay + az * az));

    let beta = Math.atan(ay / Math.sqrt(ax * ax + az * az));

    let gamma = Math.atan(Math.sqrt(ax * ax + ay * ay) / az);

    return [alpha, beta, gamma];
}

function getFilteredAngle(gyroAngle, accelAngle, timeConst, deltaTime) {
    let a = timeConst / (timeConst + deltaTime);
    let filteredAngle = a * gyroAngle + (1 - a) * accelAngle;

    return filteredAngle;
}