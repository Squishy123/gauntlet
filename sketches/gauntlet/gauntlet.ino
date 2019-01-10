/*
  Gauntlet Board Driver
  By: Christian Wang
  Release Date: 2019/01/09

  Description: Main driver for ESP1866 boards using the MPU6050 board.
  Grabs sensor readings and sends the data across a customizable websocket server.
*/
#include <ArduinoJson.h>
#include <ESP8266WiFi.h>
#include <WebSockets.h>
#include <WebSocketsClient.h>
#include <WebSocketsServer.h>
#include <Wire.h>

#define USE_SERIAL Serial1

//MPU6050 slave device address
const uint8_t MPU6050_SLAVE_ADDRESS = 0x68;

//SCL_PIN and SDA_PIN pins for I2C
const uint8_t SCL_PIN = D6;
const uint8_t SDA_PIN = D7;

//MPU6050 configuration register addresses
const uint8_t MPU6050_REGISTER_SMPLRT_DIV = 0x19;
const uint8_t MPU6050_REGISTER_USER_CTRL = 0x6A;
const uint8_t MPU6050_REGISTER_PWR_MGMT_1 = 0x6B;
const uint8_t MPU6050_REGISTER_PWR_MGMT_2 = 0x6C;
const uint8_t MPU6050_REGISTER_CONFIG = 0x1A;
const uint8_t MPU6050_REGISTER_GYRO_CONFIG = 0x1B;
const uint8_t MPU6050_REGISTER_ACCEL_CONFIG = 0x1C;
const uint8_t MPU6050_REGISTER_FIFO_EN = 0x23;
const uint8_t MPU6050_REGISTER_INT_ENABLE = 0x38;
const uint8_t MPU6050_REGISTER_ACCEL_XOUT_H = 0x3B;
const uint8_t MPU6050_REGISTER_SIGNAL_PATH_RESET = 0x68;

// sensitivity scale factor respective to full scale setting provided in datasheet
const uint16_t ACCEL_SCALE_FACTOR = 16384;
const uint16_t GYRO_SCALE_FACTOR = 131;

//Keep track of polling rate
unsigned long lastTime, deltaTime;

//DOF measured_values
int16_t maccelX, maccelY, maccelZ, mtemp, mgyroX, mgyroY, mgyroZ;

//DOF measured_values
double accelX, accelY, accelZ, temp, gyroX, gyroY, gyroZ;

//Calculated Angles
double accelAngleX, accelAngleY, accelAngleZ, gyroAngleX, gyroAngleY, gyroAngleZ;

//Filtered Angles
double filtAngleX, filtAngleY, filtAngleZ;

//network config
const String NETWORK_SSID = "BELL117";
const String NETWORK_PASSWORD = "7AD92993691F";

//meta tag
const String meta = "tester2";

//rate
const int BAUD_RATE = 115200;

//socket config
const String WEBSOCKET_SERVER_HOST = "192.168.2.116";
const int WEBSOCKET_PORT = 25565;

//create TCP connections
WiFiClient client;

//websocket client
WebSocketsClient webSocketClient;

bool isConnected = false;

void setup()
{
  //start serial port
  Serial.begin(BAUD_RATE);

  //start I2C connection
  Wire.begin(SDA_PIN, SCL_PIN);

  //Configure MPU6050
  MPU6050Init();

  //Configure websocket connection
  socketClientInit();
}

void loop()
{
  webSocketClient.loop();

  //get readings
  getMPU6050Readings();
  
  //calculate angles
  calculateAngles();

  //apply filter
  applyFilter();

  //package data
  JsonObject& packagedJSON = packageValues();

  //display output on serial port
  //packagedJSON.printTo(Serial);
  //Serial.print("\n");

  //if the server is connected
  if (isConnected) {

    //write to string form
    String jsonStr;
    packagedJSON.printTo(jsonStr);

    //send it via websockets
    webSocketClient.sendTXT(jsonStr);
  }
}

//get MPU6050 readings
void getMPU6050Readings()
{ 
  Read_RawValue(MPU6050_SLAVE_ADDRESS, MPU6050_REGISTER_ACCEL_XOUT_H);

  //divide each by their sensitivity scale factors
  accelX = (double)maccelX / ACCEL_SCALE_FACTOR;
  accelY = (double)maccelY / ACCEL_SCALE_FACTOR;
  accelZ = (double)maccelZ / ACCEL_SCALE_FACTOR;
  gyroX = (double)mgyroX / GYRO_SCALE_FACTOR;
  gyroY = (double)mgyroY / GYRO_SCALE_FACTOR;
  gyroZ = (double)mgyroZ / GYRO_SCALE_FACTOR;

  //apply temperature formula
  temp = (double)mtemp / 340 + 36.53;

  //delay to prevent overload
  delay(100);

}

//calculate accelerometer angles and drifting gyro angles
void calculateAngles()
{
  //calculate accelerometer angles
  accelAngleX = atan(accelX / sqrt(accelY * accelY + accelZ * accelZ));
  accelAngleY = atan(accelY / sqrt(accelX * accelX + accelZ * accelZ));
  accelAngleZ = atan(sqrt(accelX * accelX + accelY * accelY) / accelZ);

  //set gyro angles
  gyroAngleX = gyroX;
  gyroAngleY = gyroY;
  gyroAngleZ = gyroZ;
}

//apply a complimentary filter to get a more precise orientation
void applyFilter()
{
  const double filtConst1 = 0.98, filtConst2 = 0.02;
  filtAngleX = filtConst1 * (filtAngleX + gyroAngleX * deltaTime) + filtConst2 * accelAngleX;
  filtAngleY = filtConst1 * (filtAngleY + gyroAngleY * deltaTime) + filtConst2 * accelAngleY;
  filtAngleZ = filtConst1 * (filtAngleZ + gyroAngleZ * deltaTime) + filtConst2 * accelAngleZ;
}

//helper function to turn doubles into strings
String doubleToString(double flt) {
  //char doubleStr[20];

  //min width=4, precision=5
  //return dtostrf(flt, 4, 5, doubleStr);

  return String(flt);
}

//package all the values into JSON and return it
JsonObject& packageValues() {
  DynamicJsonBuffer jsonBuffer(300);
  JsonObject& package = jsonBuffer.createObject();


  //measured acceleration in format (ax, ay, az)
  package["measuredAccel"] = jsonBuffer.parseArray("[" + doubleToString(accelX) + "," + doubleToString(accelY) + "," + doubleToString(accelZ) + "]");

  //measured gyro orientation in format (gx, gy, gz)
  package["measuredGyro"] = jsonBuffer.parseArray("[" + doubleToString(gyroX) + "," + doubleToString(gyroY) + "," + doubleToString(gyroZ) + "]");

  //calculated acceleration angles in format(angle_x, angle_y, angle_z)
  package["calculatedAccelAngles"] = jsonBuffer.parseArray("[" + doubleToString(accelAngleX) + "," + doubleToString(accelAngleY) + "," + doubleToString(accelAngleZ) + "]");

  //calculated gyro orientation in format (gx, gy, gz)
  package["calculatedGyroAngles"] = jsonBuffer.parseArray("[" + doubleToString(gyroAngleX) + "," + doubleToString(gyroAngleY) + "," + doubleToString(gyroAngleZ) + "]");

  //filtered angles in format (fx, fy, fz)
  package["filteredAngles"] = jsonBuffer.parseArray("[" + doubleToString(filtAngleX) + "," + doubleToString(filtAngleY) + "," + doubleToString(filtAngleZ) + "]");
  
  //measured temperature
  package["measuredTemp"] = temp;

  //measured deltaTime
  package["deltaTime"] = deltaTime;

  //metatag
  package["meta"] = meta;

  return package;
}

//configure socket client and network
void socketClientInit()
{
  //connect to wifi network
  Serial.println("Connecting to " + NETWORK_SSID);

  WiFi.begin(NETWORK_SSID.c_str(), NETWORK_PASSWORD.c_str());
  Serial.println("Attempting.");

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }

  Serial.println("WiFi successfully connected!");
  Serial.println("Device IP_ADDRESS: " + WiFi.localIP().toString());

  delay(500);

  //connect to websocket server
  if (client.connect(WEBSOCKET_SERVER_HOST, WEBSOCKET_PORT))
  {
    Serial.println("Connected to Gauntlet Server!");
  }
  else
  {
    Serial.println("Connection failure! Halting!");
    //hangs on failure
    while (1)
    {
    }
  }

  webSocketClient.begin(WEBSOCKET_SERVER_HOST, WEBSOCKET_PORT, "/");
  webSocketClient.onEvent(webSocketEvent);
  //reconnect if failed
  //webSocketClient.setReconnectInterval(5000);
}

//socket event handling and looping
void webSocketEvent(WStype_t type, uint8_t *payload, size_t length)
{
  switch (type)
  {
    case WStype_DISCONNECTED:
      USE_SERIAL.printf("[WSc] Disconnected!\n");
      isConnected = false;
      break;
    case WStype_CONNECTED:
      {
        USE_SERIAL.printf("[WSc] Connected to url: %s\n", payload);

        isConnected = true;
        // send message to server when Connected
        webSocketClient.sendTXT("Connected");
      }
      break;
    case WStype_TEXT:
      USE_SERIAL.printf("[WSc] get text: %s\n", payload);

      // send message to server
      // webSocket.sendTXT("message here");
      break;
    case WStype_BIN:
      USE_SERIAL.printf("[WSc] get binary length: %u\n", length);
      hexdump(payload, length);

      // send data to server
      // webSocket.sendBIN(payload, length);
      break;
  }
}

//configure MPU6050
void MPU6050Init()
{
  delay(150);
  I2C_Write(MPU6050_SLAVE_ADDRESS, MPU6050_REGISTER_SMPLRT_DIV, 0x07);
  I2C_Write(MPU6050_SLAVE_ADDRESS, MPU6050_REGISTER_PWR_MGMT_1, 0x01);
  I2C_Write(MPU6050_SLAVE_ADDRESS, MPU6050_REGISTER_PWR_MGMT_2, 0x00);
  I2C_Write(MPU6050_SLAVE_ADDRESS, MPU6050_REGISTER_CONFIG, 0x00);
  I2C_Write(MPU6050_SLAVE_ADDRESS, MPU6050_REGISTER_GYRO_CONFIG, 0x00);  //set +/-250 degree/second full scale
  I2C_Write(MPU6050_SLAVE_ADDRESS, MPU6050_REGISTER_ACCEL_CONFIG, 0x00); // set +/- 2g full scale
  I2C_Write(MPU6050_SLAVE_ADDRESS, MPU6050_REGISTER_FIFO_EN, 0x00);
  I2C_Write(MPU6050_SLAVE_ADDRESS, MPU6050_REGISTER_INT_ENABLE, 0x01);
  I2C_Write(MPU6050_SLAVE_ADDRESS, MPU6050_REGISTER_SIGNAL_PATH_RESET, 0x00);
  I2C_Write(MPU6050_SLAVE_ADDRESS, MPU6050_REGISTER_USER_CTRL, 0x00);
}

//write to I2C bus
void I2C_Write(uint8_t deviceAddress, uint8_t regAddress, uint8_t data)
{
  Wire.beginTransmission(deviceAddress);
  Wire.write(regAddress);
  Wire.write(data);
  Wire.endTransmission();
}

//read all 14 register
void Read_RawValue(uint8_t deviceAddress, uint8_t regAddress)
{
  Wire.beginTransmission(deviceAddress);
  Wire.write(regAddress);
  Wire.endTransmission();
  Wire.requestFrom(deviceAddress, (uint8_t)14);
  maccelX = (((int16_t)Wire.read() << 8) | Wire.read());
  maccelY = (((int16_t)Wire.read() << 8) | Wire.read());
  maccelZ = (((int16_t)Wire.read() << 8) | Wire.read());
  mtemp = (((int16_t)Wire.read() << 8) | Wire.read());
  mgyroX = (((int16_t)Wire.read() << 8) | Wire.read());
  mgyroY = (((int16_t)Wire.read() << 8) | Wire.read());
  mgyroZ = (((int16_t)Wire.read() << 8) | Wire.read());

  //record time since last reading
  unsigned long currentTime = millis();
  deltaTime = currentTime - lastTime;
  lastTime = currentTime;
}
