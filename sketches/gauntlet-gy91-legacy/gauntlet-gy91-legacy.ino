#include <ArduinoJson.h>

#include "MPU9250.h"

#include <ESP8266WiFi.h>
#include <WebSockets.h>
#include <WebSocketsClient.h>
#include <WebSocketsServer.h>

//network config
String WIFI_SSID = "";
String WIFI_PASSWORD = "";

//unique client id
String CLIENT_ID = "";

//websocket config
String WS_HOST = "";
int WS_PORT = 25565;

//client for creating TCP connections
WiFiClient client;

//websocket client
WebSocketsClient wsClient;

//number of tries for wifi connection before quitting
int WIFI_CONNECTION_ATTEMPTS = 50;

//number of tries for ws connection before quitting
int WS_CONNECTION_ATTEMPTS = 50;

//tracking connectivity
bool isWifiConnected = false;
bool isWSConnected = false;

#define USE_SERIAL Serial1

MPU9250 mpu = MPU9250();

void setup(void)
{
  Serial.begin(115200);

  //set network properties
  WIFI_SSID = "public wifi";
  WIFI_PASSWORD = "91122919";

  CLIENT_ID = "GY-91 Module 1";
  WS_HOST = "192.168.2.17";

  //connect to wifi network
  if (!wifiClientInit())
  {
    //hang on failure
    while (1) {}
  }

  if (!wsClientInit()) {
    //hang on failure
    while (1) {}
  }

  //start getting readings from mpu
  uint8_t temp = mpu.begin();
}

void loop()
{
  wsClient.loop();
  
  //Accel
  mpu.set_accel_range(RANGE_4G);
  mpu.get_accel();
  Serial.print("X: ");
  Serial.print(mpu.x);
  Serial.print(" Y: ");
  Serial.print(mpu.y);
  Serial.print(" Z: ");
  Serial.print(mpu.z);

  mpu.get_accel_g();
  Serial.print(" X_g: ");
  Serial.print(mpu.x_g, 2);
  Serial.print(" Y_g: ");
  Serial.print(mpu.y_g, 2);
  Serial.print(" Z_g: ");
  Serial.print(mpu.z_g, 2);
  Serial.println(" G");

  //Gyro
  mpu.set_gyro_range(RANGE_GYRO_250);
  mpu.get_gyro();
  Serial.print("GX: ");
  Serial.print(mpu.gx);
  Serial.print(" GY: ");
  Serial.print(mpu.gy);
  Serial.print(" GZ: ");
  Serial.print(mpu.gz);

  mpu.get_gyro_d();
  Serial.print(" GX_g: ");
  Serial.print(mpu.gx_d, 2);
  Serial.print(" GY_g: ");
  Serial.print(mpu.gy_d, 2);
  Serial.print(" GZ_g: ");
  Serial.print(mpu.gz_d, 2);
  Serial.println(" º/s");

  //Mag
  mpu.set_mag_scale(SCALE_14_BITS);
  mpu.set_mag_speed(MAG_8_Hz);
  if (!mpu.get_mag())
  {
    Serial.print("MX: ");
    Serial.print(mpu.mx);
    Serial.print(" MY: ");
    Serial.print(mpu.my);
    Serial.print(" MZ: ");
    Serial.print(mpu.mz);

    mpu.get_mag_t();
    Serial.print(" MX_t: ");
    Serial.print(mpu.mx_t, 2);
    Serial.print(" MY_t: ");
    Serial.print(mpu.my_t, 2);
    Serial.print(" MZ_t: ");
    Serial.print(mpu.mz_t, 2);
    Serial.println(" uT");
  }
  else
  {
    // |X|+|Y|+|Z| must be < 4912μT to sensor measure correctly
    Serial.println("Overflow no magnetometro.");
  }

  // Temp
  Serial.print("Temperature is ");
  Serial.print((((float)mpu.get_temp()) / 333.87 + 21.0), 1);
  Serial.println(" degrees C");
  
  //package data
  JsonObject& packagedJSON = packageValues();

  //if (isWSConnected) {

  //write to string form
  String jsonStr;
  packagedJSON.printTo(jsonStr);

  //send it via websockets
  wsClient.sendTXT(jsonStr);
  //}

  //delay to prevent overflow
  delay(100);
}


//helper function to turn doubles into strings
String doubleToString(double flt) {
  //char doubleStr[20];

  //min width=4, precision=5
  //return dtostrf(flt, 4, 5, doubleStr);

  return String(flt, 10);
}

JsonObject& packageValues() {
  DynamicJsonBuffer jsonBuffer(300);
  JsonObject& package = jsonBuffer.createObject();


  //measured acceleration in format (ax, ay, az)
  package["measuredAccel"] = jsonBuffer.parseArray("[" + doubleToString(mpu.x) + "," + doubleToString(mpu.y) + "," + doubleToString(mpu.z) + "]");

  //measured gyro orientation in format (gx, gy, gz)
  package["measuredGyro"] = jsonBuffer.parseArray("[" + doubleToString(mpu.gx) + "," + doubleToString(mpu.gy) + "," + doubleToString(mpu.gz) + "]");

  //calculated acceleration angles in format(angle_x, angle_y, angle_z)
  //package["calculatedAccelAngles"] = jsonBuffer.parseArray("[" + doubleToString(accelAngleX) + "," + doubleToString(accelAngleY) + "," + doubleToString(accelAngleZ) + "]");

  //calculated gyro orientation in format (gx, gy, gz)
  //package["calculatedGyroAngles"] = jsonBuffer.parseArray("[" + doubleToString(gyroAngleX) + "," + doubleToString(gyroAngleY) + "," + doubleToString(gyroAngleZ) + "]");

  //filtered angles in format (fx, fy, fz)
  //package["filteredAngles"] = jsonBuffer.parseArray("[" + doubleToString(filtAngleX) + "," + doubleToString(filtAngleY) + "," + doubleToString(filtAngleZ) + "]");

  //measured temperature
  package["measuredTemp"] = ((float)mpu.get_temp()) / 333.87 + 21.0;

  //measured deltaTime
  //package["deltaTime"] = deltaTime;

  //metatag
  package["meta"] = CLIENT_ID;

  return package;
}

/**
   Attempt to connect to wifi network
   Tries connection WIFI_CONNECTION_ATTEMPTS times
   with a 500ms delay each time
   Return true if successful, false if unsuccessful
*/
boolean wifiClientInit()
{
  //verbose stuff
  Serial.println("Connecting to WiFi network" + WIFI_SSID);
  Serial.println("Attempting...");

  WiFi.begin(WIFI_SSID.c_str(), WIFI_PASSWORD.c_str());
  for (int i = 0; i < WIFI_CONNECTION_ATTEMPTS; i++)
  {
    //check if connected
    if (WiFi.status() == WL_CONNECTED)
    {
      isWifiConnected = true;

      Serial.println("Successfully connected to " + WIFI_SSID + "!");
      Serial.println("Device IP_ADDRESS: " + WiFi.localIP().toString());
      return true;
    }
    //if not delay and try again
    delay(500);
    Serial.println("Attempt #" + i);
  }

  Serial.println("Connection unsuccessful! Halting!");
  return false;
}


void webSocketEvent(WStype_t type, uint8_t *payload, size_t length)
{
  switch (type)
  {
    case WStype_DISCONNECTED:
      USE_SERIAL.printf("[WSc] Disconnected!\n");
      isWSConnected = false;
      break;
    case WStype_CONNECTED:
      {
        USE_SERIAL.printf("[WSc] Connected to url: %s\n", payload);

        isWSConnected = true;
        // send message to server when Connected
        wsClient.sendTXT("Connected");
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

/**
  Attempt to connect to WS server
   Tries connection WS_CONNECTION_ATTEMPTS times
   with a 500ms delay each time
   Return true if successful, false if unsuccessful
*/
boolean wsClientInit()
{
  //verbose stuff
  Serial.println("Connecting to WS server" + WS_HOST);
  Serial.println("Attempting...");

  for (int i = 0; i < WS_CONNECTION_ATTEMPTS; i++)
  {
    //check if connected
    if (client.connect(WS_HOST, WS_PORT))
    {
      isWSConnected = true;
      wsClient.begin(WS_HOST, WS_PORT, "/");
      wsClient.onEvent(webSocketEvent);

      Serial.println("Successfully connected to Gauntlet Server!");
      return true;
    }
    //if not delay and try again
    delay(500);
    Serial.println("Attempt #" + i);
  }

  Serial.println("Connection unsuccessful! Halting!");
  return false;
}
