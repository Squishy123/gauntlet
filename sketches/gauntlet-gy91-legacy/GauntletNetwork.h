/*
    Gauntlet Network Manager (header)
    By: Christian Wang
    Release Date: 2019/01/28

    Description: Provides functions to setup and connect to WiFi networks
    using the ESP8266 board. Also helps connect as a client of a websocket server.

    CONTAINS ALL DECLARATIONS
*/
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
 int WIFI_CONNECTION_ATTEMPTS = 5;

//number of tries for ws connection before quitting
 int WS_CONNECTION_ATTEMPTS = 5;

//tracking connectivity
bool isWifiConnected = false;
bool isWSConnected = false;
