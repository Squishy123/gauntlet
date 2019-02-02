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
const String WIFI_SSID = "";
const String WIFI_PASSWORD = "";

//unique client id
const String CLIENT_ID = "";

//websocket config
const String WS_HOST = "";
const int WS_PORT = 25565;

//client for creating TCP connections
WiFiClient client;

//websocket client
WebSocketsClient wsClient;

//number of tries for wifi connection before quitting
const int WIFI_CONNECTION_ATTEMPTS = 5;

//number of tries for ws connection before quitting
const int WS_CONNECTION_ATTEMPTS = 5;

//tracking connectivity
bool isWifiConnected = false;
bool isWSConnected = false;

class GauntletNetwork
{
    public:
        //configure wifi client
        boolean wifiClientInit();
        //configure socket client 
        boolean wsClientInit();
        //socket event handling and looping
        void wsEvent(WStype_t type, uint8_t *payload, size_t length);
}