/*
    Gauntlet Network Manager (source)
    By: Christian Wang
    Release Date: 2019/01/28

    Description: Provides functions to setup and connect to WiFi networks
    using the ESP8266 board. Also helps connect as a client of a websocket server.
*/
#include "GauntletNetwork.h"

/**
 * Attempt to connect to wifi network
 * Tries connection WIFI_CONNECTION_ATTEMPTS times 
 * with a 500ms delay each time 
 * Return true if successful, false if unsuccessful
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

/**
* Attempt to connect to WS server
 * Tries connection WS_CONNECTION_ATTEMPTS times 
 * with a 500ms delay each time 
 * Return true if successful, false if unsuccessful
 */
boolean wsClientInit()
{
    //verbose stuff
    Serial.println("Connecting to WS server" + WS_HOST);
    Serial.println("Attempting...");

    for (int i = 0; i < WS_CONNECTION_ATTEMPTS; i++)
    {
        //check if connected
        if (client.connect(WS_HOST, WS_PORT);)
        {
            isWSConnected = true;
            webSocketClient.begin(WS_HOST, WS_PORT, "/");
            webSocketClient.onEvent(webSocketEvent);

            Serial.println("Successfully conencted to Gauntlet Server!");
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