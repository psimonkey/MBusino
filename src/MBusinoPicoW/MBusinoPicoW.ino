/*
# MBusino PicoW: M-Bus --> MQTT-Gateway for Raspberry Pi Pico W
WiFi only (built-in CYW43). Only M-Bus, no accessories.

If no known WiFi is found when starting, an AP with a captive portal is set up.

Ported from MBusinoNano (ESP32 C3 Supermini) to Raspberry Pi Pico W.
Board support: Earle Philhower arduino-pico core v3.x+
https://github.com/earlephilhower/arduino-pico

https://github.com/Zeppelin500/MBusino/

## Licence
****************************************************
This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or (at your option) any later version. This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more details.
You should have received a copy of the GNU General Public License along with this program.  If not, see <http://www.gnu.org/licenses/>.
****************************************************
*/

#include <Arduino.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <AsyncWebServer_RP2040W.h> // Pico W port of ESPAsyncWebServer
#include <DNSServer.h>
#include <ArduinoOTA.h>
#include <WiFi.h>
#include <MBusinoLib.h>             // Library for decode M-Bus
#include <ArduinoJson.h>
#include <EEPROM.h>
#include <MBusCom.h>
#include <Updater.h>                // Pico W OTA updater (replaces ESP32 Update.h)

#define MBUSINO_VERSION "1.0.1"

// EEPROM flag constants
#define EEPROM_CREDENTIALS_OLD 500
#define EEPROM_CREDENTIALS_NEW 501

#define MBUS_ADDRESS 254

#define UPDATE_SIZE_UNKNOWN -1

// M-Bus serial pins (UART1 / Serial2 on Pico W)
// Adjust to match your hardware wiring.
#define MBUS_RX_PIN  9
#define MBUS_TX_PIN  8

MBusinoLib   payload(254);
WiFiClient   wfClient;
PubSubClient client;
DNSServer    dnsServer;
AsyncWebServer server(80);
// NOTE: MBusCom passes rx/tx pins to HardwareSerial::begin().
// arduino-pico uses setRX()/setTX() instead of the ESP32 4-argument form.
// Serial2 pins are pre-configured in setup() before mbus.begin() is called.
// If MBusCom internally calls the ESP32-specific begin(baud, config, rx, tx)
// it will not compile; in that case you will need a Pico W-compatible MBusCom.
MBusCom mbus(&Serial2, MBUS_RX_PIN, MBUS_TX_PIN);

struct settings {
  char ssid[65];
  char password[65];
  char mbusinoName[31];
  char broker[65];
  uint16_t mqttPort;
  uint16_t extension;
  char mqttUser[65];
  char mqttPswrd[65];
  uint32_t sensorInterval;
  uint32_t mbusInterval;
  bool haAutodisc;
  bool telegramDebug;
} userData = {"SSID","Password","MBusino","192.168.1.8",1883,5,"mqttUser","mqttPassword",5000,120000,true,false};


struct oldSettings {
  char ssid[30];
  char password[30];
  char mbusinoName[11];
  char broker[20];
  uint16_t mqttPort;
  uint16_t extension;
  char mqttUser[30];
  char mqttPswrd[30];
  uint32_t sensorInterval;
  uint32_t mbusInterval;
  bool haAutodisc;
  bool telegramDebug;
} oldUserData = {"SSID","Password","MBusino","192.168.1.8",1883,5,"mqttUser","mqttPassword",5000,120000,true,false};

bool mqttcon = false;
bool apMode = false;
bool credentialsReceived = false;
uint16_t conCounter = 0;

int Startadd = 0x13;  // Start address for decoding

uint8_t mbusLoopStatus = 0;
uint8_t fields = 0;
bool fcb = 0; // M-Bus Frame Count Bit
bool initializeSlave = true; // m-bus normalizing is needed
uint8_t recordCounter = 0; // count the received records for multiple telegrams
char jsonstring[4096] = { 0 };
bool engelmann = false;
bool waitForRestart = false;
bool polling = false;
bool wifiReconnect = false;

unsigned long timerMQTT = 15000;
unsigned long timerMbus = 0;
unsigned long timerSerialAvailable = 0;
unsigned long timerMbusDecoded = 0;
unsigned long timerMbusReq = 0;
unsigned long timerDebug = 0;
unsigned long decodingTime = 0;
unsigned long timerReconnect = 0;
unsigned long timerWifiReconnect = 0;
unsigned long timerReboot = 0;
unsigned long timerAutodiscover = 0;
unsigned long timerLedAPmode = 0;
unsigned long timerNetworkPoll = 0;  // Rate-limit WiFi status polling

void setupServer();

uint8_t eeAddrCalibrated = 0;
uint8_t eeAddrCredentialsSaved = 32;
uint16_t credentialsSaved = 123;  // shows if EEPROM used before for credentials

uint8_t adMbusMessageCounter = 0; // Counter for autodiscover mbus message

uint32_t minFreeHeap = 0;

//outsourced program parts
#include "networkEvents.h"
#include "html.h"
#include "mqtt.h"
#include "guiServer.h"
#include "autodiscover.h"

void heapCalc();

void setup() {

  pinMode(LED_BUILTIN, OUTPUT); // LED on if MQTT connected to server
  digitalWrite(LED_BUILTIN, HIGH);
  Serial.begin(115200);

  // Configure M-Bus UART pins before mbus.begin() so Serial2 uses the correct pins.
  Serial2.setRX(MBUS_RX_PIN);
  Serial2.setTX(MBUS_TX_PIN);
  mbus.begin();

  minFreeHeap = rp2040.getFreeHeap();

  EEPROM.begin(512);
  EEPROM.get(eeAddrCredentialsSaved, credentialsSaved);
  if(credentialsSaved == EEPROM_CREDENTIALS_OLD){  // Old version before 1.0 (size of some variables has changed)
    EEPROM.get(100, oldUserData);
    strncpy(userData.ssid, oldUserData.ssid, sizeof(userData.ssid) - 1);
    strncpy(userData.password, oldUserData.password, sizeof(userData.password) - 1);
    strncpy(userData.mbusinoName, oldUserData.mbusinoName, sizeof(userData.mbusinoName) - 1);
    strncpy(userData.broker, oldUserData.broker, sizeof(userData.broker) - 1);
    userData.mqttPort = oldUserData.mqttPort;
    userData.extension = oldUserData.extension;
    strncpy(userData.mqttUser, oldUserData.mqttUser, sizeof(userData.mqttUser) - 1);
    strncpy(userData.mqttPswrd, oldUserData.mqttPswrd, sizeof(userData.mqttPswrd) - 1);
    userData.sensorInterval = oldUserData.sensorInterval;
    userData.mbusInterval = oldUserData.mbusInterval;
    userData.haAutodisc = oldUserData.haAutodisc;
    userData.telegramDebug = oldUserData.telegramDebug;

    EEPROM.put(100, userData);
    credentialsSaved = EEPROM_CREDENTIALS_NEW;
    EEPROM.put(eeAddrCredentialsSaved, credentialsSaved);
  }
  else if(credentialsSaved == EEPROM_CREDENTIALS_NEW){  // Version after 1.0 (size of some variables has changed)
    EEPROM.get(100, userData);
  }
  EEPROM.commit();
  EEPROM.end();

  if(userData.telegramDebug > 1){
    userData.telegramDebug = 0;
  }

  snprintf(html_buffer, sizeof(html_buffer), index_html,
           userData.ssid, userData.mbusinoName, userData.haAutodisc,
           userData.telegramDebug, userData.mbusInterval/1000,
           userData.broker, userData.mqttPort, userData.mqttUser);

  WiFi.hostname(userData.mbusinoName);
  WiFi.mode(WIFI_STA);
  WiFi.begin(userData.ssid, userData.password);

  byte tries = 0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    if (tries++ > 5) {
      WiFi.mode(WIFI_AP);
      WiFi.softAP("MBusino Setup Portal");
      apMode = true;
      break;
    }
  }

  client.setClient(wfClient);
  Serial.println("MQTT set connection via WiFi");

  client.setServer(userData.broker, userData.mqttPort);
  client.setCallback(callback);

  setupServer();
  if(apMode == true){
    dnsServer.start(53, "*", WiFi.softAPIP());
  }

  // Simple Firmware Update Form (web-based OTA)
  server.on("/update", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/html", (const char*)update_html);
  });
  server.on("/update", HTTP_POST, [](AsyncWebServerRequest *request){
    waitForRestart = !Update.hasError();
    if(Update.hasError() == true){
      timerReboot = millis();
    }
    AsyncWebServerResponse *response = request->beginResponse(
      200, "text/plain", waitForRestart ? "success, restart now" : "FAIL");
    response->addHeader("Connection", "close");
    request->send(response);
  },[](AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final){
    if(!index){
      // Begin update; use UPDATE_SIZE_UNKNOWN (-1) so the updater uses available flash
      if(!Update.begin(UPDATE_SIZE_UNKNOWN)){
        Update.printError(Serial);
      }
    }
    if(!Update.hasError()){
      if(Update.write(data, len) != len){
        Update.printError(Serial);
      }
    }
    if(final){
      if(Update.end(true)){
        Serial.printf("Update Success: %uB\n", index + len);
      } else {
        Update.printError(Serial);
      }
    }
  });

  ArduinoOTA.setPassword((const char *)"mbusino");
  server.onNotFound(onRequest);
  ArduinoOTA.begin();
  server.begin();

  client.setBufferSize(6000);
}


void loop() {
  heapCalc();

  ArduinoOTA.handle();

  // Poll WiFi status (replaces ESP32 event callbacks)
  if(millis() - timerNetworkPoll > 1000){
    timerNetworkPoll = millis();
    checkWiFiStatus();
  }

  if(wifiReconnect == true && (millis() - timerWifiReconnect > 2000)){
    Serial.println("try to reconnect");
    wifiReconnect = false;
    WiFi.begin(userData.ssid, userData.password);
  }

  if(credentialsReceived == true && waitForRestart == false){
    Serial.println("credentials received, save and restart soon");
    EEPROM.begin(512);
    EEPROM.put(100, userData);
    credentialsSaved = EEPROM_CREDENTIALS_NEW;
    EEPROM.put(eeAddrCredentialsSaved, credentialsSaved);
    EEPROM.commit();
    EEPROM.end();
    timerReboot = millis();
    waitForRestart = true;
  }

  if(waitForRestart == true && (millis() - timerReboot) > 1000){
    Serial.println("restart");
    rp2040.reboot();
  }

  if(apMode == true && millis() > 300000){
    rp2040.reboot();
  }

  if(apMode == true){
    dnsServer.processNextRequest();
    if((millis() - timerLedAPmode) > 500 && digitalRead(LED_BUILTIN) == LOW){
      digitalWrite(LED_BUILTIN, HIGH);
      timerLedAPmode = millis();
    }
    if((millis() - timerLedAPmode) > 500 && digitalRead(LED_BUILTIN) == HIGH){
      digitalWrite(LED_BUILTIN, LOW);
      timerLedAPmode = millis();
    }
  }

  if (!client.connected()) {
    if((millis() - timerReconnect) > 5000){
      Serial.println("MQTT no connection");
      if(apMode == false){
        Serial.println("MQTT no connection - RECONNECT");
        reconnect();
      }
      else{
        Serial.println(" --> AP is running");
      }
      timerReconnect = millis();
    }
  }
  else{ // the whole main code runs only if MQTT is connected
    client.loop();  // MQTT function

    ///////////////// publish settings ///////////////////////////////////
    if((millis()-timerDebug) > 10000){
      timerDebug = millis();

      client.publish(String(String(userData.mbusinoName) + "/settings/wl_Connected").c_str(), String(WL_CONNECTED).c_str());
      client.publish(String(String(userData.mbusinoName) + "/settings/apMode").c_str(), String(apMode).c_str());

      client.publish(String(String(userData.mbusinoName) + "/settings/ssid").c_str(), userData.ssid);
      client.publish(String(String(userData.mbusinoName) + "/settings/broker").c_str(), userData.broker);
      client.publish(String(String(userData.mbusinoName) + "/settings/port").c_str(), String(userData.mqttPort).c_str());
      client.publish(String(String(userData.mbusinoName) + "/settings/user").c_str(), userData.mqttUser);
      client.publish(String(String(userData.mbusinoName) + "/settings/name").c_str(), userData.mbusinoName);
      client.publish(String(String(userData.mbusinoName) + "/settings/mbusInterval").c_str(), String(userData.mbusInterval).c_str());
      client.publish(String(String(userData.mbusinoName) + "/settings/wifiIP").c_str(), String(WiFi.localIP().toString()).c_str());
      client.publish(String(String(userData.mbusinoName) + "/settings/MQTTreconnections").c_str(), String(conCounter-1).c_str());
      client.publish(String(String(userData.mbusinoName) + "/settings/MQTTconnectedVia").c_str(), "WiFi");
      long rssi = WiFi.RSSI();
      client.publish(String(String(userData.mbusinoName) + "/settings/RSSI").c_str(), String(rssi).c_str());
      client.publish(String(String(userData.mbusinoName) + "/settings/version").c_str(), MBUSINO_VERSION);
      client.publish(String(String(userData.mbusinoName) + "/settings/freeHeap").c_str(), String(rp2040.getFreeHeap()).c_str());
      client.publish(String(String(userData.mbusinoName) + "/settings/minFreeHeap").c_str(), String(minFreeHeap).c_str());
    }

  ////////// M-Bus ###############################################

    if(initializeSlave == true){
      initializeSlave = false;
      mbus.normalize(MBUS_ADDRESS);
      timerMbus = millis();
    }

    /*
    mbusLoopStatus
    0 = request the records from the slave
    1 = wait for response of the slave
    2 = get response from the rx buffer and decode the telegram
    3 = Send decoded M-Bus records via MQTT
    */

    switch(mbusLoopStatus){
      case 0:
        if((millis() - timerMbus > userData.mbusInterval || polling == true) && mbusLoopStatus == 0){ // Request M-Bus Records
          if(polling == false){ // if polling is true or a following telegram of a multi telegram, don't touch the timer
            timerMbus = millis();
          }
          polling = false;
          mbusLoopStatus = 1;
          mbus.clearRXbuffer();
          mbus.request_data(MBUS_ADDRESS, fcb);
          timerMbusReq = millis();
        }
        break;
      case 1:
        if(mbus.available()){
          mbusLoopStatus = 2;
          timerSerialAvailable = millis();
        }
        if(millis() - timerMbusReq > 2000){ // failure, no data received
          initializeSlave = true;
          recordCounter = 0;
          fcb = false;
          client.publish(String(String(userData.mbusinoName) + "/MBus/MBUSerror").c_str(), "no_Data_received");
          mbusLoopStatus = 0;
          polling = false;
        }
        break;

      case 2:
        if((millis() - timerSerialAvailable) > 1500){ // Receive and decode M-Bus Records
        decodingTime = millis();
        mbusLoopStatus = 3;
        bool mbus_good_frame = false;
        byte mbus_data[MBUS_DATA_SIZE] = { 0 };
        mbus_good_frame = mbus.get_response(mbus_data, sizeof(mbus_data));

        if(userData.telegramDebug == true){
        //------------------ only for debug, receive the whole M-Bus telegram bytewise in HEX -----------------
          char telegram[769] = {0};
          for(uint8_t i = 0; i <= mbus_data[1]+6; i++){
            char buffer[3];
            sprintf(buffer, "%02X", mbus_data[i]);
            telegram[i*3] = buffer[0];
            telegram[(i*3)+1] = buffer[1];
            telegram[(i*3)+2] = ' ';
          }
          client.publish(String(String(userData.mbusinoName) + "/debug/telegram").c_str(), telegram);
          //--------------------------------------------------------------------------------------------------------------------------
        }
        if (mbus_good_frame) {
          if(fcb == true){ // toggle the FCB (Frame Count Bit) to signalize good response in the next request
            fcb = false;
          }else{
            fcb = true;
          }

          adMbusMessageCounter++;
          int packet_size = mbus_data[1] + 6;
          JsonDocument jsonBuffer;
          JsonArray root = jsonBuffer.add<JsonArray>();
          fields = payload.decode(&mbus_data[Startadd], packet_size - Startadd - 2, root);
          serializeJson(root, jsonstring); // store the json in a global array
          // test -----------------------------------------------------------------------------------------
          uint16_t arraycounter = 0;
          uint8_t findTheTerminator = 1;
          while(findTheTerminator != 0){
            findTheTerminator = jsonstring[arraycounter];
            arraycounter++;
          }
          client.publish(String(String(userData.mbusinoName) + "/MBus/jsonlen").c_str(), String(arraycounter).c_str());
          // test ende -----------------------------------------------------------------------------------------
          client.publish(String(String(userData.mbusinoName) + "/MBus/error").c_str(), String(payload.getError()).c_str());
          client.publish(String(String(userData.mbusinoName) + "/MBus/jsonstring").c_str(), jsonstring);
          uint8_t address = mbus_data[5];
          client.publish(String(String(userData.mbusinoName) + "/MBus/address").c_str(), String(address).c_str());

          client.publish(String(String(userData.mbusinoName) + "/MBus/FCB").c_str(), String(fcb).c_str());

          heapCalc();
          if(mbus_data[12] == 0x14 && mbus_data[11] == 0xC5){
            engelmann = true;
          }
          else{
            engelmann = false;
          }
        }
        else {  // Error
            mbusLoopStatus = 0;
            initializeSlave = true;
            jsonstring[0] = 0;
            client.publish(String(String(userData.mbusinoName) + "/MBUSerror").c_str(), "no_good_telegram");
        }
        timerMbusDecoded = millis();
      }
      break;

      case 3:
        if(millis() - timerMbusDecoded > 100){  // Send decoded M-Bus records via MQTT
          mbusLoopStatus = 0;
          JsonDocument root;
          deserializeJson(root, jsonstring); // load the json from a global array
          jsonstring[0] = 0;
          client.publish(String(String(userData.mbusinoName) + "/debug/adMbusMessageCounter").c_str(), String(adMbusMessageCounter).c_str());

          for (uint8_t i=0; i<fields; i++) {
            uint8_t code = root[i]["code"].as<int>();
            const char* name = root[i]["name"];
            const char* units = root[i]["units"];
            double value = root[i]["value_scaled"].as<double>();
            const char* valueString = root[i]["value_string"];
            bool telegramFollow = root[i]["telegramFollow"].as<int>();

            if(userData.haAutodisc == true && adMbusMessageCounter == 3){  // every 264 messages a HA autoconfig message
              strcpy(adVariables.haName, name);
              if(units != NULL){
                strcpy(adVariables.haUnits, units);
              }else{
                strcpy(adVariables.haUnits, "");
              }
              strcpy(adVariables.stateClass, payload.getStateClass(code));
              strcpy(adVariables.deviceClass, payload.getDeviceClass(code));
              haHandoverMbus(recordCounter+i+1, engelmann);
            }else{
              // two messages per value: value as number or as ASCII string
              client.publish(String(String(userData.mbusinoName) + "/MBus/"+String(recordCounter+i+1)+"_vs_"+String(name)).c_str(), valueString); // send the value if a ascii value is aviable (variable length)
              client.publish(String(String(userData.mbusinoName) + "/MBus/"+String(recordCounter+i+1)+"_"+String(name)).c_str(), String(value,3).c_str()); // send the value if a real value is aviable (standard)
              client.publish(String(String(userData.mbusinoName) + "/MBus/"+String(recordCounter+i+1)+"_"+String(name)+"_unit").c_str(), units);
              //or only one message
              //client.publish(String(String(userData.mbusinoName) + "/MBus/"+String(recordCounter+i+1)+"_"+String(name)+"_in_"+String(units)), String(value,3).c_str());

              if (i == 3 && engelmann == true){  // Sensostar Bugfix --> comment out if not using a Sensostar
                float flow = root[5]["value_scaled"].as<float>();
                float delta = root[9]["value_scaled"].as<float>();
                float calc_power = delta * flow * 1163;
                client.publish(String(String(userData.mbusinoName) + "/MBus/4_power_calc").c_str(), String(calc_power).c_str());
              }
            }

            if(fields == i+1){
              if(telegramFollow == 1){
                client.publish(String(String(userData.mbusinoName) + "/MBus/"+String(recordCounter+i+1)).c_str(), "--> More records follow in next telegram");
                recordCounter = recordCounter + fields;
                polling = true;
                adMbusMessageCounter = adMbusMessageCounter - 1;
                //client.publish(String(String(userData.mbusinoName) +"/debug/telegramRF").c_str(), telegram);
              }else{
                recordCounter = 0;
                //adMbusMessageCounter++;
                //polling = true;
              }
            }
          }
          client.publish(String(String(userData.mbusinoName) + "/debug/decodingTime").c_str(), String((millis() - decodingTime)).c_str());
          heapCalc();
        }
        break;
    } // Switch
  }
}

void heapCalc(){
  if(minFreeHeap > rp2040.getFreeHeap()){
    minFreeHeap = rp2040.getFreeHeap();
  }
}
