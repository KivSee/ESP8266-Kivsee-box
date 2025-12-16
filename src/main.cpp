#define ARDUINOJSON_USE_LONG_LONG 1
#define FASTLED_ESP8266_DMA

#include <Arduino.h>
#include <SPI.h>
#include <MFRC522.h>
#include <FastLED.h>
#include <ESP8266WiFi.h>
#include <Ticker.h>
#include <AsyncMqttClient.h>
#include <ArduinoJson.h>
#include <ArduinoOTA.h>
#include <stdlib.h>
#include "config.h"

AsyncMqttClient mqttClient;
Ticker mqttReconnectTimer;

WiFiEventHandler wifiConnectHandler;
WiFiEventHandler wifiDisconnectHandler;
Ticker wifiReconnectTimer;

#define RING_LEDS 16
#define RINGS     4
#define NUM_LEDS (RING_LEDS*RINGS)
#define DATA_PIN 9
#define LED_GPIO D0

// Define the array of leds
CRGB leds[NUM_LEDS];
byte color = 0x2;
byte master_state = 0x0; //Off=0, Pattern=1, Color=2
#include "LED_control.h"

MFRC522 mfrc522(SS_PIN, RST_PIN);  // Create MFRC522 instance
#include "MFRC522_func.h"

MFRC522::MIFARE_Key key;
//MFRC522::StatusCode status;
byte sector         = 1;
byte blockAddr      = 4;
byte dataBlock[]    = {
        0x00, 0x00, 0x00, 0x00, //  byte 1 for color encoding
        0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x04  // byte 15 for event track bit[0] = burnerot2018, bit[1] = contra2019, bit[2] = Midburn2022
    };
byte trailerBlock   = 7;
byte buffer[18];
byte size = sizeof(buffer);
bool read_success, write_success, auth_success;
byte PICC_version;
unsigned int readCard[4];

bool send_chip_data = true;
bool is_old_chip = false;
byte chip_color = 0x0;


void connectToMqtt() {
  Serial.println("[MQTT] Connecting to MQTT...");
  mqttClient.setClientId(WIFI_CLIENT_ID);
  mqttClient.setKeepAlive(5);
  mqttClient.setWill(MQTT_TOPIC_MONITOR,MQTT_TOPIC_MONITOR_QoS,true,"{\"alive\": false}");
  mqttClient.connect();
}

void connectToWifi() {
  Serial.printf("[WiFi] Connecting to %s...\n", WIFI_SSID);

  WiFi.hostname(WIFI_CLIENT_ID);
  WiFi.mode(WIFI_STA);
  if (WIFI_STATIC_IP != 0) {
    WiFi.config(WIFI_CLIENT_IP, WIFI_GATEWAY_IP, WIFI_SUBNET_IP, WIFI_DNS_IP);
  }

  delay(100);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  delay(10000);
  if (WiFi.status() != WL_CONNECTED) {
    WiFi.disconnect(true);
    WiFi.mode(WIFI_STA);
    Serial.println("[WiFi] timeout, disconnecting WiFi");
    ESP.restart();
  }
  else {
    Serial.println(WiFi.localIP());
    wifiReconnectTimer.detach();
    connectToMqtt();
  }
}

void onWifiConnect(const WiFiEventStationModeGotIP& event) {
  Serial.print("[WiFi] Connected, IP address: ");
  Serial.println(WiFi.localIP());
  wifiReconnectTimer.detach();
  connectToMqtt();
}

void onWifiDisconnect(const WiFiEventStationModeDisconnected& event) {
  Serial.println("[WiFi] Disconnected from Wi-Fi!");
  mqttReconnectTimer.detach(); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
  wifiReconnectTimer.once(WIFI_RECONNECT_TIME, connectToWifi);
}

void onMqttConnect(bool sessionPresent) {
  Serial.println("[MQTT] Connected to MQTT!");

  mqttReconnectTimer.detach();

  Serial.print("Session present: ");
  Serial.println(sessionPresent);
  uint16_t packetIdSub = mqttClient.subscribe(MQTT_TOPIC_LEDS, MQTT_TOPIC_LEDS_QoS);
  Serial.print("Subscribing to ");
  Serial.println(MQTT_TOPIC_LEDS);
  Serial.println("Sending alive message");
  mqttClient.publish(MQTT_TOPIC_MONITOR, MQTT_TOPIC_MONITOR_QoS, true, "{\"alive\": true}");
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println("[MQTT] Disconnected from MQTT!");

  if (WiFi.isConnected()) {
    Serial.println("[MQTT] Trying to reconnect...");
    mqttReconnectTimer.once(MQTT_RECONNECT_TIME, connectToMqtt);
  }
}

void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total) {
  if (!strcmp(topic, MQTT_TOPIC_LEDS)) {
    Serial.print("Leds message: ");
    Serial.print(len);
    Serial.print(" [");
    Serial.print(topic);
    Serial.print("] ");
    for (int i = 0; i < len; i++) {
      Serial.print((char)payload[i]);
    }
    Serial.println();

    DynamicJsonDocument doc(50);
    deserializeJson(doc, payload);
    color = doc["color"];
    master_state = doc["master_state"];
    Serial.print("color: ");
    Serial.println(color);
    Serial.print("master_state: ");
    Serial.println(master_state);
    FastLED.clear();
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("Startup!");
  SPI.begin();        // Init SPI bus
  mfrc522.PCD_Init(); // Init MFRC522 card
  mfrc522.PCD_DumpVersionToSerial();	// Show details of PCD - MFRC522 Card Reader details
  // Serial.println(F("Scan PICC to see UID, SAK, type, and data blocks..."));
  // Prepare the key (used both as key A and as key B)
  // using FFFFFFFFFFFFh which is the default at chip delivery from the factory
  for (byte i = 0; i < 6; i++) {
    key.keyByte[i] = 0xFF;
  }

  FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds, NUM_LEDS);
  FastLED.setBrightness(32);

  pinMode(LED_GPIO, OUTPUT);

  wifiConnectHandler = WiFi.onStationModeGotIP(onWifiConnect);
  wifiDisconnectHandler = WiFi.onStationModeDisconnected(onWifiDisconnect);

  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onMessage(onMqttMessage);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);

  if (MQTT_USER != "") {
    mqttClient.setCredentials(MQTT_USER, MQTT_PASS);
  }

  connectToWifi();

  // Configure OTA
  ArduinoOTA.setHostname(WIFI_CLIENT_ID);
  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else {
      type = "filesystem";
    }
    Serial.println("[OTA] Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\n[OTA] Update complete");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("[OTA] Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("[OTA] Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) {
      Serial.println("Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      Serial.println("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      Serial.println("Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      Serial.println("Receive Failed");
    } else if (error == OTA_END_ERROR) {
      Serial.println("End Failed");
    }
  });
  ArduinoOTA.begin();
  Serial.println("[OTA] Ready");

  // Turn ON board led after wifi connect
  digitalWrite(LED_GPIO,HIGH);
  Serial.println("Setup done.");
}

/**
 * Main loop.
 */
void loop() {
  ArduinoOTA.handle();

  set_leds(color, master_state);
  FastLED.show();
  FastLED.delay(20);

  // read the RFID reader version to send over the heartbeat as data
  PICC_version = 0;
  PICC_version = mfrc522.PCD_ReadRegister(MFRC522::VersionReg);

  // START RFID HANDLING
  // Look for new cards
  if ( ! mfrc522.PICC_IsNewCardPresent())
    return;

  // Select one of the cards
  if ( ! mfrc522.PICC_ReadCardSerial())
    return;

  // Dump debug info about the card; PICC_HaltA() is automatically called
  // mfrc522.PICC_DumpToSerial(&(mfrc522.uid));

  // get card uid
  Serial.print("found tag ID: ");
  for (int i = 0; i < mfrc522.uid.size; i++) {  // for size of uid.size write uid.uidByte to readCard
    readCard[i] = mfrc522.uid.uidByte[i];
    Serial.print(readCard[i], HEX);
  }
  Serial.println();

  // get PICC card type
  // Serial.print(F("PICC type: "));
//   MFRC522::PICC_Type piccType = mfrc522.PICC_GetType(mfrc522.uid.sak);
  // Serial.println(mfrc522.PICC_GetTypeName(piccType));

  // Check for compatibility
//   if (    piccType != MFRC522::PICC_TYPE_MIFARE_MINI
//       &&  piccType != MFRC522::PICC_TYPE_MIFARE_1K
//       &&  piccType != MFRC522::PICC_TYPE_MIFARE_4K) {
//       Serial.println(F("Not a MIFARE Classic card."));
//       return;
//   }

  // perform authentication to open communication
//   auth_success = authenticate(trailerBlock, key);
//   if (!auth_success) {
//     //Serial.println(F("Authentication failed"));
//     return;
//   }

  // read the tag to get coded information
//   read_success = read_block(blockAddr, buffer, size);
//   if (!read_success) {
//     //Serial.println(F("Initial read failed, closing connection"));
//     // Halt PICC
//     mfrc522.PICC_HaltA();
//     // Stop encryption on PCD
//     mfrc522.PCD_StopCrypto1();
//     return;
//   }

  // consider not sending the chip data if its the same chip? prevent system load and errors?
  String UID = String(readCard[0],HEX) + String(readCard[1],HEX) + String(readCard[2],HEX) + String(readCard[3],HEX);

  // check if its an old chip and encode it with new format
//   is_old_chip = (buffer[0] != 0x00);  // first byte not 0x00 means old chip
//   if (is_old_chip) {
//     dataBlock[1] = buffer[1] & 0x0F; // remove valid and win bits from color byte
//     dataBlock[15] = 0x04;   // last byte for event track, bit[0] = burnerot2018, bit[1] = contra2019, bit[2] = Midburn2022
//     write_success = write_and_verify(blockAddr, dataBlock, buffer, size);
//     if (write_success) {
//       Serial.println(F("write worked, old chip converted to new format"));
//     }
//     else {
//       Serial.println(F("write failed! aborting chip handling"));
//       // Halt PICC
//       mfrc522.PICC_HaltA();
//       // Stop encryption on PCD
//       mfrc522.PCD_StopCrypto1();
//       return;
//     }
//   }

  // after changing old chips to new format testing for old chips is done on byte 15 bit[0]
  is_old_chip = (buffer[15] & 0x01);
  chip_color = buffer[1]; // color is at byte 1

  if (mqttClient.connected() && send_chip_data) {
    StaticJsonDocument<128> chip_data;
    chip_data["UID"] = UID;
    chip_data["color"] = chip_color;
    chip_data["old_chip"] = is_old_chip;
    char chip_data_buffer[100];
    serializeJson(chip_data, chip_data_buffer);
    mqttClient.publish(MQTT_TOPIC_CHIP, MQTT_TOPIC_CHIP_QoS, false, chip_data_buffer);
    Serial.print("Sending chip data: ");
    serializeJson(chip_data, Serial);
    Serial.println();
    color = 128; // set leds to rainbow on chip read
  }

  // Dump the sector data, good for debug
  // Serial.println(F("Current data in sector:"));
  // mfrc522.PICC_DumpMifareClassicSectorToSerial(&(mfrc522.uid), &key, sector);
  // Serial.println();

  // Halt PICC
  mfrc522.PICC_HaltA();
  // Stop encryption on PCD
  mfrc522.PCD_StopCrypto1();
}
