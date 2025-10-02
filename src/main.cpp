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
#include <ESP8266HTTPClient.h>

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
byte readCard[4];

bool send_chip_data = true;
bool is_old_chip = false;
byte chip_color = 0x0;

HTTPClient http;
const char* trigger[] = {"psychedelic", "party", "chill", "mystery", "background"};
byte trigger_index = 0;

// UIDs that should trigger "aladdin"
const byte aladdinUIDs[][4] = {
    {0x7D, 0xC4, 0x08, 0x04},
    {0x41, 0xD2, 0x08, 0x04},
    {0x57, 0x85, 0x0B, 0x04},
    {0x19, 0xF6, 0x08, 0x04},
    {0x1F, 0x2D, 0x09, 0x04},
    {0x57, 0xBB, 0x08, 0x04},
    // Add more UIDs as needed
};
const int aladdinUIDCount = sizeof(aladdinUIDs) / sizeof(aladdinUIDs[0]);

// UIDs that should trigger "buttons"
const byte buttonsUIDs[][4] = {
    {0x23, 0x0E, 0xFC, 0x3C},
    {0x14, 0xC1, 0x0B, 0x04},
    {0xA6, 0x0C, 0xFC, 0x3C},
    {0x9A, 0xD6, 0xFC, 0x3C},
    {0x4D, 0x32, 0x0B, 0x04},
    {0x62, 0xA2, 0x0B, 0x04},
    // Add more UIDs as needed
};
const int buttonsUIDCount = sizeof(buttonsUIDs) / sizeof(buttonsUIDs[0]);

unsigned long lastChangeTime = 0;
const unsigned long COLOR_MODE_TIMEOUT = 5000; // 5 seconds

// Helper function to check if readCard matches any UID in the aladdin list
bool isAladdinUID(byte card[4]) {
  for (int i = 0; i < aladdinUIDCount; i++) {
    if (UIDcompare((byte*)aladdinUIDs[i], card, 4)) {
      return true;
    }
  }
  return false;
}

// Helper function to check if readCard matches any UID in the buttons list
bool isButtonsUID(byte card[4]) {
  for (int i = 0; i < buttonsUIDCount; i++) {
    if (UIDcompare((byte*)buttonsUIDs[i], card, 4)) {
      return true;
    }
  }
  return false;
}

// Helper function to send HTTP POST trigger
void sendTrigger(const char* triggerName) {
  Serial.print(triggerName);
  Serial.println(" trigger requested. Sending trigger...");
  http.begin("http://" + String(LED_TRIGGER_SERVICE_IP)+ ":" + String(LED_TRIGGER_SERVICE_PORT) + "/trigger/" + String(triggerName));
  http.addHeader("Content-Type", "application/json");
  int httpResponseCode = http.POST("{}");
  http.end();
  Serial.print(triggerName);
  Serial.print(" trigger sent, HTTP response: ");
  Serial.println(httpResponseCode);
}

// Helper function to send HTTP POST song
void sendSong(const char* songName) {
  Serial.print(songName);
  Serial.println(" song requested. Sending song...");
  http.begin("http://" + String(LED_TRIGGER_SERVICE_IP)+ ":" + String(LED_TRIGGER_SERVICE_PORT) + "/song/" + String(songName) + "/play");
  http.addHeader("Content-Type", "application/json");
  int httpResponseCode = http.POST("{}");
  http.end();
  Serial.print(songName);
  Serial.print(" song sent, HTTP response: ");
  Serial.println(httpResponseCode);
}

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
    // connectToMqtt();
  }
}

void onWifiConnect(const WiFiEventStationModeGotIP& event) {
  Serial.print("[WiFi] Connected, IP address: ");
  Serial.println(WiFi.localIP());
  wifiReconnectTimer.detach();
//   connectToMqtt();
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
  FastLED.setBrightness(64);

  pinMode(LED_GPIO, OUTPUT);

  wifiConnectHandler = WiFi.onStationModeGotIP(onWifiConnect);
  wifiDisconnectHandler = WiFi.onStationModeDisconnected(onWifiDisconnect);

//   mqttClient.onConnect(onMqttConnect);
//   mqttClient.onDisconnect(onMqttDisconnect);
//   mqttClient.onMessage(onMqttMessage);
//   mqttClient.setServer(MQTT_HOST, MQTT_PORT);

//   if (MQTT_USER != "") {
//     mqttClient.setCredentials(MQTT_USER, MQTT_PASS);
//   }

  connectToWifi();
  // Turn ON board led after wifi connect
  digitalWrite(LED_GPIO,HIGH);

  // send init trigger
  sendTrigger("psychedelic");
  master_state = 2; // set to color mode
  lastChangeTime = millis();
  FastLED.clear();
  trigger_index = (trigger_index + 1) % (sizeof(trigger)/sizeof(trigger[0]));
}

/**
 * Main loop.
 */
void loop() {
  if ((master_state != 0) && (millis() - lastChangeTime > COLOR_MODE_TIMEOUT)) {
    master_state = 0; // turn off after timeout
    FastLED.clear();
  }
  set_leds(color, master_state);
  FastLED.show();
  FastLED.delay(20);

  // read the RFID reader version to send over the heartbeat as data
  PICC_version = 0;
  PICC_version = mfrc522.PCD_ReadRegister(MFRC522::VersionReg);

  // Don't read RFID if we're still in an active state (within timeout period)
  if ((master_state != 0) && (millis() - lastChangeTime < COLOR_MODE_TIMEOUT)) {
    return;
  }

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
  }
  dump_byte_array(readCard, 4);

  // get PICC card type
  // Serial.print(F("PICC type: "));
  MFRC522::PICC_Type piccType = mfrc522.PICC_GetType(mfrc522.uid.sak);
  // Serial.println(mfrc522.PICC_GetTypeName(piccType));

  // Check for compatibility
  if (    piccType != MFRC522::PICC_TYPE_MIFARE_MINI
      &&  piccType != MFRC522::PICC_TYPE_MIFARE_1K
      &&  piccType != MFRC522::PICC_TYPE_MIFARE_4K) {
      Serial.println(F("Not a MIFARE Classic card."));
      return;
  }

  // Check if readCard matches any "aladdin" trigger UIDs
  if (isAladdinUID(readCard)) {
    Serial.println("Aladdin UID detected!");
    sendSong("aladdin");
    master_state = 1; // set to pattern mode
    lastChangeTime = millis();
    FastLED.clear();
    mfrc522.PICC_HaltA();
    return;
  }

  // Check if readCard matches any "buttons" trigger UIDs
  if (isButtonsUID(readCard)) {
    Serial.println("Buttons UID detected!");
    sendSong("buttons");
    master_state = 1; // set to pattern mode
    lastChangeTime = millis();
    FastLED.clear();
    mfrc522.PICC_HaltA();
    return;
  }

// send http post request to LED_TRIGGER_SERVICE_IP:LED_TRIGGER_SERVICE_PORT/trigger/{trigger}
    const char* currTrigger = trigger[trigger_index];
    sendTrigger(currTrigger);
    color = trigger_index + 1; // color by index
    Serial.print("color: ");
    Serial.println(color);
    // set master state to color mode for 5 seconds
    master_state = 2; // set to color mode
    lastChangeTime = millis();
    FastLED.clear();
    trigger_index = (trigger_index + 1) % (sizeof(trigger)/sizeof(trigger[0]));
    mfrc522.PICC_HaltA();

/*
  // perform authentication to open communication
  auth_success = authenticate(trailerBlock, key);
  if (!auth_success) {
    //Serial.println(F("Authentication failed"));
    return;
  }

  // read the tag to get coded information
  read_success = read_block(blockAddr, buffer, size);
  if (!read_success) {
    //Serial.println(F("Initial read failed, closing connection"));
    // Halt PICC
    mfrc522.PICC_HaltA();
    // Stop encryption on PCD
    mfrc522.PCD_StopCrypto1();
    return;
  }

  // consider not sending the chip data if its the same chip? prevent system load and errors?
  String UID = String(readCard[0],HEX) + String(readCard[1],HEX) + String(readCard[2],HEX) + String(readCard[3],HEX);

  // check if its an old chip and encode it with new format
  is_old_chip = (buffer[0] != 0x00);  // first byte not 0x00 means old chip
  if (is_old_chip) {
    dataBlock[1] = buffer[1] & 0x0F; // remove valid and win bits from color byte
    dataBlock[15] = 0x04;   // last byte for event track, bit[0] = burnerot2018, bit[1] = contra2019, bit[2] = Midburn2022
    write_success = write_and_verify(blockAddr, dataBlock, buffer, size);
    if (write_success) {
      Serial.println(F("write worked, old chip converted to new format"));
    }
    else {
      Serial.println(F("write failed! aborting chip handling"));
      // Halt PICC
      mfrc522.PICC_HaltA();
      // Stop encryption on PCD
      mfrc522.PCD_StopCrypto1();
      return;
    }
  }

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
  }

  // Dump the sector data, good for debug
  // Serial.println(F("Current data in sector:"));
  // mfrc522.PICC_DumpMifareClassicSectorToSerial(&(mfrc522.uid), &key, sector);
  // Serial.println();
*/

  // Halt PICC
//   mfrc522.PICC_HaltA();
  // Stop encryption on PCD
//   mfrc522.PCD_StopCrypto1();
}
