/**
 * @Author: Lukas Prokop
 * @Date:   2024-03-04 11:01:01
 * @Last Modified by:   Lukas Prokop
 * @Last Modified time: 2024-03-25 14:25:20
 */

#include <pins.h> // rename pins.h.example and adjust pins
#include <LogicData.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ArduinoOTA.h>
#include <Credentials.h> // rename Credential.h.example and adjust variables
#include <ArduinoJson.h>
#include <RemoteDebug.h>

uint8_t highTarget = 110; //116
uint8_t lowTarget = 80;
uint8_t maxHeight = 114; //maxHeight table = 128, but you may set a custom min
uint8_t minHeight = 70; //minHeight table = 62, but you may set a custom min

const uint32_t debounce_time = 50;
const uint32_t double_time = 500;
int btn_pins[] = {BTN_UP, BTN_DOWN};
const int btn_pressed_state = HIGH;// when high, button is pressed

#define ARRAY_SIZE(array) (sizeof(array) / sizeof(array[0]))
#define BTN_COUNT ARRAY_SIZE(btn_pins)
int8_t btn_last_state[BTN_COUNT] = {-1};
int8_t btn_last_double[BTN_COUNT] = {-1};
uint32_t debounce[BTN_COUNT] = {0};
uint32_t btn_last_on[BTN_COUNT] = {0};

//last_signal is just the last time input was read from buttons or from controller
//If we haven't seen anything from either in a bit, stop moving
uint32_t last_signal = 0;
uint32_t signal_giveup_time = 2000;

long lastPublish = 0;
long lastPublishHASS = 0;
bool discoveryPublished = false;
uint8_t publishedHeight = 0;

const char* versionLine = "SmartDesk v0.2 build: " __DATE__ " " __TIME__;
LogicData logicData(-1);
WiFiClient espClient;
PubSubClient mqttClient(espClient);

uint8_t currentHeight;
uint8_t targetHeight;
bool setHeight = false;
enum Directions { UP, DOWN, STOPPED };
Directions direction = STOPPED;
// uptime in miliseconds
uint32_t uptime = 0;

#ifndef ESP8266 
  #define ESP8266
#endif

// #define DEBUG_ACTIVE

RemoteDebug Debug;
#ifdef DEBUG_ACTIVE
  bool debug = true;  
#else
  bool debug = false;
#endif

/* Helpers */

/**
 * @brief checks if passed height is within allowed range
 *        also allows for up/down movement if below/above min/max
 * 
 * @param checkHeight 
 * @param direction 
 * @return true 
 * @return false 
 */
bool isValidHeight(int checkHeight, Directions direction = STOPPED) {
  if (checkHeight >= minHeight && checkHeight <= maxHeight)
    return true;
  else {
    // allow to move in the direction away from min/max
    if (direction == UP && checkHeight <= maxHeight)
      return true;
    else if (direction == DOWN && checkHeight >= minHeight)
      return true;
    else
      return false;
  }
}

/* Logging helpers */

/**
 * @brief Logs the message to the serial and mqtt
 *
 * @param message - the message to log
 */
void logItE(const char *format, ...)
{
  va_list args;
  va_start(args, format);
  char buf[256];
  vsnprintf(buf, sizeof(buf), format, args);
  va_end(args);
  debugI("ERROR: %s", buf);
  mqttClient.publish((MQTT_TOPIC + "info").c_str(), buf);
}

void logItI(const char *format, ...)
{
  va_list args;
  va_start(args, format);
  char buf[256];
  vsnprintf(buf, sizeof(buf), format, args);
  va_end(args);
  debugI("INFO: %s", buf);
  mqttClient.publish((MQTT_TOPIC + "info").c_str(), buf);
}

void logItD(const char *format, ...)
{
  if (debug)
  {
    va_list args;
    va_start(args, format);
    char buf[256];
    vsnprintf(buf, sizeof(buf), format, args);
    va_end(args);
    debugI("DEBUG: %s", buf);
    // mqttClient.publish((MQTT_TOPIC + "info").c_str(), buf);
  }
}

/* Logicdata related */

/**
 * @brief Buffered mode parses input words and sends them to output separately
 * 
 */
void IRAM_ATTR logicDataPin_ISR() {
  logicData.PinChange(HIGH == digitalRead(LOGICDATA_RX));
}

/**
 * @brief Checks the display - the display being the (non-existing)
 *        remote display to the motor fo the table
 *        Records the last time the display changed & currentHeigt
 */
void check_display() {
  static uint32_t prev = 0;
  uint32_t msg = logicData.ReadTrace();
  char buf[80];
  if (msg) {
    uint32_t now = millis();
    sprintf(buf, "%6ums %s: %s", now - prev, logicData.MsgType(msg), logicData.Decode(msg));
    logItD("Buffer: %s", buf);
    log(msg);
    prev=now;
  }

  // Reset idle-activity timer if display number changes or if any other display activity occurs (i.e. display-ON)
  if (logicData.IsNumber(msg)) {
    auto new_height = logicData.GetNumber(msg);
    if (new_height == currentHeight) {
      return;
    }
    currentHeight = new_height;
  }
  if (msg)
    last_signal = millis();
}


/* Table Movement */

/**
 * @brief Stops the table and resets variables
 * 
 */
void stop_table() {
    digitalWrite(ASSERT_UP, LOW);
    digitalWrite(ASSERT_DOWN, LOW);
    targetHeight = currentHeight;
    setHeight = false;

    if (direction != STOPPED) {
      mqttClient.publish((MQTT_TOPIC + "state").c_str(), "stopped");
      direction = STOPPED;
    }
}

/**
 * @brief Moves the table up or down
 * 
 * @param tmpDirection The direction the table moves to (up/down)
 */
void move_table(Directions tmpDirection) {
  // currentHeight is initially 0 before the first move
  if (currentHeight == 0 || isValidHeight(currentHeight, tmpDirection)) {
    // move the table up or down setting the pins to high/low
    digitalWrite(ASSERT_UP, (tmpDirection == UP ? HIGH : LOW));
    digitalWrite(ASSERT_DOWN, (tmpDirection == DOWN ? HIGH : LOW));

    //make sure to only log if there was a change
    if (direction != tmpDirection) {
      mqttClient.publish((MQTT_TOPIC + "state").c_str(), (tmpDirection == UP ? "up" : "down"));
      direction = tmpDirection;
    }
  } else if (!isValidHeight(currentHeight, tmpDirection)) {
    logItE("Non valid height [%d] received. Stopping table.", currentHeight, "e");
    stop_table();
  }
}

/**
 * @brief Moves the table to a fixed position indicated by the Directions (Up/Down)
 * 
 * @param highLowTarget Used to leverage UP/DOWN as high/low height targets
 */
void move_table_to_fixed(Directions highLowTarget) {
    setHeight = true;

    if (highLowTarget == UP)
      targetHeight = highTarget;
    else if (highLowTarget == DOWN)
      targetHeight = lowTarget;

    logItI("Start setting height. %s target: %d cm. Current height: %d cm.", highLowTarget == UP ? "High" : "Low", targetHeight, currentHeight);
}

/**
 * @brief Dispatcher function that takes care of button states
 *        or set to target height
 * 
 */
void move() {
  //btn_last_state has the current buttons pressed
  if(btn_last_state[0] && btn_last_state[1]) {
    //both buttons pressed, do nothing
    //TODO: Save position to EEPROM like https://github.com/talsalmona/RoboDesk/blob/master/RoboDesk.ino
    logItD("Both buttons pressed");
  } else if(btn_last_state[0]) {
    //left button pressed
    move_table(UP);
    return;
  } else if(btn_last_state[1]) {
    //right button pressed
    move_table(DOWN);
    return;
  } else if (!setHeight) {
    if( direction != STOPPED) {
      logItI("button [%s] press stopped. Current height: %d cm", direction == UP ? "up" : "down", currentHeight);
      stop_table();
    }
    return;
  }

  if((millis() - last_signal > signal_giveup_time) && !setHeight) {
    logItE("Haven't seen input in a while, turning everything off for safety");
    stop_table();
    while(true) ;
  }

  // move the table if in setHeight-mode
  if(currentHeight != targetHeight) {
    if (setHeight) {
      if (currentHeight > targetHeight)
        move_table(DOWN);
      else
        move_table(UP);
      return;
    }
  } else {
    logItI("Hit target height: %d cm", targetHeight);
    stop_table();
    return;
  }
}

/* MQTT functions */

/**
 * @brief Sets the device information in the JSON document
 *
 * @param doc - the JSON document
 */
void setJsonDevice(JsonDocument &doc) {
  String macAddress = WiFi.macAddress();
  macAddress.replace(":", "_"); // Replace colons with underscores
  String unique_id = "table_" + macAddress;
  String main_topic = MQTT_TOPIC.substring(0, MQTT_TOPIC.length() - 1);

  doc["device"]["identifiers"] = unique_id;
  doc["device"]["name"] = main_topic;
  doc["device"]["manufacturer"] = "Lukas Prokop";
  doc["device"]["sw_version"] = versionLine;
  doc["device"]["model"] = "SmartDesk Table ESP8266";
  doc["device"]["hw_version"] = "1.0";
}

/**
 * @brief Creates and publish the discovery message for Home Assistant
 *
 * @param field - the field to publish
 * @param name - the name of the field
 * @param icon - the icon to use
 * @param unit - the unit of measurement
 * @param deviceClass - the device class
 * @param stateClass - the state class
 * @param entityCategory - the entity category
 */
void publishSensorDiscoveryToHASS(String field, String name, String icon, String unit, String deviceClass, String stateClass, String entityCategory)
{
  JsonDocument doc;
  String output;
  String hass_topic = "homeassistant/sensor/table-" + field + "/config";
  String main_topic = MQTT_TOPIC.substring(0, MQTT_TOPIC.length() - 1);

  // check that input params are not empty
  if (field == "" || name == "")
  {
    logItE("ERROR: Missing input parameters for discovery message!");
    return;
  }

  doc["~"] = main_topic;
  doc["name"] = name;
  doc["unique_id"] = main_topic + "-" + field;
  doc["object_id"] = main_topic + "_" + field;
  doc["state_topic"] = "~/" + field;
  if (icon != "")
    doc["icon"] = "mdi:" + icon;
  // doc["value_template"] = "{{ value_json." + field + " }}";
  if (unit != "")
    doc["unit_of_measurement"] = unit;
  if (deviceClass != "")
    doc["device_class"] = deviceClass;
  if (stateClass != "")
    doc["state_class"] = stateClass;
  if (entityCategory != "")
    doc["entity_category"] = entityCategory;

  doc["availability_topic"] = "~/status";
  doc["payload_available"] = "ON";
  doc["payload_not_available"] = "OFF";

  setJsonDevice(doc);
  serializeJson(doc, output);
  mqttClient.publish(hass_topic.c_str(), output.c_str());
  logItD("Published -> Topic: %s\nMessage: %s", hass_topic, output);
}

// Publish switch discovery message to Home Assistant
void publishButtonDiscoveryToHASS(String field, String name, String icon, String deviceClass = "button") {
  JsonDocument doc;
  String output;
  String hass_topic = "homeassistant/" + deviceClass + "/table-" + field + "/config";
  String main_topic = MQTT_TOPIC.substring(0, MQTT_TOPIC.length() - 1);

  // check that input params are not empty
  if (field == "" || name == "")
  {
    logItE("ERROR: Missing input parameters for discovery message!");
    return;
  }

  doc["~"] = main_topic;
  doc["name"] = name;
  doc["unique_id"] = main_topic + "-" + field;
  doc["object_id"] = main_topic + "_" + field;
  if (icon != "")
    doc["icon"] = "mdi:" + icon;
  doc["command_topic"] = "~/set";
  doc["payload_press"] = "{\"cmd\":\"" + field + "\"}";
  doc["assume_state"] = true;

  setJsonDevice(doc);  
  serializeJson(doc, output);
  mqttClient.publish(hass_topic.c_str(), output.c_str());
  logItD("Published -> Topic: %s\nMessage: %s", hass_topic, output);
}

/**
 * @brief Publishes the discovery messages to Home Assistant - helper function
 *
 */
void publishDiscovery()
{
  publishSensorDiscoveryToHASS("height", "Height", "table", "cm", "distance", "measurement", "");
  publishSensorDiscoveryToHASS("state", "State", "power", "", "", "", "diagnostic");
  publishSensorDiscoveryToHASS("uptime", "Uptime", "clock", "s", "", "", "diagnostic");
  publishSensorDiscoveryToHASS("MAC", "MAC Address", "network-outline", "", "", "", "diagnostic");
  publishSensorDiscoveryToHASS("IP", "IP Address", "network-outline", "", "", "", "diagnostic");
  publishSensorDiscoveryToHASS("hostname", "Hostname", "network-outline", "", "", "", "diagnostic");
  publishSensorDiscoveryToHASS("wifiRSSI", "WiFi RSSI", "wifi", "dBm", "signal_strength", "", "diagnostic");
  publishSensorDiscoveryToHASS("SSID", "SSID", "network-outline", "", "", "", "diagnostic");
  logItI("MQTT: Published sensor discovery messages to Home Assistant. [height, state, uptime, MAC, IP, hostname, wifiRSSI, SSID]");
  publishButtonDiscoveryToHASS("up", "Move Up", "arrow-up", "button");
  publishButtonDiscoveryToHASS("down", "Move Down", "arrow-down", "button");
  publishButtonDiscoveryToHASS("stop", "Stop", "stop", "button");
  logItI("MQTT: Published switch discovery messages to Home Assistant. [up, down, stop]");
}

void clearDiscoveryAll() {
  String hass_topic;

  String sensor_topics[] = {"height", "state", "uptime", "MAC", "IP", "hostname", "wifiRSSI", "SSID"};
  String button_topics[] = {"up", "down", "stop"};

  for (unsigned i = 0; i < ARRAY_SIZE(sensor_topics); i++) {
    hass_topic = "homeassistant/sensor/table-" + sensor_topics[i] + "/config";
    mqttClient.publish(hass_topic.c_str(), "");
    logItI("MQTT: Cleared discovery message for %s", hass_topic);
  }

  for (unsigned i = 0; i < ARRAY_SIZE(button_topics); i++) {
    hass_topic = "homeassistant/button/table-" + button_topics[i] + "/config";
    mqttClient.publish(hass_topic.c_str(), "");
    logItI("MQTT: Cleared discovery message for %s", hass_topic);
  }
}

/**
 * @brief Publishes the initial status to Home Assistant
 *
 */
void publishTopicsForHASS()
{
  mqttClient.publish((MQTT_TOPIC + "status").c_str(), "ON");
  mqttClient.publish((MQTT_TOPIC + "MAC").c_str(), WiFi.macAddress().c_str());
  mqttClient.publish((MQTT_TOPIC + "IP").c_str(), WiFi.localIP().toString().c_str());
  mqttClient.publish((MQTT_TOPIC + "hostname").c_str(), HOSTNAME);
  mqttClient.publish((MQTT_TOPIC + "SSID").c_str(), WiFi.SSID().c_str());
  lastPublishHASS = millis();
}

/**
 * @brief Publish status to home assistant every 60 seconds
 * 
 */
void publishTopicsForHASSRepeat()
{
  if (!discoveryPublished) {
    publishDiscovery();
    publishTopicsForHASS();
    discoveryPublished = true;
  }
  if (millis() - lastPublishHASS > 60000) {    
    mqttClient.publish((MQTT_TOPIC + "uptime").c_str(), String(millis()/1000).c_str());
    mqttClient.publish((MQTT_TOPIC + "wifiRSSI").c_str(), String(WiFi.RSSI()).c_str());
    lastPublishHASS = millis();
  }  
}

/**
 * @brief Takes care of publishingt the current height if it hasn't been published in a
 *        certain amount of time
 *
 */
void mqtt_publishHeight()
{
  long now = millis();
  if (publishedHeight != currentHeight && now - lastPublish > 2000)
  {
    lastPublish = now;
    publishedHeight = currentHeight;
    mqttClient.publish((MQTT_TOPIC + "height").c_str(), String(currentHeight).c_str());
  }
}

/**
 * @brief Callback function on receiving a command
 *
 * @param length message length
 * @param topic the topic the message was received on
 * @param messageBytes the message as bytes
 */
void mqtt_callback(char *topic, byte *payload, unsigned int length)
{
  if (length == 0) {
    logItD("MQTT: No message received.");
    return;
  }

  JsonDocument doc;
  String message = String((char *)payload);
  
  DeserializationError error = deserializeJson(doc, payload);
  int height = 0;
  
  if (error) {
    logItE("MQTT: Error parsing JSON: %s", String(error.c_str()));
    return;
  }

  if (doc.containsKey("debug"))
  {
    debug = doc["debug"].as<bool>();
    logItD("MQTT: Debug set to %s.", debug ? "true" : "false");
  }
  
  
  if (doc.containsKey("cmd")) {
    String cmd = doc["cmd"].as<String>();
    if (cmd == "height") {
      if (!doc.containsKey("height")) {
        logItE("ERROR: No height provided!");
        return;
      }
      
      height = doc["height"].as<int>();
      if (isValidHeight(height))
      {
        targetHeight = height;
        setHeight = true;
        logItI("Setting height to %d cm.", height);
      }
      else
      {
        logItE("ERROR: Invalid height: %d ! [min: %d cm, max: %d cm]", height, minHeight, maxHeight);
      }
    }
    else if (cmd == "up") {
      move_table_to_fixed(UP);
      logItD("MQTT: Received up. Current height: %d cm.", currentHeight);
    }
    else if (cmd == "down") {
      move_table_to_fixed(DOWN);
      logItD("MQTT: Received down. Current height: %d cm.", currentHeight);
    }
    else if (cmd == "stop") {
      logItD("MQTT: Received stop. Current height: %d cm.", currentHeight);
      stop_table();
    }
    else if (cmd == "publish_discovery") {
      publishDiscovery();
    }
    else if (cmd == "clear_discovery") {
      clearDiscoveryAll();
    }
    else {
      logItE("MQTT: Unknown command: %s", cmd.c_str());
    }
  }
}

/* Setup: Wifi, OTA, MQTT */

void setup_wifi() {
    WiFi.mode(WIFI_STA);
    WiFi.persistent(false);
    #ifdef WIFICONFIG_H
      WiFi.config(WIFI_IP, WIFI_DNS, WIFI_GATEWAY, WIFI_SUBNET);
    #endif
    WiFi.setAutoReconnect(true);
    WiFi.hostname(HOSTNAME);
    WiFi.begin(WIFI_SSID, WIFI_PSK);
 
    while (WiFi.status() != WL_CONNECTED) {
        delay(100);
    }
    logItI("WiFi: Connected! IP: %s", (WiFi.localIP().toString().c_str()));
}

void setup_OTA() {
  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH)
      type = "sketch";
    else // U_SPIFFS
      type = "filesystem";

    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
    logItI("Start updating %s", type);
  });
  ArduinoOTA.onEnd([]() {
    logItI("End");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    logItI("Progress: %d%%\n", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    logItE("Error[%d]: ", error);
    if (error == OTA_AUTH_ERROR)
      logItE("Auth Failed");
    else if (error == OTA_BEGIN_ERROR)
      logItE("Begin Failed");
    else if (error == OTA_CONNECT_ERROR)
      logItE("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR)
      logItE("Receive Failed");
    else if (error == OTA_END_ERROR)
      logItE("End Failed");
  });
  ArduinoOTA.begin();
}

void setup_mqtt() {
  mqttClient.setServer(MQTT_BROKER, MQTT_PORT);
  mqttClient.setCallback(mqtt_callback);
}

/**
 * @brief Initializes the mqtt connection
 * 
 */
void init_mqtt() {  
  if (!mqttClient.connected()) {
      while (!mqttClient.connected()) {
          if (mqttClient.connect(HOSTNAME,
              MQTT_USER, MQTT_PASS,
              (MQTT_TOPIC + "lastConnected").c_str(),
              0,
              true,
              versionLine)) {
            mqttClient.subscribe((MQTT_TOPIC + "set").c_str());
          }
          delay(100);
          logItI("MQTT: Connected! [%s]", HOSTNAME);
      }
  } else
    mqttClient.loop();
}


void setup() {

  // BTN_UP/DOWN are the physical buttons
  pinMode(BTN_UP, INPUT);
  pinMode(BTN_DOWN, INPUT);
  pinMode(LOGICDATA_RX, INPUT);

  // ASSERT_UP/DOWN are the connections to the motor
  pinMode(ASSERT_UP, OUTPUT);
  pinMode(ASSERT_DOWN, OUTPUT);

  logItI("---------");
  logItI("%s", versionLine);

  setup_wifi();
  setup_OTA();
  setup_mqtt();

  logicDataPin_ISR();
  attachInterrupt(digitalPinToInterrupt(LOGICDATA_RX), logicDataPin_ISR, CHANGE);

  logicData.Begin();

  logItI("---------");

  // we use this just to get an initial height on startup (otherwise height is 0)
  move_table(UP);

  if (debug) {
    // RemoteDebug
    Debug.begin(HOSTNAME);
    Debug.showTime(true);
    Debug.setResetCmdEnabled(false);
    // Debug.setCallBackProjectCmds(callback);
    Debug.setSerialEnabled(true);
  }
}

void loop() {
  // sets global currentHeight and last_signal from logicdata serial
  check_display();

  ArduinoOTA.handle();
  init_mqtt();

  // check the buttons
  for(uint8_t i=0; i < ARRAY_SIZE(btn_pins); ++i) {
    int btn_state = digitalRead(btn_pins[i]);
    if((btn_state == btn_pressed_state) != btn_last_state[i] && millis() - debounce[i] > debounce_time) {
      //change state
      btn_last_state[i] = (btn_state == btn_pressed_state);
      debounce[i] = millis();
      last_signal = debounce[i];

      if(btn_last_state[i]) {
        if(millis() - btn_last_on[i] < double_time) {
          //double press
          logItI("button [%s] press (double)", i == 0 ? "up" : "down");
          move_table_to_fixed(i == 0 ? UP : DOWN);
        } else {
          btn_last_on[i] = debounce[i];
          //single press
          logItI("button [%s] press", i == 0 ? "up" : "down");
          if (setHeight) {
            logItI("Setting height end.");
            setHeight = false;
          }
        }
      }// endif pressed
    }
  }

  move();
  mqtt_publishHeight();
  // Publish status to Home Assistant every 60 seconds  
  publishTopicsForHASSRepeat();
  // RemoteDebug
  if (debug)
    Debug.handle();
}
