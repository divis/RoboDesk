/**
 * @Author: Lukas Prokop
 * @Date:   2024-03-04 11:01:01
 * @Last Modified by:   Lukas Prokop
 * @Last Modified time: 2024-03-04 20:13:33
 */

#include <pins.h> // rename pins.h.example and adjust pins
#include <LogicData.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ArduinoOTA.h>
#include <Credentials.h> // rename Credential.h.example and adjust variables
#include <Logging.h>

uint8_t highTarget = 125;
uint8_t lowTarget = 88;
uint8_t maxHeight = 128; //maxHeight table = 128, but you may set a custom min
uint8_t minHeight = 78; //minHeight table = 62, but you may set a custom min

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
uint8_t publishedHeight = 0;

const char* versionLine = "Robodesk v3.0  build: " __DATE__ " " __TIME__;
LogicData logicData(-1);
WiFiClient espClient;
PubSubClient mqttClient(espClient);

uint8_t currentHeight;
uint8_t targetHeight;
bool setHeight = false;
enum Directions { UP, DOWN, STOPPED };
Directions direction = STOPPED;
bool mqttLog = false;

#pragma region Helpers

template <class T>
/**
 * @brief Converts anything to char *
 * 
 * @param value 
 * @return char* 
 */
char * convertToChar (T value) {
    String valueString = String(value);
    char * output = new char[valueString.length() +1];
    strcpy(output,valueString.c_str());
    return output;
}
int convertCharToInt (char* value, int length) {
  value[length] = '\0';
  return atoi(value);
}

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

#pragma endregion

#pragma region Logicdata related

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
    Log.Debug("%s" CR, buf);
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

#pragma endregion

#pragma region Table Movement

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
    Log.Error("Non valid height [%d] received. Stopping table." CR, currentHeight);
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

    Log.Info("Start setting height. %s target: %d cm. Current height: %d cm." CR,
              highLowTarget == UP ? "High" : "Low",
              targetHeight,
              currentHeight);
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
    Log.Debug("Both buttons pressed" CR);
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
      Log.Info("button [%s] press stopped. Current height: %d cm" CR, direction == UP ? "up" : "down", currentHeight);
      stop_table();
    }
    return;
  }

  if((millis() - last_signal > signal_giveup_time) && !setHeight) {
    Log.Error("Haven't seen input in a while, turning everything off for safety" CR);
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
    Log.Info("Hit target height: %d cm" CR, targetHeight);
    stop_table();
    return;
  }
}

#pragma endregion

#pragma region MQTT functions

/**
 * @brief Callback function on receiving a command
 * 
 * @param message is checked to correspond to prexisting commands
 */
void mqtt_callCmd(String message) {
    if (message == "debug") {
      if (mqttLog == false)
        mqttLog = true;
      else
        mqttLog = false;
      Log.Debug("%s MQTT Logging" CR, mqttLog ? "Activated" : "Deactivated");
      //TODO: Here be dragons - actually implement mqtt logging
    } else if (message == "up") {
        move_table_to_fixed(UP);
    } else if (message == "down") {
        move_table_to_fixed(DOWN);
    } else if (message == "stop") {
        Log.Info("MQTT: Received stop. Current height: %d cm" CR, currentHeight);
        stop_table();
    } else if (message == "ping") {
        // we do want some kind of test message to see if things work
        Log.Debug("MQTT: pong. Current height: %d cm" CR, currentHeight);
        mqttClient.publish((MQTT_TOPIC + "cmd").c_str(), "pong");
    }
}

/**
 * @brief Callback function on receiving a height
 * 
 * @param message must be an int, needs to be within height range
 * @param length message length
 */
void mqtt_callSet(byte* message, int length) {
    int height_in = convertCharToInt((char* )message, length);
    if (isValidHeight(height_in)) {
      targetHeight = height_in;
      setHeight = true;
    } else {
      Log.Error("Invalid height: %d! [min: %d cm, max: %d cm]" CR, height_in, minHeight, maxHeight);
    }
    
    Log.Info("Setting height. Target: %d cm. Current height: %d cm" CR, targetHeight, currentHeight);
}

/**
 * @brief Default MQTT callback function returning everything that is sent to subscribed topics
 *        also takes care of sending messages to dedicated callback functions if necessary
 * 
 * @param topic subscribed topic the message was received on
 * @param message 
 * @param length message length
 */
void mqtt_callback(char* topic, byte* message, unsigned int length) {
  Log.Debug("MQTT: Topic: %s. Message [%d]: ", topic, length);
  String messageTemp;
  
  for (unsigned int i = 0; i < length; i++) {
    Log.Debug("%C", message[i]);
    messageTemp += (char)message[i];
  }
  Log.Debug(CR);

  if ((String) topic == MQTT_TOPIC + "set") {
    mqtt_callSet(message, length);
  }

  if ((String) topic == MQTT_TOPIC + "cmd") {
    mqtt_callCmd(messageTemp);
  }
}

/**
 * @brief Takes care of publishingt the current height if it hasn't been published in a
 *        certain amount of time
 * 
 */
void mqtt_publishHeight() {
    long now = millis();
    if (publishedHeight != currentHeight && now - lastPublish > 2000)
    {
        lastPublish = now;
        publishedHeight = currentHeight;
        mqttClient.publish((MQTT_TOPIC + "height").c_str(), convertToChar(currentHeight));
    }
}

#pragma endregion

#pragma region Setup: Wifi, OTA, MQTT

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
    Log.Info("WiFi: Connected! IP: %s" CR, (WiFi.localIP().toString().c_str()));
}

void setup_OTA() {
  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH)
      type = "sketch";
    else // U_SPIFFS
      type = "filesystem";

    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
    Log.Info("Start updating %s" CR, type);
  });
  ArduinoOTA.onEnd([]() {
    Log.Info(CR "End" CR);
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Log.Info("Progress: %d%%\n", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Log.Error("Error[%d]: ", error);
    if (error == OTA_AUTH_ERROR)
      Log.Error("Auth Failed" CR);
    else if (error == OTA_BEGIN_ERROR)
      Log.Error("Begin Failed" CR);
    else if (error == OTA_CONNECT_ERROR)
      Log.Error("Connect Failed" CR);
    else if (error == OTA_RECEIVE_ERROR)
      Log.Error("Receive Failed" CR);
    else if (error == OTA_END_ERROR)
      Log.Error("End Failed" CR);
  });
  ArduinoOTA.begin();
}

void setup_mqtt() {
  mqttClient.setServer(MQTT_BROKER, MQTT_PORT);
  mqttClient.setCallback(mqtt_callback);
}

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
            mqttClient.subscribe((MQTT_TOPIC + "cmd").c_str());
          }
          delay(100);
      }
  } else
    mqttClient.loop();
}

#pragma endregion

void setup() {

  // BTN_UP/DOWN are the physical buttons
  pinMode(BTN_UP, INPUT);
  pinMode(BTN_DOWN, INPUT);
  pinMode(LOGICDATA_RX, INPUT);

  // ASSERT_UP/DOWN are the connections to the motor
  pinMode(ASSERT_UP, OUTPUT);
  pinMode(ASSERT_DOWN, OUTPUT);

  Log.Init(LOG_LEVEL_DEBUG, 115200L);
  Log.Info(CR "---------" CR);
  Log.Info("%s" CR, versionLine);

  setup_wifi();
  setup_OTA();
  setup_mqtt();

  logicDataPin_ISR();
  attachInterrupt(digitalPinToInterrupt(LOGICDATA_RX), logicDataPin_ISR, CHANGE);

  logicData.Begin();

  Log.Info("---------" CR);

  // we use this just to get an initial height on startup (otherwise height is 0)
  move_table(UP);
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
          Log.Info("button [%s] press (double)" CR, i == 0 ? "up" : "down");
          mqttClient.publish((MQTT_TOPIC + "button").c_str(), i == 0 ? "double up" : "double down");
          move_table_to_fixed(i == 0 ? UP : DOWN);
        } else {
          btn_last_on[i] = debounce[i];
          //single press
          Log.Info("button [%s] press" CR, i == 0 ? "up" : "down");
          mqttClient.publish((MQTT_TOPIC + "button").c_str(), i == 0 ? "single up" : "single down");
          if (setHeight) {
            Log.Info("Setting height end." CR);
            setHeight = false;
          }
        }
      }// endif pressed
    }
  }

  move();
  mqtt_publishHeight();
}
