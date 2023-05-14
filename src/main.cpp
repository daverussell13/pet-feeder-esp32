#include <Arduino.h>
#include <ESP32Servo.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <PubSubClient.h>
#include <NTPClient.h>
#include <ArduinoJson.h>
#include <vector>
#include "secrets.h"

// Task init
struct FeedSchedule {
  String day;
  uint8_t hour;
  uint8_t minute;
  uint8_t feedAmount;
};

std::vector<FeedSchedule> schedules;

String prevTime = "";

void checkSchedule();

// Servo init
#define SERVO_PIN 18

Servo servo;

void feed(int feed_multiplier);

// NTP init
#define UTC_OFFSET_IN_SECONDS 3600 * 7

char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", UTC_OFFSET_IN_SECONDS);

void checkResyncTime();
String getTimeStampString();
String getTimeString();
String getDay();
uint8_t getHour();
uint8_t getMinute();

// Wifi init
boolean isConnected = false;

String translateEncryptionType(wifi_auth_mode_t encryptionType);
void scanNetworks();
void connectToNetwork();
void checkWifiStatus();

// MQTT init
#define MQTT_RECONNECT_LOOP_INTERVAL 40

const uint16_t mqtt_port = 1883;

const String feed_topic = "damskuy/petfeeder/feed/" + device_id;
const String schedule_topic = "damskuy/petfeeder/schedule/" + device_id;

WiFiClient espClient;
PubSubClient pubSubClient(espClient);
int reconnectSignal = 0;

void mqttCallback(char* topic, byte* message, unsigned int length);
void reconnectToMqttBroker();
void publishAcknowledgement(String topic, String message);

// Setup function
void setup() {
  Serial.begin(115200);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  servo.attach(SERVO_PIN);
  servo.write(0);

  scanNetworks();
  connectToNetwork();

  pubSubClient.setServer(mqtt_host, mqtt_port);
  pubSubClient.setCallback(mqttCallback);
}

// Loop function
void loop() {
  checkWifiStatus();
  if (isConnected) {
    if (!pubSubClient.connected()) {
      digitalWrite(LED_BUILTIN, LOW);
      if (reconnectSignal <= 0) reconnectToMqttBroker();
      else reconnectSignal--;
    } else {
      pubSubClient.loop();
    }
    checkSchedule();
  }
  delay(250);
}

// Feeder functions
void checkSchedule() {
  String currentTime = getTimeString();
  if (!(prevTime == currentTime)) {
    prevTime = currentTime;
    for (FeedSchedule schedule : schedules) {
      if (schedule.day == getDay() &&
          schedule.hour == getHour() &&
          schedule.minute == getMinute()) {
        feed(schedule.feedAmount);
      }
    }
  }
}

void feed(int feed_multiplier) {
  Serial.println("Feeded at : " + getTimeStampString());
  for (int i = 0; i < feed_multiplier; i++) {
    Serial.println("Opening...");
    servo.write(90);
    delay(800);
    Serial.println("Closing...");
    servo.write(0);
    delay(800);
  }
}

// NTP functions
String getTimeStampString() {
  time_t rawtime = timeClient.getEpochTime();
  struct tm * ti;
  ti = localtime (&rawtime);

  uint16_t year = ti->tm_year + 1900;
  String yearStr = String(year);

  uint8_t month = ti->tm_mon + 1;
  String monthStr = month < 10 ? "0" + String(month) : String(month);

  uint8_t day = ti->tm_mday;
  String dayStr = day < 10 ? "0" + String(day) : String(day);

  uint8_t hours = ti->tm_hour;
  String hoursStr = hours < 10 ? "0" + String(hours) : String(hours);

  uint8_t minutes = ti->tm_min;
  String minuteStr = minutes < 10 ? "0" + String(minutes) : String(minutes);

  uint8_t seconds = ti->tm_sec;
  String secondStr = seconds < 10 ? "0" + String(seconds) : String(seconds);

  return yearStr + "-" + monthStr + "-" + dayStr + " " +
    hoursStr + ":" + minuteStr + ":" + secondStr;
}

String getTimeString() {
  return getDay() + ", " + String(getHour()) + ":" + String(getMinute());
}

uint8_t getMinute() {
  time_t rawtime = timeClient.getEpochTime();
  struct tm * ti;
  ti = localtime (&rawtime);
  return ti->tm_min;
}

uint8_t getHour() {
  time_t rawtime = timeClient.getEpochTime();
  struct tm * ti;
  ti = localtime (&rawtime);
  return ti->tm_hour;
}

String getDay() {
  return daysOfTheWeek[timeClient.getDay()];
}

// Wifi functions
String translateEncryptionType(wifi_auth_mode_t encryptionType) {
  switch (encryptionType) {
    case (WIFI_AUTH_OPEN):
      return "Open";
    case (WIFI_AUTH_WEP):
      return "WEP";
    case (WIFI_AUTH_WPA_PSK):
      return "WPA_PSK";
    case (WIFI_AUTH_WPA2_PSK):
      return "WPA2_PSK";
    case (WIFI_AUTH_WPA_WPA2_PSK):
      return "WPA_WPA2_PSK";
    case (WIFI_AUTH_WPA2_ENTERPRISE):
      return "WPA2_ENTERPRISE";
  }
  return "";
}

void scanNetworks() {
  int numberOfNetworks = WiFi.scanNetworks();

  Serial.print("Number of networks found: ");
  Serial.println(numberOfNetworks);

  for (int i = 0; i < numberOfNetworks; i++) {
    Serial.print("Network name: ");
    Serial.println(WiFi.SSID(i));

    Serial.print("Signal strength: ");
    Serial.println(WiFi.RSSI(i));

    Serial.print("MAC address: ");
    Serial.println(WiFi.BSSIDstr(i));

    Serial.print("Encryption type: ");
    String encryptionTypeDescription = translateEncryptionType(WiFi.encryptionType(i));
    Serial.println(encryptionTypeDescription);
    Serial.println("-----------------------");
  }
}

void connectToNetwork() {
  WiFi.begin(ssid, password);
  Serial.println("Establishing connection to WiFi.");
}

void checkWifiStatus() {
  if (WiFi.status() == WL_CONNECTED && !isConnected) {
    Serial.println("Connected to network");
    isConnected = true;
    timeClient.begin();
    timeClient.update();
    timeClient.setUpdateInterval(6 * 60 * 60 * 1000);
    Serial.println("Current Time : " + getTimeStampString());
  }

  if (WiFi.status() != WL_CONNECTED && isConnected) {
    Serial.println("Connection lost retry to connect");
    isConnected = false;
    connectToNetwork();
  }

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println(".");
  }
}

// MQTT functions
void reconnectToMqttBroker() {
  Serial.println("Attempting to connect MQTT broker...");
  if (pubSubClient.connect(mqtt_client_id)) {
    Serial.println("connected");
    pubSubClient.subscribe(feed_topic.c_str());
    pubSubClient.subscribe(schedule_topic.c_str());
    digitalWrite(LED_BUILTIN, HIGH);
  } else {
    Serial.println("Failed connecting to MQTT broker...");
    Serial.println("Retry in 10 seconds...");
    reconnectSignal = MQTT_RECONNECT_LOOP_INTERVAL;
  }
}

void mqttCallback(char* topic, byte* message, unsigned int length) {
  String messageBuffer;
  Serial.println("Message received");

  for (int i = 0; i < length; i++) {
    messageBuffer += (char) message[i];
  }

  String strTopic = String(topic);

  if (strTopic == feed_topic) {
    Serial.println("Topic : " + feed_topic);
    Serial.println("Payload : " + String(messageBuffer));
    publishAcknowledgement(strTopic, "Device " + device_id + " receive the message");

    int feedAmount = messageBuffer.toInt();
    feed(feedAmount);
  }

  if (strTopic == schedule_topic) {
    Serial.println("Topic : " + feed_topic);
    Serial.println("Payload : " + String(messageBuffer));
    publishAcknowledgement(strTopic, "Device receive message");

    StaticJsonDocument<128> doc;
    deserializeJson(doc, message, length);

    String day = doc["day"];
    uint8_t hour = doc["hour"].as<uint8_t>();
    uint8_t minute = doc["minute"].as<uint8_t>();
    uint8_t feedAmount = doc["amount"].as<uint8_t>();

    FeedSchedule newSchedule = {
      .day = day,
      .hour = hour,
      .minute = minute,
      .feedAmount = feedAmount
    };
    schedules.push_back(newSchedule);
  }
}

void publishAcknowledgement(String topic, String message) {
  topic += "/acknowledgement";
  pubSubClient.publish(topic.c_str(), message.c_str());
  Serial.println("Acknowledment sended");
}