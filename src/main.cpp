#include <Arduino.h>
#include <ESP32Servo.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <NTPClient.h>
#include <ArduinoJson.h>
#include <BluetoothSerial.h>
#include <queue>
#include "secrets.h"

#define SERVO_PIN 18
#define WIFI_TIMEOUT_MS 10000
#define MQTT_TIMEOUT_MS 10000
#define UTC_OFFSET_IN_SECONDS 3600 * 7

struct FeedSchedule {
  String day;
  uint8_t hour;
  uint8_t minute;
  uint8_t feedAmount;
};

Servo servo;
WiFiClient espClient;
WiFiUDP ntpUDP;
PubSubClient mqttClient(espClient);
NTPClient timeClient(ntpUDP, "pool.ntp.org", UTC_OFFSET_IN_SECONDS);
// BluetoothSerial SerialBT;
std::vector<FeedSchedule> schedules;
std::queue<int> feedQueue;

bool isConnectedToWifi = false;
bool isConnectedToMQTTBroker = false;
char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

String prevTime = "";

void connectToWifiTask(void* parameters)
{
  while(true)
  {
    if(WiFi.status() == WL_CONNECTED)
    {
      isConnectedToWifi = true;
      vTaskDelay(250 / portTICK_PERIOD_MS);
      continue;
    }

    isConnectedToWifi = false;

    Serial.println("Establishing WiFi connection");
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    unsigned long startAttemptTime = millis();
    while(WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < WIFI_TIMEOUT_MS) {}

    if(WiFi.status() != WL_CONNECTED)
    {
      Serial.println("WiFi connection failed");
      Serial.println("Retry to connect in 10s");
      vTaskDelay(10000 / portTICK_PERIOD_MS);
      continue;
    }

    Serial.println("WiFi Connected");
    timeClient.begin();
    timeClient.update();
    timeClient.setUpdateInterval(1 * 60 * 60 * 1000);
  }
}

void connectToMqttBrokerTask(void* parameters)
{
  while(true)
  {
    if(isConnectedToWifi)
    {
      if(mqttClient.connected())
      {
        isConnectedToMQTTBroker = true;
        mqttClient.loop();
        continue;
      }

      isConnectedToMQTTBroker = false;
      digitalWrite(LED_BUILTIN, LOW);

      Serial.println("Attempting to connect MQTT broker");
      if (mqttClient.connect(MQTT_CLIENT_ID))
      {
        Serial.println("Connected to MQTT broker");
        mqttClient.subscribe(feed_topic.c_str());
        mqttClient.subscribe(schedule_topic.c_str());
        digitalWrite(LED_BUILTIN, HIGH);
      }
      else
      {
        Serial.println("Failed connecting to MQTT broker");
        Serial.println("Retry in 10 seconds");
        vTaskDelay(10000 / portTICK_PERIOD_MS);
      }
    }
    else
    {
      isConnectedToMQTTBroker = false;
      digitalWrite(LED_BUILTIN, LOW);
    }
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}

void feedingTask(void* params) {
  while(true) {
    if (!feedQueue.empty()) {
      int feedAmount = feedQueue.front();
      feedQueue.pop();
      for (int i = 0; i < feedAmount; i++) {
        servo.write(90);
        delay(400);
        servo.write(0);
        delay(400);
      }
    }
    vTaskDelay(250 / portTICK_PERIOD_MS);
  }
}

void publishAcknowledgement(String topic, String message) {
  topic += "/acknowledge";
  mqttClient.publish(topic.c_str(), message.c_str(), false);
  Serial.println("Acknowledment sended");
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
    publishAcknowledgement(strTopic, "Device " + DEVICE_ID + " receive the message");
    int feedAmount = messageBuffer.toInt();
    feedQueue.push(feedAmount);
  }

  if (strTopic == schedule_topic) {
    Serial.println("Topic : " + feed_topic);
    Serial.println("Payload : " + String(messageBuffer));
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

String getTimeString() {
  return getDay() + ", " + String(getHour()) + ":" + String(getMinute());
}

// Setup function
void setup()
{
  Serial.begin(115200);
  // SerialBT.begin(DEVICE_NAME);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  servo.attach(SERVO_PIN);
  servo.write(0);

  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  mqttClient.setCallback(mqttCallback);

  xTaskCreatePinnedToCore(connectToWifiTask, "Connect to WiFi", 4096, NULL, 2, NULL, CONFIG_ARDUINO_RUNNING_CORE);
  xTaskCreate(connectToMqttBrokerTask, "Connect to MQTT broker", 4096, NULL, 1, NULL);
  xTaskCreate(feedingTask, "Feeding task", 1024 * 8, NULL, 3, NULL);
}

void loop()
{
  if (!(prevTime == getTimeString()))
  {
    prevTime = getTimeString();
    for (FeedSchedule schedule : schedules)
    {
      if (schedule.day == getDay() && schedule.hour == getHour() && schedule.minute == getMinute())
      {
        feedQueue.push(schedule.feedAmount);
      }
    }
  }
  delay(250);
}