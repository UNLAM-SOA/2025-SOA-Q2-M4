/*
 * ESP32 Automatic Pet Feeder Firmware
 * Architecture: State Machine with nested switches, FreeRTOS tasks, MQTT connectivity
 * Compatible with ESP32 Core 3.0+
 */

#include <WiFi.h>
#include <PubSubClient.h>
#include <ESP32Servo.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <ArduinoJson.h>

// ==================== PIN DEFINITIONS ====================
const int PIN_LED_WATER = 22;
const int PIN_LED_FOOD = 23;
const int PIN_TRIGGER = 19;
const int PIN_ECHO = 18;
const int PIN_SERVO = 5;
const int PIN_POT_WATER = 34;
const int PIN_POT_WEIGHT = 35;

// ==================== THRESHOLD DEFINITIONS ====================
const int THRESHOLD_WEIGHT_LOW = 1000;
const int THRESHOLD_DISTANCE_FAR = 20;
const int THRESHOLD_WATER_LOW = 500;

// ==================== PWM DEFINITIONS ====================
const int PWM_FREQ = 5000;
const int PWM_RESOLUTION = 8;
const int PWM_CHANNEL_WATER = 6;
const int PWM_CHANNEL_FOOD = 7;
const int PWM_MAX_DUTY = 255;
const int PWM_OFF_DUTY = 0;

// ==================== SERVO DEFINITIONS ====================
const int SERVO_CLOSED_POS = 120;
const int SERVO_OPEN_POS = 135;

// ==================== TIMING DEFINITIONS ====================
const unsigned long SENSOR_READ_INTERVAL_MS = 100;
const unsigned long MQTT_PUBLISH_INTERVAL_MS = 1000;
const unsigned long ULTRASONIC_TIMEOUT_US = 30000;
const int SHAKER_BLINK_COUNT = 6;
const int SHAKER_BLINK_DELAY_MS = 200;

// ==================== WIFI & MQTT DEFINITIONS ====================
const char* WIFI_SSID = "Wokwi-GUEST";
const char* WIFI_PASSWORD = "";
const char* MQTT_BROKER = "broker.hivemq.com";
const int MQTT_PORT = 1883;
const char* MQTT_TOPIC_PUBLISH = "AviFeeder/Datos";
const char* MQTT_TOPIC_SUBSCRIBE = "AviFeeder/Commands";
const char* MQTT_CLIENT_ID = "ESP32_AviFeeder";

// ==================== STATE MACHINE DEFINITIONS ====================
enum State {
  STATE_INIT,
  STATE_IDLE,
  STATE_DISPENSING
};

enum Event {
  EVENT_NONE,
  EVENT_WEIGHT_LOW,
  EVENT_WEIGHT_OK,
  EVENT_WATER_LOW,
  EVENT_WATER_OK,
  EVENT_OBJECT_FAR,
  EVENT_OBJECT_NEAR,
  EVENT_MQTT_LED_WATER_ON,
  EVENT_MQTT_LED_WATER_OFF,
  EVENT_MQTT_LED_FOOD_ON,
  EVENT_MQTT_LED_FOOD_OFF,
  EVENT_MQTT_SHAKER
};

// ==================== GLOBAL VARIABLES ====================
State currentState = STATE_INIT;
Event currentEvent = EVENT_NONE;

// Sensor readings
int potWaterValue = 0;
int potWeightValue = 0;
float distanceCm = 0.0;

// Servo control
Servo dispenserServo;

// WiFi and MQTT clients
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

// FreeRTOS queues
QueueHandle_t servoCommandQueue;

// Timing variables
unsigned long lastSensorRead = 0;
unsigned long lastMqttPublish = 0;
unsigned long systemTimeSeconds = 0;

// ==================== FUNCTION PROTOTYPES ====================
void setupPins();
void setupWiFi();
void setupMQTT();
void getEvent();
void stateMachine();
void handleMqttMessage(char* topic, byte* payload, unsigned int length);
void taskServoControl(void* parameter);
void taskMqttCommunication(void* parameter);
float readUltrasonicDistance();
void publishMqttLog();
void setLedBrightness(int pin, int brightness);
void executeLedShaker();

// ==================== SETUP ====================
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("=== ESP32 Automatic Feeder Starting ===");

  setupPins();
  setupWiFi();
  setupMQTT();

  // Create FreeRTOS queue for servo commands
  servoCommandQueue = xQueueCreate(5, sizeof(int));

  // Create FreeRTOS tasks
  xTaskCreatePinnedToCore(
    taskServoControl,
    "ServoTask",
    4096,
    NULL,
    1,
    NULL,
    0
  );

  xTaskCreatePinnedToCore(
    taskMqttCommunication,
    "MqttTask",
    8192,
    NULL,
    1,
    NULL,
    1
  );

  Serial.println("Setup complete. Entering main loop.");

  
}

// ==================== MAIN LOOP ====================
void loop() {
  unsigned long currentMillis = millis();

  // Read sensors periodically
  if (currentMillis - lastSensorRead >= SENSOR_READ_INTERVAL_MS) {
    lastSensorRead = currentMillis;
    getEvent();
  }

  // Run state machine
  stateMachine();

  // Update system time
  systemTimeSeconds = currentMillis / 1000;

  delay(10);
}

// ==================== PIN SETUP ====================
void setupPins() {
  // Configure LED PWM channels using new API (ESP32 Core 3.0+)
  ledcAttachChannel(PIN_LED_WATER, PWM_FREQ, PWM_RESOLUTION, PWM_CHANNEL_WATER);
  ledcAttachChannel(PIN_LED_FOOD, PWM_FREQ, PWM_RESOLUTION, PWM_CHANNEL_FOOD);
  
  // Initialize LEDs to off
  setLedBrightness(PIN_LED_WATER, PWM_OFF_DUTY);
  setLedBrightness(PIN_LED_FOOD, PWM_OFF_DUTY);

  // Configure ultrasonic sensor pins
  pinMode(PIN_TRIGGER, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  digitalWrite(PIN_TRIGGER, LOW);

  // Configure analog pins (ADC)
  pinMode(PIN_POT_WATER, INPUT);
  pinMode(PIN_POT_WEIGHT, INPUT);

  // Configure servo
  dispenserServo.attach(PIN_SERVO);
  dispenserServo.write(SERVO_CLOSED_POS);

  Serial.println("Pins configured successfully");
}

// ==================== WIFI SETUP ====================
void setupWiFi() {
  Serial.print("Connecting to WiFi: ");
  Serial.println(WIFI_SSID);
  
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  
  int attempts = 0;
  const int maxAttempts = 20;
  
  while (WiFi.status() != WL_CONNECTED && attempts < maxAttempts) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi connected!");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\nWiFi connection failed!");
  }
}

// ==================== MQTT SETUP ====================
void setupMQTT() {
  mqttClient.setServer(MQTT_BROKER, MQTT_PORT);
  mqttClient.setCallback(handleMqttMessage);
  Serial.println("MQTT client configured");
}

// ==================== GET EVENT FUNCTION ====================
void getEvent() {
  // Read all sensors
  potWaterValue = analogRead(PIN_POT_WATER);
  potWeightValue = analogRead(PIN_POT_WEIGHT);
  distanceCm = readUltrasonicDistance();

  // Determine event based on sensor readings
  // Priority: Weight events first
  if (potWeightValue < THRESHOLD_WEIGHT_LOW) {
    currentEvent = EVENT_WEIGHT_LOW;
  } else if (potWeightValue >= THRESHOLD_WEIGHT_LOW && currentState == STATE_DISPENSING) {
    currentEvent = EVENT_WEIGHT_OK;
  } else {
    currentEvent = EVENT_NONE;
  }

  // Check water level for LED control
  if (potWaterValue < THRESHOLD_WATER_LOW) {
    // Water low event is handled in state machine
  }

  // Check distance for LED control
  if (distanceCm > THRESHOLD_DISTANCE_FAR) {
    // Distance far event is handled in state machine
  }
}

// ==================== ULTRASONIC DISTANCE READING ====================
float readUltrasonicDistance() {
  digitalWrite(PIN_TRIGGER, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIGGER, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIGGER, LOW);

  long duration = pulseIn(PIN_ECHO, HIGH, ULTRASONIC_TIMEOUT_US);
  
  if (duration == 0) {
    return 999.9; // Timeout or no echo
  }
  
  float distance = (duration * 0.0343) / 2.0;
  return distance;
}

// ==================== STATE MACHINE ====================
void stateMachine() {
  switch (currentState) {
    // ========== STATE: INIT ==========
    case STATE_INIT: {
      Serial.println("[STATE] INIT");
      
      switch (currentEvent) {
        case EVENT_NONE:
        default: {
          // Initialize system
          setLedBrightness(PIN_LED_WATER, PWM_OFF_DUTY);
          setLedBrightness(PIN_LED_FOOD, PWM_OFF_DUTY);
          
          // Transition to IDLE
          currentState = STATE_IDLE;
          Serial.println("[TRANSITION] INIT -> IDLE");
          break;
        }
      }
      break;
    }

    // ========== STATE: IDLE ==========
    case STATE_IDLE: {
      switch (currentEvent) {
        case EVENT_WEIGHT_LOW: {
          Serial.println("[EVENT] Weight low detected");
          
          // Send command to open dispenser
          int servoPos = SERVO_OPEN_POS;
          xQueueSend(servoCommandQueue, &servoPos, portMAX_DELAY);
          
          // Transition to DISPENSING
          currentState = STATE_DISPENSING;
          Serial.println("[TRANSITION] IDLE -> DISPENSING");
          break;
        }
        
        case EVENT_MQTT_LED_WATER_ON: {
          Serial.println("[EVENT] MQTT command: Water LED ON");
          setLedBrightness(PIN_LED_WATER, PWM_MAX_DUTY);
          currentEvent = EVENT_NONE;
          break;
        }
        
        case EVENT_MQTT_LED_WATER_OFF: {
          Serial.println("[EVENT] MQTT command: Water LED OFF");
          setLedBrightness(PIN_LED_WATER, PWM_OFF_DUTY);
          currentEvent = EVENT_NONE;
          break;
        }
        
        case EVENT_MQTT_LED_FOOD_ON: {
          Serial.println("[EVENT] MQTT command: Food LED ON");
          setLedBrightness(PIN_LED_FOOD, PWM_MAX_DUTY);
          currentEvent = EVENT_NONE;
          break;
        }
        
        case EVENT_MQTT_LED_FOOD_OFF: {
          Serial.println("[EVENT] MQTT command: Food LED OFF");
          setLedBrightness(PIN_LED_FOOD, PWM_OFF_DUTY);
          currentEvent = EVENT_NONE;
          break;
        }
        
        case EVENT_MQTT_SHAKER: {
          Serial.println("[EVENT] MQTT command: LED SHAKER");
          executeLedShaker();
          currentEvent = EVENT_NONE;
          break;
        }
        
        case EVENT_NONE:
        default: {
          // Control LEDs based on sensor readings
          if (potWaterValue < THRESHOLD_WATER_LOW) {
            setLedBrightness(PIN_LED_WATER, PWM_MAX_DUTY);
          } else {
            setLedBrightness(PIN_LED_WATER, PWM_OFF_DUTY);
          }
          
          if (distanceCm > THRESHOLD_DISTANCE_FAR) {
            setLedBrightness(PIN_LED_FOOD, PWM_MAX_DUTY);
          } else {
            setLedBrightness(PIN_LED_FOOD, PWM_OFF_DUTY);
          }
          break;
        }
      }
      break;
    }

    // ========== STATE: DISPENSING ==========
    case STATE_DISPENSING: {
      switch (currentEvent) {
        case EVENT_WEIGHT_OK: {
          Serial.println("[EVENT] Weight OK detected");
          
          // Send command to close dispenser
          int servoPos = SERVO_CLOSED_POS;
          xQueueSend(servoCommandQueue, &servoPos, portMAX_DELAY);
          
          // Transition to IDLE
          currentState = STATE_IDLE;
          Serial.println("[TRANSITION] DISPENSING -> IDLE");
          break;
        }
        
        case EVENT_MQTT_SHAKER: {
          Serial.println("[EVENT] MQTT command: LED SHAKER");
          executeLedShaker();
          currentEvent = EVENT_NONE;
          break;
        }
        
        case EVENT_WEIGHT_LOW:
        case EVENT_NONE:
        default: {
          // Continue dispensing
          // Control LEDs based on sensor readings
          if (potWaterValue < THRESHOLD_WATER_LOW) {
            setLedBrightness(PIN_LED_WATER, PWM_MAX_DUTY);
          } else {
            setLedBrightness(PIN_LED_WATER, PWM_OFF_DUTY);
          }
          
          if (distanceCm > THRESHOLD_DISTANCE_FAR) {
            setLedBrightness(PIN_LED_FOOD, PWM_MAX_DUTY);
          } else {
            setLedBrightness(PIN_LED_FOOD, PWM_OFF_DUTY);
          }
          break;
        }
      }
      break;
    }

    default: {
      Serial.println("[ERROR] Unknown state!");
      currentState = STATE_INIT;
      break;
    }
  }
}

// ==================== LED BRIGHTNESS CONTROL ====================
void setLedBrightness(int pin, int brightness) {
  ledcWrite(pin, brightness);
}

// ==================== LED SHAKER EFFECT ====================
void executeLedShaker() {
  for (int i = 0; i < SHAKER_BLINK_COUNT; i++) {
    setLedBrightness(PIN_LED_WATER, PWM_MAX_DUTY);
    setLedBrightness(PIN_LED_FOOD, PWM_MAX_DUTY);
    delay(SHAKER_BLINK_DELAY_MS);
    
    setLedBrightness(PIN_LED_WATER, PWM_OFF_DUTY);
    setLedBrightness(PIN_LED_FOOD, PWM_OFF_DUTY);
    delay(SHAKER_BLINK_DELAY_MS);
  }
}

// ==================== MQTT MESSAGE HANDLER ====================
void handleMqttMessage(char* topic, byte* payload, unsigned int length) {
  Serial.print("[MQTT] Message received on topic: ");
  Serial.println(topic);
  
  // Convert payload to string
  String message;
  for (unsigned int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  
  Serial.print("[MQTT] Payload: ");
  Serial.println(message);
  
  // Parse commands
  if (message == "LED_WATER_ON") {
    currentEvent = EVENT_MQTT_LED_WATER_ON;
  } else if (message == "LED_WATER_OFF") {
    currentEvent = EVENT_MQTT_LED_WATER_OFF;
  } else if (message == "LED_FOOD_ON") {
    currentEvent = EVENT_MQTT_LED_FOOD_ON;
  } else if (message == "LED_FOOD_OFF") {
    currentEvent = EVENT_MQTT_LED_FOOD_OFF;
  } else if (message == "LED_SHAKER") {
    currentEvent = EVENT_MQTT_SHAKER;
  } else {
    Serial.println("[MQTT] Unknown command");
  }
}

// ==================== MQTT PUBLISH LOG ====================
void publishMqttLog() {
  if (!mqttClient.connected()) {
    return;
  }

  // Create JSON document
  StaticJsonDocument<256> doc;
  
  doc["time_s"] = systemTimeSeconds;
  doc["weight_g"] = potWeightValue;
  doc["potValue"] = potWaterValue;
  doc["distance_cm"] = distanceCm;
  
  // State as string
  String stateStr;
  switch (currentState) {
    case STATE_INIT: stateStr = "INIT"; break;
    case STATE_IDLE: stateStr = "IDLE"; break;
    case STATE_DISPENSING: stateStr = "DISPENSING"; break;
    default: stateStr = "UNKNOWN"; break;
  }
  doc["state"] = stateStr;
  
  // Event as string
  String eventStr;
  switch (currentEvent) {
    case EVENT_NONE: eventStr = "NONE"; break;
    case EVENT_WEIGHT_LOW: eventStr = "WEIGHT_LOW"; break;
    case EVENT_WEIGHT_OK: eventStr = "WEIGHT_OK"; break;
    case EVENT_WATER_LOW: eventStr = "WATER_LOW"; break;
    case EVENT_WATER_OK: eventStr = "WATER_OK"; break;
    case EVENT_OBJECT_FAR: eventStr = "OBJECT_FAR"; break;
    case EVENT_OBJECT_NEAR: eventStr = "OBJECT_NEAR"; break;
    default: eventStr = "MQTT_CMD"; break;
  }
  doc["event"] = eventStr;

  // Serialize and publish
  String jsonString;
  serializeJson(doc, jsonString);
  
  mqttClient.publish(MQTT_TOPIC_PUBLISH, jsonString.c_str());
  
  Serial.print("[MQTT] Published: ");
  Serial.println(jsonString);
}

// ==================== FREERTOS TASK: SERVO CONTROL ====================
void taskServoControl(void* parameter) {
  int servoPosition;
  
  Serial.println("[TASK] Servo control task started");
  
  for (;;) {
    // Wait for servo command from queue
    if (xQueueReceive(servoCommandQueue, &servoPosition, portMAX_DELAY) == pdTRUE) {
      Serial.print("[SERVO] Moving to position: ");
      Serial.println(servoPosition);
      
      dispenserServo.write(servoPosition);
    }
    
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// ==================== FREERTOS TASK: MQTT COMMUNICATION ====================
void taskMqttCommunication(void* parameter) {
  Serial.println("[TASK] MQTT communication task started");
  
  for (;;) {
    // Reconnect to MQTT if disconnected
    if (!mqttClient.connected()) {
      Serial.println("[MQTT] Attempting to connect...");
      
      if (mqttClient.connect(MQTT_CLIENT_ID)) {
        Serial.println("[MQTT] Connected successfully!");
        mqttClient.subscribe(MQTT_TOPIC_SUBSCRIBE);
        Serial.print("[MQTT] Subscribed to: ");
        Serial.println(MQTT_TOPIC_SUBSCRIBE);
      } else {
        Serial.print("[MQTT] Connection failed, rc=");
        Serial.println(mqttClient.state());
        vTaskDelay(pdMS_TO_TICKS(5000));
        continue;
      }
    }
    
    // Process MQTT messages
    mqttClient.loop();
    
    // Publish logs periodically
    unsigned long currentMillis = millis();
    if (currentMillis - lastMqttPublish >= MQTT_PUBLISH_INTERVAL_MS) {
      lastMqttPublish = currentMillis;
      publishMqttLog();
    }
    
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}