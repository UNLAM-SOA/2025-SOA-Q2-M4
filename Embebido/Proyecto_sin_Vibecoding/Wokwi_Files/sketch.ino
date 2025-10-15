#include <ESP32Servo.h> // by Kevin Harrington and John K. Bennett
#include <HX711.h> // by Rob Tillaart <rob.tillaart@gmail.com
#include <freertos/FreeRTOS.h> // native from arduino
#include <freertos/task.h> // native from arduino
#include <WiFi.h> // native from arduino
#include "PubSubClient.h" //by Nick O' Leary <nick.oleeary@gmail.com>
#include "ArduinoJson.h" // by benoit blanchon <blog.benoitblanchon.fr>

#define LedPinWater 22 // LedPinWater se enciende si falta agua (potenciometro)
#define LedPinFood 23 // LedPinFood se enciende si falta comida (ultrasonido)
#define ledResolution 8
#define ledFrequency 5000
#define ledHigh 255
#define ledLow 0
#define LedPinWaterChannel 6
#define LedPinFoodChannel 7

#define LoadCellDTPin 17
#define LoadCellSCKPin 16
#define TriggerPin 19
#define EchoPin 18
#define ServoPin 5
#define PotentiometerPin 34

#define PotThreshold 2048
#define DistanceThreshold 20

#define TIMER_CELL 1000 //1000ms = 1s
#define TIMER_INIT 300 //300ms
#define TIMER_LOGS 1500 //1500ms

#define WeightThreshold 2 //120g

#define TAM_PILA_SERVO 2048
#define TAM_PILA_MQTT 4096
#define TAM_COLA 4 //Tamano de cola xQueue

#define SECRET_SSID "Wokwi-GUEST" //"SO Avanzados"
#define SECRET_PSW "" //"SOA.2019"

enum States
{
    INIT = 1,
    SENSING = 2,
    LOAD_CELL = 3,
};

enum Events
{
    FOOD_OK = 1,
    NO_FOOD = 2,
    NO_WATER = 3,
    WATER_OK = 4,
    ULTRA_NEARBY = 5,
    ULTRA_FAR = 6,
    NO_EVENT = 7,
};

int const ServoLowWeightPosition = 180;
int const ServoNormalPosition = 0;
int const mqtt_port = 1883;

const char* ssid = SECRET_SSID;
const char* password = SECRET_PSW;
const char* mqtt_server = "broker.hivemq.com";

float const calibration_factor = 420.0;

int currentState = 0;
int currentEvent = 0;

int potValue = 0;
int objectTime = 0;
int objectDistance = 0;

float weight = 0.0;

int servoAngle = 0;

int waterLed = 0;
int ultraLed = 0;
int ServoChk = 0;

unsigned long timeSinceBoot;
unsigned long timeCell;

Servo servo1;
HX711 loadCell;

WiFiClient espClient;
PubSubClient client(espClient);

static TaskHandle_t ServoHandler = NULL;
static TaskHandle_t MqttHandler = NULL;
static QueueHandle_t ServoQueue;

// Servo se mueve si el peso baja de 1 kg (empieza a servir comida)
// Servo vuelve a su posición si el peso sube de 1 kg (deja de servir comida)
static void concurrentServoTask(void *parameters) 
{
    int value;

    while(1)
    {   
        // Cuando recive algo en al cola, ejecuta
        if (xQueueReceive(ServoQueue, &value, portMAX_DELAY) == pdPASS)
        {               
            // Resuelve tarea del servo
            servo1.write(value);
        }
    }
}

static void concurrentSendByMqtt(void *parameters)
{
    JsonDocument logs;
    char message[128];
    int value;

    while(1)
    {
        if (WiFi.status() == WL_CONNECTED)
        {       
            switch(client.loop())
            {
                case true:
                    // Armo el mensaje de Logs
                    logs["Weight (g)"] = weight;
                    logs["PotValue"] = potValue;
                    logs["Distance (cm)"] = objectDistance;
                    logs["State"] = currentState;
                    logs["Event"] = currentEvent;
                    serializeJson(logs, message);

                    // Envia mensaje al topic
                    client.publish("Wokwi/test", message);
                    Serial.println(message);
                    break;

                case false:
                    // Loop hasta que estemos conectados
                    while (!client.connected())
                    {
                        Serial.print("Connecting to MQTT... ");

                        // Intentamos conectar
                        if (client.connect("ESP32Client", NULL, "Wokwi/test")) 
                        {
                            Serial.println("Connected");
                        }
                        else
                        {
                            Serial.println("Failed, retring in 1 second");
                        }
                    }
                    break;
            }
        }

        vTaskDelay(TIMER_LOGS);
    }
}

/*
bool timestampEnabler(unsigned long* lastTimestamp)
{
    if ((millis() - *lastTimestamp) >= TIMER_LOGS)
    {
        *lastTimestamp = millis();
        return true;
    }
    return false;
}
*/

long readUltrasonicSensor()
{
    digitalWrite(TriggerPin, LOW);
    delayMicroseconds(2);

    digitalWrite(TriggerPin, HIGH);
    delayMicroseconds(10);

    digitalWrite(TriggerPin, LOW);

    return pulseIn(EchoPin, HIGH);
}

void readLoadCell()
{
    timeCell = millis();

    if (loadCell.is_ready())
    {
        weight = loadCell.get_units(5);
        if (weight < 0)
        {
            weight = 0;
        }
    }
}

void performCalculations()
{
    objectDistance = 0.01723 * objectTime;
}

void callback(char* topic, byte* message, unsigned int length) 
{
    //Do nothing
}

void initSignal()
{
    bool ledValue = false;
    int counter = 0;

    Serial.println("System starting...");

    WiFi.begin(ssid, password);
    Serial.print("Connecting to WiFi... ");

    while (WiFi.status() != WL_CONNECTED && counter <= 6) 
    {
        ledValue = !ledValue;

        if (ledValue == true)
        {
            ledcWrite(LedPinWater, ledHigh);
            ledcWrite(LedPinFood, ledHigh);
        }
        else
        {
            ledcWrite(LedPinWater, ledLow);
            ledcWrite(LedPinFood, ledLow);
        }

        counter++;
        vTaskDelay(TIMER_INIT);
    }

    Serial.println("Connected");

    // Apago los leds antes de finalizar el estado
    ledcWrite(LedPinWater, ledLow);
    ledcWrite(LedPinFood, ledLow);

    // Creo la tarea de MqTT aca porque necesita internet
    xTaskCreate(concurrentSendByMqtt,"concurrent_mqtt_task",TAM_PILA_MQTT, NULL, 1, &MqttHandler);
    
    currentState = SENSING;
}

void getEvent()
{
    // Evento por defecto
    currentEvent = NO_EVENT;

    // Evaluo eventos segun sensor de agua
    switch (waterLed)
    {
        case true:
            if (potValue >= PotThreshold)
            {
                currentEvent = WATER_OK;
            }
            break;

        case false:
            if (potValue < PotThreshold)
            {
                currentEvent = NO_WATER;
            }
            break;
    }

    // Evaluo eventos segun sensor de ultrasonido
    switch (ultraLed)
    {
        case true:
            if (objectDistance < DistanceThreshold)
            {
                currentEvent = ULTRA_NEARBY;
            }
            break;

        case false:
            if (objectDistance >= DistanceThreshold)
            {
                currentEvent = ULTRA_FAR;
            }
            break;
    }

    // Evaluo eventos segun celular de carga
    switch (ServoChk)
    {
        case true:
            if (weight >= WeightThreshold)
            {
                currentEvent = FOOD_OK;
            }
            break;

        case false:
            if (weight < WeightThreshold)
            {
                currentEvent = NO_FOOD;
            }
            break;
    }
}

void SolveEvent()
{
    // Manejo local de valores
    int servoValue = servoAngle;

    switch (currentEvent)
    {
        case NO_EVENT:
            // Do nothing
            break;

        case NO_WATER:
            waterLed = true;
            ledcWrite(LedPinWater, ledHigh);
            break;

        case WATER_OK:
            waterLed = false;
            ledcWrite(LedPinWater, ledLow);
            break;

        case ULTRA_FAR:
            ultraLed = true;
            ledcWrite(LedPinFood, ledHigh);
            break;

        case ULTRA_NEARBY:
            ultraLed = false;
            ledcWrite(LedPinFood, ledLow);
            break;

        case NO_FOOD:
            ServoChk = true;
            if (xQueueSend(ServoQueue, &servoValue, portMAX_DELAY) == pdPASS)
            {
                currentState = LOAD_CELL;
            }
            else
            {
                Serial.println("xQueue full!");
            }
            break;
        
        case FOOD_OK:
            ServoChk = false;
            if (xQueueSend(ServoQueue, &servoValue, portMAX_DELAY) == pdPASS)
            {
                currentState = SENSING;
            }
            else
            {
                Serial.println("xQueue full!");
            }
            break;
            
        default:
            Serial.println("Unknown Event!");
            break;
    }
}

void stateMachine()
{   
    // Obtencion del evento actual en base a sensores
    getEvent();
    
    switch (currentState)
    {
    case INIT:
        // Parpadeo de inico
        initSignal();
        break;
    
    case SENSING:
        // Seteo el angulo del servo para cuando corresponda abrirlo
        servoAngle = ServoLowWeightPosition;
        SolveEvent();
        break;
    
    case LOAD_CELL:
        // Seteo el angulo del servo para cuando corresponda cerrarlo
        servoAngle = ServoNormalPosition;
        SolveEvent();
        break;

    default:
        Serial.println("Unknown State!");
        break;
    }
}

void setup()
{
    Serial.begin(9600);

    servo1.attach(ServoPin, 600, 2400);
    servo1.write(ServoNormalPosition);

    loadCell.begin(LoadCellDTPin, LoadCellSCKPin);
    Serial.println("Load cell initializing...");

    while (!loadCell.is_ready()) {
        Serial.println("Waiting for load cell...");
        delay(TIMER_CELL);
    }

    loadCell.set_scale(calibration_factor);

    //Serial.println("Remove all weight from the load cell and press any key to tare...");
    //while (!Serial.available()) { }
    //Serial.read();
    loadCell.tare(10);

    Serial.println("Load cell tared and ready!");

    Serial.print("Current calibration factor: ");
    Serial.println(calibration_factor);

    timeSinceBoot = timeCell = millis();
    currentState = INIT;
    pinMode(TriggerPin, OUTPUT);
    pinMode(EchoPin, INPUT);
    ledcAttachChannel(LedPinWater, ledFrequency, ledResolution, LedPinWaterChannel);
    ledcAttachChannel(LedPinFood, ledFrequency, ledResolution, LedPinFoodChannel);

    ServoQueue = xQueueCreate(TAM_COLA, sizeof(int));
    xTaskCreate(concurrentServoTask,"concurrent_servo_task",TAM_PILA_SERVO, NULL, 1, &ServoHandler);

    client.setServer(mqtt_server, mqtt_port);
    client.setCallback(callback);
}

void loop()
{
    // Lectura de sensores
    potValue = analogRead(PotentiometerPin);
    objectTime = readUltrasonicSensor();
    
    if ((millis() - timeCell) >= TIMER_CELL)
    {   
        readLoadCell();
    }
    
    // Ejecucion de calculos en base a sensores
    performCalculations();
    
    stateMachine();
}