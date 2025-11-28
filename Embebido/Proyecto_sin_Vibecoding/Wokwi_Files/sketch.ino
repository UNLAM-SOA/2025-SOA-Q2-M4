#include <ESP32Servo.h> // by Kevin Harrington and John K. Bennett
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

#define TriggerPin 19
#define EchoPin 18
#define ServoPin 5
#define PotentiometerPin 34
#define PotentiometerPin2 35

#define PotThreshold 500
#define DistanceThreshold 20

#define TIMER_INIT 300 //300ms
#define TIMER_LOGS 1000 //1000ms = 1s
#define TIMER_MQTT 50 //50ms

#define WeightThreshold 1000 //1000g

#define TAM_PILA_SERVO 2048
#define TAM_PILA_MQTT 4096
#define TAM_COLA 4 //Tamano de cola xQueue

enum States
{
    INIT = 1,
    SERVO_OPEN = 2,
    SERVO_CLOSE = 3
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
    MQTT_MSG = 8
};

enum MqttMsgIndex
{
    LED_AGUA_OFF = 1,
    LED_AGUA_ON = 2,
    LED_COMIDA_OFF = 3,
    LED_COMIDA_ON = 4,
    LED_SHAKER = 5
};

const char* MqttMsgTy[] = { "LED_AGUA_OFF",
                            "LED_AGUA_ON",
                            "LED_COMIDA_OFF",
                            "LED_COMIDA_ON",
                            "LED_SHAKER" };

int const ServoLowWeightPosition = 135;
int const ServoNormalPosition = 120;
int const mqtt_port = 1883;

// Valores de ejecucion simulado - WiFi-Mqtt
const char* ssid = "Wokwi-GUEST";
const char* password = "";
const char* mqtt_server = "broker.hivemq.com";
const char* topicMqtt = "AviFeeder/Datos";

float const calibration_factor = 420.0;

int currentState = 0;
int currentEvent = 0;
int potValue = 0;
int objectTime = 0;
int objectDistance = 0;
int servoAngle = 0;
int waterLed = 0;
int ultraLed = 0;
int ServoChk = 0;

float weight = 0.0;

char MqttMessage[20];

unsigned long timeSinceBoot;
unsigned long timeMqttLoop;

Servo servo1;

WiFiClient espClient;
PubSubClient client(espClient);

static TaskHandle_t ServoHandler = NULL;
static TaskHandle_t MqttHandler = NULL;
static QueueHandle_t ServoQueue;

/*************************************************************/
/*            Funciones de tareas concurrentes               */
/*************************************************************/

// Se encarga de la reconeccion con Mqtt
void mqttConnection()
{
    if (WiFi.status() == WL_CONNECTED)
    {
        Serial.print("Connecting to MQTT... ");

        if (client.connect("ESP32Client"))
        {
            Serial.println("Connected");
            client.subscribe(topicMqtt);
        }
        else
        {
            Serial.println("Connection Failed");
        }
    }
}

// Se encarga de mantener viva la conexion a Mqtt y recibir mensajes
void mqttLoop()
{
    if ((millis() - timeMqttLoop) >= TIMER_MQTT)
    {
        timeMqttLoop = millis();
        client.loop();
    }
}

// Envio de mensaje Mqtt
void sendMqttLogs(unsigned long* timeLogs)
{
    JsonDocument logs;
    char message[128];

    // Si esta conectado, y se cumplio el tiempo, envia logs
    if (client.connected() && (millis() - *timeLogs) >= TIMER_LOGS)
    {
        *timeLogs = millis();

        // Armo el mensaje de Logs
        logs["TimeLog (s)"] = (int) *timeLogs / 1000;
        logs["Weight (g)"] = weight;
        logs["PotValue"] = potValue;
        logs["Distance (cm)"] = objectDistance;
        logs["State"] = currentState;
        logs["Event"] = currentEvent;
        serializeJson(logs, message);

        // Envia mensaje al topic
        if (client.publish(topicMqtt, message))
        {
            Serial.print("log: ");
            Serial.println(message);
        }
    }
}

// Recuperacion del mensaje recibido (se llama cuando Client.loop de Mqtt recibe algo)
void MqttCallback(char* topic, byte* payload, unsigned int length) 
{
    if (length < sizeof(MqttMessage))
    {
        Serial.print("Mensaje recibido: ");

        memcpy(MqttMessage, payload, sizeof(MqttMessage));
        MqttMessage[length] = '\0';
        Serial.println(MqttMessage);
    }
}

/*************************************************************/
/*                   Tareas concurrentes                     */
/*************************************************************/

// Servo se mueve si el peso baja de 1 kg (empieza a servir comida)
// Servo vuelve a su posición si el peso sube de 1 kg (deja de servir comida)
static void concurrentServoTask(void *parameters) 
{
    int value;

    while(xQueueReceive(ServoQueue, &value, portMAX_DELAY) == pdPASS)
    {   
        // Resuelve tarea del servo
        servo1.write(value);
    }
}

// Se ocupa de conectar a Mqtt, reconectar en caso de caida y enviar logs
static void concurrentSendByMqtt(void *parameters)
{
    unsigned long timeLogs = timeSinceBoot;

    // Primera conexion de seteo
    mqttConnection();

    // Loop de conexion y envio de datos
    while(1)
    {
        if (WiFi.status() == WL_CONNECTED)
        {
            mqttLoop();
            
            if (!client.connected())
            {
                Serial.println("MQTT Connection Lost.. ");

                // Loop hasta que estemos conectados
                while (!client.connected())
                {
                    mqttConnection();
                }
            }

            // Si esta conectado, envia logs
            sendMqttLogs(&timeLogs);
        }
    }
}

/*************************************************************/
/*                   Lectura de sensores                     */
/*************************************************************/

// Lectura del sensor de ultrasonido
long readUltrasonicSensor()
{
    int distance = 0;

    digitalWrite(TriggerPin, LOW);
    delayMicroseconds(2);

    digitalWrite(TriggerPin, HIGH);
    delayMicroseconds(10);

    digitalWrite(TriggerPin, LOW);

    objectTime = pulseIn(EchoPin, HIGH);
    distance = 0.01723 * objectTime;
    
    return distance;
}

/*************************************************************/
/*                       Get de eventos                      */
/*************************************************************/

// Get de evento de Agua si se cumple la condicion
void getWater()
{
    if (waterLed && potValue >= PotThreshold)
    {
        currentEvent = WATER_OK;
    }
    else if (!waterLed && potValue < PotThreshold)
    {
        currentEvent = NO_WATER;
    }
}

// Get de evento Ultrasonido si se cumple la condicion
void getUltra()
{
    if (ultraLed && objectDistance < DistanceThreshold)
    {
        currentEvent = ULTRA_NEARBY;
    }
    else if (!ultraLed && objectDistance >= DistanceThreshold)
    {
        currentEvent = ULTRA_FAR;
    }
}

// Get de evento Servo si se cumple la condicion
void getServo()
{
    if (ServoChk && weight >= WeightThreshold)
    {
        currentEvent = FOOD_OK;
    }
    else if (!ServoChk && weight < WeightThreshold)
    {
        currentEvent = NO_FOOD;
    }
}

// Get de evento Mqtt si se recibio mensaje
void getMqtt()
{
    if (*MqttMessage != '\0')
    {
        currentEvent = MQTT_MSG;
    }
}

// Evaluacion de eventos
void getEvent()
{
    // Lectura de sensores
    potValue = analogRead(PotentiometerPin); // Agua
    weight = analogRead(PotentiometerPin2); // Peso
    objectDistance = readUltrasonicSensor(); // Distancia

    // Evento por defecto
    currentEvent = NO_EVENT;

    // Evaluo eventos segun sensor de agua
    getWater();

    // Evaluo eventos segun sensor de ultrasonido
    getUltra();

    // Evaluo eventos segun potenciometro
    getServo();

    // Evaluo eventos segun Mqtt
    getMqtt();
}

/*************************************************************/
/*                   Resolucion de eventos                   */
/*************************************************************/

// Resolucion de evento de Comida
void foodEvent(int State, int servoValue)
{
    if (xQueueSend(ServoQueue, &servoValue, portMAX_DELAY) == pdPASS)
    {
        currentState = State;
    }
    else
    {
        Serial.println("xQueue full!");
    }
}

// Accion de Shake Android
void ledShaker()
{
    bool ledValue;
    int counter = 0;

    // Parpadeo de respuesta
    while (counter++ < 6)                 
    {
        ledValue = !ledValue;
        if (ledValue)
        {
            ledcWrite(LedPinWater, ledHigh);
            ledcWrite(LedPinFood, ledHigh);
        }
        else
        {
            ledcWrite(LedPinWater, ledLow);
            ledcWrite(LedPinFood, ledLow);
        }

        vTaskDelay(TIMER_INIT);
    }

    // Reseteo de senseores
    waterLed = false;
    ultraLed = false;
}

// Resolucion de evento Mqtt
void mqttEvent()
{
    int msgTy = 0;

    for (int i = 0; i < 5; i++)
    {
        if (!strcmp(MqttMsgTy[i], MqttMessage))
        {
            *MqttMessage = '\0';
            msgTy = i + 1;
            i = 5;
        }
    }

    switch (msgTy)
    {
    case LED_AGUA_OFF:
        ledcWrite(LedPinWater, ledLow);
        break;
    
    case LED_AGUA_ON:
        ledcWrite(LedPinWater, ledHigh);
        break;

    case LED_COMIDA_OFF:
        ledcWrite(LedPinFood, ledLow);
        break;

    case LED_COMIDA_ON:
        ledcWrite(LedPinFood, ledHigh);
        break;

    case LED_SHAKER:
        ledShaker();
        break;

    default:
        break;
    }
}

// Resolucion de eventos detectados
void solveEvent()
{
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
            foodEvent(SERVO_CLOSE, servoAngle);
            break;
        
        case FOOD_OK:
            ServoChk = false;
            foodEvent(SERVO_OPEN, servoAngle);
            break;
        
        case MQTT_MSG:
            mqttEvent();
            break;

        default:
            Serial.println("Unknown Event!");
            break;
    }
}

/*************************************************************/
/*                     Manejo de estados                     */
/*************************************************************/

// Señal de evento inicial
void initSignal()
{
    bool ledValue = false;
    int counter = 0;

    Serial.println("System starting...");

    WiFi.begin(ssid, password);
    Serial.print("Connecting to WiFi... ");

    while (WiFi.status() != WL_CONNECTED || counter <= 6) 
    {
        ledValue = !ledValue;

        if (ledValue)
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
    xTaskCreatePinnedToCore(concurrentSendByMqtt,"concurrent_mqtt_task_s",TAM_PILA_MQTT, NULL, 1, &MqttHandler, 1);

    currentState = SERVO_OPEN;
}

// Maquina de estados
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
    
    case SERVO_OPEN:
        // Seteo el angulo del servo para cuando corresponda abrirlo
        servoAngle = ServoLowWeightPosition;
        solveEvent();
        break;
    
    case SERVO_CLOSE:
        // Seteo el angulo del servo para cuando corresponda cerrarlo
        servoAngle = ServoNormalPosition;
        solveEvent();
        break;

    default:
        Serial.println("Unknown State!");
        break;
    }
}

/*************************************************************/
/*                   Funciones de sistema                    */
/*************************************************************/

// Setup inicial de parametros de hardware
void setup()
{
    Serial.begin(9600);

    servo1.attach(ServoPin, 600, 2400);
    servo1.write(ServoNormalPosition);

    timeSinceBoot = timeMqttLoop = millis();
    currentState = INIT;
    pinMode(TriggerPin, OUTPUT);
    pinMode(EchoPin, INPUT);
    ledcAttachChannel(LedPinWater, ledFrequency, ledResolution, LedPinWaterChannel);
    ledcAttachChannel(LedPinFood, ledFrequency, ledResolution, LedPinFoodChannel);

    ServoQueue = xQueueCreate(TAM_COLA, sizeof(int));
    xTaskCreatePinnedToCore(concurrentServoTask,"concurrent_servo_task",TAM_PILA_SERVO, NULL, 0, &ServoHandler, 0);

    client.setServer(mqtt_server, mqtt_port);
    client.setCallback(MqttCallback);
}

// Hilo principal de ejecucion
void loop()
{    
    // Reviso mensajes de Mqtt
    if (WiFi.status() == WL_CONNECTED)
    {
        mqttLoop();
    }

    stateMachine();
}
