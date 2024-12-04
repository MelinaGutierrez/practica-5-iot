#include "secrets.h"
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "WiFi.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h>

// AWS IoT Core Tópicos
#define AWS_IOT_UPDATE_TOPIC "$aws/things/Env_Control_0001/shadow/update"
#define AWS_IOT_DELTA_TOPIC "$aws/things/Env_Control_0001/shadow/update/delta"

#define SDA_PIN 21
#define SCL_PIN 22
#define FAN_PIN1 25
#define FAN_PIN2 32
#define ALARM_PIN 5
#define MOTOR_PWM_PIN 26
#define MOTOR_INA1_PIN 27
#define MOTOR_INA2_PIN 14
#define MOTOR_STBY_PIN 15

#define MQ2_PIN 12

// Botones
#define BUTTON_MONITOR_PIN 17
#define BUTTON_CONVEYOR_PIN 16
#define BUTTON_MANUAL_PIN 18

// Configura el sensor BME680 en el pin I2C
Adafruit_BME680 bme;  // usa el address I2C por defecto (0x77)

WiFiClientSecure net = WiFiClientSecure();
PubSubClient client(net);

// Variables para los actuadores y estado
bool fanStatus = false;
bool alarmStatus = false;
bool motorStatus = false;
bool systemStarted = false;
bool conveyorRunning = false;
bool manualControl = false;
bool reportingActive = false;

float motorSpeed = 0.0;  // Velocidad del motor (esto depende de cómo estés controlando el motor)
float temperature = 0.0;  // Temperatura leída del BME680
float humidity = 0.0;     // Humedad leída del BME680
float airQuality = 0.0;   // Calidad del aire medida por BME680
float lpgValue = 0.0;     // Valor del sensor MQ-2 para LPG
float coValue = 0.0;      // Valor del sensor MQ-2 para CO
float smokeValue = 0.0;   // Valor del sensor MQ-2 para humo
bool airQualityDrop = false; // Para indicar si la calidad del aire ha bajado más del 15%


// Variables de anti-rebote
unsigned long lastMonitorButtonTime = 0;
unsigned long lastConveyorButtonTime = 0;
unsigned long lastManualButtonTime = 0;
const unsigned long debounceDelay = 200; // Tiempo de anti-rebote en milisegundos

// Variables de tiempo
unsigned long lastActionTime = 0;
unsigned long actionInterval = 2000; // 2 segundos para la lectura de sensores

// Variables para la calibración
float baseAirQuality = 0;
bool isCalibrating = false; // La calibración ya no se inicia automáticamente
unsigned long calibrationStartTime = 0;
unsigned long calibrationDuration = 5000; // 5 segundos para calibración

float lastTemperature = 0.0;
float lastHumidity = 0.0;
float lastAirQuality = 0.0;
unsigned long lastReportTime = 0;
const unsigned long reportInterval = 5000; // 5 segundos para pruebas

const float significantChangeThreshold = 0.15; // 15% de cambio en los sensores

// Definir los umbrales para los sensores MQ-2
const float LPG_THRESHOLD = 100;
const float CO_THRESHOLD = 1500;
const float SMOKE_THRESHOLD = 100;

unsigned long previousMillis = 0;  // Almacena el tiempo anterior
const long interval = 5000;  // Intervalo de 5 segundos para reportes

// Funciones de control de actuadores
void activateFan() {
  digitalWrite(FAN_PIN1, HIGH);
  digitalWrite(FAN_PIN2, HIGH);
  Serial.println("Ventiladores activados.");
}

void deactivateFan() {
  digitalWrite(FAN_PIN1, LOW);
  digitalWrite(FAN_PIN2, LOW);
  Serial.println("Ventiladores desactivados.");
}

void activateAlarm() {
  digitalWrite(ALARM_PIN, HIGH);
  Serial.println("Alarma activada.");
}

void deactivateAlarm() {
  digitalWrite(ALARM_PIN, LOW);
  Serial.println("Alarma desactivada.");
}

void activateMotor() {
  digitalWrite(MOTOR_INA1_PIN, HIGH);
  digitalWrite(MOTOR_INA2_PIN, LOW);
  Serial.println("Motor activado.");
}

void deactivateMotor() {
  digitalWrite(MOTOR_INA1_PIN, LOW);
  digitalWrite(MOTOR_INA2_PIN, LOW);
  Serial.println("Motor desactivado.");
}

// Función de calibración
void calibrateAirQuality() {
  if (isCalibrating) {
    // Iniciamos calibración, tomamos el valor base de la calidad de aire
    baseAirQuality = bme.gas_resistance;
    Serial.println("Calibración iniciada...");
    calibrationStartTime = millis();
    isCalibrating = false;  // Detenemos la calibración después de tomar la medición inicial
  } else {
    // Verificamos si ya pasó el tiempo de calibración
    if (millis() - calibrationStartTime >= calibrationDuration) {
      Serial.println("Calibración completada.");
    }
  }
}

// Función para manejar botones con anti-rebote
void handleButtons() {
  unsigned long currentMillis = millis();

  // Botón para iniciar el sistema de monitoreo (encender/apagar reporte)
  if (digitalRead(BUTTON_MONITOR_PIN) == LOW && currentMillis - lastMonitorButtonTime > debounceDelay) {
    reportingActive = !reportingActive;
    if (reportingActive) {
      Serial.println("Sistema de monitoreo activado. Reportando a AWS y localmente.");
      isCalibrating = true;  // Comienza la calibración al iniciar el sistema
      calibrationStartTime = millis();  // Guarda el tiempo de inicio de la calibración
    } else {
      Serial.println("Sistema de monitoreo desactivado. Dejando de reportar.");
      isCalibrating = false;  // Detiene la calibración cuando se apaga el sistema
    }
    lastMonitorButtonTime = currentMillis;
  }

  // Otros botones para controlar la banda transportadora y el control manual
  if (digitalRead(BUTTON_CONVEYOR_PIN) == LOW && currentMillis - lastConveyorButtonTime > debounceDelay) {
    conveyorRunning = !conveyorRunning;
    if (conveyorRunning) {
      activateMotor();
    } else {
      deactivateMotor();
    }
    lastConveyorButtonTime = currentMillis;
  }

  if (digitalRead(BUTTON_MANUAL_PIN) == LOW && currentMillis - lastManualButtonTime > debounceDelay) {
    manualControl = !manualControl;
    if (manualControl) {
      activateFan();
      activateAlarm();
      deactivateMotor();  // Desactivar motor en modo manual
    } else {
      deactivateFan();
      deactivateAlarm();
      activateMotor();  // Reactivar motor al salir del modo manual
    }
    lastManualButtonTime = currentMillis;
  }
}

// Función para verificar si la calidad del aire ha bajado un 15%
bool hasAirQualityReduced() {
  float currentAirQuality = bme.gas_resistance;
  float reduction = (baseAirQuality - currentAirQuality) / baseAirQuality * 100;
  return reduction >= 15; // Si hay una reducción de 15% o más
}


void messageReceived(String &topic, String &payload) {
  Serial.println("Mensaje recibido:");
  Serial.println(topic);
  Serial.println(payload);

  // Parsear el payload JSON para obtener los valores del delta
  StaticJsonDocument<1024> doc;
  DeserializationError error = deserializeJson(doc, payload);

  if (error) {
    Serial.print("Error al parsear JSON: ");
    Serial.println(error.c_str());
    return;
  }

  JsonObject state = doc["state"];
  if (state.containsKey("desired")) {
    JsonObject desired = state["desired"];
    if (desired.containsKey("fan")) {
      fanStatus = desired["fan"].as<int>() == 1;
      if (fanStatus) {
        activateFan();
      } else {
        deactivateFan();
      }
    }
    if (desired.containsKey("alarm")) {
      alarmStatus = desired["alarm"].as<int>() == 1;
      if (alarmStatus) {
        activateAlarm();
      } else {
        deactivateAlarm();
      }
    }
    if (desired.containsKey("motor")) {
      motorStatus = desired["motor"].as<int>() == 1;
      if (motorStatus) {
        activateMotor();
      } else {
        deactivateMotor();
      }
    }
  }
}

// Función para conectar a AWS IoT
void connectAWS() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  Serial.println("Conectando a Wi-Fi");

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("WiFi conectado.");

  net.setCACert(AWS_CERT_CA);
  net.setCertificate(AWS_CERT_CRT);
  net.setPrivateKey(AWS_CERT_PRIVATE);

  client.setServer(AWS_IOT_ENDPOINT, 8883);

  Serial.println("Conectando a AWS IoT");

  while (!client.connected()) {
    String clientId = "ESP32-" + String(random(0xffff), HEX);

    if (client.connect(clientId.c_str())) {
      Serial.println("Conectado a AWS IoT");
    } else {
      Serial.print("Error al conectar: ");
      Serial.println(client.state());
      delay(5000);  // Intentar nuevamente
    }
  }
}

// Función para publicar datos a AWS IoT
void updateShadow() {
  StaticJsonDocument<1024> jsonDoc;
  JsonObject state = jsonDoc.createNestedObject("state");

  JsonObject desired = state.createNestedObject("desired");
  desired["actuators"]["fan"] = fanStatus ? 1 : 0;
  desired["actuators"]["alarm"] = alarmStatus ? 1 : 0;
  desired["actuators"]["motor"] = motorStatus ? 1 : 0;

  JsonObject reported = state.createNestedObject("reported");
  reported["sensors"]["temperature"] = temperature;
  reported["sensors"]["humidity"] = humidity;
  reported["sensors"]["air_quality"] = airQuality;
  reported["sensors"]["mq2"]["lpg"] = lpgValue;
  reported["sensors"]["mq2"]["co"] = coValue;
  reported["sensors"]["mq2"]["smoke"] = smokeValue;

  String payload;
  serializeJson(jsonDoc, payload);

  if (client.publish(AWS_IOT_UPDATE_TOPIC, payload.c_str())) {
    Serial.println("Reporte exitoso a AWS IoT.");
  } else {
    Serial.println("Error al publicar en AWS IoT.");
  }
}

void reconnect() {
  while (!client.connected()) {
    if (client.connect(THINGNAME)) {
      Serial.println("Reconectado a AWS IoT!");
    } else {
      Serial.print("Intentando reconectar...");
      delay(5000);  // Esperar 5 segundos para reintentar
    }
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(FAN_PIN1, OUTPUT);
  pinMode(FAN_PIN2, OUTPUT);
  pinMode(ALARM_PIN, OUTPUT);
  pinMode(MOTOR_PWM_PIN, OUTPUT);
  pinMode(MOTOR_INA1_PIN, OUTPUT);
  pinMode(MOTOR_INA2_PIN, OUTPUT);
  pinMode(MOTOR_STBY_PIN, OUTPUT);

  pinMode(BUTTON_MONITOR_PIN, INPUT_PULLUP);
  pinMode(BUTTON_CONVEYOR_PIN, INPUT_PULLUP);
  pinMode(BUTTON_MANUAL_PIN, INPUT_PULLUP);

  if (!bme.begin()) {
    Serial.println("No se pudo encontrar un sensor BME680.");
    while (1);
  }

  connectAWS();
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  handleButtons();  // Revisa los botones

  // Solo reporta si el sistema está en modo activo
  if (reportingActive) {
    unsigned long currentMillis = millis();

    // Lee los sensores solo si ha pasado el intervalo
    if (currentMillis - lastActionTime >= actionInterval) {
      lastActionTime = currentMillis;

      temperature = bme.readTemperature();
      humidity = bme.readHumidity();
      airQuality = bme.gas_resistance;

      lpgValue = analogRead(MQ2_PIN) * 0.1;
      coValue = analogRead(MQ2_PIN) * 0.08;
      smokeValue = analogRead(MQ2_PIN) * 0.06;

      if (isCalibrating) {
        calibrateAirQuality();
      }

      if (millis() - lastReportTime >= reportInterval) {
        updateShadow();
        lastReportTime = millis();
      }
    }
  }
}