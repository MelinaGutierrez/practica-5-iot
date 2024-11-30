// main.ino
#include "secrets.h"
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h>
#include <ArduinoJson.h>

// Pines y configuración
#define SDA_PIN 21
#define SCL_PIN 22
#define FAN_PIN1 25
#define FAN_PIN2 32
#define ALARM_PIN 5
#define MQ2_PIN 12
#define LED_PIN 2

// AWS IoT Core Tópicos
#define AWS_IOT_UPDATE_TOPIC "$aws/things/Env_Control_0001/shadow/update"
#define AWS_IOT_DELTA_TOPIC "$aws/things/Env_Control_0001/shadow/update/delta"

WiFiClientSecure net;
PubSubClient client(net);
Adafruit_BME680 bme;

// Estados globales
bool isIncidentActive = false;
float baseAirQuality = 0;

// Funciones de conexión
void setupWiFi() {
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi conectado.");
}

void connectAWS() {
  while (!client.connected()) {
    Serial.print("Conectando a AWS IoT...");
    if (client.connect("ESP32_Client", MQTT_USER, MQTT_PASSWORD)) {
      Serial.println("Conectado a AWS IoT!");
      client.subscribe(AWS_IOT_DELTA_TOPIC);
    } else {
      Serial.println("Fallo en la conexión. Reintentando...");
      delay(5000);
    }
  }
}

// Manejador de mensajes MQTT
void messageHandler(char* topic, byte* payload, unsigned int length) {
  StaticJsonDocument<200> doc;
  if (deserializeJson(doc, payload, length) == DeserializationError::Ok) {
    if (strcmp(topic, AWS_IOT_DELTA_TOPIC) == 0 && doc["state"]["fan"].is<bool>()) {
      bool fanCommand = doc["state"]["fan"];
      digitalWrite(FAN_PIN1, fanCommand ? HIGH : LOW);
      Serial.println(fanCommand ? "Ventilador activado." : "Ventilador desactivado.");
    }
  } else {
    Serial.println("Error al procesar mensaje MQTT.");
  }
}

// Publicar datos en AWS IoT
void updateShadow() {
  StaticJsonDocument<300> doc;
  doc["state"]["reported"]["sensors"]["temperature"] = bme.temperature;
  doc["state"]["reported"]["sensors"]["humidity"] = bme.humidity;
  doc["state"]["reported"]["sensors"]["gasResistance"] = bme.gas_resistance;

  char jsonBuffer[512];
  serializeJson(doc, jsonBuffer);
  client.publish(AWS_IOT_UPDATE_TOPIC, jsonBuffer);
  Serial.println("Estado reportado a AWS IoT.");
}

// Setup inicial
void setup() {
  Serial.begin(115200);

  // Configurar pines
  pinMode(FAN_PIN1, OUTPUT);
  pinMode(FAN_PIN2, OUTPUT);
  pinMode(ALARM_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);

  // Configurar WiFi y AWS IoT
  setupWiFi();
  net.setCACert(AWS_CERT_CA);
  net.setCertificate(AWS_CERT_CRT);
  net.setPrivateKey(AWS_CERT_PRIVATE);
  client.setServer(AWS_IOT_ENDPOINT, 8883);
  client.setCallback(messageHandler);
  connectAWS();

  // Inicializar BME680
  Wire.begin(SDA_PIN, SCL_PIN);
  if (!bme.begin()) {
    Serial.println("No se pudo inicializar el sensor BME680.");
    while (1);
  }
  Serial.println("Sensor BME680 inicializado.");
}

// Bucle principal
void loop() {
  if (!client.connected()) {
    connectAWS();
  }
  client.loop();

  // Leer sensores y actualizar AWS IoT
  if (bme.performReading()) {
    float airQuality = bme.gas_resistance;
    if (abs((airQuality - baseAirQuality) / baseAirQuality) > 0.15) {
      digitalWrite(ALARM_PIN, HIGH);
      isIncidentActive = true;
      Serial.println("Calidad de aire comprometida: Alarma activada.");
    } else {
      digitalWrite(ALARM_PIN, LOW);
      isIncidentActive = false;
    }
    updateShadow();
  }

  delay(2000);
}
