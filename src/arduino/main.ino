#include "secrets.h"
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "WiFi.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h>

// Pines y constantes
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

#define BUTTON_MONITOR_PIN 17
#define BUTTON_CONVEYOR_PIN 16
#define BUTTON_MANUAL_PIN 18

#define LPG_THRESHOLD 100
#define CO_THRESHOLD 1500
#define SMOKE_THRESHOLD 100

class SensorManager {
private:
    Adafruit_BME680 bme;
    float baseAirQuality;
    bool isCalibrating;
    unsigned long calibrationStartTime;
    unsigned long calibrationDuration;

public:
    SensorManager() : baseAirQuality(0), isCalibrating(false), calibrationDuration(5000) {}

    bool initialize() {
        return bme.begin();
    }

    void startCalibration() {
        isCalibrating = true;
        calibrationStartTime = millis();
    }

    void calibrate() {
        if (isCalibrating) {
            baseAirQuality = bme.gas_resistance;
            isCalibrating = false;
        }
    }

    float readTemperature() {
        return bme.readTemperature();
    }

    float readHumidity() {
        return bme.readHumidity();
    }

    float readAirQuality() {
        return bme.gas_resistance;
    }

    bool airQualityReduced(float currentAirQuality) {
        return ((baseAirQuality - currentAirQuality) / baseAirQuality * 100) >= 15;
    }
};

class ActuatorManager {
public:
    void initialize() {
        pinMode(FAN_PIN1, OUTPUT);
        pinMode(FAN_PIN2, OUTPUT);
        pinMode(ALARM_PIN, OUTPUT);
        pinMode(MOTOR_PWM_PIN, OUTPUT);
        pinMode(MOTOR_INA1_PIN, OUTPUT);
        pinMode(MOTOR_INA2_PIN, OUTPUT);
        pinMode(MOTOR_STBY_PIN, OUTPUT);
    }

    void activateFan() {
        digitalWrite(FAN_PIN1, HIGH);
        digitalWrite(FAN_PIN2, HIGH);
    }

    void deactivateFan() {
        digitalWrite(FAN_PIN1, LOW);
        digitalWrite(FAN_PIN2, LOW);
    }

    void activateAlarm() {
        digitalWrite(ALARM_PIN, HIGH);
    }

    void deactivateAlarm() {
        digitalWrite(ALARM_PIN, LOW);
    }

    void activateMotor() {
        digitalWrite(MOTOR_INA1_PIN, HIGH);
        digitalWrite(MOTOR_INA2_PIN, LOW);
    }

    void deactivateMotor() {
        digitalWrite(MOTOR_INA1_PIN, LOW);
        digitalWrite(MOTOR_INA2_PIN, LOW);
    }
};

class AWSManager {
private:
    WiFiClientSecure net;
    PubSubClient client;

public:
    AWSManager() : client(net) {}

    void initialize() {
        WiFi.mode(WIFI_STA);
        WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
        while (WiFi.status() != WL_CONNECTED) {
            delay(500);
        }

        net.setCACert(AWS_CERT_CA);
        net.setCertificate(AWS_CERT_CRT);
        net.setPrivateKey(AWS_CERT_PRIVATE);
        client.setServer(AWS_IOT_ENDPOINT, 8883);
    }

    void connect() {
        while (!client.connected()) {
            String clientId = "ESP32-" + String(random(0xffff), HEX);
            if (!client.connect(clientId.c_str())) {
                delay(5000);
            }
        }
    }

    void publishShadow(float temperature, float humidity, float airQuality, float lpg, float co, float smoke) {
        StaticJsonDocument<1024> jsonDoc;
        JsonObject state = jsonDoc.createNestedObject("state");

        JsonObject reported = state.createNestedObject("reported");
        reported["sensors"]["temperature"] = temperature;
        reported["sensors"]["humidity"] = humidity;
        reported["sensors"]["air_quality"] = airQuality;
        reported["sensors"]["mq2"]["lpg"] = lpg;
        reported["sensors"]["mq2"]["co"] = co;
        reported["sensors"]["mq2"]["smoke"] = smoke;

        String payload;
        serializeJson(jsonDoc, payload);
        client.publish(AWS_IOT_UPDATE_TOPIC, payload.c_str());
    }

    void loop() {
        client.loop();
    }
};

// Instancias globales
SensorManager sensorManager;
ActuatorManager actuatorManager;
AWSManager awsManager;

void setup() {
    Serial.begin(115200);

    if (!sensorManager.initialize()) {
        Serial.println("No se pudo inicializar el sensor BME680.");
        while (true);
    }

    actuatorManager.initialize();
    awsManager.initialize();
    awsManager.connect();
}

void loop() {
    awsManager.loop();
}
