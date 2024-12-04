#include "secrets.h"
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "WiFi.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h>

// Constantes y Pines
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

#define AWS_IOT_UPDATE_TOPIC "$aws/things/Env_Control_0001/shadow/update"
#define AWS_IOT_DELTA_TOPIC "$aws/things/Env_Control_0001/shadow/update/delta"

class SensorManager {
private:
    Adafruit_BME680 bme;
    float baseAirQuality;
    bool calibrating;
    unsigned long calibrationStartTime;
    unsigned long calibrationDuration;

public:
    float temperature, humidity, airQuality;
    float lpgValue, coValue, smokeValue;

    SensorManager() : baseAirQuality(0), calibrating(false), calibrationStartTime(0), calibrationDuration(5000) {}

    void begin() {
        if (!bme.begin()) {
            Serial.println("No se pudo encontrar un sensor BME680.");
            while (1);
        }
    }

    void calibrate() {
        if (calibrating) {
            baseAirQuality = bme.gas_resistance;
            Serial.println("Calibración iniciada...");
            calibrationStartTime = millis();
            calibrating = false;
        } else if (millis() - calibrationStartTime >= calibrationDuration) {
            Serial.println("Calibración completada.");
        }
    }

    void readSensors() {
        temperature = bme.readTemperature();
        humidity = bme.readHumidity();
        airQuality = bme.gas_resistance;

        // Simulación de MQ-2 (debes ajustar según tu hardware)
        lpgValue = analogRead(MQ2_PIN) * 0.1;
        coValue = analogRead(MQ2_PIN) * 0.08;
        smokeValue = analogRead(MQ2_PIN) * 0.06;
    }

    bool hasAirQualityReduced(float threshold = 15.0) {
        float reduction = (baseAirQuality - airQuality) / baseAirQuality * 100;
        return reduction >= threshold;
    }

    void startCalibration() { calibrating = true; }
};

class ActuatorManager {
public:
    void begin() {
        pinMode(FAN_PIN1, OUTPUT);
        pinMode(FAN_PIN2, OUTPUT);
        pinMode(ALARM_PIN, OUTPUT);
        pinMode(MOTOR_INA1_PIN, OUTPUT);
        pinMode(MOTOR_INA2_PIN, OUTPUT);
        pinMode(MOTOR_STBY_PIN, OUTPUT);
    }

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
};

class AWSManager {
private:
    WiFiClientSecure net;
    PubSubClient client;

public:
    AWSManager() : client(net) {}

    void begin() {
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
        connect();
    }

    void connect() {
        Serial.println("Conectando a AWS IoT");
        while (!client.connected()) {
            String clientId = "ESP32-" + String(random(0xffff), HEX);
            if (client.connect(clientId.c_str())) {
                Serial.println("Conectado a AWS IoT");
            } else {
                Serial.print("Error al conectar: ");
                Serial.println(client.state());
                delay(5000);
            }
        }
    }

    void publishState(SensorManager &sensors, ActuatorManager &actuators, bool fanStatus, bool alarmStatus, bool motorStatus) {
        StaticJsonDocument<1024> jsonDoc;
        JsonObject state = jsonDoc.createNestedObject("state");

        JsonObject reported = state.createNestedObject("reported");
        reported["sensors"]["temperature"] = sensors.temperature;
        reported["sensors"]["humidity"] = sensors.humidity;
        reported["sensors"]["air_quality"] = sensors.airQuality;
        reported["sensors"]["mq2"]["lpg"] = sensors.lpgValue;
        reported["sensors"]["mq2"]["co"] = sensors.coValue;
        reported["sensors"]["mq2"]["smoke"] = sensors.smokeValue;

        JsonObject actuatorsState = reported.createNestedObject("actuators");
        actuatorsState["fan"] = fanStatus ? 1 : 0;
        actuatorsState["alarm"] = alarmStatus ? 1 : 0;
        actuatorsState["motor"] = motorStatus ? 1 : 0;

        String payload;
        serializeJson(jsonDoc, payload);

        if (client.publish(AWS_IOT_UPDATE_TOPIC, payload.c_str())) {
            Serial.println("Reporte exitoso a AWS IoT.");
        } else {
            Serial.println("Error al publicar en AWS IoT.");
        }
    }

    void loop() { client.loop(); }
};

class ButtonManager {
private:
    unsigned long lastMonitorButtonTime, lastConveyorButtonTime, lastManualButtonTime;
    const unsigned long debounceDelay;

public:
    ButtonManager() : lastMonitorButtonTime(0), lastConveyorButtonTime(0), lastManualButtonTime(0), debounceDelay(200) {}

    void begin() {
        pinMode(BUTTON_MONITOR_PIN, INPUT_PULLUP);
        pinMode(BUTTON_CONVEYOR_PIN, INPUT_PULLUP);
        pinMode(BUTTON_MANUAL_PIN, INPUT_PULLUP);
    }

    bool handleButton(int pin, unsigned long &lastButtonTime) {
        unsigned long currentMillis = millis();
        if (digitalRead(pin) == LOW && currentMillis - lastButtonTime > debounceDelay) {
            lastButtonTime = currentMillis;
            return true;
        }
        return false;
    }

    bool isMonitorPressed() { return handleButton(BUTTON_MONITOR_PIN, lastMonitorButtonTime); }
    bool isConveyorPressed() { return handleButton(BUTTON_CONVEYOR_PIN, lastConveyorButtonTime); }
    bool isManualPressed() { return handleButton(BUTTON_MANUAL_PIN, lastManualButtonTime); }
};

class SystemController {
private:
    SensorManager sensors;
    ActuatorManager actuators;
    AWSManager aws;
    ButtonManager buttons;

    bool fanStatus, alarmStatus, motorStatus, reportingActive, conveyorRunning, manualControl;

public:
    void setup() {
        Serial.begin(115200);
        sensors.begin();
        actuators.begin();
        aws.begin();
        buttons.begin();

        fanStatus = false;
        alarmStatus = false;
        motorStatus = false;
        reportingActive = false;
        conveyorRunning = false;
        manualControl = false;
    }

    void loop() {
        aws.loop();

        if (buttons.isMonitorPressed()) {
            reportingActive = !reportingActive;
            if (reportingActive) {
                sensors.startCalibration();
            }
        }

        if (reportingActive) {
            sensors.readSensors();
            if (sensors.hasAirQualityReduced()) {
                actuators.activateFan();
            }

            aws.publishState(sensors, actuators, fanStatus, alarmStatus, motorStatus);
        }
    }
};

// Instancia principal del sistema
SystemController system;

void setup() {
    system.setup();
}

void loop() {
    system.loop();
}
