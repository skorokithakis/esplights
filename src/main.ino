// MQTT library: http://github.com/Imroy/pubsubclient
#include "PubSubClient.h"
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <DHT.h>

#define SSID "MyWiFi"
#define WIFI_PASS "the wifi password of my wifi"
#define OTA_PASS "the password for OTA updates"

#define MQTT_ENABLED 1
#define MQTT_SERVER "test.mosquitto.org"
#define MQTT_PORT 1883

#define ACTIVE_MOTION_THRESHOLD 10 * 60
#define LIGHT_ON_THRESHOLD 90
#define LIGHT_OFF_THRESHOLD 200

#define MOTION_PIN D1
#define TEMPERATURE_PIN D6
#define LED_PIN D4
#define LED2_PIN D0
#define RF_PIN D2
#define ALARM_PIN D5
#define LIGHT_PIN A0

#define UPDATE_INTERVAL_MS 200
#define PUBLISH_STATE_MS 1000
#define MAX_MESSAGE_LENGTH 600
#define DELAY_MULTIPLIER 100
#define UNAUTO_DELAY 13 * 60 * 60L
#define CPU_LATENCY 1

WiFiClient wclient;

enum {OTA, CONNECTING_WIFI, CONNECTING_MQTT, CONNECTED} state = CONNECTING_WIFI;

#if MQTT_ENABLED == 1
PubSubClient client(wclient, MQTT_SERVER, MQTT_PORT);
#endif

struct Sensors {
    unsigned long motion = 0;
    unsigned long motionTime = 0;
    unsigned int motionCounter = 0;
    unsigned int humidity = 0;
    unsigned int temperature = 0;
    unsigned int lightLevel = 0;
} sensors;

struct Misc {
    unsigned int lastUpdate = 0;
    unsigned int lastShow = 0;
    unsigned int alarm = 0;
    unsigned long autoDisabledTime = 0;
    char automation = 1;
    char lampState = 0;
} misc;

// The RF timings for turning the lamp on and off, in big-endian 2-byte ints.
unsigned char messageOn[] = {2, 134, 4, 247, 2, 155, 4, 226};
unsigned char messageOff[] = {2, 175, 4, 163, 5, 53, 2, 50};


// Publish a message to MQTT if connected.
void mqttPublish(String topic, String payload) {
#if MQTT_ENABLED == 1
    if (!client.connected()) {
        return;
    }
    client.publish(topic, payload);
#endif
}


// Receive a message from MQTT and act on it.
#if MQTT_ENABLED == 1
void mqttCallback(const MQTT::Publish& pub) {
    unsigned char cPayload[MAX_MESSAGE_LENGTH];
    String payload = pub.payload_string();

    payload.toCharArray((char *)cPayload, MAX_MESSAGE_LENGTH);

    if (payload == "auto on") {
        enableAuto(1);
    } else if (payload == "auto off") {
        enableAuto(0);
    } else if (payload == "alarm on") {
        misc.alarm = 1;
        digitalWrite(ALARM_PIN, LOW);
    } else if (payload == "alarm off") {
        misc.alarm = 0;
        digitalWrite(ALARM_PIN, HIGH);
    } else if (payload[0] == 1) {
        cmdSend(cPayload[3], &cPayload[6], cPayload[4], cPayload[5] * DELAY_MULTIPLIER, cPayload[1] - 1,  cPayload[2] - 1);
    }
}
#endif


// Transmit a command.
// pin: The pin to transmit to.
// message[[]: The 2-byte timings to send.
// repeat: How many times to repeat the message.
// intraDelay: How long to wait between repeats.
// dcHigh: How long the high duty cycle is (in microseconds).
//         Anything over 24 means 100% duty cycle. Will have a constant
//         added to it due to processing latency.
// dcLow: How long the low duty cycle is (in microseconds).
void transmit(char pin, unsigned char message[], unsigned char repeat, unsigned int intraDelay, unsigned char dcHigh, unsigned char dcLow) {
    int status = LOW;
    unsigned int out = 0;
    unsigned int i, j;
    unsigned int k;
    unsigned int dcTotal = dcHigh + dcLow;
    bool carrier = true;

    // Compensate for CPU latency.

    if (dcHigh < CPU_LATENCY) {
        dcHigh = 0;
    } else {
        dcHigh = dcHigh - CPU_LATENCY;
    }

    if (dcLow < CPU_LATENCY) {
        dcLow = 0;
    } else {
        dcLow = dcLow - CPU_LATENCY;
    }

    Serial.print("PIN: ");
    Serial.println(pin, DEC);
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);

    if (dcLow == 0) {
        // If the low time is 0, there's no carrier frequency.
        carrier = false;
    }

    // Send the message.
    for (j = 0; j < repeat; j++) {
        for (i = 0; i < MAX_MESSAGE_LENGTH; i = i + 2) {
            if (message[i] == 0 || message[i+1] == 0) {
                break;
            }

            // Convert two bytes to an int.
            out = (message[i] << 8) + message[i+1];
            if (status == HIGH) {
                status = LOW;
                digitalWrite(pin, status);
                delayMicroseconds(out);
            } else {
                status = HIGH;
                if (carrier) {
                    // Loop, once per pulse.
                    for (k = 0; k < out / dcTotal; k++) {
                        digitalWrite(pin, HIGH);
                        delayMicroseconds(dcHigh);
                        digitalWrite(pin, LOW);
                        delayMicroseconds(dcLow);
                    }
                } else {
                    digitalWrite(pin, status);
                    delayMicroseconds(out);
                }
            }
        }

        // Reset, just in case.
        digitalWrite(pin, LOW);
        delayMicroseconds(intraDelay);
        yield();
    }
}


// Send a command, printing some debug information.
void cmdSend(char pin, unsigned char message[], unsigned char repeat, unsigned int intraDelay, unsigned char dcHigh, unsigned char dcLow) {
    unsigned int out = 0;
    unsigned int i;

    digitalWrite(LED2_PIN, LOW);

    Serial.print("Sending on pin ");
    Serial.print(pin, DEC);
    Serial.print(", repeating ");
    Serial.print(repeat, DEC);
    Serial.print(" times, for ");
    Serial.print(intraDelay);
    Serial.print(" with a high duty cycle of ");
    Serial.print(dcHigh, DEC);
    Serial.print(" and a low duty cycle of ");
    Serial.print(dcLow, DEC);
    Serial.println(".");

    // Print the received timings to the console.
    for (i = 0; i < MAX_MESSAGE_LENGTH; i = i + 2) {
        if (message[i] == 0 || message[i+1] == 0) {
            break;
        }
        out = (message[i] << 8) + message[i+1];
        Serial.print(out, DEC);
        Serial.print(" ");
    }
    Serial.println("");
    yield();

    transmit(pin, message, repeat, intraDelay, dcHigh, dcLow);

    Serial.println("Command sent.");
    digitalWrite(LED2_PIN, HIGH);
}


// Enable or disable automation.
void enableAuto(char enable) {
    if (enable == 0) {
        misc.autoDisabledTime = millis() / 1000;
    } else {
        misc.autoDisabledTime = 0;
    }
    misc.automation = enable;
}


void checkTimers() {
    if ((misc.automation == 0) && (((millis() / 1000) - misc.autoDisabledTime) > UNAUTO_DELAY)) {
        enableAuto(1);
    }
}


// Return a JSON string of the entire state of the inputs.
String sensorState() {
    return String(String("{\n") +
                  "\"LAST_MOTION_SEC\": " + String((unsigned long)((millis() / 1000) - sensors.motionTime), DEC) + ",\n" +
                  "\"MOTION_COUNTER\": " + String(sensors.motionCounter, DEC) + ",\n" +
                  "\"ALARM\": " + String(misc.alarm, DEC) + ",\n" +
                  "\"HUMIDITY\": " + String(sensors.humidity, DEC) + ",\n" +
                  "\"TEMPERATURE\": " + String(sensors.temperature, DEC) + ",\n" +
                  "\"LIGHT_LEVEL\": " + String(sensors.lightLevel, DEC) + ",\n" +
                  "\"LAMP_STATE\": " + String(misc.lampState, DEC) + ",\n" +
                  "\"AUTOMATION\": " + String(misc.automation, DEC) + ",\n" +
                  "\"AUTO_DISABLED_TIME\": " + String(misc.autoDisabledTime, DEC) + ",\n" +
                  "\"SECS\": " + String(millis() / 1000, DEC) + ",\n" +
                  "\"UNAUTO_DELAY\": " + String(UNAUTO_DELAY, DEC) + "\n" +
                  "}\n");
}


// Decide whether to control the light.
void decide() {
    unsigned long lastMotionSec = (millis() / 1000) - sensors.motionTime;
    unsigned int lightLevel = analogRead(LIGHT_PIN);

    if (misc.automation == 0) {
        // Do nothing if we aren't managing the thing.
        return;
    }

    if ((lastMotionSec < ACTIVE_MOTION_THRESHOLD) &&
            (sensors.lightLevel < LIGHT_ON_THRESHOLD) &&
            (misc.lampState == 0)) {
        // Turn on if motion is detected and there is no light.
        cmdSend(RF_PIN, messageOn, 3, 10000, 0, 0);
        misc.lampState  = 1;
        mqttPublish("esplights_log", "Turning on...");
    } else if ((lastMotionSec >= ACTIVE_MOTION_THRESHOLD) &&
               (misc.lampState  == 1)) {
        // Turn off if motion is not detected for some time.
        cmdSend(RF_PIN, messageOff, 3, 10000, 0, 0);
        misc.lampState  = 0;
        mqttPublish("esplights_log", "Turning off because of motion...");
    } else if ((sensors.lightLevel > LIGHT_OFF_THRESHOLD) && (misc.lampState  == 1)) {
        // Turn off if other lights are turned on.
        cmdSend(RF_PIN, messageOff, 3, 10000, 0, 0);
        misc.lampState  = 0;
        mqttPublish("esplights_log", "Turning off because of lights...");
    }
}


// Update the sensors to their latest values.
void updateSensors() {
    char motionInput = 0;

    // Check last update.
    if (millis() > misc.lastUpdate + UPDATE_INTERVAL_MS) {
        misc.lastUpdate = millis();
    } else {
        return;
    }

    // Check motion.
    motionInput = digitalRead(MOTION_PIN);

    if (motionInput) {
        sensors.motionTime = millis() / 1000;
    }

    if ((motionInput == 1) && (sensors.motion == 0)) {
        sensors.motionCounter++;
    }
    sensors.motion = motionInput;
    sensors.lightLevel = analogRead(LIGHT_PIN);

    // We don't need to read this every second.
    if (millis() % 10 == 0) {
        DHT sensor(TEMPERATURE_PIN, DHT11);

        sensors.humidity = sensor.readHumidity();
        sensors.temperature = sensor.readTemperature();
    }
    decide();
}


// Publish our current sensor readings to MQTT.
void publishState() {
    // Check last update.
    if (millis() > misc.lastShow + PUBLISH_STATE_MS) {
        misc.lastShow = millis();
    } else {
        return;
    }

    mqttPublish("esplights_state", sensorState());
}


// Check the MQTT connection and reboot if we can't connect.
void connectMQTT() {
#if MQTT_ENABLED == 1
    if (state == OTA) return;

    if (client.connected()) {
        client.loop();
        state = CONNECTED;
    } else {
        state = CONNECTING_MQTT;

        int retries = 4;
        Serial.println("\nConnecting to MQTT...");
        while (!client.connect(MQTT::Connect("esplights").set_auth("", "")) && retries--) {
            delay(500);
        }


        if (!client.connected()) {
            Serial.println("\nfatal: MQTT server connection failed. Rebooting.");
            delay(200);
            ESP.restart();
        }

        client.subscribe("esplights_command");
    }
#endif
}


// Check the WiFi connection and connect if it's down.
void connectWifi() {
    if (WiFi.waitForConnectResult() != WL_CONNECTED) {
        state = CONNECTING_WIFI;

        WiFi.mode(WIFI_STA);
        while (WiFi.waitForConnectResult() != WL_CONNECTED) {
            WiFi.begin(SSID, WIFI_PASS);
            Serial.println("Connecting to wifi...");
        }

        Serial.print("Wifi connected, IP address: ");
        Serial.println(WiFi.localIP());
    }

    state = CONNECTING_MQTT;
}


void setup() {
    Serial.begin(115200);

    pinMode(MOTION_PIN, INPUT);
    pinMode(LIGHT_PIN, INPUT);
    pinMode(LED_PIN, OUTPUT);
    pinMode(LED2_PIN, OUTPUT);
    pinMode(RF_PIN, OUTPUT);
    pinMode(ALARM_PIN, OUTPUT);
    digitalWrite(LED2_PIN, HIGH);
    digitalWrite(ALARM_PIN, LOW);

#if MQTT_ENABLED == 1
    client.set_callback(mqttCallback);
#endif

    connectWifi();

    Serial.println("\nConnected to Wifi.");

    connectMQTT();
    state = CONNECTED;

    // Enable OTA updates.
    ArduinoOTA.setPassword((const char *) OTA_PASS);

    ArduinoOTA.onStart([]() {
            Serial.println("Start");
            state = OTA;
            });
    ArduinoOTA.onEnd([]() {
            Serial.println("\nEnd");
            });
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
            Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
            });
    ArduinoOTA.onError([](ota_error_t error) {
            Serial.printf("Error[%u]: ", error);
            if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
            else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
            else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
            else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
            else if (error == OTA_END_ERROR) Serial.println("End Failed");
            });
    ArduinoOTA.begin();
}


void loop() {
    digitalWrite(LED_PIN, !digitalRead(MOTION_PIN));

    connectWifi();
    connectMQTT();

    state = CONNECTED;

    checkTimers();
    updateSensors();
    publishState();

    ArduinoOTA.handle();
}
