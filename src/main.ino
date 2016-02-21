#include "PubSubClient.h"
#include <EEPROM.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <DHT.h>
#include "transmit.h"

#ifndef WIFI_SSID
#define WIFI_SSID "your ssid"
#endif

#ifndef WIFI_PASS
#define WIFI_PASS "your wifi password"
#endif

#ifndef OTA_PASS
#define OTA_PASS "an ota pass"
#endif

#define MQTT_ENABLED 1

#ifndef MQTT_SERVER
#define MQTT_SERVER "test.mosquitto.org"
#endif

#define MQTT_PORT 1883

#ifndef NTP_SERVER
#define NTP_SERVER "pool.ntp.org"
#endif

#define ACTIVE_MOTION_THRESHOLD 10 * 60
#define LIGHT_ON_THRESHOLD 90
#define LIGHT_OFF_THRESHOLD 200

#define MOTION_PIN D7
#define TEMPERATURE_PIN D6
#define LED_PIN D4
#define LED2_PIN D0
#define RF_PIN D1
#define ALARM_PIN D5
#define LIGHT_PIN A0

#define UPDATE_INTERVAL_MS 200
#define PUBLISH_STATE_MS 1000
#define MAX_MESSAGE_LENGTH 600
#define DELAY_MULTIPLIER 100
#define UNAUTO_DELAY 13 * 60 * 60L

#define CONFIG_VERSION "cf1"

WiFiClient wclient;
char COMMAND_CHANNEL[] = "esplights_command";
char STATE_CHANNEL[] = "esplights_state";
char LOG_CHANNEL[] = "esplights_log";
String NAME = "esplights1";

enum {OTA, CONNECTING_WIFI, CONNECTING_MQTT, CONNECTED} state = CONNECTING_WIFI;

#if MQTT_ENABLED == 1
PubSubClient client(wclient);
#endif

struct Sensors {
    unsigned long motion = 0;
    unsigned long motionTime = 0;
    unsigned int motionCounter = 0;
    unsigned int humidity = 0;
    unsigned int temperature = 0;
    unsigned int lightLevel = 0;
} sensors;

struct Persistent {
    char version[4] = CONFIG_VERSION;
    unsigned long autoDisableTime = 0;
    char automation = 1;
} persistent;

struct Misc {
    unsigned int lastUpdate = 0;
    unsigned int lastShow = 0;
    unsigned int alarm = 0;
    unsigned long bootTime = 0;
    char lampState = 0;
} misc;

unsigned char messageOn[] = {2, 113, 2, 113, 5, 32, 2, 113, 5, 12, 2, 113, 5, 12, 2, 113, 5, 32, 2, 113, 5, 12, 2, 134, 5, 12, 2, 113, 5, 12, 2, 134, 4, 247, 2, 155, 4, 247, 2, 134, 4, 247, 2, 134, 5, 12, 2, 134, 4, 247, 2, 155, 4, 226, 5, 12, 2, 92, 2, 155, 4, 205, 5, 12, 2, 92, 2, 155, 4, 247, 2, 134, 4, 247, 2, 134, 4, 247, 2, 155, 4, 226};
unsigned char messageOff[] = {2, 113, 2, 113, 5, 32, 2, 113, 5, 12, 2, 113, 5, 32, 2, 92, 5, 32, 2, 134, 5, 12, 2, 113, 5, 12, 2, 113, 5, 12, 2, 134, 4, 247, 2, 155, 4, 247, 2, 134, 5, 12, 2, 134, 4, 247, 2, 134, 4, 247, 2, 155, 4, 247, 5, 53, 2, 71, 2, 155, 4, 247, 2, 134, 4, 247, 2, 155, 4, 226, 2, 155, 4, 205, 2, 175, 4, 163, 5, 53, 2, 50};


// Publish a message to MQTT if connected.
void mqttPublish(String topic, String payload) {
#if MQTT_ENABLED == 1
    if (!client.connected()) {
        return;
    }
    client.publish(topic.c_str(), payload.c_str());
#endif
}


// Receive a message from MQTT and act on it.
#if MQTT_ENABLED == 1
void mqttCallback(char* chTopic, byte* chPayload, unsigned int length) {
    chPayload[length] = '\0';
    String payload = String((char*)chPayload);

    if (payload == "auto on") {
        enableAuto(1);
    } else if (payload == "auto off") {
        enableAuto(0);
    } else if (payload == "alarm on") {
        misc.alarm = 1;
        digitalWrite(ALARM_PIN, HIGH);
    } else if (payload == "alarm off") {
        misc.alarm = 0;
        digitalWrite(ALARM_PIN, LOW);
    } else if (payload[0] == 1) {
        cmdSend(chPayload[3], &chPayload[6], chPayload[4], chPayload[5] * DELAY_MULTIPLIER, chPayload[1] - 1,  chPayload[2] - 1);
    }
}
#endif


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


unsigned long getNTPTime() {
    IPAddress address;
    WiFiUDP udp;
    const int NTP_PACKET_SIZE = 48;
    byte packetBuffer[NTP_PACKET_SIZE];

    udp.begin(2390);
    // set all bytes in the buffer to 0
    memset(packetBuffer, 0, NTP_PACKET_SIZE);
    // Initialize values needed to form NTP request
    // (see URL above for details on the packets)
    packetBuffer[0] = 0b11100011;   // LI, Version, Mode
    packetBuffer[1] = 0;     // Stratum, or type of clock
    packetBuffer[2] = 6;     // Polling Interval
    packetBuffer[3] = 0xEC;  // Peer Clock Precision
    // 8 bytes of zero for Root Delay & Root Dispersion
    packetBuffer[12]  = 49;
    packetBuffer[13]  = 0x4E;
    packetBuffer[14]  = 49;
    packetBuffer[15]  = 52;

    // all NTP fields have been given values, now
    // you can send a packet requesting a timestamp:
    WiFi.hostByName(NTP_SERVER, address);
    udp.beginPacket(address, 123); //NTP requests are to port 123
    udp.write(packetBuffer, NTP_PACKET_SIZE);
    udp.endPacket();

    delay(1000);

    int cb = udp.parsePacket();
    if (!cb) {
        return 0;
    } else {
        udp.read(packetBuffer, NTP_PACKET_SIZE);
        unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);
        unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);
        unsigned long secsSince1900 = highWord << 16 | lowWord;
        const unsigned long seventyYears = 2208988800UL;
        unsigned long epoch = secsSince1900 - seventyYears;
        return epoch;
    }
}


void resetPins() {
    pinMode(1, OUTPUT);
    digitalWrite(1, LOW);
    pinMode(2, OUTPUT);
    digitalWrite(2, LOW);
    pinMode(3, OUTPUT);
    digitalWrite(3, LOW);
    pinMode(4, OUTPUT);
    digitalWrite(4, LOW);
    pinMode(5, OUTPUT);
    digitalWrite(5, LOW);
    pinMode(12, OUTPUT);
    digitalWrite(12, LOW);
    pinMode(13, OUTPUT);
    digitalWrite(13, LOW);
    pinMode(14, OUTPUT);
    digitalWrite(14, LOW);
    pinMode(15, OUTPUT);
    digitalWrite(15, LOW);
}

void loadState() {
    if (EEPROM.read(0) == CONFIG_VERSION[0] &&
        EEPROM.read(1) == CONFIG_VERSION[1] &&
        EEPROM.read(2) == CONFIG_VERSION[2]) {
        for (unsigned int t=0; t<sizeof(persistent); t++)
            *((char*)&persistent + t) = EEPROM.read(t);
    }
}


void saveState() {
    for (unsigned int t=0; t<sizeof(persistent); t++)
        EEPROM.write(t, *((char*)&persistent + t));
    EEPROM.commit();
}


unsigned long currentTime() {
    return misc.bootTime + (millis() / 1000);
}


// Enable or disable automation.
void enableAuto(char enable) {
    if (enable == 0) {
        persistent.autoDisableTime = currentTime() + UNAUTO_DELAY;
    } else {
        persistent.autoDisableTime = 0;
    }
    persistent.automation = enable;
    saveState();
}


void checkTimers() {
    if ((persistent.automation == 0) && ((currentTime() > persistent.autoDisableTime))) {
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
                  "\"AUTOMATION\": " + String(persistent.automation, DEC) + ",\n" +
                  "\"AUTO_DISABLE_TIME\": " + String(persistent.autoDisableTime, DEC) + ",\n" +
                  "\"CURRENT_TIME\": " + String(currentTime(), DEC) + ",\n" +
                  "}\n");
}


// Decide whether to control the light.
void decide() {
    unsigned long lastMotionSec = (millis() / 1000) - sensors.motionTime;
    unsigned int lightLevel = analogRead(LIGHT_PIN);

    if (persistent.automation == 0) {
        // Do nothing if we aren't managing the thing.
        return;
    }

    if ((lastMotionSec < ACTIVE_MOTION_THRESHOLD) &&
            (sensors.lightLevel < LIGHT_ON_THRESHOLD) &&
            (misc.lampState == 0)) {
        // Turn on if motion is detected and there is no light.
        cmdSend(RF_PIN, messageOn, 3, 10000, 0, 0);
        misc.lampState = 1;
        mqttPublish(LOG_CHANNEL, "Turning on...");
    } else if ((lastMotionSec >= ACTIVE_MOTION_THRESHOLD) &&
               (misc.lampState == 1)) {
        // Turn off if motion is not detected for some time.
        cmdSend(RF_PIN, messageOff, 3, 10000, 0, 0);
        misc.lampState = 0;
        mqttPublish(LOG_CHANNEL, "Turning off because of motion...");
    } else if ((sensors.lightLevel > LIGHT_OFF_THRESHOLD) && (misc.lampState == 1)) {
        // Turn off if other lights are turned on.
        cmdSend(RF_PIN, messageOff, 3, 10000, 0, 0);
        misc.lampState = 0;
        mqttPublish(LOG_CHANNEL, "Turning off because of lights...");
    }
}


// Update the sensors to their latest values.
void updateSensors() {
    char motionInput = 0;

    motionInput = digitalRead(MOTION_PIN);
    digitalWrite(LED_PIN, !motionInput);

    // Check last update.
    if (millis() > misc.lastUpdate + UPDATE_INTERVAL_MS) {
        misc.lastUpdate = millis();
    } else {
        return;
    }

    // Check motion.
    Serial.println(motionInput, DEC);

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

    mqttPublish(STATE_CHANNEL, sensorState());
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
        while (!client.connect(NAME.c_str()) && retries--) {
            delay(500);
            Serial.println("Retry...");
        }

        if (!client.connected()) {
            Serial.println("\nfatal: MQTT server connection failed. Rebooting.");
            delay(200);
            ESP.restart();
        }

        Serial.println("Connected.");
        client.subscribe(COMMAND_CHANNEL);
    }
#endif
}


// Check the WiFi connection and connect if it's down.
void connectWifi() {
    if (WiFi.waitForConnectResult() != WL_CONNECTED) {
        state = CONNECTING_WIFI;

        WiFi.mode(WIFI_STA);
        while (WiFi.waitForConnectResult() != WL_CONNECTED) {
            WiFi.begin(WIFI_SSID, WIFI_PASS);
            Serial.println("Connecting to wifi...");
        }

        Serial.print("Wifi connected, IP address: ");
        Serial.println(WiFi.localIP());
    }

    state = CONNECTING_MQTT;
}


void setup() {
    resetPins();
    Serial.begin(115200);
    EEPROM.begin(32);
    loadState();

    pinMode(MOTION_PIN, INPUT);
    pinMode(LIGHT_PIN, INPUT);
    pinMode(LED_PIN, OUTPUT);
    pinMode(LED2_PIN, OUTPUT);
    pinMode(RF_PIN, OUTPUT);
    pinMode(ALARM_PIN, OUTPUT);
    digitalWrite(LED2_PIN, HIGH);
    digitalWrite(ALARM_PIN, LOW);

#if MQTT_ENABLED == 1
    client.setServer(MQTT_SERVER, MQTT_PORT);
    client.setCallback(mqttCallback);
#endif

    connectWifi();
    connectMQTT();
    state = CONNECTED;

    misc.bootTime = getNTPTime() - (millis() / 1000);

    // Enable OTA updates.
    ArduinoOTA.setPassword((const char *) OTA_PASS);
    ArduinoOTA.setHostname("ESPlights");

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
    connectWifi();
    connectMQTT();

    state = CONNECTED;

    checkTimers();
    updateSensors();
    publishState();

    ArduinoOTA.handle();
}
