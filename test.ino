#include "secrets.h"
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "SparkFunLIS3DH.h"
#include "Wire.h"
#include "SPI.h"
#include <Arduino.h>

#define FALL_THRESHOLD 1.2  // Threshold for fall detection
#define STABLE_THRESHOLD 1.0  // Threshold to determine stability

#define AWS_IOT_PUBLISH_TOPIC "stick/pub"
#define AWS_IOT_SUBSCRIBE_TOPIC "stick/sub"

#define TRIG_PIN 14
#define ECHO_PIN 13

LIS3DH SensorOne(I2C_MODE, 0x19);
LIS3DH SensorTwo(I2C_MODE, 0x18);

WiFiClientSecure net;
PubSubClient client(net);

// Function to measure distance
float measureDistance() {
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    float duration = pulseIn(ECHO_PIN, HIGH);
    float distance = duration * 0.034 / 2;  // Convert time to distance
    return distance;
}

void connectWiFi() {
    Serial.println("Connecting to Wi-Fi");
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    int retries = 0;
    while (WiFi.status() != WL_CONNECTED && retries < 30) {
        delay(1000);
        Serial.print(".");
        retries++;
    }
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\nConnected to WiFi");
        Serial.print("IP Address: ");
        Serial.println(WiFi.localIP());
    } else {
        Serial.println("\nFailed to connect to WiFi");
    }
}

void connectAWS() {
    Serial.println("Configuring WiFiClientSecure credentials");
    net.setCACert(AWS_CERT_CA);
    net.setCertificate(AWS_CERT_CRT);
    net.setPrivateKey(AWS_CERT_PRIVATE);
    client.setServer(AWS_IOT_ENDPOINT, 8883);
    client.setCallback(messageHandler);
    Serial.println("Connecting to AWS IoT");
    while (!client.connected()) {
        Serial.print("Attempting MQTT connection...");
        if (client.connect(THINGNAME, NULL, NULL)) {
            Serial.println("connected");
            client.subscribe(AWS_IOT_SUBSCRIBE_TOPIC);
        } else {
            handleConnectionFailure();
        }
    }
}

void handleConnectionFailure() {
    Serial.print("failed, rc=");
    Serial.print(client.state());
    Serial.println(" Check the reason and handle accordingly");
    Serial.println("Reattempting in 5 seconds...");
    delay(5000);
}

void publishMessage(String message, float x, float y, float z, float magnitude, float distance) {
    StaticJsonDocument<200> doc;
    doc["message"] = message;
    doc["accelX"] = x;
    doc["accelY"] = y;
    doc["accelZ"] = z;
    doc["magnitude"] = magnitude;
    doc["distance"] = distance;
    char jsonBuffer[512];
    serializeJson(doc, jsonBuffer);
    client.publish(AWS_IOT_PUBLISH_TOPIC, jsonBuffer);
}

void messageHandler(char* topic, byte* payload, unsigned int length) {
    Serial.print("Incoming: ");
    Serial.println(topic);
    StaticJsonDocument<200> doc;
    deserializeJson(doc, payload);
    const char* message = doc["message"];
    Serial.println(message);
}

void setup() {
    Serial.begin(115200);
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
    SensorOne.begin();
    SensorTwo.begin();
    connectWiFi();
    connectAWS();
}

void loop() {
    if (!client.connected()) {
        connectAWS();
    }
    client.loop();
    float x1 = SensorOne.readFloatAccelX();
    float y1 = SensorOne.readFloatAccelY();
    float z1 = SensorOne.readFloatAccelZ();
    float magnitude = sqrt(x1 * x1 + y1 * y1 + z1 * z1);
    float distance = measureDistance();  // Measure the distance

    Serial.print("Magnitude: ");
    Serial.println(magnitude);
    Serial.print("Distance: ");
    Serial.println(distance);

    if (magnitude > FALL_THRESHOLD && distance < 30 || magnitude < 0.5 ||  magnitude > 1.6) {
        Serial.println("Fall Detected!");
        publishMessage("F", x1, y1, z1, magnitude, distance);
    } else if (magnitude > STABLE_THRESHOLD - 0.2 && magnitude < STABLE_THRESHOLD + 0.2) {
        Serial.println("Stick is stable.");
        publishMessage("S.", x1, y1, z1, magnitude, distance);
    } else {
        Serial.println(magnitude > STABLE_THRESHOLD + 0.2 && magnitude < FALL_THRESHOLD);
        publishMessage("M.", x1, y1, z1, magnitude, distance);
    }

    delay(100);  // Delay for stability
}
