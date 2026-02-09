#include <Arduino.h>
#include <Print.h>
#include <WiFi.h>
#include <WiFiClientSecure.h> // Required for secure MQTT connection
#include <PubSubClient.h>     // MQTT library
#include <ArduinoJson.h>
#include <Wire.h>
#include <RCSwitch.h>
#include "SparkFun_BNO08x_Arduino_Library.h"
#include "LIDAR_MODULE.h"

// =================================================================
// === NETWORK & MQTT CONFIGURATION ===
// =================================================================
// --- Wi-Fi Credentials ---
const char *ssid = "HIEULELE";
const char *password = "21072001";

// --- MQTT Broker Configuration (from your provided details) ---
const char *mqtt_server = "7569b4195fac4d7693e32460f9f6a1c1.s1.eu.hivemq.cloud";
const int mqtt_port = 8883;
const char *mqtt_user = "Hieule07";
const char *mqtt_password = "Hieule07@";

// --- MQTT Topics ---
// Topic for receiving target waypoints and current position updates from the server
const char *topic_subscribe_target = "robot/target";
// Topic for publishing the robot's status to the server
const char *topic_publish_status = "robot/status";

// --- MQTT Objects ---
WiFiClientSecure wifiClient;
PubSubClient mqttClient(wifiClient);
unsigned long lastMqttPublishTime = 0;
const long MQTT_PUBLISH_INTERVAL_MS = 1000; // Publish status every second

// =================================================================
// === HARDWARE & SENSOR CONFIGURATION (Unchanged) ===
// =================================================================
BNO08x myIMU;
RCSwitch mySwitch = RCSwitch();
LidarModule frontLidar(25, 26); // RX, TX

#define RF_RECEIVER_PIN 32
#define NUVOTON_SERIAL Serial2
volatile float currentHeading = 0.0f;
volatile uint16_t nuvoton_sensor1_cm = 999, nuvoton_sensor2_cm = 999;
volatile unsigned long lastNuvotonDataTime = 0;
const unsigned long NUVOTON_TIMEOUT_MS = 1000;

// =================================================================
// === POSITIONING & NAVIGATION ===
// =================================================================
struct Position
{
    float x = 0.0f, y = 0.0f;
    bool valid = false;
} currentPos, targetPos; // Stores current position and the next waypoint
bool autoMode = false;

// =================================================================
// === STATE MACHINE & CONTROL CONSTANTS (Unchanged) ===
// =================================================================
enum RobotState
{
    IDLE,
    NAVIGATING_TO_TARGET,
    OBSTACLE_AVOIDANCE,
    RECOVERY_STUCK,
    TARGET_REACHED
};
RobotState currentState = IDLE;
const char *stateNames[] = {"IDLE", "NAVIGATING_TO_TARGET", "OBSTACLE_AVOIDANCE", "RECOVERY_STUCK", "TARGET_REACHED"};

// --- Control Constants ---
const float DISTANCE_TOLERANCE = 15.0f; // Distance (cm) to a waypoint to consider it "reached"
const int MAX_SPEED = 150, MOVE_SPEED = 80, TURN_SPEED = 60, MAX_STEER_ANGLE = 45;
const float KP_STEER = 1.8f;
const float MIN_SAFE_DISTANCE_FRONT = 50.0f, MIN_SAFE_DISTANCE_NUVOTON = 50.0f;
const float MIN_SAFE_CLEARANCE_MM = 420.0f + 100.0f;

// --- Anti-Stuck Logic ---
unsigned long lastProgressTime = 0;
float lastDistanceToTarget = 9999.0f;
#define STUCK_TIMEOUT_MS 5000
#define PROGRESS_THRESHOLD 5.0f
#define RECOVERY_BACKUP_TIME_MS 1500
#define RECOVERY_TURN_TIME_MS 2000

// =================================================================
// === CORE FUNCTIONS (IMU, RF, NUVOTON, MOTOR) ===
// (These functions are kept 100% from your original code)
// =================================================================
int last_angelA = -999, last_angelB = -999, last_m1_dir = -1, last_m1_speed = -1, last_m2_dir = -1, last_m2_speed = -1;
void sendCarLikeCommand(int angelA, int angelB, int dirA, int speedA, int dirB, int speedB)
{
    if (angelA == last_angelA && angelB == last_angelB && dirA == last_m1_dir && speedA == last_m1_speed && dirB == last_m2_dir && speedB == last_m2_speed)
        return;
    char buf[128];
    snprintf(buf, sizeof(buf), "%d,50,%d,50,%d,%d,%d,%d|", angelA, angelB, dirA, speedA, dirB, speedB);
    NUVOTON_SERIAL.print(buf);
    last_angelA = angelA;
    last_angelB = angelB;
    last_m1_dir = dirA;
    last_m1_speed = speedA;
    last_m2_dir = dirB;
    last_m2_speed = speedB;
}
void stopAll() { sendCarLikeCommand(0, 0, 0, 0, 0, 0); }
void driveWithSteer(int angle, int speed)
{
    int dir = (speed >= 0) ? 1 : 0;
    sendCarLikeCommand(angle, angle, dir, abs(speed), dir, abs(speed));
}
void setReports()
{
    if (myIMU.enableRotationVector(10))
        Serial.println(F("Rotation vector enabled"));
    else
        Serial.println("Could not enable rotation vector");
}
void updateIMU()
{
    if (myIMU.getSensorEvent() && myIMU.getSensorEventID() == SENSOR_REPORTID_ROTATION_VECTOR)
    {
        float yaw = -(myIMU.getYaw()) * 180.0 / PI;
        if (yaw < 0)
            yaw += 360;
        currentHeading = yaw;
    }
}
struct RFData
{
    uint8_t joy1_x = 0, joy1_y = 0, joy2_x = 0, joy2_y = 0;
    bool valid = false;
    unsigned long lastReceived = 0;
} rfData;
const uint8_t JOY_CENTER = 7, JOY_DEADZONE = 2;
uint32_t lastDataValue = 0;
const unsigned long RF_TIMEOUT_MS = 1000;
void handleRFInput()
{
    if (!mySwitch.available())
        return;
    uint32_t val = mySwitch.getReceivedValue();
    mySwitch.resetAvailable();
    if (val == 0 || val == lastDataValue)
        return;
    lastDataValue = val;
    rfData.joy1_x = (val >> 16) & 0x0F;
    rfData.joy1_y = (val >> 12) & 0x0F;
    rfData.joy2_x = (val >> 6) & 0x0F;
    rfData.joy2_y = (val >> 2) & 0x0F;
    rfData.valid = true;
    rfData.lastReceived = millis();
}
void handleNuvotonInput()
{
    while (NUVOTON_SERIAL.available() >= 3)
    {
        uint8_t id = NUVOTON_SERIAL.read(), hi = NUVOTON_SERIAL.read(), lo = NUVOTON_SERIAL.read();
        uint16_t dist = (hi << 8) | lo;
        if (id == 1)
            nuvoton_sensor1_cm = dist;
        else if (id == 2)
            nuvoton_sensor2_cm = dist;
        lastNuvotonDataTime = millis();
    }
}
void controlManual()
{
    int steer = abs(JOY_CENTER - rfData.joy2_x) > JOY_DEADZONE ? map(JOY_CENTER - rfData.joy2_x, -JOY_CENTER, JOY_CENTER, -MAX_STEER_ANGLE, MAX_STEER_ANGLE) : 0;
    int speed = abs(JOY_CENTER - rfData.joy1_y) > JOY_DEADZONE ? map(JOY_CENTER - rfData.joy1_y, -JOY_CENTER, JOY_CENTER, -MAX_SPEED, MAX_SPEED) : 0;
    if (steer == 0 && speed == 0)
        stopAll();
    else
        driveWithSteer(steer, speed);
}
static inline float normalizeAngleDeg(float ang)
{
    while (ang > 180.0f)
        ang -= 360.0f;
    while (ang < -180.0f)
        ang += 360.0f;
    return ang;
}

// =================================================================
// === NEW MQTT & NETWORK FUNCTIONS ===
// =================================================================

void mqtt_callback(char *topic, byte *payload, unsigned int length);
void reconnect_mqtt();

/**
 * @brief Connects the ESP32 to the configured Wi-Fi network.
 */
void setup_wifi()
{
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    Serial.print("Connecting to WiFi");
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nWiFi connected! IP address: " + WiFi.localIP().toString());
}

/**
 * @brief Publishes the robot's current status to the MQTT broker.
 * This includes position, heading, state, and sensor data.
 */
void publishStatus()
{
    if (!mqttClient.connected())
        return;

    JsonDocument doc;
    doc["x"] = currentPos.x;
    doc["y"] = currentPos.y;
    doc["heading"] = currentHeading;
    doc["state"] = stateNames[currentState];
    doc["auto_mode"] = autoMode;
    doc["nuvoton1_cm"] = nuvoton_sensor1_cm;
    doc["nuvoton2_cm"] = nuvoton_sensor2_cm;
    doc["rf_active"] = (rfData.valid && (millis() - rfData.lastReceived <= RF_TIMEOUT_MS));

    String json_output;
    serializeJson(doc, json_output);
    mqttClient.publish(topic_publish_status, json_output.c_str());
}

/**
 * @brief Callback function executed when a message is received from the MQTT broker.
 * It parses JSON data to update the robot's current position and next target waypoint.
 * @param topic The topic the message was received on.
 * @param payload The message payload.
 * @param length The length of the payload.
 */
void mqtt_callback(char *topic, byte *payload, unsigned int length)
{
    Serial.printf("\n[MQTT] Message arrived on topic: %s\n", topic);

    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, payload, length);

    if (error)
    {
        Serial.print(F("deserializeJson() failed: "));
        Serial.println(error.c_str());
        return;
    }

    currentPos.x = doc["x"] | currentPos.x;
    currentPos.y = doc["y"] | currentPos.y;
    currentPos.valid = doc["position_valid"] | currentPos.valid;

    targetPos.x = doc["target_x"] | targetPos.x;
    targetPos.y = doc["target_y"] | targetPos.y;
    targetPos.valid = doc["target_valid"] | targetPos.valid;

    autoMode = doc["auto_mode"] | autoMode;

    Serial.printf("[MQTT] Received: Pos(%.1f, %.1f), Target(%.1f, %.1f), Auto: %d\n",
                  currentPos.x, currentPos.y, targetPos.x, targetPos.y, autoMode);

    if ((currentState == IDLE || currentState == TARGET_REACHED) && autoMode && targetPos.valid)
    {
        currentState = NAVIGATING_TO_TARGET;
        lastProgressTime = millis();
        lastDistanceToTarget = 9999.0f;
    }
}

/**
 * @brief Connects or reconnects to the MQTT broker and subscribes to the target topic.
 */
void reconnect_mqtt()
{
    while (!mqttClient.connected())
    {
        Serial.print("Attempting MQTT connection...");
        String clientId = "ESP32-DeliveryRobot-";
        clientId += String(random(0xffff), HEX);

        if (mqttClient.connect(clientId.c_str(), mqtt_user, mqtt_password))
        {
            Serial.println("connected!");
            mqttClient.subscribe(topic_subscribe_target);
            Serial.printf("Subscribed to topic: %s\n", topic_subscribe_target);
        }
        else
        {
            Serial.print("failed, rc=");
            Serial.print(mqttClient.state());
            Serial.println(" try again in 2 seconds");
            delay(2000);
        }
    }
}

// =================================================================
// === UPDATED AUTONOMOUS CONTROL LOGIC ===
// =================================================================
/**
 * @brief Manages the robot's autonomous navigation state machine.
 * Handles moving to the target, avoiding obstacles, and recovering if stuck.
 */
void controlAuto()
{
    float dx = targetPos.x - currentPos.x;
    float dy = targetPos.y - currentPos.y;
    float distance = sqrtf(dx * dx + dy * dy);
    float angleError = normalizeAngleDeg(atan2f(dy, dx) * 180.0f / PI - currentHeading);

    auto isPathClearCombined = [&]()
    {
        bool lidarClear = frontLidar.isPathClear(0.0f, MIN_SAFE_DISTANCE_FRONT);
        bool nuvotonSensorsClear = true;
        if (millis() - lastNuvotonDataTime < NUVOTON_TIMEOUT_MS)
        {
            nuvotonSensorsClear = (nuvoton_sensor1_cm > MIN_SAFE_DISTANCE_NUVOTON && nuvoton_sensor2_cm > MIN_SAFE_DISTANCE_NUVOTON);
        }
        return lidarClear && nuvotonSensorsClear;
    };

    switch (currentState)
    {
    case IDLE:
        stopAll();
        if (autoMode && targetPos.valid)
        {
            currentState = NAVIGATING_TO_TARGET;
        }
        break;

    case NAVIGATING_TO_TARGET:
    {
        if (distance < DISTANCE_TOLERANCE)
        {
            Serial.printf("[Nav] Waypoint reached! Distance: %.1f\n", distance);
            currentState = TARGET_REACHED;
            stopAll();
            break; // Stop and wait for the server to send the next waypoint
        }

        if (millis() - lastProgressTime > STUCK_TIMEOUT_MS)
        {
            if (abs(distance - lastDistanceToTarget) < PROGRESS_THRESHOLD)
            {
                currentState = RECOVERY_STUCK;
                stopAll();
                break;
            }
            lastProgressTime = millis();
            lastDistanceToTarget = distance;
        }

        if (!isPathClearCombined())
        {
            stopAll();
            currentState = OBSTACLE_AVOIDANCE;
            break;
        }

        int steeringAngle = constrain((int)(angleError * KP_STEER), -MAX_STEER_ANGLE, MAX_STEER_ANGLE);
        int moveSpeed = (abs(angleError) > 45.0) ? TURN_SPEED : MOVE_SPEED;
        driveWithSteer(steeringAngle, moveSpeed);
        break;
    }

    case OBSTACLE_AVOIDANCE:
    {
        stopAll();
        LidarModule::GapInfo bestGap = frontLidar.findBestGap(angleError, -90, 90, MIN_SAFE_CLEARANCE_MM);
        if (bestGap.isValid)
        {
            float newAngleError = normalizeAngleDeg(bestGap.angle_deg - 0);
            int steeringAngle = constrain((int)(newAngleError * KP_STEER), -MAX_STEER_ANGLE, MAX_STEER_ANGLE);
            driveWithSteer(steeringAngle, TURN_SPEED);
            delay(1500);
            currentState = NAVIGATING_TO_TARGET;
        }
        else
        {
            currentState = RECOVERY_STUCK;
        }
        break;
    }
    case RECOVERY_STUCK:
    {
        driveWithSteer(MAX_STEER_ANGLE, -MOVE_SPEED);
        delay(RECOVERY_BACKUP_TIME_MS);
        driveWithSteer(MAX_STEER_ANGLE, TURN_SPEED);
        delay(RECOVERY_TURN_TIME_MS);
        stopAll();
        lastProgressTime = millis();
        lastDistanceToTarget = 9999.0f;
        currentState = NAVIGATING_TO_TARGET;
        break;
    }
    case TARGET_REACHED:
        // Robot will stay in this state until the mqtt_callback receives a new
        // waypoint from the server and transitions the state back to NAVIGATING_TO_TARGET.
        stopAll();
        break;
    }
}

// =================================================================
// === MAIN SETUP FUNCTION ===
// =================================================================
/**
 * @brief Initializes serial communication, hardware, Wi-Fi, and MQTT client.
 */
void setup()
{
    Serial.begin(9600);
    delay(1000);
    Serial.println("\n=== ESP32 Robot Controller (MQTT Waypoint Version) ===");

    // Initialize hardware
    NUVOTON_SERIAL.begin(9600);
    mySwitch.enableReceive(RF_RECEIVER_PIN);
    Wire.begin();
    if (!myIMU.begin(0x4B, Wire))
    {
        Serial.println("✗ IMU BNO08x not detected. Halting.");
        while (1)
            ;
    }
    Wire.setClock(400000);
    setReports();
    frontLidar.begin();

    // Initialize network
    setup_wifi();
    wifiClient.setInsecure(); // Required for HiveMQ Cloud without a CA certificate
    mqttClient.setServer(mqtt_server, mqtt_port);
    mqttClient.setCallback(mqtt_callback);

    stopAll();
    Serial.println("=== Robot Controller Ready ===\n");
}

// =================================================================
// === MAIN LOOP FUNCTION ===
// =================================================================
/**
 * @brief The main execution loop.
 * Handles MQTT connection, sensor updates, status publishing, and control logic.
 */
void loop()
{
    if (!mqttClient.connected())
    {
        reconnect_mqtt();
    }
    mqttClient.loop(); // Crucial for processing MQTT messages

    // Read sensor data continuously
    updateIMU();
    handleRFInput();
    handleNuvotonInput();
    frontLidar.loop();

    // Periodically publish status to the server
    if (millis() - lastMqttPublishTime > MQTT_PUBLISH_INTERVAL_MS)
    {
        lastMqttPublishTime = millis();
        publishStatus();
    }

    // Main control logic: Prioritize manual control over autonomous mode
    bool rfActive = rfData.valid && (millis() - rfData.lastReceived <= RF_TIMEOUT_MS);
    if (rfActive)
    {
        if (currentState != IDLE)
        {
            currentState = IDLE;
            stopAll();
        }
        controlManual();
    }
    else if (autoMode && currentPos.valid && targetPos.valid)
    {
        controlAuto();
    }
    else
    {
        stopAll();
    }
}

