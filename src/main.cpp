#include <Arduino.h>
#include <WiFi.h>
#include <FirebaseESP32.h>
#include "oled.h"
#include "max30102_agent.h"
#include "bmi160_agent.h"

// Provide the token generation process info
#include "addons/TokenHelper.h"
// Provide the RTDB payload printing info
#include "addons/RTDBHelper.h"

#define I2C_SDA 8
#define I2C_SCL 9

// WiFi credentials
#define WIFI_SSID "CEIOT"
#define WIFI_PASSWORD "CE-1OT@!"

// Firebase credentials (from Firebase Console → Project Settings)
#define API_KEY "AIzaSyCfV1SKtW8JPujExcmQbfmMZktto_yBJP4"
#define DATABASE_URL "https://iotmade-default-rtdb.asia-southeast1.firebasedatabase.app"

// Firebase objects
FirebaseData fbdo;
FirebaseData fbdo_upload;
FirebaseAuth auth;
FirebaseConfig config;
OledDisplay oled;

Max30102Agent hrSensor;
Bmi160Agent fallSensor;

bool firebaseReady = false;
unsigned long sendDataPrevMillis = 0;

// Device identity variables
String deviceUID = "";
String pairingCode = "";
String ownerUID = "";

// Peripheral health tracking
bool oledEnabled = false;
bool max30102Enabled = false;
bool bmi160Enabled = false;
uint8_t activeBmi160Addr = 0x68;

// FreeRTOS Mutexes
SemaphoreHandle_t i2cMutex = NULL;
SemaphoreHandle_t dataMutex = NULL;

// Shared sensor data structure
struct SensorData_t {
    float bpm = 0.0f;
    float spo2 = 0.0f;
    bool isFall = false;
    float accelX = 0.0f;
    float accelY = 0.0f;
    float accelZ = 0.0f;
    float accelMag = 1.0f;
    float gyroX = 0.0f;
    float gyroY = 0.0f;
    float gyroZ = 0.0f;
    uint32_t rawIr = 0;
    uint32_t rawRed = 0;
};
SensorData_t sharedData;

void setupWiFi() {
    Serial.println("Resetting WiFi...");
    WiFi.disconnect(true, true);
    WiFi.mode(WIFI_OFF);
    delay(500);

    WiFi.mode(WIFI_STA);
    
    // Set WiFi output power to 10 dBm to stabilize signal and avoid I2C interference.
    WiFi.setTxPower((wifi_power_t)40);

    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.print("Connecting to WiFi");
    unsigned long startMs = millis();
    while (WiFi.status() != WL_CONNECTED && (millis() - startMs < 15000)) {
        Serial.print(".");
        delay(300);
    }
    Serial.println();
    if (WiFi.status() == WL_CONNECTED) {
        Serial.print("Connected with IP: ");
        Serial.println(WiFi.localIP());
    } else {
        Serial.println("WiFi connection failed (timeout).");
    }
}

void setupFirebase() {
    Serial.print("Syncing time via NTP");
    configTime(0, 0, "pool.ntp.org", "time.nist.gov");
    time_t now = time(nullptr);
    int retries = 0;
    while (now < 1000000000 && retries < 20) {
        delay(250);
        Serial.print(".");
        now = time(nullptr);
        retries++;
    }
    Serial.println();
    if (now > 1000000000) {
        Serial.printf("Time synchronized successfully. Epoch: %ld\n", (long)now);
    } else {
        Serial.println("WARNING: Time synchronization timed out! Firebase operations might fail.");
    }

    config.api_key = API_KEY;
    config.database_url = DATABASE_URL;

    // Anonymous sign-in
    if (Firebase.signUp(&config, &auth, "", "")) {
        Serial.println("Firebase sign-up OK");
        firebaseReady = true;
    } else {
        Serial.printf("Firebase sign-up error: %s\n", config.signer.signupError.message.c_str());
    }

    config.token_status_callback = tokenStatusCallback;
    Firebase.begin(&config, &auth);
    Firebase.reconnectWiFi(true);
}

void checkDeviceStatus() {
    if (WiFi.status() != WL_CONNECTED || !firebaseReady || !Firebase.ready()) {
        return;
    }

    String ownerPath = "/devices/" + deviceUID + "/ownerUID";
    if (Firebase.getString(fbdo, ownerPath)) {
        String val = fbdo.stringData();
        String dataType = fbdo.dataType();
        
        if (dataType == "null" || val.length() == 0 || val == "null") {
            ownerUID = "";
            if (dataType == "null" || val.length() == 0) {
                Serial.println("[Firebase] Registering device...");
                if (Firebase.setString(fbdo, ownerPath, "null") && 
                    Firebase.setString(fbdo, "/devices/" + deviceUID + "/pairingCode", pairingCode)) {
                    Serial.println("[Firebase] Device registered successfully.");
                } else {
                    Serial.printf("[Firebase Error] Registration failed: %s\n", fbdo.errorReason().c_str());
                }
            }
        } else {
            ownerUID = val;
            static bool testedWrite = false;
            if (!testedWrite) {
                testedWrite = true;
                Serial.printf("[Firebase Test] Synchronous test write to /users/%s/devices_data/%s/sensor_data...\n", ownerUID.c_str(), deviceUID.c_str());
                FirebaseJson testJson;
                testJson.set("test_field", "working");
                String testPath = "/users/" + ownerUID + "/devices_data/" + deviceUID + "/sensor_data";
                if (Firebase.setJSON(fbdo, testPath, testJson)) {
                    Serial.println("[Firebase Test] Synchronous test write SUCCEEDED!");
                } else {
                    Serial.printf("[Firebase Test] Synchronous test write FAILED: %s\n", fbdo.errorReason().c_str());
                }
            }
        }
    } else {
        String err = fbdo.errorReason();
        if (err.indexOf("path not exist") != -1 || err.indexOf("not found") != -1 || fbdo.httpCode() == 404) {
            ownerUID = "";
            Serial.println("[Firebase] Path not found. Registering device...");
            if (Firebase.setString(fbdo, ownerPath, "null") && 
                Firebase.setString(fbdo, "/devices/" + deviceUID + "/pairingCode", pairingCode)) {
                Serial.println("[Firebase] Device registered successfully.");
            } else {
                Serial.printf("[Firebase Error] Registration failed: %s\n", fbdo.errorReason().c_str());
            }
        } else {
            Serial.printf("[Firebase Error] checkDeviceStatus failed: %s\n", err.c_str());
            ownerUID = "";
        }
    }
}

void recoverI2CBus(int sdaPin, int sclPin) {
    pinMode(sdaPin, INPUT_PULLUP);
    pinMode(sclPin, INPUT_PULLUP);
    delayMicroseconds(10);
    
    pinMode(sclPin, OUTPUT);
    
    // Toggle SCL up to 9 times if SDA is held low
    for (int i = 0; i < 9; i++) {
        if (digitalRead(sdaPin) == HIGH) {
            break; // SDA has been released
        }
        digitalWrite(sclPin, LOW);
        delayMicroseconds(10);
        digitalWrite(sclPin, HIGH);
        delayMicroseconds(10);
    }
    
    // Generate a manual STOP condition
    pinMode(sdaPin, OUTPUT);
    digitalWrite(sdaPin, LOW);
    delayMicroseconds(10);
    
    pinMode(sclPin, OUTPUT);
    digitalWrite(sclPin, HIGH);
    delayMicroseconds(10);
    
    digitalWrite(sdaPin, HIGH);
    delayMicroseconds(10);
    
    // Return pins to floating state before Wire resets
    pinMode(sdaPin, INPUT);
    pinMode(sclPin, INPUT);
}

// --- FreeRTOS Task: Sensor Polling at 100Hz ---
void vSensorTask(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(10); // 10ms (100Hz ODR)

    GyroData_t localGyro = {0};
    uint32_t localRawIr = 0;
    uint32_t localRawRed = 0;
    float localBpm = 0.0f;
    float localSpo2 = 0.0f;

    for (;;) {
        // 1. Read sensors (requires I2C mutex)
        if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
            if (max30102Enabled) {
                hrSensor.readRaw(localRawIr, localRawRed);
                localBpm = hrSensor.getBPM();
                localSpo2 = hrSensor.getSpO2();
            }
            if (bmi160Enabled) {
                fallSensor.readData(&localGyro);
            }
            xSemaphoreGive(i2cMutex);
        }

        // 2. Update shared data structure (requires data mutex)
        if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
            sharedData.rawIr = localRawIr;
            sharedData.rawRed = localRawRed;
            sharedData.bpm = localBpm;
            sharedData.spo2 = localSpo2;
            
            if (bmi160Enabled) {
                sharedData.isFall = localGyro.isFall;
                sharedData.accelX = localGyro.accelX_g;
                sharedData.accelY = localGyro.accelY_g;
                sharedData.accelZ = localGyro.accelZ_g;
                sharedData.accelMag = localGyro.accel_g;
                sharedData.gyroX = localGyro.gyroX_dps;
                sharedData.gyroY = localGyro.gyroY_dps;
                sharedData.gyroZ = localGyro.gyroZ_dps;
            }
            xSemaphoreGive(dataMutex);
        }

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// --- FreeRTOS Task: OLED Updates at 2Hz ---
void vDisplayTask(void *pvParameters) {
    float localBpm = 0.0f;
    float localSpo2 = 0.0f;
    bool localIsFall = false;
    uint32_t localRawIr = 0;

    for (;;) {
        // 1. Copy shared data locally (requires data mutex)
        if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            localBpm = sharedData.bpm;
            localSpo2 = sharedData.spo2;
            localIsFall = sharedData.isFall;
            localRawIr = sharedData.rawIr;
            xSemaphoreGive(dataMutex);
        }

        // 2. Update OLED (requires I2C access)
        if (oledEnabled) {
            if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                if (ownerUID == "" || ownerUID == "null") {
                    String msg = "Pairing Code:\n" + pairingCode;
                    oled.showMessage(msg.c_str());
                } else if (bmi160Enabled && localIsFall) {
                    oled.showEmergency();
                } else if (max30102Enabled && localRawIr < 20000) {
                    oled.showMessage("Please wear \nthe device...");
                } else {
                    oled.showVitals(localBpm, localSpo2);
                }
                xSemaphoreGive(i2cMutex);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

// --- FreeRTOS Task: Firebase Uploads & Network Polling ---
void vFirebaseUploadTask(void *pvParameters) {
    float localBpm = 0.0f;
    float localSpo2 = 0.0f;
    bool localIsFall = false;
    float localAccelX = 0.0f;
    float localAccelY = 0.0f;
    float localAccelZ = 0.0f;
    float localAccelMag = 1.0f;
    float localGyroX = 0.0f;
    float localGyroY = 0.0f;
    float localGyroZ = 0.0f;
    uint32_t localRawIr = 0;
    uint32_t localRawRed = 0;

    for (;;) {
        // 1. Manage Wi-Fi / Firebase connection state
        if (WiFi.status() == WL_CONNECTED) {
            if (!firebaseReady) {
                Serial.println("[Firebase] Initializing Firebase dynamically...");
                setupFirebase();
            }
            checkDeviceStatus();
        }

        // 2. Copy sensor data locally
        if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            localBpm = sharedData.bpm;
            localSpo2 = sharedData.spo2;
            localIsFall = sharedData.isFall;
            localAccelX = sharedData.accelX;
            localAccelY = sharedData.accelY;
            localAccelZ = sharedData.accelZ;
            localAccelMag = sharedData.accelMag;
            localGyroX = sharedData.gyroX;
            localGyroY = sharedData.gyroY;
            localGyroZ = sharedData.gyroZ;
            localRawIr = sharedData.rawIr;
            localRawRed = sharedData.rawRed;
            xSemaphoreGive(dataMutex);
        }

        // 3. Print debug logs
        Serial.printf("[DEBUG Task] Peripherals: [OLED:%s] [MAX30102:%s] [BMI160:%s (0x%02X)]\n",
                      oledEnabled ? "OK" : "FAILED",
                      max30102Enabled ? "OK" : "FAILED/MISSING",
                      bmi160Enabled ? "OK" : "FAILED",
                      activeBmi160Addr);
        Serial.printf("[DEBUG Task] BPM: %.2f, SpO2: %.2f, Fall: %d | Accel: X=%.2fg, Y=%.2fg, Z=%.2fg, Mag=%.2fg | Gyro: X=%.2f, Y=%.2f, Z=%.2f | Raw IR: %u, Raw Red: %u | Owner: %s\n",
                      localBpm, localSpo2, localIsFall, localAccelX, localAccelY, localAccelZ, localAccelMag,
                      localGyroX, localGyroY, localGyroZ, localRawIr, localRawRed, ownerUID.c_str());

        // 4. Upload data asynchronously to Firebase
        if (firebaseReady && Firebase.ready() && ownerUID != "" && ownerUID != "null") {
            FirebaseJson json;
            json.set("bpm", localBpm);
            json.set("spo2", localSpo2);
            json.set("fall_detected", localIsFall);
            json.set("accel_x", localAccelX);
            json.set("accel_y", localAccelY);
            json.set("accel_z", localAccelZ);
            json.set("accel_magnitude", localAccelMag);
            json.set("gyro_x", localGyroX);
            json.set("gyro_y", localGyroY);
            json.set("gyro_z", localGyroZ);
            json.set("timestamp", (int)millis());
            
            String uploadPath = "/users/" + ownerUID + "/devices_data/" + deviceUID + "/sensor_data";
            if (Firebase.setJSONAsync(fbdo_upload, uploadPath, json)) {
                Serial.println("[Firebase Task] Async upload started.");
            } else {
                Serial.printf("[Firebase Task Error] %s\n", fbdo_upload.errorReason().c_str());
            }
        }

        vTaskDelay(pdMS_TO_TICKS(1500));
    }
}

void setup() {
    // Turn off WiFi immediately at startup to conserve power
    WiFi.disconnect(true, true);
    WiFi.mode(WIFI_OFF);
    delay(200);

    Serial.begin(115200);
    unsigned long serialStart = millis();
    while (!Serial && (millis() - serialStart < 3000)) {
        delay(10);
    }
    delay(3000); // Allow USB CDC connection to enumerate on host

    Serial.println("Booting GuardPulse v2 (RTOS Version)...");

    // Initialize I2C bus
    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(400000);
    Wire.setTimeOut(50); // Prevent I2C calls from hanging

    if (oled.begin(&Wire)) {
        Serial.println("OLED initialized.");
        oledEnabled = true;
    } else {
        Serial.println("OLED initialization failed.");
    }    

    // Initialize MAX30102
    if (hrSensor.begin(Wire, 400000)) {
        Serial.println("MAX30102 initialized.");
        max30102Enabled = true;
    } else {
        Serial.println("MAX30102 initialization failed.");
    }

    // Try BMI160 at address 0x68. If that fails, try address 0x69.
    bool bmiSuccess = fallSensor.begin(&Wire, 0x68);
    if (bmiSuccess) {
        activeBmi160Addr = 0x68;
    } else {
        bmiSuccess = fallSensor.begin(&Wire, 0x69);
        if (bmiSuccess) {
            activeBmi160Addr = 0x69;
        }
    }
    if (bmiSuccess) {
        Serial.println("BMI160 initialized.");
        bmi160Enabled = true;
        fallSensor.setPowerModes(BMI160_ACCEL_LOWPOWER_MODE, BMI160_GYRO_SUSPEND_MODE);
    } else {
        Serial.println("BMI160 initialization failed.");
    }

    // Connect to WiFi
    setupWiFi();
    
    String rawMac = WiFi.macAddress();
    rawMac.replace(":", "");
    rawMac.toUpperCase();
    deviceUID = rawMac;
    if (deviceUID.length() >= 6) {
        pairingCode = deviceUID.substring(deviceUID.length() - 6);
    } else {
        pairingCode = deviceUID;
    }
    Serial.print("Device UID: ");
    Serial.println(deviceUID);
    Serial.print("Pairing Code: ");
    Serial.println(pairingCode);

    if (WiFi.status() == WL_CONNECTED) {
        setupFirebase();
        checkDeviceStatus();
    }

    // Create FreeRTOS Mutexes
    i2cMutex = xSemaphoreCreateMutex();
    dataMutex = xSemaphoreCreateMutex();

    if (i2cMutex == NULL || dataMutex == NULL) {
        Serial.println("Fatal Error: Could not create FreeRTOS semaphores.");
        while (1) {
            delay(1000);
        }
    }

    // Spawn FreeRTOS Tasks
    xTaskCreatePinnedToCore(vSensorTask, "SensorTask", 4096, NULL, 2, NULL, 0);
    xTaskCreatePinnedToCore(vDisplayTask, "DisplayTask", 4096, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(vFirebaseUploadTask, "UploadTask", 8192, NULL, 1, NULL, 0);

    Serial.println("FreeRTOS Scheduler successfully configured and tasks spawned.");
}

void loop() {
    if (firebaseReady) {
        Firebase.ready();
    }

    // Self-healing I2C recovery check every 1 second (runs on main loopTask)
    static unsigned long lastI2CCheck = 0;
    if (millis() - lastI2CCheck > 1000) {
        lastI2CCheck = millis();
        
        uint8_t checkAddr = 0;
        if (oledEnabled) {
            checkAddr = 0x3C;
        } else if (bmi160Enabled) {
            checkAddr = activeBmi160Addr;
        } else if (max30102Enabled) {
            checkAddr = 0x57;
        }
        
        if (checkAddr != 0) {
            // Lock the I2C mutex to safely suspend other I2C tasks during recovery
            if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                Wire.beginTransmission(checkAddr);
                uint8_t err = Wire.endTransmission();
                if (err != 0 && err != 2) { // 2 is NACK (not present)
                    Serial.printf("[I2C loop] Bus lock detected (err=%d). Performing recovery...\n", err);
                    Wire.end();
                    recoverI2CBus(I2C_SDA, I2C_SCL);
                    
                    Wire.begin(I2C_SDA, I2C_SCL);
                    Wire.setClock(400000);
                    Wire.setTimeOut(50);
                    
                    // Re-initialize active devices
                    if (oledEnabled) {
                        oled.begin(&Wire);
                    }
                    if (max30102Enabled) {
                        hrSensor.begin(Wire, 400000);
                    }
                    if (bmi160Enabled) {
                        if (fallSensor.begin(&Wire, activeBmi160Addr)) {
                            fallSensor.setPowerModes(BMI160_ACCEL_LOWPOWER_MODE, BMI160_GYRO_SUSPEND_MODE);
                        }
                    }
                }
                xSemaphoreGive(i2cMutex);
            }
        }
    }

    // Yield control to let low-priority tasks run
    vTaskDelay(pdMS_TO_TICKS(100));
}
