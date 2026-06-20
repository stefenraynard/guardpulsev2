#ifndef BMI160_AGENT_H
#define BMI160_AGENT_H

#include <Arduino.h>
#include <Wire.h>

// Bosch BMI160 API is pure C — wrap for C++ linkage
extern "C" {
#include "bmi160.h"
}

enum FallState_t {
    STATE_MONITOR_ACCEL,
    STATE_AWAIT_IMPACT,
    STATE_MONITOR_INACTIVITY
};

struct GyroData_t {
    int16_t ax;
    int16_t ay;
    int16_t az;
    int16_t gx;
    int16_t gy;
    int16_t gz;
    float accel_g;
    float accelX_g;
    float accelY_g;
    float accelZ_g;
    float gyroX_dps;
    float gyroY_dps;
    float gyroZ_dps;
    bool isFall;
};

class Bmi160Agent {
public:
    Bmi160Agent();
    bool begin(TwoWire *wire = &Wire, uint8_t i2cAddr = 0x69);
    void readData(GyroData_t *data);
    bool setPowerModes(uint8_t accelPower, uint8_t gyroPower);
    uint8_t getGyroPowerMode() const;
    uint8_t getAccelPowerMode() const;

private:
    struct bmi160_dev bmi160Dev;

    // Static I2C bridge callbacks for Bosch API
    static TwoWire *_wire;
    static int8_t i2cRead(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len);
    static int8_t i2cWrite(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len);
    static void delayMs(uint32_t period);

    // LPF States
    float filtAx, filtAy, filtAz;
    float filtGx, filtGy, filtGz;
    bool lpfInitialized;

    // Fall detection states
    FallState_t _fallState;
    unsigned long _freeFallTime;
    unsigned long _impactTime;
    bool _gyroMovementDetected;
    bool _isFall;
};

#endif

