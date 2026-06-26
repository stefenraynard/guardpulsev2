#include "bmi160_agent.h"

// Static member initialization
TwoWire* Bmi160Agent::_wire = nullptr;

Bmi160Agent::Bmi160Agent()
    : filtAx(0.0f), filtAy(0.0f), filtAz(1.0f),
      filtGx(0.0f), filtGy(0.0f), filtGz(0.0f),
      lpfInitialized(false),
      _fallState(STATE_MONITOR_ACCEL),
      _freeFallTime(0),
      _impactTime(0),
      _gyroMovementDetected(false),
      _isFall(false) {
    memset(&bmi160Dev, 0, sizeof(bmi160Dev));
}

// --- Bosch API I2C bridge callbacks ---

int8_t Bmi160Agent::i2cRead(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len) {
    if (!_wire) return -1;
    _wire->beginTransmission(dev_addr);
    _wire->write(reg_addr);
    if (_wire->endTransmission(false) != 0) return -1;
    _wire->requestFrom(dev_addr, (uint8_t)len);
    for (uint16_t i = 0; i < len; i++) {
        if (_wire->available()) {
            data[i] = _wire->read();
        } else {
            return -1;
        }
    }
    return 0; // BMI160_OK
}

int8_t Bmi160Agent::i2cWrite(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len) {
    if (!_wire) return -1;
    _wire->beginTransmission(dev_addr);
    _wire->write(reg_addr);
    for (uint16_t i = 0; i < len; i++) {
        _wire->write(data[i]);
    }
    return (_wire->endTransmission() == 0) ? 0 : -1;
}

void Bmi160Agent::delayMs(uint32_t period) {
    delay(period);
}

// --- Public API ---

bool Bmi160Agent::begin(TwoWire *wire, uint8_t i2cAddr) {
    _wire = wire;

    // Configure device handle for I2C
    bmi160Dev.id = i2cAddr;  // BMI160 I2C address (default: 0x69, SDO HIGH)
    bmi160Dev.intf = BMI160_I2C_INTF;
    bmi160Dev.read = &Bmi160Agent::i2cRead;
    bmi160Dev.write = &Bmi160Agent::i2cWrite;
    bmi160Dev.delay_ms = &Bmi160Agent::delayMs;

    // Initialize sensor — reads chip ID and performs soft reset
    int8_t rslt = bmi160_init(&bmi160Dev);
    if (rslt != BMI160_OK) {
        Serial.printf("[BMI160] Init failed: %d\n", rslt);
        return false;
    }

    // Configure accelerometer: ±2g, 100Hz, normal power
    bmi160Dev.accel_cfg.odr = BMI160_ACCEL_ODR_100HZ;
    bmi160Dev.accel_cfg.range = BMI160_ACCEL_RANGE_2G;
    bmi160Dev.accel_cfg.bw = BMI160_ACCEL_BW_NORMAL_AVG4;
    bmi160Dev.accel_cfg.power = BMI160_ACCEL_NORMAL_MODE;

    // Configure gyroscope: ±2000 dps, 100Hz, normal power
    bmi160Dev.gyro_cfg.odr = BMI160_GYRO_ODR_100HZ;
    bmi160Dev.gyro_cfg.range = BMI160_GYRO_RANGE_2000_DPS;
    bmi160Dev.gyro_cfg.bw = BMI160_GYRO_BW_NORMAL_MODE;
    bmi160Dev.gyro_cfg.power = BMI160_GYRO_NORMAL_MODE;

    // Apply configuration
    rslt = bmi160_set_sens_conf(&bmi160Dev);
    if (rslt != BMI160_OK) {
        Serial.printf("[BMI160] Config failed: %d\n", rslt);
        return false;
    }

    return true;
}

void Bmi160Agent::readData(GyroData_t *data) {
    struct bmi160_sensor_data accel = {0};
    struct bmi160_sensor_data gyro = {0};

    uint8_t select = BMI160_ACCEL_SEL;
    if (bmi160Dev.gyro_cfg.power == BMI160_GYRO_NORMAL_MODE) {
        select |= BMI160_GYRO_SEL;
    }

    int8_t rslt = bmi160_get_sensor_data(
        select,
        &accel, &gyro, &bmi160Dev
    );

    if (rslt != BMI160_OK) {
        data->ax = 0;
        data->ay = 0;
        data->az = 0;
        data->gx = 0;
        data->gy = 0;
        data->gz = 0;
        data->accel_g = 1.0f;
        data->accelX_g = 0.0f;
        data->accelY_g = 0.0f;
        data->accelZ_g = 1.0f;
        data->gyroX_dps = 0.0f;
        data->gyroY_dps = 0.0f;
        data->gyroZ_dps = 0.0f;
        data->isFall = _isFall;
        return;
    }

    // Populate data struct with raw values
    data->ax = accel.x;
    data->ay = accel.y;
    data->az = accel.z;
    data->gx = gyro.x;
    data->gy = gyro.y;
    data->gz = gyro.z;

    // Convert raw values to float units (g and dps)
    float ax_raw_g = (float)accel.x / 16384.0f;
    float ay_raw_g = (float)accel.y / 16384.0f;
    float az_raw_g = (float)accel.z / 16384.0f;

    // Calculate raw acceleration magnitude to capture high-speed transient impact spikes
    float raw_accel_mag = sqrt(ax_raw_g * ax_raw_g + ay_raw_g * ay_raw_g + az_raw_g * az_raw_g);

    float gx_raw_dps = (float)gyro.x / 16.4f;
    float gy_raw_dps = (float)gyro.y / 16.4f;
    float gz_raw_dps = (float)gyro.z / 16.4f;

    // Apply simple Low Pass Filter (EMA LPF)
    if (!lpfInitialized) {
        filtAx = ax_raw_g;
        filtAy = ay_raw_g;
        filtAz = az_raw_g;
        filtGx = gx_raw_dps;
        filtGy = gy_raw_dps;
        filtGz = gz_raw_dps;
        lpfInitialized = true;
    } else {
        const float alpha_accel = 0.3f;
        filtAx = alpha_accel * ax_raw_g + (1.0f - alpha_accel) * filtAx;
        filtAy = alpha_accel * ay_raw_g + (1.0f - alpha_accel) * filtAy;
        filtAz = alpha_accel * az_raw_g + (1.0f - alpha_accel) * filtAz;

        const float alpha_gyro = 0.2f;
        if (bmi160Dev.gyro_cfg.power == BMI160_GYRO_NORMAL_MODE) {
            filtGx = alpha_gyro * gx_raw_dps + (1.0f - alpha_gyro) * filtGx;
            filtGy = alpha_gyro * gy_raw_dps + (1.0f - alpha_gyro) * filtGy;
            filtGz = alpha_gyro * gz_raw_dps + (1.0f - alpha_gyro) * filtGz;
        } else {
            filtGx = 0.0f;
            filtGy = 0.0f;
            filtGz = 0.0f;
        }
    }

    // Calculate magnitude of acceleration
    float accel_mag = sqrt(filtAx * filtAx + filtAy * filtAy + filtAz * filtAz);

    // Populate filtered values
    data->accel_g = accel_mag;
    data->accelX_g = filtAx;
    data->accelY_g = filtAy;
    data->accelZ_g = filtAz;
    data->gyroX_dps = filtGx;
    data->gyroY_dps = filtGy;
    data->gyroZ_dps = filtGz;

    unsigned long now = millis();

    // FALL DETECTION STATE MACHINE
    switch (_fallState) {
        case STATE_MONITOR_ACCEL:
            // 1. Accel only is monitored. Check for low gravity (free fall < 0.5g)
            if (accel_mag < 0.5f) {
                _freeFallTime = now;
                _fallState = STATE_AWAIT_IMPACT;
                _isFall = false; // Reset if we start a new detection sequence
            }
            break;

        case STATE_AWAIT_IMPACT:
            // 2. Free fall detected, wait for impact (>= 1.8g)
            if (now - _freeFallTime > 2000) {
                // Timeout, no impact occurred within 2.0s
                _fallState = STATE_MONITOR_ACCEL;
            } else if (raw_accel_mag >= 1.8f) {
                // Impact detected! Wake up gyro to collect IMU data
                setPowerModes(BMI160_ACCEL_LOWPOWER_MODE, BMI160_GYRO_NORMAL_MODE);
                _impactTime = now;
                _gyroMovementDetected = false;
                _fallState = STATE_MONITOR_INACTIVITY;
            }
            break;

        case STATE_MONITOR_INACTIVITY:
            // 3. Gyro collects IMU data. Monitor inactivity for 5.0 seconds.
            if (now - _impactTime <= 5000) {
                // Introduce a 500ms settling delay to ignore landing bounce/vibration
                if (now - _impactTime > 500) {
                    // Check if any gyro axis registers significant movement (> 80 dps)
                    float gx_abs = fabsf(filtGx);
                    float gy_abs = fabsf(filtGy);
                    float gz_abs = fabsf(filtGz);
                    if (gx_abs > 80.0f || gy_abs > 80.0f || gz_abs > 80.0f) {
                        _gyroMovementDetected = true;
                    }
                }
            } else {
                // 5.0 seconds elapsed. If NOT much movement was detected:
                if (!_gyroMovementDetected) {
                    _isFall = true;
                    Serial.println("\n[ALERT] !!! FALL DETECTED !!! (Subject is inactive after impact)");
                } else {
                    Serial.println("\n[INFO] Motion detected post-impact. Fall alert cancelled.");
                }
                // Disable gyro and return to low power monitoring mode
                setPowerModes(BMI160_ACCEL_LOWPOWER_MODE, BMI160_GYRO_SUSPEND_MODE);
                _fallState = STATE_MONITOR_ACCEL;
            }
            break;
    }

    data->isFall = _isFall;
}

bool Bmi160Agent::setPowerModes(uint8_t accelPower, uint8_t gyroPower) {
    bmi160Dev.accel_cfg.power = accelPower;
    bmi160Dev.gyro_cfg.power = gyroPower;
    int8_t rslt = bmi160_set_power_mode(&bmi160Dev);
    return rslt == BMI160_OK;
}

uint8_t Bmi160Agent::getGyroPowerMode() const {
    return bmi160Dev.gyro_cfg.power;
}

uint8_t Bmi160Agent::getAccelPowerMode() const {
    return bmi160Dev.accel_cfg.power;
}

void Bmi160Agent::clearFall() {
    _isFall = false;
    _fallState = STATE_MONITOR_ACCEL;
}

