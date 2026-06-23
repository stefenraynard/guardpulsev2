#include "max30102_agent.h"
#include "max30102.h"
#include "algorithm_by_RF.h"

Max30102Agent::Max30102Agent()
    : lastIrValue(0), lastRedValue(0), bufferLength(0),
      newSamplesCount(0), fingerAbsentSamples(0), lastSpo2(-1.0f), lastBpm(-1),
      dc_ir(0.0f), dc_red(0.0f), ac_ir_fil(0.0f), ac_red_fil(0.0f) {
    memset(irBuffer, 0, sizeof(irBuffer));
    memset(redBuffer, 0, sizeof(redBuffer));
}

bool Max30102Agent::begin(TwoWire &wirePort, uint32_t i2cSpeed) {
    wirePort.setClock(i2cSpeed);

    // I2C scan: check if MAX30102 is visible at 0x57
    wirePort.beginTransmission(0x57);
    uint8_t scanErr = wirePort.endTransmission();
    Serial.printf("[MAX30102] I2C scan 0x57: %s (err=%d)\n", scanErr == 0 ? "FOUND" : "NOT FOUND", scanErr);
    if (scanErr != 0) {
        Serial.println("[MAX30102] FATAL: Sensor not on I2C bus! Check wiring.");
        return false;
    }
    
    // 1. Reset device
    Serial.println("[MAX30102] Resetting...");
    if (!maxim_max30102_write_reg(0x09, 0x40)) { Serial.println("[MAX30102] FAIL: reset write"); return false; }
    delay(100);
    // Wait for reset bit to clear
    uint8_t modeReg = 0xFF;
    for (int t = 0; t < 50; t++) {
        maxim_max30102_read_reg(0x09, &modeReg);
        if ((modeReg & 0x40) == 0) break;
        delay(10);
    }
    Serial.printf("[MAX30102] Mode reg after reset: 0x%02X\n", modeReg);
    
    // 2. Read and clear interrupt registers
    uint8_t dummy = 0;
    maxim_max30102_read_reg(0x00, &dummy);
    maxim_max30102_read_reg(0x01, &dummy);
    
    // 3. Configure Interrupt Enables (0x02=INTR_ENABLE_1, 0x03=INTR_ENABLE_2)
    if (!maxim_max30102_write_reg(0x02, 0xC0)) { Serial.println("[MAX30102] FAIL: intr enable 1"); return false; }
    if (!maxim_max30102_write_reg(0x03, 0x00)) { Serial.println("[MAX30102] FAIL: intr enable 2"); return false; }
    
    // 4. Reset FIFO pointers
    if (!maxim_max30102_write_reg(0x04, 0x00)) { Serial.println("[MAX30102] FAIL: fifo wr ptr"); return false; }
    if (!maxim_max30102_write_reg(0x05, 0x00)) { Serial.println("[MAX30102] FAIL: ovf counter"); return false; }
    if (!maxim_max30102_write_reg(0x06, 0x00)) { Serial.println("[MAX30102] FAIL: fifo rd ptr"); return false; }
    
    // 5. FIFO Configuration (sample avg=4, rollover ENABLED, almost full=15)
    // Rollover MUST be enabled: without it, FIFO fills during WiFi connect, wrPtr wraps
    // back to rdPtr=0, numSamples=(0-0)&31=0, and FIFO appears empty forever.
    if (!maxim_max30102_write_reg(0x08, 0x5F)) { Serial.println("[MAX30102] FAIL: fifo config"); return false; }
    
    // 6. Mode Configuration (SpO2 mode)
    if (!maxim_max30102_write_reg(0x09, 0x03)) { Serial.println("[MAX30102] FAIL: mode config"); return false; }
    
    // 7. SpO2 Configuration (ADC range 16384nA, 100Hz sample rate, 215us pulse width)
    if (!maxim_max30102_write_reg(0x0A, 0x66)) { Serial.println("[MAX30102] FAIL: spo2 config"); return false; }
    
    // 8. LED current (Red set to 0xDF (~44mA) for deep penetration, IR set to 0x9F (~32mA) to balance signals)
    if (!maxim_max30102_write_reg(0x0C, 0xDF)) { Serial.println("[MAX30102] FAIL: LED1 PA"); return false; }
    if (!maxim_max30102_write_reg(0x0D, 0x9F)) { Serial.println("[MAX30102] FAIL: LED2 PA"); return false; }
    
    // 9. Pilot PA
    if (!maxim_max30102_write_reg(0x10, 0x7F)) { Serial.println("[MAX30102] FAIL: pilot PA"); return false; }

    // Verify: read back part ID (should be 0x15 for MAX30102)
    uint8_t partId = 0;
    maxim_max30102_read_reg(0xFF, &partId);
    Serial.printf("[MAX30102] Part ID: 0x%02X (expected 0x15)\n", partId);

    // Verify mode register
    uint8_t modeCheck = 0;
    maxim_max30102_read_reg(0x09, &modeCheck);
    Serial.printf("[MAX30102] Mode reg: 0x%02X (expected 0x03)\n", modeCheck);
    
    Serial.println("[MAX30102] Init complete.");
    return true;
}

bool Max30102Agent::readRaw(uint32_t &ir, uint32_t &red) {
    uint8_t wrPtr = 0;
    uint8_t rdPtr = 0;

    bool wrOk = maxim_max30102_read_reg(0x04, &wrPtr);
    bool rdOk = maxim_max30102_read_reg(0x06, &rdPtr);

    /*
    // Print FIFO pointer values every 2 seconds for diagnosis
    static unsigned long lastPtrLog = 0;
    if (millis() - lastPtrLog > 2000) {
        lastPtrLog = millis();
        // Also read interrupt status and mode register for full picture
        uint8_t intr1 = 0, modeReg = 0, ovf = 0;
        maxim_max30102_read_reg(0x00, &intr1);
        maxim_max30102_read_reg(0x05, &ovf);
        maxim_max30102_read_reg(0x09, &modeReg);
        Serial.printf("[MAX30102 DIAG] wrPtr=%d rdPtr=%d ovf=%d intr1=0x%02X mode=0x%02X wrOk=%d rdOk=%d\n",
                      wrPtr, rdPtr, ovf, intr1, modeReg, wrOk, rdOk);
    }
    */

    if (!wrOk || !rdOk) {
        Serial.println("[MAX30102] ERROR: Failed to read FIFO pointers");
        ir = lastIrValue;
        red = lastRedValue;
        return false;
    }

    int numSamples = (wrPtr - rdPtr) & 31;
    if (numSamples <= 0) {
        ir = lastIrValue;
        red = lastRedValue;
        return false;
    }
    
    bool gotNewSamples = false;
    for (int i = 0; i < numSamples; i++) {
        uint32_t tempRed = 0;
        uint32_t tempIr = 0;
        if (maxim_max30102_read_fifo(&tempRed, &tempIr)) {
            gotNewSamples = true;
            if (tempIr > 250000 || tempRed > 250000) {
                Serial.println("[WARNING] MAX30102 saturated! Raw readings too high.");
            }
            lastIrValue = tempIr;
            lastRedValue = tempRed;
            
            if (tempIr > 15000) {
                fingerAbsentSamples = 0;
                
                if (dc_ir == 0.0f) {
                    dc_ir = (float)tempIr;
                }
                if (dc_red == 0.0f) {
                    dc_red = (float)tempRed;
                }
                
                dc_ir = 0.95f * dc_ir + 0.05f * tempIr;
                dc_red = 0.95f * dc_red + 0.05f * tempRed;
                
                float raw_ac_ir = (float)tempIr - dc_ir;
                float raw_ac_red = (float)tempRed - dc_red;
                
                ac_ir_fil = 0.5f * ac_ir_fil + 0.5f * raw_ac_ir;
                ac_red_fil = 0.5f * ac_red_fil + 0.5f * raw_ac_red;
                
                // No decimation: 100Hz sensor / 4 avg = 25Hz FIFO output = FS=25 expected by RF algorithm
                if (bufferLength < 100) {
                    irBuffer[bufferLength] = (uint32_t)(ac_ir_fil + 100000.0f);
                    redBuffer[bufferLength] = (uint32_t)(ac_red_fil + 100000.0f);
                    bufferLength++;
                } else {
                    for (int j = 1; j < 100; j++) {
                        irBuffer[j - 1] = irBuffer[j];
                        redBuffer[j - 1] = redBuffer[j];
                    }
                    irBuffer[99] = (uint32_t)(ac_ir_fil + 100000.0f);
                    redBuffer[99] = (uint32_t)(ac_red_fil + 100000.0f);
                }
                
                newSamplesCount++;
                if (bufferLength == 100 && newSamplesCount >= 10) {
                    newSamplesCount = 0;
                    float n_spo2 = 0.0f;
                    int8_t ch_spo2_valid = 0;
                    int32_t n_heart_rate = 0;
                    int8_t ch_hr_valid = 0;
                    float ratio = 0.0f;
                    float correl = 0.0f;
                    
                    uint32_t cleanIrBuffer[100];
                    uint32_t cleanRedBuffer[100];
                    for (int i = 0; i < 100; i++) {
                        cleanIrBuffer[i] = (uint32_t)((int32_t)irBuffer[i] - 100000 + (int32_t)dc_ir);
                        cleanRedBuffer[i] = (uint32_t)((int32_t)redBuffer[i] - 100000 + (int32_t)dc_red);
                    }
                    
                    rf_heart_rate_and_oxygen_saturation(cleanIrBuffer, 100, cleanRedBuffer, &n_spo2, &ch_spo2_valid, &n_heart_rate, &ch_hr_valid, &ratio, &correl);
                    
                    Serial.printf("[RF debug] Correlation: %.3f (min: 0.8), Ratio: %.3f, Raw HR: %d (valid: %d), Raw SpO2: %.2f (valid: %d)\n",
                                  correl, ratio, n_heart_rate, ch_hr_valid, n_spo2, ch_spo2_valid);
                    
                    if (ch_hr_valid) {
                        if (lastBpm == 0) {
                            lastBpm = n_heart_rate;
                        } else {
                            lastBpm = (int32_t)(0.8f * (float)lastBpm + 0.2f * (float)n_heart_rate);
                        }
                    }
                    if (ch_spo2_valid) {
                        if (lastSpo2 == 0.0f) {
                            lastSpo2 = n_spo2;
                        } else {
                            lastSpo2 = 0.8f * lastSpo2 + 0.2f * n_spo2;
                        }
                    }
                }
            } else {
                dc_ir = 0.0f;
                dc_red = 0.0f;
                ac_ir_fil = 0.0f;
                ac_red_fil = 0.0f;
                fingerAbsentSamples++;
                if (fingerAbsentSamples > 75) { // 3 seconds at 25 Hz
                    bufferLength = 0;
                    lastBpm = 0;
                    lastSpo2 = 0.0f;
                }
            }
        }
    }
    
    ir = lastIrValue;
    red = lastRedValue;
    return gotNewSamples;
}

uint32_t Max30102Agent::getRawIR() {
    return lastIrValue;
}

uint32_t Max30102Agent::getRawRed() {
    return lastRedValue;
}

int Max30102Agent::getBufferLength() {
    return bufferLength;
}

float Max30102Agent::getBPM() {
    if (lastIrValue >= 15000) {
        return (float)lastBpm;
    } else {
        return 0.0f;
    }
}

float Max30102Agent::getSpO2() {
    if (lastIrValue >= 15000) {
        return lastSpo2;
    } else {
        return 0.0f;
    }
}

