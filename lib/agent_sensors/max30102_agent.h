#ifndef AGENT_SENSORS_MAX30102_AGENT_H
#define AGENT_SENSORS_MAX30102_AGENT_H

#include <Arduino.h>
#include <Wire.h>

class Max30102Agent {
public:
    Max30102Agent();
    
    // Initialize the sensor
    bool begin(TwoWire &wirePort = Wire, uint32_t i2cSpeed = 100000);
    
    // Read latest raw IR and Red values from FIFO
    bool readRaw(uint32_t &ir, uint32_t &red);

    // Getters for latest values
    uint32_t getRawIR();
    uint32_t getRawRed();
    int getBufferLength();

    // Legacy methods returning raw values
    float getBPM();
    float getSpO2();

private:
    uint32_t lastIrValue;
    uint32_t lastRedValue;
    uint32_t irBuffer[100];
    uint32_t redBuffer[100];
    int bufferLength;

    int newSamplesCount;
    int fingerAbsentSamples;
    float lastSpo2;
    int32_t lastBpm;

    // Filter states
    float dc_ir;
    float dc_red;
    float ac_ir_fil;
    float ac_red_fil;
};

#endif // AGENT_SENSORS_MAX30102_AGENT_H

