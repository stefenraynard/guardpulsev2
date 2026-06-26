#ifndef AGENT_DISPLAY_OLED_H
#define AGENT_DISPLAY_OLED_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1          // Reset pin (-1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C    // I2C address for 128x64 OLED [[6]] [[9]]

class OledDisplay {
public:
    OledDisplay();

    // Initialize the OLED display
    bool begin(TwoWire *wire = &Wire);

    // Clear the display
    void clear();

    // Display BPM and SpO2 values
    void showVitals(float bpm, float spo2);

    // Display a message (e.g., "No finger detected")
    void showMessage(const char *message);

    void showEmergency();

    void showReminder(const char *medicine, const char *time);

private:
    Adafruit_SSD1306 display;
};

#endif // AGENT_DISPLAY_OLED_H
