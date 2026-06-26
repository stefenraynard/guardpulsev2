#include "oled.h"

OledDisplay::OledDisplay()
    : display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET) {
}

bool OledDisplay::begin(TwoWire *wire) {
    // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
    if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
        Serial.println("SSD1306 allocation failed!");
        return false;
    }

    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.println("Initializing...");
    display.display();
    delay(1000);

    return true;
}

void OledDisplay::clear() {
    display.clearDisplay();
    display.display();
}

void OledDisplay::showVitals(float bpm, float spo2) {
    display.clearDisplay();

    // --- BPM Section ---
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.println("Heart Rate:");

    display.setTextSize(2);
    display.setCursor(0, 12);
    if (bpm > 0) {
        display.print((int)bpm);
        display.setTextSize(1);
        display.print(" BPM");
    } else {
        display.print("--");
        display.setTextSize(1);
        display.print(" BPM");
    }

    // --- SpO2 Section ---
    display.setTextSize(1);
    display.setCursor(0, 36);
    display.println("SpO2:");

    display.setTextSize(2);
    display.setCursor(0, 48);
    if (spo2 > 0) {
        display.print((int)spo2);
        display.setTextSize(1);
        display.print(" %");
    } else {
        display.print("--");
        display.setTextSize(1);
        display.print(" %");
    }

    display.display();
}

void OledDisplay::showMessage(const char *message) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setCursor(0, 28); // Center vertically
    display.println(message);
    display.display();
}

void OledDisplay::showEmergency() {
    display.clearDisplay();
    display.setTextSize(2);
    display.setCursor(0, 20);
    display.setTextColor(SSD1306_WHITE);
    display.println("EMERGENCY!");
    display.setCursor(0, 45);
    display.println("FALL DETECTED!");
    display.display();
}

void OledDisplay::showReminder(const char *medicine, const char *time) {
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.println("DRUG REMINDER:");
    
    display.setTextSize(2);
    display.setCursor(0, 16);
    display.println(medicine);
    
    display.setTextSize(1);
    display.setCursor(0, 48);
    display.printf("Time: %s", time);
    display.display();
}