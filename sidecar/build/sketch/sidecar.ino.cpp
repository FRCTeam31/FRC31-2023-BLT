#line 1 "C:\\Users\\prime\\Documents\\Projects\\FRC31-2023\\sidecar\\sidecar.ino"
#include <Adafruit_NeoPixel.h>
#include <Arduino.h>
#include <Wire.h>
#include <NeoPatterns.hpp>

// LED variables ======================
// Pins
uint8_t LED_OUTPUT_PIN = 10;

// LED counts
uint8_t LED_COUNT = 13;

// Strip objects
NeoPatterns ledStrip(LED_COUNT, LED_OUTPUT_PIN, NEO_GRB + NEO_KHZ800);

// Variable to hold our current LED modes
byte currentLEDMode = 0;
// ====================================

#line 20 "C:\\Users\\prime\\Documents\\Projects\\FRC31-2023\\sidecar\\sidecar.ino"
void setModeToStrip(byte mode);
#line 50 "C:\\Users\\prime\\Documents\\Projects\\FRC31-2023\\sidecar\\sidecar.ino"
void ledStripUpdateEvent(int byteCount);
#line 64 "C:\\Users\\prime\\Documents\\Projects\\FRC31-2023\\sidecar\\sidecar.ino"
void setup();
#line 81 "C:\\Users\\prime\\Documents\\Projects\\FRC31-2023\\sidecar\\sidecar.ino"
void loop();
#line 20 "C:\\Users\\prime\\Documents\\Projects\\FRC31-2023\\sidecar\\sidecar.ino"
void setModeToStrip(byte mode)
{
    switch (mode)
    {
    default:
    case 0:
        // SOLID RED
        ledStrip.fill(ledStrip.Color(255, 0, 0), 0, LED_COUNT);
        break;
    case 1:
        // AUTO
        break;
    case 2:
        // TELEOP
        break;
    case 3:
        // SCAN_OUT
        break;
    case 4:
        // SCAN_UP
        break;
    case 5:
        // SCAN_DOWN
        break;
    }

    Serial.print(">> Set LED mode to ");
    Serial.println(mode);
}

void ledStripUpdateEvent(int byteCount)
{
    if (byteCount < 1)
    {
        Serial.println(">> [ERROR] I2C message contained 0 bytes");
        return;
    }

    currentLEDMode = Wire.read();
    Serial.print(">> Read value from I2C: ");
    Serial.println(currentLEDMode);
    setModeToStrip(currentLEDMode);
}

void setup()
{
    Serial.begin(115200);
    Wire.begin(10); // Join I2C bus as #10

    // LED setup ========================
    // Start up the LED strip
    ledStrip.begin();
    ledStrip.printConnectionInfo(&Serial);

    // Set strip modes
    setModeToStrip(0);
    //===================================

    Serial.println(">> Started.");
}

void loop()
{
    // LED strip code ===================
    ledStrip.updateAndShowAlsoAllPartialPatterns();
    //===================================
}
