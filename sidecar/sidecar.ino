#include <Adafruit_NeoPixel.h>
#include <Arduino.h>
#include <Wire.h>
#include <NeoPatterns.hpp>

// LED variables ======================
// Pins
uint8_t LED_OUTPUT_PIN = 13;

// LED counts
uint8_t LED_COUNT = 44;

// Strip objects
NeoPatterns ledStrip(LED_COUNT, LED_OUTPUT_PIN, NEO_GRB + NEO_KHZ800);

// Variable to hold our current LED modes
String currentMode = "DISABLED";
// ====================================

void setup()
{
    Serial.begin(115200);
    // LED setup ========================
    // Start up the LED strip
    ledStrip.begin();
    ledStrip.printConnectionInfo(&Serial);
    ledStrip.clear();
    ledStrip.show();

    // Set initial mode
    setMode();
    //===================================

    Serial.println(">> Started.");
}

void loop()
{
    // LED strip code ===================
    ledStrip.updateAndShowAlsoAllPartialPatterns();
    // ledStrip.show();
    //===================================
    if (!ledStrip.isActive())
        setMode();
}

void serialEvent()
{
    if (Serial.available() == 0)
        return;

    String newMode = Serial.readString();
    Serial.println("Received new LED mode: " + newMode);

    if (newMode != currentMode)
    {
        currentMode = newMode;
        setMode();
    }
}

void setMode()
{
    if (currentMode == "DISABLED")
    {
        // SOLID RED
        ledStrip.Flash(COLOR32(255, 0, 0), 50, COLOR32(5, 0, 0), 1000, 1);
    }
    else if (currentMode == "AUTO")
    {
        ledStrip.Flash(COLOR32(100, 100, 0), 100, COLOR32(10, 10, 0), 100, 1);
    }
    else if (currentMode == "TELEOP")
    {
        ledStrip.ScannerExtended(COLOR32(0, 0, 255), LED_COUNT, 10, 0, FLAG_SCANNER_EXT_START_AT_BOTH_ENDS | FLAG_SCANNER_EXT_VANISH_COMPLETE, DIRECTION_UP);
    }
    else if (currentMode == "SCAN_OUT")
    {
        ledStrip.ScannerExtended(COLOR32(255, 0, 0), LED_COUNT, 20, 1, FLAG_SCANNER_EXT_CYLON, DIRECTION_DOWN);
    }
    else if (currentMode == "SCAN_DOWN")
    {
        ledStrip.BouncingBall(COLOR32(random(200, 255), 0, random(0, 10)), random(LED_COUNT - 4, LED_COUNT), 40, random(15, 60));
    }
    else if (currentMode == "SCAN_UP")
    {
        ledStrip.ScannerExtended(COLOR32(0, 0, 255), LED_COUNT, 20, 1, FLAG_SCANNER_EXT_VANISH_COMPLETE, DIRECTION_DOWN);
    }
    else
    {
        ledStrip.Flash(COLOR32(255, 0, 0), 50, COLOR32(50, 50, 50), 500, 1);
    }
}