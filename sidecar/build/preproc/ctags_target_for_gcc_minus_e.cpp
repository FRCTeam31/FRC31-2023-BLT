# 1 "C:\\Users\\prime\\Documents\\Projects\\FRC31-2023\\sidecar\\sidecar.ino"
# 2 "C:\\Users\\prime\\Documents\\Projects\\FRC31-2023\\sidecar\\sidecar.ino" 2
# 3 "C:\\Users\\prime\\Documents\\Projects\\FRC31-2023\\sidecar\\sidecar.ino" 2
# 4 "C:\\Users\\prime\\Documents\\Projects\\FRC31-2023\\sidecar\\sidecar.ino" 2
# 5 "C:\\Users\\prime\\Documents\\Projects\\FRC31-2023\\sidecar\\sidecar.ino" 2

// LED variables ======================
// Pins
uint8_t LED_OUTPUT_PIN = 10;

// LED counts
uint8_t LED_COUNT = 13;

// Strip objects
NeoPatterns ledStrip(LED_COUNT, LED_OUTPUT_PIN, ((1 << 6) | (1 << 4) | (0 << 2) | (2)) /*|< Transmit as G,R,B*/ + 0x0000 /*|< 800 KHz data transmission*/);

// Variable to hold our current LED modes
byte currentLEDMode = 0;
// ====================================

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
