#include <Adafruit_NeoPixel.h>
#include <Arduino.h>
#include <Wire.h>

/// NeoPatterns definition =============================================================================
// from https://learn.adafruit.com/multi-tasking-the-arduino-part-3/put-it-all-together-dot-dot-dot

// Pattern types supported:
enum pattern
{
    NONE,
    RAINBOW_CYCLE,
    THEATER_CHASE,
    COLOR_WIPE,
    SCANNER,
    FADE
};
// Patern directions supported:
enum direction
{
    FORWARD,
    REVERSE
};

// NeoPattern Class - derived from the Adafruit_NeoPixel class
class NeoPatterns : public Adafruit_NeoPixel
{
public:
    // Member Variables:
    pattern ActivePattern; // which pattern is running
    direction Direction;   // direction to run the pattern

    unsigned long Interval;   // milliseconds between updates
    unsigned long lastUpdate; // last update of position

    uint32_t Color1, Color2; // What colors are in use
    uint16_t TotalSteps;     // total number of steps in the pattern
    uint16_t Index;          // current step within the pattern

    void (*OnComplete)(); // Callback on completion of pattern

    // Constructor - calls base-class constructor to initialize strip
    NeoPatterns(uint16_t pixels, uint8_t pin, uint8_t type, void (*callback)())
        : Adafruit_NeoPixel(pixels, pin, type)
    {
        OnComplete = callback;
    }

    NeoPatterns(uint16_t pixels, uint8_t pin, uint8_t type)
        : Adafruit_NeoPixel(pixels, pin, type)
    {
    }

    // Update the pattern
    void Update()
    {
        if ((millis() - lastUpdate) > Interval) // time to update
        {
            lastUpdate = millis();
            switch (ActivePattern)
            {
            case RAINBOW_CYCLE:
                RainbowCycleUpdate();
                break;
            case THEATER_CHASE:
                TheaterChaseUpdate();
                break;
            case COLOR_WIPE:
                ColorWipeUpdate();
                break;
            case SCANNER:
                ScannerUpdate();
                break;
            case FADE:
                FadeUpdate();
                break;
            default:
                break;
            }
        }
    }

    // Increment the Index and reset at the end
    void Increment()
    {
        if (Direction == FORWARD)
        {
            Index++;
            if (Index >= TotalSteps)
            {
                Index = 0;
                if (OnComplete != NULL)
                {
                    OnComplete(); // call the comlpetion callback
                }
            }
        }
        else // Direction == REVERSE
        {
            --Index;
            if (Index <= 0)
            {
                Index = TotalSteps - 1;
                if (OnComplete != NULL)
                {
                    OnComplete(); // call the comlpetion callback
                }
            }
        }
    }

    // Reverse pattern direction
    void Reverse()
    {
        if (Direction == FORWARD)
        {
            Direction = REVERSE;
            Index = TotalSteps - 1;
        }
        else
        {
            Direction = FORWARD;
            Index = 0;
        }
    }

    // Initialize for a RainbowCycle
    void RainbowCycle(uint8_t interval, direction dir = FORWARD)
    {
        ActivePattern = RAINBOW_CYCLE;
        Interval = interval;
        TotalSteps = 255;
        Index = 0;
        Direction = dir;
    }

    // Update the Rainbow Cycle Pattern
    void RainbowCycleUpdate()
    {
        for (int i = 0; i < numPixels(); i++)
        {
            setPixelColor(i, Wheel(((i * 256 / numPixels()) + Index) & 255));
        }
        show();
        Increment();
    }

    // Initialize for a Theater Chase
    void TheaterChase(uint32_t color1, uint32_t color2, uint8_t interval, direction dir = FORWARD)
    {
        ActivePattern = THEATER_CHASE;
        Interval = interval;
        TotalSteps = numPixels();
        Color1 = color1;
        Color2 = color2;
        Index = 0;
        Direction = dir;
    }

    // Update the Theater Chase Pattern
    void TheaterChaseUpdate()
    {
        for (int i = 0; i < numPixels(); i++)
        {
            if ((i + Index) % 3 == 0)
            {
                setPixelColor(i, Color1);
            }
            else
            {
                setPixelColor(i, Color2);
            }
        }
        show();
        Increment();
    }

    // Initialize for a ColorWipe
    void ColorWipe(uint32_t color, uint8_t interval, direction dir = FORWARD)
    {
        ActivePattern = COLOR_WIPE;
        Interval = interval;
        TotalSteps = numPixels();
        Color1 = color;
        Index = 0;
        Direction = dir;
    }

    // Update the Color Wipe Pattern
    void ColorWipeUpdate()
    {
        setPixelColor(Index, Color1);
        show();
        Increment();
    }

    // Initialize for a SCANNNER
    void Scanner(uint32_t color1, uint8_t interval)
    {
        ActivePattern = SCANNER;
        Interval = interval;
        TotalSteps = (numPixels() - 1) * 2;
        Color1 = color1;
        Index = 0;
    }

    // Update the Scanner Pattern
    void ScannerUpdate()
    {
        for (int i = 0; i < numPixels(); i++)
        {
            if (i == Index) // Scan Pixel to the right
            {
                setPixelColor(i, Color1);
            }
            else if (i == TotalSteps - Index) // Scan Pixel to the left
            {
                setPixelColor(i, Color1);
            }
            else // Fading tail
            {
                setPixelColor(i, DimColor(getPixelColor(i)));
            }
        }
        show();
        Increment();
    }

    // Initialize for a Fade
    void Fade(uint32_t color1, uint32_t color2, uint16_t steps, uint8_t interval, direction dir = FORWARD)
    {
        ActivePattern = FADE;
        Interval = interval;
        TotalSteps = steps;
        Color1 = color1;
        Color2 = color2;
        Index = 0;
        Direction = dir;
    }

    // Update the Fade Pattern
    void FadeUpdate()
    {
        // Calculate linear interpolation between Color1 and Color2
        // Optimise order of operations to minimize truncation error
        uint8_t red = ((Red(Color1) * (TotalSteps - Index)) + (Red(Color2) * Index)) / TotalSteps;
        uint8_t green = ((Green(Color1) * (TotalSteps - Index)) + (Green(Color2) * Index)) / TotalSteps;
        uint8_t blue = ((Blue(Color1) * (TotalSteps - Index)) + (Blue(Color2) * Index)) / TotalSteps;

        ColorSet(Color(red, green, blue));
        show();
        Increment();
    }

    // Calculate 50% dimmed version of a color (used by ScannerUpdate)
    uint32_t DimColor(uint32_t color)
    {
        // Shift R, G and B components one bit to the right
        uint32_t dimColor = Color(Red(color) >> 1, Green(color) >> 1, Blue(color) >> 1);
        return dimColor;
    }

    // Set all pixels to a color (synchronously)
    void ColorSet(uint32_t color)
    {
        for (int i = 0; i < numPixels(); i++)
        {
            setPixelColor(i, color);
        }
        show();
    }

    // Returns the Red component of a 32-bit color
    uint8_t Red(uint32_t color)
    {
        return (color >> 16) & 0xFF;
    }

    // Returns the Green component of a 32-bit color
    uint8_t Green(uint32_t color)
    {
        return (color >> 8) & 0xFF;
    }

    // Returns the Blue component of a 32-bit color
    uint8_t Blue(uint32_t color)
    {
        return color & 0xFF;
    }

    // Input a value 0 to 255 to get a color value.
    // The colours are a transition r - g - b - back to r.
    uint32_t Wheel(byte WheelPos)
    {
        WheelPos = 255 - WheelPos;
        if (WheelPos < 85)
        {
            return Color(255 - WheelPos * 3, 0, WheelPos * 3);
        }
        else if (WheelPos < 170)
        {
            WheelPos -= 85;
            return Color(0, WheelPos * 3, 255 - WheelPos * 3);
        }
        else
        {
            WheelPos -= 170;
            return Color(WheelPos * 3, 255 - WheelPos * 3, 0);
        }
    }
};

// =====================================================================================================

// Ping Sensor variables ==============
uint8_t TRIG = 58;
uint8_t ECHO1 = 57;
uint8_t ECHO2 = 56;
uint8_t ECHO3 = 55;
uint8_t ECHO4 = 54;

const long THRESHOLD_MM = 500; // Threshold for max distance to the ground before the sensor reads "over-cliff"
long distance1 = -1;
long distance2 = -1;
long distance3 = -1;
long distance4 = -1;
// ====================================

// LED variables ======================
// Pins
uint8_t FRONT_STRIP_OUTPUT_PIN = 10;
uint8_t INDICATOR_STRIP_OUTPUT_PIN = 11;
uint8_t ACCENT_STRIP_RIGHT_OUTPUT_PIN = 12;
uint8_t ACCENT_STRIP_LEFT_OUTPUT_PIN = 13;

// LED counts
uint8_t FRONT_STRIP_LED_COUNT = 40;
uint8_t INDICATOR_STRIP_LED_COUNT = 24;
uint8_t ACCENT_STRIP_LED_COUNT = 48;

// Strip objects
NeoPatterns frontStrip(FRONT_STRIP_LED_COUNT, FRONT_STRIP_OUTPUT_PIN, NEO_GRB + NEO_KHZ800);
NeoPatterns indicatorStrip(INDICATOR_STRIP_LED_COUNT, INDICATOR_STRIP_OUTPUT_PIN, NEO_GRB + NEO_KHZ800);
NeoPatterns accentStripRight(ACCENT_STRIP_LED_COUNT, ACCENT_STRIP_RIGHT_OUTPUT_PIN, NEO_GRB + NEO_KHZ800);
NeoPatterns accentStripLeft(ACCENT_STRIP_LED_COUNT, ACCENT_STRIP_LEFT_OUTPUT_PIN, NEO_GRB + NEO_KHZ800);

// Variable to hold our current LED modes
byte currentLEDModes[4] = {'0', '0', '0', '0'};
// ====================================

void pingSensorPollEvent()
{
    Serial.println("> Received request");

    // Check each distance to see if we're over a cliff and if so, return "0"; else, "1"
    Wire.write(distance1 > THRESHOLD_MM ? "1" : "0");
    Wire.write(distance2 > THRESHOLD_MM ? "1" : "0");
    Wire.write(distance3 > THRESHOLD_MM ? "1" : "0");
    Wire.write(distance4 > THRESHOLD_MM ? "1" : "0");

    // Send the sensor daata by flushing the buffer
    Wire.flush();
    Serial.print("> Flushed reply");
}

long pingSensorPulseDistanceMM(uint8_t echoPin)
{
    long usDuration, distance;

    // Clears the trigger
    digitalWrite(TRIG, LOW);
    delayMicroseconds(2);

    // Sets the trigger on HIGH state for 10 micro seconds
    digitalWrite(TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG, LOW);

    // Reads the echo, returns the sound wave travel time in microseconds
    usDuration = pulseInLong(echoPin, HIGH);

    // Calculating the distance in mm
    distance = (usDuration / 2) / 2.91;

    return distance;
}

void setModeToStrip(NeoPatterns strip, byte mode)
{
    switch (mode)
    {
    default:
    case 0:
        /* code */
        break;
    case 1:
        // SOLID GREEN
        frontStrip.ColorSet(frontStrip.Green());
        break;
    case 2:
        // GIMMEACONE (yellow)
        indicatorStrip.ColorSet(frontStrip.Color(255, 255, 0));
        break;
    case 3:
        // GIMMEACUBE (purple)
        indicatorStrip.ColorSet(frontStrip.Color(255, 0, 255));
        break;
    case 4:
        /* code */
        break;
    case 5:
        /* code */
        break;
    case 6:
        /* code */
        break;
    case 7:
        /* code */
        break;
    case 8:
        /* code */
        break;
    case 9:
        /* code */
        break;
    }
}

void ledStripUpdateEvent(int byteCount)
{
    // Read the available characters from the I2C message
    byte msg[byteCount];
    for (int i = 0; i < byteCount; i++)
    {
        if (Wire.available() <= 1)
            break;

        msg[i] = Wire.read();
    }

    // If we have 4 bytes or less, try to fill the current modes variable
    // with as many new modes as we can
    if (byteCount <= 4 && strlen(msg) <= 4)
    {
        for (int i = 0; i < byteCount; i++)
        {
            currentLEDModes[i] = msg[i];
        }

        // Show the new modes on the LED strips
        setModeToStrip(frontStrip, msg[0]);
        setModeToStrip(indicatorStrip, msg[1]);
        setModeToStrip(accentStripRight, msg[2]);
        setModeToStrip(accentStripLeft, msg[3]);
    }

    /* Anything else that should happen when we receive a new LED update goes here. */
}

void setup()
{
    Serial.begin(115200);
    Wire.begin(10); // Join I2C bus as #10

    // Ping sensor setup ================
    pinMode(TRIG, OUTPUT);
    pinMode(ECHO1, INPUT);
    pinMode(ECHO2, INPUT);
    pinMode(ECHO3, INPUT);
    pinMode(ECHO4, INPUT);

    Wire.onRequest(pingSensorPollEvent); // Register the polling request from BLT
    // ==================================

    // LED setup ========================
    // Start up the LED strips
    frontStrip.begin();
    indicatorStrip.begin();
    accentStripRight.begin();
    accentStripLeft.begin();

    // Set strip modes
    setModeToStrip(frontStrip, currentLEDModes[0]);
    setModeToStrip(indicatorStrip, currentLEDModes[1]);
    setModeToStrip(accentStripRight, currentLEDModes[2]);
    setModeToStrip(accentStripLeft, currentLEDModes[3]);
    //===================================

    Serial.println(">> Started.");
}

void loop()
{
    // Ping sensor code =================
    distance1 = pingSensorPulseDistanceMM(ECHO1);
    distance2 = pingSensorPulseDistanceMM(ECHO2);
    distance3 = pingSensorPulseDistanceMM(ECHO3);
    distance4 = pingSensorPulseDistanceMM(ECHO4);
    delay(1);
    // ==================================
    // LED strip code ===================
    frontStrip.Update();
    indicatorStrip.Update();
    accentStripRight.Update();
    accentStripLeft.Update();
    //===================================
}