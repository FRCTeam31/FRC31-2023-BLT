#include <Arduino.h>
#include <Wire.h>

uint8_t TRIG1 = 46;
uint8_t ECHO1 = 47;

uint8_t TRIG2 = 44;
uint8_t ECHO2 = 45;

uint8_t TRIG3 = 42;
uint8_t ECHO3 = 43;

uint8_t TRIG4 = 40;
uint8_t ECHO4 = 41;

const long THRESHOLD_MM = 500; // Threshold for max distance to the ground before the sensor reads "over-cliff"
long distance1 = -1;
long distance2 = -1;
long distance3 = -1;
long distance4 = -1;

void setup()
{
    pinMode(TRIG1, OUTPUT);
    pinMode(ECHO1, INPUT);
    pinMode(TRIG2, OUTPUT);
    pinMode(ECHO2, INPUT);
    pinMode(TRIG3, OUTPUT);
    pinMode(ECHO3, INPUT);
    pinMode(TRIG4, OUTPUT);
    pinMode(ECHO4, INPUT);

    Wire.begin(10);               // Join I2C bus as #10
    Wire.onRequest(requestEvent); // Register the polling request function

    Serial.begin(115200);
    Serial.println(">> Started.");
}

void loop()
{
    distance1 = pulseDistanceMM(TRIG1, ECHO1);
    distance2 = pulseDistanceMM(TRIG2, ECHO2);
    distance3 = pulseDistanceMM(TRIG3, ECHO3);
    distance4 = pulseDistanceMM(TRIG4, ECHO4);
    delay(10);
}

void requestEvent()
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

long pulseDistanceMM(uint8_t trigPin, uint8_t echoPin)
{
    long usDuration, distance;

    // Clears the trig
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);

    // Sets the trig on HIGH state for 10 micro seconds
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    // Reads the echo, returns the sound wave travel time in microseconds
    usDuration = pulseInLong(echoPin, HIGH);

    // Calculating the distance in mm
    distance = (usDuration / 2) / 2.91;

    return distance;
}
