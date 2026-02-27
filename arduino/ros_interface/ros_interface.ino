/*
 * ROS Interface for Arduino Lighting & Load Cell
 *
 * Commands:
 * 'r': Red Light
 * 'b': Blue Light
 * 'o': Lights Off
 * 't': Tare Scale
 *
 * Output:
 * DATA:<timestamp>,<weight>
 */

#include "HX711.h"
#include <Adafruit_NeoPixel.h>

// --- CONFIGURATION ---
#define PIN 6
#define NUMPIXELS 3

// WIRING
const int LOADCELL_DOUT_PIN = 2;
const int LOADCELL_SCK_PIN = 3;

float calibration_factor = 1681;
const int DIRECTION_FLIP = 1;

// --- SMOOTHING SETTINGS ---
// (Smoothing removed for raw data)

// --- TIMING ---
const long scaleInterval = 0;

// --- OBJECTS ---
Adafruit_NeoPixel strip(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);
HX711 scale;

unsigned long previousMillisScale = 0;
unsigned long startTime = 0;

// --- COLORS ---
uint32_t blue, red, off;

void setup() {
  Serial.begin(9600);

  strip.begin();
  strip.setBrightness(50);
  strip.show();

  blue = strip.Color(0, 0, 255);
  red = strip.Color(255, 0, 0);
  off = strip.Color(0, 0, 0);

  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  scale.set_scale(calibration_factor);

  // --- AUTO-ZERO ---
  delay(930);
  scale.tare();

  startTime = millis();
}

void loop() {
  // Check for serial commands
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    handleCommand(cmd);
  }

  // Continuous data streaming
  readWeight();
}

void handleCommand(char cmd) {
  if (cmd == 't') {
    scale.tare();
    Serial.println("STATE:TARE_DONE");
  } else if (cmd ==
             's') { // Keep 's' as a safety catch-all for turning off lights
    setStripColor(off);
    Serial.println("STATE:IDLE");
  } else if (cmd == 'r') {
    setStripColor(red);
    Serial.println("STATE:IDLE");
  } else if (cmd == 'b') {
    setStripColor(blue);
    Serial.println("STATE:IDLE");
  } else if (cmd == 'o') {
    setStripColor(off);
    Serial.println("STATE:IDLE");
  }
}

void readWeight() {
  float rawReading = 0.0;
  float finalReading = 0.0;

  if (scale.is_ready()) {
    rawReading = scale.get_units(1);
    finalReading = rawReading * DIRECTION_FLIP;

    // Format: DATA:<time>,<weight>
    unsigned long currentMillis = millis() - startTime;
    float timeSeconds = currentMillis / 1000.0;

    Serial.print("DATA:");
    Serial.print(timeSeconds);
    Serial.print(",");
    Serial.println(finalReading);
  }
}

// 0: OFF, 1: BLUE, 2: RED
void setStripColor(uint32_t color) {
  for (int i = 0; i < strip.numPixels(); i++) {
    strip.setPixelColor(i, color);
  }
  strip.show();

  if (color == blue) {
    Serial.println("LED:BLUE");
  } else if (color == red) {
    Serial.println("LED:RED");
  } else {
    Serial.println("LED:OFF");
  }
}
