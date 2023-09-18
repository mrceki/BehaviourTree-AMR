#define USE_HWCDC // For ESP32-S3

#include <ros.h>
#include <std_msgs/Int32.h>
#include "FastLED.h"

#define NUM_LEDS 100
#define LED_PIN D10
#define BRIGHTNESS 50

CRGB leds[NUM_LEDS];
ros::NodeHandle nh;
std_msgs::Int32 light_msg;

int currentMode = -1;

void light_msg_subCB(const std_msgs::Int32& msg) {
  light_msg.data = msg.data;
  if (light_msg.data != currentMode) {
    currentMode = light_msg.data;
  }
}

ros::Subscriber<std_msgs::Int32> light_msg_sub("toggle_led", light_msg_subCB);

void changeMode(int mode) {
  if (currentMode != mode) {
    currentMode = mode;
  }
}

void showStrip() {
  FastLED.show();
}

void switchMode(int mode) {
  switch (mode) {
    case 0:
      idleMode();
      break;
    case 1:
      taskMode();
      break;
    case 2:
      warningMode();
      break;
    case 3:
      runningMode(0, 255, 0, 10);
      break;
  }
}

void loop() {
  nh.spinOnce();
  switchMode(currentMode);
}

void runningMode(byte red, byte green, byte blue, int WaveDelay) {
  int Position = 0;
  for (int j = 0; j < NUM_LEDS; j++) {
    Position++;

    for (int i = 0; i < NUM_LEDS; i++) {
      if (i < NUM_LEDS / 2) {
        setPixel(i, 0, ((sin((i + Position) / 4) * 105 + 150) / 255) * green, 0);
      } else {
        setPixel(i, 0, ((sin((i - Position) / 4) * 105 + 150) / 255) * green, 0);
      }
    }
    showStrip();
    delay(WaveDelay);
    nh.spinOnce();
    if (currentMode != 3) {
      break;
    }
  }
}

void fadeInOut(int firstCounter, int lastCounter, int multiplier, bool colors[], int stepDelay, int mode){
  // Fade IN
  for (int k = firstCounter; k < lastCounter; k++) {
    if (k % 15 == 0) {
      nh.spinOnce();
    }
    if (currentMode != mode) {
      break;
    }
    setAll(k * colors[0] * multiplier, k * colors[1] * multiplier, k * colors[2]* multiplier);
    delay(stepDelay);
  }

  // Fade OUT
  for (int k = lastCounter -1; k >= firstCounter; k--) {
    if (k % 15 == 0) {
      nh.spinOnce();
    }
    if (currentMode != mode) {
      break;
    }
    setAll(k * colors[0] * multiplier, k * colors[1] * multiplier, k * colors[2]*  multiplier);
    delay(stepDelay);
  }
}

void idleMode() {
  bool colors[3] = {0,1,0};
  fadeInOut(50, 256, 1, colors, 5, 0);
}

void taskMode() {
  bool colors[3] = {1,1,0};
  fadeInOut(30, 128, 2, colors, 1, 1);
}

void warningMode() {
  bool colors[3] = {1,0,0};
  fadeInOut(20, 84, 3, colors, 1, 2);
}

void setPixel(int Pixel, byte red, byte green, byte blue) {
  leds[Pixel].r = red;
  leds[Pixel].g = green;
  leds[Pixel].b = blue;
}

void setAll(byte red, byte green, byte blue) {
  for (int i = 0; i < NUM_LEDS; i++) {
    setPixel(i, red, green, blue);
  }
  showStrip();
}

void setup() {
  nh.initNode();
  nh.subscribe(light_msg_sub);

  FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
  FastLED.setBrightness(BRIGHTNESS);
}
