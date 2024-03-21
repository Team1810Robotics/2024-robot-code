#include <FastLED.h>
constexpr int PIN_8 = 8;
constexpr int PIN_9 = 9;

constexpr int SHORT_LED_PIN[] = {2, 3, 4, 5, 6};
constexpr int LONG_LED_PIN = 7;

constexpr int LONG_STRIP_LENGTH = 18;
constexpr int SHORT_STRIP_LENGTH = 4;

const int NUM_LEDS =
  LONG_STRIP_LENGTH +
  (5 * SHORT_STRIP_LENGTH);

enum LEDState {
  hasTarget,
  noTarget,
  isAligned,
  off
};

CRGB leds[NUM_LEDS]{};

struct Color {
  int r, g, b;
};

void setup() {
  pinMode(PIN_9, INPUT_PULLUP);
  pinMode(PIN_8, INPUT_PULLUP);

  FastLED.addLeds<WS2812B, LONG_LED_PIN,     RGB>(leds, 0, 18);
  FastLED.addLeds<WS2812B, SHORT_LED_PIN[0], RGB>(leds, 18, 4);
  FastLED.addLeds<WS2812B, SHORT_LED_PIN[1], RGB>(leds, 22, 4);
  FastLED.addLeds<WS2812B, SHORT_LED_PIN[2], RGB>(leds, 26, 4);
  FastLED.addLeds<WS2812B, SHORT_LED_PIN[3], RGB>(leds, 30, 4);
  FastLED.addLeds<WS2812B, SHORT_LED_PIN[4], RGB>(leds, 34, 4);
}

int getStatus() {
  uint8_t p1 = digitalRead(PIN_8);
  uint8_t p2 = digitalRead(PIN_9);
  delay(100);

  uint8_t status = (p1 << 1) | (p2);

  return (~status) & 0b11;
}

void solidColor(const Color& color) {
  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i] = CRGB{ color.g, color.r, color.b };
  }
}

void loop() {
  Serial.println(getStatus());
  switch (getStatus()) {
    case hasTarget:
      solidColor({ 0, 255, 0 }); // green
      break;
    case noTarget:
      solidColor({ 255, 0, 0 }); // red
      break;
    case isAligned:
      solidColor({ 255, 0, 255 }); // purple
      break;
    case off:
      solidColor({ 0, 0, 0}); // black
      break;
  }

  FastLED.show();
}
