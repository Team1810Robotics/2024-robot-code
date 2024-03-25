#include <FastLED.h>
constexpr int PIN_8 = 8;
constexpr int PIN_9 = 9;

constexpr int LED_PINS[] = {2, 3, 4, 5, 6};
constexpr int NUMBER_OF_PINS = sizeof(LED_PINS) / sizeof(LED_PINS[0]);

constexpr int LED_STRIP_LENGTH = 8;

const int NUM_LEDS = (NUMBER_OF_PINS * LED_STRIP_LENGTH);

enum LEDState {
  off       = 0b00,
  isAligned = 0b01,
  noTarget  = 0b10,
  hasTarget = 0b11
};

CRGB leds[NUM_LEDS]{};

struct Color {
  int r, g, b;
};

void setup() {
  pinMode(PIN_9, INPUT_PULLUP);
  pinMode(PIN_8, INPUT_PULLUP);

  Serial.begin(9600);

  FastLED.addLeds<WS2812B, LED_PINS[0], RGB>(leds, 0,  LED_STRIP_LENGTH);
  FastLED.addLeds<WS2812B, LED_PINS[1], RGB>(leds, 8,  LED_STRIP_LENGTH);
  FastLED.addLeds<WS2812B, LED_PINS[2], RGB>(leds, 16, LED_STRIP_LENGTH);
  FastLED.addLeds<WS2812B, LED_PINS[3], RGB>(leds, 24, LED_STRIP_LENGTH);
  FastLED.addLeds<WS2812B, LED_PINS[4], RGB>(leds, 32, LED_STRIP_LENGTH);
}

int getStatus() {
  uint8_t p1 = digitalRead(PIN_8);
  uint8_t p2 = digitalRead(PIN_9);
  delay(100);

  Serial.print("p1: ");
  Serial.print(p1);
  Serial.print(" p2: ");
  Serial.println(p2);

  uint8_t status = (p1 << 1) | (p2);

  return (~status) & 0b11;
}

void solidColor(const Color& color) {
  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i] = CRGB{ color.g, color.r, color.b };
  }
}

void loop() {
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
      solidColor({ 0, 0, 0 }); // black
      break;
  }

  FastLED.show();
}
