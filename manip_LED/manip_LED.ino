#include <FastLED.h>
constexpr int PIN_8 = 8;
constexpr int PIN_9 = 9;

constexpr int LED_PINS[] = {2, 3, 4, 5, 6};
constexpr int NUMBER_OF_PINS = sizeof(LED_PINS) / sizeof(LED_PINS[0]);

constexpr int LED_STRIP_LENGTH = 8;

const int NUM_LEDS = (NUMBER_OF_PINS * LED_STRIP_LENGTH);

#define BRETT_MODE false

#define BLACK { 0, 0, 0 }
#define PURPLE { 255, 0, 255 }
#define RED { 255, 0, 0 }
#define GREEN { 0, 255, 0 }
#define BLUE { 0, 0, 255 }
#define YELLOW { 255, 255, 0 }

Color OFF         = BLACK;
Color NO_TARGET   = RED;
Color HAS_TARGET  = (BRETT_MODE) ? YELLOW : GREEN;
Color IS_ALIGNED  = (BRETT_MODE) ? BLUE   : PURPLE;

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
      solidColor(HAS_TARGET);
      break;
    case noTarget:
      solidColor(NO_TARGET);
      break;
    case isAligned:
      solidColor(IS_ALIGNED);
      break;
    case off:
      solidColor(OFF);
      break;
  }

  FastLED.show();
}
