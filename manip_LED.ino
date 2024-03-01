#include <FastLED.h>
const int PIN_2 = 8;
const int PIN_3 = 9;
const int PIN_4 = 10;
const int LED_PIN = 5;

const int NUM_LEDS = 11;

#define SEC (millis() / 1000.0f)

bool half = false;
float lastTime = SEC;

CRGB leds[NUM_LEDS]{};

struct Color {
  int r, g, b;

  Color(int r, int g, int b) : r{r}, g{g}, b{b} {}
};

void setup() {
  pinMode(PIN_2, INPUT_PULLUP);
  pinMode(PIN_4, INPUT_PULLUP);
  pinMode(PIN_3, INPUT_PULLUP);

  Serial.begin(9600);

  FastLED.addLeds<WS2812B, LED_PIN, RGB>(leds, NUM_LEDS);
}

int getStatus() {
  uint8_t p1 = digitalRead(PIN_2);
  uint8_t p2 = digitalRead(PIN_3);
  uint8_t p3 = digitalRead(PIN_4);
  delay(100);

  uint8_t status = (p1 << 2) | (p2 << 1) | (p3);

  return (~status) & 0x07;
}

void solidColor(const Color& color) {
  for (int i = 0; i < NUM_LEDS; i++){
    leds[i] = CRGB{color.g , color.r, color.b};
  } 
}

void swap(const Color& firstHalf, const Color& secondHalf) {
  if (SEC - lastTime >= 0.25) {
    half = !half;
    lastTime = SEC;
  }

  if (half) {
    partialColor(firstHalf, secondHalf);
  } else {
    partialColor(secondHalf, firstHalf);
  }
}

void partialColor(const Color& firstHalf, const Color& secondHalf){
  for (int i = 0; i < NUM_LEDS / 2; i++){
    leds[i] = CRGB{firstHalf.g, firstHalf.r, firstHalf.b};
  }
  for (int i = NUM_LEDS / 2; i < NUM_LEDS; i++){
    leds[i] = CRGB{secondHalf.g, secondHalf.r, secondHalf.b};
  }

  
}
void loop() {
  Serial.println(getStatus());
  switch (getStatus()) {
    case 0b000:
      solidColor({255, 0 , 0}); // set leds to red
      break;
    case 0b010:
     solidColor({0, 255 , 0}); // set leds to green
      break;
    case 0b101:
       solidColor({255, 0 , 255}); // set leds to purple
      break;
    case 0b110:
      solidColor({255, 255 , 255}); // set leds to white
      break;
    case 0b111:
       solidColor({0, 0, 0}); // set leds to black/off
      break;
    case 0b001:
      swap({255, 0 ,0}, {0, 0, 255});// flashes red and blue
      break;
       case 0b011:
      swap({255, 0 ,0}, {0, 0, 0});// flashes red and black/off
      break;
       case 0b100:
      swap({0, 255 ,0}, {0, 0, 0});// flashes green and black/off
      break;
  }
 
  FastLED.show();

}
