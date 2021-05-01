#include <FastLED.h>

// How many leds in your strip?
#define NUM_LEDS 7

// For led chips like WS2812, which have a data line, ground, and power, you just
// need to define DATA_PIN.  For led chipsets that are SPI based (four wires - data, clock,
// ground, and power), like the LPD8806 define both DATA_PIN and CLOCK_PIN
// Clock pin only needed for SPI based chipsets when not using hardware SPI

#define LED_DATA_PIN 26
#define LED_CLOCK_PIN 27

// Define the array of leds
CRGB leds[NUM_LEDS];

void setup()
{
    // Uncomment/edit one of the following lines for your leds arrangement.
    // ## Clocked (SPI) types ##

    // FastLED.addLeds<APA102, LED_DATA_PIN, LED_CLOCK_PIN, RGB>(leds, NUM_LEDS);  // BGR ordering is typical
    FastLED.addLeds<SK9822, LED_DATA_PIN, LED_CLOCK_PIN, BGR>(leds, NUM_LEDS); // BGR ordering is typical
}

void loop()
{
    // Turn the LED on, then pause
    for (size_t i = 0; i < NUM_LEDS; i++)
    {
        leds[i] = CRGB::Red;
        FastLED.show();
        delay(20);
    }
    delay(700);

    // Now turn the LED off, then pause
    for (size_t i = 0; i < NUM_LEDS; i++)
    {
        leds[i] = CRGB::Black;
        FastLED.show();
        delay(30);
    }

    delay(700);
}