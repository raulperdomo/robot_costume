// A basic everyday NeoPixel strip test program.

// NEOPIXEL BEST PRACTICES for most reliable operation:
// - Add 1000 uF CAPACITOR between NeoPixel strip's + and - connections.
// - MINIMIZE WIRING LENGTH between microcontroller board and first pixel.
// - NeoPixel strip's DATA-IN should pass through a 300-500 OHM RESISTOR.
// - AVOID connecting NeoPixels on a LIVE CIRCUIT. If you must, ALWAYS
//   connect GROUND (-) first, then +, then data.
// - When using a 3.3V microcontroller with a 5V-powered NeoPixel strip,
//   a LOGIC-LEVEL CONVERTER on the data line is STRONGLY RECOMMENDED.
// (Skipping these may work OK on your workbench but can fail in the field)

#include <Adafruit_NeoPixel.h>
#include <seesaw_neopixel.h>
#include "Adafruit_seesaw.h"
#include <Wire.h>
// Which pin on the Arduino is connected to the NeoPixels?
// On a Trinket or Gemma we suggest changing this to 1:
#define LED_PIN 2

// How many NeoPixels are attached to the Arduino?
#define LED_COUNT 20
#define RED_BUT 10
#define GRN_BUT 11
#define BLU_BUT 12
#define WHT_BUT 13

#define SEESAW_ADDR 0x49

#define SCL 0
#define SDA 1
// Declare our NeoPixel strip object:
Adafruit_NeoPixel strip(LED_COUNT, 18, NEO_GRB + NEO_KHZ800);
//Adafruit_NeoPixel strip1(LED_COUNT, 19, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel ring(12, 17, NEO_GRB + NEO_KHZ800);
// Argument 1 = Number of pixels in NeoPixel strip
// Argument 2 = Arduino pin number (most are valid)
// Argument 3 = Pixel type flags, add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
//   NEO_RGBW    Pixels are wired for RGBW bitstream (NeoPixel RGBW products)


// setup() function -- runs once at startup --------------------------------
uint32_t RED = strip.Color(255, 0, 0);
uint32_t GREEN = strip.Color(0, 255, 0);
uint32_t BLUE = strip.Color(0, 0, 255);
volatile uint32_t ring_color = strip.gamma32(strip.Color(128, 128, 128));
uint32_t last_color = strip.gamma32(strip.Color(128, 128, 128));


//const char state[] = { 'r', 'g', 'b', 'x' };
int ctr = 3;
enum button_pushed {
  NONE,
  bRED,
  bGREEN, 
  bBLUE,
  bWHITE
};

enum strip_states {
  RAINBOW,
  SOLID
};

volatile enum strip_states state = RAINBOW;
volatile enum button_pushed pushed = NONE;

int32_t enc_positions[4] = { 128, 128, 128, 255};
uint8_t maxVal = enc_positions[0];
uint8_t minVal = enc_positions[0];

uint8_t fromLow = 0;
uint8_t fromHigh = 255;

Adafruit_seesaw ss = Adafruit_seesaw(&Wire);
uint8_t addr = 0;
long now = millis();

void chState_red() {

  pushed = bRED;
}
void chState_grn() {

pushed = bGREEN;
}
void chState_blu() {

  pushed = bBLUE;
}
void chState_wht() {

  pushed = bWHITE;
}


void setup() {

  strip.begin();            // INITIALIZE NeoPixel strip object (REQUIRED)
  strip.show();             // Turn OFF all pixels ASAP
  strip.setBrightness(50);  // Set BRIGHTNESS to about 1/5 (max = 255)
  //strip1.begin();            // INITIALIZE NeoPixel strip object (REQUIRED)
  //strip1.show();             // Turn OFF all pixels ASAP
  //strip1.setBrightness(50);  // Set BRIGHTNESS to about 1/5 (max = 255)



  //GPIO 25 set onbord LED HIGH.
  pinMode(25, OUTPUT);
  digitalWrite(25, HIGH);

  //Push Button
  pinMode(RED_BUT, INPUT);
  pinMode(GRN_BUT, INPUT);
  pinMode(BLU_BUT, INPUT);
  pinMode(WHT_BUT, INPUT);



  attachInterrupt(digitalPinToInterrupt(RED_BUT), chState_red, LOW);
  attachInterrupt(digitalPinToInterrupt(GRN_BUT), chState_grn, LOW);
  attachInterrupt(digitalPinToInterrupt(BLU_BUT), chState_blu, LOW);
  attachInterrupt(digitalPinToInterrupt(WHT_BUT), chState_wht, LOW);


  rainbow(10, &strip);
}

void setup1() {
  Serial.begin();
  ring.begin();            // INITIALIZE NeoPixel strip object (REQUIRED)
  ring.show();             // Turn OFF all pixels ASAP
  ring.setBrightness(10);  // Set BRIGHTNESS to about 1/5 (max = 255)
  rainbow(10, &ring);

  if(! ss.begin(SEESAW_ADDR)) {
    colorWipe(ring.Color(165,165,0), 50, &ring);
  }
  else {
    colorWipe(ring.Color(0,0,255), 50, &ring);

  }

}

// loop() function -- runs repeatedly as long as board is on ---------------

void loop() {
  if (state == RAINBOW) {
    rainbow(10, &strip);
  }
  if (pushed != NONE) {

    if (pushed == bRED) {
      colorWipe(RED, 50, &strip);
      //colorWipe(RED, 50, &strip1);
      state = SOLID;

    } else if (pushed == bGREEN) {
      colorWipe(GREEN, 50, &strip);
      //colorWipe(GREEN, 50, &strip1);
      state = SOLID;

    } else if (pushed == bBLUE) {
      colorWipe(BLUE, 50, &strip);
      //colorWipe(BLUE, 50, &strip1);
      state = SOLID;

    } else if (pushed == bWHITE) {
      if(state == SOLID && last_color == ring_color){
        rainbow(10, &strip);
        //rainbow(10, &strip1);
        state = RAINBOW;
      }
      else if(state == RAINBOW || last_color != ring_color){
        colorWipe(ring_color, 50, &strip);
        last_color = ring_color;
        state = SOLID;
      }


    } else {
      colorWipe(strip.Color(255, 165, 0), 50, &strip);
      //colorWipe(strip1.Color(255, 165, 0), 50, &strip1);
      state = SOLID;


    }
    pushed = NONE;  
    }

}

void loop1() {
  //rainbow(10, &ring);
  maxVal = 0;
  minVal = 255;
  for (int enc = 0; enc < 3; enc++){
    int32_t enc_delta = ss.getEncoderDelta(enc);
    if(enc_delta){
      now = millis();
      enc_positions[enc] = enc_positions[enc] + enc_delta * 8;
      if(enc_positions[enc] > 255){
        enc_positions[enc] = 255;
      }
      if(enc_positions[enc] < 0){
        enc_positions[enc] = 0;
      }
    }  
    maxVal = max(maxVal, enc_positions[enc]);
    minVal = min(minVal, enc_positions[enc]);
  }
  int32_t enc_delta = ss.getEncoderDelta(3);
  if(enc_delta){
    now = millis();
    for(int e = 0; e < 3; e++){
      if(maxVal == 0xFF && enc_delta > 0) { break; }
      else if(minVal == 1 && enc_delta < 0) { break; }
      else{
        enc_positions[e] = map(enc_positions[e], fromLow, maxVal, 0, maxVal + (int8_t)(enc_delta)  * 4);
        if (enc_positions[e] > 0xFF){
          enc_positions[e] = 0xFF;
        } else if (enc_positions[e] < 0){
          enc_positions[e] = 1;
        }
      }
    }
  }
  Serial.printf("R: %d, G: %d, B: %d, W: %d, MAX: %d, MIN: %d\n", enc_positions[0], enc_positions[1], enc_positions[2], (int8_t) enc_delta, maxVal, minVal);
  ring.fill(ring.gamma32(ring.Color(enc_positions[0], enc_positions[1], enc_positions[2])), 0, 12);
  ring_color = ring.gamma32(ring.Color(enc_positions[0], enc_positions[1], enc_positions[2]));
  ring.show();   

    //pixels.setPixelColor(0, Wheel((new_enc_position*4) & 0xFF));
  if(millis() - now > 10000 && state == RAINBOW){
    rainbow(10, &ring);
  }

}


// Fill along the length of the strip in various colors...

//delay(1000);



/*
  colorWipe(strip.Color(255,   0,   0), 50); // Red
  colorWipe(strip.Color(  0, 255,   0), 50); // Green
  colorWipe(strip.Color(  0,   0, 255), 50); // Blue

  // Do a theater marquee effect in various colors...
  theaterChase(strip.Color(127, 127, 127), 50); // White, half brightness
  theaterChase(strip.Color(127,   0,   0), 50); // Red, half brightness
  theaterChase(strip.Color(  0,   0, 127), 50); // Blue, half brightness

  rainbow(10);             // Flowing rainbow cycle along the whole strip
  theaterChaseRainbow(50); // Rainbow-enhanced theaterChase variant

  
  colorWipe(strip.Color(255,   0,   0), 50); // Red
  delay(1000);
  colorWipe(strip.Color(  0, 255,   0), 50); // Green
  delay(1000);
  colorWipe(strip.Color(  0,   0, 255), 50); // Blue
  delay(1000);
*/



// Some functions of our own for creating animated effects -----------------
uint32_t Wheel(byte WheelPos) {
  WheelPos = 255 - WheelPos;
  if (WheelPos < 85) {
    return seesaw_NeoPixel::Color(255 - WheelPos * 3, 0, WheelPos * 3);
  }
  if (WheelPos < 170) {
    WheelPos -= 85;
    return seesaw_NeoPixel::Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
  WheelPos -= 170;
  return seesaw_NeoPixel::Color(WheelPos * 3, 255 - WheelPos * 3, 0);
}
// Fill strip pixels one after another with a color. Strip is NOT cleared
// first; anything there will be covered pixel by pixel. Pass in color
// (as a single 'packed' 32-bit value, which you can get by calling
// strip.Color(red, green, blue) as shown in the loop() function above),
// and a delay time (in milliseconds) between pixels.
void colorWipe(uint32_t color, int wait, Adafruit_NeoPixel *device) {
  for (int i = 0; i < strip.numPixels(); i++) {  // For each pixel in strip...
    device->setPixelColor(i, color);               //  Set pixel's color (in RAM)
    device->show();                                //  Update strip to match
    delay(wait);                                 //  Pause for a moment
  }
}

// Theater-marquee-style chasing lights. Pass in a color (32-bit value,
// a la strip.Color(r,g,b) as mentioned above), and a delay time (in ms)
// between frames.
void theaterChase(uint32_t color, int wait) {
  for (int a = 0; a < 10; a++) {   // Repeat 10 times...
    for (int b = 0; b < 3; b++) {  //  'b' counts from 0 to 2...
      strip.clear();               //   Set all pixels in RAM to 0 (off)
      // 'c' counts up from 'b' to end of strip in steps of 3...
      for (int c = b; c < strip.numPixels(); c += 3) {
        strip.setPixelColor(c, color);  // Set pixel 'c' to value 'color'
      }
      strip.show();  // Update strip with new contents
      delay(wait);   // Pause for a moment
    }
  }
}

// Rainbow cycle along whole strip. Pass delay time (in ms) between frames.
void rainbow(int wait, Adafruit_NeoPixel *device) {
  pushed = NONE;
  // Hue of first pixel runs 5 complete loops through the color wheel.
  // Color wheel has a range of 65536 but it's OK if we roll over, so
  // just count from 0 to 5*65536. Adding 256 to firstPixelHue each time
  // means we'll make 5*65536/256 = 1280 passes through this loop:
  for (long firstPixelHue = 0; firstPixelHue < 65536; firstPixelHue += 256) {
    if (pushed != NONE && firstPixelHue > 2000) {
      break;
    }
    // strip.rainbow() can take a single argument (first pixel hue) or
    // optionally a few extras: number of rainbow repetitions (default 1),
    // saturation and value (brightness) (both 0-255, similar to the
    // ColorHSV() function, default 255), and a true/false flag for whether
    // to apply gamma correction to provide 'truer' colors (default true).
    device->rainbow(firstPixelHue);
    // Above line is equivalent to:
    // strip.rainbow(firstPixelHue, 1, 255, 255, true);
    device->show();  // Update strip with new contents
    delay(wait);     // Pause for a moment
  }
}

// Rainbow-enhanced theater marquee. Pass delay time (in ms) between frames.
void theaterChaseRainbow(int wait) {
  int firstPixelHue = 0;           // First pixel starts at red (hue 0)
  for (int a = 0; a < 30; a++) {   // Repeat 30 times...
    for (int b = 0; b < 3; b++) {  //  'b' counts from 0 to 2...
      strip.clear();               //   Set all pixels in RAM to 0 (off)
      // 'c' counts up from 'b' to end of strip in increments of 3...
      for (int c = b; c < strip.numPixels(); c += 3) {
        // hue of pixel 'c' is offset by an amount to make one full
        // revolution of the color wheel (range 65536) along the length
        // of the strip (strip.numPixels() steps):
        int hue = firstPixelHue + c * 65536L / strip.numPixels();
        uint32_t color = strip.gamma32(strip.ColorHSV(hue));  // hue -> RGB
        strip.setPixelColor(c, color);                        // Set pixel 'c' to value 'color'
      }
      strip.show();                 // Update strip with new contents
      delay(wait);                  // Pause for a moment
      firstPixelHue += 65536 / 90;  // One cycle of color wheel over 90 frames
    }
  }
}
