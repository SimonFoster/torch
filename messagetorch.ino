#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
  #include <avr/power.h>
#endif

#define PIN 6
#define NUM_LEDS 300
#define LEDS_PER_ROW  23
#define NUM_ROWS 12

// Parameter 1 = number of pixels in strip
// Parameter 2 = Arduino pin number (most are valid)
// Parameter 3 = pixel type flags, add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
//   NEO_RGBW    Pixels are wired for RGBW bitstream (NeoPixel RGBW products)
Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_LEDS, PIN, NEO_GRB + NEO_KHZ800);

// IMPORTANT: To reduce NeoPixel burnout risk, add 1000 uF capacitor across
// pixel power leads, add 300 - 500 Ohm resistor on first pixel's data input
// and minimize distance between Arduino and first pixel.  Avoid connecting
// on a live circuit...if you must, connect GND first.

void setup() {
  strip.begin();
  strip.show(); // Initialize all pixels to 'off'
  resetEnergy();
}

// Configuration
// =============

// set to true if you wound the torch clockwise (as seen from top). Note that
// this reverses the entire animation (in contrast to mirrorText, which only
// mirrors text).
const bool reversedX = false;
// set to true if every other row in the LED matrix is ordered backwards.
// This mode is useful for WS2812 modules which have e.g. 16x16 LEDs on one
// flexible PCB. On these modules, the data line starts in the lower left
// corner, goes right for row 0, then left in row 1, right in row 2 etc.
const bool alternatingX = false;
// set to true if your WS2812 chain runs up (or up/down, with alternatingX set) the "torch",
// for example if you want to do a wide display out of multiple 16x16 arrays
const bool swapXY = false;


// Set this to true if you wound the LED strip clockwise, starting at the bottom of the
// tube, when looking onto the tube from the top. The default winding direction
// for versions of MessageTorch which did not have this setting was 0, which
// means LED strip was usually wound counter clock wise from bottom to top.
// Note: this setting only reverses the direction of text rendering - the torch
//   animation itself is not affected
const bool mirrorText = false;

/*
 * Spark Core library to control WS2812 based RGB LED devices
 * using SPI to create bitstream.
 * Future plan is to use DMA to feed the SPI, so WS2812 bitstream
 * can be produced without CPU load and without blocking IRQs
 *
 * (c) 2014-2015 by luz@plan44.ch (GPG: 1CC60B3A)
 * Licensed as open source under the terms of the MIT License
 * (see LICENSE.TXT)
 */


// Declaration (would go to .h file once library is separated)
// ===========================================================

class p44_ws2812 {

  bool xReversed; // even (0,2,4...) rows go backwards, or all if not alternating
  bool alternating; // direction changes after every row
  bool swapXY; // x and y swapped


public:
  /// create driver for a WS2812 LED chain
  /// @param aNumLeds number of LEDs in the chain
  /// @param aLedsPerRow number of consecutive LEDs in the WS2812 chain that build a row (usually x direction, y if swapXY was set)
  /// @param aXReversed X direction is reversed
  /// @param aAlternating X direction is reversed in first row, normal in second, reversed in third etc..
  /// @param aSwapXY X and Y reversed (for up/down wiring)
  p44_ws2812(uint16_t aNumLeds, uint16_t aLedsPerRow=0, bool aXReversed=false, bool aAlternating=false, bool aSwapXY=false);

  /// destructor
  ~p44_ws2812();

  /// begin using the driver
  void begin();

  /// transfer RGB values to LED chain
  /// @note this must be called to update the actual LEDs after modifying RGB values
  /// with setColor() and/or setColorDimmed()
  void show();


  /// set color of one LED
  /// @param aRed intensity of red component, 0..255
  /// @param aGreen intensity of green component, 0..255
  /// @param aBlue intensity of blue component, 0..255
  void setColorXY(uint16_t aX, uint16_t aY, byte aRed, byte aGreen, byte aBlue);
  void setColor(uint16_t aLedNumber, byte aRed, byte aGreen, byte aBlue);

  /// set color of one LED, scaled by a visible brightness (non-linear) factor
  /// @param aRed intensity of red component, 0..255
  /// @param aGreen intensity of green component, 0..255
  /// @param aBlue intensity of blue component, 0..255
  /// @param aBrightness brightness, will be converted non-linear to PWM duty cycle for uniform brightness scale, 0..255
  void setColorDimmedXY(uint16_t aX, uint16_t aY, byte aRed, byte aGreen, byte aBlue, byte aBrightness);
  void setColorDimmed(uint16_t aLedNumber, byte aRed, byte aGreen, byte aBlue, byte aBrightness);

  /// get current color of LED
  /// @param aRed set to intensity of red component, 0..255
  /// @param aGreen set to intensity of green component, 0..255
  /// @param aBlue set to intensity of blue component, 0..255
  /// @note for LEDs set with setColorDimmed(), this returns the scaled down RGB values,
  ///   not the original r,g,b parameters. Note also that internal brightness resolution is 5 bits only.
  void getColorXY(uint16_t aX, uint16_t aY, byte &aRed, byte &aGreen, byte &aBlue);
  void getColor(uint16_t aLedNumber, byte &aRed, byte &aGreen, byte &aBlue);

  
private:

  uint16_t ledIndexFromXY(uint16_t aX, uint16_t aY);


};

// Implementation (would go to .cpp file once library is separated)
// ================================================================

static const uint8_t pwmTable[32] = {0, 1, 1, 2, 3, 4, 6, 7, 9, 10, 13, 15, 18, 21, 24, 28, 33, 38, 44, 50, 58, 67, 77, 88, 101, 115, 132, 150, 172, 196, 224, 255};

p44_ws2812::p44_ws2812(uint16_t aNumLeds, uint16_t aLedsPerRow, bool aXReversed, bool aAlternating, bool aSwapXY)
{
}

p44_ws2812::~p44_ws2812()
{
  // free the buffer
  // if (pixelBufferP) delete pixelBufferP;
}

uint16_t p44_ws2812::ledIndexFromXY(uint16_t aX, uint16_t aY)
{
  if (swapXY) { uint16_t tmp=aY; aY=aX; aX=tmp; }
  uint16_t ledindex = aY*LEDS_PER_ROW;
  bool reversed = xReversed;
  if (alternating) {
    if (aY & 0x1) reversed = !reversed;
  }
  if (reversed) {
    ledindex += (LEDS_PER_ROW-1-aX);
  }
  else {
    ledindex += aX;
  }
  return ledindex;
}


void p44_ws2812::setColor(uint16_t aLedNumber, byte aRed, byte aGreen, byte aBlue)
{
  int y = aLedNumber / LEDS_PER_ROW;
  int x = aLedNumber % LEDS_PER_ROW;
  setColorXY(x, y, aRed, aGreen, aBlue);
}


void p44_ws2812::setColorXY(uint16_t aX, uint16_t aY, byte aRed, byte aGreen, byte aBlue)
{
  uint16_t ledindex = ledIndexFromXY(aX,aY);
  if (ledindex>=NUM_LEDS) return;
  strip.setPixelColor(ledindex, aRed, aGreen, aBlue);
}


void p44_ws2812::setColorDimmed(uint16_t aLedNumber, byte aRed, byte aGreen, byte aBlue, byte aBrightness)
{
  int y = aLedNumber / LEDS_PER_ROW;
  int x = aLedNumber % LEDS_PER_ROW;
  setColorDimmedXY(x, y, aRed, aGreen, aBlue, aBrightness);
}


void p44_ws2812::setColorDimmedXY(uint16_t aX, uint16_t aY, byte aRed, byte aGreen, byte aBlue, byte aBrightness)
{
  setColorXY(aX, aY, (aRed*aBrightness)>>8, (aGreen*aBrightness)>>8, (aBlue*aBrightness)>>8);
}

void p44_ws2812::getColorXY(uint16_t aX, uint16_t aY, byte &aRed, byte &aGreen, byte &aBlue)
{
  uint16_t ledindex = ledIndexFromXY(aX,aY);
  if (ledindex>=NUM_LEDS) return;
  uint32_t color = strip.getPixelColor(ledindex);
  aRed   = (color >> 16) & 0xFF;
  aGreen = (color >> 8 ) & 0xFF;
  aBlue  = (color >> 0 ) & 0xFF;
}



// Utilities
// =========



inline void reduce(byte &aByte, byte aAmount, byte aMin = 0)
{
  int r = aByte-aAmount;
  if (r<aMin)
    aByte = aMin;
  else
    aByte = (byte)r;
}


inline void increase(byte &aByte, byte aAmount, byte aMax = 255)
{
  int r = aByte+aAmount;
  if (r>aMax)
    aByte = aMax;
  else
    aByte = (byte)r;
}


// Main program, torch simulation
// ==============================

// moved defining constants for number of LEDs to top of file

p44_ws2812 leds(NUM_LEDS, swapXY ? NUM_ROWS : LEDS_PER_ROW, reversedX, alternatingX, swapXY); // create WS2812 driver


// byte mode = mode_torch; // main operation mode
int brightness = 100; // overall brightness
// byte fade_base = 140; // crossfading base brightness level

// torch parameters

uint16_t cycle_wait = 1; // 0..255

byte flame_min = 100; // 0..255
byte flame_max = 220; // 0..255

byte random_spark_probability = 2; // 0..100
byte spark_min = 200; // 0..255
byte spark_max = 255; // 0..255

byte spark_tfr = 40; // 0..256 how much energy is transferred up for a spark per cycle
uint16_t spark_cap = 200; // 0..255: spark cells: how much energy is retained from previous cycle

uint16_t up_rad = 20; // up radiation
uint16_t side_rad = 35; // sidewards radiation
uint16_t heat_cap = 0; // 0..255: passive cells: how much energy is retained from previous cycle

byte red_bg = 0;
byte green_bg = 0;
byte blue_bg = 0;
byte red_bias = 10;
byte green_bias = 0;
byte blue_bias = 0;
int red_energy = 180;
int green_energy = 80; // 145;
int blue_energy = 0;

byte upside_down = 0; // if set, flame (or rather: drop) animation is upside down. Text remains as-is

// torch mode
// ==========

byte currentEnergy[NUM_LEDS]; // current energy level
byte nextEnergy[NUM_LEDS]; // next energy level
byte energyMode[NUM_LEDS]; // mode how energy is calculated for this point

enum {
  torch_passive = 0, // just environment, glow from nearby radiation
  torch_nop = 1, // no processing
  torch_spark = 2, // slowly looses energy, moves up
  torch_spark_temp = 3, // a spark still getting energy from the level below
};



void resetEnergy()
{
  for (int i=0; i<NUM_LEDS; i++) {
    currentEnergy[i] = 0;
    nextEnergy[i] = 0;
    energyMode[i] = torch_passive;
  }
}




void calcNextEnergy()
{
  int i = 0;
  for (int y=0; y<NUM_ROWS; y++) {
    for (int x=0; x<LEDS_PER_ROW; x++) {
      byte e = currentEnergy[i];
      byte m = energyMode[i];
      switch (m) {
        case torch_spark: {
          // loose transfer up energy as long as the is any
          reduce(e, spark_tfr);
          // cell above is temp spark, sucking up energy from this cell until empty
          if (y<NUM_ROWS-1) {
            energyMode[i+LEDS_PER_ROW] = torch_spark_temp;
          }
          break;
        }
        case torch_spark_temp: {
          // just getting some energy from below
          byte e2 = currentEnergy[i-LEDS_PER_ROW];
          if (e2<spark_tfr) {
            // cell below is exhausted, becomes passive
            energyMode[i-LEDS_PER_ROW] = torch_passive;
            // gobble up rest of energy
            increase(e, e2);
            // loose some overall energy
            e = ((int)e*spark_cap)>>8;
            // this cell becomes active spark
            energyMode[i] = torch_spark;
          }
          else {
            increase(e, spark_tfr);
          }
          break;
        }
        case torch_passive: {
          e = ((int)e*heat_cap)>>8;
          increase(e, ((((int)currentEnergy[i-1]+(int)currentEnergy[i+1])*side_rad)>>9) + (((int)currentEnergy[i-LEDS_PER_ROW]*up_rad)>>8));
        }
        default:
          break;
      }
      nextEnergy[i++] = e;
    }
  }
}


// const uint8_t energymap[32] = {0, 64, 96, 112, 128, 144, 152, 160, 168, 176, 184, 184, 192, 200, 200, 208, 208, 216, 216, 224, 224, 224, 232, 232, 232, 240, 240, 240, 240, 248, 248, 248};
const uint8_t energymap[] = { 10, 10, 10, 10, 10, 11, 11, 12, 14, 15, 17, 20, 22, 26, 29, 33, 38, 44, 49, 56, 63, 71, 80, 89, 99, 110, 121, 134, 147, 161, 176, 192 };

void calcNextColors()
{
  for (int i=0; i<NUM_LEDS; i++) {
      int ei; // index into energy alculation buffer
      if (upside_down)
        ei = NUM_LEDS-i;
      else
        ei = i;
      uint16_t e = nextEnergy[ei];
      currentEnergy[ei] = e;
      if (e>250)
        leds.setColorDimmed(i, 170, 170, e, brightness); // blueish extra-bright spark
      else {
        if (e>0) {
          // energy to brightness is non-linear
          byte eb = energymap[e>>3];
          byte r = red_bias;
          byte g = green_bias;
          byte b = blue_bias;
          increase(r, (eb*red_energy)>>8);
          increase(g, (eb*green_energy)>>8);
          increase(b, (eb*blue_energy)>>8);
          leds.setColorDimmed(i, r, g, b, brightness);
        }
        else {
          // background, no energy
          leds.setColorDimmed(i, red_bg, green_bg, blue_bg, brightness);
        }
      }
    }
}


void injectRandom()
{
  // random flame energy at bottom row
  for (int i=0; i<LEDS_PER_ROW; i++) {
    currentEnergy[i] = random(flame_min, flame_max);
    energyMode[i] = torch_nop;
  }
  // random sparks at second row
  for (int i=LEDS_PER_ROW; i<2*LEDS_PER_ROW; i++) {
    if (energyMode[i]!=torch_spark && random(100)<random_spark_probability) {
      currentEnergy[i] = random(spark_min, spark_max);
      energyMode[i] = torch_spark;
    }
  }
}

void loop()
{
  // torch animation + text display + cheerlight background
  injectRandom();
  calcNextEnergy();
  calcNextColors();
  strip.show();
  delay(cycle_wait); // latch & reset needs 50 microseconds pause, at least.
}
