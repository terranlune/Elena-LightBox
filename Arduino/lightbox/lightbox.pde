/*
*
 * Copyright 2009 Windell H. Oskay.  All rights reserved.
 *
 *    This program is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*

 A demo program for Peggy 2, filled with RGBW LEDs in 2x2 super-pixels.
 
 Credit: Most of this program was written by Jay Clegg and released by him under
 the GPL.   Please see  http://www.planetclegg.com/projects/Twi2Peggy.html
 
 Please see http://www.evilmadscientist.com/go/peggyRGB for additional details.
 
 */

#include <EEPROM.h>

////////////////////////////////////////////////////////////////////////////////////////////
// FPS must be high enough to not have obvious flicker, low enough that main loop has 
// time to process one byte per pass.
// ~140 seems to be about the absolute max for me (with this code on avr-gcc 4.2, -Os), 
// but compiler differences might make this maximum value larger or smaller. 
// if the value is too high errors start to occur or it will stop receiving altogether
// conversely, any lower than 60 and flicker becomes apparent.
// note: further code optimization might allow this number to
// be a bit higher, but only up to a point...  
// it *must* result in a value for OCR0A in the range of 1-255



// Suggested refresh rates: 80 Hz, 60 Hz.
#define FPS 80
//#define FPS 60     // Can be changed somewhat...  A tradeoff between faster refresh versus
// faster computations.



// 25 rows * 13 bytes per row == 325
#define DISP_BUFFER_SIZE 325
#define MAX_BRIGHTNESS 15


////////////////////////////////////////////////////////////////////////////////////////////
uint8_t frameBuffer[DISP_BUFFER_SIZE];

uint8_t *currentRowPtr = frameBuffer;
uint8_t currentRow=0;
uint8_t currentBrightness=0;


// Note: the refresh code has been optimized heavily from the previous version.
SIGNAL(TIMER0_COMPA_vect)
{	 

  // there are 15 passes through this interrupt for each row per frame.
  // ( 15 * 25) = 375 times per frame.
  // during those 15 passes, a led can be on or off.
  // if it is off the entire time, the perceived brightness is 0/15
  // if it is on the entire time, the perceived brightness is 15/15
  // giving a total of 16 average brightness levels from fully on to fully off.
  // currentBrightness is a comparison variable, used to determine if a certain
  // pixel is on or off during one of those 15 cycles.   currentBrightnessShifted
  // is the same value left shifted 4 bits:  This is just an optimization for
  // comparing the high-order bytes.
  if (++currentBrightness >= MAX_BRIGHTNESS)  
  {
    currentBrightness=0;
    if (++currentRow > 24)
    {
      currentRow =0;
      currentRowPtr = frameBuffer;
    }
    else
    {
      currentRowPtr += 13;
    }
  }

  ////////////////////  Parse a row of data and write out the bits via spi
  uint8_t currentBrightnessShifted = currentBrightness <<4;

  uint8_t *ptr = currentRowPtr + 12;  // its more convenient to work from right to left
  uint8_t p, bits=0;

  // optimization: by using variables for these two masking constants, we can trick gcc into not 
  // promoting to 16-bit int (constants are 16 bit by default, causing the 
  // comparisons to get promoted to 16bit otherwise)].  This turns out to be a pretty
  // substantial optimization for this handler
  uint8_t himask = 0xf0;  
  uint8_t lomask = 0x0f;

  // Opimization: interleave waiting for SPI with other code, so the CPU can do something useful
  // when waiting for each SPI transmission to complete

  p = *ptr--;
  if ((p & lomask) > currentBrightness)  			bits|=1;
  SPDR = bits;

  bits=0;
  p = *ptr--;
  if ((p & lomask) > currentBrightness)  			bits|=64;
  if ((p & himask) > currentBrightnessShifted)	bits|=128;
  p = *ptr--;
  if ((p & lomask) > currentBrightness)  			bits|=16;
  if ((p & himask) > currentBrightnessShifted)	bits|=32;
  p = *ptr--;
  if ((p & lomask) > currentBrightness)  			bits|=4;
  if ((p & himask) > currentBrightnessShifted)	bits|=8;
  p = *ptr--;
  if ((p & lomask) > currentBrightness)  			bits|=1;
  if ((p & himask) > currentBrightnessShifted)	bits|=2;

  while (!(SPSR & (1<<SPIF)))  { 
  } // wait for prior bitshift to complete
  SPDR = bits;


  bits=0;
  p = *ptr--;
  if ((p & lomask) > currentBrightness)  			bits|=64;
  if ((p & himask) > currentBrightnessShifted)	bits|=128;
  p = *ptr--;
  if ((p & lomask) > currentBrightness)  			bits|=16;
  if ((p & himask) > currentBrightnessShifted)	bits|=32;
  p = *ptr--;
  if ((p & lomask) > currentBrightness)  			bits|=4;
  if ((p & himask) > currentBrightnessShifted)	bits|=8;
  p = *ptr--;
  if ((p & lomask) > currentBrightness)  			bits|=1;
  if ((p & himask) > currentBrightnessShifted)	bits|=2;

  while (!(SPSR & (1<<SPIF)))  { 
  } // wait for prior bitshift to complete
  SPDR = bits;


  bits=0;
  p = *ptr--;
  if ((p & lomask) > currentBrightness)  			bits|=64;
  if ((p & himask) > currentBrightnessShifted)	bits|=128;
  p = *ptr--;
  if ((p & lomask) > currentBrightness)  			bits|=16;
  if ((p & himask) > currentBrightnessShifted)	bits|=32;
  p = *ptr--;
  if ((p & lomask) > currentBrightness)  			bits|=4;
  if ((p & himask) > currentBrightnessShifted)	bits|=8;
  p = *ptr--;
  if ((p & lomask) > currentBrightness)  			bits|=1;
  if ((p & himask) > currentBrightnessShifted)	bits|=2;

  while (!(SPSR & (1<<SPIF)))  { 
  }// wait for prior bitshift to complete
  SPDR = bits;

  ////////////////////  Now set the row and latch the bits

    uint8_t portD;


  if (currentRow < 15)
    portD = currentRow+1;
  else
    portD = (currentRow -14)<<4;


  while (!(SPSR & (1<<SPIF)))  { 
  } // wait for last bitshift to complete

  //if (currentBrightness == 0)
  PORTD = 0;				// set all rows to off
  PORTB |= _BV(1); //  latch it, values now set
  //if (currentBrightness == 0)
  PORTD = portD;     // set row
  PORTB &= ~(_BV(1)); // reset latch for next time

  // notes to self, calculations from the oscope:
  // need about minimum of 6us total to clock out all 4 bytes
  // roughly 1.5ms per byte, although some of that is
  // idle time taken between bytes.  6=7us therefore is our
  // absolute minimum time needed to refresh a row, not counting calculation time.
  // Thats just if we do nothing else when writing out SPI and toggle to another row.
  //Measured values from this routine	
  // @ 144 fps the latch is toggled every 19us with an actual 4byte clock out time of 12-13us
  // @ 70 fps the latch is toggle every 39us, with a clock out time of 13-14us
  // times do not count setup/teardown of stack frame

  // one byte @ 115k takes 86us (max) 78us (min) , measured time
  // one byte @ 230k takes 43us (max) 39us (min) , measured time
  // so 230k serial might barely be possible, but not with a 16mhz crystal (error rate to high)
  // 250k might just barely be possible
}

void displayInit(void) 
{
  // need to set output for SPI clock, MOSI, SS and latch.  Eventhough SS is not connected,
  // it must apparently be set as output for hardware SPI to work.
  DDRB =  (1<<DDB5) | (1<<DDB3) | (1<<DDB2) | (1<<DDB1);
  // set all portd pins as output
  DDRD = 0xff; 

  PORTD=0; // select no row

  // enable hardware SPI, set as master and clock rate of fck/2
  SPCR = (1<<SPE) | (1<<MSTR);
  SPSR = (1<<SPI2X); 

  // setup the interrupt.
  TCCR0A = (1<<WGM01); // clear timer on compare match
  TCCR0B = (1<<CS01); // timer uses main system clock with 1/8 prescale
  OCR0A  = (F_CPU >> 3) / 25 / 15 / FPS; // Frames per second * 15 passes for brightness * 25 rows
  TIMSK0 = (1<<OCIE0A);	// call interrupt on output compare match


  for (uint8_t i=0; i < 4; i++)
  {
    SPDR = 0;
    while (!bit_is_set(SPSR, SPIF)) {
    }
  }
}







void SetPx(byte x,byte y, byte red, byte green,byte blue, byte white)
{
  /*  A "Pixel" is a 2x2 unit cell, consisting of a RED, GREEN, BLUE, and WHITE LED.
   Looking at the top left square, we have
   NW: White   Red
   NE: Red     Green
   SW: Green   Blue
   SE: Blue    White
   */

  //  unsigned int Offset = 13*(y << 1) + x;
  red = constrain(red, 0, 15);
  green = constrain(green, 0, 15);
  blue = constrain(blue, 0, 15);
  white = constrain(white, 0, 15);

  unsigned int Offset = y << 1;
  Offset *= 13;
  Offset += x;

  if (x < 13) {
    if (y & 1) {
      frameBuffer[Offset] = red | (green << 4);  // x overflow, drawing "green" dot if x=12 is ok.
    }
    else
    {
      frameBuffer[Offset] = green | (red << 4);  // x overflow, drawing "red" dot if x=12 is ok.
    }    
  }

  if (y < 12) {
    if (y & 1) {
      frameBuffer[13 + Offset] = blue | (white << 4);
    }
    else
    {
      frameBuffer[13 + Offset] = white | (blue << 4);
    }
  }

}


void SetPxSeq(byte x,byte y, byte step, byte white)
{

  byte redTemp, greenTemp, blueTemp;

  if (step <= 15)
  {
    redTemp = step;
    greenTemp = 0;
    blueTemp = 15 - step;
  }
  else  if (step <= 31)
  {
    redTemp = 31 - step;
    greenTemp = step - 16;
    blueTemp = 0;
  }
  else  if (step <= 47)
  {
    redTemp = 0;
    greenTemp = 47 - step;
    blueTemp = step - 32; 
  }

  SetPx(x,y,redTemp,greenTemp,blueTemp,white);

}




float r,g,b;



// Used by loop() method
byte debounce, buttonPressed, numPressed;
int loopDelay = 20; // Only do anything every X loops
unsigned long loopCount = 0;
// Randomize the start so the sin functions are in a more interesting place
unsigned long fullRefreshCount = 500 + random(500);

// Used for rotateHue() method
float myH,myS,myV;

// Used for updatePixelColor() method
float base_h, base_s, base_v;
float MAX_H_AMPLITUDE = 10.0;
float MAX_S_AMPLITUDE = 0.2;
float MAX_V_AMPLITUDE = 0.4;
unsigned int MAX_H_PERIOD = 50; // Expressed in number of fullRefreshes
unsigned int MAX_S_PERIOD = 100;
unsigned int MAX_V_PERIOD = 100;
unsigned long pixel_amplitude[13][13];
unsigned long pixel_period[13][13];
uint8_t pixel_order[13*13];
int pixel_order_index;
float BASE_H_ADJUSTMENT = 0.005;
float BASE_S_ADJUSTMENT = 0.1;



void setup()                    // run once, when the sketch starts
{ 
  // Using a fixed seed for debugging
  //randomSeed(0);
  randomSeed(analogRead(0));

  // Enable pullups for buttons/i2c
  PORTB |= _BV(0); 
  PORTC = _BV(5) | _BV(4) | _BV(3) | _BV(2) | _BV(1) | _BV(0);

  UCSR0B =0; // turn OFF serial RX/TX, necessary if using arduino bootloader 

  displayInit(); 

  sei( );

  // clear display and set to test pattern
  // pattern should look just like the "gray test pattern" from EMS

  uint8_t v = 0;
  for (int i =0; i < DISP_BUFFER_SIZE; i++)
  {
    v = (v+2) % 16;
    // set to 0 for blank startup display
    // low order bits on the left, high order bits on the right
    //	frameBuffer[i]= v + ((v+1)<<4);  
    frameBuffer[i]=0;


  } 

  /*
  r = 0;
   g = 0;
   b = 0;
   w = 0;
   */

  // Initialize the pixel order array, and pixel attributes

  byte i,j;
  pixel_order_index = 0;
  i= 0;
  while (i < 13)
  {
    j = 0;
    while (j < 13)
    {      
      // Build the pixel order array (which will later be shuffled)
      pixel_order[pixel_order_index] = (i << 4) | j;
      pixel_order_index++;

      // Per-pixel save 10-bit random seeds for hsv * amp/period
      unsigned long h_amp = random(1024);
      unsigned long s_amp = random(1024);
      unsigned long v_amp = random(1024);
      unsigned long h_per = random(1024);
      unsigned long s_per = random(1024);
      unsigned long v_per = random(1024);

      /*
       Hue, Sat, and Value each get 10 bits to represent 0-1
       __HHHHHHHHHHSSSSSSSSSSVVVVVVVVVV
       */
      pixel_amplitude[i][j] = (h_amp << 20) | (s_amp << 10) | v_amp;
      pixel_period[i][j] = (h_per << 20) | (s_per << 10) | v_per;

      // Clear each pixel
      SetPx(i,j,0,0,0,0);

      j++;
    }

    i++;
  }

  // Shuffle pixel orders using Knuth shuffle
  // i is the number of items remaining to be shuffled.
  for (int i = 13*13; i > 1; i--) {
    // Pick a random element to swap with the i-th element.
    int j = random(13*13);
    // Swap array elements.
    uint8_t tmp = pixel_order[j];
    pixel_order[j] = pixel_order[i-1];
    pixel_order[i-1] = tmp;
  }
  pixel_order_index = 0;

  // Initialize the base color
  base_h = random(360);
  base_s = 0.8;
  base_v = 0.6;

  // Variables for pressing buttons
  buttonPressed = 0;
  debounce = 0;
  numPressed = -1;

  // Used by rotateHue method
  myH = 0.0;
  myS,myV = 1.0;
}


// r,g,b values are from 0 to 1
// h = [0,360], s = [0,1], v = [0,1]
//		if s == 0, then h = -1 (undefined)


float f, p, q, t, vs, vsf, hsv_i;

void HSVtoRGB( float *r, float *g, float *b, float h, float s, float v )
{
  int hsv_i;

  if( s == 0 ) {
    // achromatic (grey)
    *r = *g = *b = v;
    return;
  }

  h /= 60;			// sector 0 to 5
  hsv_i = floor( h );
  f = h - hsv_i;			// factorial part of h
  //p = v * ( 1 - s );
  //q = v * ( 1 - s * f );
  //t = v * ( 1 - s * ( 1 - f ) );

  vs = v * s;
  vsf = vs * f;

  p = v - vs;
  q = v - vsf;
  t = v - vs + vsf;

  switch( hsv_i ) {
  case 0:
    *r = v;
    *g = t;
    *b = p;
    break;
  case 1:
    *r = q;
    *g = v;
    *b = p;
    break;
  case 2:
    *r = p;
    *g = v;
    *b = t;
    break;
  case 3:
    *r = p;
    *g = q;
    *b = v;
    break;
  case 4:
    *r = t;
    *g = p;
    *b = v;
    break;
  default:		// case 5:
    *r = v;
    *g = p;
    *b = q;
    break;
  }

}


void loop()                     // run over and over again
{

  /* Example usage of the SetPx routine.  In each case, a "pixel" is a 2x2 LED square, and the 0,0 block
   is in the upper left hand corner.  RGBW values may each be as high as 15.
   
   SetPx(0,0,0,0,0,15);    // Set 0,0, white.   Works
   SetPx(0,1,0,0,0,15);    // Set 0,1, white.
   SetPx(3,3,15,15,15,15);    // Set 3,3, RGBW 
   SetPx(11,11,15,15,15,15);    // Set 3,3, RGBW
   
   */

  byte i,j,k;
  float temp;

  if (buttonPressed) 
  {

    if ((PINB & 1U) == 0) 
    {
      debounce = 1;
      buttonPressed = 1;
    }
    else if (debounce == 1)
    {
      debounce = 0;
      numPressed++;

      switch(numPressed)
      {

      case 0:
        setAllToColor(15,0,0,0);
        break;
      case 1:
        setAllToColor(0,15,0,0);
        break;
      case 2:
        setAllToColor(0,0,15,0);
        break;
      case 3:
        setAllToColor(15,15,0,0);
        break;
      case 4:
        setAllToColor(0,15,15,0);
        break;
      case 5:
        setAllToColor(15,0,15,0);
        break;
      case 6:
        setAllToColor(0,0,0,15);
        break;
      case 7:
        setAllToColor(15,15,15,15);
        break;
      default:
        numPressed = -1;
        buttonPressed = 0;
        return;
      }

      //debugEeprom();
      //debugRandomNumbers();
      debugNumber(fullRefreshCount);

    }

  }
  else  // i.e., no buttons pressed-- in the RGB waves part of the program
  {      // RGB waves part:

    if ((PINB & 1U) == 0) 
    {
      debounce = 1;
      buttonPressed = 1;
    }

    // Only actually do anything every X refreshes
    loopCount++;
    if (loopCount % loopDelay == 0)
    {
      if (true)
      {
        updatePixelColor();
      }
      else
      {
        rotateHue();
      }
    }


  } // End "buttons pressed" if/else


} // End loop()

void incrementBaseColor()
{
  // Rotate the hue
  base_h += BASE_H_ADJUSTMENT;
  if (base_h > 360.0) 
  {
    base_h -= 360.0;
  }

  // Randomly move saturation, but not as big as the pixel max
  base_s += random() * BASE_S_ADJUSTMENT;
  base_s = constrain(base_s, 0.0, 1.0);

  // The base value doesn't change

}

void updatePixelColor()
{
  // Get the pixel we're updating
  uint8_t pixelCoords = pixel_order[pixel_order_index];
  uint8_t i = pixelCoords >> 4;
  uint8_t j = pixelCoords & 15;

  // Skip the top and bottom rows
  if (j <= 0 || j >= 12)
  {
    SetPx(i,j, 0,0,0,0);
  }
  else
  {
    // Increment the base color (very slowly!)
    incrementBaseColor();

    /*
    Get this pixel's personality (random numbers)
     
     Hue, Sat, and Value each get 10 bits to represent 0-1
     __HHHHHHHHHHSSSSSSSSSSVVVVVVVVVV
     */
    unsigned long pixel_amp = pixel_amplitude[i][j];
    unsigned long pixel_per = pixel_period[i][j];

    unsigned long h_amp = pixel_amp >> 20 & 1023;
    unsigned long s_amp = pixel_amp >> 10 & 1023;
    unsigned long v_amp = pixel_amp >> 00 & 1023;
    unsigned long h_per = pixel_per >> 20 & 1023;
    unsigned long s_per = pixel_per >> 10 & 1023;
    unsigned long v_per = pixel_per >> 00 & 1023;

    float h_amp_f = 1.0 - 2.0 * h_amp / 1024.0;
    float s_amp_f = 1.0 - 2.0 * s_amp / 1024.0;
    float v_amp_f = 1.0 - 2.0 * v_amp / 1024.0;
    float h_per_f = 1.0 - 2.0 * h_per / 1024.0;
    float s_per_f = 1.0 - 2.0 * s_per / 1024.0;
    float v_per_f = 1.0 - 2.0 * v_per / 1024.0;

    // Calculate the offset from the base color
    //float h = base_h + sin(curTime * h_per_f * MAX_H_PERIOD) * h_amp_f * MAX_H_AMPLITUDE;
    float h = base_h + MAX_H_AMPLITUDE * sin(2.0 * PI * fullRefreshCount / MAX_H_PERIOD * h_per_f );
    float s = base_s + MAX_S_AMPLITUDE * sin(2.0 * PI * fullRefreshCount / MAX_S_PERIOD * s_per_f );
    float v = base_v + MAX_V_AMPLITUDE * sin(2.0 * PI * fullRefreshCount / MAX_V_PERIOD * v_per_f );

    // Correct for out of range
    if (h > 360.0) { 
      h -= 360.0; 
    }
    if (h < 0.0) { 
      h += 360.0; 
    }
    s = constrain(s, 0.0, 0.5);
    v = constrain(v, 0.0, 1.0);

    // Translate to RGB
    float r,g,b,w;
    HSVtoRGB( &r, &g, &b, h, 1.0 - s * 0.3, v); 
    w = (1.0 - s) * 0.6 * v; // Partially implement saturation with the white LED

    // The very left column looks better when it's dimmer
    if (i == 12) {
      r *= 0.75;
      g *= 0.75;
      b *= 0.75;
      w *= 0.75; 
    }

    // Set the color of the pixel
    SetPx(i,j, 16*r, 16*g, 16*b, 16*w);

  }

  // Increment to the next pixel
  pixel_order_index++;
  
  // Increment to the next full refresh
  if (pixel_order_index > 13*13)
  {
    pixel_order_index = 0;
    fullRefreshCount++;
  }
}

void debugRandomNumbers()
{
  uint8_t i = 7;
  uint8_t j = 5;
  unsigned long pixel_amp = pixel_amplitude[i][j];
  unsigned long h_amp = pixel_amp >> 20 & 1023;
  unsigned long v_amp = pixel_amp >> 00 & 1023;
  float h_amp_f = 1.0 - 2.0 * h_amp / 1024.0;

  debugNumber(v_amp); 
}

void debugEeprom()
{
  // Debug to test getting and setting numbers
  unsigned int x;
  set10bitdata(22, 654);
  x = get10bitdata(22);
  debugNumber(x);
}

void rotateHue()
{
  // Rotate the hue
  myH += 0.3;
  if (myH > 360.0) 
  {
    myH -= 360.0;
  }

  HSVtoRGB( &r, &g, &b, myH, 1.0, 1.0);
  setAllToColor(16 * r, 15 * g, 15 * b, 0);
}

void setAllToColor(int r, int g, int b, int w)
{
  byte i,j;
  i= 0;
  while (i < 13)
  {
    j = 0;
    while (j < 13)
    {
      SetPx(i,j,r,g,b,w);
      j++;
    }
    i++;
  } 
}

/*
  The output of this function shows the number in both 
 decimal (the top 12 rows) and binary in the last row.
 
 For example, the number 456 would look like this:
 
 - - - - - - - - - - R R R
 R R R R R R R R R R R R R
 R R R R R R R R R R R R R
 R R R R R R R R R R R R R
 R R R R R R R R R R - R R
 R R R R R R R R R R R - R
 R R R R R R R R R R R R -
 R R R R R R R R R R R R R
 R R R R R R R R R R R R R
 R R R R R R R R R R R R R
 R R R R R R R R R R R R R
 R R R R R R R R R R R R R
 R R R - - - R R - R R R
 
 */
void debugNumber(unsigned long x)
{
  int i=0;
  unsigned long x2 = x;
  unsigned long x10 = x;
  while (i < 13)
  {
    unsigned long r10 = x10 % 10;
    SetPx(i, r10, 0,15,0,0);

    if (x2 % 2 == 0)
    {
      SetPx(i, 12, 15,0,0,15);
    }
    else{
      SetPx(i, 12, 0,15,15,0);
    }

    x2 = x2 / 2;
    x10 = x10 / 10;

    i++;
  }
}

/*
  10-bit data stored in bytes maps cleanly such that each index
 of 10-bit data requires pieces from 2 bytes, and every 4 10-bits is
 stored as exactly 5 bytes.
 
 0       1       2       3       4      
 --------||||||||--------||||||||--------
 ____0000000000
 ____1111111111
 ____2222222222
 ____3333333333
 
 get(0) = (eeprom[0] << 2 | eeprom[1] >> 6) & b0000111111111111
 get(1) = (eeprom[1] << 4 | eeprom[2] >> 4) & b0000111111111111
 get(2) = (eeprom[2] << 6 | eeprom[3] >> 2) & b0000111111111111
 get(3) = (eeprom[3] << 8 | eeprom[4] >> 0) & b0000111111111111
 
 b00001111111111 = d1023
 */


int get10bitdata(int i)
{
  int r = i % 4;  // Use this to determine how much of each byte to keep
  int d = i / 4;  // Which set of 4 10-bits / 5 bytes are we in?
  int e = d * 5;  // Offset into eeprom bytes
  return (EEPROM.read(e+r) << 2*r+2 | EEPROM.read(e+r+1) >> 6-2*r) & 1023;
}

void set10bitdata(int i, unsigned int x)
{
  int r = i % 4;  // Use this to determine how much goes in each byte
  int d = i / 4;  // Which set of 4 10-bits / 5 bytes are we in?
  int e = d * 5;  // Offset into eeprom bytes

  // Max is 1023 (10bit)
  if (x > 1023)
  {
    x = 1023;
  }

  // Get and combine the two bytes we're dealing with
  unsigned int c = EEPROM.read(e+r) << 8 | EEPROM.read(e+r+1);

  // Shift x to align properly in the two bytes
  x = x << 6-2*r;

  // Clear out the spot for the new number
  c = c & (~(1023 << 6-2*r));

  // Add in the new number
  c = c | x;

  // Set the new values
  EEPROM.write(e+r, c >> 8);
  EEPROM.write(e+r+1, (c << 8) >> 8);
}












