#include <Arduino.h>

#include <Adafruit_DotStar.h>
#include <SPI.h>

#include <HUB08DMA.h>
#include <BufferPaint.h>
#include <fonts.h>

#define FRAMES 2
#define ROWS 16
#define COLUMNS 64
#define DISPLAYS 2

Adafruit_DotStar RGBLED(1, 8, 6, DOTSTAR_BRG);
//Painter buffer1(ROWS, COLUMNS*10);

// think about definition


void setup() {
 

  pinMode(A5, OUTPUT);

  Hub08DMAInit(DISPLAYS, ROWS, COLUMNS, FRAMES, DISPLAYS*COLUMNS*4);  // ?? why is the bufferwidth *4?

  setliveframe(0);

  RGBLED.begin();
  RGBLED.show();
  RGBLED.setPixelColor(0,0,0,100);
  RGBLED.show();

  Hub08DMABegin();                // initialise Hub08DMA library - sets pins, establishes DMA (addresses, interrupts etc) and starts


  // ---- these commands are referencing Hub08DMA library, not Painter class --------

  clearBuffer();                  // clears buffer
  TestPattern();                  // paints a pattern into buffer - stripes?
  setPixel(5,5,true,false,false); // ?? sets pixel 5,5 to red (but which frame?) -nb this routine onlys seems to address red part of buffer?
  setextent(256);                 // ?? really unsure what this does now
  fillframe(1,24);                // this transfers from buffer into framedata (it is the framedata array that is transferred by DMA)

  // paintDirect(1,0,0,1,0);
  // paintDirect(1,0,1,1,1);
  // paintDirect(1,0,2,0,1);

}

void loop() {


  while (1==1) {
      setliveframe(0);                      // switches live frame
      delay(2000);                          // waits 2 seconds
      RGBLED.setPixelColor(0,100,0,0);      // changes colour of RGB LED on control board
      RGBLED.show();
      
      setliveframe(1);                      // switches live frame
      delay(2000);                          // waits 2 seconds
      RGBLED.setPixelColor(0,100,100,0);    // changes colour of RGB LED on control board
      RGBLED.show();


  }

}

