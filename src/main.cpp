#include <Arduino.h>

#include <Adafruit_DotStar.h>
#include <SPI.h>

#include <HUB08DMA.h>
#include <BufferPaint.h>

#define FRAMES 2
#define ROWS 16
#define COLUMNS 64
#define DISPLAYS 2

Adafruit_DotStar RGBLED(1, 8, 6, DOTSTAR_BRG);
Painter buffer1(ROWS, COLUMNS);

void setup() {
  
  pinMode(A5, OUTPUT);

  Hub08DMAInit(DISPLAYS, ROWS, COLUMNS, FRAMES);

  setliveframe(0);

  RGBLED.begin();
  RGBLED.show();
  RGBLED.setPixelColor(0,0,0,100);
  RGBLED.show();

  Hub08DMABegin();

  
}

void loop() {


  while (1==1) {
      setliveframe(0);
      delay(2000);
      RGBLED.setPixelColor(0,100,0,0);
      RGBLED.show();
      
      setliveframe(1);
      delay(2000);
      RGBLED.setPixelColor(0,100,100,0);
      RGBLED.show();

  }

}

