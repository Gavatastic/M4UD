#include <Arduino.h>

#include <Adafruit_DotStar.h>
#include <SPI.h>

#include <HUB08DMA.h>
#include <BufferPaint.h>

#define FRAMES 2
#define ROWS 16
#define COLUMNS 64
#define DISPLAYS 1

Adafruit_DotStar RGBLED(1, 8, 6, DOTSTAR_BRG);
Painter buffer1(ROWS, COLUMNS);

void setup() {
  
  pinMode(SCL, OUTPUT);
  analogWrite(SCL, 240);

  pinMode(13,OUTPUT);
  digitalWrite(0,LOW);
  pinMode(1,OUTPUT);
  digitalWrite(13,LOW);

  Hub08DMAInit(DISPLAYS, ROWS, COLUMNS, FRAMES);

  setliveframe(0);

  RGBLED.begin();
  RGBLED.show();
  RGBLED.setPixelColor(0,0,0,100);
  RGBLED.show();


  Hub08DMABegin();

  //buffer1.drawPoint(10,10,2);


  // uint8_t*** tempbuf[2][16][8];

  // for (uint8_t f=0; f<1; f++){
  //   for (uint8_t r=0; r<16; r++){
  //     for (uint8_t c=0; c<8; c++){

  //       tempbuf[f][r][c]=(uint8_t)0xf0;

  //     }
  //   }
  // }



  //LEDs.fillframe(0,(uint8_t*)&buffer1.buffer);
  //LEDs.fillframe(1,&tempbuf);


}

void loop() {


  while (1==1) {
      setliveframe(0);
      delay(2000);
      RGBLED.setPixelColor(0,100,0,0);
      RGBLED.show();
      //digitalWrite(13, !digitalRead(13));
      
      setliveframe(1);
      delay(2000);
      RGBLED.setPixelColor(0,100,100,0);
      RGBLED.show();
      //digitalWrite(13, !digitalRead(13));
  }

}

