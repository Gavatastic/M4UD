#include <Arduino.h>
#include <HUB08DMA.h>
#include <BufferPaint.h>

#define FRAMES 2
#define ROWS 16
#define COLUMNS 64
#define DISPLAYS 1

HUB8DMAClass LEDs(DISPLAYS, ROWS, COLUMNS, FRAMES);
Painter buffer1(ROWS, COLUMNS);

void setup() {
  
  pinMode(SCL, OUTPUT);
  analogWrite(SCL, 240);

  pinMode(0,OUTPUT);
  digitalWrite(0,LOW);
  pinMode(1,OUTPUT);
  digitalWrite(1,LOW);

  LEDs.liveframe=0;

  LEDs.begin();

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
  
  // LEDs.liveframe=0;
  // delay();
  // LEDs.liveframe=1;
  // delay(2000);

}

