#include <Arduino.h>
#include <HUB08DMA.h>

#define FRAMES 2
#define ROWS 16
#define COLUMNS 64
#define DISPLAYS 2

HUB8DMAClass LEDs(DISPLAYS,ROWS,COLUMNS,FRAMES);
uint8_t buffer[128];

void setup() {
  // put your setup code here, to run once:
  LEDs.liveframe=0;
  LEDs.begin();

  
}

void loop() {
  // put your main code here, to run repeatedly:
}

