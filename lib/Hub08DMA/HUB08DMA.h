
#ifndef _HUB08DMA_H
#define _HUB08DMA_H

#if (ARDUINO >= 100) 
# include <Arduino.h>
#else
# include <WProgram.h>
#endif

// Pin and bit mappings 

// D0(RX)   Latch   L   Port A Pin 16   TCC0.WO4    0x10
// D1(TX)   Clock   S   Port A Pin 17   TCC0.WO5    0x20
// D7       Red     R1  Port A Pin 18   TCC0.WO6    0x40
// D9       Green   G1  Port A Pin 19   TCC0.WO7    0x80
// D10      A       A   Port A Pin 20   TCC0.WO0    0x01    
// D11      B       B   Port A Pin 21   TCC0.WO1    0x02
// D13      C       C   Port A Pin 22   TCC0.WO2    0x04
// D12      Enable  EN  Port A Pin 23   TCC0.WO3    0x08 

// A5       D       D   Controlled by callback 

#define GREEN 0x40
#define RED 0x80
#define CLOCK 0x20
#define LATCH 0x10
#define EN 0x08

#define BUFRED 0
#define BUFGREEN 1
#define BUFBLINK 2

// Required for work with Adafruit ZeroDMA library
#include <Adafruit_ZeroDMA.h>
#include "wiring_private.h"

void prepareframes();       // fill frame structure with row selection, clock and latch data

//void fillframe(uint8_t frame, uint8_t buffer[]); // transfer data from buffer into frame
void fillframe(uint8_t frame, uint16_t column); // transfer data from buffer from specified column into frame
       
void dma_callback(Adafruit_ZeroDMA *dma); 
        
void Hub08DMAInit(uint8_t ndisplays, uint8_t nrows, uint8_t ncolumns, uint8_t nframes, uint16_t bufferwidth); 

void Hub08DMABegin();               // begin class - includes initialisation of DMA
void setliveframe(uint8_t setframe);
void setextent(uint16_t _extent);
void setPixel(uint16_t x, uint8_t y, bool red, bool green, bool flash);
void TestPattern();
void clearBuffer();
uint16_t index(uint8_t col, uint16_t x, uint8_t y);
void paintDirect(uint8_t frame, uint16_t x, uint8_t y, bool red, bool green);

#endif