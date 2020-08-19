

#ifndef _HUB08DMA_H
#define _HUB08DMA_H

#if (ARDUINO >= 100) 
# include <Arduino.h>
#else
# include <WProgram.h>
#endif

// Required for work with Adafruit ZeroDMA library
#include <Adafruit_ZeroDMA.h>
#include "wiring_private.h"



void prepareframes();       // fill frame structure with row selection, clock and latch data
void prepareframes2();       // test on OE / latch timing. Remove after use

void fillframe(uint8_t frame, uint8_t buffer[]); // transfer data from buffer into frame
void fillframe(uint8_t frame, uint8_t &buffer, int column); // transfer data from buffer from specified column into frame
       
void dma_callback(Adafruit_ZeroDMA *dma); 
        
void Hub08DMAInit(uint8_t ndisplays, uint8_t nrows, uint8_t ncolumns, uint8_t nframes); 

void Hub08DMABegin();               // begin class - includes initialisation of DMA
void setliveframe(uint8_t setframe);

#endif