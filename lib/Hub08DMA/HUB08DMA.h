
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

// Actions required to implement
// 1. Connect A5 to D
// 2. Connect D12 to EN
// 2a. Check wiring of R1 and G1
// 3. Rewrite fillframe so that 0x04 is enable, Green is returned to G1, set some pixels green

// 4. Declare second DMADesc, double-up Descriptor code
// 5. Toggle pin A5 in callback routine
// 6. Done

// 7. Not so fast! - the framedata arrays need to be restructured so that bytes where D should
// be set/not-set are stored in two contiguous blocks - this is not as simple as r0-7 and r8-15
// This means starting wth the latch for row 0, then all the data for rows 1-8 (minus the latch for row 8)
// followed by the row for latch 8, all the way to row 0 minus the latch for row 0. Simple
// Need a formula for calculating position in framedata on an RC basis. Still, at least it's only one function!


// Required for work with Adafruit ZeroDMA library
#include <Adafruit_ZeroDMA.h>
#include "wiring_private.h"



void prepareframes();       // fill frame structure with row selection, clock and latch data
void prepareframes2();

void fillframe(uint8_t frame, uint8_t buffer[]); // transfer data from buffer into frame
void fillframe(uint8_t frame, uint8_t &buffer, int column); // transfer data from buffer from specified column into frame
       
void dma_callback(Adafruit_ZeroDMA *dma); 
        
void Hub08DMAInit(uint8_t ndisplays, uint8_t nrows, uint8_t ncolumns, uint8_t nframes); 

void Hub08DMABegin();               // begin class - includes initialisation of DMA
void setliveframe(uint8_t setframe);

#endif