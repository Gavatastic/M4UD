

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


class HUB8DMAClass{

    private:
        uint8_t frames;             // number of frames to be maintained
        uint8_t columns;            // number of columns per display
        uint8_t rows;               // number of rows per display
        uint8_t displays;           // number of displays
        uint8_t** framedata;        // array to store frame data
        int framelen;               // length of frame, including latches, that needs to be output via DMA
        void prepareframes();       // fill frame structure with row selection, clock and latch data
        Adafruit_ZeroDMA DMA;       // DMA instance
        ZeroDMAstatus    status;    // DMA return status
        DmacDescriptor   *DMACDesc; // DMA configuration object for instance

    public:

        void fillframe(uint8_t frame, uint8_t (&buffer)[]); // transfer data from buffer into frame
        void fillframe(uint8_t frame, uint8_t &buffer, int column); // transfer data from buffer from specified column into frame
       
        static HUB8DMAClass *HUB8Ptr;    // pointer to instance
        static void callback_wrapper(Adafruit_ZeroDMA *dma); 
        void dma_callback(); 
        
        HUB8DMAClass(uint8_t ndisplays, uint8_t nrows, uint8_t ncolumns, uint8_t nframes); 

        void begin();               // begin class - includes initialisation of DMA
        uint8_t liveframe;          // index of frame to be displayed

 
};


#endif