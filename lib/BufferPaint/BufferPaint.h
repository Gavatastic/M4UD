#ifndef _BUFFERPAINT_H
#define _BUFFERPAINT_H


#if (ARDUINO >= 100) 
# include <Arduino.h>
#else
# include <WProgram.h>
#endif



class Painter{

    private:
        uint8_t rows;
        uint16_t columns;
        uint16_t extent;
        
    public:
        Painter(uint8_t _rows, uint16_t _columns); 
        uint8_t** buffer; 
        uint8_t* flash;
        uint8_t* red;
        uint8_t* green;
        void clearBuffer();
        void drawPoint(uint16_t x, uint16_t y);
        void clearPoint(uint16_t x, uint16_t y);
        void setColour(uint16_t x, bool red_on, bool green_on, bool flashing);
};


#endif