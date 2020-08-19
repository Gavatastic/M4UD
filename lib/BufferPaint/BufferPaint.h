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
        uint8_t columns;
        
    public:
        Painter(uint8_t _rows, uint8_t _columns); 
        uint8_t*** buffer; 
        void clearBuffer();
        void drawPoint(uint16_t x, uint16_t y, uint8_t colour);

};


#endif