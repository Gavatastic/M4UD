#include <Arduino.h>
#include <BufferPaint.h>



Painter::Painter (uint8_t _rows, uint8_t _columns){

    rows=_rows;
    columns=_columns;

    // Create buffer according to rows and columns
    buffer = new uint8_t**[2];  // 2 colours (red and green)
    for (uint8_t c=0; c<2; c++)
    {
        buffer[c]=new uint8_t*[rows];
        for (uint8_t r=0; r<rows; r++)
            buffer[c][r]=new uint8_t[columns/8];
    }
}

void Painter::clearBuffer() {

    memset(buffer, 0, sizeof buffer);
}

void Painter::drawPoint(uint16_t x, uint16_t y, uint8_t colour)  {

    if (colour & 0x01) {
        buffer[0][y][x/8] |= (0x80 >> x%8);
    } else {
        buffer[0][y][x/8] &= ~(0x80 >> x%8);
    }

    if (colour & 0x02) {
        buffer[1][y][x/8] |= (0x80 >> x%8);
    } else {
        buffer[1][y][x/8] &= ~(0x80 >> x%8);
    }

}
