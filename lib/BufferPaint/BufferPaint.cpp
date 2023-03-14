#include <Arduino.h>
#include <BufferPaint.h>


Painter::Painter (uint8_t _rows, uint16_t _columns){

    rows=_rows;
    columns=_columns;

    // Create buffer according to rows and columns
    buffer = new uint8_t*[rows];  // 2 colours (red and green)
    for (uint8_t r=0; r<rows; r++)
    {
        buffer[r]=new uint8_t[columns];
    }
    memset(buffer, 0, rows * columns);

    red = new uint8_t[columns];
    green = new uint8_t[columns];
    flash= new uint8_t[columns];

    // set to default values - yellow, non-flashing
    memset(red,1,columns);
    memset(green,1,columns);
    memset(flash,0,columns);

}

void Painter::clearBuffer() {
    memset(buffer, 0, rows * columns);
}

void Painter::drawPoint(uint16_t x, uint16_t y)  {
    buffer[y][x/8] |= (0x80 >> x%8);
}

void Painter::clearPoint(uint16_t x, uint16_t y){
    buffer[y][x/8] &= ~(0x80 >> x%8);
}

void Painter::setColour(uint16_t x, bool red_on, bool green_on, bool flashing){
    if (red_on) red[x]=1;
    if (green_on) green[x]=1;
    if (flashing) flash[x]=1;
}