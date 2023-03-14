#include <Arduino.h>
#include <Adafruit_ZeroDMA.h>
#include "wiring_private.h"
#include <HUB08DMA.h>
#include <fonts.h>
#include <sstream>

#define LATCHLEN 2
#define BLANKS 0

using namespace std;

uint8_t frames;             // number of frames to be maintained
uint8_t displays;           // number of displays

int rowlen;                 // length of frame, including latches, that needs to be output via DMA
volatile bool framechanged; // indicates that frame has changed and that DMA Descriptors need to be changed
uint8_t liveframe;          // index of frame to be displayed
uint8_t columns;            // number of columns per display
uint8_t rows;               // number of rows per display
uint8_t** framedata;        // array to store frame data 

//uint8_t*** buffer;          // array of colour (red/green/flash) by rows by bufferwidth
//uint8_t* buffer;

uint8_t buffer[3*16*128*4];  // colours * rows * columns * what?
uint16_t bufferwidth;        // length of buffer in columns
//uint16_t extent;             // amount of buffer to display

Adafruit_ZeroDMA DMA;       // DMA instance
ZeroDMAstatus    status;    // DMA return status
DmacDescriptor   *DMACDesc1; // DMA configuration object for top half of display(s)
DmacDescriptor   *DMACDesc2; // DMA configuration object for bottom half of display(s)
uint8_t *startAddrTop;      // start address of array containing top half of display(s)
uint8_t *startAddrBot;      // start address of array containing bottom half of display(s) 

EPortType Dport;            // Determine port and pin numbers for A5 to save time in callback routine
uint32_t Dpin;
uint32_t DpinMask;

static struct {
  EPortType port;      // PORTA|PORTB
  uint8_t bit;         // Port bit (0-31)
  uint8_t wo;          // TCC0/WO# (0-7)
  EPioType peripheral; // Peripheral to select for TCC0 out
} tcc0pinMap[] = {
    {PORTA, 8, 0, PIO_TIMER_ALT},  // FLASH_IO0 on Metro M4
    {PORTA, 9, 1, PIO_TIMER_ALT},  // FLASH_IO1
    {PORTA, 10, 2, PIO_TIMER_ALT}, // FLASH_IO2
    {PORTA, 11, 3, PIO_TIMER_ALT}, // FLASH_IO3
    {PORTA, 12, 6, PIO_TIMER_ALT}, // MOSI   PCC/DEN1  NOT WORKING?
    {PORTA, 13, 7, PIO_TIMER_ALT}, // SCK    PCC/DEN2
    //  PORTA  14  (no TCC0 function)     MISO   PCC/CLK (PIO_COM = peripheral
    //  G)
    {PORTA, 16, 4, PIO_TCC_PDEC},  // D13    PCC[0]
    {PORTA, 17, 5, PIO_TCC_PDEC},  // D12    PCC[1]
    {PORTA, 18, 6, PIO_TCC_PDEC},  // D10    PCC[2]
    {PORTA, 19, 7, PIO_TCC_PDEC},  // D11    PCC[3]
    {PORTA, 20, 0, PIO_TCC_PDEC},  // D9     PCC[4]
    {PORTA, 21, 1, PIO_TCC_PDEC},  // D8     PCC[5]
    {PORTA, 22, 2, PIO_TCC_PDEC},  // D0     PCC[6]
    {PORTA, 23, 3, PIO_TCC_PDEC},  // D1     PCC[7]
    {PORTB, 10, 4, PIO_TIMER_ALT}, // FLASH_SCK
    {PORTB, 11, 5, PIO_TIMER_ALT}, // FLASH_CS
    {PORTB, 12, 0, PIO_TCC_PDEC},  // D7
    {PORTB, 13, 1, PIO_TCC_PDEC},  // D4
    {PORTB, 14, 2, PIO_TCC_PDEC},  // D5     PCC[8]
    {PORTB, 15, 3, PIO_TCC_PDEC},  // D6     PCC[9]
    {PORTB, 16, 4, PIO_TCC_PDEC},  // D3
    {PORTB, 17, 5, PIO_TCC_PDEC},  // D2
    {PORTB, 30, 6, PIO_TCC_PDEC},  // SWO
    {PORTB, 31, 7, PIO_TCC_PDEC}  // NC
};
#define PINMAPSIZE                                                             \
  (sizeof(tcc0pinMap) /                                                        \
   sizeof(tcc0pinMap[0])) ///< Number of elements in the tcc0pinMap[] array

static uint8_t configurePin(uint8_t pin) {
  if ((pin >= 0) && (pin < PINS_COUNT)) {
    EPortType port = g_APinDescription[pin].ulPort;
    uint8_t bit = g_APinDescription[pin].ulPin;
    for (uint8_t i = 0; i < PINMAPSIZE; i++) {
      if ((port == tcc0pinMap[i].port) && (bit == tcc0pinMap[i].bit)) {
        pinPeripheral(pin, tcc0pinMap[i].peripheral);
        return (1 << tcc0pinMap[i].wo);
      }
    }
  }
  return 0;
}


void Hub08DMAInit(uint8_t ndisplays, uint8_t nrows, uint8_t ncolumns, uint8_t nframes, uint16_t _bufferwidth) {

    frames=nframes;
    displays=ndisplays;
    columns=ncolumns;
    rows=nrows;
    bufferwidth=_bufferwidth;

    rowlen=((displays*columns*2)+2);  // ?? this needs explaining

    // create framedata array to right dimensions
    framedata = new uint8_t*[frames];
    for (uint8_t i=0; i<frames; i++)
        framedata[i]=new uint8_t[(rows*rowlen)];

    // fill data with basic data required to toggle clock, select rows and latch data
    prepareframes();

    // Pin to be used to D (the MSB of the line select) - toggled manually outside of DMA
    Dport = g_APinDescription[A5].ulPort;
    Dpin = g_APinDescription[A5].ulPin;
    DpinMask = (1ul << Dpin);

    // create buffer array to right dimensions
    // buffer = new uint8_t**[3]; // (red/green/flash)
    // for (uint8_t i=0; i<3; i++){
    //     buffer[i]=new uint8_t*[rows];
    //     for (uint16_t j=0; j<rows; j++) {
    //         buffer[i][j]=new uint8_t[bufferwidth/8];
    //     }
    // }
    //buffer = new uint8_t[3*rows*(bufferwidth/8)];

};


void prepareframes()
{

// When sending out data for a row, we are line selecting for the previous row
    uint8_t i,s;

    for (uint8_t f=0; f<frames; f++){    

        for (uint8_t r=1; r<=rows; r++){
            i=r-1; // for indexing array
            s=r%rows; // for array contents - loops from 1 to 0 via (rows-1)

            framedata[f][i*rowlen] = (i & 0x07) | EN | LATCH; // Latch previous row's data out, 
            //framedata[f][i*((columns*displays*2)+2)] = (i & 0x07) |  EN ; 

            for (uint8_t c=0; c<columns*displays; c++){

                // fill with clocks and line selects. Line selects relate to data sent out in previous iteration so i
                framedata[f][i*rowlen+c*2+1]=(i & 0x07) ;
                framedata[f][i*rowlen+c*2+2]=(i & 0x07) | CLOCK ;

                // add a pattern so we can test
                if (s==c%rows && f==0){  //  - pattern only added to frame 0
                    if (s%2==1){ 
                        framedata[f][i*rowlen+c*2+1] |= (RED+GREEN);
                        framedata[f][i*rowlen+c*2+2] |= (RED+GREEN);
                    } else {
                        framedata[f][i*rowlen+c*2+1] |= GREEN;
                        framedata[f][i*rowlen+c*2+2] |= GREEN;
                    }
                } 

                // add a pattern so we can test
                if (s==(c+8)%rows && f==0){   // - pattern only added to frame 0
                    if (s%2==1){ 
                        framedata[f][i*rowlen+c*2+1] |= (RED+GREEN);
                        framedata[f][i*rowlen+c*2+2] |= (RED+GREEN);
                    } else {
                        framedata[f][i*rowlen+c*2+1] |= RED;
                        framedata[f][i*rowlen+c*2+2] |= RED;
                    }
                } 


            }

            framedata[f][i*rowlen+(columns*displays*2)+1] = (i & 0x07) | EN; 

        }

    }    

}


// This is the routine that we really want to be called at callback, a non-static member that can see the member variables
void dma_callback(Adafruit_ZeroDMA *dma) {

    if (framechanged) {
        uint8_t *dst = &((uint8_t *)(&TCC0->PATT))[1]; // PAT.vec.PGV  < define destination for transfer (the pattern generator register)
        DMA.changeDescriptor(DMACDesc1, startAddrTop, dst, (rows*rowlen)/2);
        DMA.changeDescriptor(DMACDesc2, startAddrBot, dst, (rows*rowlen)/2);
        framechanged=false;
    }

    PORT->Group[Dport].OUTTGL.reg = DpinMask;
    DMA.resume();

}


void Hub08DMABegin()
{
  
    uint8_t bitmask[8];

    static const int8_t pins[] = {10,11,13,12,0,1,7,9}; // nb. pins for Adafruit Itsy Bitsy M4

    DMA.setTrigger(TCC0_DMAC_ID_OVF);
    DMA.setAction(DMA_TRIGGER_ACTON_BEAT);

    status = DMA.allocate();  // allocate channel for this process DMA

    uint8_t *dst = &((uint8_t *)(&TCC0->PATT))[1]; // PAT.vec.PGV  < define destination for transfer (the pattern generator register)


    uint32_t* alignedAddr = (uint32_t *)((uint32_t)(&framedata[0][0]) & ~3);
    startAddrTop = (uint8_t *)alignedAddr;
    DMACDesc1 = DMA.addDescriptor(startAddrTop,dst,(rows*rowlen)/2,DMA_BEAT_SIZE_BYTE,true,false,0,0); // define action for DMA task to take
    DMACDesc1->BTCTRL.bit.BLOCKACT = DMA_BLOCK_ACTION_BOTH;

    alignedAddr = (uint32_t *)((uint32_t)(&framedata[0][(rows*rowlen)/2]) & ~3);
    startAddrBot = (uint8_t *)alignedAddr;
    DMACDesc2 = DMA.addDescriptor(startAddrBot,dst,(rows*rowlen)/2,DMA_BEAT_SIZE_BYTE,true,false,0,0); // define action for DMA task to take
    DMACDesc2->BTCTRL.bit.BLOCKACT = DMA_BLOCK_ACTION_BOTH;

    DMA.loop(true);

    DMA.setCallback(dma_callback,DMA_CALLBACK_CHANNEL_SUSPEND); // <<<<<<< THIS IS WHERE WE SET THE CALLBACK TYPE (2nd param)

   // Set up generic clock gen 2 as source for TCC0
    // Datasheet recommends setting GENCTRL register in a single write,
    // so a temp value is used here to more easily construct a value.
    GCLK_GENCTRL_Type genctrl;
    genctrl.bit.SRC = GCLK_GENCTRL_SRC_DFLL_Val; // 48 MHz source
    genctrl.bit.GENEN = 1;                       // Enable
    genctrl.bit.OE = 1;
    genctrl.bit.DIVSEL = 0; // Do divide clock source
    genctrl.bit.DIV = 0;
    // genctrl.bit.DIVSEL = 0; // Do not divide clock source
    // genctrl.bit.DIV = 0;
    GCLK->GENCTRL[2].reg = genctrl.reg;
    while (GCLK->SYNCBUSY.bit.GENCTRL1 == 1)
        ;

    GCLK->PCHCTRL[TCC0_GCLK_ID].bit.CHEN = 0;
    while (GCLK->PCHCTRL[TCC0_GCLK_ID].bit.CHEN)
        ; // Wait for disable
    GCLK_PCHCTRL_Type pchctrl;
    pchctrl.bit.GEN = GCLK_PCHCTRL_GEN_GCLK2_Val;
    pchctrl.bit.CHEN = 1;
    GCLK->PCHCTRL[TCC0_GCLK_ID].reg = pchctrl.reg;
    while (!GCLK->PCHCTRL[TCC0_GCLK_ID].bit.CHEN)
        ; // Wait for enable

    // Disable TCC before configuring it
    TCC0->CTRLA.bit.ENABLE = 0;
    while (TCC0->SYNCBUSY.bit.ENABLE)
        ;

    TCC0->CTRLA.bit.PRESCALER = TCC_CTRLA_PRESCALER_DIV1_Val; // 1:1 Prescale

    TCC0->WAVE.bit.WAVEGEN = TCC_WAVE_WAVEGEN_NPWM_Val; // Normal PWM mode
    while (TCC0->SYNCBUSY.bit.WAVE)
        ;

    TCC0->CC[0].reg = 0; // No PWM out
    while (TCC0->SYNCBUSY.bit.CC0)
        ;

    // 2.4 GHz clock: 3 DMA xfers per NeoPixel bit = 800 KHz
    //TCC0->PER.reg = ((120000000 + 1200000) / 90000) - 1;
    TCC0->PER.reg = ((48000000 + 1200000) / 2400000) - 1;

    while (TCC0->SYNCBUSY.bit.PER)
        ;

    // set bit mask according to pins defined at top of this function
    memset(bitmask, 0, sizeof(bitmask));
    uint8_t enableMask = 0x00; // Bitmask of pattern gen outputs
    for (uint8_t i = 0; i < 8; i++) {
        if (bitmask[i] = configurePin(pins[i]))
        {
            enableMask |= bitmask[i];
        }
    }

    TCC0->PATT.vec.PGV = 0; // Set all pattern outputs to 0
    while (TCC0->SYNCBUSY.bit.PATT)
        ;
    TCC0->PATT.vec.PGE = enableMask; // Enable pattern outputs
    while (TCC0->SYNCBUSY.bit.PATT)
        ;
    TCC0->CTRLA.bit.ENABLE = 1;
    while (TCC0->SYNCBUSY.bit.ENABLE)
        ;

    status = DMA.startJob();

    DMA.trigger(); // and we're off!


}


void setliveframe(uint8_t setframe){

    uint32_t* alignedAddr = (uint32_t *)((uint32_t)(&framedata[setframe][0]) & ~3);
    startAddrTop = (uint8_t *)alignedAddr;
    alignedAddr = (uint32_t *)((uint32_t)(&framedata[setframe][(rows*rowlen)/2]) & ~3);  
    startAddrBot = (uint8_t *)alignedAddr;  

    framechanged=true; 
}

void setextent(uint16_t _extent){
    //extent=_extent;
}


void buffertoframe(uint8_t frame){

    bool red;
    bool green;

    for(uint8_t y=0; y<rows; y++){
        for(uint16_t x=0;x<columns*displays; x++){

            red=false;
            green=false;

            if((buffer[index(BUFRED,x,y)] & (0x80 >> (x%8)))!=0)        red=true;
            if((buffer[index(BUFGREEN,x,y)] & (0x80 >> (x%8)))!=0)      green=true;
            paintDirect(frame,x,y,red,green);
            
        }
    }

}

void fillframe(uint8_t frame, uint16_t column){
    
    uint8_t fr;
    uint16_t fc;

    // Remember - rows numbered upwards from bottom, first bit out is right-hand side
    for (uint8_t r=0; r<15; r++){
        for (uint16_t c=column; c<column+(columns*displays); c++){  // c refers to the column in the buffer

            fr=((15-r)-1)&0x0f;  // and the frame is stored 1-15, then 0
            fc=(column+(columns*displays))-c;    // used for frame array and buffer referencing 

            // deal with need for displays*columns worth of blanks at end of buffer first
            if ((c<(columns*displays))||(c>(columns*displays))){
                // set column to blank - red and green
                framedata[frame][(fr*rowlen)+(fc*2)+1] &= ~(RED|GREEN); 
                framedata[frame][(fr*rowlen)+(fc*2)+2] &= ~(RED|GREEN); 

            } else {

                //set column according to content of buffer array
                
                if (buffer[index(BUFRED,c,r)] & (0x80 >> (c%8))) {
                    framedata[frame][(fr*rowlen)+(fc*2)+1] |= RED;
                    framedata[frame][(fr*rowlen)+(fc*2)+2] |= RED;
                } else {
                    framedata[frame][(fr*rowlen)+(fc*2)+1] &= ~(RED); 
                    framedata[frame][(fr*rowlen)+(fc*2)+2] &= ~(RED);                     
                }
                
                if (buffer[index(BUFGREEN,c,r)] & (0x80 >> (c%8))) {
                    framedata[frame][(fr*rowlen)+(fc*2)+1] |= GREEN;
                    framedata[frame][(fr*rowlen)+(fc*2)+2] |= GREEN;
                } else {
                    framedata[frame][(fr*rowlen)+(fc*2)+1] &= ~(GREEN); 
                    framedata[frame][(fr*rowlen)+(fc*2)+2] &= ~(GREEN);      
                }               

            }
        }
    }

}

void setPixel(uint16_t x, uint8_t y, bool red, bool green, bool flash){

    if (red) {
        buffer[index(BUFRED,x,y)] |= (0x80 >> (x%8));
    } else {
        buffer[index(BUFRED,x,y)] &= ~(0x80 >> (x%8));       
    }

    if (green) {
        buffer[index(BUFGREEN,x,y)] |= (0x80 >> (x%8));
    } else {
        buffer[index(BUFGREEN,x,y)] &= ~(0x80 >> (x%8));        
    }

    // if (flash) {
    //     buffer[index(BUFRED,x,y)] |= (0x80 >> (x%8));
    // } else {
    //     buffer[index(BUFRED,x,y)] &= ~(0x80 >> (x%8));        
    // }

}

void TestPattern() {

    for (uint8_t z=0; z<15; z++){

        for (uint16_t x=0; x<14; x++){
            if ((z%4)==1) {
                setPixel(x,z,true,false,false);

            }
            if ((z%4)==2) {
                setPixel(x,z,true,true,false);

            }
            if ((z%4)==3) {
                setPixel(x,z,false,true,false);

            }
             if ((z%4)==0) {
                setPixel(x,z,false,false,false);

            }           
        }

    }

}

void clearBuffer(){

    for(uint8_t c=0; c<3; c++){
        for(uint8_t y=0; y<16; y++){
            for(uint16_t x=0; x<bufferwidth/8; x++){
                buffer[index(c,x,y)]=0;
            }
        }
    }

}

uint16_t index(uint8_t colour, uint16_t x, uint8_t y){

        return colour*rows*(bufferwidth/8) + y*(bufferwidth/8) + (x/8);
}

void paintDirect(uint8_t frame, uint16_t x, uint8_t y, bool red, bool green){

    if (red) {
        framedata[frame][(((15-y)-1)&0x0f)*rowlen + (127-x)*2 + 1] |=RED;
        framedata[frame][(((15-y)-1)&0x0f)*rowlen + (127-x)*2 + 2] |=RED;
    } else {
        framedata[frame][(((15-y)-1)&0x0f)*rowlen + (127-x)*2 + 1] &=~(RED);
        framedata[frame][(((15-y)-1)&0x0f)*rowlen + (127-x)*2 + 2] &=~(RED);
    }

    if (green) {
        framedata[frame][(((15-y)-1)&0x0f)*rowlen + (127-x)*2 + 1] |=GREEN;
        framedata[frame][(((15-y)-1)&0x0f)*rowlen + (127-x)*2 + 2] |=GREEN;
    } else {
        framedata[frame][(((15-y)-1)&0x0f)*rowlen + (127-x)*2 + 1] &=~(GREEN);
        framedata[frame][(((15-y)-1)&0x0f)*rowlen + (127-x)*2 + 2] &=~(GREEN);
    }


}

void renderText(string text, bool red, bool green){

    uint16_t x=0;
    uint8_t c=0; 
    uint8_t ascii=0;
    uint8_t findex=0;
    bool skip=false;

    while(c<text.length()){

        skip=true;
        ascii=static_cast<int>(text[c]);
        if (ascii>=65 && ascii<=90){
            skip=false;
            findex=ascii-65;
        }
        if (ascii>=97 && ascii<=122) {
            skip=false;
            findex=ascii-97+26;
        }
        if (ascii>=48 && ascii<=57) {
            skip=false;
            findex=ascii-48+52;
        }

        if(!skip) {
            for(uint8_t charcol=8-LUHeavy_Descriptors[findex].widthBits;charcol<8;charcol++){
                if(x<displays*columns){
                    for(uint8_t charrow=0;charrow<LUHeavy_FontInfo.heightPixels;charrow++){
                        if((LUHeavy_Bitmaps[LUHeavy_Descriptors[findex].offset+charrow] & (0x80 >> charcol))!=0){
                            setPixel(x,charrow+2,red,green,false);
                        }

                    }
                    x++;
                }
            }
            x++; // add a gap
        }
        if (ascii==32) x+=2;

        c++;
    }

}
