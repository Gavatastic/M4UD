#include <Arduino.h>
#include <Adafruit_ZeroDMA.h>
#include "wiring_private.h"
#include <HUB08DMA.h>

#define LATCHLEN 2
#define BLANKS 0

uint8_t frames;             // number of frames to be maintained

uint8_t displays;           // number of displays

int rowlen;               // length of frame, including latches, that needs to be output via DMA
uint8_t liveframe;          // index of frame to be displayed
uint8_t columns;            // number of columns per display
uint8_t rows;               // number of rows per display
uint8_t** framedata;        // array to store frame data 

Adafruit_ZeroDMA DMA;       // DMA instance
ZeroDMAstatus    status;    // DMA return status
DmacDescriptor   *DMACDesc1; // DMA configuration object for top half of display
DmacDescriptor   *DMACDesc2; // DMA configuration object for bottom half of display

EPortType Dport; // Determine port and pin numbers for A5 to save time in callback routine

uint32_t Apin;
uint32_t Bpin;
uint32_t Cpin;

uint32_t Dpin;

uint32_t ABCpinMask;
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


void Hub08DMAInit(uint8_t ndisplays, uint8_t nrows, uint8_t ncolumns, uint8_t nframes) {

    frames=nframes;
    displays=ndisplays;
    columns=ncolumns;
    rows=nrows;

    rowlen=((displays*columns*2)+2);

    // create framedata array to right dimensions
    framedata = new uint8_t*[frames];
    for (uint8_t i=0; i<frames; i++)
        framedata[i]=new uint8_t[(rows*rowlen)];




    // fill data with basic data required to toggle clock, select rows and latch data
    prepareframes();

    Dport = g_APinDescription[A5].ulPort;
    Dpin = g_APinDescription[A5].ulPin;
    DpinMask = (1ul << Dpin);

    Apin= g_APinDescription[10].ulPin;
    Bpin= g_APinDescription[11].ulPin;
    Cpin= g_APinDescription[13].ulPin;
    ABCpinMask=(1ul << Apin) | (1ul << Bpin) | (1ul << Cpin);

};



void prepareframes()
{

// 7. Not so fast! - the framedata arrays need to be restructured so that bytes where D should
// be set/not-set are stored in two contiguous blocks - this is not as simple as r0-7 and r8-15
// This means starting wth the latch for row 0, then all the data for rows 1-8 (minus the latch for row 8)
// followed by the row for latch 8, all the way to row 0 minus the latch for row 0. Simple
// Need a formula for calculating position in framedata on an RC basis. Still, at least it's only one function!


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
                if (s==c%rows && f==0){
                    if (s%2==1){ 
                        framedata[f][i*rowlen+c*2+1] |= (RED+GREEN);
                        framedata[f][i*rowlen+c*2+2] |= (RED+GREEN);
                    } else {
                        framedata[f][i*rowlen+c*2+1] |= GREEN;
                        framedata[f][i*rowlen+c*2+2] |= GREEN;
                    }
                } 

                //turn LEDs off early to reduce bleed when latching
                // if (c>columns*rows*0.25){
                //     framedata[f][i*((columns*displays*2)+2)+c*2+1] |= EN;
                //     framedata[f][i*((columns*displays*2)+2)+c*2+2] |= EN;  

                // }

            }

            framedata[f][i*rowlen+(columns*displays*2)+1] = (i & 0x07) | EN; 
            //framedata[f][i*rowlen+(columns*displays*2)+2] = (i & 0x07) | EN | LATCH ;  
        }

    }    

}


// This is the routine that we really want to be called at callback, a non-static member that can see the member variables
void dma_callback(Adafruit_ZeroDMA *dma) {

    uint8_t *dst = &((uint8_t *)(&TCC0->PATT))[1]; // PAT.vec.PGV  < define destination for transfer (the pattern generator register)

    // DMA.changeDescriptor(DMACDesc1, framedata[liveframe], dst, rows*((columns*2)+LATCHLEN+BLANKS));
    // DMA.changeDescriptor(DMACDesc2, framedata[liveframe]+(rows*((columns*2)+LATCHLEN+BLANKS))/2, dst, rows*((columns*2)+LATCHLEN+BLANKS));

    PORT->Group[Dport].OUTTGL.reg = DpinMask | ABCpinMask;
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
    uint8_t *startAddr = (uint8_t *)alignedAddr;
    DMACDesc1 = DMA.addDescriptor(startAddr,dst,(rows*rowlen)/2,DMA_BEAT_SIZE_BYTE,true,false,0,0); // define action for DMA task to take
    DMACDesc1->BTCTRL.bit.BLOCKACT = DMA_BLOCK_ACTION_BOTH;

    alignedAddr = (uint32_t *)((uint32_t)(&framedata[0][(rows*rowlen)/2]) & ~3);
    startAddr = (uint8_t *)alignedAddr;
    DMACDesc2 = DMA.addDescriptor(startAddr,dst,(rows*rowlen)/2,DMA_BEAT_SIZE_BYTE,true,false,0,0); // define action for DMA task to take
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


void fillframe(uint8_t frame, uint8_t buffer[])
{
    int i=0;
    for (uint8_t r=0; r<rows; r++){
        for (uint8_t byte=0; byte<(columns/8); byte++){
            for (uint8_t bit=7; bit>=0; bit--){

                // RED
                if (buffer[r*(columns/8)+byte] & (1<<bit)) {
                    framedata[frame][i] |= 0b01000000;  // set R bit to 1
                    framedata[frame][i+1] |= 0b01000000;  // set R bit to 1
                } else {
                    framedata[frame][i] &= ~(0b01000000); // set R bit to 0
                    framedata[frame][i+1] &= ~(0b01000000); // set R bit to 0
                }

                // GREEN
                if (buffer[(rows*(columns/8))+r*(columns/8)+byte] & (1<<bit)) {
                    framedata[frame][i] |= 0b10000000;      // set G bit to 1
                    framedata[frame][i+1] |= 0b10000000;      // set G bit to 1
                } else {
                    framedata[frame][i] &= ~(0b10000000);   // set G bit to 0
                    framedata[frame][i+1] &= ~(0b10000000);   // set G bit to 0
                }
                i+=2; 
            }
        }
        i+=2; // pass over latches
    }

}

void setliveframe(uint8_t setframe){
    liveframe=setframe;

}