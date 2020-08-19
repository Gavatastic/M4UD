#include <Arduino.h>
#include <Adafruit_ZeroDMA.h>
#include "wiring_private.h"
#include <HUB08DMA.h>

#define LATCHLEN 2
#define BLANKS 0

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

HUB8DMAClass * HUB8DMAClass::HUB8Ptr; // required else it fails at linking!!

HUB8DMAClass::HUB8DMAClass (uint8_t ndisplays, uint8_t nrows, uint8_t ncolumns, uint8_t nframes) {

    frames=nframes;
    displays=ndisplays;
    columns=ncolumns;
    rows=nrows;

    // create framedata array to right dimensions
    framedata = new uint8_t*[frames];
    for (uint8_t i=0; i<frames; i++)
        framedata[i]=new uint8_t[(rows*((columns*2)+LATCHLEN+BLANKS))];

    // fill data with basic data required to toggle clock, select rows and latch data
    prepareframes2();

    // This sets the pointer to the classs instance, so that the callback wrapper knows where to find the routine to 
    // handle the callback actions
    HUB8Ptr=this;

};

void HUB8DMAClass::prepareframes()
{
    for (uint8_t f=0; f<frames; f++){                           // each frame in turn
        for (uint8_t r=0; r<rows; r++){                         // each row needs to be populated and have latches added
            for (uint8_t c=0; c<displays*columns; c++){       // each row has displays*columns*2 bytes of data (the 2 required for clock off/on)
                
                // fill with clock signals, row selects and latch off data
                framedata[f][r*((columns*2)+LATCHLEN+BLANKS)+c*2]= r | 0x10;            // clock low
                framedata[f][r*((columns*2)+LATCHLEN+BLANKS)+c*2+1]= r | 0x20 | 0x10;    // clock high

                if (r==c%rows && f==0){
                    framedata[f][r*((columns*2)+LATCHLEN+BLANKS)+c*2] |= 0xC0 ;      // add red pin
                    framedata[f][r*((columns*2)+LATCHLEN+BLANKS)+c*2+1] |= 0xC0;    // add red pin
                }

             }
            // add latches to end of sequence of columns
            framedata[f][r*((columns*2)+LATCHLEN+BLANKS)+(columns*2)] = r;   // latch set to high, just for final two entries of row, no clock
            framedata[f][r*((columns*2)+LATCHLEN+BLANKS)+(columns*2)+1] = r ;

            // add blanks to end of sequence of columns after latches
            for (uint8_t i=0; i<BLANKS; i++){
                if (i<BLANKS/2){
                    framedata[f][r*((columns*2)+LATCHLEN+BLANKS)+(columns*2)+LATCHLEN+i]=r|0x10;
                } else {
                    framedata[f][r*((columns*2)+LATCHLEN+BLANKS)+(columns*2)+LATCHLEN+i]=r;
                }
            }

        }
    }
}

// variant prepared to tewts timing of OE and latch to minimise glare  - one colour only
// Pin D7 (0x40) becomes OE

void HUB8DMAClass::prepareframes2()
{
    for (uint8_t f=0; f<frames; f++){                           // each frame in turn
        for (uint8_t r=0; r<rows; r++){                         // each row needs to be populated and have latches added
            for (uint8_t c=0; c<displays*columns; c++){       // each row has displays*columns*2 bytes of data (the 2 required for clock off/on)
                
                // fill with clock signals, row selects and latch off data
                framedata[f][r*((columns*2)+LATCHLEN+BLANKS)+c*2]= (r-1)&0x0f ;            // clock low
                framedata[f][r*((columns*2)+LATCHLEN+BLANKS)+c*2+1]= ((r-1)&0x0f) | 0x20 ;    // clock high

                if (r==c%rows && f==0){
                    framedata[f][r*((columns*2)+LATCHLEN+BLANKS)+c*2] |= 0x80 ;      // add red pin
                    framedata[f][r*((columns*2)+LATCHLEN+BLANKS)+c*2+1] |= 0x80;    // add red pin
                }

             }
            // add latches to end of sequence of columns
            framedata[f][r*((columns*2)+LATCHLEN+BLANKS)+(columns*2)] = ((r-1)&0x0f) | 0x40 ;   // latch set to high, just for final two entries of row, no clock
            framedata[f][r*((columns*2)+LATCHLEN+BLANKS)+(columns*2)+1] = r |  0x40 | 0x10;

            // add blanks to end of sequence of columns after latches
            for (uint8_t i=0; i<BLANKS; i++){
                if (i<BLANKS/2){
                    framedata[f][r*((columns*2)+LATCHLEN+BLANKS)+(columns*2)+LATCHLEN+i]=r|0x10;
                } else {
                    framedata[f][r*((columns*2)+LATCHLEN+BLANKS)+(columns*2)+LATCHLEN+i]=r;
                }
            }

        }
    }
}




// This is the routine that we are telling ZeroDMA to callback to, declared as static
void HUB8DMAClass::callback_wrapper(Adafruit_ZeroDMA *dma){

    // HUB8Ptr->dma_callback();  // call the member function to perform the actions that we want
    // pinMode(13,OUTPUT);
    // digitalWrite(13,LOW);

    uint8_t *dst = &((uint8_t *)(&TCC0->PATT))[1]; // PAT.vec.PGV  < define destination for transfer (the pattern generator register)
    HUB8Ptr->DMA.changeDescriptor(HUB8Ptr->DMACDesc, HUB8Ptr->framedata[HUB8Ptr->liveframe], dst, HUB8Ptr->rows*((HUB8Ptr->columns*2)+2));
//    HUB8Ptr->DMA.changeDescriptor(HUB8Ptr->DMACDesc, HUB8Ptr->framedata[HUB8Ptr->liveframe], dst, HUB8Ptr->rows*((HUB8Ptr->columns*2)+2));
    //HUB8Ptr->DMA.trigger();
}

// This is the routine that we really want to be called at callback, a non-static member that can see the member variables
void HUB8DMAClass::dma_callback() {

    uint8_t *dst = &((uint8_t *)(&TCC0->PATT))[1]; // PAT.vec.PGV  < define destination for transfer (the pattern generator register)

    pinMode(13,OUTPUT);
    digitalWrite(13,HIGH);

    DMA.changeDescriptor(DMACDesc, &framedata[liveframe], dst, rows*((columns*2)+LATCHLEN+BLANKS));
    DMA.startJob();
    DMA.trigger();


}



void HUB8DMAClass::begin()
{
  
    uint8_t bitmask[8];

    static const int8_t pins[] = {10,11,13,12,0,1,7,9}; // nb. pins for Adafruit Itsy Bitsy M4

    DMA.setTrigger(TCC0_DMAC_ID_OVF);
    DMA.setAction(DMA_TRIGGER_ACTON_BEAT);

    status = DMA.allocate();  // allocate channel for this process DMA

    uint8_t *dst = &((uint8_t *)(&TCC0->PATT))[1]; // PAT.vec.PGV  < define destination for transfer (the pattern generator register)


    uint32_t* alignedAddr = (uint32_t *)((uint32_t)(&framedata[0][0]) & ~3);
    uint8_t *startAddr = (uint8_t *)alignedAddr;


    DMACDesc = DMA.addDescriptor(startAddr,dst,rows*((columns*2)+LATCHLEN+BLANKS),DMA_BEAT_SIZE_BYTE,true,false,0,0); // define action for DMA task to take

    DMA.loop(true);

    DMA.setCallback(callback_wrapper); // <<<<<<< THIS IS WHERE WE SET THE CALLBACK TYPE (2nd param)

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


void HUB8DMAClass::fillframe(uint8_t frame, uint8_t buffer[])
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