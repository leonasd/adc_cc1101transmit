#include <msp430f5529.h>
#include <MSP430PORT_DEF.h>

void gpio_init()
{
    SYNC_DIR_OUT;
    SCLK_DIR_OUT;
    DIN_DIR_OUT;
}

void DAC8411_Init()
{
    gpio_init();
    SCLK_HIGH;
    SYNC_HIGH;
}

void write2DAC8411(unsigned int Data)
{
    unsigned int Temp = 0;
    unsigned char i = 0;

    Temp = Data;
    SYNC_LOW;
    SCLK_HIGH;
    DIN_LOW;
    SCLK_LOW;
    SCLK_HIGH;
    DIN_LOW;
    SCLK_LOW;

    for(i=0; i<16; i++)
    {
    	SCLK_HIGH;

    	if(Temp & BITF)   DIN_HIGH;
    	else                     DIN_LOW;

    	SCLK_LOW;
    	Temp = Temp << 1;
    }

    SYNC_HIGH;
}


