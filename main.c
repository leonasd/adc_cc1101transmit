#include <msp430.h>
#include <DAC7811.h>
#include <BCSplus_init.h>
#include <cc1100.h>
#include <LCD5510_V2.H>
#include <stdio.h>

//#define txmode

volatile unsigned int result = 0;
unsigned char Txbuff[3] = {0};
unsigned char RxBuf[3] = {0};

void GPIO_Init()
{
    P1DIR |= BIT0;
    P6SEL |= BIT0;                            // Enable A/D channel A0
}

void ADC_Init()
{
	ADC12CTL0 = ADC12ON+ADC12SHT0_4 + ADC12REFON + ADC12REF2_5V;
	ADC12CTL1 = ADC12SHP+ADC12CONSEQ_2 + ADC12SHS_1;       // Use sampling timer, set mode
	ADC12IE = 0x01;                           // Enable ADC12IFG.0
	ADC12CTL0 |= ADC12ENC;
	//ADC12CTL0 |= ADC12SC;
}

void TimerA_Init()
{
	//TA0CCTL1 = CCIE;                          // CCR0 interrupt enabled
	//TA0CCR0 = 3050;
	TA0CCR0 = 1050;
	TA0CCR1 = 20;
	TA0CCTL1 |= OUTMOD_3;                       // CCR1 set/reset mode
	TA0CTL = TASSEL_2 + MC_1 + TACLR;         // SMCLK, upmode, clear TAR
}

int main(void)
{
    BCSplus_graceInit();

    int pklen;

	//DAC8411_Init();
	SpiInit();
	POWER_UP_RESET_CC1100();
	halRfWriteRfSettings();
	pklen = halSpiReadReg(CCxxx0_PKTLEN);
	halSpiWriteBurstReg(CCxxx0_PATABLE, PaTabel, 8);
#ifdef txmode
  GPIO_Init();
  ADC_Init();
  TimerA_Init();
  //LCD5510_Init();
  __enable_interrupt();
  while(1)
  {
	  setSleepMpde();
	  __no_operation();                         // For debugger
  }
#else
    DAC8411_Init();
    P1DIR |= BIT0;
    unsigned char i = 0;
    CC1101_InitWOR(1880); //初始化CC1101进入WOR模式
    while(1) //接收端
    {
  	    if(halRfReceivePacket(RxBuf,&leng))
		{
            P1OUT ^= 0X01;
            result = RxBuf[1];
            result <<= 8;
            result += RxBuf[0];
            write2DAC8411(result);
            for(i=0;i<5;i++)
            {
            	RxBuf[i]=0;
            }
		}
        //CC1101_WOR(); //再次进入WOR模式
        //__bis_SR_register(LPM0_bits+GIE);   // Enter LPM0
	}
#endif
}

#ifdef txmode
#pragma vector=ADC12_VECTOR
__interrupt void ADC12ISR (void)
{
	switch(__even_in_range(ADC12IV,34))
	{
		case  0: break;                           // Vector  0:  No interrupt
		case  2: break;                           // Vector  2:  ADC overflow
		case  4: break;                           // Vector  4:  ADC timing overflow
		case  6:
			           //result = ADC12MEM0;        // Vector  6:  ADC12IFG0
			           result = 4092;
				       Txbuff[0] = result & 0xff;
				       Txbuff[1] = result >> 8;
			           P1OUT ^= 0x01;
				       halRfSendPacket(Txbuff,2);
		               break;
		case  8: break;                           // Vector  8:  ADC12IFG1
		case 10: break;                           // Vector 10:  ADC12IFG2
		case 12: break;                           // Vector 12:  ADC12IFG3
		case 14: break;                           // Vector 14:  ADC12IFG4
		case 16: break;                           // Vector 16:  ADC12IFG5
		case 18: break;                           // Vector 18:  ADC12IFG6
		case 20: break;                           // Vector 20:  ADC12IFG7
		case 22: break;                           // Vector 22:  ADC12IFG8
		case 24: break;                           // Vector 24:  ADC12IFG9
		case 26: break;                           // Vector 26:  ADC12IFG10
		case 28: break;                           // Vector 28:  ADC12IFG11
		case 30: break;                           // Vector 30:  ADC12IFG12
		case 32: break;                           // Vector 32:  ADC12IFG13
		case 34: break;                           // Vector 34:  ADC12IFG14
		default: break;
	}
}
#endif


