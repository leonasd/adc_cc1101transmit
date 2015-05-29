#include <msp430.h>

void SetVcoreUp (unsigned int level)
{
	PMMCTL0_H = PMMPW_H;
	SVSMHCTL = SVSHE + SVSHRVL0 * level + SVMHE + SVSMHRRL0 * level;
	SVSMLCTL = SVSLE + SVMLE + SVSMLRRL0 * level;

	while ((PMMIFG & SVSMLDLYIFG) == 0);

	PMMIFG &= ~(SVMLVLRIFG + SVMLIFG);
	PMMCTL0_L = PMMCOREV0 * level;

	if ((PMMIFG & SVMLIFG))
	{
		while ((PMMIFG & SVMLVLRIFG) == 0);
	}

	SVSMLCTL = SVSLE + SVSLRVL0 * level + SVMLE + SVSMLRRL0 * level;
	PMMCTL0_H = 0x00;
}

void BCSplus_graceInit()
{
	WDTCTL = WDTPW | WDTHOLD;

	SetVcoreUp (0x01);
	SetVcoreUp (0x02);
	SetVcoreUp (0x03);

	UCSCTL3 = SELREF_2;                       // Set DCO FLL reference = REFO
	UCSCTL4 |= SELA_2;                        // Set ACLK = REFO

	__bis_SR_register(SCG0);                  // Disable the FLL control loop

	UCSCTL0 = 0x0000;                         // Set lowest possible DCOx, MODx
	UCSCTL1 = DCORSEL_7;                      // Select DCO range 50MHz operation
	UCSCTL2 = FLLD_0 + 762;                   // Set DCO Multiplier for 25MHz

	__bic_SR_register(SCG0);                  // Enable the FLL control loop
	__delay_cycles(782000);

	// Loop until XT1,XT2 & DCO stabilizes - In this case only DCO has to stabilize
	do
	{
	  UCSCTL7 &= ~(XT2OFFG + XT1LFOFFG + DCOFFG);
	  SFRIFG1 &= ~OFIFG;                      // Clear fault flags
	}while (SFRIFG1&OFIFG);                   // Test oscillator fault flag
}
