/*
SI     P1.4
SCLK   P1.5
SO     P1.2
GD2    P1.3
CSN    P1.0
GD0    P1.1
*/
#include <msp430f5529.h>


//GPIO defination for CC1101

#define 	GDO0_BIT    BIT6
#define 	GDO2_BIT	BIT6
#define     MISO_BIT	BIT3
#define 	MOSI_BIT	BIT4
#define	    SCK_BIT	    BIT3
#define  	CSN_BIT		BIT2


#define 	GDO0_DIR    P6DIR
#define 	GDO2_DIR	P1DIR
#define     MISO_DIR	P1DIR
#define 	MOSI_DIR	P1DIR
#define	    SCK_DIR	    P3DIR
#define  	CSN_DIR		P1DIR


#define 	GDO0_POUT   P6OUT
#define 	GDO2_POUT	P1OUT
#define     MISO_POUT	P1OUT
#define 	MOSI_POUT	P1OUT
#define	    SCK_POUT	P3OUT
#define  	CSN_POUT	P1OUT


#define 	GDO0_PIN    P6IN
#define 	GDO2_PIN	P1IN
#define     MISO_PIN	P1IN
#define 	MOSI_PIN	P1IN
#define	    SCK_PIN	    P3IN
#define  	CSN_PIN		P1IN


#define 	GDO0_IN         GDO0_PIN&GDO0_BIT 
#define 	GDO2_IN	        GDO2_PIN&GDO2_BIT
#define     MISO_IN	        MISO_PIN&MISO_BIT
#define 	MOSI_IN	        MOSI_PIN&MOSI_BIT
#define	    SCK_IN	        SCK_PIN &SCK_BIT
#define  	CSN_IN		    CSN_PIN &CSN_BIT


#define 	GDO0_UP         GDO0_POUT|=GDO2_BIT 
#define 	GDO2_UP	        GDO2_POUT|=GDO2_BIT
#define     MISO_UP  	    MISO_POUT|=MISO_BIT
#define 	MOSI_UP	        MOSI_POUT|=MOSI_BIT
#define	    SCK_UP	        SCK_POUT|=SCK_BIT
#define  	CSN_UP		    CSN_POUT|=CSN_BIT


#define 	GDO0_DN         GDO0_POUT&=~GDO2_BIT 
#define 	GDO2_DN	        GDO2_POUT&=~GDO2_BIT
#define     MISO_DN  	    MISO_POUT&=~MISO_BIT
#define 	MOSI_DN	        MOSI_POUT&=~MOSI_BIT
#define	    SCK_DN	        SCK_POUT &=~SCK_BIT
#define  	CSN_DN		    CSN_POUT &=~CSN_BIT


#define 	GDO0_DIR_OUT        GDO0_DIR|=GDO0_BIT 
#define 	GDO2_DIR_OUT	    GDO2_DIR|=GDO2_BIT 
#define     MISO_DIR_OUT	    MISO_DIR|=MISO_BIT 
#define 	MOSI_DIR_OUT	    MOSI_DIR|=MOSI_BIT 
#define	    SCK_DIR_OUT	        SCK_DIR |=SCK_BIT  
#define  	CSN_DIR_OUT         CSN_DIR |=CSN_BIT 


#define 	GDO0_DIR_IN         GDO0_DIR&=~GDO0_BIT  
#define 	GDO2_DIR_IN	        GDO2_DIR&=~GDO2_BIT 
#define     MISO_DIR_IN	        MISO_DIR&=~MISO_BIT 
#define 	MOSI_DIR_IN	        MOSI_DIR&=~MOSI_BIT 
#define	    SCK_DIR_IN	        SCK_DIR &=~SCK_BIT 
#define  	CSN_DIR_IN          CSN_DIR &=~CSN_BIT 

//GPIO defination for DAC7811

#define SYNC_BIT   BIT5
#define SCLK_BIT   BIT1
#define DIN_BIT    BIT2

#define SYNC_DIR   P6DIR
#define SCLK_DIR   P4DIR
#define DIN_DIR    P3DIR

#define SYNC_OUT   P6OUT
#define SCLK_OUT   P4OUT
#define DIN_OUT    P3OUT

#define SYNC_DIR_OUT  SYNC_DIR |= SYNC_BIT
#define SCLK_DIR_OUT  SCLK_DIR |= SCLK_BIT
#define DIN_DIR_OUT   DIN_DIR  |= DIN_BIT

#define SYNC_HIGH   SYNC_OUT |= SYNC_BIT
#define SYNC_LOW    SYNC_OUT &= ~SYNC_BIT

#define SCLK_HIGH   SCLK_OUT |= SCLK_BIT
#define SCLK_LOW    SCLK_OUT &= ~SCLK_BIT

#define DIN_HIGH    DIN_OUT |= DIN_BIT
#define DIN_LOW     DIN_OUT &= ~DIN_BIT

//GPIO defination for LCD5110

#define LCD_CLK_BIT   BIT3
#define LCD_DIN_BIT   BIT0
#define LCD_DC_BIT    BIT7
#define LCD_CS_BIT    BIT1
#define LCD_RST_BIT   BIT2

#define LCD_CLK_DIR   P4DIR
#define LCD_DIN_DIR   P4DIR
#define LCD_DC_DIR    P3DIR
#define LCD_CS_DIR    P8DIR
#define LCD_RST_DIR   P8DIR

#define LCD_CLK_OUT   P4OUT
#define LCD_DIN_OUT   P4OUT
#define LCD_DC_OUT    P3OUT
#define LCD_CS_OUT    P8OUT
#define LCD_RST_OUT   P8OUT

#define LCD_CLK_DIR_OUT   LCD_CLK_DIR |= LCD_CLK_BIT
#define LCD_DIN_DIR_OUT   LCD_DIN_DIR |= LCD_DIN_BIT
#define LCD_DC_DIR_OUT    LCD_DC_DIR  |= LCD_DC_BIT
#define LCD_CS_DIR_OUT    LCD_CS_DIR  |= LCD_CS_BIT
#define LCD_RST_DIR_OUT   LCD_RST_DIR |= LCD_RST_BIT

#define LCD_CLK_H         LCD_CLK_OUT |= LCD_CLK_BIT
#define LCD_CLK_L         LCD_CLK_OUT &= ~LCD_CLK_BIT

#define LCD_DIN_H         LCD_DIN_OUT |= LCD_DIN_BIT
#define LCD_DIN_L         LCD_DIN_OUT &= ~LCD_DIN_BIT

#define LCD_DC_H          LCD_DC_OUT  |= LCD_DC_BIT
#define LCD_DC_L          LCD_DC_OUT  &= ~LCD_DC_BIT

#define LCD_CS_H          LCD_CS_OUT  |= LCD_CS_BIT
#define LCD_CS_L          LCD_CS_OUT  &= ~LCD_CS_BIT

#define LCD_RST_H         LCD_RST_OUT |= LCD_RST_BIT
#define LCD_RST_L         LCD_RST_OUT &= ~LCD_RST_BIT
