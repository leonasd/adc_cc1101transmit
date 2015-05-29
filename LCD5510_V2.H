#ifndef	__LCD5510_V2_H__
#define __LCD5510_V2_H__

/*--------------------------------------------------------------*/
//指令宏定义
#define X_Col_Addr     0x80		//定位到第0列指令(列起始地址)(0 - 83)
#define Y_Page_Addr    0x40		//定位到第0页指令(页起始地址)(0 - 5)


/*--------------------------------------------------------------*/
//功能宏定义
//液晶复位
#define LCD_reset_hard	LCD_RST_L; LCD_RST_H	//硬件复位
#define LCD_reset_soft	LCD_reset_5510()			//软件复位
//液晶显示控制(不影响DDRAM)
#define LCD_show_blank		LCD_write_cmd(0x08)		//显示空白
#define LCD_show_normal		LCD_write_cmd(0x0c)		//正常显示
#define LCD_show_black		LCD_write_cmd(0x09)		//显示全黑
#define LCD_show_inverse 	LCD_write_cmd(0x0d)		//反色显示
//便于理解
#define LCD_write_cmd(cmd)	LCD_write_byte(cmd, 0)	//写入指令
#define LCD_write_dat(dat)	LCD_write_byte(dat, 1)	//写入数据

/*--------------------------------------------------------------*/
//函数声明(私有)
void LCD_write_byte(unsigned char wbyte, unsigned char dat_cmd);//写入字节
void LCD_reset_5510(void);										//复位LCD5510


/*--------------------------------------------------------------*/
//函数声明(公有)
/*--------------------------------------------------------------*/
//清屏参数(清DDRAM)
void LCD_clr_scr(void);


/*--------------------------------------------------------------*/
//液晶字节定位(1*1)
//液晶规划:
//x: 0 - 83
//y: 0 - 5
void LCD_pos_byte(unsigned char x, unsigned char y);


/*--------------------------------------------------------------*/
//显示清行
//num:	0 - 5
void LCD_clr_row(unsigned char num);


/*--------------------------------------------------------------*/
//液晶字符输出(6*8字体)
//x: 0 - 13
//y: 0 - 5
void LCD_printc(unsigned char x, unsigned char y, unsigned char c_dat);


/*--------------------------------------------------------------*/
//液晶字符串输出(6*8字体)
//x: 0 - 13
//y: 0 - 5
void LCD_prints(unsigned char x, unsigned char y, unsigned char *s_dat);


/*--------------------------------------------------------------*/
//液晶字符串输出,自动换行(6*8字体)
//x: 0 - 13
//y: 0 - 5
void LCD_printsl(unsigned char x, unsigned char y, unsigned char *s_dat);


/*--------------------------------------------------------------*/
//初始化LCD5510
void LCD5510_Init(void);				


/*--------------------------------------------------------------*/
//包含文件
//#include "LCD5510_V2.C"

#include <msp430.h>
/*--------------------------------------------------------------*/
#endif





 
