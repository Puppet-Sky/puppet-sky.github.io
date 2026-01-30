---
title: IMX6ULL寄存器封装
date: 2026-01-30 10:25:04
tags:
---

# 目录
1. [时钟封装](#时钟封装)
2. [IO口PAD域与MUX域封装](#IO口PAD域与MUX域封装)
3. [GPIO封装](#GPIO封装)
4. [main.c主程序改写](#main.c主程序改写)

<!-- more -->

# 时钟封装
目前02节，只使用到了GPIO1 时钟，位于CCGR1寄存器中，所以只对CCGR1进行了结构体封装。
```c
/*********************************时钟开启与关闭************************************* */
/* 
 * CCM相关寄存器地址 
 */
#define CCM_CCGR0_BASE    	0X020C4068
#define CCM_CCGR1_BASE 		0X020C406C
#define CCM_CCGR2_BASE 		0X020C4070
#define CCM_CCGR3_BASE 		0X020C4074
#define CCM_CCGR4_BASE 		0X020C4078
#define CCM_CCGR5_BASE 		0X020C407C
#define CCM_CCGR6_BASE 		0X020C4080

typedef enum {
    CGC_OFF          = 0b00, // 时钟关闭
    CGC_ON_RUN       = 0b01, // 仅在运行模式下时钟开启
    CGC_ON_ALL       = 0b11  // 在所有模式下时钟开启
} CCM_ClockGatingConfig;

// 定义一个联合体来表示CCM_CCGR1寄存器
typedef struct {
    uint32_t CG0_ECSPI1 : 2;       // 1-0: ECSPI1时钟使能
    uint32_t CG1_ECSPI2 : 2;       // 3-2: ECSPI2时钟使能
    uint32_t CG2_ECSPI3 : 2;       // 5-4: ECSPI3时钟使能
    uint32_t CG3_ECSPI4 : 2;       // 7-6: ECSPI4时钟使能
    uint32_t CG4_ADC2 : 2;         // 9-8: ADC2时钟使能
    uint32_t CG5_UART3 : 2;        // 11-10: UART3时钟使能
    uint32_t CG6_EPIT1 : 2;        // 13-12: EPIT1时钟使能
    uint32_t CG7_EPIT2 : 2;        // 15-14: EPIT2时钟使能
    uint32_t CG8_ADC1 : 2;         // 17-16: ADC1时钟使能
    uint32_t CG9_SIM_S : 2;        // 19-18: SIM_S时钟使能
    uint32_t CG10_GPT_BUS : 2;     // 21-20: GPT总线时钟使能
    uint32_t CG11_GPT_SERIAL : 2;  // 23-22: GPT串行时钟使能
    uint32_t CG12_UART4 : 2;       // 25-24: UART4时钟使能
    uint32_t CG13_GPIO1 : 2;       // 27-26: GPIO1时钟使能
    uint32_t CG14_CSU : 2;         // 29-28: CSU时钟使能
    uint32_t CG15_GPIO5 : 2;       // 31-30: GPIO5时钟使能
} CCM_CCGR1_Type;

#define CCM_CCGR1 ((CCM_CCGR1_Type *)(CCM_CCGR1_BASE))


#define __IMX_CCM_GPIO1_CLK_ENABLE() 	(CCM_CCGR1->CG13_GPIO1 = CGC_ON_ALL)
#define __IMX_CCM_GPIO1_CLK_DISABLE() 	(CCM_CCGR1->CG13_GPIO1 = CGC_OFF)
```
# IO口PAD域与MUX域封装
目前看下来PAD和MUX的配置是全部引脚通用的，除了寄存器地址以外其他的好像都能复用
```c
/* 
 * IOMUX相关寄存器地址 
 */
#define GPIO1_PIN3_MUX	((GPIO_MUX *)0X020C4080)
#define GPIO1_PIN3_PAD	((GPIO_PAD *)0X020E02F4)

/***********************************GPIO PAD 配置**************************************** */
// 定义SRE字段的值
#define SRE_Slow_Slew_Rate   0  // 慢速压摆率
#define SRE_Fast_Slew_Rate   1  // 快速压摆率

// 定义DSE字段的值
#define DSE_Disabled         0  // 输出驱动禁用
#define DSE_R0_260_Ohm       1  // R0(260欧姆@3.3V, 150欧姆@1.8V, 240欧姆用于DDR)
#define DSE_R0_2             2  // R0/2
#define DSE_R0_3             3  // R0/3
#define DSE_R0_4             4  // R0/4
#define DSE_R0_5             5  // R0/5
#define DSE_R0_6             6  // R0/6
#define DSE_R0_7             7  // R0/7

// 定义SPEED字段的值
#define SPEED_Low_50MHz      0  // 低速(50 MHz)
#define SPEED_Medium_100MHz  1  // 中速(100 MHz)
#define SPEED_High_100MHz    2  // 高速(100 MHz)
#define SPEED_Max_200MHz     3  // 最大速度(200 MHz)

// 定义ODE字段的值
#define ODE_Disabled         0  // 开漏输出禁用
#define ODE_Enabled          1  // 开漏输出使能

// 定义PKE字段的值
#define PKE_Disabled         0  // 上下拉或保持禁用
#define PKE_Enabled          1  // 上下拉或保持使能

// 定义PUE字段的值
#define PUE_Keeper           0  // 保持
#define PUE_Pull             1  // 上下拉

// 定义PUS字段的值
#define PUS_100K_Ohm_Pull_Down  0  // 100K欧姆下拉
#define PUS_47K_Ohm_Pull_Up     1  // 47K欧姆上拉
#define PUS_100K_Ohm_Pull_Up    2  // 100K欧姆上拉
#define PUS_22K_Ohm_Pull_Up     3  // 22K欧姆上拉

// 定义HYS字段的值
#define HYS_Disabled         0  // 迟滞比较器禁用
#define HYS_Enabled          1  // 迟滞比较器使能

typedef struct
{
	uint32_t SRE : 1;           /* 压摆率 */
	uint32_t : 2;               /* 保留 */
	uint32_t DSE : 3;           /* 设置驱动能力 */
	uint32_t SPEED : 2;         /* 设置IO口速度 */
	uint32_t ODE : 1;           /* 使能开漏输出*/
	uint32_t PKE : 1;           /* 上下拉或状态保持使能， 使能PUE*/
	uint32_t PUE : 1;           /* 设置上下拉或状态保持 */
	uint32_t PUS : 2;           /* 用来设置上下拉电阻的一共有四种 */
	uint32_t HYS : 1;           /* 使能迟滞比较器 */
	uint32_t : 15;
}GPIO_PAD;

// 定义SION字段的值
#define SION_DISABLED  0  // 输入路径由功能确定
#define SION_ENABLED   1  // 强制输入路径的复用功能

// GPIO1_3的复用功能选择
#define GPIO1_3_ALT_I2C1        0b0000  // 选择复用模式：ALT0，I2C1_SDA (i2c1)
#define GPIO1_3_ALT_GPT1        0b0001  // 选择复用模式：ALT1，GPT1_COMPARE3 (gpt1)
#define GPIO1_3_ALT_USB         0b0010  // 选择复用模式：ALT2，USB_OTG2_OC (usb)
#define GPIO1_3_ALT_OSC32K      0b0011  // 选择复用模式：ALT3，OSC32K_32K_OUT (osc32k)
#define GPIO1_3_ALT_USDHC1      0b0100  // 选择复用模式：ALT4，USDHC1_CD_B (usdhc1)
#define GPIO1_3_ALT_GPIO1       0b0101  // 选择复用模式：ALT5，GPIO1_IO03 (gpio1)
#define GPIO1_3_ALT_CCM         0b0110  // 选择复用模式：ALT6，CCM_DI0_EXT_CLK (ccm)
#define GPIO1_3_ALT_SRC         0b0111  // 选择复用模式：ALT7，SRC_TESTER_ACK (src)
#define GPIO1_3_ALT_UART1       0b1000  // 选择复用模式：ALT8，UART1_RX (uart1)

typedef struct
{
	uint32_t MUX : 4;       /* 3-0: 选择pad复用功能 */
	uint32_t SION : 1;      /* 强制输入回环配置*/
	uint32_t  : 27;
}GPIO_MUX;
```
# GPIO封装
led节只使用到了输出配置，所以其他寄存器的参数值都没有列出来。
```c
/********************************GPIO驱动封装********************************************* */
typedef struct
{
    uint32_t DR;         /* 0x00 GPIO数据寄存器，输出数据/输入数据 */
    uint32_t GDIR;       /* 0x04 GPIO方向寄存器，输出/输入 */
    uint32_t PSR;        /* 0x08 GPIO状态寄存器，输入状态 */
    uint32_t ICR1;       /* 0x0C 中断配置寄存器1 */
    uint32_t ICR2;       /* 0x10 中断配置寄存器2 */
    uint32_t IMR;        /* 0x14 中断屏蔽寄存器 */
    uint32_t ISR;        /* 0x18 中断状态寄存器 */
    uint32_t EDGE_SEL;   /* 0x1C 边缘选择寄存器 */
}GPIO_Type;

#define GPIO1_BASE 		    0X0209C000
#define GPIO2_BASE          0x020A0000
#define GPIO3_BASE          0x020A4000
#define GPIO4_BASE          0x020A8000
#define GPIO5_BASE          0x020AC000

#define GPIO1 ((GPIO_Type *)GPIO1_BASE)
#define GPIO2 ((GPIO_Type *)GPIO2_BASE)
#define GPIO3 ((GPIO_Type *)GPIO3_BASE)
#define GPIO4 ((GPIO_Type *)GPIO4_BASE)
#define GPIO5 ((GPIO_Type *)GPIO5_BASE)


#define GPIO_PIN_0          (0x0001 << 0)
#define GPIO_PIN_1          (0x0001 << 1)
#define GPIO_PIN_2          (0x0001 << 2)
#define GPIO_PIN_3          (0x0001 << 3)
#define GPIO_PIN_4          (0x0001 << 4)
#define GPIO_PIN_5          (0x0001 << 5)
#define GPIO_PIN_6          (0x0001 << 6)
#define GPIO_PIN_7          (0x0001 << 7)
#define GPIO_PIN_8          (0x0001 << 8)
#define GPIO_PIN_9          (0x0001 << 9)
#define GPIO_PIN_10         (0x0001 << 10)
#define GPIO_PIN_11         (0x0001 << 11)
#define GPIO_PIN_12         (0x0001 << 12)
#define GPIO_PIN_13         (0x0001 << 13)
#define GPIO_PIN_14         (0x0001 << 14)
#define GPIO_PIN_15         (0x0001 << 15)
#define GPIO_PIN_16         (0x0001 << 16)
#define GPIO_PIN_17         (0x0001 << 17)
#define GPIO_PIN_18         (0x0001 << 18)
#define GPIO_PIN_19         (0x0001 << 19)
#define GPIO_PIN_20         (0x0001 << 20)
#define GPIO_PIN_21         (0x0001 << 21)
#define GPIO_PIN_22         (0x0001 << 22)
#define GPIO_PIN_23         (0x0001 << 23)
#define GPIO_PIN_24         (0x0001 << 24)
#define GPIO_PIN_25         (0x0001 << 25)
#define GPIO_PIN_26         (0x0001 << 26)
#define GPIO_PIN_27         (0x0001 << 27)
#define GPIO_PIN_28         (0x0001 << 28)
#define GPIO_PIN_29         (0x0001 << 29)
#define GPIO_PIN_30         (0x0001 << 30)
#define GPIO_PIN_31         (0x0001 << 31)
```

# main.c主程序改写
程序中我一般尽量避免直接使用数值参与编程，这样在程序编写中一目了然。虽然IMX6ULL有官方的固件库，但从0开始自己搭建一个也能更好的了解驱动底层。在后续驱动封装中应该不会使用函数进行封装，不然工作量太大了。
```c
#include "main.h"

/*
 * @description	: 使能I.MX6U所有外设时钟
 * @param 		: 无
 * @return 		: 无
 */
void clk_enable(void)
{
	__IMX_CCM_GPIO1_CLK_ENABLE();
}

/*
 * @description	: 初始化LED对应的GPIO
 * @param 		: 无
 * @return 		: 无
 */
void led_init(void)
{
	/* 1、初始化IO复用 */
	// SW_MUX_GPIO1_IO03 = 0x5;	/* 复用为GPIO1_IO03 */

	GPIO1_PIN3_MUX->MUX = GPIO1_3_ALT_GPIO1;		/* 复用为GPIO1_IO03 */
	GPIO1_PIN3_MUX->SION = SION_DISABLED;			/* 复用功能 */

	/* 2、、配置GPIO1_IO03的IO属性	
	 *bit 16:0 HYS关闭
	 *bit [15:14]: 00 默认下拉
     *bit [13]: 0 kepper功能
     *bit [12]: 1 pull/keeper使能
     *bit [11]: 0 关闭开路输出
     *bit [7:6]: 10 速度100Mhz
     *bit [5:3]: 110 R0/6驱动能力
     *bit [0]: 0 低转换率
     */
	// SW_PAD_GPIO1_IO03 = 0X10B0;
	
	GPIO1_PIN3_PAD->HYS = HYS_Disabled;
	GPIO1_PIN3_PAD->PUS = PUS_100K_Ohm_Pull_Down;
	GPIO1_PIN3_PAD->PUE = PUE_Keeper;
	GPIO1_PIN3_PAD->PKE = PKE_Enabled;
	GPIO1_PIN3_PAD->ODE = ODE_Disabled;
	GPIO1_PIN3_PAD->SPEED = SPEED_Low_50MHz;
	GPIO1_PIN3_PAD->DSE = DSE_R0_6;
	GPIO1_PIN3_PAD->SRE = SRE_Slow_Slew_Rate;


	/* 3、初始化GPIO */
	GPIO1->GDIR |= GPIO_PIN_3;		/* 设置GPIO1_IO03为输出 */

	/* 4、设置GPIO1_IO03输出低电平，打开LED0 */
	GPIO1->DR &= ~(GPIO_PIN_3);
}

/*
 * @description	: 打开LED灯
 * @param 		: 无
 * @return 		: 无
 */
void led_on(void)
{
	/* 
	 * 将GPIO1_DR的bit3清零	 
	 */
	GPIO1->DR &= ~(GPIO_PIN_3); 
}

/*
 * @description	: 关闭LED灯
 * @param 		: 无
 * @return 		: 无
 */
void led_off(void)
{
	/*    
	 * 将GPIO1_DR的bit3置1
	 */
	GPIO1->DR |= GPIO_PIN_3;
}

/*
 * @description	: 短时间延时函数
 * @param - n	: 要延时循环次数(空操作循环次数，模式延时)
 * @return 		: 无
 */
void delay_short(volatile unsigned int n)
{
	while(n--){}
}

/*
 * @description	: 延时函数,在396Mhz的主频下
 * 			  	  延时时间大约为1ms
 * @param - n	: 要延时的ms数
 * @return 		: 无
 */
void delay(volatile unsigned int n)
{
	while(n--)
	{
		delay_short(0x7ff);
	}
}

/*
 * @description	: mian函数
 * @param 	    : 无
 * @return 		: 无
 */
int main(void)
{
	clk_enable();		/* 使能所有的时钟		 	*/
	led_init();			/* 初始化led 			*/

	while(1)			/* 死循环 				*/
	{	
		led_off();		/* 关闭LED   			*/
		delay(500);		/* 延时大约500ms 		*/

		led_on();		/* 打开LED		 	*/
		delay(500);		/* 延时大约500ms 		*/
	}

	return 0;
}
```
