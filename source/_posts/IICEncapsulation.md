---
title: 软件I2C封装
date: 2026-01-30 10:30:01
tags:
---

# 目录
1. [H文件定义](#H文件定义)
2. [硬件初始化部分](#硬件初始化部分)
3. [硬件控制封装部分](#硬件控制封装部分)
4. [时序控制部分](#时序控制部分)
5. [读写操作部分](#读写操作部分)

<!-- more -->

# 分享缘由

因为项目需要，经常需要主控MCU对I2C模块进行操作，但之前的项目怎么说了，I2C这块写得很乱，而且一个项目里面经常会有好几个模拟I2C的程序，硬件层控制与时序层代码完全没有解耦，以至于想要复用代码基本要把整个代码重新梳理一遍，要是漏了一点没有改到，那这个I2C就废了。经历过几次后，发现不如自己重新写一个。这次代码比较简单，一共抽象封装了四层，一个硬件初始化部分，一个硬件控制封装部分，一个时序部分，一个是读写操作部分。不废话了，直接上代码。

## H文件定义
H文件定义了一个基本的I2C数据结构，I2C管脚以及I2C速率控制，其实还可以在这个封装里加入`读写操作函数，初始化函数`，让这个I2C封装性更好一点。然后就是声明对外的函数接口，让调用者不需要关心内部函数如何实现。
```c
	/*----------------------------------------------*
 *  数据类型定义                                *
 *----------------------------------------------*/
typedef struct
{
    uint32_t SCL_GPIO;          //SCL GPIO口
    uint32_t SCL_Pin;           //SCL PIN脚
    uint32_t SDA_GPIO;          //SDA GPIO口
    uint32_t SDA_PIN;           //SDA PIN脚
    uint32_t I2C_Speed;         //I2C速率
    /*********************************
    void (*init)(void *paramater);		这三个函数是在写博客的时候突然想到，但在下面的代码并没有用
    void (*read)(void *paraneter);		如果，有需要可以用起来，这个当做留个坑吧
    void (*write)(void *parameter);
    ************************************/
}I2C_Struct;

typedef enum{I2C_ACK = 0, I2C_NACK = !I2C_ACK}I2C_ACKFlag; 

/*----------------------------------------------*
 *  函数声明                                    *
 *----------------------------------------------*/
void rt_hw_soft_I2C_init(void);
uint8_t soft_i2c_read_bytes(I2C_Struct * i2c, uint8_t Device_Addr, uint8_t Register_Addr, void *data, uint16_t data_len);

```
## 硬件初始化部分

这部分没啥好说的，根据I2C协议，需要将管脚配置成开漏，然后就是各个单片机对管脚初始化的过程和初始化函数的调用，完成I2C初始化。以GD32为例：~~其实这部分写得很差，明明都定义了I2C_Struct为什么当时没用~~
```c
/*************************宏定义部分，对管脚进行封装**************************************/
#ifdef RT_USING_I2C0
#define I2C0_SDA_PIN		    GPIO_PIN_7      //GPIO_PIN_9
#define I2C0_SDA_PIN_SOURCE     GPIO_AF_4
#define I2C0_SCL_PIN		    GPIO_PIN_6      //GPIO_PIN_8
#define I2C0_SCL_PIN_SOURCE     GPIO_AF_4
#define I2C0_SDA_GPIO			GPIOB
#define I2C0_SCL_GPIO			GPIOB
#define I2C0_SDA_GPIO_RCC       RCU_GPIOB
#define I2C0_SCL_GPIO_RCC       RCU_GPIOB
#define RCC_I2C0	            RCU_I2C0
#endif /*RT_USING_I2C0*/
    
    
#ifdef RT_USING_I2C1
#define I2C1_SDA_PIN	        GPIO_PIN_11
#define I2C1_SDA_PIN_SOURCE     GPIO_AF_4
#define I2C1_SCL_PIN	        GPIO_PIN_10
#define I2C1_SCL_PIN_SOURCE     GPIO_AF_4
#define I2C1_SDA_GPIO	    	GPIOB
#define I2C1_SCL_GPIO	    	GPIOB
#define I2C1_SDA_GPIO_RCC   	RCU_GPIOB
#define I2C1_SCL_GPIO_RCC   	RCU_GPIOB
#define RCC_I2C1	    RCU_I2C1
#endif /*RT_USING_I2C1*/
    
    
#ifdef RT_USING_I2C2
#define I2C2_SDA_PIN		    GPIO_PIN_9
#define I2C2_SDA_PIN_SOURCE     GPIO_AF_4
#define I2C2_SCL_PIN		    GPIO_PIN_8
#define I2C2_SCL_PIN_SOURCE     GPIO_AF_4
#define I2C2_SDA_GPIO			GPIOC
#define I2C2_SCL_GPIO			GPIOA
#define I2C2_SDA_GPIO_RCC   	RCU_GPIOC
#define I2C2_SCL_GPIO_RCC   	RCU_GPIOA
#define RCC_I2C2	            RCU_I2C2
#endif /*RT_USING_I2C2*/

/*初始化管脚功能*/
static void soft_I2C_RCC_Configuration(void)
{
#ifdef RT_USING_I2C0
    /* Enable I2C0  SDA/SCL GPIO clocks */
    rcu_periph_clock_enable(I2C0_SCL_GPIO_RCC);
    rcu_periph_clock_enable(I2C0_SDA_GPIO_RCC);
#endif
}
static void soft_I2C_GPIO_Configuration(void)
{
#ifdef RT_USING_I2C0
     /* configure USART Tx as alternate function push-pull */
    gpio_mode_set(I2C0_SCL_GPIO, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, I2C0_SCL_PIN);
    gpio_output_options_set(I2C0_SCL_GPIO, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, I2C0_SCL_PIN);

    /* configure USART Rx as alternate function push-pull */
    gpio_mode_set(I2C0_SDA_GPIO, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, I2C0_SDA_PIN);
    gpio_output_options_set(I2C0_SDA_GPIO, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, I2C0_SDA_PIN);
    gpio_bit_set(I2C0_SDA_GPIO, I2C0_SDA_PIN);
    gpio_bit_set(I2C0_SCL_GPIO, I2C0_SCL_PIN);
#endif
}
/*此函数才是最终提供给外部的接口函数，需要在此函数完成管脚初始化*/
void rt_hw_soft_I2C_init(void)
{
    soft_I2C_RCC_Configuration();
    soft_I2C_GPIO_Configuration();
}
```
## 硬件控制封装部分
这部分代码与各个硬件息息相关，不能照搬，需要自己按照自己板子，去实现9个函数功能
```c
/*I2C软件延时函数*/
static void soft_i2c_us_delay(I2C_Struct *i2c)
{
    rt_hw_us_delay(i2c->I2C_Speed);
}

/*将SDA线设置成输出模式*/
void soft_I2C_SDA_OUT(I2C_Struct *i2c)
{
    gpio_mode_set(i2c->SDA_GPIO, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, i2c->SDA_PIN);
    gpio_output_options_set(i2c->SDA_GPIO, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, i2c->SDA_PIN);
    gpio_bit_set(i2c->SDA_GPIO, i2c->SDA_PIN);
}

/*将SDA线设置成输入模式*/
void soft_I2C_SDA_IN(I2C_Struct *i2c)
{
    gpio_mode_set(i2c->SDA_GPIO, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, i2c->SDA_PIN);
}

/*读取SDA输入*/
FlagStatus soft_i2c_SDA_read(I2C_Struct *i2c)
{
    return gpio_input_bit_get(i2c->SDA_GPIO, i2c->SDA_PIN);
}

/*拉高SDA线*/
void soft_i2c_SDA_set(I2C_Struct *i2c)
{
    gpio_bit_set(i2c->SDA_GPIO, i2c->SDA_PIN);
}

/*拉低SDA线*/
void soft_i2c_SDA_reset(I2C_Struct *i2c)
{
    gpio_bit_reset(i2c->SDA_GPIO, i2c->SDA_PIN);
}

/*拉高SCL线*/
void soft_i2c_SCL_set(I2C_Struct *i2c)
{
    gpio_bit_set(i2c->SCL_GPIO, i2c->SCL_Pin);
}


/*拉低SCL线*/
void soft_i2c_SCL_reset(I2C_Struct *i2c)
{
    gpio_bit_reset(i2c->SCL_GPIO, i2c->SCL_Pin);
}

/*读取时钟线状态*/
uint8_t soft_i2c_SCL_input(I2C_Struct *i2c)
{
    return gpio_input_bit_get(i2c->SCL_GPIO, i2c->SCL_Pin);
}
```
## 时序控制部分
这部分没啥好说的，必须严格按照I2C协议去编写。如果你认真实现硬件控制部分的9个函数，这部分原则上是不需要进行修改就能使用的。  
这里必须贴个链接，在之前调试时序的时候，读数据老是出现问题，虽然之前就有感觉我写的时序可能会有点BUG。
[I2C时序调试过程中出现的问题](https://www.codeprj.com/blog/7ef3991.html)
```c
/************************************软件I2C时序函数*******************************************/
//产生IIC起始信号
static void soft_i2c_start(I2C_Struct *i2c)
{
    uint16_t temp = 0;
    soft_I2C_SDA_OUT(i2c);
    /*恢复初始状态*/
    soft_i2c_SDA_set(i2c);
    soft_i2c_us_delay(i2c);
    soft_i2c_SCL_set(i2c); 
    soft_i2c_us_delay(i2c);

    while(soft_i2c_SCL_input(i2c) == 0)
    {
        if(temp++ > 20000)
            return;
    }
    
    /*当SCL为高时，拉到SDA，表示一个启动信号*/
    soft_i2c_us_delay(i2c);
    soft_i2c_SDA_reset(i2c);
    soft_i2c_us_delay(i2c);
    /*钳住I2C总线，准备发送数据*/
    soft_i2c_SCL_reset(i2c);
}

//产生IIC停止信号
void soft_i2c_stop(I2C_Struct *i2c)
{
    soft_I2C_SDA_OUT(i2c);

    /*准备发送结束信号*/
    soft_i2c_SCL_reset(i2c);
    soft_i2c_SDA_reset(i2c);
    soft_i2c_us_delay(i2c);
    /*发送结束信号*/
    soft_i2c_SCL_set(i2c);
    soft_i2c_us_delay(i2c);
    soft_i2c_SDA_set(i2c);

    /*发送完成，释放总线*/
    soft_i2c_us_delay(i2c);
}

//等待应答信号到来
//返回值：1，接收应答成功
//        0，接收应答失败
FlagStatus soft_i2c_wait_ack(I2C_Struct * i2c)
{
    uint16_t temp = 0;
    FlagStatus read_bit = SET;
    /*释放SDA线*/
    soft_i2c_SCL_reset(i2c);
    soft_i2c_SDA_set(i2c);
    /*设置成输入模式*/
    soft_I2C_SDA_IN(i2c);

    /*准备产生一个时钟脉冲读取ACK*/
    soft_i2c_us_delay(i2c);
    soft_i2c_SCL_set(i2c);
    soft_i2c_us_delay(i2c);

    while(soft_i2c_SCL_input(i2c) == 0)
    {
        if(temp++ > 20000)
            return SET;
    }

    read_bit = soft_i2c_SDA_read(i2c);
    soft_i2c_us_delay(i2c);
    soft_i2c_SCL_reset(i2c);
    /*读取ACK信号*/
   return read_bit;
}

//被控器产生ACK应答
void soft_i2c_ack(I2C_Struct * i2c)
{
    soft_I2C_SDA_OUT(i2c);

    /*SDA: 低电平， ACK*/
    soft_i2c_SCL_reset(i2c);
    soft_i2c_us_delay(i2c);
    
    soft_i2c_SDA_reset(i2c);
    soft_i2c_us_delay(i2c);

    /*发送ACK信号*/
    soft_i2c_SCL_set(i2c);
    soft_i2c_us_delay(i2c);

    /*继续钳住总线*/
    soft_i2c_SCL_reset(i2c);
}

//被控器不产生ACK应答，将数据线置高电平，释放总线，产生一个停止信号信号来终止数据传输。		    
void soft_i2c_nack(I2C_Struct * i2c)
{
    soft_I2C_SDA_OUT(i2c);

    /*SDA: 低电平， ACK*/
    soft_i2c_SCL_reset(i2c);
    soft_i2c_us_delay(i2c);
    
    soft_i2c_SDA_set(i2c);
    soft_i2c_us_delay(i2c);

    /*发送ACK信号*/
    soft_i2c_SCL_set(i2c);
    soft_i2c_us_delay(i2c);

    /*继续钳住总线*/
    soft_i2c_SCL_reset(i2c);
}

/*传输一字节数据*/
void soft_i2c_send_byte(I2C_Struct * i2c, uint8_t data)
{
    uint16_t temp = 0;
    soft_I2C_SDA_OUT(i2c);
    
    /*MSB先传*/
    for(int i = 0; i < 8; i++)
    {
        /*拉低时钟线线开始传送*/
        soft_i2c_SCL_reset(i2c);
        soft_i2c_us_delay(i2c);
        
        /*按位传输数据*/
        if(data & (0x80 >> i))
            soft_i2c_SDA_set(i2c);
        else
            soft_i2c_SDA_reset(i2c);

        /*维持一个delay, 拉高SCL, 开始传输*/
        soft_i2c_us_delay(i2c);
        soft_i2c_SCL_set(i2c);
        soft_i2c_us_delay(i2c);

        while(soft_i2c_SCL_input(i2c) == 0)
        {
            if(temp++ > 20000)
                return;
        }
    }

    soft_i2c_SCL_reset(i2c);
}

/*读取一字节数据*/
uint8_t soft_i2c_read_byte(I2C_Struct * i2c)
{
    uint8_t recv_data = 0;
    uint16_t temp = 0;

    /*释放SDA总线*/
    soft_i2c_SDA_set(i2c);
    /*设置SDA为输入*/
    soft_I2C_SDA_IN(i2c);

    /*从机准备数据*/
    soft_i2c_SCL_reset(i2c);
    /*先读入MSB*/
    for(int i = 0; i < 8; i++)
    {
        /*从机准备数据*/
        soft_i2c_SCL_reset(i2c);
        soft_i2c_us_delay(i2c);

        /*拉高时钟准备读取*/
        soft_i2c_SCL_set(i2c);
        soft_i2c_us_delay(i2c);

        /*等待从设备准备好了数据释放时钟线*/
        while(soft_i2c_SCL_input(i2c) == 0)
        {
            if(temp++ > 20000)
                return 0x00;
        }

        recv_data <<= 1;
        if(soft_i2c_SDA_read(i2c) == SET)
            recv_data |= 0x01;

        soft_i2c_SCL_reset(i2c);
    }
    
	return recv_data;
}

/******************************************软件I2C时序函数 完************************************************/
```

## 读写操作部分
这部分最好是根据自己要`操作的模块`去编写这个读写函数
```c
/*****************************************************************************
*  函 数 名：soft_i2c_write_bytes
*  功能描述：软件I2C发送数据
*  输入参数：I2C_Struct * i2c              I2C总线
*            uint8_t Device_Addr    从设备地址
*            uint8_t Register_Addr  寄存器地址
*            void *data             数据指针
             uint16_t data_len      数据长度
*  输出参数：无
*  返 回 值：uint8_t
*  
*  修改历史：
*         1. 日    期：2022年9月19日
*            作    者：九月听雨眠
*            修改内容：新生成函数
*  
*****************************************************************************/
uint8_t soft_i2c_write_bytes(I2C_Struct * i2c, uint8_t Device_Addr, uint8_t Register_Addr, void *data, uint16_t data_len)
{
    uint8_t *SendData = (uint8_t *)data;
    /*开始传输*/
    soft_i2c_start(i2c);

    /*传输器件地址, 写模式*/
    soft_i2c_send_byte(i2c, (Device_Addr & 0xFE));

    /*确认ACK, ACK不对直接返回*/
    if(soft_i2c_wait_ack(i2c) != I2C_ACK)
    {
        SOFT_I2C_PRINT("ADDR ACK Error\r\n");
        goto STOP;
    }

    /*传输寄存器地址*/
    soft_i2c_send_byte(i2c, Register_Addr);

    /*确认ACK, ACK不对直接返回*/
    if(soft_i2c_wait_ack(i2c) != I2C_ACK)
    {
        SOFT_I2C_PRINT("ADDR ACK Error\r\n");
        goto STOP;
    }

    /*循环传输字节数据*/
    for(int i = 0; i < data_len; i++)
    {
        soft_i2c_send_byte(i2c, *SendData);

        if(soft_i2c_wait_ack(i2c) != I2C_ACK)
        {
            SOFT_I2C_PRINT("Send data[%d] ACK Error\r\n", i);
            goto STOP;
        }

        SendData++;
    }

    /*发送结束信号*/
    soft_i2c_stop(i2c);
    return SUCCESS;
    
STOP:
    soft_i2c_stop(i2c);
    return ERROR;
}

/*****************************************************************************
*  函 数 名：soft_i2c_read_bytes
*  功能描述：软件I2C读取数据
*  输入参数：I2C_Struct * i2c       I2C总线
*            uint8_t Device_Addr    设备地址
*            uint8_t Register_Addr  寄存器地址
*            void *data             读取数据buff
*            uint16_t data_len      读取数据长度
*  输出参数：无
*  返 回 值：uint8_t                
*  
*  修改历史：
*         1. 日    期：2022年9月19日
*            作    者：九月听雨眠
*            修改内容：新生成函数
*  
*****************************************************************************/
uint8_t soft_i2c_read_bytes(I2C_Struct * i2c, uint8_t Device_Addr, uint8_t Register_Addr, void *data, uint16_t data_len)
{
    uint8_t *RecvData = (uint8_t *)data;
    /*开始传输*/
    soft_i2c_start(i2c);

    /*传输器件地址, 写模式*/
    soft_i2c_send_byte(i2c, (Device_Addr & 0xFE));

    /*确认ACK*/
    if(soft_i2c_wait_ack(i2c) != I2C_ACK)
    {
        SOFT_I2C_PRINT("SEND ADDR ACK Error\r\n");
        goto STOP;
    }

    /*传输寄存器地址*/
    soft_i2c_send_byte(i2c, Register_Addr);

    /*确认ACK, ACK不对直接返回*/
    if(soft_i2c_wait_ack(i2c) != I2C_ACK)
    {
        SOFT_I2C_PRINT("Register ACK Error\r\n");
        goto STOP;
    }
    
    /*重新开始传输信号*/
    soft_i2c_start(i2c);
    /*传输器件地址, 读模式*/
    soft_i2c_send_byte(i2c, (Device_Addr | 0x01));

    /*确认ACK*/
    if(soft_i2c_wait_ack(i2c) != I2C_ACK)
    {
        SOFT_I2C_PRINT("READ ADDR ACK Error\r\n");
        goto STOP;
    }
    
    /*循环读取字节数据*/
    for(int i = 0; i < data_len; i++)
    {
        *RecvData = soft_i2c_read_byte(i2c);
        /*发送应答信号*/
        if(i == data_len - 1)
            soft_i2c_nack(i2c);
        else
            soft_i2c_ack(i2c);

        RecvData++;
    }
        /*发送结束信号*/
    soft_i2c_stop(i2c);
    return SUCCESS;
    
STOP:
    soft_i2c_stop(i2c);
    return ERROR;

}
```
																																				