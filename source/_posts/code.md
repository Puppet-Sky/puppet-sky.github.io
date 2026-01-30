---
title: code
date: 2026-01-30 11:11:05
tags:
---
# 目录
1. [ADC 频率采集模块](#1-adc-频率采集模块)
2. [遥控按键模块](#2-遥控按键模块)
3. [TJC 显示屏模块](#3-tjc-显示屏模块)
4. [主控板模块](#4-主控板模块)
<!-- more -->
---

## 1. ADC 频率采集模块
```c
#include "adc_frequence.h"
#include <rtthread.h>
#include "board.h"

#define DBG_TAG "FREQ"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

/***********************ADC 采集设置*************************************/
#define SAMPLE_HANDLE_PERIOD    1000    //两秒处理一次数据
#define PERIOD_SAMPLE_COUNT       80      //一个周期80个点
#define AD_SAMPLE_COUNT           160     //采集2个周期
#define AD_SAMPLE_CHANNEL_COUNT   2

uint16_t SampleCount = 0;                                       //记录采样次数
uint16_t PeriodADCSampleBuff[AD_SAMPLE_CHANNEL_COUNT][AD_SAMPLE_COUNT] = { 0 };          //adc采样的数据
uint16_t PeriodTempBuff[AD_SAMPLE_CHANNEL_COUNT][AD_SAMPLE_COUNT] = { 0 };               //adc 暂存的数据
uint16_t PeriodTempBuff2[AD_SAMPLE_COUNT] = {0};
uint16_t PeriodDMABuff[AD_SAMPLE_CHANNEL_COUNT] = {0};          //DMA 数据暂存地址

ADC_HandleTypeDef PeriodADC;
DMA_HandleTypeDef PeriodADCDMA;
#define FREQUENCY_ADC_INSTANCE            ADC1
#define FREQUENCY_ADC_CHANNEL1            ADC_CHANNEL_0
#define FREQUENCY_ADC_CHANNEL2            ADC_CHANNEL_6

#define ADC_CURRENT_GPIOX               GPIOA
#define ADC_CURRENT_PIN                 GPIO_PIN_0

#define ADC_VOLTAGE_GPIOX               GPIOA
#define ADC_VOLTAGE_PIN                 GPIO_PIN_6


rt_tick_t   SampleConsumeTick = 0;

/****************************定时器频率设置********************************************/
TIM_HandleTypeDef htim3;
#define TIM3_PRESCALER      72      //72分频， 定时器时钟频率1MHz,1us
#define TIM3_PERIOD_DEFALT  250     //定时器定时时间为250us, 为一个频率为50Hz的信号，采集80次信号

/******************************************************************************************************/
void frequency_adc_dma_init(void)
{
    /* Peripheral clock enable */
    __HAL_RCC_ADC1_CLK_ENABLE();
    /* ADC1 DMA Init */
    /* ADC1 Init */
    PeriodADCDMA.Instance = DMA1_Channel1;
    PeriodADCDMA.Init.Direction = DMA_PERIPH_TO_MEMORY;
    PeriodADCDMA.Init.PeriphInc = DMA_PINC_DISABLE;
    PeriodADCDMA.Init.MemInc = DMA_MINC_ENABLE;
    PeriodADCDMA.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    PeriodADCDMA.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    PeriodADCDMA.Init.Mode = DMA_NORMAL;
    PeriodADCDMA.Init.Priority = DMA_PRIORITY_MEDIUM;
    if (HAL_DMA_Init(&PeriodADCDMA) != HAL_OK)
    {
        RT_ASSERT(RT_FALSE);
    }

    __HAL_LINKDMA(&PeriodADC, DMA_Handle, PeriodADCDMA);
}

void frequency_adc_init(void)
{
    /* 使能DMA中断传输 */
    __HAL_RCC_DMA1_CLK_ENABLE();

    /* Peripheral clock enable */
    GPIO_InitTypeDef GPIO_InitStruct = { 0 };
    __HAL_RCC_ADC1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitStruct.Pin = ADC_CURRENT_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    HAL_GPIO_Init(ADC_CURRENT_GPIOX, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = ADC_VOLTAGE_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    HAL_GPIO_Init(ADC_VOLTAGE_GPIOX, &GPIO_InitStruct);

    ADC_ChannelConfTypeDef sConfig = {0};

    PeriodADC.Instance = ADC1;
    PeriodADC.Init.ScanConvMode = ADC_SCAN_ENABLE;
    PeriodADC.Init.ContinuousConvMode = DISABLE;
    PeriodADC.Init.DiscontinuousConvMode = DISABLE;
    PeriodADC.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    PeriodADC.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    PeriodADC.Init.NbrOfConversion = AD_SAMPLE_CHANNEL_COUNT;
    if (HAL_ADC_Init(&PeriodADC) != HAL_OK)
    {
        RT_ASSERT(RT_FALSE);
    }

    /* 配置通道 */
    sConfig.Channel = FREQUENCY_ADC_CHANNEL1;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_71CYCLES_5;
    if (HAL_ADC_ConfigChannel(&PeriodADC, &sConfig) != HAL_OK)
    {
        RT_ASSERT(RT_FALSE);
    }

    sConfig.Channel = FREQUENCY_ADC_CHANNEL2;
    sConfig.Rank = ADC_REGULAR_RANK_2;
    if (HAL_ADC_ConfigChannel(&PeriodADC, &sConfig) != HAL_OK)
    {
        RT_ASSERT(RT_FALSE);
    }

    //连接DMA
    frequency_adc_dma_init();

    if(HAL_OK != HAL_ADCEx_Calibration_Start(&PeriodADC))
        LOG_E("ADC calibration error\n");

}

void periodic_signal_adc_init(void)
{
    ADC_ChannelConfTypeDef sConfig = { 0 };

    /* Peripheral clock enable */
    GPIO_InitTypeDef GPIO_InitStruct = { 0 };
    __HAL_RCC_ADC1_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitStruct.Pin = ADC_CURRENT_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    HAL_GPIO_Init(ADC_CURRENT_GPIOX, &GPIO_InitStruct);

    /** Common config
     */
    PeriodADC.Instance = FREQUENCY_ADC_INSTANCE;
    PeriodADC.Init.ScanConvMode = ADC_SCAN_DISABLE;
    PeriodADC.Init.ContinuousConvMode = ENABLE;
    PeriodADC.Init.DiscontinuousConvMode = DISABLE;
    PeriodADC.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    PeriodADC.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    PeriodADC.Init.NbrOfConversion = 1;
    if (HAL_ADC_Init(&PeriodADC) != HAL_OK)
    {
        while (1)
            ;
    }

    /** Configure Regular Channel
     */
    sConfig.Channel = FREQUENCY_ADC_CHANNEL1;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_13CYCLES_5;
    if (HAL_ADC_ConfigChannel(&PeriodADC, &sConfig) != HAL_OK)
    {
        while (1)
            ;
    }

    if (HAL_OK != HAL_ADCEx_Calibration_Start(&PeriodADC))
        while(1);

    HAL_ADC_Start(&PeriodADC);
}

uint16_t dong_get_adc()
{
//    //开启ADC1
//    HAL_ADC_Start(&PeriodADC);
    //等待ADC转换完成，超时为1ms
//    HAL_ADC_PollForConversion(&PeriodADC, 10);
//    //判断ADC是否转换成功
//    if (HAL_IS_BIT_SET(HAL_ADC_GetState(&PeriodADC), HAL_ADC_STATE_REG_EOC))
//    {
//        //读取值
//        return HAL_ADC_GetValue(&PeriodADC);
//    }
//    return 0xFFFF;

    uint16_t Result = 0;
    HAL_ADC_Stop_DMA(&PeriodADC);

    PeriodADCSampleBuff[AD_CURRENT_CHANNEL_RANK][SampleCount] = PeriodDMABuff[AD_CURRENT_CHANNEL_RANK];
    PeriodADCSampleBuff[AD_VOLTAGE_CHANNEL_RANK][SampleCount] = PeriodDMABuff[AD_VOLTAGE_CHANNEL_RANK];

    SampleCount++;

    HAL_ADC_Start_DMA(&PeriodADC, (uint32_t*)&PeriodDMABuff, (AD_SAMPLE_CHANNEL_COUNT));

    return Result;
}

/*******************************定时器初始化设置********************************************************/
void periodic_signal_tim_init(void)
{
    TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
    TIM_MasterConfigTypeDef sMasterConfig = { 0 };

    __HAL_RCC_TIM3_CLK_ENABLE();

    htim3.Instance = TIM3;
    htim3.Init.Prescaler = TIM3_PRESCALER - 1;
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = TIM3_PERIOD_DEFALT - 1;
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
    {
        while (1)
            ;
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
    {
        while (1)
            ;
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
    {
        while (1)
            ;
    }

    /* TIM3 interrupt Init */
    HAL_NVIC_SetPriority(TIM3_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM3_IRQn);
}

void set_TIM3_frequency(uint16_t frequency)
{
    uint16_t PeiodCount = (uint16_t)(1000000 / PERIOD_SAMPLE_COUNT / frequency);

    HAL_TIM_Base_Stop_IT(&htim3);

    //重新设置定时器
    SampleCount = 0;
    rt_memset(PeriodADCSampleBuff, 0, sizeof(PeriodADCSampleBuff));
    __HAL_TIM_SET_AUTORELOAD(&htim3, PeiodCount - 1);
    __HAL_TIM_SET_COUNTER(&htim3, 0);

    SampleConsumeTick = rt_tick_get();

    HAL_ADC_Start_DMA(&PeriodADC, (uint32_t*)&PeriodDMABuff, (AD_SAMPLE_CHANNEL_COUNT));
    HAL_TIM_Base_Start_IT(&htim3);
}


void stop_TIM3(void)
{
    HAL_TIM_Base_Stop_IT(&htim3);
}
/***************************** 定时器中断服务函数   **************************************/
void TIM3_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim3);
}

//检测到TIM定时器定时时间，开始采集一次数据，采集次数达到AD_SAMPLE_COUNT，停止采集。等待其他程序处理
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3)
    {
        dong_get_adc();

        if (SampleCount >= AD_SAMPLE_COUNT)
        {
            HAL_TIM_Base_Stop_IT(htim);
            HAL_ADC_Stop_DMA(&PeriodADC);
            SampleConsumeTick = rt_tick_get() - SampleConsumeTick;
        }
    }
}


//冒泡排序
void periodic_sortArray(uint16_t *arr, int size)
{
    for (int i = 0; i < size - 1; i++)
    {
        for (int j = 0; j < size - i - 1; j++)
        {
            if (arr[j] > arr[j + 1])
            {
                int temp = arr[j];
                arr[j] = arr[j + 1];
                arr[j + 1] = temp;
            }
        }
    }
}



/*************************** 周期采集  *********************************************/
rt_tick_t   SampleTick = 0;             //记录采样时间间隔
PeriodicSampleResults_t    LastSampleValue = {0};        //记录上次采样的值
uint8_t     FrequencySetFlag = RT_FALSE;//进入采集时，频率是否重新进行设置



float sampling_signal_process(void);
PeriodicCalcResult sampling_signal_process2(uint16_t *SampleData, uint16_t ZeroOffset);

void init_periodic_signal_sample(void)
{
//    periodic_signal_adc_init();
    frequency_adc_init();
    periodic_signal_tim_init();
}


/**
 * @brief 给定信号频率，采集信号电流电压有效值， 返回信号的采样值
 * @param frequency
 * @return 采样的有效值
 */
PeriodicSampleResults_t get_periodic_signal_value(uint16_t frequency, uint8_t ADChannel)
{
    if(rt_tick_get() - SampleTick > SAMPLE_HANDLE_PERIOD)
    {
        if(FrequencySetFlag == RT_FALSE)
        {
            //输入检测，频率不能太高也不能为零
            if(frequency > 1000 || frequency == 0)
                frequency = 50;

            set_TIM3_frequency(frequency);
            FrequencySetFlag = RT_TRUE;
        }

        if(SampleCount < AD_SAMPLE_COUNT)
            return LastSampleValue;


        LOG_D("A Sample comsume tick: %d \n\n", SampleConsumeTick);
        PeriodicSampleResults_t SampleResult;
        //float会损失一点精度 ，同时4舍6入
        SampleResult.ACCurrent = (sampling_signal_process2(PeriodADCSampleBuff[AD_CURRENT_CHANNEL_RANK], 2068));
        SampleResult.ACOutputVoltage = (sampling_signal_process2(PeriodADCSampleBuff[AD_VOLTAGE_CHANNEL_RANK], 2063));

        FrequencySetFlag = RT_FALSE;
        SampleTick = rt_tick_get();
        LastSampleValue = SampleResult;
        return SampleResult;
    }

    return LastSampleValue;
}

//给float数开根号
float square_Root_float(float number)
{
    long i;
    float x, y;
    const float f = 1.5F;

    x = number * 0.5F;
    y = number;
    i = * ( long * ) &y;
    i = 0x5f3759df - ( i >> 1 );
    y = * ( float * ) &i;
    y = y * ( f - ( x * y * y ) );
    y = y * ( f - ( x * y * y ) );
    return number * y;
}



// 定义卷积核大小和权值
#define KERNEL_SIZE 7
#define KERNEL_WEIGHTS {4, 9, 12, 15, 12, 9, 4}
void gaussian_filter(uint16_t  *data, uint16_t data_len)
{
    if(data_len > AD_SAMPLE_COUNT)
        return;
    if(data == RT_NULL)
        return;

    uint16_t temp[AD_SAMPLE_COUNT] = {0};
    // 创建一个卷积核数组，用于存储权值
    uint16_t kernel[KERNEL_SIZE] = KERNEL_WEIGHTS;

    // 计算卷积核中所有元素之和
    uint16_t kernel_sum = 0;
    for (int i = 0; i < KERNEL_SIZE; i++)
    {
        kernel_sum += kernel[i];
    }

    // 对数据数组中每个元素进行滤波
    for (int i = 0; i < data_len; i++)
    {
        // 初始化滤波结果为0
        uint32_t result = 0;

        // 对卷积核中每个元素进行加权求和
        for (int j = 0; j < KERNEL_SIZE; j++)
        {
            // 计算数据数组中对应的索引
            int index = i + j - (KERNEL_SIZE - 1) / 2;

            // 如果索引超出数据数组的范围，则使用边界值
            if (index < 0)
            {
                index = 0;
            }
            else if (index >= data_len)
            {
                index = data_len - 1;
            }

            // 将数据数组中的元素乘以卷积核中的权值，并累加到滤波结果中
            result += data[index] * kernel[j];
        }

        // 将滤波结果除以卷积核之和，得到归一化的结果
        result /= kernel_sum;

        // 将滤波结果存储到临时数组中
        temp[i] = result;
    }

    // 将临时数组中的数据复制回原数组
    for (int i = 0; i < data_len; i++)
    {
        data[i] = temp[i];
    }
}

//以均值为基准
PeriodicCalcResult sampling_signal_process2(uint16_t *SampleData, uint16_t ZeroOffset)
{
    uint16_t ADC_Average = 0;
    int Startcount = 0;
    PeriodicCalcResult CaluRe= {0.0};

    //数据拷贝
    rt_memcpy(PeriodTempBuff2, SampleData, sizeof(PeriodTempBuff2));
    periodic_sortArray(PeriodTempBuff2, AD_SAMPLE_COUNT);

//    LOG_D("ADC Buff without sort\n");
//    for(int i = 0; i < AD_SAMPLE_COUNT; i++)
//    {
//        rt_kprintf("%d ", SampleData[i]);
//    }
//    LOG_D("\n");
//
//    LOG_D("Temp Buff sorted\n");
//    for(int i = 0; i < AD_SAMPLE_COUNT; i++)
//        rt_kprintf("%d ", PeriodTempBuff2[i]);
//    LOG_D("\n");

    //进行一次高斯滤波处理
    gaussian_filter(SampleData, AD_SAMPLE_COUNT);
//    LOG_D("Gaussian Filter\n");
//    for(int i = 0; i < AD_SAMPLE_COUNT; i++)
//        rt_kprintf("%d ", SampleData[i]);
//    LOG_D("\n");

    //假设波峰与波谷的平均值为直流分量，从最接近这个值点计算一个周期的均方根值
    ADC_Average = (PeriodTempBuff2[AD_SAMPLE_COUNT - 1] + PeriodTempBuff2[0]) / 2;
    for(Startcount = 0; Startcount < AD_SAMPLE_COUNT/2; Startcount++)
    {
        if(((SampleData[Startcount] <= ADC_Average) && (SampleData[Startcount + 1] >= ADC_Average)) ||
                ((SampleData[Startcount] >= ADC_Average) && (SampleData[Startcount + 1] <= ADC_Average)) )
            break;
    }


    //将采样周期的进行均方根处理 经核算采样值最高4096, 4096*4096*80不会超过32位无符号整形数
    uint32_t   SampleSum = 0;
    float      SampleResult = 0.0;


    for(int i = 0; i < PERIOD_SAMPLE_COUNT; i++)
        SampleSum += SampleData[Startcount + i];

    ADC_Average = SampleSum / PERIOD_SAMPLE_COUNT;
    LOG_D("Start count: %d, value: %d, the average is: %d\n\n", Startcount + 1, SampleData[Startcount], ADC_Average);

    //均方值
    SampleSum = 0;
    for(int i = 0; i < PERIOD_SAMPLE_COUNT; i++)
        SampleSum += (SampleData[Startcount + i] - ZeroOffset) * (SampleData[Startcount + i] - ZeroOffset);

    SampleSum /= PERIOD_SAMPLE_COUNT;

    //开方
    SampleResult = square_Root_float(SampleSum);
    CaluRe.BaseZeroResult = SampleResult + ZeroOffset;


    //平均值开均方根
    SampleSum = 0;
    for(int i = 0; i < PERIOD_SAMPLE_COUNT; i++)
       SampleSum += (SampleData[Startcount + i] - ADC_Average) * (SampleData[Startcount + i] - ADC_Average);


    SampleSum /= PERIOD_SAMPLE_COUNT;

    //开方
    SampleResult = square_Root_float(SampleSum);
    CaluRe.BaseAverageResult = SampleResult + ADC_Average;

    return CaluRe;
}
```

## 2. 遥控按键模块
```c
#include "remote_key.h"
#include "global_var.h"
#include "drv_pin.h"

rt_tick_t KeyTick = 0;
/***********************************遥控引脚配置 开始 *****************************/
struct KEY_GPIO_config{
    char *Name;             //引脚名
    uint32_t Mode;          //模式配置
};
const struct KEY_GPIO_config KeyGPIOConfig[] = {
        {KEY_FUNC, GPIO_MODE_IT_RISING_FALLING},
        {KEY_S1, GPIO_MODE_IT_RISING_FALLING},
        {KEY_S2, GPIO_MODE_IT_RISING_FALLING},
        {KEY_ADD, GPIO_MODE_IT_RISING_FALLING},
        {KEY_DEL, GPIO_MODE_IT_RISING_FALLING},
};

#define KEY_DETCET_INTERVAL 250

void __init_remote_key_gpio(void)
{
    uint8_t ItemNum = sizeof(KeyGPIOConfig)/sizeof(KeyGPIOConfig[0]);

    for(int i = 0; i < ItemNum; i++)
    {
        switch(KeyGPIOConfig[i].Mode)
        {
        case GPIO_MODE_OUTPUT_PP:
            gpio_init_output(KeyGPIOConfig[i].Name);
            break;
        case GPIO_MODE_IT_RISING_FALLING:
            gpio_init_exti(KeyGPIOConfig[i].Name);
            break;
        }
    }
}

//各自中断触发按键事件发送，显示屏收到事件发送，进行页面切换
void EXTI9_5_IRQHandler(void)
{
    rt_event_t ReEvent = get_remote_key_event();

    if(rt_tick_get() - KeyTick < KEY_DETCET_INTERVAL)
    {
        __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_All);
        return;
    }


    rt_interrupt_enter();
    /* EXTI line interrupt detected */
    if(__HAL_GPIO_EXTI_GET_IT(get_pin_from_name(KEY_DEL)->pin) != 0x00u)
    {
        rt_event_send(ReEvent, REMOTE_KEY_DEL_EVENT);
        __HAL_GPIO_EXTI_CLEAR_IT(get_pin_from_name(KEY_DEL)->pin);
    }

    if(__HAL_GPIO_EXTI_GET_IT(get_pin_from_name(KEY_ADD)->pin) != 0x00u)
    {
        rt_event_send(ReEvent, REMOTE_KEY_ADD_EVENT);
        __HAL_GPIO_EXTI_CLEAR_IT(get_pin_from_name(KEY_ADD)->pin);
    }
    rt_interrupt_leave();

    KeyTick = rt_tick_get();
}

void EXTI15_10_IRQHandler(void)
{
    rt_event_t ReEvent = get_remote_key_event();

    if(rt_tick_get() - KeyTick < KEY_DETCET_INTERVAL)
    {
        __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_All);
        return;
    }

    rt_interrupt_enter();
    /* EXTI line interrupt detected */
    if(__HAL_GPIO_EXTI_GET_IT(get_pin_from_name(KEY_FUNC)->pin) != 0x00u)
    {
        rt_event_send(ReEvent, REMOTE_KEY_FUNC_EVENT);
        __HAL_GPIO_EXTI_CLEAR_IT(get_pin_from_name(KEY_FUNC)->pin);
    }
    if (__HAL_GPIO_EXTI_GET_IT(get_pin_from_name(KEY_S2)->pin) != 0x00u)
    {
        rt_event_send(ReEvent, REMOTE_KEY_S2_EVENT);
        __HAL_GPIO_EXTI_CLEAR_IT(get_pin_from_name(KEY_S2)->pin);
    }

    if(__HAL_GPIO_EXTI_GET_IT(get_pin_from_name(KEY_S1)->pin) != 0x00u)
    {
        rt_event_send(ReEvent, REMOTE_KEY_S1_EVENT);
        __HAL_GPIO_EXTI_CLEAR_IT(get_pin_from_name(KEY_S1)->pin);
    }

    rt_interrupt_leave();
    KeyTick = rt_tick_get();
}

```

## 3. TJC 显示屏模块
```c
#include "TJC_Screen.h"

#define DBG_TAG "LCD"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>
#include <stdlib.h>

#include "remote_key.h"
#include "global_var.h"
#include "board.h"
#include "main_control_board.h"

typedef struct
{
    rt_device_t Device;
    uint8_t rx_ind;         //接收标志

    void (* show_background)(void *parameter);
    void (* update_page)(void *parameter);
    void (* function_key)(void *parameter);
    void (* add_key)(void *parameter);
    void (* del_key)(void *parameter);
    void (* s1_key)(void *parameter);
    void (* s2_key)(void *parameter);

    uint8_t LCDCurrentPage;        //LCD 当前页面
    uint8_t LCDPageSwitchFlag;     //是否进行切换操作

}LCD_Device_t;

ScreenData_t ScreenData;    //电池需要显示的数据
LCD_Device_t LCDDevice;

static rt_err_t __lcd_rx_ind(rt_device_t dev, rt_size_t size)
{
    /* release semaphore to let bootsh thread rx data */
    LCDDevice.rx_ind = RT_TRUE;
    return RT_EOK;
}

/**
 * @brief 从BMS更新电池数据
 * @param Pack
 */
void update_screen_data_from_BMS(UartRecieveDataType_t *UpdateData)
{
    //进入临界区，关闭全部中断响应
    ScreenData.bat_voltage_total = UpdateData->bat_voltage_total;
    ScreenData.bat_capiaity = UpdateData->bat_capiaity;
    ScreenData.discharge_current = UpdateData->bat_discharge_current;
    ScreenData.charge_current = UpdateData->bat_charge_current;
    ScreenData.bat_health = UpdateData->bat_health;
    ScreenData.charge_time = UpdateData->charge_time;
    ScreenData.discharge_time = UpdateData->discharge_time;
    ScreenData.insulation_resistance_Rn = UpdateData->insulation_resistance_Rn;
    ScreenData.insulation_resistance_Rp = UpdateData->insulation_resistance_Rp;

    //电池数量不一致时，需要重新绘制单体电池显示数据
    if(ScreenData.bat_num != UpdateData->bat_num)
    {
        ScreenData.bat_num = UpdateData->bat_num;
        ScreenData.battery_num_update_flag = RT_TRUE;
    }

    // 更新电池异常状态  电池充电报警那一位不判断
    if((UpdateData->abnormal_alarm & 0xFF) && (ScreenData.abnormal_alarm != UpdateData->abnormal_alarm))
        ScreenData.over_alam_flag = RT_TRUE;

    if(UpdateData->temp_line_open && (ScreenData.temp_line_open != UpdateData->temp_line_open))
        ScreenData.over_alam_flag = RT_TRUE;

    if(UpdateData->bat_line_open && (ScreenData.bat_line_open != UpdateData->bat_line_open))
        ScreenData.over_alam_flag = RT_TRUE;

    ScreenData.abnormal_alarm = UpdateData->abnormal_alarm;
    ScreenData.temp_line_open = UpdateData->temp_line_open;
    ScreenData.bat_line_open = UpdateData->bat_line_open;

    // 更新单体电池数据
    for(int i = 0; i < UpdateData->bat_num; i++)
    {
        ScreenData.bat_data[i].bat_voltage_single = UpdateData->bat_data[i].bat_voltage_single ;
        ScreenData.bat_data[i].bat_temp = UpdateData->bat_data[i].bat_temp;
        ScreenData.bat_data[i].bat_balance_status = UpdateData->bat_data[i].bat_balance_status ;
    }
}


/**
 * @brief 从maincontrol更新数据
 * @param UpdateData
 */
void update_screen_data_from_control(UpdateLoadData_t *UpdateData, uint32_t flag)
{
    //更新充放电过流标志
    if((ScreenData.overcharge_flag != UpdateData->overcharge_flag) || (ScreenData.overdischarge_flag != UpdateData->overdischarge_flag))
    {
        if(UpdateData->overdischarge_flag || UpdateData->overcharge_flag)
            ScreenData.over_alam_flag = RT_TRUE;                   //此标志位置高，马上跳转到报警界面
    }


    ScreenData.overcharge_flag = UpdateData->overcharge_flag;
    ScreenData.overdischarge_flag = UpdateData->overdischarge_flag;

    //更新输出电压电流频率数据
    ScreenData.ACFrenquency = UpdateData->ac_frequency;
    ScreenData.ACOutputCurrent = UpdateData->AC_output_current;
    ScreenData.ACOutputVoltage = UpdateData->AC_output_voltage;
    ScreenData.ACLoardPower = UpdateData->AC_load_power;
}

void update_gateway_communication_flag(uint8_t flag)
{
    if(flag == 0)
    {
        ScreenData.GatewayFlag = 0;
        ScreenData.GatewayTimeoutTick = rt_tick_get();
    }
    else if(rt_tick_get() - ScreenData.GatewayTimeoutTick > 2000)
    {
        ScreenData.GatewayFlag = 1;
    }
}

void update_BMS_communication_flag(uint8_t flag)
{
    if(flag == 0)
    {
        ScreenData.BMSFlag = 0;
        ScreenData.BMSTimeoutTick = rt_tick_get();
        //reset_battery_stm(BATTERY_STM_BMS_COMMUNICATION_ERROR);
    }
    else if(rt_tick_get() - ScreenData.BMSTimeoutTick > 5000)
    {
        ScreenData.BMSFlag = 1;
        //set_battery_stm(BATTERY_STM_BMS_COMMUNICATION_ERROR);
    }
}

rt_err_t lcd_send_buff(void *buff, uint16_t length)
{
    uint16_t send_length = 0;
    uint8_t *pbuff = (uint8_t *)buff;
    if(LCDDevice.Device == RT_NULL)
    {
        LOG_E("LCD device not exist\n");
        return RT_ERROR;
    }

   pbuff[length++] = 0xFF;
   pbuff[length++] = 0xFF;
   pbuff[length++] = 0xFF;

   send_length = rt_device_write(LCDDevice.Device, 0, pbuff, length);

   if(send_length == length)
       return RT_EOK;
   else
    return RT_ERROR;

}

/**
 * @brief 展示page0的状态信息
 */
void show_page0(void)
{
    uint8_t SendBuff[50] = "page page0";

    //切换页面
    lcd_send_buff(SendBuff, rt_strlen((char *)SendBuff));

    //电池电量
    rt_sprintf((char *)SendBuff, PAGE0_X, 1, ScreenData.bat_capiaity);
    lcd_send_buff(SendBuff, rt_strlen((char *)SendBuff));
    if(ScreenData.bat_capiaity > 20 && ScreenData.bat_capiaity <= 100)
    {
        rt_sprintf((char *)SendBuff, "page0.x1.pco=GREEN");
        lcd_send_buff(SendBuff, rt_strlen((char *)SendBuff));
    }
    else
    {
        rt_sprintf((char *)SendBuff, "page0.x1.pco=RED");
        lcd_send_buff(SendBuff, rt_strlen((char *)SendBuff));
    }
    //电池组总电压
    rt_sprintf((char *)SendBuff, PAGE0_X, 2, ScreenData.bat_voltage_total);
    lcd_send_buff(SendBuff, rt_strlen((char *)SendBuff));
    //放电电流
    rt_sprintf((char *)SendBuff, PAGE0_X, 3, ScreenData.discharge_current);
    lcd_send_buff(SendBuff, rt_strlen((char *)SendBuff));
    //充电电流
    rt_sprintf((char *)SendBuff, PAGE0_X, 4, ScreenData.charge_current);
    lcd_send_buff(SendBuff, rt_strlen((char *)SendBuff));
    //正极绝缘电阻
    rt_sprintf((char *)SendBuff, PAGE0_X, 5, ScreenData.insulation_resistance_Rp);
    lcd_send_buff(SendBuff, rt_strlen((char *)SendBuff));
    //负极绝缘电阻
    rt_sprintf((char *)SendBuff, PAGE0_X, 6, ScreenData.insulation_resistance_Rn);
    lcd_send_buff(SendBuff, rt_strlen((char *)SendBuff));

    //过流状态保护
//    if(ScreenData.overcharge_flag == RT_TRUE || ScreenData.overdischarge_flag == RT_TRUE)
//    {
//        rt_sprintf((char *)SendBuff, PAGE0_S, 12, "RED");
//        lcd_send_buff(SendBuff, rt_strlen((char *)SendBuff));
//    }
//    else {
//        rt_sprintf((char *)SendBuff, PAGE0_S, 12, "GREEN");
//            lcd_send_buff(SendBuff, rt_strlen((char *)SendBuff));
//    }

    //BMS 通讯标志位
    switch(ScreenData.BMSFlag)
    {
        case BMS_NORMAL:
            rt_sprintf((char *)SendBuff, PAGE0_S, 12, "GREEN");
            break;
        case BMS_DISCONNECT:
        case BMS_HEAD_ERROR:
        case BMS_CRC_ERROR:
            rt_sprintf((char *)SendBuff, PAGE0_S, 12, "RED");
            break;
    }
    lcd_send_buff(SendBuff, rt_strlen((char *)SendBuff));

}

//将page1指定到某一行
void page1_roll(uint8_t culunm)
{
    uint8_t SendBuff[30] = {0};
    rt_sprintf((char *)SendBuff, "data0.val_y=data0.hig*%d", culunm);
    lcd_send_buff(SendBuff, rt_strlen((char *)SendBuff));
}

rt_err_t __lcd_screen_init(const char *DeviceName)
{
    rt_device_t dev = RT_NULL;
    RT_ASSERT(DeviceName != RT_NULL);

    dev = rt_device_find(DeviceName);

    if (dev != RT_NULL && rt_device_open(dev, RT_DEVICE_OFLAG_RDWR | RT_DEVICE_FLAG_INT_RX) == RT_EOK)
    {
        LCDDevice.Device = dev;
        /* 设置接收回调函数 */
        rt_device_set_rx_indicate(LCDDevice.Device, __lcd_rx_ind);
    }
    else
    {
        if (LCDDevice.Device!= RT_NULL)
        {
            rt_device_close(LCDDevice.Device);
        }
        LOG_E("modbus can not find device: %s\n", DeviceName);
        return RT_ERROR;
    }

    //初始化ScreenData数据
    ScreenData.BMSTimeoutTick = 0;
    ScreenData.GatewayTimeoutTick = 0;

    ScreenData.SoftVersion = SOFTWARE_VERSION;

    return RT_EOK;
}

/**************************************新LCD绘制界面********************************************************/
#define BATTERY_INFO_PAGE_I         "page5"
#define BATTERY_SINGLE_INFO_PAGE_I  "page1"
#define ALARM_INFO_PAGE1_I           "page6"
#define ALARM_INFO_PAGE2_I          "page7"
#define OUTPUT_CONTROL_PAGE_I       "page8"
#define MENU_PAGE_I                 "page4"

#define LCD_PAGE_UPDATE_TICK_INTERVAL       1000

//页面类型枚举
typedef enum
{
    LCD_BATTERY_INFO_PAGE = 0x00,
    LCD_BATTERY_SINGLE_INFO_PAGE,
    LCD_ALARM_INFO_PAGE1,
    LCD_ALARM_INFO_PAGE2,
    LCD_OUTPUT_CONTROL_PAGE,
    LCD_MENU_PAGE,
    LCD_ALL_PAGE
}LCDPage_e;


void lcd_page_switch(LCDPage_e Page);
/************************电池信息界面*****************************/
#define INFO_PAGE_UPDATE_FORMATE    "%s.x%d.val=%d"

void _battery_info_page_show_background(void *parameter)
{
    char SendBuff[20] = {0};

    rt_sprintf(SendBuff, "page %s", BATTERY_INFO_PAGE_I);

    lcd_send_buff((uint8_t *)SendBuff, rt_strlen(SendBuff));
}

void _battery_info_page_func_key(void *parameter)
{
    lcd_page_switch(LCD_MENU_PAGE);
}

void _battery_info_page_update_data(void *parameter)
{
    char SendBuff[30] = {0};
    uint16_t UpdateData[10] = {0};

    UpdateData[0] = ScreenData.ACFrenquency;
    UpdateData[1] = ScreenData.bat_capiaity;
    UpdateData[2] = ScreenData.bat_voltage_total;
    UpdateData[3] = ScreenData.charge_current;
    UpdateData[4] = ScreenData.discharge_current;
    UpdateData[5] = ScreenData.insulation_resistance_Rp;
    UpdateData[6] = ScreenData.insulation_resistance_Rn;
    UpdateData[7] = ScreenData.ACOutputVoltage/100;
    UpdateData[8] = ScreenData.ACOutputCurrent;
    UpdateData[9] = ScreenData.ACLoardPower;

    rt_sprintf(SendBuff, "%s.x%d.txt=\"%s\"", BATTERY_INFO_PAGE_I, 0, "充电状态0");
    lcd_send_buff(SendBuff, rt_strlen(SendBuff));

    for(int i = 1; i < 10; i++)
    {
        rt_sprintf(SendBuff, INFO_PAGE_UPDATE_FORMATE, BATTERY_INFO_PAGE_I, i, UpdateData[i]);
        lcd_send_buff(SendBuff, rt_strlen(SendBuff));
    }
}

/**********************MENU 界面********************************************/
#define MENU_PAGE_BUTTON_SWITCH     "click bt%d,1"          //切换按键状态

#define MAX_BUTTON_NUM      4                               //当前菜单界面一共四个选项
uint8_t ButtonID = 0;

void _menu_page_show_backgroud(void *parameter)
{
    char SendBuff[20] = {0};

    rt_sprintf(SendBuff, "page %s", MENU_PAGE_I);

    lcd_send_buff((uint8_t *)SendBuff, rt_strlen(SendBuff));

    //切换到menu界面从按键0开始表示
    ButtonID = 0;
    rt_sprintf(SendBuff, MENU_PAGE_BUTTON_SWITCH, ButtonID);
    lcd_send_buff((uint8_t *)SendBuff, rt_strlen(SendBuff));
}

void _menu_page_add_key(void *parameter)
{
    char SendBuff[20] = {0};

    //add 键按下，将当前按键切换一下状态，再对下一个按键进行一次切换
    rt_sprintf(SendBuff, MENU_PAGE_BUTTON_SWITCH, ButtonID);
    lcd_send_buff((uint8_t *)SendBuff, rt_strlen(SendBuff));

    ButtonID = (ButtonID + 1)%MAX_BUTTON_NUM;

    rt_sprintf(SendBuff, MENU_PAGE_BUTTON_SWITCH, ButtonID);
    lcd_send_buff((uint8_t *)SendBuff, rt_strlen(SendBuff));
}

void _menu_page_del_key(void *parameter)
{
    char SendBuff[20] = {0};

    //add 键按下，将当前按键切换一下状态，再对上一个按键进行一次切换
    rt_sprintf(SendBuff, MENU_PAGE_BUTTON_SWITCH, ButtonID);
    lcd_send_buff((uint8_t *)SendBuff, rt_strlen(SendBuff));

    ButtonID = (ButtonID - 1 + MAX_BUTTON_NUM)%MAX_BUTTON_NUM;

    rt_sprintf(SendBuff, MENU_PAGE_BUTTON_SWITCH, ButtonID);
    lcd_send_buff((uint8_t *)SendBuff, rt_strlen(SendBuff));
}

void _menu_page_s1_key(void *parametr)
{
    LCDPage_e MenuItem[MAX_BUTTON_NUM] = {LCD_BATTERY_INFO_PAGE, LCD_BATTERY_SINGLE_INFO_PAGE,\
                                          LCD_ALARM_INFO_PAGE1, LCD_OUTPUT_CONTROL_PAGE};

    lcd_page_switch(MenuItem[ButtonID]);
}

void _menu_page_s2_key(void *paramter)
{
    lcd_page_switch(LCD_BATTERY_INFO_PAGE);
}


/************************单体电池数据界面**************************/
#define BATTERY_SINGLE_INFO_INSERT      "data0.insert(\"%d^%d.%dV^%d^%d.%d\")"    //往表格中插入单体电池数据
#define BATTERY_SINGLE_INFO_UPDATE      "data0.up(\"%d^%d.%03dV^%s^%d.%01d\",%d)"    //往表格中更新单体电池数据
#define BATTERY_SINGLE_INFO_UPDATE_     "data0.up(\"%d^%d.%03dV^%s^-%d.%01d\",%d)"    //往表格中更新带负数的温度

//清空单体数据表格
void __init_battery_single_info_datasheet(void)
{
    char SendBuff[80] = {0};

    rt_sprintf(SendBuff, "data0.clear()");
    lcd_send_buff(SendBuff, rt_strlen(SendBuff));

    for(int i = ScreenData.bat_num; i > 0; i--)
    {
        rt_memset(SendBuff, 0, sizeof(SendBuff));

        rt_sprintf(SendBuff, BATTERY_SINGLE_INFO_INSERT, i, 0, 0, 0, 0, 0);
        lcd_send_buff(SendBuff, rt_strlen(SendBuff));
    }
}

int8_t BatterySingleRowCount = 0;      //当前单体信息页面所在行数
void _battery_single_info_page_show_backgroud(void *parameter)
{
    char SendBuff[20] = {0};

    rt_sprintf(SendBuff, "page %s", BATTERY_SINGLE_INFO_PAGE_I);

    lcd_send_buff((uint8_t *)SendBuff, rt_strlen(SendBuff));

    //电池节数更新后，需要清空数据表，根据现有的数据重新填充
    if(ScreenData.battery_num_update_flag == RT_TRUE)
    {
        __init_battery_single_info_datasheet();
        ScreenData.battery_num_update_flag = RT_FALSE;
    }
}

void _battery_single_info_page_update_date(void *parameter)
{
    char SendBuff[80] = {0};
    char BalanceStatus[8] = {0};
    uint8_t BatteryNum = ScreenData.bat_num;

    for(int i = BatteryNum; i > 0; i--)
    {
        uint16_t Voltage_int, Voltage_f;
        int16_t Temp_int, Temp_f;


        rt_memset(SendBuff, 0, sizeof(SendBuff));
        Voltage_int = (uint16_t)(ScreenData.bat_data[i - 1].bat_voltage_single/1000);
        Voltage_f = (uint16_t)(ScreenData.bat_data[i - 1].bat_voltage_single%1000);
        Temp_int = (int16_t)(ScreenData.bat_data[i - 1].bat_temp/10);
        Temp_f = (int16_t)(ScreenData.bat_data[i - 1].bat_temp%10);

        //LOG_I("Battery[%02d] status: 0x%02X\n", i-1, ScreenData.bat_data[i - 1].bat_balance_status);
        if(ScreenData.bat_data[i - 1].bat_balance_status > 0)
        {
            rt_sprintf(BalanceStatus, "%s", "OPEN");
        }
        else
        {
            rt_sprintf(BalanceStatus, "%s", "CLOSE");
        }

        if(ScreenData.bat_data[i - 1].bat_temp < 0)
            rt_sprintf(SendBuff, BATTERY_SINGLE_INFO_UPDATE_, i, Voltage_int, Voltage_f, BalanceStatus,
                                        abs(Temp_int), abs(Temp_f), BatteryNum-i);
        else
            rt_sprintf(SendBuff, BATTERY_SINGLE_INFO_UPDATE, i, Voltage_int, Voltage_f, BalanceStatus,
                                Temp_int, Temp_f, BatteryNum-i);

        lcd_send_buff(SendBuff, rt_strlen(SendBuff));
    }
}


void _battery_single_info_page_add_key(void *parameter)
{
    BatterySingleRowCount -= 3;
    if(BatterySingleRowCount < 0)
        BatterySingleRowCount = 0;

    page1_roll(BatterySingleRowCount);
}

void _battery_single_info_page_del_key(void *parameter)
{
    BatterySingleRowCount += 3;

    if(BatterySingleRowCount > ScreenData.bat_num)
        BatterySingleRowCount = ScreenData.bat_num;

    page1_roll(BatterySingleRowCount);
}

/*********************报警信息界面********************************/       //报警界面有两个， 区分报警界面1和报警界面2
#define ALRAM_INFO_PAGE_SET_COLOR  "page2.t%d.bco=%s"                    // 填充报警色
#define ALARM_INFO_PAGE1_UPDATE    "bt%d.val=%d"                         // 切换双态按钮
#define ALARM_INFO_PAGE2_V_SWITCH   "vt%d.val=%d"                        //电压线开路按钮切换
uint8_t AlarmPage = 0;                                                   // 记录报警界面当前界面，切换报警时

void _alarm_info_page1_show_background(void *parameter)
{
    char SendBuff[20] = {0};

    rt_sprintf(SendBuff, "page %s", ALARM_INFO_PAGE1_I);

    lcd_send_buff(SendBuff, rt_strlen(SendBuff));
}

void _alarm_info_page2_show_background(void *parameter)
{
    char SendBuff[20] = {0};

    rt_sprintf(SendBuff, "page %s", ALARM_INFO_PAGE2_I);

    lcd_send_buff(SendBuff, rt_strlen(SendBuff));
}

void _alarm_info_page1_update_data(void *paramter)
{
    char SendBuff[40] = {0};
    //电压开路报警  温度开路报警
    for(int i = 1; i <= ScreenData.bat_num; i++)
    {
        if((ScreenData.bat_line_open & (1 << (i -1))) && (ScreenData.temp_line_open & (1 << (i - 1))))
            rt_sprintf((char *)SendBuff, ALRAM_INFO_PAGE_SET_COLOR, i, "RED");

        else if(ScreenData.bat_line_open & (1 << (i -1)))
            rt_sprintf((char *)SendBuff, ALRAM_INFO_PAGE_SET_COLOR, i, "BLUE");

        else if(ScreenData.temp_line_open & (1 << (i - 1)))
            rt_sprintf((char *)SendBuff, ALRAM_INFO_PAGE_SET_COLOR, i, "YELLOW");

        else
            rt_sprintf((char *)SendBuff, ALRAM_INFO_PAGE_SET_COLOR, i, "GREEN");

        lcd_send_buff(SendBuff, rt_strlen(SendBuff));
    }

    uint16_t AlarmFlag = ScreenData.abnormal_alarm;

    //报警标志赋值
    uint8_t flag[8] = {0};
    flag[0] = ScreenData.overcharge_flag;
    flag[1] = ScreenData.overdischarge_flag;
    flag[2] = IS_BAT_OVERCHARGE_ALARM_SET(AlarmFlag);
    flag[3] = IS_BAT_OVERDISCHARGE_ALARM_SET(AlarmFlag);
    flag[4] = IS_BAT_OVERTEMP_ALARM_SET(AlarmFlag);
    flag[5] = IS_BAT_LOWTEMP_ALARM_SET(AlarmFlag);
    flag[6] = IS_BAT_OVERCHARGE_FAIL_SET(AlarmFlag);
    flag[7] = IS_BAT_OVERDISCHARGE_FAIL_SET(AlarmFlag);

    //更新各个标志位
    for(int i = 0; i < 8; i++)
    {
        rt_sprintf(SendBuff, ALARM_INFO_PAGE1_UPDATE, i, flag[i]);
        lcd_send_buff(SendBuff, rt_strlen(SendBuff));
    }

}

void _alarm_info_page2_update_data(void *paramter)
{
    char SendBuff[40] = {0};
    //电压开路报警  温度开路报警
    for(int i = 0; i < ScreenData.bat_num; i++)
    {
        if(ScreenData.temp_line_open & (0x01 << i))
            rt_sprintf(SendBuff, ALARM_INFO_PAGE1_UPDATE, i, 1);
        else
            rt_sprintf(SendBuff, ALARM_INFO_PAGE1_UPDATE, i, 0);
        lcd_send_buff(SendBuff, rt_strlen(SendBuff));


        if(ScreenData.bat_line_open & (1 << i))
            rt_sprintf(SendBuff, ALARM_INFO_PAGE2_V_SWITCH, i, 1);
        else
            rt_sprintf(SendBuff, ALARM_INFO_PAGE2_V_SWITCH, i, 0);

        lcd_send_buff(SendBuff, rt_strlen(SendBuff));
    }
}

void _alarm_info_page1_del_key(void *parameter)
{
    lcd_page_switch(LCD_ALARM_INFO_PAGE2);
}

void _alarm_info_page2_add_key(void *parameter)
{
    lcd_page_switch(LCD_ALARM_INFO_PAGE1);
}

/*****************************************************************/
void _output_control_page_show_background(void *parameter)
{
    char SendBuff[20] = {0};

    rt_sprintf(SendBuff, "page %s", OUTPUT_CONTROL_PAGE_I);

    lcd_send_buff(SendBuff, rt_strlen(SendBuff));
}
/*****************************************************************/


void __lcd_nop_page(void *parameter)
{
    return;
}

void __jump_menu_page(void *parameter)
{
    lcd_page_switch(LCD_MENU_PAGE);
}
void __jump_battery_info_page(void *parameter)
{
    lcd_page_switch(LCD_BATTERY_INFO_PAGE);
}

//更新标题栏信息 通讯状态以及软件版本号
#define TITLE_BUTTON_SWITCH     "Tbt%d.val=%d"
#define TITLE_VERSION_UPDATE    "Tx0.val=%d"
void __update_title_info(void)
{
    char SendBuff[20] = {0};

    rt_sprintf(SendBuff, TITLE_BUTTON_SWITCH, 0, ScreenData.GatewayFlag);
    lcd_send_buff(SendBuff, rt_strlen(SendBuff));

    rt_sprintf(SendBuff, TITLE_BUTTON_SWITCH, 1, ScreenData.BMSFlag);
    lcd_send_buff(SendBuff, rt_strlen(SendBuff));

    rt_sprintf(SendBuff, TITLE_VERSION_UPDATE, ScreenData.SoftVersion);
    lcd_send_buff(SendBuff, rt_strlen(SendBuff));

}


//处理界面显示刷新
void __update_handle(void)
{
    static rt_tick_t LCDUpdateTick = 0;
    if(LCDDevice.LCDPageSwitchFlag)
    {
        if(LCDDevice.show_background != RT_NULL)
            LCDDevice.show_background(&LCDDevice);

        //更新后，强制刷新界面
        LCDUpdateTick = 0;
        LCDDevice.LCDPageSwitchFlag = RT_FALSE;
    }

    //定时刷新界面
    if(rt_tick_get() - LCDUpdateTick > LCD_PAGE_UPDATE_TICK_INTERVAL)
    {
        __update_title_info();

        if(LCDDevice.update_page != RT_NULL)
            LCDDevice.update_page(&LCDDevice);

        LCDUpdateTick = rt_tick_get();
    }
}

//处理按键事件
void __key_event_handle(void)
{
    rt_event_t ReEvent = get_remote_key_event();
    rt_uint32_t ReceiveEvent = 0;

    //通过按键对页面进行切换
    if(rt_event_recv(ReEvent, REMOTE_KEY_ALL_EVENT, RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR,
            RT_WAITING_NO, &ReceiveEvent) == RT_EOK)
    {
        if(ReceiveEvent & REMOTE_KEY_FUNC_EVENT)
        {
            LOG_D("FUNC KEY Pressed\n.");
            if(LCDDevice.function_key != RT_NULL)
                LCDDevice.function_key(&LCDDevice);
        }

        if(ReceiveEvent & REMOTE_KEY_ADD_EVENT)
        {
            LOG_D("ADD KEY Pressed\n.");
            if(LCDDevice.add_key != RT_NULL)
                LCDDevice.add_key(&LCDDevice);
        }

        if(ReceiveEvent & REMOTE_KEY_DEL_EVENT)
        {
            LOG_D("DEL KEY Pressed\n.");
            if(LCDDevice.del_key != RT_NULL)
                LCDDevice.del_key(&LCDDevice);
        }

        if(ReceiveEvent & REMOTE_KEY_S1_EVENT)
        {
            LOG_D("S1 KEY Pressed\n.");
            if(LCDDevice.s1_key != RT_NULL)
                LCDDevice.s1_key(&LCDDevice);
        }

        if(ReceiveEvent & REMOTE_KEY_S2_EVENT)
        {
            LOG_D("S2 KEY Pressed\n.");
            if(LCDDevice.s2_key != RT_NULL)
                LCDDevice.s2_key(&LCDDevice);
        }
    }
}

//处理突发事件 比如报警、比如超时返回。。。
void __burst_event_handle(void)
{
    if(ScreenData.over_alam_flag == RT_TRUE)
    {
        ScreenData.over_alam_flag = RT_FALSE;

        if(ScreenData.abnormal_alarm & 0xFF)
            lcd_page_switch(LCD_ALARM_INFO_PAGE1);
        else
            lcd_page_switch(LCD_ALARM_INFO_PAGE2);
    }
}


//用于在主循环中切换页面
void lcd_page_switch(LCDPage_e Page)
{
    LCDDevice.add_key = __lcd_nop_page;
    LCDDevice.del_key = __lcd_nop_page;
    LCDDevice.function_key = __lcd_nop_page;
    LCDDevice.update_page = __lcd_nop_page;
    LCDDevice.show_background = __lcd_nop_page;
    LCDDevice.s1_key = __lcd_nop_page;
    LCDDevice.s2_key = __lcd_nop_page;
    switch(Page)
    {
        case LCD_BATTERY_INFO_PAGE:
            LCDDevice.show_background = _battery_info_page_show_background;
            LCDDevice.update_page = _battery_info_page_update_data;
            LCDDevice.function_key = __jump_menu_page;
            break;
        case LCD_BATTERY_SINGLE_INFO_PAGE:
            LCDDevice.add_key = _battery_single_info_page_add_key;
            LCDDevice.del_key = _battery_single_info_page_del_key;
            LCDDevice.function_key = __jump_menu_page;
            LCDDevice.update_page = _battery_single_info_page_update_date;
            LCDDevice.show_background = _battery_single_info_page_show_backgroud;
            LCDDevice.s1_key = __lcd_nop_page;
            LCDDevice.s2_key = __jump_battery_info_page;
            break;
        case LCD_ALARM_INFO_PAGE1:
            LCDDevice.show_background = _alarm_info_page1_show_background;
            LCDDevice.update_page = _alarm_info_page1_update_data;
            LCDDevice.del_key = _alarm_info_page1_del_key;
            LCDDevice.function_key = __jump_menu_page;
            LCDDevice.s2_key = __jump_battery_info_page;
            break;
        case LCD_ALARM_INFO_PAGE2:
            LCDDevice.show_background = _alarm_info_page2_show_background;
            LCDDevice.update_page = _alarm_info_page2_update_data;
            LCDDevice.add_key = _alarm_info_page2_add_key;
            LCDDevice.function_key = __jump_menu_page;
            LCDDevice.s2_key = __jump_battery_info_page;
            break;
        case LCD_OUTPUT_CONTROL_PAGE:
            LCDDevice.show_background = _output_control_page_show_background;
            LCDDevice.function_key = __jump_menu_page;
            LCDDevice.s2_key = __jump_battery_info_page;
            break;
        case LCD_MENU_PAGE:
            LCDDevice.add_key = _menu_page_add_key;
            LCDDevice.del_key = _menu_page_del_key;
            LCDDevice.function_key = __lcd_nop_page;
            LCDDevice.update_page = __lcd_nop_page;
            LCDDevice.show_background = _menu_page_show_backgroud;
            LCDDevice.s1_key = _menu_page_s1_key;
            LCDDevice.s2_key = _menu_page_s2_key;
            break;
        default:
            break;
    }

    LCDDevice.LCDPageSwitchFlag = RT_TRUE;
}

void LCD_thread_entery(void *parameter)
{
    __init_remote_key_gpio();
    __lcd_screen_init(LCD_DEVICE_NAME);

    //初始切换到主体信息界面
    lcd_page_switch(LCD_BATTERY_INFO_PAGE);
    while(1)
    {
        __burst_event_handle();
        __update_handle();
        __key_event_handle();

        rt_thread_delay(20);
    }
}

static uint8_t LCDThreadStack[LCD_THREAD_STACK_SIZE] = { 0 };
static struct rt_thread lcd_thread;
static int lcd_thread_init(void)
{
    rt_err_t result = 0;
    result = rt_thread_init(&lcd_thread, "lcd", LCD_thread_entery, RT_NULL, LCDThreadStack,
            LCD_THREAD_STACK_SIZE, LCD_THREAD_PRIORITY, LCD_THREAD_TICK);

    if (RT_EOK == result)
    {
        LOG_D("thread[%s] init ok\n", lcd_thread.name);
        rt_thread_startup(&lcd_thread);
    }
    else
    {
        LOG_E("thread init fail\n");
    }

    return result;
}
INIT_APP_EXPORT(lcd_thread_init);

```

## 4. 主控板模块
```c
#include <applications/main_control_board.h>
#include <applications/StringNumberConvert.h>
/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2023-07-07     18197       the first version
 */

#define DBG_TAG "control"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

#include "TJC_Screen.h"
#include "modbus_register.h"
#include "global_var.h"
#include "adc_frequence.h"
/*********************************全局变量**********************************************/
ADC_Data_t ADCData;
/**************************************************************************************/

/* 主控引脚设置 */
struct GPIO_config
{
    char *Name;             //引脚名
    uint32_t Mode;          //模式配置
};
const struct GPIO_config GPIOConfig[] = { { AC_CONTROPL, GPIO_MODE_OUTPUT_PP },
        { CHARGING_CONTROL, GPIO_MODE_OUTPUT_PP },
        { BATTERY_CONTROL, GPIO_MODE_OUTPUT_PP },
        { MODBUS_RS485_CONTROL, GPIO_MODE_OUTPUT_PP },
        { BMS_RS485_CONTROL, GPIO_MODE_OUTPUT_PP },
        { AC_SWITCH,  GPIO_MODE_IT_RISING_FALLING },
        { AC_DETECTION, GPIO_MODE_IT_RISING_FALLING },
        { BAT_CHARGING_CURRENT, GPIO_MODE_ANALOG }, { BAT_DISCHARGING_CURRENT,  GPIO_MODE_ANALOG },
        { AC_FREQUENCY_IN, GPIO_MODE_AF_INPUT },
        { AC_FREQUENCY_CURRENT_OUT, GPIO_MODE_AF_INPUT },
        { AC_FREQUENCY_VOLTAGE, GPIO_MODE_AF_INPUT },
        { POWER24_CONTROL, GPIO_MODE_OUTPUT_PP},
        { POWER5_CONTROL, GPIO_MODE_OUTPUT_PP},
};

/*************************************主板控制状态机***************************************/
struct ControlSTM_t
{
    /* 电池模块状态机 */
    Battery_STM_Flag_e BatteySTM;    //电池状态机
    uint32_t BATAbnormalFlag;        //异常状态标志位  高16位为禁止放电开关， 低16位为禁止充电开关
    uint8_t BATSwitchFlag;           //电池状态发生切换标志

    rt_tick_t BATChargeDelayTick;     //当充电过流后，需要延迟3s后才能开启，当充电过流后，置零；过流后恢复，记录Tick.
    rt_tick_t BATOverchargeTick;
    uint8_t BATOverchargeCount;     //电池过充后，记录过充的时间，如果在20s内连续过充三次，永久关闭充电开关

    rt_tick_t BATDisChargeDelayTick;     //当放电过流后，需要延迟3s后才能开启，当放电过流后，置零；过流后恢复，记录Tick.
    rt_tick_t BATOverDischargeTick;
    uint8_t BATOverDischargeCount;     //电池放电过流后，记录过流的时间，如果在20s内连续过流三次，永久关闭放电开关

    /* 中断触发标志位 */
    uint32_t TriggleFlag;
    uint32_t TriggleLock;
    rt_tick_t TriggleTick1;                //中断防抖超时10ms
    rt_tick_t TriggleTick2;                //中断防抖超时10ms

    /* DC_SW触发状态*/
    GPIO_PinState  AC_SWFlag;                  //AC_SW 触发标志  用于检测是否处于维护放电， 维护放电时，放电电流不得低于阈值，在BMS判断
    uint8_t    MaintainDischargeFlag;          //维护放电标志，进入维护放电置1，退出维护放电置零。在维护放电模式下，交流切换不能影响状态

    /* 打印信息标志位 */
    FlagStatus PrintFlag;                  //打印信息
};
struct ControlSTM_t ControlSTM;

//提供给BMS检测AC——SW状态
rt_bool_t get_control_ac_sw_status(void)
{
    return ControlSTM.AC_SWFlag;
}
/**
 * @brief 设置电池状态机
 * @param flag
 */
rt_err_t set_battery_stm(Battery_STM_Flag_e flag)
{
    //输入不合法
    if (flag >= BATTERY_STM_ALL_FLAG)
        return RT_ERROR;

    if (flag == ControlSTM.BatteySTM)
        return RT_ERROR;

    //当准备跳转的状态为正常的几个状态，需要判断有无权限操作对应的开关
    if (flag < BATTERY_STM_ABNORMAL)
    {
        if(flag == BATTERY_STM_MAINTAINFUNCTION)
        {
            ControlSTM.MaintainDischargeFlag = RT_TRUE;
            modbus_register_update_data_from_control(MAINTENANCE_DISCHARGE_REGISTER, 1);
        }

        else if(flag == BATTERY_STM_MAINTAINFUNCTION_PROTECT)
        {
            ControlSTM.MaintainDischargeFlag = RT_FALSE;
            modbus_register_update_data_from_control(MAINTENANCE_DISCHARGE_REGISTER, 0);
        }
    }

    //切换到异常状态， 置位异常状态标志
    if (flag > BATTERY_STM_ABNORMAL)
    {
        if (flag >= BATTERY_STM_LOW_TEMP)
        {
            //当对应标志位置1时，不设置该状态
            if ((ControlSTM.BATAbnormalFlag & (0x0001 << (flag - BATTERY_STM_TEMP_OFFSET)))
                    && (ControlSTM.BATAbnormalFlag & (0x10000 << (flag - BATTERY_STM_TEMP_OFFSET))))
                return RT_EOK;

            ControlSTM.BATAbnormalFlag |= (0x0001 << (flag - BATTERY_STM_TEMP_OFFSET));
            ControlSTM.BATAbnormalFlag |= (0x10000 << (flag - BATTERY_STM_TEMP_OFFSET));
        }
        else if (flag >= BATTERY_STM_OVERDISCHARGING)
        {
            //当对应标志位置高时，不设置该状态
            if (ControlSTM.BATAbnormalFlag & (0x10000 << (flag - BATTERY_STM_OVERDISCHARGING)))
                return RT_EOK;

            ControlSTM.BATAbnormalFlag |= (0x10000 << (flag - BATTERY_STM_OVERDISCHARGING));
            ControlSTM.BATDisChargeDelayTick = 0;

            //距离上次进入过充状态超过10s，开始重新计数
            if (rt_tick_get() - ControlSTM.BATOverDischargeTick > 20000 && flag == BATTERY_STM_OVERDISCHARGING)
            {
                ControlSTM.BATOverDischargeTick = rt_tick_get();
                ControlSTM.BATOverDischargeCount = 0;
            }
            else
            {
                if (flag == BATTERY_STM_OVERDISCHARGING)
                {
                    if (++ControlSTM.BATOverDischargeCount >= 2)
                    {
                        ControlSTM.BATAbnormalFlag |= (0x10000
                                << (BATTERY_STM_OVERDISCHARGING_ALARM - BATTERY_STM_OVERDISCHARGING));
                    }
                }
            }
        }
        else if (flag >= BATTERY_STM_OVERCHARGING)
        {
            if (ControlSTM.BATAbnormalFlag & (0x0001 << (flag - BATTERY_STM_OVERCHARGING)))
                return RT_EOK;

            ControlSTM.BATAbnormalFlag |= (0x0001 << (flag - BATTERY_STM_OVERCHARGING));
            ControlSTM.BATChargeDelayTick = 0;

            //距离上次进入过充状态超过10s，开始重新计数
            if (rt_tick_get() - ControlSTM.BATOverchargeTick > 20000 && flag == BATTERY_STM_OVERCHARGING)
            {
                ControlSTM.BATOverchargeTick = rt_tick_get();
                ControlSTM.BATOverchargeCount = 0;
            }
            else
            {
                if (flag == BATTERY_STM_OVERCHARGING)
                {
                    if (++ControlSTM.BATOverchargeCount >= 2)
                    {
                        ControlSTM.BATAbnormalFlag |= (0x0001
                                << (BATTERY_STM_OVERCHARGING_ALARM - BATTERY_STM_OVERCHARGING));
                    }
                }
            }
        }
    }

    LOG_D("now alarm flag: 0x%04X  0x%02X\n", (uint16_t)(ControlSTM.BATAbnormalFlag >> 16), (uint16_t)(ControlSTM.BATAbnormalFlag)); LOG_D("Now control stm is %d\n", ControlSTM.BatteySTM);
    ControlSTM.BatteySTM = flag;
    ControlSTM.BATSwitchFlag = RT_TRUE;

    return RT_EOK;
}

//取消异常状态标志位
rt_err_t reset_battery_stm(Battery_STM_Flag_e flag)
{
    //输入不合法
    if (flag >= BATTERY_STM_ALL_FLAG || flag <= BATTERY_STM_ABNORMAL)
        return RT_ERROR;

    //取消异常状态标志
    if (flag >= BATTERY_STM_LOW_TEMP)
    {
        //对应标志位没有置位
        if ((ControlSTM.BATAbnormalFlag & (0x0001 << (flag - BATTERY_STM_TEMP_OFFSET))) == 0)
        {
            return RT_ERROR;
        }
        ControlSTM.BATAbnormalFlag ^= (0x0001 << (flag - BATTERY_STM_TEMP_OFFSET));
        ControlSTM.BATAbnormalFlag ^= (0x10000 << (flag - BATTERY_STM_TEMP_OFFSET));
    }
    else if (flag >= BATTERY_STM_OVERDISCHARGING)
    {
        if ((ControlSTM.BATAbnormalFlag & (0x10000 << (flag - BATTERY_STM_OVERDISCHARGING))) == 0)
        {
            return RT_ERROR;
        }

        //放电过流异常状态恢复需要 需要延迟一段时间, 记录第一次恢复时时间
        if (flag == BATTERY_STM_OVERDISCHARGING)
        {
            if (ControlSTM.BATDisChargeDelayTick == 0)
            {
                ControlSTM.BATDisChargeDelayTick = rt_tick_get();
            }
            //延迟3s后打开放电开关
            if (rt_tick_get() - ControlSTM.BATDisChargeDelayTick < 3000)
            {
                return RT_ERROR;
            }
        }

        ControlSTM.BATAbnormalFlag ^= (0x10000 << (flag - BATTERY_STM_OVERDISCHARGING));
    }
    else if (flag >= BATTERY_STM_OVERCHARGING)
    {
        if ((ControlSTM.BATAbnormalFlag & (0x0001 << (flag - BATTERY_STM_OVERCHARGING))) == 0)
        {
            return RT_ERROR;
        }

        //充电过流异常状态恢复需要 需要延迟一段时间, 记录第一次恢复时时间
        if (ControlSTM.BATChargeDelayTick == 0)
        {
            ControlSTM.BATChargeDelayTick = rt_tick_get();
        }
        //延迟3s后打开充电开关
        if (rt_tick_get() - ControlSTM.BATChargeDelayTick < 3000)
        {
            return RT_ERROR;
        }

        ControlSTM.BATAbnormalFlag ^= (0x0001 << (flag - BATTERY_STM_OVERCHARGING));
        ControlSTM.BATChargeDelayTick = 0;
    }

    //检查标志位是否全部清空，全部清空就将置为正常状态
    if (ControlSTM.BATAbnormalFlag == 0)
    {
        if (IS_AC_SWITCHED())
            ControlSTM.BatteySTM = BATTERY_STM_MAINTAINFUNCTION;

        else if (!IS_AC_DETECTED())
            ControlSTM.BatteySTM = BATTERY_STM_DISCHARGING;

        else
            ControlSTM.BatteySTM = BATTERY_STM_CHARGING;
    }

    LOG_D("now alarm flag: 0x%02X  0x%02X\n", (uint8_t)(ControlSTM.BATAbnormalFlag >> 8), (uint8_t)(ControlSTM.BATAbnormalFlag)); LOG_D("Now control stm is %d\n", ControlSTM.BatteySTM);
    ControlSTM.BATSwitchFlag = RT_TRUE;

    return RT_EOK;
}

/* 处理电池状态 */
void battery_STM_process(void)
{
    if (ControlSTM.BATSwitchFlag != RT_TRUE)
        return;

    switch (ControlSTM.BatteySTM)
    {
    case BATTERY_STM_LOW_TEMP:            //电池低温异常
    case BATTERY_STM_OVER_TEMP:                  //电池高温异常
    case BATTERY_STM_CHARGING_PROTECT_FAIL:      //电池过充保护失效
    case BATTERY_STM_DISCHARGING_PROTECT_FAIL:   //电池过放保护失效
    case BATTERY_STM_OPEN_CIRCUIT_ERROR:         //电池开路故障
    case BATTERY_STM_BMS_COMMUNICATION_ERROR:    //电池通讯故障
    {
        ENABLE_AC();
        DISABLE_BATTERY_CHARGING();
        DISABLE_BATTERY_OUTPUT();
        break;
    }

    case BATTERY_STM_OVERDISCHARGING:
    case BATTERY_STM_VLOTAGE_LOW_ALARM:          //电池过放报警
    case BATTERY_STM_LOW_CAPACITY_ALARM:         //低电量不让放电
    case BATTERY_STM_AC_SW_DISABLE:              //禁止电池对外输出
    {
        ENABLE_AC();

        //充电开关没被禁止
        if ((ControlSTM.BATAbnormalFlag & 0xFFFF) == 0)
            ENABLE_BATTERY_CHARGING();

        DISABLE_BATTERY_OUTPUT();

        break;
    }

    case BATTERY_STM_OVERCHARGING:        //充电过流异常
    case BATTERY_STM_VOLATGE_HIGH_ALARM:
    case BATTERY_STM_HIGH_CAPACITY_ALARM:       //电量充满后，低于95才让冲
    {
        ENABLE_AC();
        DISABLE_BATTERY_CHARGING();

        if ((ControlSTM.BATAbnormalFlag & 0xFFFF0000) == 0)
            ENABLE_BATTERY_OUTPUT();
        break;
    }


    case BATTERY_STM_MAINTAINFUNCTION:
        DISABLE_BATTERY_CHARGING();
        DISABLE_AC();

        if ((ControlSTM.BATAbnormalFlag & 0xFFFF0000) == 0)
            ENABLE_BATTERY_OUTPUT();
        break;

    case BATTERY_STM_CHARGING:
        ENABLE_AC();
        if ((ControlSTM.BATAbnormalFlag & 0xFFFF) == 0)
            ENABLE_BATTERY_CHARGING();

        ENABLE_BATTERY_OUTPUT();
        break;

    case BATTERY_STM_DISCHARGING:

        DISABLE_BATTERY_CHARGING();

        if ((ControlSTM.BATAbnormalFlag & 0xFFFF0000) == 0)
            ENABLE_BATTERY_OUTPUT();

        ENABLE_AC();
        break;

    //进入维护放电状态临界切换状态时，需要判断AC输入状态
    case BATTERY_STM_MAINTAINFUNCTION_PROTECT:

        ENABLE_AC();
        if (gpio_read_input(AC_DETECTION) == GPIO_PIN_SET)
        {
            set_battery_stm(BATTERY_STM_DISCHARGING);         //无交流输入，切换到电池输出，设置状态机
            modbus_register_update_data_from_control(OUTPUT_MODE_REGISTER, 1);
        }
        else if (gpio_read_input(AC_DETECTION) == GPIO_PIN_RESET)
        {
            set_battery_stm(BATTERY_STM_CHARGING);         //有交流输入， 设置状态机
            modbus_register_update_data_from_control(OUTPUT_MODE_REGISTER, 0);
        }
        break;

    default:
        break;
    }

    LOG_D("NOW Battery state has switched to 0x%02X\n", ControlSTM.BatteySTM); LOG_D("AC ON/OFF: %s, CHARGING ON/OFF:%s, DISCHARGING ON/OFF:%s\n", (GPIOA->ODR & GPIO_PIN_6)?"SET":"RESET",
            (GPIOA->ODR & GPIO_PIN_5)?"SET":"RESET" , (GPIOA->ODR & GPIO_PIN_7)?"SET":"RESET");
    ControlSTM.BATSwitchFlag = RT_FALSE;
}

/* 处理供电状态 */
void power_surply_process(void)
{

}

/********************************************ADC配置 开始*************************************************/
/* 处理ADC采样周期 */
rt_timer_t ADCSampleTimer;

uint16_t ADC_DMA_ConvertedValue[ADC1_CHANNEL_CNT * ADC1_CHANNEL_FRE] = { 0 };       //DMA 传输目的地址
uint16_t ADCSampleBuff[ADC1_CHANNEL_CNT * ADC1_CHANNEL_FRE] = { 0 };              //处理ADC 采样数据

//0：I2电池放电电流   1：I1 电池充电电流； 2 I_IN:交流电电流

//电流校准函数
//float ADC_value_to_discharge_current(float adc_value);
//float ADC_value_to_charge_current(float adc_value);

void __update_modbus_data(UpdateLoadData_t CurrentData)
{

}

uint16_t ADCSample1[ADC1_CHANNEL_FRE] = { 0 };
uint16_t ADCSample2[ADC1_CHANNEL_FRE] = { 0 };
uint16_t ADCSample3[ADC1_CHANNEL_FRE] = { 0 };
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;
void adc_sample_process(void)
{
    static rt_tick_t ADCSampleTick = 0;
    float ADCValue[ADC1_CHANNEL_CNT] = { 0 };
//    float ADCTemp[ADC1_CHANNEL_CNT] = {0};
//    current_update_t CurrentData = {0};

    //定时更新ADC数据
    if (rt_tick_get() - ADCSampleTick > ADC_SAMPLE_TIME_INTERVAL)
    {
        rt_memset(ADCSampleBuff, 0, sizeof(ADCSampleBuff));

        /* 从采集buff中获取数据 */
        HAL_ADC_Stop_DMA(&hadc1);
        rt_memcpy(ADCSampleBuff, ADC_DMA_ConvertedValue, sizeof(ADCSampleBuff));

        //滤波算法
        rt_memset(ADCValue, 0, sizeof(ADCValue));
        for (int i = 0; i < ADC1_CHANNEL_FRE; i++)
        {
            // 获取三个通道的ADC一个周期采样值和
            ADCValue[0] += ADCSampleBuff[i * ADC1_CHANNEL_CNT + 0];
//            ADCValue[1] += ADCSampleBuff[i*ADC1_CHANNEL_CNT + 1];

            ADCSample1[i] = ADCSampleBuff[i * ADC1_CHANNEL_CNT + 0];
//            ADCSample2[i] = ADCSampleBuff[i*ADC1_CHANNEL_CNT + 1];
        }

        ADCValue[0] = ADCValue[0] / ADC1_CHANNEL_FRE;
//        ADCValue[1] = ADCValue[1] / ADC1_CHANNEL_FRE;

//        ADCTemp[0] = ADC_filter_algorithm(ADCSample1, ADC1_CHANNEL_FRE);
//        ADCTemp[1] = ADC_filter_algorithm(ADCSample2, ADC1_CHANNEL_FRE);
//        ADCTemp[2] = ADC_filter_algorithm(ADCSample3, ADC1_CHANNEL_FRE);

//        LOG_D("SampleValue: %d, %d\n", (uint16_t)ADCValue[0], (uint16_t)ADCTemp[0]);
//        LOG_D("SampleValue: %d, %d\n", (uint16_t)ADCValue[1], (uint16_t)ADCTemp[1]);
//        LOG_D("SampleValue: %d, %d\n", (uint16_t)ADCValue[2], (uint16_t)ADCTemp[2]);

        //数据处理完成回调函数
//        CurrentData.discharge_current = (uint16_t)ADC_value_to_discharge_current(ADCValue[1]);
//        CurrentData.charge_current = (uint16_t) ADC_value_to_charge_current(ADCValue[0]);
//        CurrentData.discharge_current = (uint16_t) ADC_value_to_discharge_current(ADCData.ADC_ACCurrent);
////        CurrentData.charge_current = 0;
//        //现在充电时，放电电流采样依旧会有电压，强制置零。
//        if (CurrentData.charge_current > 200)
//        {
//            CurrentData.discharge_current = 0;
//        }
//
//        //充电过流
//        if (CurrentData.charge_current > ADC_CHARGE_THRESHOLD)
//        {
//            CurrentData.overcharge_flag = RT_TRUE;
//            set_battery_stm(BATTERY_STM_OVERCHARGING);
//        }
//        else
//        {
//            reset_battery_stm(BATTERY_STM_OVERCHARGING);
//            //永久充电过流保护标志
//            if ((ControlSTM.BATAbnormalFlag & (0x0001 << (BATTERY_STM_OVERCHARGING_ALARM - BATTERY_STM_OVERCHARGING)))
//                    || (ControlSTM.BATAbnormalFlag & (0x0001 << (BATTERY_STM_OVERCHARGING - BATTERY_STM_OVERCHARGING))))
//                CurrentData.overcharge_flag = RT_TRUE;
//            else
//                CurrentData.overcharge_flag = RT_FALSE;
//
////            if(reset_battery_stm(BATTERY_STM_OVERCHARGING) == RT_EOK)
////                CurrentData.overcharge_flag = RT_FALSE;
//        }
//
//        //放电过流 防止上电瞬间，电流过载导致关断
//        if (CurrentData.discharge_current > ADC_DISCHARGE_THRESHOLD && (CurrentData.last_discharge_current > 100))
//        {
//            CurrentData.overdischarge_flag = RT_TRUE;
//            set_battery_stm(BATTERY_STM_OVERDISCHARGING);
//        }
//        else
//        {
//            reset_battery_stm(BATTERY_STM_OVERDISCHARGING);
//
//            //永久充电过流保护标志
//            if ((ControlSTM.BATAbnormalFlag
//                    & (0x10000 << (BATTERY_STM_OVERDISCHARGING_ALARM - BATTERY_STM_OVERDISCHARGING)))
//                    || (ControlSTM.BATAbnormalFlag
//                            & (0x10000 << (BATTERY_STM_OVERDISCHARGING - BATTERY_STM_OVERDISCHARGING))))
//                CurrentData.overdischarge_flag = RT_TRUE;
//            else
//                CurrentData.overdischarge_flag = RT_FALSE;
//        }
        //向MODBUS更新数据 三个电流、一个电压

        ADCSampleTick = rt_tick_get();
        if (HAL_ADC_Start_DMA(&hadc1, (uint32_t*) &ADC_DMA_ConvertedValue, (ADC1_CHANNEL_CNT * ADC1_CHANNEL_FRE))
                != HAL_OK)
        {
            /* Start Conversation Error */
            LOG_E("adc error");
        }
    }
}

//转换出来电压单位为V
float ADC_value_to_ac_output_voltage(uint16_t adc_value)
{
    float a = -465930;
    float b = 225.94;
    float result = 0.0;

    result = b*adc_value + a;

    return result;
}

//转换出来电流单位为mA
float ADC_value_to_ac_output_current(uint16_t adc_value)
{
    float a = -12006;
    float b = 5.8207;

    float result = b*adc_value + a;

    return result;
}
/********************************************ADC配置 结束*************************************************/

/*****************************************TIM CH2配置 开始*************************************************/
TIM_HandleTypeDef htim2;

#define TIM2_PRESCALER      7200
#define TIM2_PERIOD         0xFFFF

//初始化TIM2 CH2作为PWM输入检测
static void MX_TIM2_Init(void)
{
    TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
    TIM_MasterConfigTypeDef sMasterConfig = { 0 };
    TIM_IC_InitTypeDef sConfigIC = { 0 };

    /* USER CODE BEGIN TIM2_Init 1 */

    /* USER CODE END TIM2_Init 1 */
    htim2.Instance = TIM2;
    htim2.Init.Prescaler = TIM2_PRESCALER - 1;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = TIM2_PERIOD;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
    {
        Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
    {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }
    sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
    sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
    sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
    sConfigIC.ICFilter = 0;
    if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
    {
        Error_Handler();
    }
}

//TIM2 底层初始化
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* htim_base)
{
    if (htim_base->Instance == TIM2)
    {
        /* Peripheral clock enable */
        __HAL_RCC_TIM2_CLK_ENABLE();

        /* TIM2 interrupt Init */
        HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(TIM2_IRQn);
    }
}

void __init_AC_frequency_detect(void)
{
    MX_TIM2_Init();

    ADCData.ADC_ACFreCount = 0;
    ADCData.ADC_ACFrequence = 0;
    ADCData.UpdateFlag = 0;
    rt_memset(ADCData.ADC_ACFreTemp, 0, sizeof(ADCData.ADC_ACFreTemp));
}

//TIM2 中断函数
void TIM2_IRQHandler(void)
{
    uint32_t Value = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_2);

    if (Value != 0)
    {
        ADCData.ADC_ACFreTemp[ADCData.ADC_ACFreCount++] = Value;

        if (ADCData.ADC_ACFreCount == ADC_ADC_ACFre_Temp_NUM)
        {
            HAL_TIM_IC_Stop_IT(&htim2, TIM_CHANNEL_2);
        }
    }
}

/**
 * @brief 计算交流电频率
 * @param frequency
 * @return 计算成功，返回RT_EOK；失败，返回RT_ERROR
 */
rt_err_t calculate_ac_frequency(uint16_t *frequency)
{
    HAL_TIM_IC_Stop_IT(&htim2, TIM_CHANNEL_2);

    //产生足够数量的数据
    if (ADCData.ADC_ACFreCount == ADC_ADC_ACFre_Temp_NUM)
    {
        float SampleCount = 0.0;

        //计算每两次中断之间的计数， 并过滤掉毛刺带来的毛刺
        for (int i = 0; i < ADC_ADC_ACFre_Temp_NUM - 1; i++)
        {
            SampleCount += (ADCData.ADC_ACFreTemp[i + 1] - ADCData.ADC_ACFreTemp[i] + TIM2_PERIOD + 1)%(TIM2_PERIOD + 1);
        }

        for(int i = 0; i < ADC_ADC_ACFre_Temp_NUM; i++)
            LOG_D("Frequency Buff[%d]: %04d", i,  ADCData.ADC_ACFreTemp[i]);

        SampleCount = SampleCount / (ADC_ADC_ACFre_Temp_NUM - 1);

        if (SampleCount != 0)
        {
            ADCData.ADC_ACFrequence = (uint16_t) (72000000 / TIM2_PRESCALER / SampleCount);
        }

        ADCData.ADC_ACFreCount = 0;
        ADCData.UpdateFlag = 1;
        HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);
    }
    //有可能并没有产生中断，没有波动
    else
    {
        ADCData.ADC_ACFrequence = 0;
        ADCData.ADC_ACFreCount = 0;
        ADCData.UpdateFlag = 1;
        HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);
    }

    return RT_EOK;
}

void AC_sample_process(void)
{
    static rt_tick_t ACFreTick = 0;

    //数据记录数量到6个后
    if (rt_tick_get() - ACFreTick > 1000)
    {
        HAL_TIM_IC_Stop_IT(&htim2, TIM_CHANNEL_2);

        //产生足够数量的数据
        if (ADCData.ADC_ACFreCount == ADC_ADC_ACFre_Temp_NUM)
        {
            float SampleCount = 0.0;

            //计算每两次中断之间的计数， 并过滤掉毛刺带来的毛刺
            for (int i = 0; i < ADC_ADC_ACFre_Temp_NUM - 1; i++)
            {
                SampleCount += (ADCData.ADC_ACFreTemp[i + 1] - ADCData.ADC_ACFreTemp[i] + TIM2_PERIOD + 1)%(TIM2_PERIOD + 1);
            }

            for(int i = 0; i < ADC_ADC_ACFre_Temp_NUM; i++)
                LOG_D("Frequency Buff[%d]: %04d", i,  ADCData.ADC_ACFreTemp[i]);

            SampleCount = SampleCount / (ADC_ADC_ACFre_Temp_NUM - 1);

            if (SampleCount != 0)
            {
                ADCData.ADC_ACFrequence = (uint16_t) (72000000 / TIM2_PRESCALER / SampleCount);
            }

            ADCData.ADC_ACFreCount = 0;
            ADCData.UpdateFlag = 1;
            HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);
        }
        //有可能并没有产生中断，没有波动
        else
        {
            ADCData.ADC_ACFrequence = 0;
            ADCData.ADC_ACFreCount = 0;
            ADCData.UpdateFlag = 1;
            HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);
        }

        ACFreTick = rt_tick_get();
    }

    //采集交流电
    PeriodicSampleResults_t Results;
    UpdateLoadData_t LoadData;
    Results= get_periodic_signal_value(0, AD_CURRENT_CHANNEL_RANK);

    ADCData.ADC_Current_test = (uint16_t)Results.ACCurrent.BaseAverageResult;
    ADCData.ADC_volatge_test = (uint16_t)Results.ACOutputVoltage.BaseAverageResult;

    ADCData.ADC_ADCOutputVolage = (uint32_t)(ADC_value_to_ac_output_voltage(Results.ACOutputVoltage.BaseAverageResult)*1000);
    ADCData.ADC_ACCurrent = (uint16_t)(ADC_value_to_ac_output_current(Results.ACCurrent.BaseAverageResult));
    ADCData.ADC_ADCOutputVolage = (uint32_t)(ADC_value_to_ac_output_voltage(Results.ACOutputVoltage.BaseAverageResult));
    ADCData.ADC_LoadPower = (uint16_t)((ADCData.ADC_ACCurrent/1000.0)*(ADCData.ADC_ADCOutputVolage/1000.0));

    //更新负载数据
    LoadData.ac_frequency = ADCData.ADC_ACFrequence;
    LoadData.AC_output_current = ADCData.ADC_ACCurrent;
    LoadData.AC_output_voltage = ADCData.ADC_ADCOutputVolage;
    LoadData.AC_load_power = ADCData.ADC_LoadPower;

    //更新电流过流标志， 为了使显示屏上的标识与充放电开关保持一致
    if((ControlSTM.BATAbnormalFlag & 0x0001) ||
       (ControlSTM.BATAbnormalFlag & (0x0001 << (BATTERY_STM_OVERCHARGING_ALARM - BATTERY_STM_OVERCHARGING))))
        LoadData.overcharge_flag = RT_TRUE;
    else
        LoadData.overcharge_flag = RT_FALSE;

    if((ControlSTM.BATAbnormalFlag & 0x10000) ||
            (ControlSTM.BATAbnormalFlag & (0x10000 << (BATTERY_STM_OVERDISCHARGING_ALARM - BATTERY_STM_OVERDISCHARGING))))
        LoadData.overdischarge_flag = RT_TRUE;
    else
        LoadData.overdischarge_flag = RT_FALSE;

    update_screen_data_from_control(&LoadData, ControlSTM.BATAbnormalFlag);

    //向MODBUS寄存器更新数据
    modbus_register_update_data_from_control(OUTPUT_FREQUENCY_REGISTER, ADCData.ADC_ACFrequence);
    modbus_register_update_data_from_control(LOAD_CURRENT_REGISTER, ADCData.ADC_ACCurrent);
    modbus_register_update_data_from_control(LOARD_PWOER_REGISTER, ADCData.ADC_LoadPower);

    //输出电压32位长，分两次更新到寄存器里面
    modbus_register_update_data_from_control(OUTPUT_VOLTAGE_REGISTER, (uint16_t)(ADCData.ADC_ADCOutputVolage >> 16));
    modbus_register_update_data_from_control(OUTPUT_VOLTAGE_REGISTER + 1, (uint16_t)(ADCData.ADC_ADCOutputVolage));
}

/*****************************************TIM CH2配置 结束*************************************************/

/**
 * @brief 初始化主控板控制状态机
 */
void __init_control_STM(void)
{

    ControlSTM.BatteySTM = BATTERY_STM_CHARGING;    //默认上电为充电状态
    ControlSTM.BatteySTM = RT_TRUE;
    ControlSTM.BATSwitchFlag = RT_TRUE;
    ControlSTM.BATChargeDelayTick = 0;
    ControlSTM.BATOverchargeTick = 0;
    ControlSTM.BATOverchargeCount = 0;

    ControlSTM.TriggleFlag = 0;
    ControlSTM.TriggleLock = 0;
    ControlSTM.TriggleTick1 = 0;
    ControlSTM.TriggleTick2 = 0;

   ControlSTM.AC_SWFlag = GPIO_PIN_RESET;
   ControlSTM.MaintainDischargeFlag = RT_TRUE;

    //开机读取AC 开关状态
    if(gpio_read_input(AC_SWITCH) == GPIO_PIN_SET)
    {
        set_battery_stm(BATTERY_STM_MAINTAINFUNCTION);
        return;
    }

    //读取交流输入状态
    if(gpio_read_input(AC_DETECTION) == GPIO_PIN_RESET)
    {
        set_battery_stm(BATTERY_STM_CHARGING);
        modbus_register_update_data_from_control(OUTPUT_MODE_REGISTER, 1);
    }
    else
    {
        set_battery_stm(BATTERY_STM_DISCHARGING);
        modbus_register_update_data_from_control(OUTPUT_MODE_REGISTER, 0);
    }
}

/* 打印状态机信息 */
void print_STM_info(void)
{
    LOG_D("\n\n"); LOG_D("Control STM      : 0x%02X\n", ControlSTM.BatteySTM);
    LOG_D("alarm flag       : 0x%04X  0x%04X\n", (uint16_t)(ControlSTM.BATAbnormalFlag >> 16), (uint16_t)(ControlSTM.BATAbnormalFlag));
    LOG_D("TriggleFlag      : 0x%02X\n", ControlSTM.TriggleFlag); LOG_D("TriggleLock      : 0x%02X\n", ControlSTM.TriggleLock);
    LOG_D("AC SW State: %d\n", gpio_read_input(AC_SWITCH));
    LOG_D("GPIOA ODR:0x%04X\n", GPIOA->ODR);
    LOG_D("AC ON/OFF: %s, CHARGING ON/OFF:%s, DISCHARGING ON/OFF:%s\n", (GPIOB->ODR & GPIO_PIN_5)?"SET":"RESET",
            (GPIOA->ODR & GPIO_PIN_5)?"SET":"RESET" , (GPIOA->ODR & GPIO_PIN_7)?"SET":"RESET");
    LOG_D("ADC Current: %d, AC Current test: %d\n", ADCData.ADC_ACCurrent, ADCData.ADC_Current_test);
    LOG_D("AC Voltage: %d, AC voltage test: %d\n", ADCData.ADC_ADCOutputVolage, ADCData.ADC_volatge_test);
    LOG_D("AC Frequency: %dHz", ADCData.ADC_ACFrequence);
    LOG_D("\n\n");

}
/*******************************************主板状态机 结束*************************************************************/

/**
 * @brief 从配置列表中初始化配置GPIO口，复用功能需要额外配置
 */
void __init_control_board_gpio(void)
{
    uint8_t ItemNum = sizeof(GPIOConfig) / sizeof(GPIOConfig[0]);
    __HAL_RCC_AFIO_CLK_ENABLE();
    __HAL_AFIO_REMAP_SWJ_NOJTAG();

    for (int i = 0; i < ItemNum; i++)
    {
        switch (GPIOConfig[i].Mode)
        {
        case GPIO_MODE_OUTPUT_PP:
            gpio_init_output(GPIOConfig[i].Name);
            break;
        case GPIO_MODE_IT_RISING_FALLING:
            gpio_init_exti(GPIOConfig[i].Name);
            break;
        case GPIO_MODE_ANALOG:
            gpio_init_analog(GPIOConfig[i].Name);
            break;
        case GPIO_MODE_AF_INPUT:
            gpio_init_af_in(GPIOConfig[i].Name);
            break;
        }
    }

    //两组电源控制引脚导通
    gpio_write_output(POWER24_CONTROL, GPIO_PIN_SET);
    gpio_write_output(POWER5_CONTROL, GPIO_PIN_SET);

    //初始化引脚输出
    gpio_write_output(AC_CONTROPL, GPIO_PIN_SET);
    gpio_write_output(CHARGING_CONTROL, GPIO_PIN_SET);
    gpio_write_output(BATTERY_CONTROL, GPIO_PIN_RESET);

    //检测到AC——SW置高了 断交流输入, 打开电池输出，关闭电池输入
    if (IS_AC_SWITCHED())
    {
        DISABLE_AC();
        ENABLE_BATTERY_OUTPUT();
        DISABLE_BATTERY_CHARGING();
    }
    else
    {
        ENABLE_AC();
        ENABLE_BATTERY_CHARGING();
        DISABLE_BATTERY_OUTPUT();
    }

}

/*****************************************ADC配置 开始 ***********************************/
/* ADC ADC_DMA 句柄 */

/**
 * @brief ADC MSP Initialization
 * This function configures the hardware resources used in this example
 * @param hadc: ADC handle pointer
 * @retval None
 */
void HAL_ADC_MspInit(ADC_HandleTypeDef* hadc)
{
//    if (hadc->Instance == ADC1)
//    {
//        /* Peripheral clock enable */
//        __HAL_RCC_ADC1_CLK_ENABLE()
//        ;
//        /* ADC1 DMA Init */
//        /* ADC1 Init */
//        hdma_adc1.Instance = DMA1_Channel1;
//        hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
//        hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
//        hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;
//        hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
//        hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
//        hdma_adc1.Init.Mode = DMA_CIRCULAR;
//        hdma_adc1.Init.Priority = DMA_PRIORITY_MEDIUM;
//        if (HAL_DMA_Init(&hdma_adc1) != HAL_OK)
//        {
//            RT_ASSERT(RT_FALSE);
//        }
//
//        __HAL_LINKDMA(hadc, DMA_Handle, hdma_adc1);
//    }

}

//DMA1_Channel1_IRQHandler

/* ADC DMA传输配置*/
void __init_ADC_DMA_sample(void)
{
    /* 使能DMA中断传输 */
    __HAL_RCC_DMA1_CLK_ENABLE()
    ;
    //HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
    //HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

    ADC_ChannelConfTypeDef sConfig = { 0 };

    hadc1.Instance = ADC1;
    hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
    hadc1.Init.ContinuousConvMode = ENABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion = ADC1_CHANNEL_CNT;
    if (HAL_ADC_Init(&hadc1) != HAL_OK)
    {
        RT_ASSERT(RT_FALSE);
    }

    /* 配置通道 */
    sConfig.Channel = ADC_CHANNEL_0;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_71CYCLES_5;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
        RT_ASSERT(RT_FALSE);
    }

    /** Configure Regular Channel
     */
//    sConfig.Channel = ADC_CHANNEL_8;
//    sConfig.Rank = ADC_REGULAR_RANK_2;
//    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
//    {
//        RT_ASSERT(RT_FALSE);
//    }
//    sConfig.Channel = ADC_CHANNEL_9;
//    sConfig.Rank = ADC_REGULAR_RANK_3;
//    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
//    {
//        RT_ASSERT(RT_FALSE);
//    }
    if (HAL_OK != HAL_ADCEx_Calibration_Start(&hadc1))
        LOG_E("ADC calibration error\n");

}

/*****************************************ADC配置 结束 **************************************/

/*********************************中断处理开始 **********************************************/
/**
 * @brief 外部EXTI中断处理函数
 */
void EXTI4_IRQHandler(void)
{
    rt_interrupt_enter();
    /* EXTI line interrupt detected */
    if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_4) != 0x00u)
    {
        if ((ControlSTM.TriggleLock & GPIO_PIN_4) != GPIO_PIN_4)
        {
            ControlSTM.TriggleFlag |= GPIO_PIN_4;                         //中断触发标志
            ControlSTM.TriggleLock |= GPIO_PIN_4;                          //短时间内不容许触发多次，防抖
            ControlSTM.TriggleTick1 = rt_tick_get();
        }

        __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_4);
    }

    rt_interrupt_leave();
}

void EXTI3_IRQHandler(void)
{
    rt_interrupt_enter();
    /* EXTI line interrupt detected */
    if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_3) != 0x00u)
    {
        if ((ControlSTM.TriggleLock & GPIO_PIN_3) != GPIO_PIN_3)
        {
            ControlSTM.TriggleFlag |= GPIO_PIN_3;                         //中断触发标志
            ControlSTM.TriggleLock |= GPIO_PIN_3;                          //短时间内不容许触发多次，防抖
            ControlSTM.TriggleTick2 = rt_tick_get();
        }

        __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_3);
    }
    rt_interrupt_leave();
}

/**
 * @brief 延时处理按键事件
 */
void exti_STM_process(void)
{
    /* GPIO_PIN_4 中断处理*/
    if (((ControlSTM.TriggleFlag & GPIO_PIN_4) == GPIO_PIN_4) && (ControlSTM.TriggleLock & GPIO_PIN_4) == GPIO_PIN_4)
    {
        if (((rt_tick_get() - ControlSTM.TriggleTick1) > 500) && ControlSTM.TriggleTick1 != 0)
        {
            if (gpio_read_input(AC_SWITCH) == GPIO_PIN_SET)
            {
                set_battery_stm(BATTERY_STM_MAINTAINFUNCTION);          //关闭交流输入，设置状态机 开启维护放电
                ControlSTM.AC_SWFlag = RT_TRUE;
                modbus_register_update_data_from_control(MAINTENANCE_DISCHARGE_REGISTER, 1);
            }
            else if (gpio_read_input(AC_SWITCH) == GPIO_PIN_RESET)
            {
//                if (gpio_read_input(AC_DETECTION) == GPIO_PIN_SET)
//                {
//                    set_battery_stm(BATTERY_STM_DISCHARGING);         //无交流输入，切换到电池输出，设置状态机
//                    modbus_register_update_data_from_control(OUTPUT_MODE_REGISTER, 1);
//                }
//                else if (gpio_read_input(AC_DETECTION) == GPIO_PIN_RESET)
//                {
//                    set_battery_stm(BATTERY_STM_CHARGING);         //有交流输入， 设置状态机
//                    modbus_register_update_data_from_control(OUTPUT_MODE_REGISTER, 0);
//                }

                set_battery_stm(BATTERY_STM_MAINTAINFUNCTION_PROTECT);         //关闭维护放电
//                    modbus_register_update_data_from_control(OUTPUT_MODE_REGISTER, 0);
                ControlSTM.AC_SWFlag = RT_FALSE;
                modbus_register_update_data_from_control(MAINTENANCE_DISCHARGE_REGISTER, 0);
            }

            ControlSTM.TriggleFlag &= (~GPIO_PIN_4);
            ControlSTM.TriggleLock &= (~GPIO_PIN_4);
            ControlSTM.TriggleTick1 = 0;

            ControlSTM.PrintFlag = SET;
        }
    }

    /* GPIO_PIN_3 中断处理*/
    if (((ControlSTM.TriggleFlag & GPIO_PIN_3) == GPIO_PIN_3) && (ControlSTM.TriggleLock & GPIO_PIN_3) == GPIO_PIN_3)
    {
        if (((rt_tick_get() - ControlSTM.TriggleTick2) > 500) && ControlSTM.TriggleTick2 != 0)
        {
            //当处于维护放电状态时，有无交流输入切换都不能影响引起状态切换
            if(ControlSTM.MaintainDischargeFlag == RT_TRUE)
                return;

            if (gpio_read_input(AC_DETECTION) == GPIO_PIN_SET)
            {
                set_battery_stm(BATTERY_STM_DISCHARGING);         //无交流输入，切换到电池输出，设置状态机

                ControlSTM.AC_SWFlag = RT_FALSE;                    //维护放电时断交流，直接退出维护放电
                modbus_register_update_data_from_control(OUTPUT_MODE_REGISTER, 1);
            }
            else if (gpio_read_input(AC_DETECTION) == GPIO_PIN_RESET)
            {
                set_battery_stm(BATTERY_STM_CHARGING);         //有交流输入， 设置状态机

                ControlSTM.AC_SWFlag = gpio_read_input(AC_SWITCH);  //交流输入时，状态检测是不是AC开关打开的，是的话，进入维护放电
                modbus_register_update_data_from_control(OUTPUT_MODE_REGISTER, 0);
            }

            ControlSTM.TriggleFlag &= (~GPIO_PIN_3);
            ControlSTM.TriggleLock &= (~GPIO_PIN_3);
            ControlSTM.TriggleTick2 = 0;

            ControlSTM.PrintFlag = SET;
        }
    }
}
/**********************************中断处理 结束 *******************************************/
/**
 * @brief 状态机处理函数
 */
void control_STM_process(void)
{
    static rt_tick_t STM_PRINT_TICK = 0;
    /* EXTI4 中断处理, 需要延时防抖*/
    exti_STM_process();

    /* 依据状态机处理各个引脚控制 */
    battery_STM_process();
    power_surply_process();

    /* ADC 采样处理函数 */
//    adc_sample_process();
    /* 交流电采样处理函数 */
    AC_sample_process();

    /* 打印状态机信息 */
    if (ControlSTM.PrintFlag == SET || (rt_tick_get() - STM_PRINT_TICK > 2000))
    {
        print_STM_info();
        ControlSTM.PrintFlag = RESET;
        STM_PRINT_TICK = rt_tick_get();

    }
}
/**
 * @brief 线程入口函数
 * @param parameter
 */
void main_control_thread_entry(void *parameter)
{
    __init_control_STM();
    __init_control_board_gpio();
//    __init_ADC_DMA_sample();
    __init_AC_frequency_detect();
    init_periodic_signal_sample();

//    if(HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&ADC_DMA_ConvertedValue, (ADC1_CHANNEL_CNT * ADC1_CHANNEL_FRE)) != HAL_OK)
//    {
//        /* Start Conversation Error */
//        LOG_E("adc error");
//    }

    if (HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2) != HAL_OK)
    {
        LOG_E("TIM2 error");
    }

    while (1)
    {
        control_STM_process();
        rt_thread_delay(2);

    }
}

static uint8_t main_control_thread_stack[MAIN_CONTROL_THREAD_STACK_SIZE] = { 0 };
static struct rt_thread control_thread;
static int main_control_board_thread_init(void)
{
    rt_err_t result = 0;
    result = rt_thread_init(&control_thread, "control", main_control_thread_entry, RT_NULL, main_control_thread_stack,
    MAIN_CONTROL_THREAD_STACK_SIZE, MAIN_CONTROL_THREAD_PRIORITY, MAIN_CONTROL_THREAD_TICK);

    if (RT_EOK == result)
    {
        LOG_D("thread[%s] init ok\n", control_thread.name);
        rt_thread_startup(&control_thread);
    }
    else
    {
        LOG_E("thread init fail\n");
    }

    return result;
}
INIT_APP_EXPORT(main_control_board_thread_init);

```