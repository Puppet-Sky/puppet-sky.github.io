---
title: SimpleFOC 移植
layout: page
---

# 硬件选型

- 电机：2804，7 极对，额定电压 12 V（7.4~16 V DC），绕线电阻 5.1 Ω，相电阻 2.3 Ω，KV 220，相电感 0.86 mH。开环电压和极对数按这组数填。
- 编码器：AS5600，I2C 地址 `0x36`，12 位，角度寄存器 `0x0C`。开环不拿它做反馈，只拿来看实际转速。
- 驱动板：MKS SimpleFOC Shield V3.2。使能 PA6，三相 PA8 / PA9 / PA10，接到 TIM1 的 CH1 / CH2 / CH3。

板子是 STM32F407ZGT6，168 MHz，系统是 RT-Thread。应用层用 C，FOC 算法继续用原来的 C++。

# FOC移植

- common（数学工具、关键类）：开环用到的正弦、限幅、`LowPassFilter`、`Sensor` 基类都不动。
- communication（SimpleFOC 上位机）：这次不用。
- current_sense（电流采样）：开环速度用不到。
- drivers（三相驱动）：官方按自己的板子写的，PWM 输出要换成 HAL。
- sensors（编码器）：`AS5600_I2C` 里的 I2C 读取要换成 HAL。
- `BLDCMotor.cpp` / `BLDCMotor.h`：开环速度就是 `velocityOpenloop()`，算法没改。只把调试打印换成了 `rt_kprintf`。

```cpp
#include "rtthread.h"
#define SIMPLEFOC_DEBUG(fmt, ...) rt_kprintf(fmt, ##__VA_ARGS__)
```

# 开环速度控制移植

开环时 `motor.controller = MotionControlType::velocity_openloop`。TIM2 每 1 ms 进一次中断，调用 `motor.move(目标速度)`。`move()` 里走 `velocityOpenloop()`，算出 q 轴电压，d 轴为 0，再 `setPhaseVoltage()` 写成三相占空比。电角度是自己累加的，不看编码器。

编码器放在另一个线程里，1 ms 读一次，滤波后得到测量速度，给 `speed` / `vprint` 打印。TIM2 里不读 I2C。

CubeMX 侧先备好外设，后面的代码默认它们已经初始化完：

- TIM1：中心对齐 1，PSC = 0，ARR = 2099。168 MHz 下 PWM 是 \(168\mathrm{MHz}/(2\times2100)=40\mathrm{kHz}\)。
- TIM2：PSC = 168-1，ARR = 499。APB1 定时器时钟 84 MHz，中断 1 kHz。`main` 里延时结束后才 `HAL_TIM_Base_Start_IT`。
- TIM5：32 位自由计数，1 拍 = 1 μs，在 `main` 之前启动。
- I2C1：400 kHz，AS5600 挂在这路上。

RT-Thread Studio 默认不编 `.cpp`。`.project` 加上 C++ 工程属性，`.cproject` 的 C++ 编译器单独加上 `simpleFoc/common` 和 `simpleFoc/common/base_classes`。只加在 C 编译器上，`Sensor.h` 还是找不到。改完要在 Studio 里 Rebuild，否则 `Debug` 里的 makefile 不会收进新的 cpp。有 C++ 静态对象时，链接要能找到 C++ 运行时，`gcc` 链不上就改 `g++` 或加上 `-lstdc++`。

## 时间接口

`time_utils.cpp` 还放在 `simpleFoc/common/`，文件不挪走。库里到处调用的是 `_delay()` 和 `_micros()`。

改前，非 AVR 分支直接调 HAL：

```cpp
#else
  // regular micros
//  delay(ms);
  HAL_Delay(ms);
#endif
```

改后把 `HAL_Delay` 去掉，留下 SimpleFOC 原来的 `delay()`：

```cpp
#else
  // regular micros
  delay(ms);
#endif
```

`_micros()` 这一支本来就是 `return micros();`，没动。

`delay()` 和 `micros()` 是自己加的，放在 `applications/hw_time.c`。`main.h` 里原先有 `#define delay _delay`，会和 `_delay()` 里面的 `delay()` 绕成递归，这个宏删掉，改成函数声明。

```c
unsigned long micros(void)
{
    return TIM5->CNT;
}

void delay(unsigned long ms)
{
    micros_delay((uint32_t)ms * 1000u);
}
```

TIM5 在 `__board_init()` 里启动，早于 `main`。`LowPassFilter` 的构造函数会读 `_micros()`，所以对象要在这之后再构造。

## PWM

改前，三相驱动把 TIM1 写死在类里面。`simpleFoc/drivers/BLDCDriver3PWM.cpp`：

```cpp
#define PWM_TIME htim1

int BLDCDriver3PWM::init() {
  if(!_isset(voltage_limit) || voltage_limit > voltage_power_supply)
    voltage_limit = voltage_power_supply;

  pwm_frequency = __HAL_TIM_GET_AUTORELOAD(&PWM_TIME);
  HAL_TIM_PWM_Start(&PWM_TIME, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&PWM_TIME, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&PWM_TIME, TIM_CHANNEL_3);
  initialized = true;
  return 1;
}

void BLDCDriver3PWM::enable(){
    rt_pin_write(PWM_ENABLE_PIN, PIN_HIGH);
    setPwm(0,0,0);
}

void BLDCDriver3PWM::setPwm(float Ua, float Ub, float Uc) {
  Ua = _constrain(Ua, 0.0f, voltage_limit);
  Ub = _constrain(Ub, 0.0f, voltage_limit);
  Uc = _constrain(Uc, 0.0f, voltage_limit);
  dc_a = _constrain(Ua / voltage_power_supply, 0.0f , 1.0f );
  dc_b = _constrain(Ub / voltage_power_supply, 0.0f , 1.0f );
  dc_c = _constrain(Uc / voltage_power_supply, 0.0f , 1.0f );
  _writeDutyCycle3PWM(&PWM_TIME, dc_a, dc_b, dc_c);
}
```

`_writeDutyCycle3PWM()` 当时放在 `applications/main_Cpp.cpp`，通道也是写死的：

```cpp
void _writeDutyCycle3PWM(TIM_HandleTypeDef *htimx, float dc_a, float dc_b, float dc_c)
{
    uint32_t ccr_a = (uint32_t)(dc_a * __HAL_TIM_GET_AUTORELOAD(htimx));
    uint32_t ccr_b = (uint32_t)(dc_b * __HAL_TIM_GET_AUTORELOAD(htimx));
    uint32_t ccr_c = (uint32_t)(dc_c * __HAL_TIM_GET_AUTORELOAD(htimx));

    __HAL_TIM_SET_COMPARE(htimx, TIM_CHANNEL_1, ccr_a);
    __HAL_TIM_SET_COMPARE(htimx, TIM_CHANNEL_2, ccr_b);
    __HAL_TIM_SET_COMPARE(htimx, TIM_CHANNEL_3, ccr_c);
}
```

这样第二台电机没法再来一个定时器。类里其实已经有 `pwmA`、`pwmB`、`pwmC`，改完之后用引脚号去查表。

新加 `simpleFoc/port/stm32_pwm.h`：

```c
long stm32_pwm_start(int pin_a, int pin_b, int pin_c);
void stm32_pwm_write(int pin_a, int pin_b, int pin_c, float dc_a, float dc_b, float dc_c);
void stm32_pwm_enable(int enable_pin, int enable);
```

`stm32_pwm.cpp` 里的表。引脚号是 RT-Thread 的 `GET_PIN`，和 `board.h` 里的宏是同一套：

```cpp
static const stm32_pwm_pin pwm_map[] = {
    { GET_PIN(A, 8),  &htim1, TIM_CHANNEL_1 },
    { GET_PIN(A, 9),  &htim1, TIM_CHANNEL_2 },
    { GET_PIN(A, 10), &htim1, TIM_CHANNEL_3 },
};
```

`stm32_pwm_start()` 把三相都查一遍。有一相查不到，或者三相不是同一个定时器，就返回 0。查到了就 `HAL_TIM_PWM_Start`，返回自动重装值。`stm32_pwm_write()` 每次写 CCR 之前再查一次，表只有三项。`stm32_pwm_enable()` 就是 `rt_pin_write(enable_pin, ...)`。

`BLDCDriver3PWM.cpp` 改成把成员里的引脚传下去：

```cpp
int BLDCDriver3PWM::init() {
  if(!_isset(voltage_limit) || voltage_limit > voltage_power_supply)
    voltage_limit = voltage_power_supply;

  pwm_frequency = stm32_pwm_start(pwmA, pwmB, pwmC);
  initialized = pwm_frequency > 0;
  return initialized ? 1 : 0;
}

void BLDCDriver3PWM::enable(){
    if (_isset(enableA_pin)) stm32_pwm_enable(enableA_pin, 1);
    setPwm(0,0,0);
}

void BLDCDriver3PWM::disable()
{
  setPwm(0, 0, 0);
  if (_isset(enableA_pin)) stm32_pwm_enable(enableA_pin, 0);
}

void BLDCDriver3PWM::setPwm(float Ua, float Ub, float Uc) {
  /* 限幅、算 dc_a/dc_b/dc_c，和原来一样 */
  stm32_pwm_write(pwmA, pwmB, pwmC, dc_a, dc_b, dc_c);
}
```

`drivers/board.h` 里的宏原来和 TIM1 对不上（A 相写成了 PA7）。改成和 CubeMX 一致，否则查表失败，`init()` 返回 0：

```c
#define PWM_ENABLE_PIN          GET_PIN(A, 6)
#define PWM_A_PIN               GET_PIN(A, 8)
#define PWM_B_PIN               GET_PIN(A, 9)
#define PWM_C_PIN               GET_PIN(A, 10)
```

`_writeDutyCycle3PWM()` 和 `main_Cpp.h` 里的声明一起删掉。

## AS5600

改前，句柄存在对象里，`read()` 直接调 HAL。`AS5600_I2C.h` 的 `init` 带一个 `I2C_HandleTypeDef*`，cpp 里还有一个全局配置：

```cpp
struct AS5600_I2CConfig_s AS5600_I2C_Config = {
  .chip_address   = 0x36,
  .bit_resolution = 12,
  .angle_register = 0x0C,
  .data_start_bit = 11
};

void AS5600_I2C::init(I2C_HandleTypeDef* _hi2c1){
  hi2c1 = _hi2c1;
  this->Sensor::init();
}

int AS5600_I2C::read(uint8_t angle_reg_msb) {
    uint8_t buffer[2];
    HAL_StatusTypeDef status;
    status = HAL_I2C_Mem_Read(hi2c1, chip_address << 1, angle_reg_msb,
                              I2C_MEMADD_SIZE_8BIT, buffer, 2, 50);
    if (status != HAL_OK) {
        return -1;
    }
    return (buffer[0] << 8) | buffer[1];
}
```

调用处是 `AS5600_I2C_1.init(&hi2c1)`，应用层得 `extern hi2c1`。

改后 `init()` 不再收句柄，全局配置也删了，配置由构造 `FocAxis` 的地方传入。`read()` 只负责组 16 位角度：

```cpp
void AS5600_I2C::init(){
  this->Sensor::init();
}

int AS5600_I2C::read(uint8_t angle_reg_msb) {
    uint8_t buffer[2];
    if (stm32_i2c_read_mem(chip_address, angle_reg_msb, buffer, 2) != 0)
    {
        return -1;
    }
    return (buffer[0] << 8) | buffer[1];
}
```

HAL 读寄存器是新加的 `simpleFoc/port/stm32_i2c.cpp`。地址仍是 7 位左移 1 位，超时仍是 50 ms：

```c
int stm32_i2c_read_mem(uint8_t addr_7bit, uint8_t reg, uint8_t *data, uint16_t len)
{
    if (HAL_I2C_Mem_Read(&hi2c1, addr_7bit << 1, reg, I2C_MEMADD_SIZE_8BIT,
                         data, len, 50) != HAL_OK)
    {
        return -1;
    }
    return 0;
}
```

`getSensorAngle()` 还是 `原始值 / cpr * 2π`。`Sensor::update()` 没改。两次采样如果慢到跨过大约 0.8 圈，圈数会计错，所以采样线程用 1 ms，不要放进 500 ms 的打印循环。

## 电机对象

改前，传感器、驱动、电机、目标速度全是 `main_Cpp.cpp` 里的全局变量，中断也直接抓那个全局 `motor`：

```cpp
extern struct AS5600_I2CConfig_s AS5600_I2C_Config;
extern I2C_HandleTypeDef hi2c1;
extern TIM_HandleTypeDef htim2;

AS5600_I2C AS5600_I2C_1(AS5600_I2C_Config);
BLDCDriver3PWM motor_driver(PWM_A_PIN, PWM_B_PIN, PWM_C_PIN);
BLDCMotor motor(7);
static volatile float target_velocity = 20.0f;

void main_Cpp(void) {
    AS5600_I2C_1.init(&hi2c1);
    AS5600_I2C_1.min_elapsed_time = 0.001f;

    motor_driver.voltage_power_supply = 12;
    motor_driver.voltage_limit = 10;
    motor_driver.init();

    motor.linkDriver(&motor_driver);
    motor.voltage_limit = 5;
    motor.controller = MotionControlType::velocity_openloop;
    motor.init();

    rt_pin_write(PWM_ENABLE_PIN, PIN_HIGH);
    rt_thread_mdelay(1000);
    HAL_TIM_Base_Start_IT(&htim2);
    /* 接着在这个函数里创建 sensor 线程，自己死循环打印 */
}

extern "C" void foc_timer_isr(void)
{
    motor.move(target_velocity);
}
```

改后这些成员进了 `FocAxis`，文件是新加的 `simpleFoc/port/foc_port.cpp`。`main_Cpp.cpp` 和 `main_Cpp.h` 删除。一台电机自己带传感器、驱动、`BLDCMotor`、滤波器和两个速度：

```cpp
class FocAxis
{
public:
    FocAxis(const AS5600_I2CConfig_s &sensor_config, int pole_pairs,
            int pin_a, int pin_b, int pin_c, int pin_en)
        : sensor(sensor_config),
          driver(pin_a, pin_b, pin_c, pin_en),
          motor(pole_pairs),
          lpf(0.05f),
          target_velocity(20.0f),
          measured_velocity(0.0f) {}

    void init()
    {
        sensor.init();
        sensor.min_elapsed_time = 0.001f;
        driver.voltage_power_supply = 12;
        driver.voltage_limit = 10;
        driver.init();
        motor.linkDriver(&driver);
        motor.voltage_limit = 5;
        motor.controller = MotionControlType::velocity_openloop;
        motor.init();
        driver.enable();
    }

    void poll()
    {
        sensor.update();
        measured_velocity = lpf(sensor.getVelocity());
    }

    void isr() { motor.move(target_velocity); }

private:
    AS5600_I2C sensor;
    BLDCDriver3PWM driver;
    BLDCMotor motor;
    LowPassFilter lpf;
    volatile float target_velocity;
    volatile float measured_velocity;
};

AxisGroup &axes()
{
    static FocAxis motor0(as5600_config, 7, PWM_A_PIN, PWM_B_PIN, PWM_C_PIN, PWM_ENABLE_PIN);
    static AxisGroup group = { { &motor0 }, 1 };
    return group;
}
```

`as5600_config` 就是原来的 `{0x36, 12, 0x0C, 11}`，跟着对象走，不再做全局变量。`foc_sensor_poll()` 和 `foc_timer_isr()` 按 `group.count` 往下调用，列表里有几台就跑几台。速度命令要带编号，现在只有 0 号。

第二台就是再写一个 `FocAxis motor1(...)`，放进 `group.list`，`count` 改成 2。引脚和传感器地址用电机自己的。I2C 读函数目前还是固定 `hi2c1`，两台编码器得地址不同，或者以后把总线也放进对象。

给 C 文件用的声明在新加的 `simpleFoc/port/foc_api.h`：

```c
void foc_init(void);
void foc_sensor_poll(void);
void foc_timer_isr(void);
float foc_get_velocity(int axis);
void foc_set_target_velocity(int axis, float rad_s);
float foc_get_target_velocity(int axis);
```

## 线程和 main

改前 `applications/main.c` 几乎是空的，进 `main_Cpp()` 就再也不返回。TIM2 中断和微秒计数也堆在这个文件里：

```c
int main(void)
{
    main_Cpp();
    return RT_EOK;
}

void TIM2_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&htim2);
  foc_timer_isr();
}
```

改后 `main` 只排顺序。`BLDCMotor::init()` 里面已经有两段 ` _delay(500)`，返回之后再等 1 秒才打开 TIM2：

```c
int main(void)
{
    foc_init();
    rt_thread_mdelay(1000);
    board_foc_timer_start();
    app_sensor_start();

    while (1)
    {
        app_log_poll();
        rt_thread_mdelay(500);
    }
    return RT_EOK;
}
```

TIM5 初始化和 TIM2 中断挪到 `applications/hw_board.c`。中断函数还是先 HAL，再 `foc_timer_isr()`。`cubemx/Src/stm32f4xx_it.c` 里也有一个 `TIM2_IRQHandler`，Studio 工程目前没编那个文件。加回去会重复定义。

1 ms 采样从 `main_Cpp` 的内部线程拆到 `applications/app_sensor.c`，优先级 5，循环里只有 `foc_sensor_poll()` 和 `rt_thread_mdelay(1)`。`speed`、`vprint` 拆到 `applications/app_shell.c`，设置速度时写成 `foc_set_target_velocity(0, rad_s)`。

采样线程比 `main`（优先级 10）和 FinSH（20）高。I2C 读成功大约一百到两百微秒；失败时 HAL 会在这个线程里等到超时（50 ms），这段时间命令行打不出字。不要在 TIM2 里 `rt_kprintf`，控制台发送是轮询的，中断里会把 1 kHz 的节拍拖死。
