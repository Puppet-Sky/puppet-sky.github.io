---
title: LVGL 移植总结
layout: page
---

# LVGL 移植总结文档

> 基于 RT-Thread + STM32F103xE + ILI9341 LCD + LVGL v8.2.0

---

## 一、移植步骤

### 1. 环境准备

| 项目 | 说明 |
|------|------|
| 硬件 | STM32F103xE（64KB RAM, 512KB Flash） |
| 显示屏 | ILI9341 TFT LCD（320×240，FSMC 16 位接口） |
| RTOS | RT-Thread v4.1.0 |
| LVGL | v8.2.0 |

### 2. 添加 LVGL 软件包

在 RT-Thread 工程中通过 **RT-Thread Settings** 或 **menuconfig** 启用 LVGL 软件包：

- `CONFIG_PKG_USING_LVGL=y`
- 选择版本：`CONFIG_PKG_LVGL_USING_V08020=y`

### 3. 配置 LVGL 参数

RT-Thread 下 LVGL 使用 `lv_rt_thread_conf.h` 作为配置文件（通过 `lv_conf_kconfig.h` 在 `__RTTHREAD__` 下自动包含），**不使用** 根目录的 `lv_conf.h`。

配置文件路径：`packages/LVGL-v8.2.0/env_support/rt-thread/lv_rt_thread_conf.h`

### 4. 实现显示驱动移植层

创建 `lv_port_disp.c` / `lv_port_disp.h`，或从LVGL官方github库中**exmples/porting/**`lv_port_xxx_template.c` / `lv_port_xxx_template.h`中复制修改，完成以下工作：
> lv_port_disp_template.c: 用于移植底层显示驱动以及显示设备初始化，需要根据实际硬件进行修改

1. **初始化 LCD 硬件**：在 `disp_init()` 中调用底层 LCD 初始化（如 `ILI9341_Init()`）
2. **分配显示缓冲区**：为 LVGL 分配绘制缓冲区（单缓冲 / 双缓冲 / 全屏双缓冲）
3. **实现 flush 回调**：在 `disp_flush()` 中将 LVGL 渲染的像素数据写入 LCD
4. **注册显示驱动**：`lv_disp_drv_register(&disp_drv)`

> lv_port_indev_template.c: 用于移植底层输入设备驱动以及输入设备(触摸屏、鼠标、键盘器等)初始化，需要根据实际硬件进行修改

> lv_port_fs_template.c: 用于移植底层文件系统驱动以及文件系统初始化，需要根据实际硬件进行修改

### 5. 集成 RT-Thread 端口

RT-Thread 官方 LVGL 包提供 `lv_rt_thread_port.c`，通过 `INIT_COMPONENT_EXPORT(lv_port_init)` 在系统启动时自动执行：

- `lv_init()`
- `lv_port_disp_init()`（显示驱动）
- （可选）`lv_port_indev_init()`（输入设备）

### 6. 应用层调用

在 `main.c` 中创建 LVGL 任务线程，周期性调用 `lv_task_handler()`（LVGL v8 中为 `lv_timer_handler()` 的兼容封装），并调用 Demo 或自定义界面。

---

## 二、关键参数配置

### 2.1 内存配置（`lv_rt_thread_conf.h`）

| 配置项 | 推荐值 | 说明 |
|--------|--------|------|
| `LV_MEM_CUSTOM` | 0 | 使用 LVGL 内置内存池，避免与 RT-Thread 堆竞争 |
| `LV_MEM_SIZE` | 24×1024 (24KB) | LVGL 专用内存池大小，64KB RAM 下建议 20~24KB |

**注意**：若使用 `LV_MEM_CUSTOM 1` 并接入 `rt_malloc`，在 64KB RAM 下 RT-Thread 堆仅约 3.8KB，Widgets Demo 等会因 `rt_realloc` 失败触发断言。

### 2.2 日志配置（`lv_rt_thread_conf.h`）

| 配置项 | 说明 |
|--------|------|
| `LVGL_LOG_ENABLE` | 取消注释 `#define LVGL_LOG_ENABLE 1` 以启用 LVGL 日志 |
| `LV_LOG_LEVEL` | 日志级别：0(TRACE) 1(INFO) 2(WARN) 3(ERROR) 4(USER) 5(NONE) |

启用后日志通过 RT-Thread `LOG_I()` 输出到控制台。

### 2.2.1 FPS 监视器

| 配置项 | 说明 |
|--------|------|
| `LV_USE_PERF_MONITOR` | 设为 1 启用，在屏幕角落显示 FPS 和 CPU 占用 |
| `LV_USE_PERF_MONITOR_POS` | 位置：`LV_ALIGN_BOTTOM_RIGHT`（右下）、`LV_ALIGN_TOP_LEFT`（左上）等 |

### 2.3 时钟与系统时间

| 配置项 | 值 | 说明 |
|--------|-----|------|
| `LV_TICK_CUSTOM` | 1 | 使用 RT-Thread 系统时钟 |
| `LV_TICK_CUSTOM_SYS_TIME_EXPR` | `rt_tick_get_millisecond()` | 毫秒级时间源 |

### 2.4 显示参数（`lv_port_disp.c`）

| 参数 | 值 | 说明 |
|------|-----|------|
| `MY_DISP_HOR_RES` | 320 | 水平分辨率 |
| `MY_DISP_VER_RES` | 240 | 垂直分辨率 |
| 缓冲区大小 | `320×24` 像素 | 单缓冲，24 行，约 15KB |

### 2.5 RT-Thread 线程配置（`.config`）

| 配置项 | 值 | 说明 |
|--------|-----|------|
| `PKG_LVGL_THREAD_PRIO` | 20 | LVGL 线程优先级 |
| `PKG_LVGL_THREAD_STACK_SIZE` | 4096 | LVGL 线程栈大小 |
| `PKG_LVGL_DISP_REFR_PERIOD` | 5 | 刷新周期（ms） |

### 2.6 字体与 Demo 裁剪

> 完整可裁剪项见 [LVGL可裁剪项清单.md](LVGL可裁剪项清单.html)

| 配置项 | 说明 |
|--------|------|
| `LV_FONT_MONTSERRAT_14` | 常用小字号 |
| `LV_FONT_MONTSERRAT_24` | 常用大字号 |
| `LV_USE_DEMO_WIDGETS` | Widgets Demo（内存需求大） |
| `LV_USE_DEMO_BENCHMARK` | Benchmark Demo（需 `LV_USE_FONT_COMPRESSED 1`） |
| `LV_USE_DEMO_KEYPAD_AND_ENCODER` | 按键/编码器 Demo（轻量） |

---

## 三、移植过程中常见问题

### 3.1 assertion failed at lv_table_set_row_cnt:228

**现象**：运行 Widgets Demo 时断言失败。

**原因**：`rt_realloc()` 返回 NULL，RT-Thread 堆内存不足。64KB RAM 下堆仅约 3.8KB，table 等控件动态分配失败。

**解决**：
- 将 `LV_MEM_CUSTOM` 设为 0，使用 LVGL 内置内存池
- 设置 `LV_MEM_SIZE` 为 20~24KB，为 LVGL 预留专用内存

### 3.2 链接错误：RAM 溢出

**现象**：链接阶段报错，提示 `.bss` 或 RAM 超出限制。

**原因**：`LV_MEM_SIZE` 过大，或启用多个 Demo 导致 BSS 超出 64KB。

**解决**：
- 减小 `LV_MEM_SIZE`（如 20KB）
- 仅启用一个 Demo，关闭不需要的字体和控件

### 3.3 屏幕花屏、颜色异常

**现象**：显示错乱或颜色不对。

**原因**：
- RGB565 字节序与 LCD 不一致（`LV_COLOR_16_SWAP`）
- `disp_flush` 中写入方式与 LCD 扫描方向不匹配

**解决**：
- 检查 `LV_COLOR_16_SWAP` 是否与硬件一致
- 确认 `LCD_SCAN_MODE` 与 `disp_flush` 的坐标、像素顺序一致

### 3.4 刷新卡顿、界面不流畅

**现象**：界面刷新慢、有明显卡顿。

**原因**：
- `disp_flush` 使用 CPU 逐像素写入，未使用 DMA
- 缓冲区过小，导致多次 flush
- `lv_task_handler` 调用周期过长

**解决**：
- 使用 DMA 传输像素数据（若硬件支持）
- 适当增大缓冲区（如 40 行）
- 将 `PKG_LVGL_DISP_REFR_PERIOD` 调小（如 5ms）

### 3.5 配置文件不生效

**现象**：修改 `lv_conf.h` 后无效果。

**原因**：RT-Thread 下使用 `lv_rt_thread_conf.h`，通过 `lv_conf_kconfig.h` 在 `__RTTHREAD__` 时包含，不会使用根目录的 `lv_conf.h`。

**解决**：直接修改 `packages/LVGL-v8.2.0/env_support/rt-thread/lv_rt_thread_conf.h`。

### 3.6 Benchmark Demo 编译失败

**现象**：启用 Benchmark 后编译报错，缺少字体或资源。

**原因**：Benchmark 依赖压缩字体和图片资源。

**解决**：在 `lv_rt_thread_conf.h` 中设置：
- `LV_USE_DEMO_BENCHMARK 1`
- `LV_USE_FONT_COMPRESSED 1`

---

## 四、文件结构速查

| 文件 | 作用 |
|------|------|
| `lv_rt_thread_conf.h` | LVGL 配置（内存、时钟、字体、Demo） |
| `lv_rt_thread_port.c` | RT-Thread 端口，自动初始化 |
| `lv_port_disp.c` | 显示驱动移植（flush、缓冲区） |
| `applications/FSMC/bsp_ili9341_lcd.c` | ILI9341 底层驱动 |
| `main.c` | 应用入口，LVGL 线程与 Demo 调用 |

---

## 五、推荐配置组合（64KB RAM）

| 场景 | LV_MEM_SIZE | Demo | 说明 |
|------|-------------|------|------|
| 轻量 UI | 16KB | Keypad_Encoder | 按键/编码器 Demo，最省内存 |
| 常规 UI | 24KB | 自定义 / 简单 Widgets | 平衡性能与功能 |
| 完整 Demo | 32KB+ | Widgets | 需更大 RAM 或外扩 SRAM |

---

*文档基于当前工程实际配置整理，如有更新请以最新代码为准。*
