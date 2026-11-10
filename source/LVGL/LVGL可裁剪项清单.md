---
title: LVGL 可裁剪项清单
layout: page
---

# LVGL 可裁剪项清单

> 基于 LVGL v8.2.0，配置于 `lv_rt_thread_conf.h` 或 `lv_conf_internal.h` 默认值

---

## 一、颜色与显示

| 配置项 | 默认值 | 裁剪说明 |
|--------|--------|----------|
| `LV_COLOR_DEPTH` | 16 | 1/8/16/32，越小 Flash 越少，16 常用 |
| `LV_COLOR_16_SWAP` | 0 | RGB565 字节序，按 LCD 调整 |
| `LV_COLOR_SCREEN_TRANSP` | 0 | 屏幕透明，需 32 位色 |
| `LV_DISP_ROT_MAX_BUF` | 10×1024 | 软件旋转缓冲，不用可减小 |

---

## 二、内存

| 配置项 | 默认值 | 裁剪说明 |
|--------|--------|----------|
| `LV_MEM_CUSTOM` | 0 | 1=用 rt_malloc，0=内置池 |
| `LV_MEM_SIZE` | 48KB | 内置池大小，按需减小 |
| `LV_MEM_BUF_MAX_NUM` | 16 | 渲染缓冲数量，可减至 8 |
| `LV_MEMCPY_MEMSET_STD` | 0 | 1=用标准 memcpy/memset |

---

## 三、绘制引擎

| 配置项 | 默认值 | 裁剪说明 |
|--------|--------|----------|
| `LV_DRAW_COMPLEX` | 1 | 0=禁用阴影/渐变/圆角/弧等，**大幅减 Flash** |
| `LV_SHADOW_CACHE_SIZE` | 0 | 阴影缓存，0 不缓存 |
| `LV_CIRCLE_CACHE_SIZE` | 4 | 圆/弧缓存，0 禁用 |
| `LV_IMG_CACHE_DEF_SIZE` | 0 | 图片解码缓存 |
| `LV_GRADIENT_MAX_STOPS` | 2 | 渐变色阶数 |
| `LV_GRAD_CACHE_DEF_SIZE` | 0 | 渐变缓存 |
| `LV_DITHER_GRADIENT` | 0 | 渐变抖动 |

---

## 四、GPU（硬件加速）

| 配置项 | 默认值 | 裁剪说明 |
|--------|--------|----------|
| `LV_USE_GPU_STM32_DMA2D` | 0 | STM32 DMA2D |
| `LV_USE_GPU_NXP_PXP` | 0 | NXP PXP |
| `LV_USE_GPU_NXP_VG_LITE` | 0 | NXP VG-Lite |
| `LV_USE_GPU_SDL` | 0 | SDL 渲染 |

---

## 五、调试与监视

| 配置项 | 默认值 | 裁剪说明 |
|--------|--------|----------|
| `LV_USE_LOG` | 0 | 日志，发布可关 |
| `LV_USE_PERF_MONITOR` | 0 | FPS/CPU 监视器 |
| `LV_USE_MEM_MONITOR` | 0 | 内存监视器 |
| `LV_USE_REFR_DEBUG` | 0 | 刷新调试 |
| `LV_USE_ASSERT_NULL` | 1 | NULL 检查 |
| `LV_USE_ASSERT_MALLOC` | 1 | 分配检查 |
| `LV_USE_ASSERT_STYLE` | 0 | 样式检查 |
| `LV_USE_ASSERT_MEM_INTEGRITY` | 0 | 内存完整性 |
| `LV_USE_ASSERT_OBJ` | 0 | 对象检查 |

---

## 六、字体（Flash 占用大）

| 配置项 | 默认值 | 裁剪说明 |
|--------|--------|----------|
| `LV_FONT_MONTSERRAT_8` | 0 | 每字号约 2~50KB |
| `LV_FONT_MONTSERRAT_10` | 0 | |
| `LV_FONT_MONTSERRAT_12` | 0 | |
| `LV_FONT_MONTSERRAT_14` | 1 | 常用 |
| `LV_FONT_MONTSERRAT_16` | 0 | |
| `LV_FONT_MONTSERRAT_18` | 0 | |
| `LV_FONT_MONTSERRAT_20` | 0 | |
| `LV_FONT_MONTSERRAT_22` | 0 | |
| `LV_FONT_MONTSERRAT_24` | 1 | 常用 |
| `LV_FONT_MONTSERRAT_26` ~ `48` | 0 | 按需开启 |
| `LV_FONT_MONTSERRAT_12_SUBPX` | 0 | 亚像素 |
| `LV_FONT_MONTSERRAT_28_COMPRESSED` | 0 | 压缩字体 |
| `LV_FONT_DEJAVU_16_PERSIAN_HEBREW` | 0 | 波斯/希伯来 |
| `LV_FONT_SIMSUN_16_CJK` | 0 | 宋体中文 |
| `LV_FONT_UNSCII_8` | 0 | 等宽 8px |
| `LV_FONT_UNSCII_16` | 0 | 等宽 16px |
| `LV_USE_FONT_COMPRESSED` | 0 | 压缩字体支持（Benchmark 需 1） |
| `LV_USE_FONT_SUBPX` | 0 | 亚像素渲染 |
| `LV_FONT_FMT_TXT_LARGE` | 0 | 大字体格式 |

---

## 七、文本与语言

| 配置项 | 默认值 | 裁剪说明 |
|--------|--------|----------|
| `LV_USE_BIDI` | 0 | 双向文本 |
| `LV_USE_ARABIC_PERSIAN_CHARS` | 0 | 阿拉伯/波斯字符 |
| `LV_TXT_ENC` | UTF-8 | 编码 |
| `LV_TXT_LINE_BREAK_LONG_LEN` | 0 | 长文本换行 |

---

## 八、基础控件（Widgets）

| 配置项 | 默认值 | 裁剪说明 |
|--------|--------|----------|
| `LV_USE_ARC` | 1 | 弧形 |
| `LV_USE_ANIMIMG` | 1 | 动图 |
| `LV_USE_BAR` | 1 | 进度条 |
| `LV_USE_BTN` | 1 | 按钮 |
| `LV_USE_BTNMATRIX` | 1 | 按钮矩阵 |
| `LV_USE_CANVAS` | 1 | 画布 |
| `LV_USE_CHECKBOX` | 1 | 复选框 |
| `LV_USE_DROPDOWN` | 1 | 下拉框 |
| `LV_USE_IMG` | 1 | 图片 |
| `LV_USE_LABEL` | 1 | 标签（**多数控件依赖，慎关**） |
| `LV_USE_LINE` | 1 | 线条 |
| `LV_USE_ROLLER` | 1 | 滚轮 |
| `LV_USE_SLIDER` | 1 | 滑块 |
| `LV_USE_SWITCH` | 1 | 开关 |
| `LV_USE_TEXTAREA` | 1 | 文本框 |
| `LV_USE_TABLE` | 1 | 表格 |

---

## 九、扩展控件（Extra Widgets）

| 配置项 | 默认值 | 裁剪说明 |
|--------|--------|----------|
| `LV_USE_CALENDAR` | 1 | 日历 |
| `LV_USE_CHART` | 1 | 图表 |
| `LV_USE_COLORWHEEL` | 1 | 色轮 |
| `LV_USE_IMGBTN` | 1 | 图片按钮 |
| `LV_USE_KEYBOARD` | 1 | 键盘 |
| `LV_USE_LED` | 1 | LED |
| `LV_USE_LIST` | 1 | 列表 |
| `LV_USE_MENU` | 1 | 菜单 |
| `LV_USE_METER` | 1 | 仪表 |
| `LV_USE_MSGBOX` | 1 | 消息框 |
| `LV_USE_SPINBOX` | 1 | 数字框 |
| `LV_USE_SPINNER` | 1 | 旋转加载 |
| `LV_USE_TABVIEW` | 1 | 标签页 |
| `LV_USE_TILEVIEW` | 1 | 瓦片视图 |
| `LV_USE_WIN` | 1 | 窗口 |
| `LV_USE_SPAN` | 1 | 富文本 |

---

## 十、主题（Themes）

| 配置项 | 默认值 | 裁剪说明 |
|--------|--------|----------|
| `LV_USE_THEME_DEFAULT` | 1 | 默认主题 |
| `LV_USE_THEME_BASIC` | 1 | 基础主题 |
| `LV_USE_THEME_MONO` | 1 | 单色主题 |

---

## 十一、布局（Layouts）

| 配置项 | 默认值 | 裁剪说明 |
|--------|--------|----------|
| `LV_USE_FLEX` | 1 | Flex 布局 |
| `LV_USE_GRID` | 1 | Grid 布局 |

---

## 十二、文件系统

| 配置项 | 默认值 | 裁剪说明 |
|--------|--------|----------|
| `LV_USE_FS_STDIO` | 0 | 标准 C 文件 API |
| `LV_USE_FS_POSIX` | 0 | POSIX |
| `LV_USE_FS_WIN32` | 0 | Windows |
| `LV_USE_FS_FATFS` | 0 | FATFS |

---

## 十三、图片解码

| 配置项 | 默认值 | 裁剪说明 |
|--------|--------|----------|
| `LV_USE_PNG` | 0 | PNG |
| `LV_USE_BMP` | 0 | BMP |
| `LV_USE_SJPG` | 0 | 硬件 JPEG |
| `LV_USE_GIF` | 0 | GIF |
| `LV_USE_QRCODE` | 0 | 二维码 |
| `LV_USE_FREETYPE` | 0 | FreeType |
| `LV_USE_RLOTTIE` | 0 | Lottie 动画 |
| `LV_USE_FFMPEG` | 0 | FFmpeg |

---

## 十四、其他功能

| 配置项 | 默认值 | 裁剪说明 |
|--------|--------|----------|
| `LV_USE_SNAPSHOT` | 0 | 屏幕截图 |
| `LV_USE_MONKEY` | 0 | 随机测试 |
| `LV_USE_GRIDNAV` | 0 | 网格导航 |
| `LV_USE_USER_DATA` | 0 | 用户数据 |

---

## 十五、Demo

| 配置项 | 默认值 | 裁剪说明 |
|--------|--------|----------|
| `LV_USE_DEMO_WIDGETS` | 0 | Widgets Demo |
| `LV_USE_DEMO_KEYPAD_AND_ENCODER` | 0 | 按键/编码器 Demo |
| `LV_USE_DEMO_BENCHMARK` | 0 | 性能 Demo |
| `LV_USE_DEMO_STRESS` | 0 | 压力测试 |
| `LV_USE_DEMO_MUSIC` | 0 | 音乐 Demo |
| `LV_BUILD_EXAMPLES` | 0 | 示例 |

---

## 裁剪优先级建议（64KB RAM / 512KB Flash）

| 优先级 | 项目 | 建议 |
|--------|------|------|
| 高 | 字体 | 只保留 14/24，其余关 |
| 高 | Demo | 仅保留需要的 1 个 |
| 高 | Demo 依赖 | Benchmark 需 `LV_USE_FONT_COMPRESSED 1` |
| 中 | 扩展控件 | 关未用：Calendar、Chart、Colorwheel 等 |
| 中 | 主题 | 保留 1 个，关其余 |
| 中 | 布局 | 未用 Flex/Grid 可关 |
| 中 | `LV_DRAW_COMPLEX` | 无阴影/渐变可关，减 Flash 明显 |
| 低 | 断言 | 发布可关 `LV_USE_ASSERT_*` |
| 低 | 日志 | 发布关 `LV_USE_LOG` |

---

*配置项在 `lv_rt_thread_conf.h` 中定义会覆盖 `lv_conf_internal.h` 默认值。*
