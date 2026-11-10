---
title: SimpleFOC 移植
layout: page
---

# 硬件选型
- 电机：2804电机， 7级对，额定电压12V(7.4~16V DC), 绕线电阻5.1Ω，相电阻2.3Ω，KV值220，相电感0.86mH。**此部分移植需配置**
- 编码器：AS5600磁编码器，IIC协议。**IIC读写相关函数需要一直移植**
- 驱动板：MKS SimpleFOC Shield V3.2