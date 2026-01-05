---
title: boot
date: 2026-01-04 13:55:04
tags: [uboot, linux]
---

# 关键定义说明
- __image_copy_start： 地址(0X87800000)，在boot.lds文件中定义
- _start: 地址(0X87800000)
- CONFIG_SYS_INIT_RAM_ADDR ：0X00900000。芯片内部OCRAM地址
- CONFIG_SYS_INIT_RAM_SIZE ：0X00040000。芯片内部OCRAM大小
- CONFIG_SYS_INIT_SP_ADDR：0X0091FF00。初始化的栈地址
<!-- more -->
# Uboot前期启动流程（前期启动是指在DDR初始化之前，完成CPU、内存等硬件的初始化）

## boot.lds 编译连接文件
- ENTRY(_start) 为程序入口点，**_start**为汇编函数，定义在 ++*arch/arm/lib/vectors.S*++。下一步切入 <a href="#reset">reset</a>

```arm
#ifdef CONFIG_SYS_DV_NOR_BOOT_CFG
	.word	CONFIG_SYS_DV_NOR_BOOT_CFG
#endif

	b	reset
	ldr	pc, _undefined_instruction
	ldr	pc, _software_interrupt
	ldr	pc, _prefetch_abort
	ldr	pc, _data_abort
	ldr	pc, _not_used
	ldr	pc, _irq
	ldr	pc, _fiq
```

##  arch/arm/cpu/armv7/start.S启动入口汇编程序
- <a id="reset">.globl  reset </a>，下一步跳转到<a href="#save_boot_params">save_boot_params</a>

```arm
reset:
	/* Allow the board to save important registers */
	b	save_boot_params
```
- <a id="save_boot_params">ENTRY(save_boot_params)</a> 下一步跳转到<a href="#save_boot_params_ret">save_boot_params_ret</a>

```arm
ENTRY(save_boot_params)
	b	save_boot_params_ret	@ back to my caller
```

- <a id="save_boot_params_ret">save_boot_params_ret</a> 禁止中断，重定位向量表，下一步分别调用**cpu_init_crit**和 <a href="#_main">**_main**</a> 函数。而**cpu_init_crit**又是只调用了<a href="#lowlevel_init">**lowlevel_init**</a>

```arm
save_boot_params_ret:
	/* 禁用中断（快速中断请求与普通中断请求），
	 * 同时将 CPU 切换至 32 位超级用户模式；
	 * 重定位向量表,
	 * 若已处于超级监控模式，则无需执行上述操作*/
	mrs	r0, cpsr
	and	r1, r0, #0x1f		@ mask mode bits
	teq	r1, #0x1a		@ test for HYP mode
	bicne	r0, r0, #0x1f		@ clear all mode bits
	orrne	r0, r0, #0x13		@ set SVC mode
	orr	r0, r0, #0xc0		@ disable FIQ and IRQ
	msr	cpsr,r0
	/* 重定位向量表 至 _start 地址 0x87800000*/
#if !(defined(CONFIG_OMAP44XX) && defined(CONFIG_SPL_BUILD))
	mrc	p15, 0, r0, c1, c0, 0	@ Read CP15 SCTLR Register
	bic	r0, #CR_V		@ V = 0
	mcr	p15, 0, r0, c1, c0, 0	@ Write CP15 SCTLR Register

	/* 重定位向量表 至 _start 地址 0x87800000*/
	ldr	r0, =_start
	mcr	p15, 0, r0, c12, c0, 0	@Set VBAR
#endif

	/* 在跳转之前，应确保 PLL 和其他稳定 */
#ifndef CONFIG_SKIP_LOWLEVEL_INIT
	bl	cpu_init_cp15
	bl	cpu_init_crit
#endif

	bl	_main
```

## arch/arm/cpu/armv7/lowlevel_init.S 底层初始化
- <a id="lowlevel_init">lowlevel_init</a>
lowlevel_init函数的主要功能是完成早期的底层初始化，其关键步骤如下：
1. 设置一个临时的栈空间（sp），以便后续可以安全调用C函数完成进一步初始化。此时全局数据（global data）还不可用，所以只能用临时栈。
2. 根据不同的配置初始化全局数据指针r9（如SPL场景下 r9=gdata，普通场景下分配GD_SIZE空间给r9）。
3. 保存返回地址（ip, lr）到栈，为后续流程恢复做准备。
4. 调用<a href="#s_init">s_init</a>函数做最小化的早期初始化，这里不会初始化DRAM、不会使用全局数据、不会清除BSS段，也不会启动控制台。
5. 恢复返回地址，返回上层调用。

简而言之，lowlevel_init为CPU早期启动创建必要的运行环境（如栈和部分必需指针），并留给平台/板级代码以最小代价带起后续的初始化流程。

## arch/arm/cpu/armv7/mx6/soc.c 
- <a id="s_init">s_init</a>
当前SOC（MX6ULL）不支持s_init函数，所以直接返回。

```c
void s_init(void)
{
	......
	if (is_cpu_type(MXC_CPU_MX6SX) || is_cpu_type(MXC_CPU_MX6UL) ||
	is_cpu_type(MXC_CPU_MX6ULL) || is_cpu_type(MXC_CPU_MX6SLL))
	return;
}
```

## 前期启动函数调用路径图
![前期启动函数调用路径图](./image/save_boot_params_ret.png)

# _main阶段启动流程

## arch/arm/lib/crt0.S _main阶段启动入口汇编程序
- <a id="_main">_main</a>

_main 函数是 U-Boot 启动流程中的 C 运行环境入口，其主要功能如下：
1. 设置最初的栈环境（sp），调用***board_init_f_alloc_reserve***设置malloc内存分配区域（1KB）和初始化全局数据(gd)所需的空间（248B）。
2. 调用***board_init_f_init_reserve***初始化全局数据(gd), 清0。
3. 调用 <a href="#board_init_f">board_init_f(0)</a> 完成硬件初始化，如内存初始化。
4. 初始化完DDR后，并将代码重定位到DDR中，实现从OCRAM到DDR的代码迁移,调用<a href="#relocate_code">relocate_code</a>将代码拷贝到DDR中。
5. 调用函数 <a href="#relocate_vectors">relocate_vectors</a>，对中断向量表做重定位。
6. 调用函数<a href="#board_init_r">board_init_r</a>, 继续对硬件进行初始化


 ```arm
ENTRY(_main)
    // 1. 设置初始的栈指针（sp）
    sp = CONFIG_SYS_INIT_SP_ADDR;     // 或 CONFIG_SPL_STACK（SPL场景下）
    sp = sp & ~0x7;                   // 8字节对齐

    // 2. 为全局数据分配空间，并初始化
    r0 = sp;
    r0 = board_init_f_alloc_reserve(r0);
    sp = r0;
    r9 = r0;                          // r9 作为 gd 指针
    board_init_f_init_reserve();

    // 3. 调用板级初始化，完成内存等早期环境配置
    board_init_f(0);

    // 4. （非SPL场景）中间阶段设置新sp与gd，并进行代码重定位
	adr	lr, here
	ldr	r0, [r9, #GD_RELOC_OFF]		/* r0 = gd->reloc_off */
	add	lr, lr, r0
#if defined(CONFIG_CPU_V7M)
	orr	lr, #1				/* As required by Thumb-only */
#endif
	ldr	r0, [r9, #GD_RELOCADDR]		/* r0 = gd->relocaddr */
	b	relocate_code
here:
	bl	relocate_vectors

	bl	c_runtime_cpu_setup	/* we still call old routine here */

    // 5. 后续进入board_init_r ...
```

## common/board_f.c 
- <a id="board_init_f">board_init_f</a>
	> 初始化 DDR，定时器，完成代码拷贝等等，通过函数 initcall_run_list 来运行初始化序列 init_sequence_f 里面的一些列函数，init_sequence_f 里面包含了一系列的初始化函数。

	- setup_mon_len：设置gd.mon_len，Uboot核心代码长度(__bss_end -_start)
	- initf_malloc: 初始化 gd 中跟 malloc 有关的成员变量
	- env_init：设置 gd 的成员变量 env_addr，也就是环境变量的保存地址。
	- reserve_mmu：留出 MMU 的 TLB 表的位置，16KB，但是做了64KB对齐，实际占用64KB。此时
	- reserve_uboot：预留出uboot代码拷贝的区域，大小为gd.mon_len，也就是Uboot核心代码长度(__bss_end -_start), 并进行4KB对齐
	- reserve_malloc：预留出malloc的区域，大小为0x1002000, 16MB+8KB
	- reserve_board：留出板子 bd 所占的内存区，bd 是结构体 bd_t，bd_t 大小为80 字节
	- reserve_global_data：保留出 gd_t 的内存区域，gd_t 结构体大小为 248B


> 初始化后DRAM的地址为0x80000000,DRAM大小为512MB。SP指针为0X9EF44E90 
![uboot内存分配图](./image/uboot_RAM.png)

## arch/arm/lib/relocate.S 代码重定位
- <a id="relocate_code">relocate_code</a>
	> 代码移动到DDR中后，需要对代码中绝对地址进行重定位，将地址加上偏移地址指向DDR中的新地址。在编译过程中，程序生成了一张“变量地址修正表”（.rel.dyn 段）。

- <a id="relocate_vectors">relocate_vectors</a>
	> 重定位向量表,将新的中断向量表位置写入VBAR寄存器中

## common/board_r.c
- <a id="board_init_r">board_init_r</a>
	> 继续对硬件进行初始化，通过init_sequence_r初始化序列来完成包括时钟初始化、串口初始化、网卡初始化等。初始化完成后，就会跳转到linux系统。
	- board_init 板级初始化，初始化外围芯片、IIC、USB等。执行的是mx6ull_alientek_emmc.c 文件中的board_init函数。
	- initr_env 初始化环境变量，读取环境变量文件，并设置环境变量。
	- run_main_loop</a> 运行主循环，调用<a href="#main_loop">main_loop</a>函数，等待用户输入命令。main_loop中为死循环，如果linux设置正常，则会自动启动linux系统。

## common/main.c
- <a id="main_loop">main_loop</a>     