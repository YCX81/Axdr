# AxDr

基于 STM32G431 的 FOC 无刷电机驱动器。

## 功能

- FOC 矢量控制（Clarke/Park 变换 + SVPWM）
- AS5600 磁编码器（软件 I2C）
- 电流环 PI 闭环控制
- 开环、电压、电流三种运行模式
- VOFA+ JustFloat 调试输出（USB CDC，10kHz 采样率）
- CCMRAM 优化（LUT + FOC 状态零等待访问）

## 硬件参数

| 参数 | 值 |
|------|-----|
| MCU | STM32G431RBT6 (Cortex-M4F, 170MHz) |
| PWM 频率 | 20kHz（中心对齐） |
| 死区时间 | 100ns |
| 电流采样 | 3 路低侧采样（1mΩ 采样电阻 + 20x 运放） |
| ADC | 注入通道，TIM1 CH4 触发同步采样 |
| 编码器 | AS5600 磁编码器（软件 I2C） |
| 母线电压 | 24V（电阻分压检测） |
| 调试接口 | USB CDC (VOFA+) / USART3 |

## 软件架构

### 目录结构

```
Core/
├── Inc/                    头文件
│   ├── foc_config.h        硬件参数配置（PWM、ADC、采样电阻等）
│   ├── foc_ctrl.h          FOC 控制器结构体与接口
│   ├── foc_math.h          数学运算（Clarke/Park/sincos）
│   ├── svpwm.h             SVPWM 输出
│   ├── pid.h               PI 控制器
│   ├── as5600.h            磁编码器驱动
│   ├── soft_i2c.h          软件 I2C
│   └── vofa.h              VOFA+ 调试输出
├── Src/                    源文件
│   ├── foc_ctrl.c          FOC 主控制逻辑（状态机 + 电流环）
│   ├── foc_math.c          256 点 sine LUT + sincos + Clarke/Park 变换
│   ├── svpwm.c             min-max 零序注入 SVPWM
│   ├── pid.c               PI 控制器（带积分抗饱和）
│   ├── as5600.c            AS5600 I2C 读取 + 电角度计算
│   ├── soft_i2c.c          GPIO 模拟 I2C
│   ├── vofa.c              USB CDC JustFloat 协议输出
│   └── main.c              系统初始化 + 主循环 + ISR 回调
```

### 控制环路架构

```
                        20kHz ISR (TIM1 Update)
                        ┌──────────────────────────────────────────────┐
                        │                                              │
 iq_ref ────────┐       │  ADC (JDR1/2/3)                             │
 (来自上层或手动) │       │      │                                      │
                │       │      ▼                                      │
                │       │  ia, ib, ic  (三相电流)                      │
                │       │      │                                      │
                │       │      ▼                                      │
                │       │  Clarke 变换                                 │
                │       │  iα = ia                                    │
                │       │  iβ = (ia + 2·ib) / √3                     │
                │       │      │                                      │
                │       │      ▼          ┌──────────┐                │
                │       │  Park 变换 ◄────┤ sin/cos  │                │
                │       │  id, iq         │ (LUT)    │                │
                │       │      │          └────┬─────┘                │
                │       │      ▼               │                      │
                │       │  ┌────────┐          │                      │
                ├──────►│  │ PI (d) │──► vd    │                      │
  id_ref=0 ────►│       │  │ PI (q) │──► vq    │                      │
                │       │  └────────┘          │                      │
                │       │      │               │                      │
                │       │      ▼               │                      │
                │       │  逆 Park 变换 ◄──────┘ (复用 sin/cos)       │
                │       │  vα, vβ                                     │
                │       │      │                                      │
                │       │      ▼                                      │
                │       │  SVPWM (min-max 零序注入)                    │
                │       │  duty_a, duty_b, duty_c                     │
                │       │      │                                      │
                │       │      ▼                                      │
                │       │  TIM1 CCR1/2/3                              │
                │       └──────────────────────────────────────────────┘
                │
  编码器 ◄──── 主循环 (as5600_update)
  θ_elec        每次 ISR 从缓存读取最新角度
```

### 运行模式

FOC 控制器有三种运行模式，通过 `foc_ctrl_set_mode()` 切换：

| 模式 | 编码器 | 电流采样 | 电流环 | 用途 |
|------|--------|----------|--------|------|
| `FOC_MODE_OPEN_LOOP` | 不使用 | 不使用 | 关闭 | 自增角度驱动，用于无编码器启动 |
| `FOC_MODE_VOLTAGE` | 使用 | 采样但不闭环 | 关闭 | 固定 Vq 驱动，用于调试电流采样和编码器 |
| `FOC_MODE_CURRENT` | 使用 | 使用 | 开启 | 完整 FOC 闭环，id=0 控制，iq 跟踪力矩指令 |

### 启动流程

```
main()
  │
  ├── 1. 外设初始化 (GPIO, DMA, SPI, TIM, ADC, USART, USB)
  │
  ├── 2. 编码器初始化 (soft_i2c_init + as5600_init)
  │
  ├── 3. FOC 控制器初始化 (foc_ctrl_init)
  │
  ├── 4. ADC 校准 + 零电流偏移校准 (foc_ctrl_calibrate_offsets)
  │      采集 64 次 ADC 值取平均作为零电流基准
  │
  ├── 5. 编码器零点校准
  │      施加 Vα=15%Vbus, Vβ=0 → 转子对齐到 d 轴 (θ_elec=0)
  │      等待 1s 稳定后读取编码器原始值作为零偏移
  │
  ├── 6. 启动 FOC (foc_ctrl_start)
  │      开启 TIM1 PWM (CH1/2/3 + 互补) + CH4 ADC 触发
  │      开启 ADC 注入转换 + TIM1 更新中断
  │
  └── 7. 主循环
         ├── as5600_update() — 读取编码器（I2C，非实时）
         └── (其他非实时任务)

         ┌── TIM1 更新中断 (20kHz)
         │   ├── foc_ctrl_update()  — FOC 完整计算
         │   └── vofa_send_from_isr() — USB CDC 调试输出
         └──
```

### SVPWM 实现

采用 min-max 零序注入法，与传统扇区查表法数学等价，但无分支、代码更简洁：

```
输入: vα, vβ (逆 Park 输出)

1. 逆 Clarke: vα,vβ → va, vb, vc
   va = vα
   vb = -0.5·vα + (√3/2)·vβ
   vc = -0.5·vα - (√3/2)·vβ

2. 零序注入:
   vn = -(max(va,vb,vc) + min(va,vb,vc)) / 2
   va += vn,  vb += vn,  vc += vn

3. 归一化到占空比:
   duty = (v / Vbus + 0.5) × PWM_PERIOD → CCR
```

母线利用率 115%（= SPWM 的 2/√3 倍），等效于 7 段式 SVPWM 的居中对齐。

### 性能优化

| 优化项 | 说明 |
|--------|------|
| **CCMRAM** | `g_foc` 结构体和 `sin_lut[256]` 放在 CCMRAM (0x10000000)，CPU 独占总线，DMA 不抢占 |
| **sincos 合并** | `foc_sincos()` 一次算出 sin+cos，Park + 逆 Park 共用结果，查表从 4 次降到 2 次 |
| **无 while 角度归一化** | 用乘法+截断替代 while 循环，固定周期数 |
| **ADC 直接寄存器** | `ADC1->JDR1/2/3` 替代 HAL 函数，省去参数检查开销 |
| **USB CDC 调试** | 比 UART 115200 快 ~70 倍，支持 10kHz 采样率实时波形 |

### 内存布局

```
FLASH  (0x08000000, 128KB): 代码 + 常量 + CCMRAM 初始值
SRAM   (0x20000000,  32KB): 全局变量 + 堆 + 栈（DMA 可访问）
CCMRAM (0x10000000,  10KB): sin_lut (1KB) + g_foc (~120B)，CPU 独占
```

### VOFA+ 调试通道

通过 USB CDC 以 JustFloat 二进制协议输出，10kHz 采样率：

| 通道 | 变量 | 单位 |
|------|------|------|
| CH0 | theta_elec | rad |
| CH1 | ia | A |
| CH2 | ib | A |
| CH3 | ic | A |
| CH4 | id | A |
| CH5 | iq | A |
| CH6 | vd | V |
| CH7 | vq | V |

VOFA+ 设置：协议选 JustFloat，连接 USB 虚拟串口。

## 构建

需要 STM32CubeMX + CMake + arm-none-eabi-gcc 工具链。

```bash
cmake --preset Debug
cmake --build build/Debug
```

## 致谢

本项目基于 [disnox/AxDr_L](https://github.com/disnox/AxDr_L) 修改开发，感谢原作者的开源贡献。
