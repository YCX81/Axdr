# AxDr

基于 STM32G431 的 FOC 无刷电机驱动器。

## 功能

- FOC 矢量控制（Clarke/Park 变换 + SVPWM）
- AS5600 磁编码器（软件 I2C）
- 电流环 PI 闭环控制
- 开环、电压、电流三种运行模式
- VOFA+ JustFloat 调试输出（USB CDC）
- CCMRAM 优化（LUT + FOC 状态零等待访问）

## 致谢

本项目基于 [disnox/AxDr_L](https://github.com/disnox/AxDr_L) 修改开发，感谢原作者的开源贡献。
