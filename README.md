# 钢球动态平衡小车

## 结构

- `main.c`：任务状态机；10 ms 调度循迹、平衡和计时。
- `balance.c/.h`：钢球平衡双环控制；外环由球位置/速度生成目标倾角，内环跟踪倾角并驱动步进电机。
- `Driver/k230_uart.c/.h`：K230 串口环形缓冲与 `S<pos>,<vel>,<acc>` 数据帧解析。
- `Board/mt6816.c/.h`：MT6816 编码器；AB 相计数、PWM 绝对角度和 Z 相索引。
- `Driver/motor.c/.h`：车轮电机与速度/循迹控制。
- `Driver/encoder.c/.h`：车轮霍尔编码器计数和转速计算。
- `Driver/gray.c/.h`：8 路灰度传感器采样与线位置计算。
- `Driver/imu.c/.h`：车体加速度读取，供平衡前馈使用。
- `Driver/seven_seg.c/.h`：比赛计时显示。
- `Board/car_config.h`：可调控制参数。
- `Board/hw_config.h`、`empty.syscfg`：硬件引脚和外设配置。

## 控制流程

1. K230 输出球位置、速度；MT6816 输出摆杆角度。
2. 平衡外环计算目标倾角，内环生成步进电机命令。
3. 循迹模块按灰度位置控制车轮；IMU 加速度作为平衡前馈。
4. `main.c` 根据任务号组合并启停模块。

## 调参

- 先确认步进方向、编码器方向和 K230 坐标方向。
- 先调 `SCR_OUTER_KP/KD`，再调 `SCR_INNER_KP`，最后调 `SCR_ACC_FF`。
- 循迹先调速度内环，再调位置外环；每次只改一个参数。

