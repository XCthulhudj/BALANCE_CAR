# Balance Car HAL Project

基于 **STM32F103C8T6** 的两轮自平衡小车控制工程，采用 **STM32 HAL + FreeRTOS/CMSIS-RTOS2** 架构开发。

当前工程已经包含：

- MPU6050 六轴 IMU 数据采集与姿态融合
- 四元数姿态解算与坐标系旋转
- 双直流减速电机 PWM + 正交编码器反馈
- 速度环 + 姿态环 + 转向环三级控制
- HC-SR04 超声波测距
- 0.96 英寸 SSD1306 OLED
- 3 颗 WS2812 RGB LED
- USART1 调试串口
- USART2 蓝牙遥控输入
- FreeRTOS 多任务调度
- UART / I2C DMA + 软件队列
- 独立看门狗
- CMake + Ninja + arm-none-eabi-gcc 构建

> 项目定位：个人两轮自平衡机器人控制工程，重点用于 STM32 嵌入式开发、FreeRTOS 任务设计、传感器驱动、姿态融合和闭环控制算法学习与实践。

---

## 1. Hardware

### MCU

| 项目 | 参数 |
|---|---|
| MCU | STM32F103C8T6 |
| 内核 | ARM Cortex-M3 |
| 主频 | 72 MHz |
| 封装 | LQFP48 |
| 工程框架 | STM32 HAL |
| RTOS | FreeRTOS / CMSIS-RTOS2 |

### 外设

| 模块 | 用途 | STM32 外设 / 引脚 |
|---|---|---|
| MPU6050 | 加速度、角速度采集 | I2C2：PB10/PB11 |
| MPU6050 INT | 数据就绪中断 | PB5 / EXTI |
| OLED SSD1306 | 状态显示 | I2C1：PB8/PB9 |
| 电机 1 PWM | 电机驱动 | TIM1 CH4 |
| 电机 2 PWM | 电机驱动 | TIM1 CH1 |
| 电机 1 编码器 | 速度反馈 | TIM3 |
| 电机 2 编码器 | 速度反馈 | TIM4 |
| HC-SR04 TRIG | 超声波触发 | PA0 |
| HC-SR04 ECHO | 超声波回波 | PA1 / EXTI1 |
| WS2812 | 状态指示 | TIM3 CH3 |
| 调试串口 | 调试 / VOFA 数据 | USART1，115200 |
| 蓝牙串口 | 遥控输入 | USART2，115200 |
| ADC | 模拟量采集接口 | ADC1_IN4 / PA4 |
| 独立看门狗 | 系统异常恢复 | IWDG |

### 电机驱动

工程中的 `motor520` 驱动负责：

- TIM1 输出两路 PWM
- TIM3 / TIM4 工作在编码器模式
- 读取两个电机的增量编码器计数
- 根据正负方向控制 H 桥方向引脚
- 输出 0～7200 的 PWM 比较值

对应关系：

```text
TIM1 CH4 -> 电机 1 PWM
TIM1 CH1 -> 电机 2 PWM

TIM3 -> 电机 1 编码器
TIM4 -> 电机 2 编码器
```

---

## 2. Software Architecture

工程采用分层结构：

```text
Application
├── Task
│   ├── initTask
│   ├── motionCtrlTask
│   ├── serialTask
│   └── stateTask
│
├── App
│   ├── attitude
│   ├── rc
│   └── app_debug
│
├── BSP
│   ├── UART
│   ├── I2C
│   └── EXTI / Interrupt Hook
│
├── Driver
│   ├── MPU6050
│   ├── Motor520
│   ├── HC-SR04
│   ├── OLED
│   └── WS2812
│
└── Lib
    ├── PID
    ├── IMU Fusion
    ├── Delay
    └── Queue
```

底层由 STM32CubeMX 生成：

```text
Core
Drivers
Middlewares/Third_Party/FreeRTOS
```

构建系统采用：

```text
CMake
 └── Ninja
      └── arm-none-eabi-gcc
```

---

## 3. FreeRTOS Tasks

当前应用任务由 `initTask` 动态创建。

| Task | 运行频率 | 初始延时 | Stack |
|---|---:|---:|---:|
| `serialTask` | 100 Hz | 500 ms | 256 words |
| `motionCtrlTask` | 500 Hz | 1000 ms | 512 words |
| `stateTask` | 50 Hz | 500 ms | 256 words |

FreeRTOS Tick：

```text
configTICK_RATE_HZ = 1000 Hz
```

任务优先级当前均设置为：

```text
osPriorityRealtime
```

`initTask` 完成任务和消息队列创建后自行退出。

当前消息队列：

```text
serial     : 16 × uint32_t
motionCtrl : 16 × uint32_t
state      : 16 × uint32_t
```

FreeRTOS 动态堆：

```text
configTOTAL_HEAP_SIZE = 6656 bytes
```

内存管理方案：

```text
heap_4
```

---

## 4. Motion Control

核心控制流程位于：

```text
Usr/Task/Src/motionCtrlTask.cpp
```

每次控制周期执行：

```text
MPU6050 更新
      ↓
姿态融合
      ↓
坐标系旋转
      ↓
读取编码器
      ↓
读取超声波
      ↓
读取遥控状态
      ↓
计算目标速度 / 转向
      ↓
速度 PID
      ↓
姿态 PID
      ↓
转向 PID
      ↓
左右电机 PWM
```

当前控制任务频率：

```text
500 Hz
```

---

## 5. PID Control

当前使用 3 个位置式 PID：

### 姿态环

```text
Kp = 400.0
Ki = 0.0
Kd = 1.0

max_out  = 3000
max_Iout = 100
```

输入：

```text
当前 Pitch
```

设定值来自速度环输出。

---

### 速度环

```text
Kp = 0.1
Ki = 0.05
Kd = 0.0

max_out  = 100
max_Iout = 100
```

输入：

```text
cn1_enc - cn2_enc
```

---

### 转向环

```text
Kp = 10.0
Ki = 0.0
Kd = 0.6

max_out  = 100
max_Iout = 100
```

输入：

```text
gyro_z
```

目标值来自遥控转向指令。

---

### 电机输出

最终左右电机控制量：

```text
motor1 = -pitch_output + turn_output
motor2 =  pitch_output - turn_output
```

电机方向由输出正负决定，PWM 使用绝对值。

---

## 6. MPU6050 Attitude Fusion

MPU6050 当前配置：

```text
加速度计：±4 g
陀螺仪：±2000 °/s
采样分频：SMPLRT_DIV = 1
数字低通：CONFIG = 0x03
```

每次通过 MPU6050 的 `INT` 信号触发数据读取：

```text
MPU6050 INT
    ↓
EXTI5
    ↓
设置 rx_sig
    ↓
motionCtrlTask 调用 mpu6050_update()
    ↓
I2C2 DMA 读取 14 bytes
    ↓
DMA 完成回调
    ↓
解析加速度 / 温度 / 陀螺仪
```

一次读取：

```text
ACC_X
ACC_Y
ACC_Z
TEMP
GYRO_X
GYRO_Y
GYRO_Z
```

共：

```text
14 bytes
```

### 陀螺仪零偏校准

启动后先累计约 999 组陀螺仪数据：

```text
gyro_offset = sum / count
```

之后从原始陀螺仪数据中减去零偏。

校准完成后：

```text
cali_sig = 1
```

并通过线程标志通知 `stateTask`。

---

## 7. Quaternion Fusion

姿态融合文件：

```text
Usr/Lib/Src/usr_fusion.c
```

主要功能：

- 原始数据单位转换
- 加速度归一化
- 根据四元数估计重力方向
- 加速度与重力方向叉积计算误差
- PI 反馈修正陀螺仪
- 四元数积分
- 四元数归一化
- 四元数转换为 Roll / Pitch / Yaw

当前融合参数：

```text
Kp = 1.50
Ki = 0.005
```

积分周期参数：

```text
halfT = 0.001 s
```

### 坐标系旋转

工程额外提供：

```c
usr_coordinate_rotation_internal()
```

用于对姿态四元数进行坐标系旋转。

当前运动控制中使用：

```c
usr_coordinate_rotation_internal(
    q_in,
    q_out,
    euler_out,
    0, 0, 1,
    90
);
```

同时通过：

```c
usr_rq_vector()
```

将加速度和角速度向量转换到机器人使用的坐标系。

---

## 8. MPU6050 Temperature

MPU6050 原始温度值转换：

```c
temperature = raw_temperature / 340.0f + 36.53f;
```

单位：

```text
°C
```

---

## 9. Remote Control

USART2 当前作为蓝牙 / 遥控输入接口：

```text
115200 8N1
```

采用 1 byte 指令。

定义：

```text
0x01 -> 前进
0x02 -> 后退
0x11 -> 左转
0x12 -> 右转
0xFF -> 刹车
```

遥控数据通过：

```text
USART2 RX interrupt
    ↓
rc_parser()
    ↓
rc.dir
    ↓
motionCtrlTask
```

---

## 10. Debug Interface

USART1：

```text
115200 8N1
```

用于：

- `printf`
- 调试数据
- VOFA+ 数据发送
- 简单调试控制

当前调试命令：

```text
'a' -> app_debug.flag = 1
'z' -> app_debug.flag = 0
```

调试标志当前会影响运动控制中的速度目标：

```text
flag == 1 -> v_enc_set = 10
flag == 0 -> v_enc_set = 0
```

因此目前代码中的 DEBUG 控制逻辑会覆盖遥控产生的前进速度设定，后续如果用于正式遥控控制，需要根据实际需求调整这一部分。

### VOFA+ 数据

运动控制任务会周期性发送：

```text
Roll
Pitch
Yaw
HC-SR04 distance
Object Euler X
Object Euler Y
Object Euler Z
Encoder setpoint
Encoder feedback
Debug flag
Velocity PID output
Pitch PID output
Turn PID output
```

帧尾：

```text
0x7F800000
```

---

## 11. HC-SR04

HC-SR04 使用：

```text
TRIG -> PA0
ECHO -> PA1 / EXTI1
```

触发脉冲：

```text
约 12 us
```

回波时间通过：

```text
DWT->CYCCNT
```

进行高精度计时。

有效距离范围代码当前设置为：

```text
约 2 cm ~ 400 cm
```

距离计算：

```text
distance(cm) = echo_time(us) × 0.017
```

无效数据：

```text
distance = -1.0f
```

---

## 12. OLED

OLED 使用 SSD1306 类 128 × 64 显示器：

```text
I2C1
Address = I2C1_ADDRESS
Resolution = 128 × 64
```

驱动支持：

- 清屏
- ASCII 字符
- 字符串
- Bitmap
- 自定义字体
- 动画
- I2C DMA 发送

OLED 数据通过 I2C1 软件队列发送，避免应用层直接阻塞等待 I2C 完成。

---

## 13. WS2812

WS2812 使用：

```text
TIM3 CH3
PWM + DMA
```

PWM 频率：

```text
800 kHz
```

系统时钟：

```text
72 MHz
```

定时器：

```text
PSC = 0
ARR = 89
```

当前 LED 数量：

```text
3
```

支持：

- 单色控制
- 单颗 LED 控制
- 流水灯
- 闪烁
- 渐变
- HSV → RGB
- 彩虹效果

---

## 14. UART / I2C DMA Architecture

UART 和 I2C 外设均采用队列 + DMA 的方式处理发送。

### UART

```text
Application
    ↓
uart_write_data()
    ↓
software queue
    ↓
DMA
    ↓
TX complete callback
    ↓
继续发送 / 队列空闲
```

USART1 和 USART2 各自拥有：

```text
256 bytes TX queue
```

接收目前采用：

```text
HAL_UART_Receive_IT()
```

以固定长度接收，并通过 hook 回调交给上层解析。

### I2C

I2C1：

```text
OLED TX queue + DMA
```

I2C2：

```text
MPU6050 memory read + DMA
```

---

## 15. Project Structure

```text
Balance_Car_HAL_Proj/
│
├── Core/
│   ├── Inc/
│   └── Src/
│
├── Drivers/
│   ├── CMSIS/
│   └── STM32F1xx_HAL_Driver/
│
├── Middlewares/
│   └── Third_Party/
│       └── FreeRTOS/
│
├── Usr/
│   ├── App/
│   │   ├── Inc/
│   │   └── Src/
│   │
│   ├── Bsp/
│   │   ├── Inc/
│   │   └── Src/
│   │
│   ├── Driver/
│   │   ├── HC_SR04/
│   │   ├── Motor520/
│   │   ├── MPU6050/
│   │   ├── OLED/
│   │   └── WS2812/
│   │
│   ├── Lib/
│   │   ├── Inc/
│   │   └── Src/
│   │
│   ├── Task/
│   │   ├── Inc/
│   │   └── Src/
│   │
│   ├── usr_config.c
│   └── usr_config.h
│
├── cmake/
│   └── gcc-arm-none-eabi.cmake
│
├── Balance_Car_HAL_Proj.ioc
├── CMakeLists.txt
├── CMakePresets.json
├── STM32F103XX_FLASH.ld
├── startup_stm32f103xb.s
└── README.md
```

---

## 16. Build

### Toolchain

需要准备：

- CMake
- Ninja
- `arm-none-eabi-gcc`
- STM32CubeMX（如果需要修改 `.ioc`）
- VS Code（推荐）

工程已经提供：

```text
CMakePresets.json
cmake/gcc-arm-none-eabi.cmake
```

### Debug 构建

```bash
cmake --preset Debug
cmake --build --preset Debug
```

### Release 构建

```bash
cmake --preset Release
cmake --build --preset Release
```

生成的主要文件位于：

```text
build/Debug/
build/Release/
```

其中包含：

```text
Balance_Car_HAL_Proj.elf
Balance_Car_HAL_Proj.hex
Balance_Car_HAL_Proj.map
```

如果使用 VS Code，也可以直接通过 CMake Tools / Ninja 进行配置和编译。

---

## 17. Configuration

主要工程参数集中在：

```text
Usr/usr_config.h
Usr/usr_config.c
```

例如任务频率：

```c
#define TASK_FREQ_SERIAL       100u
#define TASK_FREQ_MOTIONCTRL   500u
#define TASK_FREQ_STATE         50u
```

PID 参数：

```text
Usr/Task/Src/motionCtrlTask.cpp
```

FreeRTOS 参数：

```text
Core/Inc/FreeRTOSConfig.h
```

STM32 外设配置：

```text
Balance_Car_HAL_Proj.ioc
```

---

## 18. Current Development Status

当前工程已经完成基础软件框架和主要外设驱动，并能够形成完整的：

```text
传感器采集
    ↓
姿态融合
    ↓
状态估计
    ↓
目标设定
    ↓
PID 控制
    ↓
电机输出
```

目前仍属于持续开发中的个人项目。

后续可以继续完善：

- [ ] 更完善的遥控协议
- [ ] 姿态 / 速度环参数整定
- [ ] 编码器速度单位和实际车速标定
- [ ] 电机死区补偿
- [ ] 电池电压监测与低压保护
- [ ] 更完善的异常状态机
- [ ] 电机堵转 / 失控检测
- [ ] MPU6050 姿态融合参数优化
- [ ] OLED 实时状态界面
- [ ] 完善正式运动控制逻辑
- [ ] 增加硬件原理图和 PCB 文档

---

## 19. Development Notes

### 代码风格

工程中底层驱动主要使用 C：

```text
.c / .h
```

任务层部分使用 C++：

```text
.cpp / .h
```

C / C++ 之间通过：

```c
#ifdef __cplusplus
extern "C"
{
#endif
```

进行接口兼容。

### 时间管理

FreeRTOS 任务周期主要使用：

```c
osDelayUntil()
```

避免使用固定延时导致任务周期随执行时间漂移。

### 调试

Debug 构建定义：

```text
DEBUG
```

支持：

```c
DEBUG_PRINT(...)
CHECK_STACK_AVAILABLE(...)
CHECK_HEAP_AVAILABLE()
VOFA_SEND_FLOATS(...)
```

用于观察：

- 任务栈余量
- FreeRTOS Heap
- 姿态角
- 编码器
- PID 输出
- 超声波距离

---

## 20. Author

个人嵌入式 / 机器人控制项目。

技术方向：

```text
STM32
C / C++
FreeRTOS
传感器驱动
姿态解算
PID
电机控制
机器人电子控制
PCB / 嵌入式系统
```

项目用于个人学习、机器人控制算法验证以及嵌入式工程实践。
