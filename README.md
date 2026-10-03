# BALANCE_CAR

基于 **STM32F103C8T6** 的两轮平衡车控制项目，使用 **STM32 HAL + FreeRTOS（CMSIS-RTOS v2）** 构建，工程同时包含 C 与 C++ 源文件。

项目目前以“**底层驱动 + 实时任务框架 + 姿态数据采集 + 调试通信**”为主，正在逐步完善电机控制、编码器闭环和完整平衡控制逻辑。

---

## 1. 项目特点

- STM32F103C8T6，系统时钟 72 MHz
- STM32 HAL 驱动框架
- FreeRTOS + CMSIS-RTOS v2
- C / C++ 混合开发
- MPU6050 六轴 IMU
  - I2C2 通信
  - DMA 接收 14 字节原始数据
  - 加速度、角速度、温度解析
  - 四元数姿态解算
- HC-SR04 超声波测距
  - GPIO + EXTI 双边沿捕获
  - DWT 周期计时
- OLED 128×64
  - I2C1
  - DMA 发送队列
- 双串口通信
  - USART1：调试 / VOFA 数据输出
  - USART2：蓝牙遥控数据接收
- PWM 电机驱动接口
  - TIM1 CH1 / CH4
  - 4 路方向控制 GPIO
- TIM3 / TIM4 编码器接口已在 CubeMX 中配置
- WS2812 驱动
  - PWM + DMA
  - RGB、流水、闪烁、渐变、彩虹效果
- PID 控制器
  - 位置式 PID
  - 增量式 PID
  - 输出限幅与积分限幅
- 环形队列 / DMA 通信基础设施
- 独立的 BSP / Driver / Lib / Task 分层结构
- CMake + Ninja + arm-none-eabi-gcc 构建

---

## 2. 软件架构

```text
                    +----------------------+
                    |      FreeRTOS        |
                    |   CMSIS-RTOS v2      |
                    +----------+-----------+
                               |
        +----------------------+----------------------+
        |                      |                      |
        v                      v                      v
+---------------+      +---------------+      +---------------+
|  serialTask   |      | motionCtrlTask|      |   stateTask   |
|  调试/通信     |      | 运动控制框架   |      | 状态机框架     |
+-------+-------+      +-------+-------+      +-------+-------+
        |                      |                      |
        v                      v                      v
+--------------------------------------------------------------+
|                         Application                           |
|    app_debug / rc / task config / control logic              |
+--------------------------------------------------------------+
                               |
                               v
+--------------------------------------------------------------+
|                           Driver                             |
| MPU6050 | HC-SR04 | OLED | WS2812 | Motor520               |
+--------------------------------------------------------------+
                               |
                               v
+--------------------------------------------------------------+
|                            BSP                               |
|           UART / I2C / Interrupt / Queue                    |
+--------------------------------------------------------------+
                               |
                               v
+--------------------------------------------------------------+
|                 STM32 HAL + FreeRTOS Kernel                  |
+--------------------------------------------------------------+
```

---

## 3. 任务设计

项目通过一个一次性的 `initTask` 创建应用任务、消息队列和事件标志。

| 任务 | 频率 | 初始延时 | 栈大小 |
|---|---:|---:|---:|
| `serialTask` | 100 Hz | 500 ms | 256 words |
| `motionCtrlTask` | 500 Hz | 1000 ms | 512 words |
| `stateTask` | 50 Hz | 500 ms | 256 words |
| `initTask` | 初始化后退出 | - | 128 words |

任务频率通过：

```c
#define TASK_FREQ_SERIAL       100u
#define TASK_FREQ_MOTIONCTRL   500u
#define TASK_FREQ_STATE         50u
```

统一配置。

任务主体采用 `osDelayUntil()` 进行周期调度，避免单纯使用阻塞式延时造成周期漂移。

---

## 4. 外设与引脚

当前 CubeMX 工程中的主要外设配置如下。

| 外设 | 引脚 | 用途 |
|---|---|---|
| USART1 | PA9 / PA10 | 调试串口、VOFA 数据 |
| USART2 | PA2 / PA3 | 蓝牙串口 |
| I2C1 | PB8 / PB9 | OLED |
| I2C2 | PB10 / PB11 | MPU6050 |
| TIM1 CH1 | PA8 | 电机 PWM |
| TIM1 CH4 | PA11 | 电机 PWM |
| 电机方向 | PB12~PB15 | 双路电机方向控制 |
| TIM3 CH1 / CH2 | PA6 / PA7 | 编码器接口 |
| TIM4 CH1 / CH2 | PB6 / PB7 | 编码器接口 |
| HC-SR04 Trigger | PA0 | 超声波触发 |
| HC-SR04 Echo | PA1 | EXTI 双边沿回波检测 |
| ADC1 IN4 | PA4 | ADC 输入 |

基础串口参数：

```text
USART1：115200 8N1
USART2：115200 8N1
```

I2C 当前配置为：

```text
I2C1：100 kHz
I2C2：100 kHz
```

---

## 5. MPU6050

MPU6050 通过 I2C2 连接，采用 DMA 读取：

```text
ACCEL_XOUT_H ~ GYRO_ZOUT_L
```

一次读取 14 字节：

```text
加速度 X
加速度 Y
加速度 Z
温度
陀螺仪 X
陀螺仪 Y
陀螺仪 Z
```

当前配置：

- 加速度量程：±4 g
- 陀螺仪量程：±2000 °/s
- I2C DMA 接收
- EXTI5 作为数据更新触发入口
- 温度转换：

```c
temperature = raw_temperature / 340.0f + 36.53f;
```

姿态解算部分将原始加速度、角速度转换为物理量后，通过四元数积分与加速度反馈进行姿态融合，输出：

```text
roll
pitch
yaw
```

同时保留归一化后的加速度与角速度数据用于调试。

---

## 6. HC-SR04

超声波模块使用：

```text
PA0 -> TRIG
PA1 -> ECHO
```

Echo 采用双边沿 EXTI：

1. 上升沿记录开始时间
2. 下降沿记录结束时间
3. 利用 DWT `CYCCNT` 计算高电平持续时间
4. 换算距离

当前有效测量窗口约为：

```text
100 us < Echo < 25 ms
```

距离单位为：

```text
cm
```

无效回波返回：

```c
-1.0f
```

---

## 7. 电机控制接口

当前电机接口使用 TIM1 的两个 PWM 通道：

```text
TIM1_CH1 -> PA8
TIM1_CH4 -> PA11
```

方向控制：

```text
PB12
PB13
PB14
PB15
```

PWM 周期：

```text
TIM1 PSC = 0
TIM1 ARR = 7199
```

在 72 MHz 定时器时钟下，PWM 频率约为：

```text
10 kHz
```

当前 `motor520.c` 已完成 PWM 启动和方向 GPIO 初始化；完整的速度闭环、编码器反馈以及平衡控制仍在持续完善。

---

## 8. 编码器

CubeMX 中已经配置：

```text
TIM3 -> PA6 / PA7
TIM4 -> PB6 / PB7
```

均工作在 Encoder Interface 模式，计数周期：

```text
ARR = 65535
PSC = 0
```

当前工程已经具备硬件接口和定时器配置，但完整的编码器读取、速度计算和闭环控制逻辑还需要继续集成。

---

## 9. OLED

OLED 驱动位于：

```text
Usr/Driver/OLED/
```

当前屏幕参数：

```text
128 × 64
I2C1
```

底层使用 I2C1 DMA + 环形队列处理发送，可以减少任务中频繁阻塞式发送带来的影响。

---

## 10. WS2812

WS2812 驱动位于：

```text
Usr/Driver/WS2812/
```

支持：

```c
ws2812_rgb_all()
ws2812_rgb_unit()
ws2812_effect_flow()
ws2812_effect_blink()
ws2812_effect_gradient()
ws2812_rainbow()
```

颜色格式：

```text
0x00RRGGBB
```

驱动采用：

```text
72 MHz timer clock
800 kHz PWM
DMA
```

并提供：

```text
RGB / HSV
亮度缩放
流水
闪烁
渐变
彩虹
```

### 注意

当前 WS2812 驱动源码使用 `TIM3_CH3`，而当前 CubeMX 工程中的 TIM3 同时配置为 `CH1 / CH2` 编码器接口。

因此，在当前版本中，**WS2812 与 TIM3 编码器配置存在资源复用冲突**。实际使用 WS2812 前需要根据硬件重新分配定时器/通道，或者修改驱动映射。

---

## 11. PID

PID 实现位于：

```text
Usr/Lib/
```

支持：

- 位置式 PID
- 增量式 PID
- 输出限幅
- 积分限幅

接口示例：

```c
float usr_pid_caculate(
    pid_t *pid,
    float input,
    float set
);
```

当前运动控制任务中已经建立 PID 实例，后续可用于：

```text
角度环
速度环
位置环
```

等控制结构。

---

## 12. 调试与 VOFA

Debug 模式在 `CMakeLists.txt` 中默认启用：

```cmake
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    DEBUG
)
```

支持：

```c
DEBUG_PRINT(...)
```

并通过 USART1 向 VOFA 发送浮点数据。

当前运动控制任务周期性输出：

```text
Acc X
Acc Y
Acc Z
Gyro X
Gyro Y
Gyro Z
Roll
Pitch
Yaw
Temperature
Distance
```

VOFA 帧尾：

```text
0x7F800000
```

---

## 13. 蓝牙遥控

USART2 用于接收蓝牙遥控数据。

当前定义的方向命令：

```c
RC_DIR_AHEAD = 0x01
RC_DIR_BACK  = 0x02
RC_DIR_LEFT  = 0x11
RC_DIR_RIGHT = 0x12
RC_DIR_BRAKE = 0xFF
```

接收采用中断方式，并封装在：

```text
Usr/Bsp/bsp_uart.c
```

之上。

---

## 14. 工程目录

```text
BALANCE_CAR/
├── Core/                       # STM32CubeMX 生成的核心代码
│   ├── Inc/
│   └── Src/
│
├── Drivers/                   # CMSIS / STM32 HAL / FreeRTOS 等底层依赖
│
├── Middlewares/
│   └── Third_Party/
│       └── FreeRTOS/
│
├── Usr/
│   ├── App/                   # 应用层
│   ├── Bsp/                   # 板级支持层
│   ├── Driver/                # 外设/器件驱动
│   ├── Lib/                   # PID、队列、延时、姿态融合等基础库
│   └── Task/                  # FreeRTOS 任务
│
├── cmake/                     # ARM GCC 工具链与 CubeMX CMake
│
├── Balance_Car_HAL_Proj.ioc   # STM32CubeMX 工程配置
├── CMakeLists.txt
├── CMakePresets.json
└── STM32F103XX_FLASH.ld
```

---

## 15. 构建环境

推荐环境：

- Windows / Linux
- CMake >= 3.22
- Ninja
- `arm-none-eabi-gcc`
- STM32CubeMX
- VS Code
- Cortex-Debug / OpenOCD / ST-LINK 等烧录调试工具

工程使用 CMake Presets，并自带 ARM GCC Toolchain：

```text
cmake/gcc-arm-none-eabi.cmake
```

确保：

```text
arm-none-eabi-gcc
arm-none-eabi-g++
arm-none-eabi-objcopy
arm-none-eabi-size
```

已经加入系统 `PATH`。

---

## 16. CMake 构建

### Debug

```powershell
cmake --preset Debug
cmake --build --preset Debug
```

### Release

```powershell
cmake --preset Release
cmake --build --preset Release
```

构建目录：

```text
build/
├── Debug/
└── Release/
```

工程目标文件使用 `.elf` 格式。

---

## 17. STM32CubeMX

如果需要修改 MCU 外设配置：

1. 使用 STM32CubeMX 打开：

```text
Balance_Car_HAL_Proj.ioc
```

2. 修改 GPIO / DMA / TIM / USART / I2C / FreeRTOS 等配置
3. 重新生成代码
4. 检查 `Usr/` 下的用户代码是否需要同步修改
5. 重新使用 CMake 构建

建议优先把应用逻辑放在：

```text
Usr/
```

中，减少 CubeMX 重新生成代码时的改动冲突。

---

## 18. 当前开发状态

### 已完成 / 已搭建

- [x] STM32F103C8T6 HAL 工程
- [x] 72 MHz 系统时钟
- [x] FreeRTOS + CMSIS-RTOS v2
- [x] 多任务框架
- [x] UART DMA 发送队列
- [x] I2C DMA 发送/接收基础框架
- [x] MPU6050 驱动
- [x] MPU6050 姿态融合
- [x] HC-SR04 驱动
- [x] OLED 驱动
- [x] WS2812 驱动
- [x] PID 基础库
- [x] 蓝牙遥控数据解析
- [x] VOFA 调试输出
- [x] TIM3 / TIM4 编码器外设配置
- [x] 电机 PWM / 方向控制基础接口

### 后续完善

- [ ] 编码器速度 / 位置计算
- [ ] 电机速度闭环
- [ ] 平衡车角度环
- [ ] 级联 PID 控制
- [ ] 蓝牙指令与底盘运动控制联动
- [ ] 状态机完善
- [ ] WS2812 与编码器的定时器资源重新分配
- [ ] 低功耗、异常恢复与更完整的故障保护

---

## 19. 许可证

本仓库当前未单独附加项目 License 文件。

如果将项目公开发布，建议根据实际情况补充合适的开源许可证。

---

## 20. Repository

GitHub：

https://github.com/XCthulhudj/BALANCE_CAR
