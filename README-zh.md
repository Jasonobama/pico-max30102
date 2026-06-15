# SpO2 / 心率监测器 (MAX30102 + SSD1306, Pico / Pico 2)

[English](README.md)

基于内置 MAX30102 和 SSD1306 驱动的血氧与心率监测器，适用于树莓派 Pico 和 Pico 2。

## 文件结构

| 文件 | 用途 |
|---|---|
| `MAX30102.c` | 程序入口：传感器初始化、采集循环、OLED 仪表盘 |
| `algorithm.c` / `algorithm.h` | SpO2（比率法）和心率（自相关）估算 |
| `max30102.c` / `max30102.h` | MAX30102 驱动（未修改） |
| `ssd1306.c` / `ssd1306.h` | SSD1306 OLED 驱动（波特率改为可配置） |
| `CMakeLists.txt` | 构建配置，默认为 Pico 2 (RP2350) |

## 接线

| 信号 | MAX30102 | SSD1306 OLED |
|---|---|---|
| I2C 总线 | I2C0 | I2C1 |
| SDA | GP4 | GP6 |
| SCL | GP5 | GP7 |
| VCC | 3.3V | 3.3V |
| GND | GND | GND |

两个设备挂载在独立的 I2C 总线上，避免彼此通信（尤其是 OLED 的大数据量写入）互相阻塞。

> **注意**：本项目使用的 SSD1306 OLED 在 I2C1 上以 **100kHz** 标准模式运行。这是因为 RP2350 内部上拉电阻（约 50kΩ）在 400kHz 快速模式下无法提供足够的信号边沿速率。若需 400kHz，请在 GP6/GP7 上外接 4.7kΩ 上拉电阻至 3.3V。

## 构建步骤

1. 安装 Raspberry Pi Pico SDK（**v2.0.0 或更高版本**，Pico 2 / RP2350 需要），设置 `PICO_SDK_PATH`。
2. 将 SDK 的导入脚本复制到项目目录：
   ```bash
   cp $PICO_SDK_PATH/external/pico_sdk_import.cmake .
   ```
3. 配置并构建：
   ```bash
   mkdir build && cd build
   cmake ..              # 默认构建目标为 Pico 2 (RP2350)
   make -j4
   ```
   如要编译给原始 Pico：
   ```bash
   cmake -DPICO_BOARD=pico ..
   ```
4. 烧录 `MAX30102.uf2`：按住 BOOTSEL 键插入开发板，然后将文件复制到挂载的 U 盘中。

## 工作原理

每个循环周期中，传感器 FIFO 被完全排空（通过反复调用 `max30102_check()` 直到读写指针对齐，以处理驱动层 32 字节的突发读取上限），每对 (IR, RED) 采样数据送入 `spo2_algorithm_add_sample()`。

* **直流跟踪**：一阶 IIR 低通滤波器（alpha = 0.95）跟踪每个通道缓慢变化的 DC 分量；减去 DC 后得到脉动 AC 分量。
* **手指检测**：将 IR DC 值与 `SPO2_FINGER_THRESHOLD` 比较。低于阈值时，屏幕显示"Place finger"。
* **SpO2 计算**：当 100 个采样点（4 秒）的窗口填满后，计算每个通道 AC 信号的 RMS 值，并组合为标准的比率-比率公式 `R = (RMS_red/DC_red) / (RMS_ir/DC_ir)`，然后用常用线性近似 `SpO2 = 110 - 25*R` 映射。
* **心率计算**：对 IR AC 信号在 40-200 BPM 对应的滞后范围内进行自相关计算；相关性最强的滞后即为脉搏周期。
* **显示**：每秒刷新一次，显示 SpO2、脉搏以及原始 IR DC 值（用于调试手指检测阈值）。

> **免责声明**：`110 - 25*R` 公式和默认 LED 电流是演示用途的常见起点，并非校准医疗器械。真正的脉搏血氧仪需要对照参考仪器对大量受试者进行校准。请将 SpO2 输出视为近似值，如需提高精度，可调整 `SPO2_FINGER_THRESHOLD`、LED 电流（通过传入自定义 `max30102_config_t` 替代 `NULL`）以及 SpO2 公式常数。

## 调试说明

* `SPO2_FINGER_THRESHOLD`（在 `algorithm.h` 中）：根据 OLED 上显示的"IR DC"值在有/无手指时的差异，调高或调低此阈值。
* LED 电流：将自定义的 `max30102_config_t` 传入 `max30102_setup()`（替代 `NULL`），如果 AC 信号太弱（例如较厚组织或深色外壳），可增大 `led1_current`/`led2_current`。
* `SPO2_BUFFER_SIZE` / 窗口长度：较短窗口响应更快，较长窗口更稳定，需在这两者之间权衡。

## 故障排除

| 问题 | 可能原因 | 解决方法 |
|---|---|---|
| OLED 无显示 | I2C1 波特率过高 | 确保 `SSD1306_I2C_BAUD=100000`（已在 `MAX30102.c` 中设置） |
| OLED 无显示 | 接线错误 | 检查 GP6(SDA)、GP7(SCL)、3.3V、GND |
| OLED 无显示 | 缺少上拉电阻 | GP6/GP7 外接 4.7kΩ 至 3.3V |
| MAX30102 未找到 | 接线错误 | 检查 GP4(SDA)、GP5(SCL)、3.3V、GND |
| 读数始终为 0 | 传感器未接触手指 | 将手指轻放在传感器上 |
| SpO2/HR 值异常 | 手指接触不稳定 | 保持手指不动，避免按压过重 |
