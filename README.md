# 电控组第三次培训作业

**姓名**：韩景捷　　**学号**：20263008520

| 项目 | 内容 |
| --- | --- |
| 题目 | 串口和按键控制流水灯与 OLED 状态显示 |
| 主控 | STM32F103C8T6（Blue Pill，64 KB Flash / 20 KB RAM） |
| 开发环境 | CLion + CMake + arm-none-eabi-gcc + STM32CubeMX（HAL 库）+ OpenOCD（ST-Link） |
| 仓库地址 | https://github.com/hjstm32/OLED_Three |
| 完成情况 | 必做项全部完成；选做（硬件 I²C、DMA）未做 |

## 一、参考资料和驱动来源

| 来源 | 用在哪里 | 做了哪些改动 |
| --- | --- | --- |
| 江协科技《STM32 入门教程》——「OLED 显示」章节例程 | `Core/Src/OLED.c`、`Core/Inc/OLED.h`、`Core/Inc/OLED_Font.h` | 保留其软件 I²C 位时序（起始/停止条件、逐位发送、第 9 个时钟读 ACK）、SSD1306 初始化命令序列和 8×16 ASCII 字模表（字模原表未改）。改写部分：① 删掉例程里的 `OLED_Test`、汉字显示等演示函数，改成只保留 `OLED_Init/Clear/ShowChar/ShowString/ShowNum` 的独立模块；② 上电自检——先按 PB1=SCL / PB0=SDA 探 `0x78`、`0x7A`，都没应答就把两根线对调再探一次；③ 显存数据改成一次事务连续写，供清屏和整行刷新使用；④ 速率压到 300 kHz 左右（`OLED_I2C_DELAY_LOOP = 8`）。 |
| STMicroelectronics STM32F1 HAL 驱动库（STM32CubeF1，工程内 `Drivers/`） | 全工程 | 直接调用：`HAL_UART_Receive_IT` / `HAL_UART_Transmit_IT` / `HAL_GPIO_EXTI_IRQHandler` / `HAL_GPIO_ReadPin` / `HAL_GetTick` / `HAL_Delay` / `HAL_NVIC_SetPriority`。 |
| STM32CubeMX 生成的初始化骨架 | `Core/Src/gpio.c`、`usart.c`、`stm32f1xx_it.c`、`main.c` 的 `MX_xxx_Init` | 串口、PA11 按键、PC13 灯沿用 CubeMX 生成代码。PA0/PA1/PA2（三灯）和 PA4（按键 B）是后接的，为不破坏 `.ioc` 文件，改写在 `main.c` 的 `App_Gpio_Init()`（USER CODE 段）；`EXTI4_IRQHandler` 写在 `stm32f1xx_it.c` 的 `USER CODE BEGIN 1` 段，重新生成代码不会被覆盖。 |
| 本次培训《学习与操作指南》 | 接线与硬件配置 | OLED 接法（VCC→3.3 V、GND→GND、PB1→SCL、PB0→SDA）；按键内部上拉 + 下降沿外部中断（EXTI11 / EXTI4）；USART1 = PA9(TX)/PA10(RX)，115200 8-N-1，与 USB-TTL 交叉连接并共地。 |
| 本人编写的部分 | `Core/Src/main.c` | 状态机、按键三段式消抖、环形缓冲 + 拼包校验与重对齐、OLED 差异刷新、反馈组帧与发送调度、上电自检流程。 |

## 二、串口反馈格式（自定义）

板子应用完一次状态后，向上位机回一行 ASCII：

```
STATE=<RUN|PAUSE>,DIR=<FWD|REV>,STEP=<间隔ms>,LED=<1~3>\r\n
```

| 字段 | 取值 | 含义 | 对应 OLED 行 |
| --- | --- | --- | --- |
| `STATE` | `RUN` / `PAUSE` | 运行 / 暂停 | 第 1 行 |
| `DIR` | `FWD` / `REV` | 正向 / 反向 | 第 2 行 |
| `STEP` | 100 ~ 2000 | 相邻 LED 切换间隔，单位 ms，不带单位 | 第 3 行 |
| `LED` | 1 ~ 3 | **当前正亮着**的那颗灯 | 第 4 行 |

这样定义的理由：

1. 一行读完、字段与 OLED 四行严格一一对应，方便对照"屏幕和串口是否同步"；
2. 用 `键=值` 而不是裸二进制回传，串口助手用文本模式就能直接看，也方便上位机按逗号切分；
3. 不带单位、不带空格，避免 GBK/UTF-8 编码差异导致乱码。

发送时机（每次状态真正变化后**恰好一行**）：

| 触发 | 是否回传 |
| --- | --- |
| 上电初始化完成（三灯自检、OLED 第一屏之后） | 回一行，用来确认串口通路正常 |
| 按键 A 改变方向 | 回一行 |
| 按键 B 切换暂停 / 继续 | 回一行 |
| 串口数据包校验通过并已应用 | 回一行（帧头/校验/取值不合法时不回） |

实现上统一用 `HAL_UART_Transmit_IT()` 启动发送（满足"必须用中断方式"的要求）。发送缓冲区加了 `tx_busy` 互斥：上一行没发完时只置"待发"标记，等主循环下一圈再发，因此不会出现两行数据拼在一起或半行被改写的情况。暂停期间按键和串口照常回传，恢复后按新状态执行。

实际输出示例：

```
STATE=RUN,DIR=FWD,STEP=300,LED=1
STATE=RUN,DIR=REV,STEP=300,LED=2
STATE=PAUSE,DIR=REV,STEP=300,LED=2
STATE=RUN,DIR=REV,STEP=1000,LED=2
STATE=RUN,DIR=FWD,STEP=300,LED=3
```

> 说明：`LED` 是回传那一刻亮着的灯，会随流水灯变化，不是固定值；同一时刻它与 OLED 第 4 行完全一致。
