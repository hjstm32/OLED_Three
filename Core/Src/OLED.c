/**
  * @brief  SSD1306 128x64 OLED 驱动（软件 I2C）
  * @note   驱动来源：江协科技《STM32 入门教程》OLED 例程，按本次作业改写
  *         （删掉原例程的测试函数，改成只在 USER CODE 里调用的独立模块）。
  *
  *         接线：VCC -> 3.3V，GND -> GND，SCL -> PB1，SDA -> PB0
  *
  *         为什么 SCL 用推挽、SDA 用开漏：
  *           SCL 只有主控在驱动，推挽输出边沿陡，不依赖模块上拉电阻；
  *           SDA 需要让从机把它拉低给出 ACK，所以用开漏 + 内部上拉，
  *           同时还能用 IDR 读回电平判断有没有应答。
  *
  *         为什么 LOOP = 8：
  *           F103 跑 72MHz 时软件 I2C 很容易冲到 1MHz 以上，
  *           面包板 + 长杜邦线 + 模块上拉偏弱就会丢包（表现就是屏幕全黑）。
  *           这里把它压到 300kHz 上下，留足余量。
  */
#include "stm32f1xx_hal.h"
#include "OLED.h"
#include "OLED_Font.h"

#define OLED_PORT      GPIOB

/* 正常接法：SCL = PB1，SDA = PB0；两根线插反时用下面一组 */
#define OLED_SCL_PIN_N GPIO_PIN_1
#define OLED_SDA_PIN_N GPIO_PIN_0
#define OLED_SCL_PIN_S GPIO_PIN_0
#define OLED_SDA_PIN_S GPIO_PIN_1

#define OLED_I2C_DELAY_LOOP  8u

static uint8_t s_swap;    /* 0 = SCL=PB1 / SDA=PB0；1 = 两根线对调 */
static uint8_t s_addr;    /* 实际应答的从机地址：0x78 或 0x7A */
static uint8_t s_ready;   /* 1 = 屏幕已经应答过，可以正常收发 */

#define OLED_SCL_PIN()   (s_swap ? OLED_SCL_PIN_S : OLED_SCL_PIN_N)
#define OLED_SDA_PIN()   (s_swap ? OLED_SDA_PIN_S : OLED_SDA_PIN_N)

#define OLED_W_SCL(x)    HAL_GPIO_WritePin(OLED_PORT, OLED_SCL_PIN(), (x) ? GPIO_PIN_SET : GPIO_PIN_RESET)
#define OLED_W_SDA(x)    HAL_GPIO_WritePin(OLED_PORT, OLED_SDA_PIN(), (x) ? GPIO_PIN_SET : GPIO_PIN_RESET)
#define OLED_R_SDA()     (uint8_t)((OLED_PORT->IDR & OLED_SDA_PIN()) ? 1u : 0u)

/**
  * @brief  位延时，决定软件 I2C 的速率
  */
static void OLED_I2C_Delay(void)
{
  volatile uint32_t k;
  for (k = 0u; k < OLED_I2C_DELAY_LOOP; k++)
  {
    __NOP();
  }
}

/**
  * @brief  按当前引脚映射配置 SCL/SDA
  */
static void OLED_I2C_PinsInit(void)
{
  GPIO_InitTypeDef GPIO_InitStructure = {0};

  __HAL_RCC_GPIOB_CLK_ENABLE();

  GPIO_InitStructure.Pin   = OLED_SCL_PIN();
  GPIO_InitStructure.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStructure.Pull  = GPIO_NOPULL;
  GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(OLED_PORT, &GPIO_InitStructure);

  GPIO_InitStructure.Pin   = OLED_SDA_PIN();
  GPIO_InitStructure.Mode  = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStructure.Pull  = GPIO_PULLUP;
  GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(OLED_PORT, &GPIO_InitStructure);

  OLED_W_SCL(1);
  OLED_W_SDA(1);
}

/**
  * @brief  I2C 起始：SCL 高电平期间 SDA 由高变低
  */
static void OLED_I2C_Start(void)
{
  OLED_W_SDA(1);
  OLED_W_SCL(1);
  OLED_I2C_Delay();
  OLED_W_SDA(0);
  OLED_I2C_Delay();
  OLED_W_SCL(0);
  OLED_I2C_Delay();
}

/**
  * @brief  I2C 停止：SCL 高电平期间 SDA 由低变高
  */
static void OLED_I2C_Stop(void)
{
  OLED_W_SDA(0);
  OLED_I2C_Delay();
  OLED_W_SCL(1);
  OLED_I2C_Delay();
  OLED_W_SDA(1);
  OLED_I2C_Delay();
}

/**
  * @brief  发送一个字节
  * @retval 1 = 第 9 个时钟收到 ACK，0 = 无人应答
  */
static uint8_t OLED_I2C_SendByte(uint8_t Byte)
{
  uint8_t i, ack;

  for (i = 0u; i < 8u; i++)
  {
    OLED_W_SDA((Byte & (0x80u >> i)) ? 1 : 0);
    OLED_I2C_Delay();
    OLED_W_SCL(1);
    OLED_I2C_Delay();
    OLED_W_SCL(0);
    OLED_I2C_Delay();
  }

  OLED_W_SDA(1);           /* 释放 SDA，让从机拉低表示应答 */
  OLED_I2C_Delay();
  OLED_W_SCL(1);
  OLED_I2C_Delay();
  ack = (OLED_R_SDA() == 0u) ? 1u : 0u;
  OLED_W_SCL(0);
  OLED_I2C_Delay();

  return ack;
}

/**
  * @brief  地址探测：返回 1 表示该地址有器件应答
  */
static uint8_t OLED_Probe(uint8_t Addr)
{
  uint8_t ack;

  OLED_I2C_Start();
  ack = OLED_I2C_SendByte(Addr);
  OLED_I2C_Stop();

  return ack;
}

/**
  * @brief  上电自检一次：找出「哪根线是 SCL」和「从机地址」
  * @note   有的模块丝印会把 SCL/SDA 标反，或者插线时按颜色接错。
  *         先按正常接法探一次，没人应答就换对调后的引脚再探一次，
  *         探到就记住，探不到就让 s_ready 保持 0（后面所有写操作直接跳过）。
  */
static uint8_t OLED_Detect(void)
{
  static const uint8_t addr_list[2] = {0x78u, 0x7Au};
  uint8_t m, a;

  for (m = 0u; m < 2u; m++)
  {
    s_swap = m;
    OLED_I2C_PinsInit();

    for (a = 0u; a < 2u; a++)
    {
      if (OLED_Probe(addr_list[a]) != 0u)
      {
        s_addr  = addr_list[a];
        s_ready = 1u;
        return 1u;
      }
    }
  }

  s_swap  = 0u;
  s_ready = 0u;
  return 0u;
}

/**
  * @brief  写命令
  */
static void OLED_WriteCmd(uint8_t Command)
{
  if (s_ready == 0u)
  {
    return;
  }

  OLED_I2C_Start();
  OLED_I2C_SendByte(s_addr);
  OLED_I2C_SendByte(0x00u);     /* 控制字节：后面跟的是命令 */
  OLED_I2C_SendByte(Command);
  OLED_I2C_Stop();
}

/**
  * @brief  连续写显存数据（一次事务发完，比逐字节快很多）
  */
static void OLED_WriteData(const uint8_t *Buf, uint16_t Len)
{
  uint16_t i;

  if (s_ready == 0u)
  {
    return;
  }

  OLED_I2C_Start();
  OLED_I2C_SendByte(s_addr);
  OLED_I2C_SendByte(0x40u);     /* 控制字节：后面跟的是显存数据 */
  for (i = 0u; i < Len; i++)
  {
    OLED_I2C_SendByte(Buf[i]);
  }
  OLED_I2C_Stop();
}

/**
  * @brief  设置光标：Y = 页(0~7)，X = 列(0~127)
  */
static void OLED_SetCursor(uint8_t Y, uint8_t X)
{
  OLED_WriteCmd((uint8_t)(0xB0u | (Y & 0x07u)));
  OLED_WriteCmd((uint8_t)(0x10u | ((X & 0xF0u) >> 4)));
  OLED_WriteCmd((uint8_t)(0x00u | (X & 0x0Fu)));
}

/**
  * @brief  屏幕是否可用（上电自检有没有应答）
  */
uint8_t OLED_IsReady(void)
{
  return s_ready;
}

/**
  * @brief  OLED 初始化
  */
void OLED_Init(void)
{
  s_swap  = 0u;
  s_addr  = 0x78u;
  s_ready = 0u;

  if (OLED_Detect() == 0u)
  {
    return;                     /* 没人应答：不初始化，避免死等 */
  }

  OLED_WriteCmd(0xAE);          /* 关显示 */
  OLED_WriteCmd(0xD5);          /* 时钟分频 */
  OLED_WriteCmd(0x80);
  OLED_WriteCmd(0xA8);          /* 复用率 1/64 */
  OLED_WriteCmd(0x3F);
  OLED_WriteCmd(0xD3);          /* 显示偏移 */
  OLED_WriteCmd(0x00);
  OLED_WriteCmd(0x40);          /* 起始行 */
  OLED_WriteCmd(0xA1);          /* 左右方向：正常 */
  OLED_WriteCmd(0xC8);          /* 上下方向：正常 */
  OLED_WriteCmd(0xDA);          /* COM 引脚配置 */
  OLED_WriteCmd(0x12);
  OLED_WriteCmd(0x81);          /* 对比度 */
  OLED_WriteCmd(0xCF);
  OLED_WriteCmd(0xD9);          /* 预充电周期 */
  OLED_WriteCmd(0xF1);
  OLED_WriteCmd(0xDB);          /* VCOMH */
  OLED_WriteCmd(0x30);
  OLED_WriteCmd(0xA4);          /* 按显存显示 */
  OLED_WriteCmd(0xA6);          /* 正常显示（非反色） */
  OLED_WriteCmd(0x8D);          /* 开电荷泵 */
  OLED_WriteCmd(0x14);
  OLED_WriteCmd(0xAF);          /* 开显示 */

  OLED_Clear();
}

/**
  * @brief  清屏
  */
void OLED_Clear(void)
{
  uint8_t zeros[128] = {0};
  uint8_t page;

  for (page = 0u; page < 8u; page++)
  {
    OLED_SetCursor(page, 0u);
    OLED_WriteData(zeros, 128u);
  }
}

/**
  * @brief  显示一个字符
  * @param  Line   行，1~4
  * @param  Column 列，1~16
  */
void OLED_ShowChar(uint8_t Line, uint8_t Column, char Char)
{
  uint8_t idx;

  if ((Line < 1u) || (Line > 4u) || (Column < 1u) || (Column > 16u))
  {
    return;
  }
  if ((Char < ' ') || (Char > '~'))
  {
    Char = ' ';
  }

  idx = (uint8_t)(Char - ' ');

  OLED_SetCursor((uint8_t)((Line - 1u) * 2u), (uint8_t)((Column - 1u) * 8u));
  OLED_WriteData(&OLED_F8x16[idx][0], 8u);          /* 上半部分 */

  OLED_SetCursor((uint8_t)((Line - 1u) * 2u + 1u), (uint8_t)((Column - 1u) * 8u));
  OLED_WriteData(&OLED_F8x16[idx][8], 8u);          /* 下半部分 */
}

/**
  * @brief  显示字符串（ASCII 可见字符）
  * @param  Line   起始行，1~4
  * @param  Column 起始列，1~16
  */
void OLED_ShowString(uint8_t Line, uint8_t Column, const char *String)
{
  uint8_t i = 0u;

  while ((String[i] != '\0') && ((uint8_t)(Column + i) <= 16u))
  {
    OLED_ShowChar(Line, (uint8_t)(Column + i), String[i]);
    i++;
  }
}

/**
  * @brief  显示十进制无符号数
  */
void OLED_ShowNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
  uint8_t i, ch;
  uint32_t pow10;

  for (i = 0u; i < Length; i++)
  {
    pow10 = 1u;
    for (ch = 0u; ch < (uint8_t)(Length - i - 1u); ch++)
    {
      pow10 *= 10u;
    }
    ch = (uint8_t)((Number / pow10) % 10u);
    OLED_ShowChar(Line, (uint8_t)(Column + i), (char)('0' + ch));
  }
}
