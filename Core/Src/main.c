/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    main.c
  * @brief   电控组第三次培训作业
  *          串口 + 按键控制三灯流水灯，OLED 显示状态，串口中断回传状态
  ******************************************************************************
  *  接线（完整接线表见 README.md）
  *    LED1 / LED2 / LED3   -> PA0 / PA1 / PA2     高电平点亮，每颗串 220R 到 GND
  *    按键 A（换向）        -> PA11                内部上拉，另一端接 GND，EXTI11 下降沿
  *    按键 B（暂停/继续）   -> PA4                 内部上拉，另一端接 GND，EXTI4  下降沿
  *    OLED SCL / SDA       -> PB1 / PB0           软件 I2C，见 OLED.c
  *    USART1 TX / RX       -> PA9 / PA10          115200 8-N-1，接 USB-TTL
  *
  *  程序结构（作业要求：中断只接收/记事件，解析与状态更新放主循环）
  *    中断  HAL_GPIO_EXTI_Callback  : 只记下"这个键出现了下降沿"
  *    中断  HAL_UART_RxCpltCallback : 只把 1 个字节塞进环形缓冲并重新武装
  *    主循环 Key_Scan / Rx_Task / Oled_Refresh / Fb_Send :
  *          消抖判稳、拼包校验、更新状态、刷屏、回传
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "OLED.h"
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* 串口命令包：AA | RUN | DIR | STEP_L | STEP_H | CHECK
     CHECK = (0xAA + RUN + DIR + STEP_L + STEP_H) % 256
   F103 是小端，step_ms 直接摆进结构体就是"低字节在前"。
   packed 必须加：不打包的话 uint16_t 前面会被塞一个填充字节，
   结构体变成 8 字节，和 6 字节的包永远对不上。 */
typedef struct __attribute__((packed)) {
  uint8_t  head;      /* 固定 0xAA */
  uint8_t  run;       /* 0 = 暂停，1 = 运行 */
  uint8_t  dir;       /* 0 = 正向，1 = 反向 */
  uint16_t step_ms;   /* 切换间隔 100 ~ 2000 ms，低字节在前 */
  uint8_t  check;     /* 前 5 个字节之和的低 8 位 */
} CmdPacket;

/* 一个按键的运行时状态 */
typedef struct {
  GPIO_TypeDef    *port;
  uint16_t         pin;
  volatile uint8_t edge;        /* 中断置 1：这一路出现了下降沿 */
  uint32_t         edge_ms;     /* 下降沿出现的时刻 */
  uint8_t          confirming;  /* 1 = 正在 20ms 复检窗口里 */
  uint8_t          locked;      /* 1 = 这次按下已经处理过，等松手 */
  uint32_t         release_ms;  /* 松手计时的起点 */
} KeyState;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LED_COUNT        3u
#define STEP_MIN_MS      100u
#define STEP_MAX_MS      2000u

#define KEY_DEBOUNCE_MS  20u    /* 消抖窗口：按下 20ms 后再读一次，松手也要稳定 20ms */

#define RX_RING_SIZE     32u    /* 接收环形缓冲：中断写、主循环读 */
#define PKT_HEAD         0xAAu
#define PKT_LEN          6u

#define FB_BUF_SIZE      64u
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* ---------- 唯一的一份运行状态：按键、串口、OLED、反馈都读它 ---------- */
static uint8_t  g_run       = 1u;     /* 0 = 暂停，1 = 运行 */
static uint8_t  g_dir       = 0u;     /* 0 = 正向，1 = 反向 */
static uint16_t g_step_ms   = 300u;   /* 相邻 LED 切换间隔 */
static uint8_t  g_led       = 0u;     /* 当前点亮的灯 0/1/2，显示成 LED:1/2/3 */
static uint32_t g_step_tick = 0u;     /* 上一次切换灯的时刻 */

/* 灯表：哪颗灯接在哪个脚 */
static const struct { GPIO_TypeDef *port; uint16_t pin; } LED_PIN[LED_COUNT] = {
  { GPIOA, GPIO_PIN_0 },
  { GPIOA, GPIO_PIN_1 },
  { GPIOA, GPIO_PIN_2 },
};

/* 两个按键 */
static KeyState key_dir = { GPIOA, GPIO_PIN_11, 0u, 0u, 0u, 0u, 0u };  /* 换向 */
static KeyState key_run = { GPIOA, GPIO_PIN_4,  0u, 0u, 0u, 0u, 0u };  /* 暂停/继续 */

/* 串口接收 */
static uint8_t          rx_byte;                       /* 中断每次都收进这个字节 */
static uint8_t          rx_ring[RX_RING_SIZE];
static volatile uint8_t rx_w = 0u;                     /* 中断写指针 */
static volatile uint8_t rx_r = 0u;                     /* 主循环读指针 */
static uint8_t          rx_frame[PKT_LEN];             /* 正在拼的一包 */
static uint8_t          rx_len = 0u;

/* 串口反馈：一行发完之前不能被改写 */
static char             fb_buf[FB_BUF_SIZE];
static volatile uint8_t tx_busy    = 0u;               /* 1 = 上一行还没发完 */
static volatile uint8_t fb_pending = 0u;               /* 1 = 状态变了，需要回一行 */

/* OLED 差异刷新缓存：初值给不可能出现的值，保证第一圈全部写一遍 */
static uint8_t  oled_run  = 0xFFu;
static uint8_t  oled_dir  = 0xFFu;
static uint16_t oled_step = 0xFFFFu;
static uint8_t  oled_led  = 0xFFu;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void App_Gpio_Init(void);
static void Led_AllOff(void);
static void Led_Refresh(void);
static uint8_t Key_Scan(KeyState *k, uint32_t now);
static void Rx_Task(void);
static void Oled_Refresh(void);
static void Fb_Request(void);
static void Fb_Send(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* ---------------- 灯 ---------------- */
static void Led_Write(uint8_t idx, uint8_t on)
{
  HAL_GPIO_WritePin(LED_PIN[idx].port, LED_PIN[idx].pin,
                    on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void Led_AllOff(void)
{
  for (uint8_t i = 0u; i < LED_COUNT; i++) {
    Led_Write(i, 0u);
  }
}

/* 同一时刻只亮一颗：按 g_led 刷新全部三颗 */
static void Led_Refresh(void)
{
  for (uint8_t i = 0u; i < LED_COUNT; i++) {
    Led_Write(i, (uint8_t)(i == g_led));
  }
}

/* ---------------- 按键 ----------------
   中断只做一件事：记下"这个脚出现了下降沿"。
   主循环拿到这个事件后，等 20ms 再读一次电平：
     - 20ms 后还是低电平 → 确认真的按下，执行一次动作，然后上锁；
     - 上锁后必须等到引脚回到高电平并稳定 20ms 才解锁。
   所以一次物理按下只会算一次，抖动和长按都不会重复触发。 */
static uint8_t Key_IsDown(const KeyState *k)
{
  return (HAL_GPIO_ReadPin(k->port, k->pin) == GPIO_PIN_RESET) ? 1u : 0u;
}

static uint8_t Key_Scan(KeyState *k, uint32_t now)
{
  /* 1) 中断报来一个下降沿：打开 20ms 复检窗口 */
  if (k->edge) {
    k->edge       = 0u;
    k->edge_ms    = now;
    k->confirming = 1u;
  }

  /* 2) 20ms 后复检：仍然是低电平才算一次有效按下 */
  if (k->confirming && (uint32_t)(now - k->edge_ms) >= KEY_DEBOUNCE_MS) {
    k->confirming = 0u;
    if (Key_IsDown(k) && !k->locked) {
      k->locked     = 1u;
      k->release_ms = now;
      return 1u;
    }
  }

  /* 3) 松手：连续 20ms 高电平才解锁，按住不放不会连发 */
  if (k->locked) {
    if (Key_IsDown(k)) {
      k->release_ms = now;
    } else if ((uint32_t)(now - k->release_ms) >= KEY_DEBOUNCE_MS) {
      k->locked = 0u;
    }
  }
  return 0u;
}

/* ---------------- 串口接收：拼包 + 校验 ----------------
   中断把字节搬进环形缓冲，这里按帧头同步拼包。
   校验不过就丢掉这一包；同时在后 5 个字节里再找一次 0xAA 当新帧头，
   这样上位机多发/少发一个字节之后，后面不会一直错位。 */
static uint8_t Rx_Pop(uint8_t *b)
{
  if (rx_r == rx_w) {
    return 0u;
  }
  *b   = rx_ring[rx_r];
  rx_r = (uint8_t)((rx_r + 1u) % RX_RING_SIZE);
  return 1u;
}

static uint8_t Pkt_IsValid(const uint8_t *raw)
{
  if (raw[0] != PKT_HEAD) {                                   /* 帧头错 */
    return 0u;
  }
  uint8_t sum = (uint8_t)(raw[0] + raw[1] + raw[2] + raw[3] + raw[4]);
  if (sum != raw[5]) {                                        /* 校验和错 */
    return 0u;
  }
  if (raw[1] > 1u || raw[2] > 1u) {                           /* RUN/DIR 只有 0/1 */
    return 0u;
  }
  uint16_t step = (uint16_t)(raw[3] | ((uint16_t)raw[4] << 8));
  if (step < STEP_MIN_MS || step > STEP_MAX_MS) {             /* 间隔超范围 */
    return 0u;
  }
  return 1u;
}

static void Rx_Task(void)
{
  uint8_t b;

  while (Rx_Pop(&b)) {
    if (rx_len == 0u && b != PKT_HEAD) {                      /* 还没进帧：只认帧头 */
      continue;
    }
    rx_frame[rx_len++] = b;
    if (rx_len < PKT_LEN) {
      continue;
    }

    if (Pkt_IsValid(rx_frame)) {
      CmdPacket pkt;
      memcpy(&pkt, rx_frame, PKT_LEN);                        /* 结构体解析 */
      g_run       = pkt.run;
      g_dir       = pkt.dir;
      g_step_ms   = pkt.step_ms;
      g_step_tick = HAL_GetTick();                            /* 间隔变了，从现在重新计时 */
      Fb_Request();                                           /* 校验通过并应用后回传 */
      rx_len = 0u;
    } else {
      uint8_t i = 1u;
      while (i < PKT_LEN && rx_frame[i] != PKT_HEAD) {
        i++;
      }
      rx_len = (uint8_t)(PKT_LEN - i);
      memmove(rx_frame, rx_frame + i, rx_len);
    }
  }
}

/* ---------------- OLED：哪一行变了才写哪一行 ---------------- */
static void Oled_Line(uint8_t line, const char *text)
{
  char buf[17];
  snprintf(buf, sizeof(buf), "%-16s", text);   /* 补空格，擦掉上一次更长的内容 */
  OLED_ShowString(line, 1u, buf);
}

static void Oled_Refresh(void)
{
  char buf[17];

  if (g_run != oled_run) {
    oled_run = g_run;
    snprintf(buf, sizeof(buf), "STATE: %s", g_run ? "RUN" : "PAUSE");
    Oled_Line(1u, buf);
  }
  if (g_dir != oled_dir) {
    oled_dir = g_dir;
    snprintf(buf, sizeof(buf), "DIR: %s", g_dir ? "REV" : "FWD");
    Oled_Line(2u, buf);
  }
  if (g_step_ms != oled_step) {
    oled_step = g_step_ms;
    snprintf(buf, sizeof(buf), "STEP: %ums", (unsigned)g_step_ms);
    Oled_Line(3u, buf);
  }
  if (g_led != oled_led) {
    oled_led = g_led;
    snprintf(buf, sizeof(buf), "LED: %u", (unsigned)(g_led + 1u));
    Oled_Line(4u, buf);
  }
}

/* ---------------- 串口反馈 ---------------- */
static void Fb_Request(void)
{
  fb_pending = 1u;
}

static void Fb_Send(void)
{
  if (tx_busy || !fb_pending) {
    return;
  }

  /* 反馈格式：STATE=RUN,DIR=FWD,STEP=300,LED=1 */
  int n = snprintf(fb_buf, sizeof(fb_buf), "STATE=%s,DIR=%s,STEP=%u,LED=%u\r\n",
                   g_run ? "RUN" : "PAUSE",
                   g_dir ? "REV" : "FWD",
                   (unsigned)g_step_ms,
                   (unsigned)(g_led + 1u));
  if (n <= 0) {
    fb_pending = 0u;
    return;
  }

  fb_pending = 0u;
  if (HAL_UART_Transmit_IT(&huart1, (uint8_t *)fb_buf, (uint16_t)n) == HAL_OK) {
    tx_busy = 1u;                       /* 发完在 HAL_UART_TxCpltCallback 里清掉 */
  } else {
    fb_pending = 1u;                    /* 串口还忙，下一圈再发 */
  }
}

/* ---------------- 引脚初始化 ----------------
   PA0/PA1/PA2 和 PA4 这几颗脚没有在 CubeMX 里画出来（.ioc 里只管了
   USART1、PA11、PC13），所以在这里补齐；写法和 gpio.c 里 CubeMX 生成的一样。 */
static void App_Gpio_Init(void)
{
  GPIO_InitTypeDef io = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();

  /* LED1~LED3：推挽输出，默认全灭 */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2, GPIO_PIN_RESET);
  io.Pin   = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2;
  io.Mode  = GPIO_MODE_OUTPUT_PP;
  io.Pull  = GPIO_NOPULL;
  io.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &io);

  /* 按键 B：PA4 上拉输入 + 下降沿中断（EXTI4） */
  io.Pin  = GPIO_PIN_4;
  io.Mode = GPIO_MODE_IT_FALLING;
  io.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &io);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 2u, 0u);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  App_Gpio_Init();                       /* PA0/1/2 灯、PA4 按键 */

  /* 上电自检：三灯一起闪两下，说明程序跑起来了 */
  for (uint8_t i = 0u; i < 2u; i++) {
    for (uint8_t k = 0u; k < LED_COUNT; k++) {
      Led_Write(k, 1u);
    }
    HAL_Delay(150u);
    Led_AllOff();
    HAL_Delay(150u);
  }
  Led_Refresh();                         /* 默认点亮 LED1 */

  OLED_Init();
  if (OLED_IsReady() == 0u) {
    /* 屏幕自检没应答：串口打一行，用来区分「接线/供电问题」和「程序没跑起来」 */
    const uint8_t oled_err[] = "OLED:NOACK\r\n";
    HAL_UART_Transmit(&huart1, (uint8_t *)oled_err, sizeof(oled_err) - 1u, 50u);
  }
  Oled_Refresh();                        /* 第一屏：四行全写 */

  HAL_UART_Receive_IT(&huart1, &rx_byte, 1u);   /* 收 1 字节进中断，收到后回调里重新武装 */
  g_step_tick = HAL_GetTick();
  Fb_Request();                          /* 上电先回一行，方便确认串口是通的 */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    uint32_t now = HAL_GetTick();

    /* 1) 按键：确认一次按下只执行一次 */
    if (Key_Scan(&key_dir, now)) {
      g_dir ^= 1u;                       /* 换向 */
      Fb_Request();
    }
    if (Key_Scan(&key_run, now)) {
      g_run ^= 1u;                       /* 暂停 / 继续，灯保持当前那颗 */
      g_step_tick = now;                 /* 继续时从现在重新计时 */
      Fb_Request();
    }

    /* 2) 串口：拼包 → 校验 → 应用 */
    Rx_Task();

    /* 3) 流水灯：到点走一步；暂停时什么都不做，灯保持亮着 */
    if (g_run && (uint32_t)(now - g_step_tick) >= g_step_ms) {
      g_step_tick = now;
      g_led = (uint8_t)((g_dir == 0u) ? ((g_led + 1u) % LED_COUNT)
                                      : ((g_led + LED_COUNT - 1u) % LED_COUNT));
      Led_Refresh();
    }

    /* 4) OLED 与串口反馈 */
    Oled_Refresh();
    Fb_Send();
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/**
  * @brief 按键中断：只记下"出现了下降沿"，判稳全部交给主循环 Key_Scan
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == GPIO_PIN_11) {         /* 按键 A：换向 */
    key_dir.edge = 1u;
  } else if (GPIO_Pin == GPIO_PIN_4) {   /* 按键 B：暂停/继续 */
    key_run.edge = 1u;
  }
}

/**
  * @brief 串口接收完成（每收 1 个字节进一次）：塞进环形缓冲并立刻重新武装
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1) {
    uint8_t next = (uint8_t)((rx_w + 1u) % RX_RING_SIZE);
    if (next != rx_r) {                  /* 缓冲满了就丢新字节，不覆盖没读走的数据 */
      rx_ring[rx_w] = rx_byte;
      rx_w = next;
    }
    HAL_UART_Receive_IT(&huart1, &rx_byte, 1u);
  }
}

/**
  * @brief 串口错误（溢出/帧错）：清标志并把接收重新拉起来，别让串口卡死
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1) {
    __HAL_UART_CLEAR_OREFLAG(huart);
    huart->ErrorCode = HAL_UART_ERROR_NONE;
    HAL_UART_Receive_IT(&huart1, &rx_byte, 1u);
  }
}

/**
  * @brief 一行反馈发完：允许发下一条
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1) {
    tx_busy = 0u;
  }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
