
/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "st7735.h"
#include <stdio.h>
#include "w25qxx_qspi.h"
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

QSPI_HandleTypeDef hqspi;

SPI_HandleTypeDef hspi4;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim6;

/* USER CODE BEGIN PV */

static uint32_t t_led = 0;//Для блинка

static volatile uint32_t W_THRESH = 7;   // начни с 3..10, иначе будет шум
static volatile uint32_t W_DEADBAND_MS = 10; // минимум 10мс между событиями (записи лога)

// Частота
static uint32_t cnt_prev = 0;
static uint32_t t_prev_ms = 0;
static uint32_t freq_hz = 0;

// Вес (ADC)
#define ADC_BUF_LEN 256
static uint16_t adc_buf[ADC_BUF_LEN];

static volatile uint32_t w_avg = 0;
static volatile uint32_t w_filt = 0;  // integer IIR filter state (no float)
static volatile uint32_t w0 = 0;
static volatile uint32_t events = 0;
static volatile uint32_t adc_half_cb = 0;
static volatile uint32_t adc_full_cb = 0;

// Минимальный и максимальный фиксированный вес
static volatile uint16_t w_min = 0xFFFF;
static volatile uint16_t w_max = 0;

// Структура записи
typedef struct __attribute__((packed)) {
	uint32_t n;        // номер записи
	uint32_t freq_hz;  // частота на момент события
	int16_t  dW;       // дельта веса
	uint8_t  dir;      // 1=up, 0=down
	uint8_t  rsv0;     // выравнивание
	uint32_t t_ms;     // HAL_GetTick() на момент события
} log_rec_t;           // 16 bytes

// Параметры памяти
#define LOG_PAGE_SIZE     256UL
#define LOG_SECTOR_SIZE   4096UL
#define LOG_BASE_ADDR     0x000000UL
#define LOG_TOTAL_BYTES   (8UL * 1024UL * 1024UL)
#define LOG_END_ADDR      (LOG_BASE_ADDR + LOG_TOTAL_BYTES)
#define LOG_TOTAL_KB      (LOG_TOTAL_BYTES / 1024UL)

static uint8_t  log_pages[2][LOG_PAGE_SIZE];
static uint32_t log_pos[2] = {0, 0};

static volatile uint8_t log_wr_idx = 0;        // 0/1
static volatile uint8_t log_ready_mask = 0;    // bit0/page0, bit1/page1

static uint32_t log_write_addr = LOG_BASE_ADDR;
static uint32_t log_next_erase_sector = LOG_BASE_ADDR;

static uint32_t log_used_bytes = 0;
static uint32_t log_rec_no = 0;

static volatile uint8_t log_full = 0;
static volatile uint8_t log_error = 0;

static uint32_t log_last_flush_ms = 0;

static volatile uint32_t log_prog_to = 0;
static volatile uint32_t log_erase_to = 0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI4_Init(void);
static void MX_QUADSPI_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */
static void Log_Append(int16_t dW, uint8_t dir, uint32_t freq_hz_snapshot);
static void process_block(const uint16_t* p, uint32_t n);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */



void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
	if (hadc->Instance == ADC1) {
		adc_half_cb++;
		process_block(&adc_buf[0], ADC_BUF_LEN/2);
	}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	if (hadc->Instance == ADC1) {
		adc_full_cb++;
		process_block(&adc_buf[ADC_BUF_LEN/2], ADC_BUF_LEN/2);
	}
}


static void Freq_Task(uint32_t Tgate_ms)        //Измерение частоты
{
	uint32_t t = HAL_GetTick();
	if ((t - t_prev_ms) < Tgate_ms) return;
	t_prev_ms += Tgate_ms;

	uint32_t cnt_now = __HAL_TIM_GET_COUNTER(&htim2);
	uint32_t delta = cnt_now - cnt_prev;
	cnt_prev = cnt_now;

	freq_hz = (uint32_t)((uint64_t)delta * 1000u / Tgate_ms);
}

static inline uint32_t avg_u16(const uint16_t* p, uint32_t n)
{
	/* Sum fits into 32-bits for n <= 256 (65535*256 = 16776960) -> use uint32_t for faster arithmetic */
	uint32_t s = 0;
	const uint16_t *end = p + n;
	while (p < end) {
		s += *p++;
	}
	return s / n;
}

static void process_block(const uint16_t* p, uint32_t n)
{
	uint32_t a = avg_u16(p, n);
	w_avg = a;

	if (a < w_min) w_min = (uint16_t)a;
	if (a > w_max) w_max = (uint16_t)a;

	/* Integer exponential smoothing: alpha = 1/5 -> w_filt = (4*w_filt + a)/5
	   avoids floating-point math and is sufficiently responsive for this use-case */
	const uint32_t alpha_num = 1U;
	const uint32_t alpha_den = 5U;
	w_filt = ((alpha_den - alpha_num) * w_filt + alpha_num * a) / alpha_den;

	int32_t wf = (int32_t)w_filt;          // filtered value
	int32_t dW = wf - (int32_t)w0;
	uint32_t adW = (dW >= 0) ? (uint32_t)dW : (uint32_t)(-dW);

	static uint32_t t_last_evt = 0;
	uint32_t t = HAL_GetTick();

	if (adW >= W_THRESH && (t - t_last_evt) >= W_DEADBAND_MS)
	{
		t_last_evt = t;

		events++;
		w0 = (uint32_t)wf;

		uint8_t dir = (dW >= 0) ? 1 : 0;
		Log_Append((int16_t)dW, dir, freq_hz);
	}
}



static inline void fmt_freq(char *out, size_t out_sz, uint32_t f_hz)
{
	/* Kept logic but marked inline to allow compiler to optimize away call overhead in tight UI path */
	if (f_hz < 1000000UL)
	{
		uint32_t f_khz10 = (f_hz + 50UL) / 100UL;
		uint32_t khz = f_khz10 / 10UL;
		uint32_t dec = f_khz10 % 10UL;
		snprintf(out, out_sz, "%lu.%lu kHz", (unsigned long)khz, (unsigned long)dec);
	}
	else
	{
		uint32_t mhz = f_hz / 1000000UL;
		uint32_t frac = (f_hz % 1000000UL + 500UL) / 1000UL;
		if (frac >= 1000UL) { mhz++; frac = 0; }
		snprintf(out, out_sz, "%lu.%03lu MHz", (unsigned long)mhz, (unsigned long)frac);
	}
}

/* Lightweight integer -> string helpers to avoid snprintf in UI path */
static inline int u32_to_str(char *buf, uint32_t v)
{
	char tmp[12];
	int p = 0;
	if (v == 0) tmp[p++] = '0';
	while (v != 0 && p < (int)sizeof(tmp)) { tmp[p++] = (char)('0' + (v % 10)); v /= 10; }
	int idx = 0;
	for (int i = p - 1; i >= 0; --i) buf[idx++] = tmp[i];
	buf[idx] = '\0';
	return idx;
}

static inline void u32_to_str_pad(char *out, size_t out_sz, uint32_t v, int width)
{
	char tmp[16];
	int len = u32_to_str(tmp, v);
	int pad = width - len;
	if (pad < 0) pad = 0;
	size_t i = 0;
	while (pad-- > 0 && i + 1 < out_sz) out[i++] = ' ';
	for (int j = 0; j < len && i + 1 < out_sz; ++j) out[i++] = tmp[j];
	out[i] = '\0';
}

// y-координаты под scale=2 (высота строки 16px)
#define UI_Y0  0
#define UI_Y1  20
#define UI_Y2  38
#define UI_Y3  60

static void UI_Task(uint32_t period_ms)
{
	static uint32_t t_prev = 0;
	uint32_t t = HAL_GetTick();
	if ((t - t_prev) < period_ms) return;
	t_prev += period_ms;

	char sL[24];
	char sR[24];
	char s[24];

	// 1) A/E
	sL[0] = 'A'; sL[1] = ':'; sL[2] = '\0';
	u32_to_str_pad(&sL[2], sizeof(sL) - 2, w_avg, 4);
	sR[0] = 'E'; sR[1] = ':'; sR[2] = '\0';
	u32_to_str_pad(&sR[2], sizeof(sR) - 2, events, 4);
	ST7735_DrawString5x7_Scaled(0,   UI_Y0, sL, ST7735_WHITE, ST7735_BLACK, 2);
	ST7735_DrawString5x7_Scaled(84,  UI_Y0, sR, ST7735_WHITE, ST7735_BLACK, 2);

	// 2) Freq
	u32_to_str_pad(s, sizeof(s), freq_hz, 7);
	ST7735_DrawString5x7_Scaled(0,   UI_Y1, "F:", ST7735_CYAN, ST7735_BLACK, 2);
	ST7735_DrawString5x7_Scaled(24,  UI_Y1, s,    ST7735_CYAN, ST7735_BLACK, 2);
	ST7735_DrawString5x7_Scaled(132, UI_Y1, "Hz", ST7735_CYAN, ST7735_BLACK, 2);

	// 3) LOG
	uint32_t used_kb = log_used_bytes / 1024UL;
	uint32_t pct = (LOG_TOTAL_BYTES == 0) ? 0 : (log_used_bytes * 100UL) / LOG_TOTAL_BYTES;

	if (log_error) {
		/* constant small string */
		s[0] = 'L'; s[1] = 'O'; s[2] = 'G'; s[3] = ' '; s[4] = 'E'; s[5] = 'R'; s[6] = 'R'; s[7] = '\0';
	} else if (log_full) {
		char tbuf[24];
		tbuf[0] = '\0';
		u32_to_str(tbuf, used_kb);
		/* build "LOG FULL <n>KB" */
		s[0] = 'L'; s[1] = 'O'; s[2] = 'G'; s[3] = ' '; s[4] = 'F'; s[5] = 'U'; s[6] = 'L'; s[7] = 'L'; s[8] = ' ';
		int p = 9;
		for (int i = 0; tbuf[i] != '\0' && p + 1 < (int)sizeof(s); ++i) s[p++] = tbuf[i];
		if (p + 3 < (int)sizeof(s)) { s[p++] = 'K'; s[p++] = 'B'; }
		s[p] = '\0';
	} else {
		char t1[16]; char t2[16];
		u32_to_str(t1, used_kb);
		u32_to_str(t2, pct);
		/* build "LOG:<used>KB <pct>%" */
		int p = 0;
		const char *hdr = "LOG:";
		for (int i = 0; hdr[i] != '\0' && p + 1 < (int)sizeof(s); ++i) s[p++] = hdr[i];
		for (int i = 0; t1[i] != '\0' && p + 1 < (int)sizeof(s); ++i) s[p++] = t1[i];
		if (p + 3 < (int)sizeof(s)) { s[p++] = 'K'; s[p++] = 'B'; s[p++] = ' '; }
		for (int i = 0; t2[i] != '\0' && p + 1 < (int)sizeof(s); ++i) s[p++] = t2[i];
		if (p + 1 < (int)sizeof(s)) s[p++] = '%';
		s[p] = '\0';
	}
	ST7735_DrawString5x7_Scaled(0, UI_Y2, s, ST7735_YELLOW, ST7735_BLACK, 2);

	// 4) опционально: таймауты
	/* build "TO E:<erase> P:<prog>" */
	char t_e[12]; char t_p[12];
	u32_to_str(t_e, log_erase_to);
	u32_to_str(t_p, log_prog_to);
	int pos = 0;
	const char *hdr = "TO E:";
	for (int i = 0; hdr[i] != '\0' && pos + 1 < (int)sizeof(s); ++i) s[pos++] = hdr[i];
	for (int i = 0; t_e[i] != '\0' && pos + 1 < (int)sizeof(s); ++i) s[pos++] = t_e[i];
	if (pos + 4 < (int)sizeof(s)) { s[pos++] = ' '; s[pos++] = 'P'; s[pos++] = ':'; }
	for (int i = 0; t_p[i] != '\0' && pos + 1 < (int)sizeof(s); ++i) s[pos++] = t_p[i];
	s[pos] = '\0';
	ST7735_DrawString5x7_Scaled(0, UI_Y3, s, ST7735_GREEN, ST7735_BLACK, 2);
}

// функция очистки памяти
static void Log_EraseIfNeeded(uint32_t addr)
{
	/* LOG_SECTOR_SIZE is 4096 (power of two) -> compute sector base with bitmask (faster than divide) */
	uint32_t sector_base = addr & ~(LOG_SECTOR_SIZE - 1U);

	if (log_next_erase_sector <= sector_base)
	{
		uint8_t st = W25qxx_EraseSector(sector_base);

		if (st == w25qxx_TIMEOUT) log_erase_to++;
		if (st != w25qxx_OK) { log_error = 1; return; }

		log_next_erase_sector = sector_base + LOG_SECTOR_SIZE;
	}
}

// Функция записи в память
static void Log_Append(int16_t dW, uint8_t dir, uint32_t freq_hz_snapshot)
{
	if (log_full || log_error) return;

	log_rec_t r;
	r.n = log_rec_no++;
	r.freq_hz = freq_hz_snapshot;
	r.dW = dW;
	r.dir = dir;
	r.rsv0 = 0;
	r.t_ms = HAL_GetTick();

	/* Make modifications atomic because this function may be called from IRQ context
	   (ADC DMA callbacks). We keep the critical section as short as possible. */
	uint32_t prim = __get_PRIMASK();
	__disable_irq();

	uint8_t i = (uint8_t)log_wr_idx;

	/* if current page already ready — switch to the other */
	if (log_ready_mask & (1U << i)) {
		uint8_t j = (uint8_t)(i ^ 1U);
		if (log_ready_mask & (1U << j)) {
			__set_PRIMASK(prim);
			return; // both busy -> drop
		}
		log_wr_idx = j;
		i = j;
	}

	/* if record doesn't fit -> mark current ready and switch */
	if (log_pos[i] + sizeof(r) > LOG_PAGE_SIZE) {
		log_ready_mask |= (1U << i);

		uint8_t j = (uint8_t)(i ^ 1U);
		if (log_ready_mask & (1U << j)) {
			__set_PRIMASK(prim);
			return;
		}
		log_wr_idx = j;
		i = j;

		if (log_pos[i] + sizeof(r) > LOG_PAGE_SIZE) {
			__set_PRIMASK(prim);
			return;
		}
	}

	/* copy record into page and update position */
	memcpy(&log_pages[i][log_pos[i]], &r, sizeof(r));
	log_pos[i] += sizeof(r);

	if (log_pos[i] == LOG_PAGE_SIZE) {
		log_ready_mask |= (1U << i);
		log_wr_idx ^= 1U;
	}

	__set_PRIMASK(prim);
}

static uint8_t Log_EraseSector_Safe(uint32_t addr)
{
	uint8_t st = W25qxx_EraseSector(addr);
	if (st != w25qxx_OK) log_error = 1;
	return st;
}

static uint8_t Log_PageProgram_Safe(uint8_t *buf, uint32_t addr, uint32_t sz)
{
	uint8_t st = W25qxx_PageProgram(buf, addr, sz);
	if (st == w25qxx_TIMEOUT) log_prog_to++;
	if (st != w25qxx_OK) log_error = 1;
	return st;
}

// Пометить страницу "готова к записи" из IRQ (очень коротко)
static inline void Log_MarkPageReadyFromISR(uint8_t page_idx)
{
	// атомарно выставляем бит
	uint32_t primask = __get_PRIMASK();
	__disable_irq();
	log_ready_mask |= (1U << page_idx);
	__set_PRIMASK(primask);
}

// Задача сброса готовых страниц во Flash.
// Вызывать только из main while(1), НЕ из IRQ.
static void Log_FlushTask(void)
{
	if (log_full || log_error) return;

	// Если дошли до конца области лога — считаем память заполненной
	if (log_write_addr + LOG_PAGE_SIZE > LOG_END_ADDR) {
		log_full = 1;
		return;
	}

	// Снимем маску готовности атомарно, чтобы не спорить с IRQ
	uint8_t ready;
	__disable_irq();
	ready = log_ready_mask;
	__enable_irq();

	if (!ready) return;

	// Пишем по одной странице за вызов (чтобы не "подвешивать" UI надолго)
	uint8_t page = (ready & 0x01) ? 0 : 1;

	// Перед тем как писать страницу page, гарантируем что она реально готова
	// и больше не будет дописываться (бит ready уже выставлен).
	// Адрес под запись берём общий log_write_addr (последовательная запись).
	Log_EraseIfNeeded(log_write_addr);
	if (log_error) return;

	// Пишем ровно 256 байт (как задано LOG_PAGE_SIZE)
	if (Log_PageProgram_Safe(log_pages[page], log_write_addr, LOG_PAGE_SIZE) != w25qxx_OK) {
		// log_error уже выставится внутри Safe-функции
		return;
	}

	// Успешно записали страницу -> обновляем счётчики
	log_write_addr += LOG_PAGE_SIZE;
	log_used_bytes += LOG_PAGE_SIZE;

	// Освобождаем страницу для дальнейших записей: делаем это атомарно
	__disable_irq();
	log_pos[page] = 0;
	log_ready_mask &= (uint8_t)~(1U << page);
	__enable_irq();
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

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

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
  MX_DMA_Init();
  MX_SPI4_Init();
  MX_QUADSPI_Init();
  MX_TIM2_Init();
  MX_ADC1_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */

  // 1) Запускаем TIM2 (счётчик импульсов)
	HAL_TIM_Base_Start(&htim2);

	// 2) Запускаем TIM6 (даёт TRGO для ADC)
	HAL_TIM_Base_Start(&htim6);

	// 3) Запускаем ADC1 по триггеру TIM6 + DMA circular
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buf, ADC_BUF_LEN);

  //  HAL_GPIO_WritePin(GPIOE, LCD_BL_Pin, GPIO_PIN_RESET); // подсветка ON (active-low)

	HAL_Delay(100);
	w0 = w_avg;   //не будет “всплеска” событий при старте.

	ST7735_Init();
	ST7735_FillScreen(ST7735_BLACK);// Очищаем весь экран

	w25qxx_Init();

	log_used_bytes = 0;
	log_rec_no = 0;

	log_write_addr = LOG_BASE_ADDR;
	log_next_erase_sector = LOG_BASE_ADDR;

	log_pos[0] = 0;
	log_pos[1] = 0;

	log_wr_idx = 0;
	log_ready_mask = 0;

	log_full = 0;
	log_error = 0;

	log_last_flush_ms = HAL_GetTick();



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */



	while (1)
	{
		uint32_t now = HAL_GetTick();

		// flush не чаще чем раз в 50мс
		if (!log_full && !log_error && (now - log_last_flush_ms) >= 500)
		{
			log_last_flush_ms = now;

			__disable_irq();
			uint8_t i = (uint8_t)log_wr_idx;
			if (!(log_ready_mask & (1U << i)) && log_pos[i] > 0) {
				log_ready_mask |= (1U << i);
			} else {
				uint8_t j = (uint8_t)(i ^ 1U);
				if (!(log_ready_mask & (1U << j)) && log_pos[j] > 0) {
					log_ready_mask |= (1U << j);
				}
			}
			__enable_irq();
		}

		// 1) Сброс готовых страниц во Flash
		Log_FlushTask();

		// 2) UI/измерения
		Freq_Task(20);
		UI_Task(100);


   // БЛИНК
		if (HAL_GetTick() - t_led >= 250) {
			t_led = HAL_GetTick();
			HAL_GPIO_TogglePin(GPIOE, LED_HEARTBEAT_Pin);
		}
	/* USER CODE END WHILE */

	/* USER CODE BEGIN 3 */
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 60;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
	Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
							  |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
							  |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
	Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T6_TRGO;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_CIRCULAR;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc1.Init.OversamplingMode = DISABLE;
  hadc1.Init.Oversampling.Ratio = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
	Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
	Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
	Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief QUADSPI Initialization Function
  * @param None
  * @retval None
  */
static void MX_QUADSPI_Init(void)
{

  /* USER CODE BEGIN QUADSPI_Init 0 */

  /* USER CODE END QUADSPI_Init 0 */

  /* USER CODE BEGIN QUADSPI_Init 1 */

  /* USER CODE END QUADSPI_Init 1 */
  /* QUADSPI parameter configuration*/
  hqspi.Instance = QUADSPI;
  hqspi.Init.ClockPrescaler = 16;
  hqspi.Init.FifoThreshold = 1;
  hqspi.Init.SampleShifting = QSPI_SAMPLE_SHIFTING_HALFCYCLE;
  hqspi.Init.FlashSize = 23;
  hqspi.Init.ChipSelectHighTime = QSPI_CS_HIGH_TIME_8_CYCLE;
  hqspi.Init.ClockMode = QSPI_CLOCK_MODE_0;
  hqspi.Init.FlashID = QSPI_FLASH_ID_1;
  hqspi.Init.DualFlash = QSPI_DUALFLASH_DISABLE;
  if (HAL_QSPI_Init(&hqspi) != HAL_OK)
  {
	Error_Handler();
  }
  /* USER CODE BEGIN QUADSPI_Init 2 */

  /* USER CODE END QUADSPI_Init 2 */

}

/**
  * @brief SPI4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI4_Init(void)
{

  /* USER CODE BEGIN SPI4_Init 0 */

  /* USER CODE END SPI4_Init 0 */

  /* USER CODE BEGIN SPI4_Init 1 */

  /* USER CODE END SPI4_Init 1 */
  /* SPI4 parameter configuration*/
  hspi4.Instance = SPI4;
  hspi4.Init.Mode = SPI_MODE_MASTER;
  hspi4.Init.Direction = SPI_DIRECTION_2LINES;
  hspi4.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi4.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi4.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi4.Init.NSS = SPI_NSS_SOFT;
  hspi4.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi4.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi4.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi4.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi4.Init.CRCPolynomial = 0x0;
  hspi4.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi4.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi4.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi4.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi4.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi4.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi4.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi4.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi4.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi4.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi4) != HAL_OK)
  {
	Error_Handler();
  }
  /* USER CODE BEGIN SPI4_Init 2 */

  /* USER CODE END SPI4_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
	Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_EXTERNAL1;
  sSlaveConfig.InputTrigger = TIM_TS_TI1FP1;
  sSlaveConfig.TriggerPolarity = TIM_TRIGGERPOLARITY_RISING;
  sSlaveConfig.TriggerFilter = 0;
  if (HAL_TIM_SlaveConfigSynchro(&htim2, &sSlaveConfig) != HAL_OK)
  {
	Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
	Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 5999;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 8;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
	Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
	Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, LED_HEARTBEAT_Pin|LCD_BL_Pin|LCD_CS_Pin|LCD_DC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED_HEARTBEAT_Pin LCD_BL_Pin */
  GPIO_InitStruct.Pin = LED_HEARTBEAT_Pin|LCD_BL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_CS_Pin LCD_DC_Pin */
  GPIO_InitStruct.Pin = LCD_CS_Pin|LCD_DC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
	 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
