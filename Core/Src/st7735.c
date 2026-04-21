#include "st7735.h"
#include "font5x7.h"

// Команды контроллера
#define ST7735_SWRESET 0x01
#define ST7735_SLPOUT  0x11
#define ST7735_COLMOD  0x3A
#define ST7735_MADCTL  0x36
#define ST7735_DISPON  0x29
#define ST7735_CASET   0x2A
#define ST7735_RASET   0x2B
#define ST7735_RAMWR   0x2C
#define ST7735_INVON   0x21
#define ST7735_NORON   0x13

// Статусная строка: верх экрана
#define STATUS_X      0
#define STATUS_Y      0
#define STATUS_W      160

// Высота зависит от scale: 8*scale
static uint8_t g_status_scale = 1;
static uint16_t g_status_bg = ST7735_BLACK;

static uint8_t ST7735_XSTART = 1;
static uint8_t ST7735_YSTART = 26;

// Попробуем портретную ориентацию как в BSP
// (если у тебя дисплей физически стоит иначе — потом поменяем MADCTL)
//static uint8_t ST7735_MADCTL_VALUE = 0xC0; // начнём с этого (похоже на portrait)
//И параллельно перебрать из набора:0x00, 0x60, 0xA0, 0xC0, 0xE0, 0x20, 0x40, 0x80
static uint8_t ST7735_MADCTL_VALUE = 0xA8; // заливка сверху-вниз, если кнопки сверху

static void ST7735_Select(void) {
    HAL_GPIO_WritePin(ST7735_CS_GPIO_Port, ST7735_CS_Pin, GPIO_PIN_RESET);
}

static void ST7735_Unselect(void) {
    HAL_GPIO_WritePin(ST7735_CS_GPIO_Port, ST7735_CS_Pin, GPIO_PIN_SET);
}

static void ST7735_WriteCommand(uint8_t cmd) {
    HAL_GPIO_WritePin(ST7735_DC_GPIO_Port, ST7735_DC_Pin, GPIO_PIN_RESET);
    ST7735_Select();
    HAL_SPI_Transmit(&ST7735_SPI_PORT, &cmd, 1, HAL_MAX_DELAY);
    ST7735_Unselect();
}

static void ST7735_WriteData(const uint8_t *buff, size_t buff_size) {
    HAL_GPIO_WritePin(ST7735_DC_GPIO_Port, ST7735_DC_Pin, GPIO_PIN_SET);
    ST7735_Select();
    HAL_SPI_Transmit(&ST7735_SPI_PORT, (uint8_t*)buff, buff_size, HAL_MAX_DELAY);
    ST7735_Unselect();
}

static void ST7735_WriteDataByte(uint8_t data) {
    ST7735_WriteData(&data, 1);
}

void ST7735_Init(void) {
    // Подсветка (для твоей платы active-low уже поправлено ранее)
    // Если ты оставил active-low:
    HAL_GPIO_WritePin(ST7735_BL_GPIO_Port, ST7735_BL_Pin, GPIO_PIN_RESET);

    // Держим CS в неактивном состоянии по умолчанию
    ST7735_Unselect();

    ST7735_WriteCommand(ST7735_SWRESET);
    HAL_Delay(150);

    ST7735_WriteCommand(ST7735_SLPOUT);
    HAL_Delay(150);

    // 16-bit color
    ST7735_WriteCommand(ST7735_COLMOD);
    ST7735_WriteDataByte(0x05);
    HAL_Delay(10);

    // Поворот/порядок байт/ориентация — часто критично
    ST7735_WriteCommand(ST7735_MADCTL);
    // Попробуй 0xC8 или 0xA8 или 0x08 — зависит от конкретной матрицы
    ST7735_WriteDataByte(0xC8);

    // Инверсия для многих 0.96" ST7735 нужна
    ST7735_WriteCommand(ST7735_INVON);

    ST7735_WriteCommand(ST7735_NORON);
    HAL_Delay(10);

    ST7735_WriteCommand(ST7735_DISPON);
    HAL_Delay(100);

    ST7735_WriteCommand(ST7735_MADCTL);
    ST7735_WriteDataByte(ST7735_MADCTL_VALUE);
    HAL_Delay(10);
}

// Эти смещения надо будет подобрать под твою ревизию (см. пункт B)

void ST7735_SetAddressWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {

    ST7735_WriteCommand(ST7735_CASET);
	uint8_t data[]  = { 0x00, x0 + ST7735_XSTART, 0x00, x1 + ST7735_XSTART };
    ST7735_WriteData(data, 4);

    ST7735_WriteCommand(ST7735_RASET);
	uint8_t data2[] = { 0x00, y0 + ST7735_YSTART, 0x00, y1 + ST7735_YSTART };
    ST7735_WriteData(data2, 4);

    ST7735_WriteCommand(ST7735_RAMWR);
}

void ST7735_FillScreen(uint16_t color) {
    // Для 0.96" это обычно 160x80, но адресация может отличаться
    ST7735_SetAddressWindow(0, 0, 159, 79);

    uint8_t hi = color >> 8;
    uint8_t lo = color & 0xFF;

    HAL_GPIO_WritePin(ST7735_DC_GPIO_Port, ST7735_DC_Pin, GPIO_PIN_SET);
    ST7735_Select();

    // Отправляем много пикселей: 160*80*2 байта
    for (int i = 0; i < 160 * 80; i++) {
        uint8_t px[2] = { hi, lo };
        HAL_SPI_Transmit(&ST7735_SPI_PORT, px, 2, HAL_MAX_DELAY);
    }

    ST7735_Unselect();
}

void ST7735_DrawPixel(uint16_t x, uint16_t y, uint16_t color) {
    if (x >= 160 || y >= 80) return;
    ST7735_SetAddressWindow(x, y, x, y);
    uint8_t data[] = { (uint8_t)(color >> 8), (uint8_t)(color & 0xFF) };
    ST7735_WriteData(data, 2);
}



static void ST7735_FillRectFast(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color)
{
    if (x >= 160 || y >= 80) return;
    if (x + w > 160) w = 160 - x;
    if (y + h > 80)  h = 80  - y;
    if (w == 0 || h == 0) return;

    ST7735_SetAddressWindow(x, y, x + w - 1, y + h - 1);

    uint8_t hi = (uint8_t)(color >> 8);
    uint8_t lo = (uint8_t)(color & 0xFF);

    uint8_t buf[64]; // 32 пикселя за раз
    for (uint32_t i = 0; i < sizeof(buf); i += 2) { buf[i] = hi; buf[i + 1] = lo; }

    HAL_GPIO_WritePin(ST7735_DC_GPIO_Port, ST7735_DC_Pin, GPIO_PIN_SET);
    ST7735_Select();

    uint32_t pixels = (uint32_t)w * (uint32_t)h;
    while (pixels)
    {
        uint32_t chunk_px = pixels;
        if (chunk_px > (sizeof(buf) / 2)) chunk_px = (sizeof(buf) / 2);

        HAL_SPI_Transmit(&ST7735_SPI_PORT, buf, (uint16_t)(chunk_px * 2), HAL_MAX_DELAY);
        pixels -= chunk_px;
    }

    ST7735_Unselect();
}


static void ST7735_DrawChar5x7_Scaled(uint16_t x, uint16_t y, char ch,
                                      uint16_t color, uint16_t bg, uint8_t scale)
{
    if (scale < 1) scale = 1;
    if (ch < 0x20 || ch > 0x7F) ch = '?';

    const uint8_t *glyph = Font5x7[(uint8_t)ch - 0x20];

    // фон под символ (6x8 с интервалом)
    ST7735_FillRectFast(x, y, (uint16_t)(6 * scale), (uint16_t)(8 * scale), bg);

    // рисуем 5x7
    for (uint8_t col = 0; col < 5; col++)
    {
        uint8_t bits = glyph[col];
        for (uint8_t row = 0; row < 7; row++)
        {
            if (bits & (1U << row))
            {
                ST7735_FillRectFast((uint16_t)(x + col * scale),
                                    (uint16_t)(y + row * scale),
                                    scale, scale, color);
            }
        }
    }
}

void ST7735_DrawString5x7_Scaled(uint16_t x, uint16_t y, const char *str,
                                 uint16_t color, uint16_t bg, uint8_t scale)
{
    uint16_t start_x = x;
    uint16_t cw = (uint16_t)(6 * scale);
    uint16_t ch = (uint16_t)(8 * scale);

    while (*str)
    {
        if (*str == '\n')
        {
            x = start_x;
            y = (uint16_t)(y + ch);
        }
        else
        {
            ST7735_DrawChar5x7_Scaled(x, y, *str, color, bg, scale);
            x = (uint16_t)(x + cw);

            if (x + cw > 160)
            {
                x = start_x;
                y = (uint16_t)(y + ch);
            }
        }

        if (y + ch > 80) break;
        str++;
    }
}

void ST7735_StatusInit(uint16_t bg)
{
    g_status_bg = bg;
    // по умолчанию scale=1 (можешь потом менять через StatusSet)
    ST7735_FillRectFast(STATUS_X, STATUS_Y, STATUS_W, 8, bg);
}

void ST7735_StatusSet(const char *text, uint16_t fg, uint16_t bg, uint8_t scale)
{
    if (scale < 1) scale = 1;
    g_status_scale = scale;
    g_status_bg = bg;

    uint16_t h = (uint16_t)(8U * scale);

    // очистить всю область статусной строки
    ST7735_FillRectFast(STATUS_X, STATUS_Y, STATUS_W, h, bg);

    // вывести текст с небольшим отступом
    ST7735_DrawString5x7_Scaled(0, 0, text, fg, bg, scale);
}








