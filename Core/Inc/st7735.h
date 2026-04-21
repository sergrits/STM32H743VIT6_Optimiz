#ifndef __ST7735_H__
#define __ST7735_H__

#include "stm32h7xx_hal.h"

// Настройки пинов (Порт E)
#define ST7735_SPI_PORT hspi4
extern SPI_HandleTypeDef ST7735_SPI_PORT;

// Используем имена, которые вы дали в CubeMX (Labels)
// Это свяжет библиотеку с вашим .ioc автоматически
#include "main.h"

#define ST7735_CS_Pin        LCD_CS_Pin
#define ST7735_CS_GPIO_Port  LCD_CS_GPIO_Port
#define ST7735_DC_Pin        LCD_DC_Pin
#define ST7735_DC_GPIO_Port  LCD_DC_GPIO_Port
#define ST7735_BL_Pin        LCD_BL_Pin
#define ST7735_BL_GPIO_Port  LCD_BL_GPIO_Port
#define ST7735_BL_ON()  HAL_GPIO_WritePin(ST7735_BL_GPIO_Port, ST7735_BL_Pin, GPIO_PIN_RESET)
#define ST7735_BL_OFF() HAL_GPIO_WritePin(ST7735_BL_GPIO_Port, ST7735_BL_Pin, GPIO_PIN_SET)

// Пин Reset не используется программно, так как он на NRST
// Закомментируйте или оставьте пустым, чтобы не было конфликта с SPI4_SCK
// #define ST7735_RES_Pin    ...

// Цвета (RGB565)
#define ST7735_BLACK   0x0000
#define ST7735_WHITE   0xFFFF
#define ST7735_RED     0xF800
#define ST7735_GREEN   0x07E0
#define ST7735_BLUE    0x001F
#define ST7735_CYAN    0x07FF
#define ST7735_YELLOW  0xFFE0
#define ST7735_MAGENTA 0xF81F

// Функции
void ST7735_Init(void);
void ST7735_DrawPixel(uint16_t x, uint16_t y, uint16_t color);
void ST7735_FillScreen(uint16_t color);
void ST7735_DrawString(uint16_t x, uint16_t y, const char* str, uint16_t color, uint16_t bg);
void ST7735_DrawStringScaled(uint16_t x, uint16_t y, const char* str, uint16_t color, uint16_t bg, uint8_t scale);
void ST7735_DrawString5x7_Scaled(uint16_t x, uint16_t y, const char *str, uint16_t color, uint16_t bg, uint8_t scale);
void ST7735_StatusInit(uint16_t bg);
void ST7735_StatusSet(const char *text, uint16_t fg, uint16_t bg, uint8_t scale);


#endif
