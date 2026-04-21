#ifndef FONT5X7_H
#define FONT5X7_H

#include <stdint.h>

/*
  font5x7:
  - 96 символов ASCII: от 0x20 (space) до 0x7F (~)
  - Каждый символ: 5 байт (5 колонок), младшие 7 бит = пиксели по Y (сверху вниз)
*/
extern const uint8_t Font5x7[96][5];

#endif // FONT5X7_H