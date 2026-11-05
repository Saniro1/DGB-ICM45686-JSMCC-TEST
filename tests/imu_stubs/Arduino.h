#pragma once
#include <stdint.h>
constexpr int LOW = 0, HIGH = 1, OUTPUT = 1;
void pinMode(uint8_t pin, int mode);
void digitalWrite(uint8_t pin, int level);
void delay(unsigned long ms);
void delayMicroseconds(unsigned int us);
uint32_t micros();
