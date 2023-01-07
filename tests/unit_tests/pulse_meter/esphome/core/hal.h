#pragma once
#include <string>
#include <cstdint>
#include "gpio.h"

#define IRAM_ATTR
#define PROGMEM

namespace esphome {

void yield();
uint32_t millis();
uint32_t micros();
void delay(uint32_t ms);
void delayMicroseconds(uint32_t us);  // NOLINT(readability-identifier-naming)
void __attribute__((noreturn)) arch_restart();
void arch_init();
void arch_feed_wdt();
uint32_t arch_get_cpu_cycle_count();
uint32_t arch_get_cpu_freq_hz();
uint8_t progmem_read_byte(const uint8_t *addr);

}  // namespace esphome