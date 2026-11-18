#include "delay.h"

#define SYSTICK_CTRL (*(volatile uint32_t*)0xE000E010)
#define SYSTICK_LOAD (*(volatile uint32_t*)0xE000E014)
#define SYSTICK_VAL  (*(volatile uint32_t*)0xE000E018)

void delay_ms(uint32_t ms) {
    SYSTICK_LOAD = 48000 - 1; // 48MHz / 1000
    SYSTICK_VAL = 0;
    SYSTICK_CTRL = 5; // Enable, core clock, no int

    for (uint32_t i = 0; i < ms; i++) {
        while ((SYSTICK_CTRL & (1 << 16)) == 0);
    }
    SYSTICK_CTRL = 0;
}
