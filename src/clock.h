#ifndef CLOCK_H_
#define CLOCK_H_

#include <stdint.h>
#include "S32K116.h"

typedef enum {
    PORT_A = 0,
    PORT_B,
    PORT_C,
    PORT_D,
    PORT_E
} Port_t;

typedef enum {
    PERIPH_LPUART0 = 0,
    PERIPH_LPUART1,
    PERIPH_LPI2C0,
    PERIPH_LPSPI0,
    PERIPH_ADC0
} Peripheral_t;

void     Clock_InitSystem(void);
void     Clock_EnablePort(Port_t port);
void     Clock_EnablePeripheral(Peripheral_t peri);
uint32_t Clock_GetSystemFreq(void);

#endif
