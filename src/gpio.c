#include "gpio.h"

void GPIO_PinMux(Port_t port, uint8_t pin, uint8_t mux) {
    Clock_EnablePort(port);
    switch (port) {
        case PORT_A: IP_PORTA->PCR[pin] = PORT_PCR_MUX(mux); break;
        case PORT_B: IP_PORTB->PCR[pin] = PORT_PCR_MUX(mux); break;
        case PORT_C: IP_PORTC->PCR[pin] = PORT_PCR_MUX(mux); break;
        case PORT_D: IP_PORTD->PCR[pin] = PORT_PCR_MUX(mux); break;
        case PORT_E: IP_PORTE->PCR[pin] = PORT_PCR_MUX(mux); break;
        default: break;
    }
}

void GPIO_InitOutput(Port_t port, uint8_t pin) {
    GPIO_PinMux(port, pin, 1U); /* ALT1 = GPIO */
    switch (port) {
        case PORT_A: IP_PTA->PDDR |= (1UL << pin); break;
        case PORT_B: IP_PTB->PDDR |= (1UL << pin); break;
        case PORT_C: IP_PTC->PDDR |= (1UL << pin); break;
        case PORT_D: IP_PTD->PDDR |= (1UL << pin); break;
        case PORT_E: IP_PTE->PDDR |= (1UL << pin); break;
        default: break;
    }
}

void GPIO_SetPin(Port_t port, uint8_t pin) {
    switch (port) {
        case PORT_A: IP_PTA->PSOR = (1UL << pin); break;
        case PORT_B: IP_PTB->PSOR = (1UL << pin); break;
        case PORT_C: IP_PTC->PSOR = (1UL << pin); break;
        case PORT_D: IP_PTD->PSOR = (1UL << pin); break;
        case PORT_E: IP_PTE->PSOR = (1UL << pin); break;
        default: break;
    }
}

void GPIO_ClearPin(Port_t port, uint8_t pin) {
    switch (port) {
        case PORT_A: IP_PTA->PCOR = (1UL << pin); break;
        case PORT_B: IP_PTB->PCOR = (1UL << pin); break;
        case PORT_C: IP_PTC->PCOR = (1UL << pin); break;
        case PORT_D: IP_PTD->PCOR = (1UL << pin); break;
        case PORT_E: IP_PTE->PCOR = (1UL << pin); break;
        default: break;
    }
}

void GPIO_TogglePin(Port_t port, uint8_t pin) {
    switch (port) {
        case PORT_A: IP_PTA->PTOR = (1UL << pin); break;
        case PORT_B: IP_PTB->PTOR = (1UL << pin); break;
        case PORT_C: IP_PTC->PTOR = (1UL << pin); break;
        case PORT_D: IP_PTD->PTOR = (1UL << pin); break;
        case PORT_E: IP_PTE->PTOR = (1UL << pin); break;
        default: break;
    }
}

void Watchdog_DisableInternal(void) {
    /* Unlock internal WDOG and disable it */
    IP_WDOG->CNT = 0xD928C520UL;
    while ((IP_WDOG->CS & WDOG_CS_ULK_MASK) == 0U) {
        /* Wait until unlocked */
    }
    IP_WDOG->TOVAL = 0x0000FFFFUL;
    IP_WDOG->CS = WDOG_CS_CMD32EN(1U) | WDOG_CS_CLK(1U) | WDOG_CS_UPDATE(1U);
    while ((IP_WDOG->CS & WDOG_CS_RCS_MASK) == 0U) {
        /* Wait until reconfiguration completes */
    }
}

void Watchdog_InitExternal(void) {
    /* Configure PTB4 (Pin 15) as GPIO Output for TPS3851 WDI */
    GPIO_InitOutput(WDI_PORT, WDI_PIN);
    Watchdog_Feed();
}

void Watchdog_Feed(void) {
    /* Toggle PTB4 to service external TPS3851 watchdog (Timeout = 324ms with C49=100nF) */
    IP_PTB->PTOR = (1UL << WDI_PIN);
}
