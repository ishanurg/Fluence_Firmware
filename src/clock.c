#include "clock.h"

void Clock_InitSystem(void)
{
    /* 1. Enable FIRC (48 MHz) and wait for valid */
    IP_SCG->FIRCCSR |= SCG_FIRCCSR_FIRCEN_MASK;
    while ((IP_SCG->FIRCCSR & SCG_FIRCCSR_FIRCVLD_MASK) == 0U) { }

    /* 2. Enable FIRCDIV2 (÷1 = 48 MHz) — required by LPUART / LPI2C */
    IP_SCG->FIRCDIV = SCG_FIRCDIV_FIRCDIV2(1U) | SCG_FIRCDIV_FIRCDIV1(1U);

    /* 3. Switch system clock to FIRC: core 48 MHz, bus 24 MHz, flash 24 MHz */
    IP_SCG->RCCR = SCG_RCCR_SCS(3U)     |
                   SCG_RCCR_DIVCORE(0U) |
                   SCG_RCCR_DIVBUS(1U)  |
                   SCG_RCCR_DIVSLOW(1U);

    /* Wait until FIRC is the confirmed system clock source */
    while (((IP_SCG->CSR & SCG_CSR_SCS_MASK) >> SCG_CSR_SCS_SHIFT) != 3U) { }
}

void Clock_EnablePort(Port_t port)
{
    switch (port) {
        case PORT_A: IP_PCC->PCCn[PCC_PORTA_INDEX] |= PCC_PCCn_CGC_MASK; break;
        case PORT_B: IP_PCC->PCCn[PCC_PORTB_INDEX] |= PCC_PCCn_CGC_MASK; break;
        case PORT_C: IP_PCC->PCCn[PCC_PORTC_INDEX] |= PCC_PCCn_CGC_MASK; break;
        case PORT_D: IP_PCC->PCCn[PCC_PORTD_INDEX] |= PCC_PCCn_CGC_MASK; break;
        case PORT_E: IP_PCC->PCCn[PCC_PORTE_INDEX] |= PCC_PCCn_CGC_MASK; break;
        default: break;
    }
}

void Clock_EnablePeripheral(Peripheral_t peri)
{
    uint32_t pcc_index = 0U;

    switch (peri) {
        case PERIPH_LPUART0: pcc_index = PCC_LPUART0_INDEX; break;
        case PERIPH_LPUART1: pcc_index = PCC_LPUART1_INDEX; break;
        case PERIPH_LPI2C0:  pcc_index = PCC_LPI2C0_INDEX;  break;
        case PERIPH_LPSPI0:  pcc_index = PCC_LPSPI0_INDEX;  break;
        case PERIPH_ADC0:    pcc_index = PCC_ADC0_INDEX;    break;
        default: return;
    }

    /* Hardware rule: disable CGC before changing PCS */
    IP_PCC->PCCn[pcc_index] &= ~PCC_PCCn_CGC_MASK;

    /* PCS = 3 → FIRCDIV2_CLK = 48 MHz ; then enable gate */
    IP_PCC->PCCn[pcc_index] = PCC_PCCn_PCS(3U) | PCC_PCCn_CGC_MASK;
}

uint32_t Clock_GetSystemFreq(void)
{
    return 48000000UL;
}
