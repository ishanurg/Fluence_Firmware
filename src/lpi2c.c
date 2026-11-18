#include "lpi2c.h"
#include "S32K116.h"
#include "gpio.h"

#define LPI2C_TIMEOUT    20000U
#define WDI_FEED_EVERY   2400U
static uint32_t wdi_tick = 0U;

static inline void wdi_poll(void)
{
    if (++wdi_tick >= WDI_FEED_EVERY) {
        wdi_tick = 0U;
        Watchdog_Feed();
    }
}

/* Recover from any bus error / stuck state */
static void LPI2C0_soft_reset(void)
{
    IP_LPI2C0->MCR = LPI2C_MCR_RST_MASK;
    IP_LPI2C0->MCR = 0x00000000U;
    for (volatile uint32_t i = 0U; i < 200U; i++) { __asm("nop"); }
    IP_LPI2C0->MCFGR1 = LPI2C_MCFGR1_PRESCALE(0U);
    IP_LPI2C0->MCCR0  = LPI2C_MCCR0_CLKHI(236U) |
                        LPI2C_MCCR0_CLKLO(240U) |
                        LPI2C_MCCR0_SETHOLD(119U)|
                        LPI2C_MCCR0_DATAVD(20U);
    IP_LPI2C0->MCR |= LPI2C_MCR_MEN_MASK;
}

void PORT_init(void)
{
    IP_PCC->PCCn[PCC_PORTA_INDEX] |= PCC_PCCn_CGC_MASK;

    IP_PORTA->PCR[2] = PORT_PCR_MUX(3U) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;
    IP_PORTA->PCR[3] = PORT_PCR_MUX(3U) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;
}

void LPI2C0_init(void)
{
    IP_SCG->FIRCDIV |= SCG_FIRCDIV_FIRCDIV2(1U);

    IP_PCC->PCCn[PCC_LPI2C0_INDEX] &= ~PCC_PCCn_CGC_MASK;
    IP_PCC->PCCn[PCC_LPI2C0_INDEX]  = PCC_PCCn_PCS(3U);
    IP_PCC->PCCn[PCC_LPI2C0_INDEX] |= PCC_PCCn_CGC_MASK;

    IP_LPI2C0->MCR = LPI2C_MCR_RST_MASK;
    IP_LPI2C0->MCR = 0x00000000U;
    for (volatile uint32_t i = 0U; i < 200U; i++) { __asm("nop"); }

    IP_LPI2C0->MCFGR1 = LPI2C_MCFGR1_PRESCALE(0U);
    IP_LPI2C0->MCCR0  = LPI2C_MCCR0_CLKHI(236U) |
                        LPI2C_MCCR0_CLKLO(240U) |
                        LPI2C_MCCR0_SETHOLD(119U)|
                        LPI2C_MCCR0_DATAVD(20U);

    IP_LPI2C0->MCR |= LPI2C_MCR_MEN_MASK;
}

bool LPI2C0_transmit(uint16_t saddr, uint8_t *tx_buff, uint32_t tx_num)
{
    uint32_t i, timeout;

    IP_LPI2C0->MSR  = 0x7F00U;
    IP_LPI2C0->MTDR = (4U << 8) | ((uint32_t)saddr << 1) | 0x00U;

    for (i = 0U; i < tx_num; i++) {
        timeout = LPI2C_TIMEOUT;
        while (((IP_LPI2C0->MSR & LPI2C_MSR_TDF_MASK) == 0U) && --timeout) {
            wdi_poll();
        }
        if (timeout == 0U) { LPI2C0_soft_reset(); Watchdog_Feed(); return false; }
        IP_LPI2C0->MTDR = (0U << 8) | tx_buff[i];
    }

    IP_LPI2C0->MTDR = (2U << 8) | 0U;
    timeout = LPI2C_TIMEOUT;
    while (((IP_LPI2C0->MSR & LPI2C_MSR_SDF_MASK) == 0U) && --timeout) {
        wdi_poll();
    }
    IP_LPI2C0->MSR |= LPI2C_MSR_SDF_MASK;

    if (timeout == 0U) { LPI2C0_soft_reset(); Watchdog_Feed(); return false; }
    Watchdog_Feed();
    return true;
}

bool LPI2C0_receive(uint16_t saddr, uint8_t *rx_buff, uint32_t rx_num)
{
    uint32_t i, timeout;

    IP_LPI2C0->MSR  = 0x7F00U;
    IP_LPI2C0->MTDR = (4U << 8) | ((uint32_t)saddr << 1) | 0x01U;
    IP_LPI2C0->MTDR = (1U << 8) | (rx_num - 1U);

    for (i = 0U; i < rx_num; i++) {
        timeout = LPI2C_TIMEOUT;
        while (((IP_LPI2C0->MSR & LPI2C_MSR_RDF_MASK) == 0U) && --timeout) {
            wdi_poll();
        }
        if (timeout == 0U) { LPI2C0_soft_reset(); Watchdog_Feed(); return false; }
        rx_buff[i] = (uint8_t)(IP_LPI2C0->MRDR & 0xFFU);
    }

    IP_LPI2C0->MTDR = (2U << 8) | 0U;
    timeout = LPI2C_TIMEOUT;
    while (((IP_LPI2C0->MSR & LPI2C_MSR_SDF_MASK) == 0U) && --timeout) {
        wdi_poll();
    }
    IP_LPI2C0->MSR |= LPI2C_MSR_SDF_MASK;

    if (timeout == 0U) { LPI2C0_soft_reset(); Watchdog_Feed(); return false; }
    Watchdog_Feed();
    return true;
}
