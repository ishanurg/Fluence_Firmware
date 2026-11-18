/* inf.c – S32K116 : TCI/Infineon sensor over LPI2C0
   – Delays now use SysTick (DWT is unreliable on this board)
   – Wake sequence can be called safely before every measurement */

#include "inf.h"
#include "lpi2c.h"
#include "S32K116.h"
#include "uart.h"
#include "gpio.h"          /* Watchdog_Feed() */

/* SysTick registers (Cortex-M0+ standard) */
#define SYSTICK_CTRL (*(volatile uint32_t *)0xE000E010U)
#define SYSTICK_LOAD (*(volatile uint32_t *)0xE000E014U)
#define SYSTICK_VAL  (*(volatile uint32_t *)0xE000E018U)
#define SYSTICK_COUNTFLAG (1UL << 16)

#define CPU_FREQ_HZ   48000000UL

/* ------------------------------------------------------------------ */
/*  Safe delays via SysTick                                            */
/* ------------------------------------------------------------------ */
static void inf_hw_delay_us(uint32_t us)
{
    SYSTICK_LOAD = (CPU_FREQ_HZ / 1000000UL) - 1UL;   /* 47 → 1 µs */
    SYSTICK_VAL  = 0U;
    SYSTICK_CTRL = 5UL;

    for (uint32_t i = 0U; i < us; i++) {
        Watchdog_Feed();
        while ((SYSTICK_CTRL & SYSTICK_COUNTFLAG) == 0U) { }
    }
    SYSTICK_CTRL = 0UL;
}

static void inf_hw_delay_ms(uint32_t ms)
{
    SYSTICK_LOAD = (CPU_FREQ_HZ / 1000UL) - 1UL;      /* 47 999 → 1 ms */
    SYSTICK_VAL  = 0U;
    SYSTICK_CTRL = 5UL;

    for (uint32_t i = 0U; i < ms; i++) {
        Watchdog_Feed();
        while ((SYSTICK_CTRL & SYSTICK_COUNTFLAG) == 0U) { }
    }
    SYSTICK_CTRL = 0UL;
}

/* ------------------------------------------------------------------ */
/*  Constants                                                          */
/* ------------------------------------------------------------------ */
#define TCI_CMD_CONC    0xA8
#define TCI_CMD_TEMP    0xA9
#define TCI_CMD_READ_ID 0xC2

static double leaky_acc_1 = 0.0;
static double leaky_acc_2 = 0.0;
static const double jmp_thresh   = 10.0;
static const double samp_proxy   = 0.02;
static const double jmp_thresh_2 = 15.0;
static const double samp_proxy_2 = 0.01;
static bool first_sample = true;

static uint16_t inf_crc16(const uint8_t *data, uint8_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= ((uint16_t)data[i]) << 8;
        for (uint8_t b = 0; b < 8; b++) {
            if (crc & 0x8000) crc = (uint16_t)((crc << 1) ^ 0x1021);
            else              crc = (uint16_t)(crc << 1);
        }
    }
    return crc;
}

static bool inf_write(const uint8_t *payload, uint8_t len)
{
    uint8_t tx_buf[16];
    uint16_t crc = inf_crc16(payload, len);
    for (uint8_t i = 0; i < len; i++) tx_buf[i] = payload[i];
    tx_buf[len]     = (uint8_t)(crc >> 8);
    tx_buf[len + 1] = (uint8_t)(crc & 0xFF);
    return LPI2C0_transmit(INF_I2C_ADDR, tx_buf, len + 2);
}

static bool inf_read(uint8_t *buf, uint8_t len)
{
    if (!LPI2C0_receive(INF_I2C_ADDR, buf, len)) return false;
    uint16_t crc = inf_crc16(buf, len - 2);
    uint16_t rx  = ((uint16_t)buf[len - 2] << 8) | buf[len - 1];
    return (crc == rx);
}

/* ------------------------------------------------------------------ */
void INF_WakeSequence(void)
{
    /* Disable LPI2C master while we hijack SCL */
    IP_LPI2C0->MCR &= ~LPI2C_MCR_MEN_MASK;

    /* PTA3 → GPIO output, pull-up, drive low then high */
    IP_PORTA->PCR[3] = PORT_PCR_MUX(1U) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;
    IP_PTA->PDDR    |= (1UL << 3);

    IP_PTA->PCOR = (1UL << 3);
    inf_hw_delay_ms(2);

    IP_PTA->PSOR = (1UL << 3);
    inf_hw_delay_us(50);

    /* Restore PTA3 to LPI2C0_SCL (ALT3) and reinit the LPI2C block */
    IP_PORTA->PCR[3] = PORT_PCR_MUX(3U) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;
    LPI2C0_init();
    inf_hw_delay_us(600);
}

void INF_InitLeaky(void) { first_sample = true; }

static double calculate_leaky(double raw, double cur, double th, double pr)
{
    double d = raw - cur;
    if (d < 0) d = -d;
    if (d > th) return raw;
    return ((raw - cur) * pr) + cur;
}

/* ------------------------------------------------------------------ */
bool INF_TriggerConc(void)
{
    uint8_t cmd[5] = {TCI_CMD_CONC, 0x03, 50, 25, 100};
    return inf_write(cmd, 5);
}

bool INF_ReadConcData(int16_t *out_raw)
{
    uint8_t rx[5];
    if (inf_read(rx, 5)) {
        *out_raw = (int16_t)(((uint16_t)rx[1] << 8) | rx[2]);
        return true;
    }
    return false;
}

bool INF_TriggerTemp(void)
{
    uint8_t cmd[1] = {TCI_CMD_TEMP};
    return inf_write(cmd, 1);
}

bool INF_ReadTempData(int8_t *out_temp)
{
    uint8_t rx[4];
    if (inf_read(rx, 4)) {
        *out_temp = (int8_t)rx[1];
        return true;
    }
    return false;
}

/* ------------------------------------------------------------------ */
bool INF_ReadRawConc(int16_t *out_raw)
{
    INF_WakeSequence();
    if (INF_TriggerConc()) {
        inf_hw_delay_ms(35);
        return INF_ReadConcData(out_raw);
    }
    return false;
}

bool INF_ReadTemp(int8_t *out_temp)
{
    if (INF_TriggerTemp()) {
        inf_hw_delay_ms(2);
        return INF_ReadTempData(out_temp);
    }
    return false;
}

void INF_ProcessLeaky(int16_t raw_conc, double *out_l1, double *out_l2)
{
    double raw = (double)raw_conc;
    if (first_sample) {
        leaky_acc_1 = raw; leaky_acc_2 = raw; first_sample = false;
    } else {
        leaky_acc_1 = calculate_leaky(raw, leaky_acc_1, jmp_thresh,   samp_proxy);
        leaky_acc_2 = calculate_leaky(raw, leaky_acc_2, jmp_thresh_2, samp_proxy_2);
    }
    *out_l1 = leaky_acc_1;
    *out_l2 = leaky_acc_2;
}
