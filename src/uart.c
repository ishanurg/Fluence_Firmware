/* uart.c – S32K116 : LPUART0 on PTC2(RX)/PTC3(TX) — ALT4
   – Pure polling RX (same as the original working S32K116 code)
   – No interrupts, no weak-handler risk
   – TX loop feeds external watchdog                                  */

#include "uart.h"
#include "clock.h"
#include "gpio.h"

void UART_Begin(uint32_t baudrate)
{
    IP_SCG->FIRCDIV |= SCG_FIRCDIV_FIRCDIV2(1U);

    Clock_EnablePort(PORT_C);
    Clock_EnablePeripheral(PERIPH_LPUART0);

    IP_PORTC->PCR[2] = PORT_PCR_MUX(4U) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;
    IP_PORTC->PCR[3] = PORT_PCR_MUX(4U);

    /* Software reset — matches the original working code */
    IP_LPUART0->GLOBAL |=  LPUART_GLOBAL_RST_MASK;
    IP_LPUART0->GLOBAL &= ~LPUART_GLOBAL_RST_MASK;

    uint32_t sbr = (Clock_GetSystemFreq() + (baudrate * 8U)) / (baudrate * 16U);
    if (sbr == 0U) sbr = 1U;

    IP_LPUART0->BAUD = LPUART_BAUD_OSR(15U) | LPUART_BAUD_SBR(sbr);

    /* TE + RE only. NO RIE, NO interrupt enable. */
    IP_LPUART0->CTRL = LPUART_CTRL_TE_MASK | LPUART_CTRL_RE_MASK;
}

void UART_Write(char c)
{
    uint32_t guard = 0U;
    while ((IP_LPUART0->STAT & LPUART_STAT_TDRE_MASK) == 0U) {
        if (++guard >= 2400U) { guard = 0U; Watchdog_Feed(); }
    }
    IP_LPUART0->DATA = (uint32_t)c;
}

void UART_Print(const char *str)  { while (*str) UART_Write(*str++); }
void UART_Println(const char *str){ UART_Print(str); UART_Write('\r'); UART_Write('\n'); }

void UART_PrintInt(int32_t num)
{
    char buf[12];
    int  i = 0;
    bool neg = false;
    if (num == 0) { UART_Write('0'); return; }
    if (num < 0)  { neg = true; num = -num; }
    while (num > 0) { buf[i++] = (char)((num % 10) + '0'); num /= 10; }
    if (neg) buf[i++] = '-';
    while (i > 0) UART_Write(buf[--i]);
}

void UART_PrintFloat(float num, uint8_t dec)
{
    if (num != num)     { UART_Print("nan");  return; }
    if (num >  1.0e30f) { UART_Print("inf");  return; }
    if (num < -1.0e30f) { UART_Print("-inf"); return; }
    if (num < 0.0f) { UART_Write('-'); num = -num; }
    int32_t ip = (int32_t)num;
    float   fr = num - (float)ip;
    UART_PrintInt(ip);
    if (dec > 0U) UART_Write('.');
    while (dec > 0U) {
        fr *= 10.0f;
        int32_t d = (int32_t)fr;
        UART_Write((char)(d + '0'));
        fr -= (float)d;
        dec--;
    }
}

bool UART_Available(void)
{
    if (IP_LPUART0->STAT & LPUART_STAT_OR_MASK) {
        IP_LPUART0->STAT |= LPUART_STAT_OR_MASK;
    }
    return ((IP_LPUART0->STAT & LPUART_STAT_RDRF_MASK) != 0U);
}

int UART_Read(void)
{
    if (!UART_Available()) return -1;
    return (int)(IP_LPUART0->DATA & 0xFFU);
}
