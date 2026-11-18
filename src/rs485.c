#include "rs485.h"
#include "clock.h"
#include "gpio.h"
#include "S32K116.h"

#define RS485_DE_PORT   PORT_E
#define RS485_DE_PIN    4U

#define RX_BUF_SIZE     128U
static volatile char     rx_buf[RX_BUF_SIZE];
static volatile uint8_t  rx_head = 0U;
static volatile uint8_t  rx_tail = 0U;

static inline void de_assert(void)   { GPIO_SetPin(RS485_DE_PORT, RS485_DE_PIN);   }
static inline void de_deassert(void) { GPIO_ClearPin(RS485_DE_PORT, RS485_DE_PIN); }

/* FIX: Removed the modulo (%) operator which was adding ~500us per character */
static void settle_delay(uint32_t loops)
{
    for (volatile uint32_t i = 0U; i < loops; i++) {
        __asm("nop");
    }
}

void RS485_Begin(uint32_t baudrate)
{
    IP_SCG->FIRCDIV |= SCG_FIRCDIV_FIRCDIV2(1U);
    Clock_EnablePort(PORT_C);
    Clock_EnablePort(PORT_E);
    Clock_EnablePeripheral(PERIPH_LPUART1);

    IP_PORTC->PCR[8] = PORT_PCR_MUX(2U) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;
    IP_PORTC->PCR[9] = PORT_PCR_MUX(2U);

    GPIO_InitOutput(RS485_DE_PORT, RS485_DE_PIN);
    de_deassert();

    IP_LPUART1->GLOBAL |=  LPUART_GLOBAL_RST_MASK;
    IP_LPUART1->GLOBAL &= ~LPUART_GLOBAL_RST_MASK;

    uint32_t sbr = (Clock_GetSystemFreq() + (baudrate * 8U)) / (baudrate * 16U);
    if (sbr == 0U) sbr = 1U;

    IP_LPUART1->BAUD = LPUART_BAUD_OSR(15U) | LPUART_BAUD_SBR(sbr);
    IP_LPUART1->CTRL = LPUART_CTRL_TE_MASK | LPUART_CTRL_RE_MASK;
}

void RS485_Poll(void)
{
    uint32_t stat = IP_LPUART1->STAT;

    if (stat & (LPUART_STAT_OR_MASK | LPUART_STAT_NF_MASK | LPUART_STAT_FE_MASK)) {
        IP_LPUART1->STAT = (LPUART_STAT_OR_MASK | LPUART_STAT_NF_MASK | LPUART_STAT_FE_MASK);
    }

    if (stat & LPUART_STAT_RDRF_MASK) {
        char c = (char)(IP_LPUART1->DATA & 0xFFU);
        uint8_t next = (rx_head + 1U) & (RX_BUF_SIZE - 1U);
        if (next != rx_tail) {
            rx_buf[rx_head] = c;
            rx_head = next;
        }
    }
}

void RS485_Write(char c)
{
    uint32_t guard = 0U;
    de_assert();
    settle_delay(480);

    while ((IP_LPUART1->STAT & LPUART_STAT_TDRE_MASK) == 0U) {
        if (++guard >= 2400U) { guard = 0U; Watchdog_Feed(); }
    }
    IP_LPUART1->DATA = (uint32_t)c;

    guard = 0U;
    while ((IP_LPUART1->STAT & LPUART_STAT_TC_MASK) == 0U) {
        if (++guard >= 2400U) { guard = 0U; Watchdog_Feed(); }
    }

    settle_delay(480);
    de_deassert();
}

void RS485_Print(const char *str)
{
    while (*str != '\0') RS485_Write(*str++);
}

void RS485_Println(const char *str)
{
    RS485_Print(str);
    RS485_Write('\r');
    RS485_Write('\n');
}

void RS485_PrintInt(int32_t num)
{
    char buf[12]; int  i = 0; bool neg = false;
    if (num == 0) { RS485_Write('0'); return; }
    if (num < 0)  { neg = true; num = -num; }
    while (num > 0) { buf[i++] = (char)((num % 10) + '0'); num /= 10; }
    if (neg) buf[i++] = '-';
    while (i > 0) RS485_Write(buf[--i]);
}

void RS485_PrintFloat(float num, uint8_t dec)
{
    if (num != num)     { RS485_Print("nan");  return; }
    if (num >  1.0e30f) { RS485_Print("inf");  return; }
    if (num < -1.0e30f) { RS485_Print("-inf"); return; }
    if (num < 0.0f) { RS485_Write('-'); num = -num; }

    int32_t ip = (int32_t)num;
    float   fr = num - (float)ip;
    RS485_PrintInt(ip);

    if (dec > 0U) RS485_Write('.');
    while (dec > 0U) {
        fr *= 10.0f;
        int32_t d = (int32_t)fr;
        RS485_Write((char)(d + '0'));
        fr -= (float)d;
        dec--;
    }
}

bool RS485_Available(void)
{
    return (rx_head != rx_tail);
}

int RS485_Read(void)
{
    if (rx_head == rx_tail) return -1;
    char c = rx_buf[rx_tail];
    rx_tail = (rx_tail + 1U) & (RX_BUF_SIZE - 1U);
    return (int)c;
}
