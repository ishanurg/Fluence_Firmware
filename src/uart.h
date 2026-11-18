#ifndef UART_H_
#define UART_H_

#include <stdint.h>
#include <stdbool.h>
#include "S32K116.h"

void UART_Begin(uint32_t baudrate);

void UART_Write(char c);
void UART_Print(const char *str);
void UART_Println(const char *str);
void UART_PrintInt(int32_t num);
void UART_PrintFloat(float num, uint8_t decimal_places);

bool UART_Available(void);
int  UART_Read(void);

#endif
