#ifndef RS485_H_
#define RS485_H_

#include <stdint.h>
#include <stdbool.h>

void RS485_Begin(uint32_t baudrate);

/* Polls the hardware FIFO and moves bytes into the software ring buffer */
void RS485_Poll(void);

void RS485_Write(char c);
void RS485_Print(const char *str);
void RS485_Println(const char *str);
void RS485_PrintInt(int32_t num);
void RS485_PrintFloat(float num, uint8_t decimal_places);

bool RS485_Available(void);
int  RS485_Read(void);

#endif /* RS485_H_ */
