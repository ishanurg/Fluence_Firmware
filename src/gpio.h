#ifndef GPIO_H_
#define GPIO_H_

#include "S32K116.h"
#include "clock.h"
#include <stdint.h>

/* Custom Board Pin Definitions (from CU046_HW_HGS_CNT_SCH.PDF) */
#define WDI_PORT        PORT_B
#define WDI_PIN         4U      /* PTB4 -> External Watchdog WDI (Pin 15) */

#define RED_LED_PORT    PORT_E
#define RED_LED_PIN     8U      /* PTE8 -> RED-LED (Pin 13) */

#define GREEN_LED_PORT  PORT_E
#define GREEN_LED_PIN   9U      /* PTE9 -> GREEN-LED (Pin 12) */

#define BLUE_LED_PORT   PORT_D
#define BLUE_LED_PIN    15U     /* PTD15 -> BLUE-LED (Pin 11) */

void GPIO_PinMux(Port_t port, uint8_t pin, uint8_t mux);
void GPIO_InitOutput(Port_t port, uint8_t pin);
void GPIO_SetPin(Port_t port, uint8_t pin);
void GPIO_ClearPin(Port_t port, uint8_t pin);
void GPIO_TogglePin(Port_t port, uint8_t pin);
void Watchdog_DisableInternal(void);
void Watchdog_InitExternal(void);
void Watchdog_Feed(void);

#endif /* GPIO_H_ */
