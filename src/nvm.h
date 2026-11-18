#ifndef NVM_H_
#define NVM_H_

#include "h2_sensor_logic.h"
#include <stdbool.h>
#include <stdint.h>

#define NVM_MAGIC      0x4843324E  /* "H2CN" */

void NVM_Init(H2_LogicState_t *state);
bool NVM_Save(const H2_LogicState_t *state);

#endif /* NVM_H_ */
