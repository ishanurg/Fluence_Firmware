#ifndef HDC3020_H_
#define HDC3020_H_

#include <stdbool.h>

#define HDC3020_I2C_ADDR   0x44U

bool HDC3020_TriggerMeasurement(void);
bool HDC3020_ReadData(double *out_temp, double *out_rh);

#endif
