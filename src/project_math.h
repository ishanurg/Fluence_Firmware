#ifndef PROJECT_MATH_H
#define PROJECT_MATH_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif



float math_abs(float x);

float math_floor(float x);

float math_sqrt(float x);

float math_exp(float x);



/* was: float round(float value, int decimals); */
float round_to(float value, int decimals);

float array_median(const float *values, int count);

float array_stdev_population(const float *values, int count);

float lookup_interpolate(float x,
                         const float *x_points,
                         const float *y_points,
                         int num_points,
                         int round_decimals);



float calc_hsln(float hslo, float tinf, float ref_temp);
float calc_wliv(float tliv, float rliv, float atmp);
float calc_wsll(float tliv, float hsln, float ref_temp);
float calc_temp_norm_factor(float tliv, float ref_temp);

#ifdef __cplusplus
}
#endif

#endif /* PROJECT_MATH_H */
