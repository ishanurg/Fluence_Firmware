#include "project_math.h"


float math_abs(float x)
{
    if (x < 0.0f) {
        return -x;
    }

    return x + 0.0f;
}


float math_floor(float x)
{
    int whole = (int)x;                          /* truncates toward zero */
    float whole_as_float = (float)whole;

    if (x < 0.0f && whole_as_float != x) {
        /* Negative and not an exact integer → round one step down */
        return whole_as_float - 1.0f;
    }
    return whole_as_float;
}



float math_sqrt(float x)
{
    if (x <= 0.0f) {
        return 0.0f;
    }

    float guess = x / 2.0f;
    for (int i = 0; i < 15; i++) {
        guess = 0.5f * (guess + (x / guess));
    }
    return guess;
}



float math_exp(float x)
{
    /* Clamp to safe range (avoids overflow) */
    if (x > 88.0f) {
        return 1.0e38f;              /* effectively infinity */
    }
    if (x < -88.0f) {
        return 0.0f;
    }

    /* ---- Step 1: find k such that x ≈ k * ln(2) + remainder ---- */
    float absolute_x = math_abs(x);
    int   k          = (int)(absolute_x * 1.4427f + 0.5f);   /* 1.4427 ≈ 1/ln(2) */
    if (x < 0.0f) {
        k = -k;
    }

    float r = x - ((float)k * 0.6931458f) - ((float)k * 0.0000014f);

    float result = 1.0f;
    float term   = 1.0f;
    for (int i = 1; i <= 8; i++) {
        term   = (term * r) / (float)i;
        result = result + term;
    }

    if (k > 0) {
        for (int i = 0; i < k; i++) {
            result = result * 2.0f;
        }
    } else if (k < 0) {
        for (int i = 0; i < -k; i++) {
            result = result * 0.5f;
        }
    }

    return result;
}



float round_to(float value, int decimals)
{
    float factor = 1.0f;
    if (decimals > 0) {
        for (int i = 0; i < decimals; i++) {
            factor = factor * 10.0f;
        }
    } else if (decimals < 0) {
        for (int i = 0; i < -decimals; i++) {
            factor = factor / 10.0f;
        }
    }

    float scaled = value * factor;
    float rounded;

    if (scaled >= 0.0f) {
        rounded = math_floor(scaled + 0.5f);
    } else {
        rounded = -math_floor(-scaled + 0.5f);
    }

    return rounded / factor;
}


float array_median(const float *values, int count)
{
    if (values == NULL || count <= 0) {
        return 0.0f;
    }

    float sorted[16];
    int n = count;
    if (n > 16) {
        n = 16;
    }

    /* Copy to a local array */
    for (int i = 0; i < n; i++) {
        sorted[i] = values[i];
    }

    /* Bubble sort */
    for (int pass = 0; pass < n - 1; pass++) {
        for (int j = 0; j < n - 1 - pass; j++) {
            if (sorted[j] > sorted[j + 1]) {
                float temp    = sorted[j];
                sorted[j]     = sorted[j + 1];
                sorted[j + 1] = temp;
            }
        }
    }

    /* Pick the middle */
    if (n % 2 == 1) {
        return sorted[n / 2];
    }
    return (sorted[n / 2 - 1] + sorted[n / 2]) / 2.0f;
}



float array_stdev_population(const float *values, int count)
{
    if (values == NULL || count <= 1) {
        return 0.0f;
    }


    float sum = 0.0f;
    for (int i = 0; i < count; i++) {
        sum = sum + values[i];
    }
    float average = sum / (float)count;

    float squared_sum = 0.0f;
    for (int i = 0; i < count; i++) {
        float difference = values[i] - average;
        squared_sum = squared_sum + (difference * difference);
    }

    return math_sqrt(squared_sum / (float)count);
}



float lookup_interpolate(float x,
                         const float *x_points,
                         const float *y_points,
                         int num_points,
                         int round_decimals)
{
    if (x <= x_points[0]) {
        return y_points[0];
    }
    if (x >= x_points[num_points - 1]) {
        return y_points[num_points - 1];
    }

    for (int i = 0; i < num_points - 1; i++) {
        if (x >= x_points[i] && x < x_points[i + 1]) {
            float x0 = x_points[i];
            float x1 = x_points[i + 1];
            float y0 = y_points[i];
            float y1 = y_points[i + 1];

            float slope  = (y1 - y0) / (x1 - x0);
            float result = y0 + (x - x0) * slope;
            return round_to(result, round_decimals);
        }
    }
    return y_points[num_points - 1];
}




float calc_hsln(float hslo, float tinf, float ref_temp)
{
    float numerator   = 0.1742f + (0.00045f * ref_temp);
    float denominator = 0.1742f + (0.00045f * tinf);
    return round_to((hslo / denominator) * numerator, 0);
}

float calc_wliv(float tliv, float rliv, float atmp)
{
    float exponent = ((18.678f - (tliv / 234.5f)) * tliv)
                   / (257.14f + tliv);
    float result   = (rliv / atmp) * 0.61121f * math_exp(exponent);
    return round_to(result, 2);
}

float calc_wsll(float tliv, float hsln, float ref_temp)
{
    float scale = hsln / (0.1742f + (0.00045f * ref_temp));
    float poly  = 0.0167f
                + (0.0000695f * tliv)
                + (0.00000005f * tliv * tliv);
    return round_to(scale * poly, 0);
}

float calc_temp_norm_factor(float tliv, float ref_temp)
{
    float numerator   = 0.1742f + (0.00045f * ref_temp);
    float denominator = 0.1742f + (0.00045f * tliv);
    return numerator / denominator;
}
