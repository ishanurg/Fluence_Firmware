#include "S32K116.h"
#include "clock.h"
#include "rs485.h"
#include "lpi2c.h"
#include "gpio.h"
#include "hdc3020.h"
#include "inf.h"
#include "h2_sensor_logic.h"
#include "nvm.h" // <-- Include
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#define SYSTICK_CTRL (*(volatile uint32_t *)0xE000E010U)
#define SYSTICK_LOAD (*(volatile uint32_t *)0xE000E014U)
#define SYSTICK_VAL  (*(volatile uint32_t *)0xE000E018U)
#define SYSTICK_COUNTFLAG (1UL << 16)

#define CPU_CLOCK_HZ      48000000UL
#define HDC_WAIT_MS       10U
#define CONC_WAIT_MS      35U
#define SAMPLE_INTERVAL_MS 250U

typedef enum { MODE_NORMAL = 0, MODE_CALIBRATION } FirmwareMode_t;

static FirmwareMode_t current_mode = MODE_NORMAL;
static float last_valid_tliv = 25.0f;
static float last_valid_rliv = 50.0f;
static H2_LogicState_t  h2_state;
static H2_LogicOutput_t h2_output;
static bool             streaming_enabled = false;
static volatile uint32_t current_ms = 0;
static uint32_t last_sample_ms = 0;

/* --------------------------------------------------------- */
/* Timing & Watchdog                                         */
/* --------------------------------------------------------- */
static void Delay_1ms(void)
{
    SYSTICK_LOAD = (CPU_CLOCK_HZ / 1000UL) - 1UL;
    SYSTICK_VAL  = 0UL;
    SYSTICK_CTRL = 5UL;

    while ((SYSTICK_CTRL & SYSTICK_COUNTFLAG) == 0U) {
        RS485_Poll();
    }
    SYSTICK_CTRL = 0UL;
    Watchdog_Feed();
    current_ms++;
}

static void Wait_Milliseconds(uint32_t ms)
{
    for (uint32_t i = 0U; i < ms; i++) {
        Delay_1ms();
    }
}

static inline float clampf(float v, float lo, float hi)
{
    if (v != v) return lo;
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

/* --------------------------------------------------------- */
/* Printing Helpers                                          */
/* --------------------------------------------------------- */
static void Print_Integer(int32_t value)
{
    char buf[12]; int i = 0; bool neg = false;
    if (value < 0) { neg = true; value = -value; }
    if (value == 0) { buf[i++] = '0'; }
    while (value > 0) { buf[i++] = (char)('0' + (value % 10)); value /= 10; }
    if (neg) RS485_Write('-');
    while (i > 0) { i--; RS485_Write(buf[i]); }
}

static void Print_Header(void)
{
    RS485_Println("TLIV,RLIV,TCDI,TINF,MEDT,MEDO,WAVO,WLIV,WSLL,WUNI,DRYO,"
                  "SDTM,SDEV,SDTH,COTH,PRS,PRC,NRS,NRC,NCRS,NCRC,"
                  "FNOP,RTRG,TRCN,TTRG,H2CN");
}

static void Print_Row(const H2_LogicOutput_t *o)
{
    if (o == NULL) return;
    RS485_PrintFloat(o->TLIV, 3);        RS485_Write(',');
    RS485_PrintFloat(o->RLIV, 3);        RS485_Write(',');
    RS485_PrintFloat(o->TCDI, 1);        RS485_Write(',');
    RS485_PrintFloat(o->TINF, 1);        RS485_Write(',');
    RS485_PrintFloat(o->MEDT, 4);        RS485_Write(',');
    RS485_PrintFloat(o->MEDO, 1);        RS485_Write(',');
    RS485_PrintFloat(o->WAVO, 4);        RS485_Write(',');
    RS485_PrintFloat(o->WLIV, 2);        RS485_Write(',');
    Print_Integer((int32_t)o->WSLL);     RS485_Write(',');
    RS485_PrintFloat(o->WUNI, 2);        RS485_Write(',');
    RS485_PrintFloat(o->DRYO, 2);        RS485_Write(',');
    RS485_PrintFloat(o->SDTM, 3);        RS485_Write(',');
    RS485_PrintFloat(o->SDEV, 4);        RS485_Write(',');
    RS485_PrintFloat(o->SDTH, 3);        RS485_Write(',');
    RS485_PrintFloat(o->COTH, 4);        RS485_Write(',');
    RS485_PrintFloat(o->PRS, 3);         RS485_Write(',');
    RS485_PrintFloat(o->PRC, 3);         RS485_Write(',');
    RS485_PrintFloat(o->NRS, 3);         RS485_Write(',');
    RS485_PrintFloat(o->NRC, 3);         RS485_Write(',');
    RS485_PrintFloat(o->NCRS, 3);        RS485_Write(',');
    RS485_PrintFloat(o->NCRC, 3);        RS485_Write(',');
    RS485_PrintFloat(o->FNOP, 3);        RS485_Write(',');
    Print_Integer((int32_t)o->RTRG);     RS485_Write(',');
    Print_Integer((int32_t)o->TRCN);     RS485_Write(',');
    Print_Integer((int32_t)o->TTRG);     RS485_Write(',');
    RS485_PrintFloat(o->H2CN, 4);
    RS485_Println("");
}

static void Do_Sample(void)
{
    float   tliv = last_valid_tliv;
    float   rliv = last_valid_rliv;
    int16_t raw_conc = 0;
    int8_t  raw_temp = 0;
    double  hdc_t = 0.0, hdc_rh = 0.0;

    Watchdog_Feed();

    if (HDC3020_TriggerMeasurement()) {
        Wait_Milliseconds(HDC_WAIT_MS);
        if (HDC3020_ReadData(&hdc_t, &hdc_rh)) {
            tliv = clampf((float)hdc_t,  -40.0f, 125.0f);
            rliv = clampf((float)hdc_rh,   0.0f, 100.0f);
            last_valid_tliv = tliv;
            last_valid_rliv = rliv;
        }
    }

    INF_WakeSequence();
    if (INF_TriggerConc()) {
        Wait_Milliseconds(CONC_WAIT_MS);
        (void)INF_ReadConcData(&raw_conc);
    }

    INF_WakeSequence();
    if (INF_TriggerTemp()) {
        Wait_Milliseconds(2);
        (void)INF_ReadTempData(&raw_temp);
    }

    Watchdog_Feed();

    H2_Logic_Process(&h2_state, 0.0f,
                     clampf(tliv,             -40.0f,   125.0f),
                     clampf(rliv,               0.0f,   100.0f),
                     clampf((float)raw_conc, -32000.0f, 32000.0f),
                     clampf((float)raw_temp,   -128.0f,   127.0f),
                     &h2_output);
    Print_Row(&h2_output);
}

/* --------------------------------------------------------- */
/* Command Parser & Dispatcher                               */
/* --------------------------------------------------------- */
static void Execute_Command(const char *buf)
{
    if (strcmp(buf, "FACCAL") == 0) {
        current_mode = MODE_CALIBRATION;
        streaming_enabled = false;
        RS485_Println("CALIBRATION MODE");
        return;
    }
    if (strcmp(buf, "NORMAL") == 0) {
        current_mode = MODE_NORMAL;
        RS485_Println("NORMAL MODE");
        return;
    }
    if (current_mode == MODE_CALIBRATION) {
        RS485_Println("ERR,CMD REJECTED IN CALIBRATION MODE");
        return;
    }

    if (strcmp(buf, "START") == 0) {
        if (!streaming_enabled) {
            streaming_enabled = true;
            last_sample_ms = current_ms;
            RS485_Println("OK,STARTED");
            Print_Header();
            Do_Sample();
            last_sample_ms = current_ms;
        } else {
            RS485_Println("ERR,ALREADY STARTED");
        }
        return;
    }
    if (strcmp(buf, "STOP") == 0) {
        streaming_enabled = false;
        RS485_Println("OK,STOPPED");
        return;
    }
    if (strcmp(buf, "HELP") == 0) {
        RS485_Println("Commands:");
        RS485_Println("  *FACCAL#   - enter calibration mode");
        RS485_Println("  *NORMAL#   - enter normal mode");
        RS485_Println("  *START#    - start streaming (250ms)");
        RS485_Println("  *STOP#     - stop streaming");
        RS485_Println("  *FNOPTAB#  - dump FNOP and H2CN LUT");
        RS485_Println("  *CONFTAB#  - dump current math constants");
        RS485_Println("  *SET,<name>,<value># - change constant (and save to Flash)");
        return;
    }
    if (strcmp(buf, "FNOPTAB") == 0) {
        float *fn = NULL; float *hc = NULL; int n = 0;
        get_lut_data(&fn, &hc, &n);
        Watchdog_Feed();
        RS485_Println("OK,FNOPTAB");
        for (int i = 0; i < n; i++) {
            Watchdog_Feed();
            Print_Integer(i);
            RS485_Print("\t ");
            RS485_PrintFloat(fn[i], 6U);
            RS485_Print("\t ");
            RS485_PrintFloat(hc[i], 6U);
            RS485_Println("");
        }
        return;
    }
    if (strcmp(buf, "CONFTAB") == 0) {
        RS485_Println("OK,CONFTAB");
        RS485_Println("Name           : Value");
        RS485_Println("-----------------------");
        RS485_Print("HSLO           : "); RS485_PrintFloat(h2_state.HSLO, 3); RS485_Println("");
        RS485_Print("ATMP           : "); RS485_PrintFloat(h2_state.ATMP, 3); RS485_Println("");
        RS485_Print("TCOM           : "); RS485_PrintFloat(h2_state.TCOM, 3); RS485_Println("");
        RS485_Print("MERR           : "); RS485_PrintFloat(h2_state.MERR, 3); RS485_Println("");
        RS485_Print("WAVO_INIT      : "); RS485_PrintFloat(h2_state.WAVO_INIT, 3); RS485_Println("");
        RS485_Print("FILTER_JUMP    : "); RS485_PrintFloat(h2_state.FILTER_JUMP, 3); RS485_Println("");
        RS485_Print("FILTER_ALPHA   : "); RS485_PrintFloat(h2_state.FILTER_ALPHA, 4); RS485_Println("");
        RS485_Print("SDTH_MIN       : "); RS485_PrintFloat(h2_state.SDTH_MIN, 4); RS485_Println("");
        RS485_Print("SDTH_MULT      : "); RS485_PrintFloat(h2_state.SDTH_MULT, 3); RS485_Println("");
        RS485_Print("TREND_THRESH   : "); RS485_PrintFloat(h2_state.TREND_THRESH, 4); RS485_Println("");
        RS485_Print("H2_POS_THRESH  : "); RS485_PrintFloat(h2_state.H2_POS_THRESH, 4); RS485_Println("");
        RS485_Print("N2_NEG_THRESH  : "); RS485_PrintFloat(h2_state.N2_NEG_THRESH, 4); RS485_Println("");
        RS485_Print("TARE_COUNT     : "); Print_Integer(h2_state.TARE_COUNT); RS485_Println("");
        RS485_Print("TEMP_FAULT     : "); RS485_PrintFloat(h2_state.TEMP_FAULT, 3); RS485_Println("");
        RS485_Print("REF_TEMP       : "); RS485_PrintFloat(h2_state.REF_TEMP, 3); RS485_Println("");
        RS485_Println("-----------------------");
        RS485_Println("END,CONFTAB");
        return;
    }

    if (strncmp(buf, "SET,", 4) == 0) {
        char name[32]; int ni = 0, i = 4;
        while (buf[i] != '\0' && buf[i] != ',' && ni < 31) name[ni++] = buf[i++];
        name[ni] = '\0';
        if (buf[i] != ',') { RS485_Println("ERR,BAD FORMAT"); return; }
        i++;

        float value = 0.0f, sign = 1.0f, divisor = 10.0f;
        bool in_dec = false;
        bool valid_num = false;

        if (buf[i] == '-') { sign = -1.0f; i++; }

        while (buf[i] != '\0') {
            char d = buf[i];
            if (d == '.') {
                if (in_dec) { RS485_Println("ERR,BAD FORMAT"); return; }
                in_dec = true;
            } else if (d >= '0' && d <= '9') {
                valid_num = true;
                if (in_dec) { value += (float)(d - '0') / divisor; divisor *= 10.0f; }
                else        { value  = value * 10.0f + (float)(d - '0'); }
            } else {
                RS485_Println("ERR,BAD FORMAT"); return;
            }
            i++;
        }
        if (!valid_num) { RS485_Println("ERR,BAD FORMAT"); return; }
        value *= sign;

        bool found = true;
        if (H2_Logic_SetLUT(name, value)) {
            /* LUT updated successfully */
        }
        else if (strcmp(name, "HSLO") == 0)          h2_state.HSLO = value;
        else if (strcmp(name, "ATMP") == 0)          h2_state.ATMP = value;
        else if (strcmp(name, "TCOM") == 0)          h2_state.TCOM = value;
        else if (strcmp(name, "MERR") == 0)          h2_state.MERR = value;
        else if (strcmp(name, "WAVO_INIT") == 0)     h2_state.WAVO_INIT = value;
        else if (strcmp(name, "FILTER_JUMP") == 0)   h2_state.FILTER_JUMP = value;
        else if (strcmp(name, "FILTER_ALPHA") == 0)  h2_state.FILTER_ALPHA = value;
        else if (strcmp(name, "SDTH_MIN") == 0)      h2_state.SDTH_MIN = value;
        else if (strcmp(name, "SDTH_MULT") == 0)     h2_state.SDTH_MULT = value;
        else if (strcmp(name, "TREND_THRESH") == 0)  h2_state.TREND_THRESH = value;
        else if (strcmp(name, "H2_POS_THRESH") == 0) h2_state.H2_POS_THRESH = value;
        else if (strcmp(name, "N2_NEG_THRESH") == 0) h2_state.N2_NEG_THRESH = value;
        else if (strcmp(name, "TARE_COUNT") == 0)    h2_state.TARE_COUNT = (uint16_t)value;
        else if (strcmp(name, "TEMP_FAULT") == 0)    h2_state.TEMP_FAULT = value;
        else if (strcmp(name, "REF_TEMP") == 0)      h2_state.REF_TEMP = value;
        else found = false;

        if (found) {
            if (NVM_Save(&h2_state)) {
                RS485_Print("OK,"); RS485_Print(name); RS485_Println("# (Saved)");
            } else {
                RS485_Print("OK,"); RS485_Print(name); RS485_Println("# (Save Failed!)");
            }
        } else {
            RS485_Println("ERR,UNKNOWN NAME");
        }
        return;
    }

    RS485_Println("ERR,UNKNOWN CMD");
}

static void handle_rs485(void)
{
    static char    buf[64];
    static uint8_t pos = 0U;

    RS485_Poll();

    while (RS485_Available()) {
        char c = (char)RS485_Read();

        if (c == '*') { pos = 0U; continue; }

        bool term = (c == '#' || c == '\r' || c == '\n');
        if (!term) {
            if (pos < 63U) buf[pos++] = c;
            continue;
        }
        if (pos == 0U) continue;

        buf[pos] = '\0';
        pos = 0U;
        Execute_Command(buf);
    }
}

void WDOG_disable(void)
{
    IP_WDOG->CNT = 0xD928C520U;
    while ((IP_WDOG->CS & WDOG_CS_ULK_MASK) == 0U) { }
    IP_WDOG->TOVAL = 0x0000FFFFU;
    IP_WDOG->CS    = 0x00002100U;
}

int main(void)
{
    WDOG_disable();
    Clock_InitSystem();
    Watchdog_InitExternal();

    PORT_init();
    LPI2C0_init();
    RS485_Begin(115200U);
    INF_InitLeaky();

    H2_Logic_Init(&h2_state);
    NVM_Init(&h2_state); /* Load saved constants from Flash */

    RS485_Println("*BOOT,READY (RS485)#");
    RS485_Println("*TYPE *HELP# to get all the commands.");

    while (1) {
        handle_rs485();

        if (streaming_enabled && current_mode == MODE_NORMAL) {
            if ((int32_t)(current_ms - last_sample_ms) >= (int32_t)SAMPLE_INTERVAL_MS) {
                last_sample_ms = current_ms;
                Do_Sample();
            }
        }

        Delay_1ms();
    }
}
