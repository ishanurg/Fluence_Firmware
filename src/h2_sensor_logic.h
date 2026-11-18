#ifndef H2_SENSOR_LOGIC_H
#define H2_SENSOR_LOGIC_H

#include <stdint.h>
#include <stdbool.h>
#include "project_math.h"

bool H2_Logic_SetLUT(const char *name, float value);

#ifdef __cplusplus
extern "C" {
#endif


#define H2_WINDOW_SIZE   16
#define H2_LUT_SIZE       8


#define H2_DEF_HSLO          18600.0f
#define H2_DEF_ATMP            101.325f
#define H2_DEF_TCOM              0.5f
#define H2_DEF_MERR              6.0f
#define H2_DEF_WAVO_INIT        -9.0f
#define H2_DEF_FILTER_JUMP      10.0f
#define H2_DEF_FILTER_ALPHA      0.04f
#define H2_DEF_SDTH_MIN          0.02f
#define H2_DEF_SDTH_MULT        20.0f
#define H2_DEF_TREND_THRESH      0.008f
#define H2_DEF_H2_POS_THRESH     0.02f
#define H2_DEF_N2_NEG_THRESH    -0.02f
#define H2_DEF_TARE_COUNT      480U
#define H2_DEF_TEMP_FAULT        5.0f
#define H2_DEF_REF_TEMP         25.0f



typedef struct {
    float Time;
    float TLIV;        /* TI temp       */
    float RLIV;        /* TI rh    */
    float TCDI;        /* Inf rw */
    float TINF;        /* Inf temp   */

    float MEDT;        /* Median of TLIV (12 samples) */
    float MEDO;        /* Median of TCDI (12 samples) */
    float WAVO;        /* Filtered MEDO               */
    float WLIV;        /* Water vapor_pressure  */
    float WSLL;        /* Temperature baseline offset */
    float WUNI;        /* Unified compensation        */
    float DRYO;        /* Dry-gas cleaned output      */

    float SDTM;        /* Std dev of MEDT             */
    float SDEV;        /* Std dev of WAVO             */
    float SDTH;        /* Dynamic threshold for SDEV  */
    float COTH;        /* Noise-gated conditioned signal */

    float PRS;         /* Positive step size          */
    float PRC;         /* Negative step size          */
    float NRS;         /* Normalized positive rate    */
    float NRC;         /* Normalized negative rate    */
    float NCRS;        /* Accumulated positive sum    */
    float NCRC;        /* Accumulated negative sum    */

    float    FNOP;     /* Final net output (mV)       */
    uint8_t  RTRG;     /* Reset trigger (0 or 1)      */
    uint16_t TRCN;     /* Auto-tare counter           */
    uint8_t  TTRG;     /* Auto-tare trigger (0 or 1)  */

    float TXIC;        /* TI sensor fault code        */
    float INFC;        /* Infineon fault code         */
    float RAWC;        /* Raw signal fault code       */
    float TEMC;        /* Temperature mismatch code   */
    float FSUM;        /* Fault summary               */
    float H2CN;        /* H2 concentration (0.0 - 0.2) */
} H2_LogicOutput_t;



typedef struct {
    float    HSLO;
    float    ATMP;
    float    TCOM;
    float    MERR;
    float    WAVO_INIT;
    float    FILTER_JUMP;
    float    FILTER_ALPHA;
    float    SDTH_MIN;
    float    SDTH_MULT;
    float    TREND_THRESH;
    float    H2_POS_THRESH;
    float    N2_NEG_THRESH;
    uint16_t TARE_COUNT;
    float    TEMP_FAULT;
    float    REF_TEMP;

    uint32_t sample_count;
    float    RLIV_init;
    float    TSTB;
    float    HSLN;
    float    STBV;

    float    last_medo;
    float    last_wavo;
    float    last_coth;
    float    last_ncrs;
    float    last_ncrc;
    float    last_fnop;
    uint8_t  last_rtrg;
    uint16_t last_trcn;
    uint8_t  last_ttrg;

    float    tliv_buf[H2_WINDOW_SIZE];
    float    tcdi_buf[H2_WINDOW_SIZE];
    uint8_t  raw_head;
    uint8_t  raw_count;

    float    medt_buf[H2_WINDOW_SIZE];
    float    wavo_buf[H2_WINDOW_SIZE];
    uint8_t  stat_head;
    uint8_t  stat_count;
} H2_LogicState_t;



void H2_Logic_Init(H2_LogicState_t *state);

void H2_Logic_Reset(H2_LogicState_t *state);

void H2_Logic_Process(H2_LogicState_t  *state,
                      float             time_s,
                      float             tliv,
                      float             rliv,
                      float             tcdi,
                      float             tinf,
                      H2_LogicOutput_t *out);

void H2_Logic_PrintHeader(void);
void H2_Logic_PrintRow(const H2_LogicOutput_t *out);
void get_lut_data(float **fnop_out, float **h2cn_out, int *size_out);
#ifdef __cplusplus
}
#endif

#endif
