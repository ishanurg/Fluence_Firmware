#include "h2_sensor_logic.h"
#include <stdio.h>
#include <string.h>


static float FNOP_POINTS[H2_LUT_SIZE] = {
    0.0f, 6.0f, 10.0f, 92.0f, 192.0f, 388.0f, 1847.619048f, 3695.238095f
};

static float H2CN_POINTS[H2_LUT_SIZE] = {
    0.0f, 0.00001f, 0.001079f, 0.0052f, 0.0105f, 0.021f, 0.1f, 0.2f
};


void get_lut_data(float **fnop_out, float **h2cn_out, int *size_out){
	if(fnop_out) *fnop_out = FNOP_POINTS;
	if(h2cn_out) *h2cn_out = H2CN_POINTS;
	if(size_out) *size_out = H2_LUT_SIZE;
}

void H2_Logic_Reset(H2_LogicState_t *state)
{
    if (state == NULL) {
        return;
    }

    state->sample_count = 0;
    state->RLIV_init    = 0.0f;
    state->TSTB         = 0.0f;
    state->HSLN         = 0.0f;
    state->STBV         = 0.0f;

    state->last_medo = 0.0f;
    state->last_wavo = state->WAVO_INIT;   /* seed with -9.0 */
    state->last_coth = 0.0f;
    state->last_ncrs = 0.0f;
    state->last_ncrc = 0.0f;
    state->last_fnop = 0.0f;
    state->last_rtrg = 0;
    state->last_trcn = 0;
    state->last_ttrg = 0;

    for (int i = 0; i < H2_WINDOW_SIZE; i++) {
        state->tliv_buf[i] = 0.0f;
        state->tcdi_buf[i] = 0.0f;
    }
    state->raw_head  = 0;
    state->raw_count = 0;

    for (int i = 0; i < H2_WINDOW_SIZE; i++) {
        state->medt_buf[i] = 0.0f;
        state->wavo_buf[i] = 0.0f;
    }
    state->stat_head  = 0;
    state->stat_count = 0;
}



void H2_Logic_Init(H2_LogicState_t *state)
{
    if (state == NULL) {
        return;
    }

    state->HSLO          = H2_DEF_HSLO;
    state->ATMP          = H2_DEF_ATMP;
    state->TCOM          = H2_DEF_TCOM;
    state->MERR          = H2_DEF_MERR;
    state->WAVO_INIT     = H2_DEF_WAVO_INIT;
    state->FILTER_JUMP   = H2_DEF_FILTER_JUMP;
    state->FILTER_ALPHA  = H2_DEF_FILTER_ALPHA;
    state->SDTH_MIN      = H2_DEF_SDTH_MIN;
    state->SDTH_MULT     = H2_DEF_SDTH_MULT;
    state->TREND_THRESH  = H2_DEF_TREND_THRESH;
    state->H2_POS_THRESH = H2_DEF_H2_POS_THRESH;
    state->N2_NEG_THRESH = H2_DEF_N2_NEG_THRESH;
    state->TARE_COUNT    = H2_DEF_TARE_COUNT;
    state->TEMP_FAULT    = H2_DEF_TEMP_FAULT;
    state->REF_TEMP      = H2_DEF_REF_TEMP;

    H2_Logic_Reset(state);
}



void H2_Logic_Process(H2_LogicState_t  *state,
                      float             time_s,
                      float             tliv,
                      float             rliv,
                      float             tcdi,
                      float             tinf,
                      H2_LogicOutput_t *out)
{
    if (state == NULL || out == NULL) {
        return;
    }

    state->sample_count = state->sample_count + 1;

    bool is_first = (state->sample_count == 1);
    bool window_full = (state->sample_count >= H2_WINDOW_SIZE);



    if (is_first) {
        state->RLIV_init = rliv;
        state->TSTB      = tliv;

        state->HSLN = calc_hsln(state->HSLO, tinf, state->REF_TEMP);

        float boot_wliv = calc_wliv(tliv, rliv, state->ATMP);
        float boot_wsll = calc_wsll(tliv, state->HSLN, state->REF_TEMP);
        state->STBV = -(boot_wliv / 100.0f) * boot_wsll;
    }


    state->tliv_buf[state->raw_head] = tliv;
    state->tcdi_buf[state->raw_head] = tcdi;

    state->raw_head = state->raw_head + 1;
    if (state->raw_head >= H2_WINDOW_SIZE) {
        state->raw_head = 0;
    }

    if (state->raw_count < H2_WINDOW_SIZE) {
        state->raw_count = state->raw_count + 1;
    }



    float medt = 0.0f;
    float medo = 0.0f;
    float wavo = 0.0f;
    float wliv = 0.0f;
    float wsll = 0.0f;
    float wuni = 0.0f;
    float dryo = 0.0f;
    float sdtm = 0.0f;
    float sdev = 0.0f;
    float sdth = 0.0f;

    float coth_previous = state->last_coth;
    float coth_now      = coth_previous;

    if (window_full) {
        medt = array_median(state->tliv_buf, state->raw_count);
        medo = array_median(state->tcdi_buf, state->raw_count);

        if (state->sample_count == H2_WINDOW_SIZE) {
            wavo = state->WAVO_INIT;
        } else {
        	float jump_size = math_abs(medo - state->last_wavo);

            if (jump_size > state->FILTER_JUMP) {
                wavo = (state->last_medo + medo) / 2.0f;
            } else {
                float difference = medo - state->last_wavo;
                wavo = state->last_wavo + (state->FILTER_ALPHA * difference);
            }
        }

        wliv = calc_wliv(tliv, rliv, state->ATMP);
        wsll = calc_wsll(tliv, state->HSLN, state->REF_TEMP);
        wuni = round_to((wliv / 100.0f) * wsll, 2);
        dryo = round_to(wavo - wuni, 2);

        state->medt_buf[state->stat_head] = medt;
        state->wavo_buf[state->stat_head] = wavo;

        state->stat_head = state->stat_head + 1;
        if (state->stat_head >= H2_WINDOW_SIZE) {
            state->stat_head = 0;
        }

        if (state->stat_count < H2_WINDOW_SIZE) {
            state->stat_count = state->stat_count + 1;
        }

        sdtm = array_stdev_population(state->medt_buf, state->stat_count);
        sdtm = round_to(sdtm, 3);

        sdev = array_stdev_population(state->wavo_buf, state->stat_count);

        if (sdtm > state->TREND_THRESH) {
            sdth = sdtm * state->SDTH_MULT;
        } else {
            sdth = state->SDTH_MIN;
        }

        if (state->sample_count > H2_WINDOW_SIZE && sdev > sdth) {
            float change = wavo - state->last_wavo;
            coth_now = coth_previous + change;
        }

        state->last_medo = medo;
        state->last_wavo = wavo;
        state->last_coth = coth_now;
    }



    float prs = 0.0f;
    float prc = 0.0f;

    if ((*state).sample_count > 1) {
        float coth_change = coth_now - coth_previous;

        if (coth_change > state->H2_POS_THRESH) {
            prs = coth_change;
        }

        if (coth_change < state->N2_NEG_THRESH) {
            prc = coth_change;
        }
    }



    float norm_factor = calc_temp_norm_factor(tliv, state->REF_TEMP);
    float nrs = prs * norm_factor;   /* normalized positive rate */
    float nrc = prc * norm_factor;   /* normalized negative rate */



    float ncrs = 0.0f;
    float ncrc = 0.0f;
    float fnop = 0.0f;

    if (state->sample_count > 1) {
        /* Did the previous sample reset the buffers? */
        bool buffer_was_reset = (state->last_rtrg == 1);

        /* Was the previous output zero (i.e. we're starting fresh)? */
        bool output_was_zero = (state->last_fnop == 0.0f);

        /* Update the accumulators */
        if (buffer_was_reset) {
            ncrs = 0.0f;
        } else {
            ncrs = state->last_ncrs + nrs;
        }

        if (buffer_was_reset || output_was_zero) {
            ncrc = 0.0f;
        } else {
            ncrc = state->last_ncrc + nrc;
        }

        /* Pick the final output value */
        float total = ncrs + ncrc;

        if (state->last_ttrg == 1) {
            /* A tare was just applied */
            fnop = 0.0f;
        } else if (total < 0.0f) {
            /* Should not go negative */
            fnop = 0.0f;
        } else if (total < state->MERR) {
            /* Below the minimum reportable output */
            fnop = 0.01f;
        } else {
            /* Normal case */
            fnop = total;
        }
    }

    /* Save for next time */
    state->last_ncrs = ncrs;
    state->last_ncrc = ncrc;



    uint8_t rtrg = 0;
    if (state->sample_count > 1) {
        if (state->last_fnop > 0.0f && fnop == 0.0f) {
            rtrg = 1;
        }
    }
    state->last_fnop = fnop;
    state->last_rtrg = rtrg;

    uint16_t trcn = 0;
    if (state->sample_count > 1) {
        if (state->last_ttrg == 0) {
            if (fnop > 0.0f && fnop < state->MERR) {
                trcn = state->last_trcn + 1;
            }
        }
    }
    state->last_trcn = trcn;

    uint8_t ttrg = 0;
    if (trcn == state->TARE_COUNT) {
        ttrg = 1;
    }
    state->last_ttrg = ttrg;


    float temc = 1.0f;
    float temp_difference = math_abs(tinf - tliv);
    if (temp_difference > state->TEMP_FAULT) {
        temc = 5.0f;
    }

    float txic = 0.0f;
    float infc = 0.0f;
    float rawc = 0.0f;
    float fsum = 0.0f;

    if (is_first) {
        txic = 1000.0f;
        infc = 100.0f;
        rawc = 10.0f;
        fsum = txic + infc + rawc + temc;
    }

    float h2cn = lookup_interpolate(fnop, FNOP_POINTS, H2CN_POINTS,
                                     H2_LUT_SIZE, 4);



    out->Time = time_s;
    out->TLIV = tliv;
    out->RLIV = rliv;
    out->TCDI = tcdi;
    out->TINF = tinf;

    out->MEDT = medt;
    out->MEDO = medo;
    out->WAVO = wavo;
    out->WLIV = wliv;
    out->WSLL = wsll;
    out->WUNI = wuni;
    out->DRYO = dryo;

    out->SDTM = sdtm;
    out->SDEV = sdev;
    out->SDTH = sdth;
    out->COTH = coth_now;

    out->PRS  = prs;
    out->PRC  = prc;
    out->NRS  = nrs;
    out->NRC  = nrc;
    out->NCRS = ncrs;
    out->NCRC = ncrc;

    out->FNOP = fnop;
    out->RTRG = rtrg;
    out->TRCN = trcn;
    out->TTRG = ttrg;

    out->TXIC = txic;
    out->INFC = infc;
    out->RAWC = rawc;
    out->TEMC = temc;
    out->FSUM = fsum;
    out->H2CN = h2cn;
}



void H2_Logic_PrintHeader(void)
{
    printf("Time,TLIV,RLIV,TCDI,TINF,"
           "MEDT,MEDO,WAVO,WLIV,WSLL,WUNI,DRYO,"
           "SDTM,SDEV,SDTH,COTH,"
           "PRS,PRC,NRS,NRC,NCRS,NCRC,"
           "FNOP,RTRG,TRCN,TTRG,"
           "TXIC,INFC,RAWC,TEMC,FSUM,H2CN\r\n");
}

void H2_Logic_PrintRow(const H2_LogicOutput_t *out)
{
    if (out == NULL) {
        return;
    }

    printf("%.2f,%.3f,%.3f,%.1f,%.1f,"
           "%.4f,%.1f,%.4f,%.2f,%.0f,%.2f,%.2f,"
           "%.3f,%.4f,%.3f,%.4f,"
           "%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,"
           "%.2f,%u,%u,%u,"
           "%.0f,%.0f,%.0f,%.0f,%.0f,%.4f\r\n",
           out->Time, out->TLIV, out->RLIV, out->TCDI, out->TINF,
           out->MEDT, out->MEDO, out->WAVO, out->WLIV,
           out->WSLL, out->WUNI, out->DRYO,
           out->SDTM, out->SDEV, out->SDTH, out->COTH,
           out->PRS, out->PRC, out->NRS, out->NRC,
           out->NCRS, out->NCRC,
           out->FNOP, out->RTRG, out->TRCN, out->TTRG,
           out->TXIC, out->INFC, out->RAWC, out->TEMC,
           out->FSUM, out->H2CN);
}


bool H2_Logic_SetLUT(const char *name, float value)
{
    /* --- FNOP0 .. FNOP7 --- */
    if (name[0] == 'F' && name[1] == 'N' && name[2] == 'O' && name[3] == 'P') {
        int idx = name[4] - '0';
        if (idx >= 0 && idx < H2_LUT_SIZE) {
            FNOP_POINTS[idx] = value;
            return true;
        }
    }

    /* --- H2CN0 .. H2CN7 --- */
    if (name[0] == 'H' && name[1] == '2' && name[2] == 'C' && name[3] == 'N') {
        int idx = name[4] - '0';
        if (idx >= 0 && idx < H2_LUT_SIZE) {
            H2CN_POINTS[idx] = value;
            return true;
        }
    }

    return false;
}
