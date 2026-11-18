#include "nvm.h"
#include "S32K116.h"
#include <string.h>

/* ------------------------------------------------------------------ */
/* Hardware Base Addresses                                            */
/* ------------------------------------------------------------------ */
#define FTFC_BASE        0x40020000U
#define CRC_BASE         0x40032000U

/* Target Flash Address: Last 2KB sector of the 128KB P-Flash */
#define NVM_FLASH_ADDR   0x0001F800U
#define NVM_SECTOR_SIZE  0x800U  /* 2 KB */

/* FTFC Register Offsets */
#define FTFC_FSTAT       (*(volatile uint32_t *)(FTFC_BASE + 0x00))
#define FTFC_FCCOB0      (*(volatile uint32_t *)(FTFC_BASE + 0x08))
#define FTFC_FCCOB1      (*(volatile uint32_t *)(FTFC_BASE + 0x0C))
#define FTFC_FCCOB2      (*(volatile uint32_t *)(FTFC_BASE + 0x10))
#define FTFC_FCCOB3      (*(volatile uint32_t *)(FTFC_BASE + 0x14))
#define FTFC_FCCOB4      (*(volatile uint32_t *)(FTFC_BASE + 0x18))
#define FTFC_FCCOB5      (*(volatile uint32_t *)(FTFC_BASE + 0x1C))
#define FTFC_FCCOB6      (*(volatile uint32_t *)(FTFC_BASE + 0x20))
#define FTFC_FCCOB7      (*(volatile uint32_t *)(FTFC_BASE + 0x24))

/* FTFC Status Bits */
#define FTFC_FSTAT_CCIF   (1U << 7)
#define FTFC_FSTAT_ACCERR (1U << 5)
#define FTFC_FSTAT_FPVIOL (1U << 4)

/* CRC Register Offsets */
#define CRC_CTRL         (*(volatile uint32_t *)(CRC_BASE + 0x00))
#define CRC_GPOLY        (*(volatile uint32_t *)(CRC_BASE + 0x04))
#define CRC_DATA         (*(volatile uint32_t *)(CRC_BASE + 0x08))

/* The exact structure saved to Flash (Padded to a multiple of 8 bytes) */
typedef struct __attribute__((aligned(8))) {
    uint32_t        magic;
    H2_LogicState_t consts;
    float           fnop_lut[8];
    float           h2cn_lut[8];
    uint32_t        crc;
    uint32_t        padding;
} NVM_Struct_t;

/* ------------------------------------------------------------------ */
/* Interrupt Control (Replaces missing CMSIS __disable_irq)           */
/* ------------------------------------------------------------------ */
static inline void nvm_disable_irq(void) {
    __asm volatile ("cpsid i" : : : "memory");
}

static inline void nvm_enable_irq(void) {
    __asm volatile ("cpsie i" : : : "memory");
}

/* ------------------------------------------------------------------ */
/* Hardware CRC-32 Calculation                                        */
/* ------------------------------------------------------------------ */
static uint32_t calculate_crc(const uint8_t *data, uint32_t len)
{
    CRC_CTRL = 0x00000000U;              /* Clear control */
    CRC_GPOLY = 0x04C11DB7U;             /* Standard CRC-32 polynomial */
    CRC_CTRL = 0x01000000U;              /* WAS = 1 (Write seed) */
    CRC_DATA = 0xFFFFFFFFU;              /* Seed value */
    CRC_CTRL = 0x00000000U;              /* WAS = 0 (Start data) */

    for (uint32_t i = 0; i < len; i++) {
        CRC_DATA = data[i];
    }
    return CRC_DATA;
}

/* ------------------------------------------------------------------ */
/* Bare-metal Flash Operations                                        */
/* ------------------------------------------------------------------ */
static bool flash_erase_sector(uint32_t addr)
{
    /* Clear any previous errors */
    FTFC_FSTAT = (FTFC_FSTAT_ACCERR | FTFC_FSTAT_FPVIOL);

    /* Write command (0x08 = Erase Flash Sector) and address */
    FTFC_FCCOB0 = 0x08U;
    FTFC_FCCOB1 = (addr >> 16) & 0xFFU;
    FTFC_FCCOB2 = (addr >> 8) & 0xFFU;
    FTFC_FCCOB3 = addr & 0xFFU;

    /* Launch command */
    FTFC_FSTAT = FTFC_FSTAT_CCIF;

    /* Wait for completion */
    while (!(FTFC_FSTAT & FTFC_FSTAT_CCIF)) { Watchdog_Feed(); }

    return !(FTFC_FSTAT & (FTFC_FSTAT_ACCERR | FTFC_FSTAT_FPVIOL));
}

static bool flash_program_phrase(uint32_t addr, const uint64_t *data)
{
    /* Clear any previous errors */
    FTFC_FSTAT = (FTFC_FSTAT_ACCERR | FTFC_FSTAT_FPVIOL);

    /* Write command (0x03 = Program Phrase) and address */
    FTFC_FCCOB0 = 0x03U;
    FTFC_FCCOB1 = (addr >> 16) & 0xFFU;
    FTFC_FCCOB2 = (addr >> 8) & 0xFFU;
    FTFC_FCCOB3 = addr & 0xFFU;

    /* Write 8 bytes of data */
    FTFC_FCCOB4 = (uint32_t)(*data & 0xFFFFFFFFU);
    FTFC_FCCOB5 = (uint32_t)(*data >> 32);

    /* Launch command */
    FTFC_FSTAT = FTFC_FSTAT_CCIF;

    /* Wait for completion */
    while (!(FTFC_FSTAT & FTFC_FSTAT_CCIF)) { Watchdog_Feed(); }

    return !(FTFC_FSTAT & (FTFC_FSTAT_ACCERR | FTFC_FSTAT_FPVIOL));
}

/* ------------------------------------------------------------------ */
/* Public API                                                         */
/* ------------------------------------------------------------------ */
void NVM_Init(H2_LogicState_t *state)
{
    NVM_Struct_t *nvm = (NVM_Struct_t *)NVM_FLASH_ADDR;

    if (nvm->magic == NVM_MAGIC) {
        uint32_t calc_crc = calculate_crc((const uint8_t *)&nvm->consts,
            (sizeof(NVM_Struct_t) - 8) /* Skip magic, crc, and padding */);

        if (calc_crc == nvm->crc) {
            /* Valid data: Restore Constants */
            *state = nvm->consts;

            /* Restore LUTs */
            char name[8];
            for (int i = 0; i < 8; i++) {
                name[0]='F'; name[1]='N'; name[2]='O'; name[3]='P';
                name[4]='0'+i; name[5]='\0';
                H2_Logic_SetLUT(name, nvm->fnop_lut[i]);

                name[0]='H'; name[1]='2'; name[2]='C'; name[3]='N';
                H2_Logic_SetLUT(name, nvm->h2cn_lut[i]);
            }
            return;
        }
    }

    /* Fallback Defaults (if Flash is blank or corrupted) */
    state->HSLO          = H2_DEF_HSLO;      state->ATMP          = H2_DEF_ATMP;
    state->TCOM          = H2_DEF_TCOM;      state->MERR          = H2_DEF_MERR;
    state->WAVO_INIT     = H2_DEF_WAVO_INIT; state->FILTER_JUMP   = H2_DEF_FILTER_JUMP;
    state->FILTER_ALPHA  = H2_DEF_FILTER_ALPHA; state->SDTH_MIN   = H2_DEF_SDTH_MIN;
    state->SDTH_MULT     = H2_DEF_SDTH_MULT; state->TREND_THRESH  = H2_DEF_TREND_THRESH;
    state->H2_POS_THRESH = H2_DEF_H2_POS_THRESH; state->N2_NEG_THRESH = H2_DEF_N2_NEG_THRESH;
    state->TARE_COUNT    = H2_DEF_TARE_COUNT; state->TEMP_FAULT   = H2_DEF_TEMP_FAULT;
    state->REF_TEMP      = H2_DEF_REF_TEMP;
}

bool NVM_Save(const H2_LogicState_t *state)
{
    NVM_Struct_t nvm;
    memset(&nvm, 0xFF, sizeof(nvm));

    nvm.magic  = NVM_MAGIC;
    nvm.consts = *state;

    /* Copy LUTs */
    float *fn = NULL; float *hc = NULL; int n = 0;
    get_lut_data(&fn, &hc, &n);
    for (int i = 0; i < 8; i++) {
        nvm.fnop_lut[i] = fn[i];
        nvm.h2cn_lut[i] = hc[i];
    }

    /* Compute CRC over the payload (excluding magic, crc, padding) */
    nvm.crc = calculate_crc((const uint8_t *)&nvm.consts,
        (sizeof(NVM_Struct_t) - 8));

    /* 1. Erase Sector */
    nvm_disable_irq();
    if (!flash_erase_sector(NVM_FLASH_ADDR)) {
        nvm_enable_irq();
        return false;
    }

    /* 2. Program Phrases (8 bytes at a time) */
    const uint64_t *src = (const uint64_t *)&nvm;
    uint32_t addr = NVM_FLASH_ADDR;

    for (uint32_t i = 0; i < (sizeof(NVM_Struct_t) / 8); i++) {
        if (!flash_program_phrase(addr, &src[i])) {
            nvm_enable_irq();
            return false;
        }
        addr += 8;
    }

    nvm_enable_irq();
    return true;
}
