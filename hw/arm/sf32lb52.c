/*
 * SiFli SF32LB52 HCPU
 *
 * This is an intentionally small HCPU-only model. It provides the real memory
 * map, Cortex-M33 interrupt topology, startup clock handshakes and USART1.
 * Other peripheral accesses are retained in register-backed stub regions so
 * that firmware bring-up can proceed while -d unimp identifies missing models.
 *
 * Copyright (c) 2026 Core Devices LLC
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/timer.h"
#include "qemu/log.h"
#include "qemu/units.h"
#include "qapi/error.h"
#include "hw/arm/armv7m.h"
#include "hw/arm/boot.h"
#include "hw/boards.h"
#include "hw/irq.h"
#include "hw/qdev-clock.h"
#include "hw/qdev-properties-system.h"
#include "hw/sysbus.h"
#include "system/address-spaces.h"
#include "system/reset.h"
#include "system/system.h"
#include "qom/object.h"

#define TYPE_SF32LB52_MACHINE MACHINE_TYPE_NAME("sf32lb52")
OBJECT_DECLARE_SIMPLE_TYPE(SF32LB52MachineState, SF32LB52_MACHINE)

#define SF32LB52_HPSYS_ROM_BASE       0x00000000
#define SF32LB52_HPSYS_ROM_SIZE       (64 * KiB)
#define SF32LB52_HPSYS_RAM_BASE       0x20000000
#define SF32LB52_HPSYS_RAM_SIZE       (512 * KiB)
#define SF32LB52_LPSYS_RAM_BASE       0x20400000
#define SF32LB52_LPSYS_RAM_SIZE       (64 * KiB)
#define SF32LB52_FLASH_BASE           0x12000000
#define SF32LB52_FLASH_SIZE           (32 * MiB)
#define SF32LB52_SLOT0_VECTOR_OFFSET  0x21000
#define SF32LB52_PERIPH_SIZE          (1 * MiB)
#define SF32LB52_LPSYS_PERIPH_BASE    0x40000000
#define SF32LB52_HPSYS_PERIPH_BASE    0x50000000
#define SF32LB52_USART1_BASE          0x50084000
#define SF32LB52_USART1_IRQ           59
#define SF32LB52_I2C1_IRQ             61
#define SF32LB52_I2C2_IRQ             76
#define SF32LB52_I2C4_IRQ             78
#define SF32LB52_I2C3_IRQ             93
#define SF32LB52_DWT_BASE             0xe0001000
#define SF32LB52_DWT_SIZE             0x1000
#define SF32LB52_NUM_IRQS             96
#define SF32LB52_SYSCLK_HZ            240000000
#define SF32LB52_REFCLK_HZ            1000000

#define HPSYS_RCC_HRCCAL1             0x000034
#define HPSYS_RCC_HRCCAL2             0x000038
#define HPSYS_RCC_HRCCAL1_CAL_EN      (1U << 30)
#define HPSYS_RCC_HRCCAL1_CAL_DONE    (1U << 31)
#define HPSYS_RCC_DLL1CR              0x00002c
#define HPSYS_RCC_DLL2CR              0x000030
#define HPSYS_RCC_DLLCR_EN            (1U << 0)
#define HPSYS_RCC_DLLCR_READY         (1U << 31)
#define EFUSEC_CR                     0x00c000
#define EFUSEC_SR                     0x00c008
#define EFUSEC_CR_EN                  (1U << 0)
#define EFUSEC_SR_DONE                (1U << 0)
#define MPI2_CR                       0x042000
#define MPI2_DR                       0x042004
#define MPI2_SR                       0x042010
#define MPI2_SCR                      0x042014
#define MPI2_CMDR1                    0x042018
#define MPI2_AR1                      0x04201c
#define MPI_CR_CMD2E                  (1U << 16)
#define MPI_CR_SME2                   (1U << 18)
#define MPI_SR_TCF                    (1U << 0)
#define MPI_SR_SMF                    (1U << 3)
#define MPI_SCR_TCFC                  (1U << 0)
#define MPI_SCR_SMFC                  (1U << 3)
#define SPI_FLASH_CMD_RDID            0x9f
#define SPI_FLASH_ID_GD25Q256E        0x1940c8
#define TRNG_CTRL                     0x00f000
#define TRNG_STAT                     0x00f004
#define TRNG_RAND_SEED0               0x00f010
#define TRNG_RAND_NUM0                0x00f030
#define TRNG_CTRL_GEN_SEED_START      (1U << 0)
#define TRNG_CTRL_GEN_RAND_NUM_START  (1U << 1)
#define TRNG_STAT_SEED_VALID          (1U << 1)
#define TRNG_STAT_RAND_NUM_VALID      (1U << 3)
#define WDT1_CCR                      0x09400c
#define WDT1_SR                       0x094014
#define WDT_CMD_STOP                  0x34
#define WDT_CMD_START                 0x76
#define WDT_SR_ACTIVE                 (1U << 1)
#define DMAC1_ISR                     0x081000
#define DMAC1_IFCR                    0x081004
#define DMAC1_CCR1                    0x081008
#define DMAC1_CNDTR1                  0x08100c
#define DMAC1_CPAR1                   0x081010
#define DMAC1_CM0AR1                  0x081014
#define DMAC_CHANNEL_STRIDE           0x14
#define DMAC_CHANNEL_COUNT            8
#define DMAC_CCR_EN                   (1U << 0)
#define I2C1_CR                       0x09c000
#define I2C2_CR                       0x09d000
#define I2C3_CR                       0x09e000
#define I2C4_CR                       0x09f000
#define I2C_CR_RSTREQ                 (1U << 30)
#define I2C_REG_SIZE                  0x1000
#define I2C_TCR                       0x004
#define I2C_IER                       0x008
#define I2C_SR                        0x00c
#define I2C_DBR                       0x010
#define I2C_TCR_TB                    (1U << 0)
#define I2C_TCR_START                 (1U << 1)
#define I2C_TCR_STOP                  (1U << 2)
#define I2C_SR_TE                     (1U << 6)
#define I2C_SR_RF                     (1U << 7)
#define I2C_SR_MSD                    (1U << 12)
#define AUDCODEC_PLL_CFG0             0x088080
#define AUDCODEC_PLL_CAL_CFG          0x0880a4
#define AUDCODEC_PLL_CAL_RESULT       0x0880a8
#define AUDCODEC_PLL_CFG0_FC_VCO_SHIFT 17
#define AUDCODEC_PLL_CFG0_FC_VCO_MASK (0x1fU << 17)
#define AUDCODEC_PLL_CAL_CFG_EN       (1U << 0)
#define AUDCODEC_PLL_CAL_CFG_DONE     (1U << 1)
#define AUDCODEC_PLL_CAL_CFG_LEN_SHIFT 16
#define RTC_ISR                       0x0cb00c
#define RTC_ISR_RSF                   (1U << 7)
#define RTC_ISR_INITF                 (1U << 9)
#define RTC_ISR_INIT                  (1U << 10)
#define HPSYS_AON_ACR                 0x0c0010
#define HPSYS_AON_ISSR                0x0c002c
#define HPSYS_AON_ACR_HRC48_REQ       (1U << 0)
#define HPSYS_AON_ACR_HXT48_REQ       (1U << 1)
#define HPSYS_AON_ACR_HRC48_RDY       (1U << 30)
#define HPSYS_AON_ACR_HXT48_RDY       (1U << 31)
#define HPSYS_AON_ISSR_LP_ACTIVE      (1U << 5)
#define PMUC_LRC32_CR                  0x0ca01c
#define PMUC_LRC32_CR_EN              (1U << 0)
#define PMUC_LRC32_CR_RDY             (1U << 31)

#define DWT_CTRL                      0x000
#define DWT_CYCCNT                    0x004
#define DWT_CTRL_CYCCNTENA            (1U << 0)

typedef struct SF32LB52PeripheralRegion {
    MemoryRegion iomem;
    uint32_t *regs;
    const char *name;
    SF32LB52MachineState *machine;
} SF32LB52PeripheralRegion;

typedef struct SF32LB52I2CState {
    qemu_irq irq;
    uint8_t address;
    uint8_t reg;
    bool addressed;
    bool read;
    bool have_reg;
    uint8_t data[128][256];
} SF32LB52I2CState;

struct SF32LB52MachineState {
    MachineState parent_obj;
    ARMv7MState armv7m;
    MemoryRegion hpsys_rom;
    MemoryRegion hpsys_ram;
    MemoryRegion lpsys_ram;
    MemoryRegion flash;
    MemoryRegion boot_alias;
    MemoryRegion dwt;
    MemoryRegion dwt_ppb_alias;
    SF32LB52PeripheralRegion hpsys_periph;
    SF32LB52PeripheralRegion lpsys_periph;
    SF32LB52I2CState i2c[4];
    uint32_t dwt_ctrl;
    uint32_t dwt_cyccnt_base;
    uint32_t rng_state;
    uint32_t dmac_count[DMAC_CHANNEL_COUNT];
    uint32_t dmac_periph[DMAC_CHANNEL_COUNT];
    uint32_t dmac_memory[DMAC_CHANNEL_COUNT];
    int64_t dwt_cyccnt_base_ns;
    Clock *sysclk;
    Clock *refclk;
};

static int sf32lb52_i2c_index(hwaddr offset)
{
    static const hwaddr i2c_base[] = {
        I2C1_CR, I2C2_CR, I2C3_CR, I2C4_CR,
    };

    for (int i = 0; i < ARRAY_SIZE(i2c_base); i++) {
        if (offset >= i2c_base[i] && offset < i2c_base[i] + I2C_REG_SIZE) {
            return i;
        }
    }
    return -1;
}

static void sf32lb52_i2c_update_irq(SF32LB52MachineState *s, int index)
{
    static const hwaddr i2c_base[] = {
        I2C1_CR, I2C2_CR, I2C3_CR, I2C4_CR,
    };
    uint32_t *regs = s->hpsys_periph.regs;
    hwaddr base = i2c_base[index];

    qemu_set_irq(s->i2c[index].irq,
                 regs[(base + I2C_SR) / 4] & regs[(base + I2C_IER) / 4]);
}

static void sf32lb52_i2c_transfer(SF32LB52MachineState *s, int index,
                                  hwaddr base, uint32_t value)
{
    SF32LB52I2CState *i2c = &s->i2c[index];
    uint32_t *regs = s->hpsys_periph.regs;
    uint32_t status = I2C_SR_TE;

    if (!(value & I2C_TCR_TB)) {
        return;
    }
    if (value & I2C_TCR_START) {
        uint8_t address_byte = regs[(base + I2C_DBR) / 4];
        bool read = address_byte & 1;
        uint8_t address = address_byte >> 1;

        if (!read || !i2c->addressed || i2c->address != address) {
            i2c->have_reg = false;
        }
        i2c->address = address;
        i2c->addressed = true;
        i2c->read = read;
    } else if (i2c->addressed && i2c->read) {
        regs[(base + I2C_DBR) / 4] =
            i2c->data[i2c->address][i2c->reg++];
        status = I2C_SR_RF;
    } else if (i2c->addressed && !i2c->have_reg) {
        i2c->reg = regs[(base + I2C_DBR) / 4];
        i2c->have_reg = true;
    } else if (i2c->addressed) {
        i2c->data[i2c->address][i2c->reg++] =
            regs[(base + I2C_DBR) / 4];
    }
    if (value & I2C_TCR_STOP) {
        status |= I2C_SR_MSD;
        i2c->addressed = false;
    }
    regs[(base + I2C_SR) / 4] |= status;
    sf32lb52_i2c_update_irq(s, index);
}

static void sf32lb52_dmac_clear_flags(uint32_t *regs, uint32_t value)
{
    uint32_t status = regs[DMAC1_ISR / 4];

    for (int channel = 0; channel < DMAC_CHANNEL_COUNT; channel++) {
        uint32_t shift = channel * 4;
        uint32_t clear = (value >> shift) & 0xf;

        if (clear & 1) {
            status &= ~(0xfU << shift);
        } else {
            status &= ~(clear << shift);
            if (!(status & (0xeU << shift))) {
                status &= ~(1U << shift);
            }
        }
    }
    regs[DMAC1_ISR / 4] = status;
}

static bool sf32lb52_dmac_write(SF32LB52MachineState *s, uint32_t *regs,
                                hwaddr offset, uint32_t value,
                                uint32_t *result)
{
    if (offset == DMAC1_IFCR) {
        sf32lb52_dmac_clear_flags(regs, value);
        return true;
    }

    for (int channel = 0; channel < DMAC_CHANNEL_COUNT; channel++) {
        hwaddr ccr = DMAC1_CCR1 + channel * DMAC_CHANNEL_STRIDE;

        if (offset == ccr && (value & DMAC_CCR_EN)) {
            uint32_t shift = channel * 4;

            s->dmac_count[channel] =
                regs[(DMAC1_CNDTR1 + channel * DMAC_CHANNEL_STRIDE) / 4];
            s->dmac_periph[channel] =
                regs[(DMAC1_CPAR1 + channel * DMAC_CHANNEL_STRIDE) / 4];
            s->dmac_memory[channel] =
                regs[(DMAC1_CM0AR1 + channel * DMAC_CHANNEL_STRIDE) / 4];
            regs[DMAC1_ISR / 4] |= 3U << shift;
            regs[(DMAC1_CNDTR1 + channel * DMAC_CHANNEL_STRIDE) / 4] = 0;
            *result &= ~DMAC_CCR_EN;
            break;
        }
    }
    return false;
}

static void sf32lb52_flash_command(SF32LB52MachineState *s, uint32_t command)
{
    uint32_t *regs = s->hpsys_periph.regs;
    uint32_t address = regs[MPI2_AR1 / 4];
    uint8_t *flash = memory_region_get_ram_ptr(&s->flash);
    uint32_t erase_size = 0;

    switch (command & 0xff) {
    case 0x20:
    case 0x21:
        erase_size = 4 * KiB;
        break;
    case 0x52:
        erase_size = 32 * KiB;
        break;
    case 0xd8:
    case 0xdc:
        erase_size = 64 * KiB;
        break;
    case 0x02:
    case 0x12:
    case 0x32:
    case 0x34:
        for (int channel = 0; channel < DMAC_CHANNEL_COUNT; channel++) {
            uint32_t count = s->dmac_count[channel];

            if (s->dmac_periph[channel] ==
                    SF32LB52_HPSYS_PERIPH_BASE + MPI2_DR &&
                address < SF32LB52_FLASH_SIZE && count) {
                g_autofree uint8_t *data = g_malloc(count);
                MemTxResult tx = address_space_read(&address_space_memory,
                    s->dmac_memory[channel], MEMTXATTRS_UNSPECIFIED,
                    data, MIN(count, SF32LB52_FLASH_SIZE - address));

                if (tx == MEMTX_OK) {
                    count = MIN(count, SF32LB52_FLASH_SIZE - address);
                    for (uint32_t i = 0; i < count; i++) {
                        flash[address + i] &= data[i];
                    }
                }
                s->dmac_count[channel] = 0;
                break;
            }
        }
        break;
    default:
        break;
    }

    if (erase_size && address < SF32LB52_FLASH_SIZE) {
        address &= ~(erase_size - 1);
        memset(flash + address, 0xff,
               MIN(erase_size, SF32LB52_FLASH_SIZE - address));
    }
}

static uint32_t sf32lb52_dwt_cyccnt(SF32LB52MachineState *s)
{
    int64_t elapsed_ns;

    if (!(s->dwt_ctrl & DWT_CTRL_CYCCNTENA)) {
        return s->dwt_cyccnt_base;
    }
    elapsed_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) -
                 s->dwt_cyccnt_base_ns;
    return s->dwt_cyccnt_base + elapsed_ns / 1000 * 240;
}

static uint64_t sf32lb52_dwt_read(void *opaque, hwaddr offset,
                                  unsigned int size)
{
    SF32LB52MachineState *s = opaque;

    switch (offset) {
    case DWT_CTRL:
        return s->dwt_ctrl;
    case DWT_CYCCNT:
        return sf32lb52_dwt_cyccnt(s);
    default:
        return 0;
    }
}

static void sf32lb52_dwt_write(void *opaque, hwaddr offset, uint64_t value,
                               unsigned int size)
{
    SF32LB52MachineState *s = opaque;

    switch (offset) {
    case DWT_CTRL:
        s->dwt_cyccnt_base = sf32lb52_dwt_cyccnt(s);
        s->dwt_cyccnt_base_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        s->dwt_ctrl = value;
        break;
    case DWT_CYCCNT:
        s->dwt_cyccnt_base = value;
        s->dwt_cyccnt_base_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        break;
    default:
        break;
    }
}

static const MemoryRegionOps sf32lb52_dwt_ops = {
    .read = sf32lb52_dwt_read,
    .write = sf32lb52_dwt_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static uint64_t sf32lb52_peripheral_read(void *opaque, hwaddr offset,
                                         unsigned int size)
{
    SF32LB52PeripheralRegion *r = opaque;

    qemu_log_mask(LOG_UNIMP, "%s: register read at 0x%06" HWADDR_PRIx "\n",
                  r->name, offset);
    if (!strcmp(r->name, "sf32lb52.hpsys-peripherals") &&
        offset == MPI2_DR &&
        r->regs[MPI2_CMDR1 / 4] == SPI_FLASH_CMD_RDID) {
        return SPI_FLASH_ID_GD25Q256E;
    }
    return r->regs[offset / 4];
}

static void sf32lb52_peripheral_write(void *opaque, hwaddr offset,
                                      uint64_t value, unsigned int size)
{
    SF32LB52PeripheralRegion *r = opaque;
    SF32LB52MachineState *s = r->machine;
    uint32_t result = value;
    int i2c_index;

    qemu_log_mask(LOG_UNIMP,
                  "%s: register write at 0x%06" HWADDR_PRIx
                  " = 0x%08" PRIx64 "\n",
                  r->name, offset, value);

    if (!strcmp(r->name, "sf32lb52.hpsys-peripherals")) {
        if (sf32lb52_dmac_write(s, r->regs, offset, value, &result)) {
            return;
        }
        i2c_index = sf32lb52_i2c_index(offset);
        if (i2c_index >= 0) {
            hwaddr base = offset & ~(I2C_REG_SIZE - 1);

            switch (offset - base) {
            case I2C_IER:
                r->regs[offset / 4] = value;
                sf32lb52_i2c_update_irq(s, i2c_index);
                return;
            case I2C_SR:
                r->regs[offset / 4] &= ~value;
                sf32lb52_i2c_update_irq(s, i2c_index);
                return;
            case I2C_TCR:
                r->regs[offset / 4] = value;
                sf32lb52_i2c_transfer(s, i2c_index, base, value);
                return;
            default:
                break;
            }
        }
        switch (offset) {
        case EFUSEC_CR:
            if (value & EFUSEC_CR_EN) {
                r->regs[EFUSEC_SR / 4] |= EFUSEC_SR_DONE;
            }
            break;
        case MPI2_CMDR1:
            sf32lb52_flash_command(s, value);
            r->regs[MPI2_SR / 4] |= MPI_SR_TCF;
            if ((r->regs[MPI2_CR / 4] & (MPI_CR_CMD2E | MPI_CR_SME2)) ==
                (MPI_CR_CMD2E | MPI_CR_SME2)) {
                r->regs[MPI2_SR / 4] |= MPI_SR_SMF;
            }
            break;
        case MPI2_SCR:
            if (value & MPI_SCR_TCFC) {
                r->regs[MPI2_SR / 4] &= ~MPI_SR_TCF;
            }
            if (value & MPI_SCR_SMFC) {
                r->regs[MPI2_SR / 4] &= ~MPI_SR_SMF;
            }
            break;
        case HPSYS_RCC_DLL1CR:
        case HPSYS_RCC_DLL2CR:
            if (value & HPSYS_RCC_DLLCR_EN) {
                result |= HPSYS_RCC_DLLCR_READY;
            }
            break;
        case RTC_ISR:
            result |= RTC_ISR_RSF;
            if (value & RTC_ISR_INIT) {
                result |= RTC_ISR_INITF;
            }
            break;
        case TRNG_CTRL:
            s->rng_state ^= s->rng_state << 13;
            s->rng_state ^= s->rng_state >> 17;
            s->rng_state ^= s->rng_state << 5;
            if (value & TRNG_CTRL_GEN_SEED_START) {
                r->regs[TRNG_RAND_SEED0 / 4] = s->rng_state;
                r->regs[TRNG_STAT / 4] |= TRNG_STAT_SEED_VALID;
            }
            if (value & TRNG_CTRL_GEN_RAND_NUM_START) {
                r->regs[TRNG_RAND_NUM0 / 4] = s->rng_state;
                r->regs[TRNG_STAT / 4] |= TRNG_STAT_RAND_NUM_VALID;
            }
            break;
        case WDT1_CCR:
            if (value == WDT_CMD_START) {
                r->regs[WDT1_SR / 4] |= WDT_SR_ACTIVE;
            } else if (value == WDT_CMD_STOP) {
                r->regs[WDT1_SR / 4] &= ~WDT_SR_ACTIVE;
            }
            break;
        case I2C1_CR:
        case I2C2_CR:
        case I2C3_CR:
        case I2C4_CR:
            result &= ~I2C_CR_RSTREQ;
            s->i2c[i2c_index].addressed = false;
            s->i2c[i2c_index].have_reg = false;
            break;
        case AUDCODEC_PLL_CAL_CFG:
            if (value & AUDCODEC_PLL_CAL_CFG_EN) {
                uint32_t fc_vco =
                    (r->regs[AUDCODEC_PLL_CFG0 / 4] &
                     AUDCODEC_PLL_CFG0_FC_VCO_MASK) >>
                    AUDCODEC_PLL_CFG0_FC_VCO_SHIFT;
                uint32_t cal_len = value >> AUDCODEC_PLL_CAL_CFG_LEN_SHIFT;
                uint32_t pll_count = cal_len * (64 + fc_vco) / 80;

                r->regs[AUDCODEC_PLL_CAL_RESULT / 4] =
                    (pll_count << 16) | cal_len;
                result |= AUDCODEC_PLL_CAL_CFG_DONE;
            } else {
                result &= ~AUDCODEC_PLL_CAL_CFG_DONE;
            }
            break;
        case HPSYS_RCC_HRCCAL1:
            if (value & HPSYS_RCC_HRCCAL1_CAL_EN) {
                result |= HPSYS_RCC_HRCCAL1_CAL_DONE;
            }
            break;
        case HPSYS_AON_ACR:
            if (value & HPSYS_AON_ACR_HRC48_REQ) {
                result |= HPSYS_AON_ACR_HRC48_RDY;
            }
            if (value & HPSYS_AON_ACR_HXT48_REQ) {
                result |= HPSYS_AON_ACR_HXT48_RDY;
            }
            break;
        case PMUC_LRC32_CR:
            if (value & PMUC_LRC32_CR_EN) {
                result |= PMUC_LRC32_CR_RDY;
            }
            break;
        default:
            break;
        }
    }
    r->regs[offset / 4] = result;
}

static const MemoryRegionOps sf32lb52_peripheral_ops = {
    .read = sf32lb52_peripheral_read,
    .write = sf32lb52_peripheral_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void sf32lb52_reset(void *opaque)
{
    SF32LB52MachineState *s = opaque;

    s->dwt_ctrl = 0;
    s->dwt_cyccnt_base = 0;
    s->dwt_cyccnt_base_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    s->rng_state = 0x6d2b79f5;
    memset(s->dmac_count, 0, sizeof(s->dmac_count));
    memset(s->dmac_periph, 0, sizeof(s->dmac_periph));
    memset(s->dmac_memory, 0, sizeof(s->dmac_memory));
    memset(s->hpsys_periph.regs, 0, SF32LB52_PERIPH_SIZE);
    memset(s->lpsys_periph.regs, 0, SF32LB52_PERIPH_SIZE);
    for (int i = 0; i < ARRAY_SIZE(s->i2c); i++) {
        s->i2c[i].address = 0;
        s->i2c[i].reg = 0;
        s->i2c[i].addressed = false;
        s->i2c[i].read = false;
        s->i2c[i].have_reg = false;
        memset(s->i2c[i].data, 0, sizeof(s->i2c[i].data));
        qemu_irq_lower(s->i2c[i].irq);
    }
    s->hpsys_periph.regs[HPSYS_RCC_HRCCAL2 / 4] = 0x40004000;
    s->hpsys_periph.regs[HPSYS_AON_ACR / 4] =
        HPSYS_AON_ACR_HRC48_REQ | HPSYS_AON_ACR_HRC48_RDY;
    s->hpsys_periph.regs[HPSYS_AON_ISSR / 4] = HPSYS_AON_ISSR_LP_ACTIVE;
}

static void sf32lb52_init_peripheral_region(SF32LB52MachineState *s,
                                             SF32LB52PeripheralRegion *r,
                                             const char *name, hwaddr base)
{
    r->name = name;
    r->machine = s;
    r->regs = g_new0(uint32_t, SF32LB52_PERIPH_SIZE / sizeof(uint32_t));
    memory_region_init_io(&r->iomem, NULL, &sf32lb52_peripheral_ops, r,
                          name, SF32LB52_PERIPH_SIZE);
    memory_region_add_subregion(get_system_memory(), base, &r->iomem);
}

static void sf32lb52_machine_init(MachineState *machine)
{
    SF32LB52MachineState *s = SF32LB52_MACHINE(machine);
    MemoryRegion *system_memory = get_system_memory();
    DeviceState *armv7m;
    DeviceState *usart;
    SysBusDevice *usart_sbd;
    static const int i2c_irq[] = {
        SF32LB52_I2C1_IRQ,
        SF32LB52_I2C2_IRQ,
        SF32LB52_I2C3_IRQ,
        SF32LB52_I2C4_IRQ,
    };

    s->sysclk = clock_new(OBJECT(machine), "SYSCLK");
    clock_set_hz(s->sysclk, SF32LB52_SYSCLK_HZ);
    s->refclk = clock_new(OBJECT(machine), "REFCLK");
    clock_set_hz(s->refclk, SF32LB52_REFCLK_HZ);

    memory_region_init_ram(&s->hpsys_rom, NULL, "sf32lb52.hpsys-rom",
                           SF32LB52_HPSYS_ROM_SIZE, &error_fatal);
    memory_region_set_readonly(&s->hpsys_rom, true);
    memory_region_add_subregion(system_memory, SF32LB52_HPSYS_ROM_BASE,
                                &s->hpsys_rom);
    memory_region_init_ram(&s->hpsys_ram, NULL, "sf32lb52.hpsys-ram",
                           SF32LB52_HPSYS_RAM_SIZE, &error_fatal);
    memory_region_add_subregion(system_memory, SF32LB52_HPSYS_RAM_BASE,
                                &s->hpsys_ram);
    memory_region_init_ram(&s->lpsys_ram, NULL, "sf32lb52.lpsys-ram",
                           SF32LB52_LPSYS_RAM_SIZE, &error_fatal);
    memory_region_add_subregion(system_memory, SF32LB52_LPSYS_RAM_BASE,
                                &s->lpsys_ram);
    memory_region_init_ram(&s->flash, NULL, "sf32lb52.flash",
                           SF32LB52_FLASH_SIZE, &error_fatal);
    memset(memory_region_get_ram_ptr(&s->flash), 0xff, SF32LB52_FLASH_SIZE);
    memory_region_set_readonly(&s->flash, true);
    memory_region_add_subregion(system_memory, SF32LB52_FLASH_BASE, &s->flash);

    memory_region_init_alias(&s->boot_alias, NULL,
                             "sf32lb52.slot0-boot-alias", &s->flash,
                             SF32LB52_SLOT0_VECTOR_OFFSET,
                             SF32LB52_HPSYS_ROM_SIZE);
    memory_region_add_subregion_overlap(system_memory, SF32LB52_HPSYS_ROM_BASE,
                                        &s->boot_alias, 1);

    object_initialize_child(OBJECT(s), "armv7m", &s->armv7m, TYPE_ARMV7M);
    armv7m = DEVICE(&s->armv7m);
    qdev_prop_set_uint32(armv7m, "num-irq", SF32LB52_NUM_IRQS);
    qdev_prop_set_uint8(armv7m, "num-prio-bits", 3);
    qdev_prop_set_string(armv7m, "cpu-type", ARM_CPU_TYPE_NAME("cortex-m33"));
    qdev_connect_clock_in(armv7m, "cpuclk", s->sysclk);
    qdev_connect_clock_in(armv7m, "refclk", s->refclk);
    qdev_prop_set_bit(armv7m, "enable-bitband", false);
    object_property_set_link(OBJECT(&s->armv7m), "memory",
                             OBJECT(system_memory), &error_abort);
    sysbus_realize(SYS_BUS_DEVICE(&s->armv7m), &error_fatal);

    memory_region_init_io(&s->dwt, NULL, &sf32lb52_dwt_ops, s,
                          "sf32lb52.dwt", SF32LB52_DWT_SIZE);
    memory_region_add_subregion_overlap(system_memory, SF32LB52_DWT_BASE,
                                        &s->dwt, 1);
    memory_region_init_alias(&s->dwt_ppb_alias, NULL,
                             "sf32lb52.dwt-ppb-alias", &s->dwt, 0,
                             SF32LB52_DWT_SIZE);
    memory_region_add_subregion_overlap(&s->armv7m.container,
                                        SF32LB52_DWT_BASE,
                                        &s->dwt_ppb_alias, 0);

    sf32lb52_init_peripheral_region(s, &s->lpsys_periph,
                                    "sf32lb52.lpsys-peripherals",
                                    SF32LB52_LPSYS_PERIPH_BASE);
    sf32lb52_init_peripheral_region(s, &s->hpsys_periph,
                                    "sf32lb52.hpsys-peripherals",
                                    SF32LB52_HPSYS_PERIPH_BASE);
    for (int i = 0; i < ARRAY_SIZE(s->i2c); i++) {
        s->i2c[i].irq = qdev_get_gpio_in(armv7m, i2c_irq[i]);
    }

    usart = qdev_new("sf32lb52-usart");
    qdev_prop_set_chr(usart, "chardev", serial_hd(0));
    usart_sbd = SYS_BUS_DEVICE(usart);
    sysbus_realize_and_unref(usart_sbd, &error_fatal);
    memory_region_add_subregion_overlap(system_memory, SF32LB52_USART1_BASE,
                                        sysbus_mmio_get_region(usart_sbd, 0),
                                        1);
    sysbus_connect_irq(usart_sbd, 0,
                       qdev_get_gpio_in(armv7m, SF32LB52_USART1_IRQ));

    sf32lb52_reset(s);
    qemu_register_reset(sf32lb52_reset, s);
    armv7m_load_kernel(s->armv7m.cpu, machine->kernel_filename,
                       SF32LB52_FLASH_BASE, SF32LB52_FLASH_SIZE);
}

static void sf32lb52_machine_class_init(ObjectClass *oc, const void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);
    static const char * const valid_cpu_types[] = {
        ARM_CPU_TYPE_NAME("cortex-m33"),
        NULL,
    };

    mc->desc = "SiFli SF32LB52 HCPU (Cortex-M33)";
    mc->init = sf32lb52_machine_init;
    mc->max_cpus = 1;
    mc->default_cpu_type = ARM_CPU_TYPE_NAME("cortex-m33");
    mc->valid_cpu_types = valid_cpu_types;
    mc->ignore_memory_transaction_failures = false;
}

static const TypeInfo sf32lb52_machine_info = {
    .name = TYPE_SF32LB52_MACHINE,
    .parent = TYPE_MACHINE,
    .instance_size = sizeof(SF32LB52MachineState),
    .class_init = sf32lb52_machine_class_init,
};

static void sf32lb52_machine_register_types(void)
{
    type_register_static(&sf32lb52_machine_info);
}

type_init(sf32lb52_machine_register_types)
