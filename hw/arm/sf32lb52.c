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
#include "qemu/cutils.h"
#include "qemu/error-report.h"
#include "qemu/timer.h"
#include "qemu/log.h"
#include "qemu/units.h"
#include "qapi/error.h"
#include "hw/arm/armv7m.h"
#include "hw/arm/boot.h"
#include "hw/boards.h"
#include "hw/arm/pebble_gpio.h"
#include "hw/display/pebble_display.h"
#include "hw/input/pebble_touch.h"
#include "hw/irq.h"
#include "hw/qdev-clock.h"
#include "hw/qdev-properties-system.h"
#include "hw/sysbus.h"
#include "system/address-spaces.h"
#include "system/block-backend.h"
#include "system/blockdev.h"
#include "system/reset.h"
#include "system/rtc.h"
#include "system/runstate.h"
#include "system/system.h"
#include "qom/object.h"

#define TYPE_SF32LB52_MACHINE MACHINE_TYPE_NAME("sf32lb52")
OBJECT_DECLARE_SIMPLE_TYPE(SF32LB52MachineState, SF32LB52_MACHINE)
#define TYPE_SF32LB52_FLASH "sf32lb52-flash"
OBJECT_DECLARE_SIMPLE_TYPE(SF32LB52FlashState, SF32LB52_FLASH)

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
#define SF32LB52_GPTIM2_IRQ           71
#define SF32LB52_LPTIM1_IRQ           46
#define SF32LB52_RTC_IRQ              49
#define SF32LB52_LCDC1_IRQ            63
#define SF32LB52_GPIO1_IRQ            84
#define SF32LB52_I2C1_IRQ             61
#define SF32LB52_I2C2_IRQ             76
#define SF32LB52_I2C4_IRQ             78
#define SF32LB52_I2C3_IRQ             93
#define SF32LB52_LCPU2HCPU_IRQ        58
#define SF32LB52_DMAC1_CH1_IRQ        50
#define SF32LB52_DWT_BASE             0xe0001000
#define SF32LB52_DWT_SIZE             0x1000
#define SF32LB52_NUM_IRQS             96
#define SF32LB52_SYSCLK_HZ            240000000
#define SF32LB52_REFCLK_HZ            1000000

#define BT_RFC_BASE                   0x082800
#define BT_RFC_VCO_REG1               (BT_RFC_BASE + 0x00)
#define BT_RFC_VCO_REG3               (BT_RFC_BASE + 0x08)
#define BT_RFC_FBDV_REG1              (BT_RFC_BASE + 0x14)
#define BT_RFC_FBDV_REG2              (BT_RFC_BASE + 0x18)
#define BT_RFC_EDR_CAL_REG1           (BT_RFC_BASE + 0x24)
#define BT_RFC_ROSCAL_REG2            (BT_RFC_BASE + 0x80)
#define BT_RFC_RCROSCAL_REG           (BT_RFC_BASE + 0x84)
#define BT_RFC_PACAL_REG              (BT_RFC_BASE + 0x88)
#define BT_PHY_TX_HFP_CFG             0x084118
#define BT_RFC_FKCAL_READY            (1U << 0)
#define BT_RFC_FKCAL_ENABLE           (1U << 2)
#define BT_RFC_VCO3G_ENABLE           (1U << 13)
#define BT_RFC_ROSCAL_DONE            (1U << 0)
#define BT_RFC_RCCAL_DONE             (1U << 20)
#define BT_RFC_RCCAL_CAPCODE          (16U << 26)
#define BT_RFC_PACAL_DONE             (1U << 1)
#define BT_PHY_HFP_FCW_SHIFT          22
#define BT_PHY_HFP_FCW_MASK           0x3f
#define GPADC_IRQ                     0x087044
#define GPADC_IRQ_READY               (1U << 2)
#define DMAC2_ISR                     0x001000
#define DMAC2_ISR_TCIF8               (1U << 29)
#define LPSYS_AON_ACR                 0x040010
#define LPSYS_AON_ACR_HXT48_REQ       (1U << 1)
#define LPSYS_AON_ACR_HXT48_RDY       (1U << 31)
#define BT_MAC_RCCAL_CTRL             0x090114
#define BT_MAC_RCCAL_RESULT           0x090118
#define BT_MAC_RCCAL_DONE             (1U << 31)

#define HPSYS_RCC_HRCCAL1             0x000034
#define HPSYS_RCC_HRCCAL2             0x000038
#define HPSYS_RCC_RSTR1               0x000000
#define HPSYS_RCC_RSTR2               0x000004
#define HPSYS_RCC_ENR1                0x000008
#define HPSYS_RCC_ENR2                0x00000c
#define HPSYS_RCC_ESR1                0x000010
#define HPSYS_RCC_ESR2                0x000014
#define HPSYS_RCC_ECR1                0x000018
#define HPSYS_RCC_ECR2                0x00001c
#define HPSYS_RCC_DMAC1               (1U << 0)
#define HPSYS_RCC_LCDC1               (1U << 7)
#define HPSYS_RCC_GPTIM2              (1U << 16)
#define HPSYS_RCC_I2C1                (1U << 27)
#define HPSYS_RCC_I2C2                (1U << 28)
#define HPSYS_RCC_GPIO1               (1U << 0)
#define HPSYS_RCC_MPI2                (1U << 2)
#define HPSYS_RCC_I2C3                (1U << 8)
#define HPSYS_RCC_I2C4                (1U << 25)
#define HPSYS_RCC_HRCCAL1_CAL_EN      (1U << 30)
#define HPSYS_RCC_HRCCAL1_CAL_DONE    (1U << 31)
#define HPSYS_RCC_DLL1CR              0x00002c
#define HPSYS_RCC_DLL2CR              0x000030
#define HPSYS_RCC_DLLCR_EN            (1U << 0)
#define HPSYS_RCC_DLLCR_READY         (1U << 31)
#define EFUSEC_CR                     0x00c000
#define EFUSEC_SR                     0x00c008
#define EFUSEC_BANK0_DATA0            0x00c030
#define EFUSEC_BANK0_DATA1            0x00c034
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
#define MPI_SR_BUSY                   (1U << 31)
#define MPI_SCR_TCFC                  (1U << 0)
#define MPI_SCR_SMFC                  (1U << 3)
#define SPI_FLASH_CMD_RDID            0x9f
#define SPI_FLASH_ID_GD25Q256E        0x1940c8
#define SPI_FLASH_COMMAND_NS          1000
#define SPI_FLASH_PAGE_PROGRAM_NS     (700 * 1000)
#define SPI_FLASH_SECTOR_ERASE_NS     (45 * 1000 * 1000)
#define SPI_FLASH_BLOCK_ERASE_NS      (150 * 1000 * 1000)
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
#define DMAC_CCR_TCIE                 (1U << 1)
#define DMAC_CCR_HTIE                 (1U << 2)
#define DMAC_CCR_DIR                  (1U << 4)
#define DMAC_CCR_CIRC                 (1U << 5)
#define DMAC_CCR_MINC                 (1U << 7)
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
#define GPIO1_BASE                    0x0a0000
#define GPIO_BANK_STRIDE              0x80
#define GPIO_DIR                      0x00
#define GPIO_IER                      0x1c
#define GPIO_IESR                     0x20
#define GPIO_IECR                     0x24
#define GPIO_ITR                      0x28
#define GPIO_ITSR                     0x2c
#define GPIO_ITCR                     0x30
#define GPIO_IPHR                     0x34
#define GPIO_IPHSR                    0x38
#define GPIO_IPHCR                    0x3c
#define GPIO_IPLR                     0x40
#define GPIO_IPLSR                    0x44
#define GPIO_IPLCR                    0x48
#define GPIO_ISR                      0x4c
#define OBELIX_TOUCH_INT_PIN          27
#define OBELIX_BUTTON_PIN_BASE        34
#define OBELIX_BUTTON_COUNT           4
#define AW2016_ADDRESS                0x64
#define AW2016_CHIP_ID_REG            0x00
#define AW2016_CHIP_ID                0x09
#define W1160_ADDRESS                 0x48
#define W1160_FLAG1_REG               0x10
#define W1160_FLAG_ALS_READY          (1U << 7)
#define W1160_DATA1_ALS_REG           0x13
#define W1160_DATA2_ALS_REG           0x14
#define W1160_CHIP_ID_REG             0x3e
#define W1160_CHIP_ID                 0xe5
#define LSM6DSO_ADDRESS               0x6a
#define LSM6DSO_WHO_AM_I_REG          0x0f
#define LSM6DSO_WHO_AM_I              0x6c
#define LSM6DSO_CTRL3_C_REG           0x12
#define LSM6DSO_SW_RESET              (1U << 0)
#define LSM6DSO_CTRL1_XL_REG          0x10
#define LSM6DSO_STATUS_REG            0x1e
#define LSM6DSO_OUTX_L_A_REG          0x28
#define LSM6DSO_FIFO_CTRL1_REG        0x07
#define LSM6DSO_FIFO_CTRL2_REG        0x08
#define LSM6DSO_FIFO_CTRL3_REG        0x09
#define LSM6DSO_FIFO_CTRL4_REG        0x0a
#define LSM6DSO_FIFO_STATUS1_REG      0x3a
#define LSM6DSO_FIFO_STATUS2_REG      0x3b
#define LSM6DSO_FIFO_DATA_REG         0x78
#define LSM6DSO_FIFO_MODE_STREAM      0x06
#define LSM6DSO_FIFO_WTM              (1U << 7)
#define LSM6DSO_INT1_PIN              38
#define MMC5603_ADDRESS               0x30
#define MMC5603_STATUS1_REG           0x18
#define MMC5603_MEAS_READY            (1U << 6)
#define MMC5603_OTP_READY             (1U << 4)
#define MMC5603_WHO_AM_I_REG          0x39
#define MMC5603_WHO_AM_I              0x10
#define CST816_ADDRESS                0x15
#define CST816_CHIP_ID_REG            0xa7
#define CST816_CHIP_ID                0xb6
#define CST816_FW_VERSION_REG         0xa9
#define CST816_FW_VERSION             0x07
#define AUDCODEC_PLL_CFG0             0x088080
#define AUDCODEC_PLL_CAL_CFG          0x0880a4
#define AUDCODEC_PLL_CAL_RESULT       0x0880a8
#define AUDCODEC_PLL_CFG0_FC_VCO_SHIFT 17
#define AUDCODEC_PLL_CFG0_FC_VCO_MASK (0x1fU << 17)
#define AUDCODEC_PLL_CAL_CFG_EN       (1U << 0)
#define AUDCODEC_PLL_CAL_CFG_DONE     (1U << 1)
#define AUDCODEC_PLL_CAL_CFG_LEN_SHIFT 16
#define AUDCODEC_DAC_CH0_ENTRY        0x088050
#define PDM1_DATA_L                   0x09a040
#define HPSYS_CFG_RTC_TR              0x00b014
#define HPSYS_CFG_RTC_DR              0x00b018
#define RTC_TR                        0x0cb000
#define RTC_DR                        0x0cb004
#define RTC_CR                        0x0cb008
#define RTC_ISR                       0x0cb00c
#define RTC_ALRMTR                    0x0cb018
#define RTC_ALRMDR                    0x0cb01c
#define RTC_BKP0R                     0x0cb030
#define RTC_BACKUP_REGISTER_COUNT     10
#define RTC_CR_ALRME                  (1U << 8)
#define RTC_CR_ALRMIE                 (1U << 11)
#define RTC_ISR_ALRMWF                (1U << 0)
#define RTC_ISR_ALRMF                 (1U << 1)
#define RTC_ISR_WUTWF                 (1U << 2)
#define RTC_ISR_RSF                   (1U << 7)
#define RTC_ISR_INITF                 (1U << 9)
#define RTC_ISR_INIT                  (1U << 10)
#define RTC_ALRMDR_MSKS               (1U << 24)
#define RTC_ALRMDR_MSKMN              (1U << 25)
#define RTC_ALRMDR_MSKH               (1U << 26)
#define RTC_ALRMDR_MSKD               (1U << 27)
#define RTC_ALRMDR_MSKM               (1U << 28)
#define RTC_ALRMDR_MSKWD              (1U << 29)
#define LPTIM1_BASE                   0x0c1000
#define LPTIM_ISR                     (LPTIM1_BASE + 0x00)
#define LPTIM_ICR                     (LPTIM1_BASE + 0x04)
#define LPTIM_IER                     (LPTIM1_BASE + 0x08)
#define LPTIM_CR                      (LPTIM1_BASE + 0x10)
#define LPTIM_ARR                     (LPTIM1_BASE + 0x18)
#define LPTIM_CNT                     (LPTIM1_BASE + 0x1c)
#define LPTIM_ISR_OF                  (1U << 1)
#define LPTIM_ISR_OFWKUP              (1U << 9)
#define LPTIM_IER_OFIE                (1U << 1)
#define LPTIM_CR_ENABLE               (1U << 0)
#define LPTIM_CR_SNGSTRT              (1U << 1)
#define LPTIM_CR_CNTSTRT              (1U << 2)
#define LPTIM_CR_COUNTRST             (1U << 3)
#define LPTIM_FREQUENCY_HZ            10000
#define PMUC_CR                       0x0ca000
#define PMUC_CR_REBOOT                (1U << 2)
#define GPTIM2_BASE                   0x0b0000
#define GPTIM_CR1                     (GPTIM2_BASE + 0x00)
#define GPTIM_DIER                    (GPTIM2_BASE + 0x0c)
#define GPTIM_SR                      (GPTIM2_BASE + 0x10)
#define GPTIM_EGR                     (GPTIM2_BASE + 0x14)
#define GPTIM_PSC                     (GPTIM2_BASE + 0x28)
#define GPTIM_ARR                     (GPTIM2_BASE + 0x2c)
#define GPTIM_CR1_CEN                 (1U << 0)
#define GPTIM_DIER_UIE                (1U << 0)
#define GPTIM_SR_UIF                  (1U << 0)
#define GPTIM_EGR_UG                  (1U << 0)
#define GPTIM_CLOCK_HZ                24000000
#define LCDC1_BASE                    0x008000
#define LCDC_IRQ                      (LCDC1_BASE + 0x008)
#define LCDC_SETTING                  (LCDC1_BASE + 0x00c)
#define LCDC_CANVAS_TL_POS            (LCDC1_BASE + 0x010)
#define LCDC_CANVAS_BR_POS            (LCDC1_BASE + 0x014)
#define LCDC_LAYER0_SRC               (LCDC1_BASE + 0x02c)
#define LCDC_JDI_PAR_CTRL             (LCDC1_BASE + 0x0ec)
#define LCDC_JDI_PAR_CTRL_ENABLE      (1U << 0)
#define LCDC_IRQ_EOF_STAT             (1U << 0)
#define LCDC_IRQ_JDI_STAT             (1U << 4)
#define LCDC_IRQ_EOF_RAW_STAT         (1U << 16)
#define LCDC_IRQ_JDI_RAW_STAT         (1U << 20)
#define HPSYS_AON_ACR                 0x0c0010
#define HPSYS_AON_ISSR                0x0c002c
#define HPSYS_AON_GTIMR               0x0c0034
#define HPSYS_AON_ACR_HRC48_REQ       (1U << 0)
#define HPSYS_AON_ACR_HXT48_REQ       (1U << 1)
#define HPSYS_AON_ACR_HRC48_RDY       (1U << 30)
#define HPSYS_AON_ACR_HXT48_RDY       (1U << 31)
#define HPSYS_AON_ISSR_LP_ACTIVE      (1U << 5)
#define PMUC_LRC32_CR                  0x0ca01c
#define PMUC_LRC32_CR_EN              (1U << 0)
#define PMUC_LRC32_CR_RDY             (1U << 31)
#define MAILBOX1_BASE                 0x082000
#define MAILBOX2_BASE                 0x002000
#define MAILBOX_C1IER                 0x00
#define MAILBOX_C1ITR                 0x04
#define MAILBOX_C1ICR                 0x08
#define MAILBOX_C1ISR                 0x0c
#define MAILBOX_C1MISR                0x10
#define HCPU2LCPU_RING                0x2007fe00
#define LCPU2HCPU_RING                0x20405c00
#define IPC_RING_SIZE                 512
#define IPC_RING_HEADER_SIZE          20
#define HCI_COMMAND_PACKET            0x01
#define HCI_EVENT_PACKET              0x04
#define NPM1300_ADDRESS               0x6b
#define NPM1300_EVENTS_ADC            0x0003
#define NPM1300_VBUS_STATUS           0x0207
#define NPM1300_CHARGE_STATUS         0x0334
#define NPM1300_IBAT_STATUS           0x0510
#define NPM1300_VBAT_MSB              0x0511
#define NPM1300_NTC_MSB               0x0512
#define NPM1300_GP0_LSBS              0x0515
#define NPM1300_VBAT2_MSB             0x0518
#define NPM1300_GP1_LSBS              0x051a

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
    uint16_t reg;
    uint8_t reg_bytes;
    bool addressed;
    bool read;
    bool have_reg;
    uint8_t data[128][256];
} SF32LB52I2CState;

typedef struct SF32LB52DMATimer {
    SF32LB52MachineState *machine;
    unsigned int channel;
} SF32LB52DMATimer;

struct SF32LB52FlashState {
    DeviceState parent_obj;
    BlockBackend *blk;
};

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
    DeviceState *display;
    DeviceState *buttons;
    DeviceState *touch;
    qemu_irq gpio1_irq;
    qemu_irq lcdc_irq;
    qemu_irq gptim2_irq;
    qemu_irq lptim1_irq;
    qemu_irq rtc_irq;
    qemu_irq lcpu2hcpu_irq;
    qemu_irq dmac1_irq[DMAC_CHANNEL_COUNT];
    QEMUTimer *lcdc_timer;
    QEMUTimer *gptim2_timer;
    QEMUTimer *lptim1_timer;
    QEMUTimer *mpi2_timer;
    QEMUTimer *rtc_alarm_timer;
    QEMUTimer *lsm6dso_timer;
    QEMUTimer *hci_ready_timer;
    QEMUTimer *dmac_timer[DMAC_CHANNEL_COUNT];
    SF32LB52DMATimer dmac_timer_context[DMAC_CHANNEL_COUNT];
    BlockBackend *flash_blk;
    uint32_t flash_backed_size;
    uint32_t mpi2_command;
    uint32_t mpi2_address;
    uint8_t lcdc_phase;
    uint32_t dwt_ctrl;
    uint32_t dwt_cyccnt_base;
    uint32_t rng_state;
    uint32_t dmac_count[DMAC_CHANNEL_COUNT];
    uint32_t dmac_periph[DMAC_CHANNEL_COUNT];
    uint32_t dmac_memory[DMAC_CHANNEL_COUNT];
    bool dmac_second_half[DMAC_CHANNEL_COUNT];
    uint16_t lsm6dso_fifo_samples;
    uint16_t lsm6dso_fifo_byte;
    bool lsm6dso_fifo_reading;
    uint8_t hci_command[260];
    uint8_t npm1300[0x1000];
    uint16_t hci_command_len;
    bool hci_ready;
    int64_t dwt_cyccnt_base_ns;
    int64_t rtc_base_ns;
    int64_t rtc_base_seconds;
    int64_t lptim1_start_ns;
    bool rtc_initialized;
    Clock *sysclk;
    Clock *refclk;
};

static uint32_t *sf32lb52_gpio_reg(SF32LB52MachineState *s, unsigned int pin,
                                   hwaddr reg)
{
    hwaddr offset = GPIO1_BASE + (pin / 32) * GPIO_BANK_STRIDE + reg;

    return &s->hpsys_periph.regs[offset / 4];
}

static bool sf32lb52_hpsys_clock_enabled(SF32LB52MachineState *s,
                                         unsigned int group, uint32_t mask)
{
    hwaddr enr = group == 1 ? HPSYS_RCC_ENR1 : HPSYS_RCC_ENR2;

    return s->hpsys_periph.regs[enr / 4] & mask;
}

static void sf32lb52_gpio_update_irq(SF32LB52MachineState *s)
{
    uint32_t pending = 0;

    if (!sf32lb52_hpsys_clock_enabled(s, 2, HPSYS_RCC_GPIO1)) {
        qemu_irq_lower(s->gpio1_irq);
        return;
    }

    for (int bank = 0; bank < 2; bank++) {
        uint32_t *ier = sf32lb52_gpio_reg(s, bank * 32, GPIO_IER);
        uint32_t *isr = sf32lb52_gpio_reg(s, bank * 32, GPIO_ISR);

        pending |= *ier & *isr;
    }
    qemu_set_irq(s->gpio1_irq, pending != 0);
}

static void sf32lb52_gpio_input(SF32LB52MachineState *s, unsigned int pin,
                                bool high)
{
    uint32_t mask = 1U << (pin % 32);
    uint32_t *dir = sf32lb52_gpio_reg(s, pin, GPIO_DIR);
    bool old_high = (*dir & mask) != 0;
    uint32_t *itr = sf32lb52_gpio_reg(s, pin, GPIO_ITR);
    uint32_t *iphr = sf32lb52_gpio_reg(s, pin, GPIO_IPHR);
    uint32_t *iplr = sf32lb52_gpio_reg(s, pin, GPIO_IPLR);
    uint32_t *isr = sf32lb52_gpio_reg(s, pin, GPIO_ISR);

    if (high) {
        *dir |= mask;
    } else {
        *dir &= ~mask;
    }
    if (old_high != high && (*itr & mask) &&
        ((high && (*iphr & mask)) || (!high && (*iplr & mask)))) {
        *isr |= mask;
    }
    sf32lb52_gpio_update_irq(s);
}

static bool sf32lb52_gpio_write(SF32LB52MachineState *s, hwaddr offset,
                                uint32_t value)
{
    hwaddr gpio_offset;
    hwaddr reg;
    unsigned int bank;
    uint32_t *regs = s->hpsys_periph.regs;
    uint32_t *target;

    if (offset < GPIO1_BASE ||
        offset >= GPIO1_BASE + 2 * GPIO_BANK_STRIDE) {
        return false;
    }
    gpio_offset = offset - GPIO1_BASE;
    bank = gpio_offset / GPIO_BANK_STRIDE;
    reg = gpio_offset % GPIO_BANK_STRIDE;
    target = &regs[(GPIO1_BASE + bank * GPIO_BANK_STRIDE) / 4];

    switch (reg) {
    case GPIO_IESR:
        target[GPIO_IER / 4] |= value;
        break;
    case GPIO_IECR:
        target[GPIO_IER / 4] &= ~value;
        break;
    case GPIO_ITSR:
        target[GPIO_ITR / 4] |= value;
        break;
    case GPIO_ITCR:
        target[GPIO_ITR / 4] &= ~value;
        break;
    case GPIO_IPHSR:
        target[GPIO_IPHR / 4] |= value;
        break;
    case GPIO_IPHCR:
        target[GPIO_IPHR / 4] &= ~value;
        break;
    case GPIO_IPLSR:
        target[GPIO_IPLR / 4] |= value;
        break;
    case GPIO_IPLCR:
        target[GPIO_IPLR / 4] &= ~value;
        break;
    case GPIO_ISR:
        target[GPIO_ISR / 4] &= ~value;
        break;
    default:
        regs[offset / 4] = value;
        break;
    }
    sf32lb52_gpio_update_irq(s);
    return true;
}

static void sf32lb52_button_state(void *opaque, uint32_t state)
{
    SF32LB52MachineState *s = opaque;

    for (int button = 0; button < OBELIX_BUTTON_COUNT; button++) {
        bool pressed = state & (1U << button);

        sf32lb52_gpio_input(s, OBELIX_BUTTON_PIN_BASE + button,
                            button == 0 ? pressed : !pressed);
    }
}

static void sf32lb52_touch_state(void *opaque, bool down,
                                 uint32_t x, uint32_t y)
{
    SF32LB52MachineState *s = opaque;
    uint8_t *data = s->i2c[2].data[CST816_ADDRESS];

    data[0x01] = 0;
    data[0x02] = down ? 1 : 0;
    data[0x03] = (x >> 8) & 0xf;
    data[0x04] = x;
    data[0x05] = (y >> 8) & 0xf;
    data[0x06] = y;

    sf32lb52_gpio_input(s, OBELIX_TOUCH_INT_PIN, false);
    sf32lb52_gpio_input(s, OBELIX_TOUCH_INT_PIN, true);
}

static uint16_t sf32lb52_lsm6dso_watermark(SF32LB52MachineState *s)
{
    uint8_t *data = s->i2c[1].data[LSM6DSO_ADDRESS];

    return data[LSM6DSO_FIFO_CTRL1_REG] |
           (data[LSM6DSO_FIFO_CTRL2_REG] & 1) << 8;
}

static void sf32lb52_lsm6dso_update_irq(SF32LB52MachineState *s)
{
    uint16_t watermark = sf32lb52_lsm6dso_watermark(s);

    sf32lb52_gpio_input(s, LSM6DSO_INT1_PIN,
                        watermark && s->lsm6dso_fifo_samples >= watermark);
}

static uint64_t sf32lb52_lsm6dso_sample_period(SF32LB52MachineState *s)
{
    uint8_t *data = s->i2c[1].data[LSM6DSO_ADDRESS];
    static const uint64_t sample_period_ns[] = {
        0, 80000000, 38461538, 19230769, 9615384, 4807692,
        2403846, 1200480, 600240, 300030, 150015,
    };
    unsigned int odr = data[LSM6DSO_CTRL1_XL_REG] >> 4;

    return odr < ARRAY_SIZE(sample_period_ns) ? sample_period_ns[odr] : 0;
}

static void sf32lb52_lsm6dso_sample(void *opaque)
{
    SF32LB52MachineState *s = opaque;
    uint8_t *data = s->i2c[1].data[LSM6DSO_ADDRESS];
    uint64_t period = sf32lb52_lsm6dso_sample_period(s);

    if ((data[LSM6DSO_FIFO_CTRL4_REG] & 7) != LSM6DSO_FIFO_MODE_STREAM ||
        !period) {
        return;
    }
    s->lsm6dso_fifo_samples = MIN(s->lsm6dso_fifo_samples + 1, 511);
    sf32lb52_lsm6dso_update_irq(s);
    timer_mod(s->lsm6dso_timer,
              qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + period);
}

static uint8_t sf32lb52_lsm6dso_read(SF32LB52MachineState *s, uint8_t reg)
{
    uint8_t *data = s->i2c[1].data[LSM6DSO_ADDRESS];
    static const uint8_t stationary_sample[7] = {
        0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40,
    };

    switch (reg) {
    case LSM6DSO_STATUS_REG:
        return 1;
    case LSM6DSO_FIFO_STATUS1_REG:
        return s->lsm6dso_fifo_samples;
    case LSM6DSO_FIFO_STATUS2_REG:
        return (s->lsm6dso_fifo_samples >> 8) |
               (s->lsm6dso_fifo_samples >=
                sf32lb52_lsm6dso_watermark(s) ? LSM6DSO_FIFO_WTM : 0);
    default:
        if (reg >= LSM6DSO_FIFO_DATA_REG) {
            uint8_t value = stationary_sample[s->lsm6dso_fifo_byte++ % 7];

            if (!(s->lsm6dso_fifo_byte % 7) && s->lsm6dso_fifo_samples) {
                s->lsm6dso_fifo_samples--;
                sf32lb52_lsm6dso_update_irq(s);
            }
            return value;
        }
        return data[reg];
    }
}

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
    static const uint32_t clock_mask[] = {
        HPSYS_RCC_I2C1, HPSYS_RCC_I2C2,
        HPSYS_RCC_I2C3, HPSYS_RCC_I2C4,
    };
    SF32LB52I2CState *i2c = &s->i2c[index];
    uint32_t *regs = s->hpsys_periph.regs;
    uint32_t status = I2C_SR_TE;

    if (!sf32lb52_hpsys_clock_enabled(s, index < 2 ? 1 : 2,
                                      clock_mask[index])) {
        return;
    }
    if (!(value & I2C_TCR_TB)) {
        return;
    }
    if (value & I2C_TCR_START) {
        uint8_t address_byte = regs[(base + I2C_DBR) / 4];
        bool read = address_byte & 1;
        uint8_t address = address_byte >> 1;

        if (!read || i2c->address != address) {
            i2c->have_reg = false;
            i2c->reg = 0;
            i2c->reg_bytes = 0;
        }
        i2c->address = address;
        i2c->addressed = true;
        i2c->read = read;
    } else if (i2c->addressed && i2c->read) {
        if (index == 1 && i2c->address == LSM6DSO_ADDRESS &&
            s->lsm6dso_fifo_reading) {
            regs[(base + I2C_DBR) / 4] =
                sf32lb52_lsm6dso_read(s, LSM6DSO_FIFO_DATA_REG);
            i2c->reg++;
        } else if (index == 1 && i2c->address == LSM6DSO_ADDRESS) {
            regs[(base + I2C_DBR) / 4] =
                sf32lb52_lsm6dso_read(s, i2c->reg++);
        } else if (index == 0 && i2c->address == NPM1300_ADDRESS) {
            regs[(base + I2C_DBR) / 4] = s->npm1300[i2c->reg++];
        } else {
            regs[(base + I2C_DBR) / 4] =
                i2c->data[i2c->address][(uint8_t)i2c->reg++];
        }
        status = I2C_SR_RF;
    } else if (i2c->addressed && !i2c->have_reg) {
        uint8_t byte = regs[(base + I2C_DBR) / 4];

        if (index == 0 && i2c->address == NPM1300_ADDRESS &&
            i2c->reg_bytes++ == 0) {
            i2c->reg = (uint16_t)byte << 8;
            goto complete;
        }
        if (index == 0 && i2c->address == NPM1300_ADDRESS) {
            i2c->reg |= byte;
        } else {
            i2c->reg = byte;
        }
        if (index == 1 && i2c->address == LSM6DSO_ADDRESS &&
            i2c->reg == LSM6DSO_FIFO_DATA_REG) {
            s->lsm6dso_fifo_byte = 0;
            s->lsm6dso_fifo_reading = true;
        } else if (index == 1 && i2c->address == LSM6DSO_ADDRESS) {
            s->lsm6dso_fifo_reading = false;
        }
        i2c->have_reg = true;
    } else if (i2c->addressed) {
        uint8_t reg = i2c->reg++;
        uint8_t data = regs[(base + I2C_DBR) / 4];

        if (index == 0 && i2c->address == NPM1300_ADDRESS) {
            s->npm1300[i2c->reg - 1] = data;
        } else if (index == 0 && i2c->address == AW2016_ADDRESS &&
            reg == AW2016_CHIP_ID_REG) {
            data = AW2016_CHIP_ID;
        } else if (index == 1 && i2c->address == LSM6DSO_ADDRESS &&
                   reg == LSM6DSO_CTRL3_C_REG &&
                   (data & LSM6DSO_SW_RESET)) {
            data &= ~LSM6DSO_SW_RESET;
        }
        i2c->data[i2c->address][reg] = data;
        if (index == 1 && i2c->address == LSM6DSO_ADDRESS) {
            if (reg == LSM6DSO_FIFO_DATA_REG) {
                s->lsm6dso_fifo_byte = 0;
            }
            if (reg == LSM6DSO_FIFO_CTRL4_REG) {
                s->lsm6dso_fifo_samples = 0;
                sf32lb52_lsm6dso_update_irq(s);
            }
            if (reg == LSM6DSO_FIFO_CTRL4_REG ||
                reg == LSM6DSO_CTRL1_XL_REG) {
                uint64_t period = sf32lb52_lsm6dso_sample_period(s);

                if ((i2c->data[LSM6DSO_ADDRESS]
                              [LSM6DSO_FIFO_CTRL4_REG] & 7) ==
                        LSM6DSO_FIFO_MODE_STREAM && period) {
                    timer_mod(s->lsm6dso_timer,
                              qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                              period);
                } else {
                    timer_del(s->lsm6dso_timer);
                }
            }
        }
    }
complete:
    if (value & I2C_TCR_STOP) {
        status |= I2C_SR_MSD;
        if (index == 1 && i2c->address == LSM6DSO_ADDRESS) {
            s->lsm6dso_fifo_reading = false;
        }
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

static void sf32lb52_dmac_update_irq(SF32LB52MachineState *s)
{
    uint32_t *regs = s->hpsys_periph.regs;

    for (int channel = 0; channel < DMAC_CHANNEL_COUNT; channel++) {
        uint32_t ccr = regs[(DMAC1_CCR1 +
                             channel * DMAC_CHANNEL_STRIDE) / 4];
        uint32_t status = regs[DMAC1_ISR / 4] >> (channel * 4);

        qemu_set_irq(s->dmac1_irq[channel],
                     ((ccr & DMAC_CCR_TCIE) && (status & 2)) ||
                     ((ccr & DMAC_CCR_HTIE) && (status & 4)));
    }
}

static void sf32lb52_dmac_stream(void *opaque)
{
    SF32LB52DMATimer *context = opaque;
    SF32LB52MachineState *s = context->machine;
    unsigned int channel = context->channel;
    uint32_t *regs = s->hpsys_periph.regs;
    hwaddr ccr_offset = DMAC1_CCR1 + channel * DMAC_CHANNEL_STRIDE;
    uint32_t ccr = regs[ccr_offset / 4];
    uint32_t count = s->dmac_count[channel];
    uint32_t shift = channel * 4;
    bool second_half = s->dmac_second_half[channel];

    if (!(ccr & DMAC_CCR_EN) || !count) {
        return;
    }
    if (s->dmac_periph[channel] ==
        SF32LB52_HPSYS_PERIPH_BASE + PDM1_DATA_L) {
        uint32_t length = count * sizeof(uint32_t) / 2;
        hwaddr address = s->dmac_memory[channel] +
                         (second_half ? length : 0);
        g_autofree uint8_t *samples = g_malloc0(length);

        for (uint32_t sample = 0; sample + 1 < length; sample += 2) {
            stw_le_p(samples + sample,
                     ((sample / 2 + (second_half ? length / 2 : 0)) % 64 -
                      32) * 32);
        }
        address_space_write(&address_space_memory, address,
                            MEMTXATTRS_UNSPECIFIED, samples, length);
    }

    if (second_half) {
        regs[DMAC1_ISR / 4] |= 3U << shift;
        regs[(DMAC1_CNDTR1 + channel * DMAC_CHANNEL_STRIDE) / 4] = count;
    } else {
        regs[DMAC1_ISR / 4] |= 5U << shift;
        regs[(DMAC1_CNDTR1 + channel * DMAC_CHANNEL_STRIDE) / 4] = count / 2;
    }
    s->dmac_second_half[channel] = !second_half;
    sf32lb52_dmac_update_irq(s);

    if (ccr & DMAC_CCR_CIRC) {
        timer_mod(s->dmac_timer[channel],
                  qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                  MAX(NANOSECONDS_PER_SECOND / 1000,
                      (uint64_t)count * NANOSECONDS_PER_SECOND / 16000));
    } else if (second_half) {
        regs[ccr_offset / 4] &= ~DMAC_CCR_EN;
    }
}

static bool sf32lb52_dmac_write(SF32LB52MachineState *s, uint32_t *regs,
                                hwaddr offset, uint32_t value,
                                uint32_t *result)
{
    if (offset == DMAC1_IFCR) {
        sf32lb52_dmac_clear_flags(regs, value);
        sf32lb52_dmac_update_irq(s);
        return true;
    }

    for (int channel = 0; channel < DMAC_CHANNEL_COUNT; channel++) {
        hwaddr ccr = DMAC1_CCR1 + channel * DMAC_CHANNEL_STRIDE;

        if (offset == ccr && !(value & DMAC_CCR_EN)) {
            timer_del(s->dmac_timer[channel]);
            qemu_irq_lower(s->dmac1_irq[channel]);
        } else if (offset == ccr && (value & DMAC_CCR_EN)) {
            uint32_t shift = channel * 4;

            if (!sf32lb52_hpsys_clock_enabled(s, 1, HPSYS_RCC_DMAC1)) {
                *result &= ~DMAC_CCR_EN;
                break;
            }

            s->dmac_count[channel] =
                regs[(DMAC1_CNDTR1 + channel * DMAC_CHANNEL_STRIDE) / 4];
            s->dmac_periph[channel] =
                regs[(DMAC1_CPAR1 + channel * DMAC_CHANNEL_STRIDE) / 4];
            s->dmac_memory[channel] =
                regs[(DMAC1_CM0AR1 + channel * DMAC_CHANNEL_STRIDE) / 4];
            if (s->dmac_periph[channel] ==
                    SF32LB52_HPSYS_PERIPH_BASE + PDM1_DATA_L ||
                s->dmac_periph[channel] ==
                    SF32LB52_HPSYS_PERIPH_BASE + AUDCODEC_DAC_CH0_ENTRY) {
                s->dmac_second_half[channel] = false;
                timer_mod(s->dmac_timer[channel],
                          qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                          MAX(NANOSECONDS_PER_SECOND / 1000,
                              (uint64_t)s->dmac_count[channel] *
                              NANOSECONDS_PER_SECOND / 16000));
            } else {
                regs[DMAC1_ISR / 4] |= 3U << shift;
                regs[(DMAC1_CNDTR1 +
                      channel * DMAC_CHANNEL_STRIDE) / 4] = 0;
                *result &= ~DMAC_CCR_EN;
            }
            sf32lb52_dmac_update_irq(s);
            break;
        }
    }
    return false;
}

static void sf32lb52_flash_flush(SF32LB52MachineState *s, uint32_t address,
                                 uint32_t length)
{
    uint8_t *flash;

    if (!s->flash_blk || !blk_is_writable(s->flash_blk) ||
        address >= s->flash_backed_size) {
        return;
    }
    length = MIN(length, s->flash_backed_size - address);
    flash = memory_region_get_ram_ptr(&s->flash);
    if (blk_pwrite(s->flash_blk, address, length, flash + address, 0) < 0) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "sf32lb52: failed to persist flash write at 0x%08x\n",
                      address);
    }
}

static void sf32lb52_lcdc_update_irq(SF32LB52MachineState *s)
{
    uint32_t *regs = s->hpsys_periph.regs;

    qemu_set_irq(s->lcdc_irq, regs[LCDC_IRQ / 4] & 0x7f);
}

static void sf32lb52_lcdc_timer(void *opaque)
{
    SF32LB52MachineState *s = opaque;
    uint32_t *regs = s->hpsys_periph.regs;
    uint32_t setting = regs[LCDC_SETTING / 4];

    if (s->lcdc_phase < 3) {
        regs[LCDC_IRQ / 4] |= LCDC_IRQ_JDI_RAW_STAT;
        if (setting & LCDC_IRQ_JDI_STAT) {
            regs[LCDC_IRQ / 4] |= LCDC_IRQ_JDI_STAT;
        }
        s->lcdc_phase++;
    } else {
        regs[LCDC_IRQ / 4] |= LCDC_IRQ_EOF_RAW_STAT;
        if (setting & LCDC_IRQ_EOF_STAT) {
            regs[LCDC_IRQ / 4] |= LCDC_IRQ_EOF_STAT;
        }
        s->lcdc_phase = 4;
    }
    sf32lb52_lcdc_update_irq(s);
}

static void sf32lb52_lcdc_copy_framebuffer(SF32LB52MachineState *s)
{
    uint32_t *regs = s->hpsys_periph.regs;
    uint32_t tl = regs[LCDC_CANVAS_TL_POS / 4];
    uint32_t br = regs[LCDC_CANVAS_BR_POS / 4];
    uint32_t x = tl & 0x7ff;
    uint32_t y = (tl >> 16) & 0x7ff;
    uint32_t width = (br & 0x7ff) - x + 1;
    uint32_t doubled_height = ((br >> 16) & 0x7ff) - y + 1;
    uint32_t source = regs[LCDC_LAYER0_SRC / 4];
    g_autofree uint8_t *pixels = NULL;

    if (x >= 200 || y >= 228 || width > 200 - x ||
        doubled_height < 2 || doubled_height / 2 > 228 - y) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "sf32lb52: invalid LCDC rectangle %u,%u %ux%u\n",
                      x, y, width, doubled_height / 2);
        return;
    }
    pixels = g_malloc(width * doubled_height / 2);
    if (address_space_read(&address_space_memory, source,
                           MEMTXATTRS_UNSPECIFIED, pixels,
                           width * doubled_height / 2) != MEMTX_OK) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "sf32lb52: LCDC source read failed at 0x%08x\n",
                      source);
        return;
    }
    for (uint32_t row = 0; row < doubled_height / 2; row++) {
        pbl_display_update_framebuffer(s->display,
                                       (y + row) * 200 + x,
                                       pixels + row * width, width);
    }
}

static void sf32lb52_flash_command(SF32LB52MachineState *s, uint32_t command,
                                   uint32_t address)
{
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
                    sf32lb52_flash_flush(s, address, count);
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
        sf32lb52_flash_flush(s, address,
                             MIN(erase_size,
                                 SF32LB52_FLASH_SIZE - address));
    }
}

static uint64_t sf32lb52_flash_command_time(uint32_t command)
{
    switch (command & 0xff) {
    case 0x02:
    case 0x12:
    case 0x32:
    case 0x34:
        return SPI_FLASH_PAGE_PROGRAM_NS;
    case 0x20:
    case 0x21:
        return SPI_FLASH_SECTOR_ERASE_NS;
    case 0x52:
    case 0xd8:
    case 0xdc:
        return SPI_FLASH_BLOCK_ERASE_NS;
    default:
        return SPI_FLASH_COMMAND_NS;
    }
}

static void sf32lb52_mpi2_complete(void *opaque)
{
    SF32LB52MachineState *s = opaque;
    uint32_t *regs = s->hpsys_periph.regs;

    sf32lb52_flash_command(s, s->mpi2_command, s->mpi2_address);
    regs[MPI2_SR / 4] &= ~MPI_SR_BUSY;
    regs[MPI2_SR / 4] |= MPI_SR_TCF;
    if ((regs[MPI2_CR / 4] & (MPI_CR_CMD2E | MPI_CR_SME2)) ==
        (MPI_CR_CMD2E | MPI_CR_SME2)) {
        regs[MPI2_SR / 4] |= MPI_SR_SMF;
    }
}

static void sf32lb52_mpi2_start(SF32LB52MachineState *s, uint32_t command)
{
    uint32_t *regs = s->hpsys_periph.regs;

    if (!sf32lb52_hpsys_clock_enabled(s, 2, HPSYS_RCC_MPI2)) {
        return;
    }
    if (regs[MPI2_SR / 4] & MPI_SR_BUSY) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "sf32lb52: MPI2 command 0x%02x while busy\n",
                      command & 0xff);
        return;
    }
    s->mpi2_command = command;
    s->mpi2_address = regs[MPI2_AR1 / 4];
    regs[MPI2_SR / 4] &= ~(MPI_SR_TCF | MPI_SR_SMF);
    regs[MPI2_SR / 4] |= MPI_SR_BUSY;
    timer_mod(s->mpi2_timer,
              qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
              sf32lb52_flash_command_time(command));
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

static uint8_t sf32lb52_from_bcd(uint32_t value)
{
    return (value >> 4) * 10 + (value & 0xf);
}

static uint32_t sf32lb52_to_bcd(uint32_t value)
{
    return (value / 10) << 4 | value % 10;
}

static int64_t sf32lb52_rtc_now_ns(SF32LB52MachineState *s)
{
    return s->rtc_base_seconds * NANOSECONDS_PER_SECOND +
           qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) - s->rtc_base_ns;
}

static void sf32lb52_rtc_calendar(SF32LB52MachineState *s,
                                  uint32_t *tr, uint32_t *dr)
{
    int64_t now_ns = sf32lb52_rtc_now_ns(s);
    time_t now = now_ns / NANOSECONDS_PER_SECOND;
    struct tm tm;
    uint32_t subseconds = now_ns % NANOSECONDS_PER_SECOND * 256 /
                          NANOSECONDS_PER_SECOND;

    gmtime_r(&now, &tm);
    *tr = subseconds | sf32lb52_to_bcd(tm.tm_sec) << 11 |
          sf32lb52_to_bcd(tm.tm_min) << 18 |
          sf32lb52_to_bcd(tm.tm_hour) << 25;
    *dr = sf32lb52_to_bcd(tm.tm_mday) |
          sf32lb52_to_bcd(tm.tm_mon + 1) << 8 |
          (tm.tm_wday ? tm.tm_wday : 7) << 13 |
          sf32lb52_to_bcd(tm.tm_year % 100) << 16 | (1U << 24);
}

static void sf32lb52_rtc_set_calendar(SF32LB52MachineState *s,
                                      hwaddr offset, uint32_t value)
{
    int64_t now_ns = sf32lb52_rtc_now_ns(s);
    time_t now = now_ns / NANOSECONDS_PER_SECOND;
    struct tm tm;

    gmtime_r(&now, &tm);
    if (offset == RTC_TR) {
        tm.tm_sec = sf32lb52_from_bcd((value >> 11) & 0x7f);
        tm.tm_min = sf32lb52_from_bcd((value >> 18) & 0x7f);
        tm.tm_hour = sf32lb52_from_bcd((value >> 25) & 0x3f);
    } else {
        uint32_t year = sf32lb52_from_bcd((value >> 16) & 0xff);

        tm.tm_mday = sf32lb52_from_bcd(value & 0x3f);
        tm.tm_mon = sf32lb52_from_bcd((value >> 8) & 0x1f) - 1;
        tm.tm_year = year + ((value & (1U << 24)) && year >= 70 ? 0 : 100);
    }
    s->rtc_base_seconds = mktimegm(&tm);
    s->rtc_base_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
}

static bool sf32lb52_rtc_alarm_matches(SF32LB52MachineState *s)
{
    uint32_t *regs = s->hpsys_periph.regs;
    uint32_t tr, dr;
    uint32_t alarm_tr = regs[RTC_ALRMTR / 4];
    uint32_t alarm_dr = regs[RTC_ALRMDR / 4];

    sf32lb52_rtc_calendar(s, &tr, &dr);
    return ((alarm_dr & RTC_ALRMDR_MSKS) ||
            (alarm_tr & 0x0003f800) == (tr & 0x0003f800)) &&
           ((alarm_dr & RTC_ALRMDR_MSKMN) ||
            (alarm_tr & 0x01fc0000) == (tr & 0x01fc0000)) &&
           ((alarm_dr & RTC_ALRMDR_MSKH) ||
            (alarm_tr & 0xfe000000) == (tr & 0xfe000000)) &&
           ((alarm_dr & RTC_ALRMDR_MSKD) ||
            (alarm_dr & 0x3f) == (dr & 0x3f)) &&
           ((alarm_dr & RTC_ALRMDR_MSKM) ||
            (alarm_dr & 0x1f00) == (dr & 0x1f00)) &&
           ((alarm_dr & RTC_ALRMDR_MSKWD) ||
            (alarm_dr & 0xe000) == (dr & 0xe000));
}

static void sf32lb52_rtc_alarm(void *opaque)
{
    SF32LB52MachineState *s = opaque;
    uint32_t *regs = s->hpsys_periph.regs;

    if (!(regs[RTC_CR / 4] & RTC_CR_ALRME)) {
        return;
    }
    if (sf32lb52_rtc_alarm_matches(s)) {
        regs[RTC_ISR / 4] |= RTC_ISR_ALRMF;
        qemu_set_irq(s->rtc_irq, regs[RTC_CR / 4] & RTC_CR_ALRMIE);
    }
    timer_mod(s->rtc_alarm_timer,
              (qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) /
               NANOSECONDS_PER_SECOND + 1) * NANOSECONDS_PER_SECOND);
}

static void sf32lb52_rtc_update_alarm(SF32LB52MachineState *s)
{
    uint32_t *regs = s->hpsys_periph.regs;

    qemu_set_irq(s->rtc_irq, (regs[RTC_CR / 4] & RTC_CR_ALRMIE) &&
                             (regs[RTC_ISR / 4] & RTC_ISR_ALRMF));
    if (regs[RTC_CR / 4] & RTC_CR_ALRME) {
        timer_mod(s->rtc_alarm_timer,
                  (qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) /
                   NANOSECONDS_PER_SECOND + 1) * NANOSECONDS_PER_SECOND);
    } else {
        timer_del(s->rtc_alarm_timer);
    }
}

static uint32_t sf32lb52_lptim_count(SF32LB52MachineState *s)
{
    uint32_t *regs = s->hpsys_periph.regs;
    uint64_t elapsed = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) -
                       s->lptim1_start_ns;

    if (!(regs[LPTIM_CR / 4] & LPTIM_CR_ENABLE)) {
        return regs[LPTIM_CNT / 4];
    }
    return MIN(elapsed * LPTIM_FREQUENCY_HZ / NANOSECONDS_PER_SECOND,
               regs[LPTIM_ARR / 4]);
}

static void sf32lb52_lptim1_update_irq(SF32LB52MachineState *s)
{
    uint32_t *regs = s->hpsys_periph.regs;

    qemu_set_irq(s->lptim1_irq,
                 regs[LPTIM_ISR / 4] & regs[LPTIM_IER / 4] & 0xf);
}

static void sf32lb52_lptim1_expire(void *opaque)
{
    SF32LB52MachineState *s = opaque;
    uint32_t *regs = s->hpsys_periph.regs;

    regs[LPTIM_CNT / 4] = regs[LPTIM_ARR / 4];
    regs[LPTIM_ISR / 4] |= LPTIM_ISR_OF | LPTIM_ISR_OFWKUP;
    sf32lb52_lptim1_update_irq(s);
}

static void sf32lb52_gptim2_update_irq(SF32LB52MachineState *s)
{
    uint32_t *regs = s->hpsys_periph.regs;

    qemu_set_irq(s->gptim2_irq,
                 regs[GPTIM_SR / 4] & regs[GPTIM_DIER / 4] & GPTIM_SR_UIF);
}

static uint64_t sf32lb52_gptim2_period_ns(SF32LB52MachineState *s)
{
    uint32_t *regs = s->hpsys_periph.regs;
    uint64_t ticks = (regs[GPTIM_PSC / 4] + 1ULL) *
                     (regs[GPTIM_ARR / 4] + 1ULL);

    return MAX(ticks * NANOSECONDS_PER_SECOND / GPTIM_CLOCK_HZ, 1ULL);
}

static void sf32lb52_gptim2_schedule(SF32LB52MachineState *s)
{
    uint32_t *regs = s->hpsys_periph.regs;

    if ((regs[GPTIM_CR1 / 4] & GPTIM_CR1_CEN) &&
        (regs[GPTIM_DIER / 4] & GPTIM_DIER_UIE) &&
        sf32lb52_hpsys_clock_enabled(s, 1, HPSYS_RCC_GPTIM2)) {
        timer_mod(s->gptim2_timer,
                  qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                  sf32lb52_gptim2_period_ns(s));
    } else {
        timer_del(s->gptim2_timer);
    }
    sf32lb52_gptim2_update_irq(s);
}

static void sf32lb52_gptim2_expire(void *opaque)
{
    SF32LB52MachineState *s = opaque;

    s->hpsys_periph.regs[GPTIM_SR / 4] |= GPTIM_SR_UIF;
    sf32lb52_gptim2_update_irq(s);
    sf32lb52_gptim2_schedule(s);
}

static void sf32lb52_lptim1_start(SF32LB52MachineState *s)
{
    uint32_t period = s->hpsys_periph.regs[LPTIM_ARR / 4];

    s->lptim1_start_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    timer_mod(s->lptim1_timer, s->lptim1_start_ns +
              (uint64_t)period * NANOSECONDS_PER_SECOND /
              LPTIM_FREQUENCY_HZ);
}

static uint32_t sf32lb52_load32(hwaddr address)
{
    uint32_t value = 0;

    address_space_read(&address_space_memory, address, MEMTXATTRS_UNSPECIFIED,
                       &value, sizeof(value));
    return le32_to_cpu(value);
}

static void sf32lb52_store32(hwaddr address, uint32_t value)
{
    value = cpu_to_le32(value);
    address_space_write(&address_space_memory, address, MEMTXATTRS_UNSPECIFIED,
                        &value, sizeof(value));
}

static void sf32lb52_hci_ring_init(void)
{
    uint8_t header[IPC_RING_HEADER_SIZE] = { 0 };

    stl_le_p(header, LCPU2HCPU_RING + IPC_RING_HEADER_SIZE);
    stl_le_p(header + 4, LCPU2HCPU_RING + IPC_RING_HEADER_SIZE);
    stw_le_p(header + 16, IPC_RING_SIZE - IPC_RING_HEADER_SIZE);
    address_space_write(&address_space_memory, LCPU2HCPU_RING,
                        MEMTXATTRS_UNSPECIFIED, header, sizeof(header));
}

static bool sf32lb52_hci_ring_put(const uint8_t *data, size_t length)
{
    uint32_t read = sf32lb52_load32(LCPU2HCPU_RING + 8);
    uint32_t write = sf32lb52_load32(LCPU2HCPU_RING + 12);
    uint16_t read_index = read >> 16;
    uint16_t write_index = write >> 16;
    uint16_t read_mirror = read;
    uint16_t write_mirror = write;
    const uint16_t size = IPC_RING_SIZE - IPC_RING_HEADER_SIZE;
    size_t free = read_index == write_index ?
                  (read_mirror == write_mirror ? size : 0) :
                  (write_index < read_index ? read_index - write_index :
                   size - write_index + read_index);

    if (length > free) {
        return false;
    }
    for (size_t i = 0; i < length; i++) {
        address_space_write(&address_space_memory,
                            LCPU2HCPU_RING + IPC_RING_HEADER_SIZE + write_index,
                            MEMTXATTRS_UNSPECIFIED, data + i, 1);
        if (++write_index == size) {
            write_index = 0;
            write_mirror ^= 1;
        }
    }
    sf32lb52_store32(LCPU2HCPU_RING + 12,
                     write_index << 16 | write_mirror);
    return true;
}

static void sf32lb52_hci_interrupt(SF32LB52MachineState *s)
{
    uint32_t *regs = s->lpsys_periph.regs;

    regs[(MAILBOX2_BASE + MAILBOX_C1ISR) / 4] |= 1;
    regs[(MAILBOX2_BASE + MAILBOX_C1MISR) / 4] |= 1;
    qemu_irq_raise(s->lcpu2hcpu_irq);
}

static void sf32lb52_hci_event(SF32LB52MachineState *s, uint16_t opcode);

static void sf32lb52_hci_ready(void *opaque)
{
    SF32LB52MachineState *s = opaque;

    sf32lb52_hci_ring_init();
    sf32lb52_hci_event(s, 0xfc11);
}

static void sf32lb52_hci_event(SF32LB52MachineState *s, uint16_t opcode)
{
    uint8_t event[80] = {
        HCI_EVENT_PACKET, 0x0e, 0x04, 0x01, opcode, opcode >> 8, 0x00,
    };
    size_t return_length = 1;

    switch (opcode) {
    case 0x1001:
        return_length = 9;
        event[7] = 0x0b;
        event[10] = 0x0b;
        event[11] = 0x31;
        event[12] = 0x01;
        break;
    case 0x1002:
        return_length = 65;
        break;
    case 0x1003:
    case 0x2003:
    case 0x201c:
        return_length = 9;
        memset(event + 7, 0xff, 8);
        break;
    case 0x1005:
        return_length = 8;
        event[7] = 0xfb;
        event[8] = 0x00;
        event[10] = 8;
        break;
    case 0x1009:
        return_length = 7;
        event[7] = 0x52;
        event[8] = 0x4c;
        event[9] = 0x42;
        event[10] = 0x45;
        event[11] = 0x50;
        event[12] = 0x00;
        break;
    case 0x2002:
        return_length = 4;
        event[7] = 0xfb;
        event[8] = 0x00;
        event[9] = 8;
        break;
    case 0x200f:
        return_length = 2;
        event[7] = 8;
        break;
    case 0x2018:
        return_length = 9;
        memcpy(event + 7, (uint8_t[]) {
            0x21, 0x43, 0x65, 0x87, 0xa9, 0xcb, 0xed, 0x0f,
        }, 8);
        break;
    default:
        break;
    }
    event[2] = 3 + return_length;
    if (sf32lb52_hci_ring_put(event, 3 + event[2])) {
        sf32lb52_hci_interrupt(s);
    }
}

static void sf32lb52_hci_process(SF32LB52MachineState *s)
{
    while (s->hci_command_len) {
        uint16_t packet_length;
        uint16_t opcode;

        if (s->hci_command[0] != HCI_COMMAND_PACKET) {
            memmove(s->hci_command, s->hci_command + 1,
                    --s->hci_command_len);
            continue;
        }
        if (s->hci_command_len < 4) {
            return;
        }
        packet_length = 4 + s->hci_command[3];
        if (s->hci_command_len < packet_length) {
            return;
        }
        opcode = lduw_le_p(s->hci_command + 1);
        sf32lb52_hci_event(s, opcode);
        s->hci_command_len -= packet_length;
        memmove(s->hci_command, s->hci_command + packet_length,
                s->hci_command_len);
    }
}

static void sf32lb52_hci_tx(SF32LB52MachineState *s)
{
    uint32_t read = sf32lb52_load32(HCPU2LCPU_RING + 8);
    uint32_t write = sf32lb52_load32(HCPU2LCPU_RING + 12);
    uint16_t read_index = read >> 16;
    uint16_t write_index = write >> 16;
    uint16_t read_mirror = read;
    uint16_t write_mirror = write;
    const uint16_t size = IPC_RING_SIZE - IPC_RING_HEADER_SIZE;

    while (read_index != write_index || read_mirror != write_mirror) {
        uint8_t byte;

        address_space_read(&address_space_memory,
                           HCPU2LCPU_RING + IPC_RING_HEADER_SIZE + read_index,
                           MEMTXATTRS_UNSPECIFIED, &byte, 1);
        if (s->hci_command_len < sizeof(s->hci_command)) {
            s->hci_command[s->hci_command_len++] = byte;
        }
        if (++read_index == size) {
            read_index = 0;
            read_mirror ^= 1;
        }
    }
    sf32lb52_store32(HCPU2LCPU_RING + 8, read_index << 16 | read_mirror);
    sf32lb52_hci_process(s);
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

static uint32_t sf32lb52_bt_rfc_count(SF32LB52MachineState *s)
{
    uint32_t *regs = s->lpsys_periph.regs;
    bool edr = regs[BT_RFC_VCO_REG1 / 4] & BT_RFC_VCO3G_ENABLE;
    uint32_t pdx = regs[(edr ? BT_RFC_EDR_CAL_REG1 : BT_RFC_VCO_REG3) / 4] &
                   0xff;
    uint32_t fcw = regs[BT_PHY_TX_HFP_CFG / 4] >> BT_PHY_HFP_FCW_SHIFT &
                   BT_PHY_HFP_FCW_MASK;

    return (edr ? 38800 : 39000) - 40 * pdx + 20 * fcw;
}

static void sf32lb52_rcc_reset_i2c(SF32LB52MachineState *s, int index,
                                   hwaddr base)
{
    SF32LB52I2CState *i2c = &s->i2c[index];

    memset(&s->hpsys_periph.regs[base / 4], 0, I2C_REG_SIZE);
    i2c->address = 0;
    i2c->reg = 0;
    i2c->reg_bytes = 0;
    i2c->addressed = false;
    i2c->read = false;
    i2c->have_reg = false;
    qemu_irq_lower(i2c->irq);
}

static void sf32lb52_rcc_reset_modules(SF32LB52MachineState *s,
                                       unsigned int group, uint32_t reset)
{
    uint32_t *regs = s->hpsys_periph.regs;

    if (group == 1) {
        if (reset & HPSYS_RCC_DMAC1) {
            memset(&regs[DMAC1_ISR / 4], 0, 0x100);
            for (int channel = 0; channel < DMAC_CHANNEL_COUNT; channel++) {
                timer_del(s->dmac_timer[channel]);
                qemu_irq_lower(s->dmac1_irq[channel]);
            }
        }
        if (reset & HPSYS_RCC_LCDC1) {
            memset(&regs[LCDC1_BASE / 4], 0, 0x100);
            timer_del(s->lcdc_timer);
            qemu_irq_lower(s->lcdc_irq);
            s->lcdc_phase = 0;
        }
        if (reset & HPSYS_RCC_GPTIM2) {
            memset(&regs[GPTIM2_BASE / 4], 0, 0x40);
            timer_del(s->gptim2_timer);
            qemu_irq_lower(s->gptim2_irq);
        }
        if (reset & HPSYS_RCC_I2C1) {
            sf32lb52_rcc_reset_i2c(s, 0, I2C1_CR);
        }
        if (reset & HPSYS_RCC_I2C2) {
            sf32lb52_rcc_reset_i2c(s, 1, I2C2_CR);
        }
    } else {
        if (reset & HPSYS_RCC_GPIO1) {
            memset(&regs[GPIO1_BASE / 4], 0, 2 * GPIO_BANK_STRIDE);
            qemu_irq_lower(s->gpio1_irq);
        }
        if (reset & HPSYS_RCC_MPI2) {
            memset(&regs[MPI2_CR / 4], 0, 0x100);
            timer_del(s->mpi2_timer);
        }
        if (reset & HPSYS_RCC_I2C3) {
            sf32lb52_rcc_reset_i2c(s, 2, I2C3_CR);
        }
        if (reset & HPSYS_RCC_I2C4) {
            sf32lb52_rcc_reset_i2c(s, 3, I2C4_CR);
        }
    }
}

static void sf32lb52_rcc_clock_changed(SF32LB52MachineState *s)
{
    uint32_t *regs = s->hpsys_periph.regs;

    sf32lb52_gptim2_schedule(s);
    if (!(regs[HPSYS_RCC_ENR2 / 4] & HPSYS_RCC_MPI2)) {
        timer_del(s->mpi2_timer);
    } else if ((regs[MPI2_SR / 4] & MPI_SR_BUSY) &&
               !timer_pending(s->mpi2_timer)) {
        timer_mod(s->mpi2_timer,
                  qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                  sf32lb52_flash_command_time(s->mpi2_command));
    }
    sf32lb52_gpio_update_irq(s);
}

static bool sf32lb52_rcc_write(SF32LB52MachineState *s, hwaddr offset,
                               uint32_t value)
{
    uint32_t *regs = s->hpsys_periph.regs;
    hwaddr enr;

    switch (offset) {
    case HPSYS_RCC_RSTR1:
    case HPSYS_RCC_RSTR2:
        sf32lb52_rcc_reset_modules(s,
            offset == HPSYS_RCC_RSTR1 ? 1 : 2,
            value & ~regs[offset / 4]);
        regs[offset / 4] = value;
        return true;
    case HPSYS_RCC_ENR1:
    case HPSYS_RCC_ENR2:
        regs[offset / 4] = value;
        sf32lb52_rcc_clock_changed(s);
        return true;
    case HPSYS_RCC_ESR1:
    case HPSYS_RCC_ESR2:
        enr = offset == HPSYS_RCC_ESR1 ? HPSYS_RCC_ENR1 : HPSYS_RCC_ENR2;
        regs[enr / 4] |= value;
        sf32lb52_rcc_clock_changed(s);
        return true;
    case HPSYS_RCC_ECR1:
    case HPSYS_RCC_ECR2:
        enr = offset == HPSYS_RCC_ECR1 ? HPSYS_RCC_ENR1 : HPSYS_RCC_ENR2;
        regs[enr / 4] &= ~value;
        sf32lb52_rcc_clock_changed(s);
        return true;
    default:
        return false;
    }
}

static uint64_t sf32lb52_peripheral_read(void *opaque, hwaddr offset,
                                         unsigned int size)
{
    SF32LB52PeripheralRegion *r = opaque;
    SF32LB52MachineState *s = r->machine;
    uint32_t tr, dr;

    qemu_log_mask(LOG_UNIMP, "%s: register read at 0x%06" HWADDR_PRIx "\n",
                  r->name, offset);
    if (!strcmp(r->name, "sf32lb52.lpsys-peripherals")) {
        switch (offset) {
        case DMAC2_ISR:
            return r->regs[offset / 4] | DMAC2_ISR_TCIF8;
        case BT_RFC_FBDV_REG1:
            return r->regs[offset / 4] |
                   ((r->regs[offset / 4] & BT_RFC_FKCAL_ENABLE) ?
                    BT_RFC_FKCAL_READY : 0);
        case BT_RFC_FBDV_REG2:
            return (r->regs[offset / 4] & 0xffff) |
                   sf32lb52_bt_rfc_count(s) << 16;
        case BT_RFC_PACAL_REG:
            return r->regs[offset / 4] | BT_RFC_PACAL_DONE;
        case BT_RFC_ROSCAL_REG2:
            return r->regs[offset / 4] | BT_RFC_ROSCAL_DONE;
        case BT_RFC_RCROSCAL_REG:
            return r->regs[offset / 4] | BT_RFC_RCCAL_DONE |
                   BT_RFC_RCCAL_CAPCODE;
        case BT_MAC_RCCAL_RESULT:
            return BT_MAC_RCCAL_DONE |
                   (r->regs[BT_MAC_RCCAL_CTRL / 4] & 0xffff) * 4800;
        default:
            break;
        }
    }
    if (!strcmp(r->name, "sf32lb52.hpsys-peripherals")) {
        switch (offset) {
        case RTC_TR:
        case HPSYS_CFG_RTC_TR:
            sf32lb52_rtc_calendar(s, &tr, &dr);
            return tr;
        case RTC_DR:
        case HPSYS_CFG_RTC_DR:
            sf32lb52_rtc_calendar(s, &tr, &dr);
            return dr;
        case LPTIM_CNT:
            return sf32lb52_lptim_count(s);
        case HPSYS_AON_GTIMR:
            return qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) * 32768 /
                   NANOSECONDS_PER_SECOND;
        case GPADC_IRQ:
            return r->regs[offset / 4] | GPADC_IRQ_READY;
        default:
            break;
        }
    }
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

    if (!strcmp(r->name, "sf32lb52.lpsys-peripherals")) {
        switch (offset) {
        case LPSYS_AON_ACR:
            if (value & LPSYS_AON_ACR_HXT48_REQ) {
                result |= LPSYS_AON_ACR_HXT48_RDY;
            }
            break;
        case MAILBOX2_BASE + MAILBOX_C1IER:
            r->regs[offset / 4] = value;
            return;
        case MAILBOX2_BASE + MAILBOX_C1ICR:
            r->regs[(MAILBOX2_BASE + MAILBOX_C1ISR) / 4] &= ~value;
            r->regs[(MAILBOX2_BASE + MAILBOX_C1MISR) / 4] &= ~value;
            if (!r->regs[(MAILBOX2_BASE + MAILBOX_C1MISR) / 4]) {
                qemu_irq_lower(s->lcpu2hcpu_irq);
            }
            return;
        default:
            break;
        }
    }
    if (!strcmp(r->name, "sf32lb52.hpsys-peripherals")) {
        if (sf32lb52_rcc_write(s, offset, value)) {
            return;
        }
        if (sf32lb52_gpio_write(s, offset, value)) {
            return;
        }
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
        case PMUC_CR:
            r->regs[offset / 4] = value;
            if (value & PMUC_CR_REBOOT) {
                qemu_system_reset_request(SHUTDOWN_CAUSE_GUEST_RESET);
            }
            return;
        case GPTIM_CR1:
        case GPTIM_DIER:
        case GPTIM_PSC:
        case GPTIM_ARR:
            r->regs[offset / 4] = value;
            sf32lb52_gptim2_schedule(s);
            return;
        case GPTIM_SR:
            r->regs[offset / 4] &= value;
            sf32lb52_gptim2_update_irq(s);
            return;
        case GPTIM_EGR:
            if (value & GPTIM_EGR_UG) {
                r->regs[GPTIM_SR / 4] |= GPTIM_SR_UIF;
                sf32lb52_gptim2_update_irq(s);
            }
            return;
        case MAILBOX1_BASE + MAILBOX_C1IER:
            r->regs[offset / 4] = value;
            if ((value & 1) && !s->hci_ready) {
                s->hci_ready = true;
                timer_mod(s->hci_ready_timer,
                          qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                          NANOSECONDS_PER_SECOND / 1000);
            }
            return;
        case MAILBOX1_BASE + MAILBOX_C1ITR:
            sf32lb52_hci_tx(s);
            r->regs[(MAILBOX1_BASE + MAILBOX_C1ITR) / 4] = 0;
            r->regs[(MAILBOX1_BASE + MAILBOX_C1ISR) / 4] = 0;
            return;
        case MAILBOX1_BASE + MAILBOX_C1ICR:
            r->regs[(MAILBOX1_BASE + MAILBOX_C1ISR) / 4] &= ~value;
            return;
        case RTC_TR:
        case RTC_DR:
            sf32lb52_rtc_set_calendar(s, offset, value);
            return;
        case RTC_CR:
            r->regs[offset / 4] = value;
            sf32lb52_rtc_update_alarm(s);
            return;
        case RTC_ALRMTR:
        case RTC_ALRMDR:
            r->regs[offset / 4] = value;
            sf32lb52_rtc_update_alarm(s);
            return;
        case LPTIM_ICR:
            r->regs[LPTIM_ISR / 4] &= ~value;
            sf32lb52_lptim1_update_irq(s);
            return;
        case LPTIM_IER:
            r->regs[offset / 4] = value;
            sf32lb52_lptim1_update_irq(s);
            return;
        case LPTIM_CR:
            if (!(value & LPTIM_CR_ENABLE)) {
                r->regs[LPTIM_CNT / 4] = sf32lb52_lptim_count(s);
                timer_del(s->lptim1_timer);
            }
            r->regs[offset / 4] = value &
                ~(LPTIM_CR_SNGSTRT | LPTIM_CR_CNTSTRT |
                  LPTIM_CR_COUNTRST);
            if (value & LPTIM_CR_COUNTRST) {
                r->regs[LPTIM_CNT / 4] = 0;
                s->lptim1_start_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
            }
            if ((value & LPTIM_CR_ENABLE) &&
                (value & (LPTIM_CR_SNGSTRT | LPTIM_CR_CNTSTRT))) {
                sf32lb52_lptim1_start(s);
            }
            return;
        case LCDC_IRQ:
            r->regs[LCDC_IRQ / 4] &= ~value;
            sf32lb52_lcdc_update_irq(s);
            if ((value & (LCDC_IRQ_JDI_STAT | LCDC_IRQ_JDI_RAW_STAT)) &&
                (s->lcdc_phase == 2 || s->lcdc_phase == 3)) {
                timer_mod(s->lcdc_timer,
                          qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 1000);
            } else if ((value & (LCDC_IRQ_EOF_STAT |
                                 LCDC_IRQ_EOF_RAW_STAT)) &&
                       s->lcdc_phase == 4) {
                s->lcdc_phase = 0;
            }
            return;
        case LCDC_JDI_PAR_CTRL:
            if ((value & LCDC_JDI_PAR_CTRL_ENABLE) &&
                !(r->regs[offset / 4] & LCDC_JDI_PAR_CTRL_ENABLE) &&
                sf32lb52_hpsys_clock_enabled(s, 1, HPSYS_RCC_LCDC1)) {
                sf32lb52_lcdc_copy_framebuffer(s);
                s->lcdc_phase = 1;
                timer_mod(s->lcdc_timer,
                          qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 1000);
            }
            break;
        case EFUSEC_CR:
            if (value & EFUSEC_CR_EN) {
                r->regs[EFUSEC_SR / 4] |= EFUSEC_SR_DONE;
            }
            break;
        case MPI2_CMDR1:
            r->regs[offset / 4] = value;
            sf32lb52_mpi2_start(s, value);
            return;
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
            result |= RTC_ISR_RSF | RTC_ISR_ALRMWF | RTC_ISR_WUTWF;
            if (value & RTC_ISR_INIT) {
                result |= RTC_ISR_INITF;
            }
            r->regs[offset / 4] = result;
            sf32lb52_rtc_update_alarm(s);
            return;
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
            if (value & I2C_CR_RSTREQ) {
                s->i2c[i2c_index].addressed = false;
                s->i2c[i2c_index].have_reg = false;
            }
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
    uint32_t rtc_backup[RTC_BACKUP_REGISTER_COUNT];
    struct tm tm;

    if (s->rtc_initialized) {
        memcpy(rtc_backup, &s->hpsys_periph.regs[RTC_BKP0R / 4],
               sizeof(rtc_backup));
    }

    s->dwt_ctrl = 0;
    s->dwt_cyccnt_base = 0;
    s->dwt_cyccnt_base_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    s->rng_state = 0x6d2b79f5;
    s->lcdc_phase = 0;
    timer_del(s->lcdc_timer);
    timer_del(s->gptim2_timer);
    timer_del(s->lptim1_timer);
    timer_del(s->mpi2_timer);
    timer_del(s->rtc_alarm_timer);
    timer_del(s->lsm6dso_timer);
    timer_del(s->hci_ready_timer);
    for (int channel = 0; channel < DMAC_CHANNEL_COUNT; channel++) {
        timer_del(s->dmac_timer[channel]);
        qemu_irq_lower(s->dmac1_irq[channel]);
    }
    qemu_irq_lower(s->lcdc_irq);
    qemu_irq_lower(s->gptim2_irq);
    qemu_irq_lower(s->lptim1_irq);
    qemu_irq_lower(s->rtc_irq);
    qemu_irq_lower(s->lcpu2hcpu_irq);
    memset(s->dmac_count, 0, sizeof(s->dmac_count));
    memset(s->dmac_periph, 0, sizeof(s->dmac_periph));
    memset(s->dmac_memory, 0, sizeof(s->dmac_memory));
    memset(s->dmac_second_half, 0, sizeof(s->dmac_second_half));
    s->lsm6dso_fifo_samples = 0;
    s->lsm6dso_fifo_byte = 0;
    s->lsm6dso_fifo_reading = false;
    s->hci_command_len = 0;
    s->hci_ready = false;
    memset(s->hpsys_periph.regs, 0, SF32LB52_PERIPH_SIZE);
    memset(s->lpsys_periph.regs, 0, SF32LB52_PERIPH_SIZE);
    s->hpsys_periph.regs[HPSYS_RCC_ENR1 / 4] = UINT32_MAX;
    s->hpsys_periph.regs[HPSYS_RCC_ENR2 / 4] = UINT32_MAX;
    if (s->rtc_initialized) {
        memcpy(&s->hpsys_periph.regs[RTC_BKP0R / 4], rtc_backup,
               sizeof(rtc_backup));
    }
    *sf32lb52_gpio_reg(s, OBELIX_TOUCH_INT_PIN, GPIO_DIR) |=
        1U << OBELIX_TOUCH_INT_PIN;
    *sf32lb52_gpio_reg(s, OBELIX_BUTTON_PIN_BASE, GPIO_DIR) |=
        0xeU << (OBELIX_BUTTON_PIN_BASE % 32);
    qemu_irq_lower(s->gpio1_irq);
    for (int i = 0; i < ARRAY_SIZE(s->i2c); i++) {
        s->i2c[i].address = 0;
        s->i2c[i].reg = 0;
        s->i2c[i].reg_bytes = 0;
        s->i2c[i].addressed = false;
        s->i2c[i].read = false;
        s->i2c[i].have_reg = false;
        memset(s->i2c[i].data, 0, sizeof(s->i2c[i].data));
        qemu_irq_lower(s->i2c[i].irq);
    }
    memset(s->npm1300, 0, sizeof(s->npm1300));
    s->npm1300[NPM1300_EVENTS_ADC] = 0x43;
    s->npm1300[NPM1300_VBUS_STATUS] = 0;
    s->npm1300[NPM1300_CHARGE_STATUS] = 0;
    s->npm1300[NPM1300_IBAT_STATUS] = 0x04;
    s->npm1300[NPM1300_VBAT_MSB] = 0xc7;
    s->npm1300[NPM1300_NTC_MSB] = 0x80;
    s->npm1300[NPM1300_GP0_LSBS] = 0x02;
    s->npm1300[NPM1300_VBAT2_MSB] = 0;
    s->npm1300[NPM1300_GP1_LSBS] = 0x10;
    s->i2c[0].data[AW2016_ADDRESS][AW2016_CHIP_ID_REG] = AW2016_CHIP_ID;
    s->i2c[0].data[W1160_ADDRESS][W1160_CHIP_ID_REG] = W1160_CHIP_ID;
    s->i2c[0].data[W1160_ADDRESS][W1160_FLAG1_REG] = W1160_FLAG_ALS_READY;
    s->i2c[0].data[W1160_ADDRESS][W1160_DATA1_ALS_REG] = 0x04;
    s->i2c[0].data[W1160_ADDRESS][W1160_DATA2_ALS_REG] = 0x00;
    s->i2c[1].data[LSM6DSO_ADDRESS][LSM6DSO_WHO_AM_I_REG] =
        LSM6DSO_WHO_AM_I;
    s->i2c[1].data[LSM6DSO_ADDRESS][LSM6DSO_STATUS_REG] = 1;
    s->i2c[1].data[LSM6DSO_ADDRESS][LSM6DSO_OUTX_L_A_REG + 5] = 0x40;
    s->i2c[1].data[MMC5603_ADDRESS][MMC5603_STATUS1_REG] =
        MMC5603_MEAS_READY | MMC5603_OTP_READY;
    s->i2c[1].data[MMC5603_ADDRESS][MMC5603_WHO_AM_I_REG] =
        MMC5603_WHO_AM_I;
    s->i2c[2].data[CST816_ADDRESS][CST816_CHIP_ID_REG] = CST816_CHIP_ID;
    s->i2c[2].data[CST816_ADDRESS][CST816_FW_VERSION_REG] =
        CST816_FW_VERSION;
    s->hpsys_periph.regs[HPSYS_RCC_HRCCAL2 / 4] = 0x40004000;
    s->hpsys_periph.regs[HPSYS_AON_ACR / 4] =
        HPSYS_AON_ACR_HRC48_REQ | HPSYS_AON_ACR_HRC48_RDY;
    s->hpsys_periph.regs[HPSYS_AON_ISSR / 4] = HPSYS_AON_ISSR_LP_ACTIVE;
    s->hpsys_periph.regs[EFUSEC_BANK0_DATA0 / 4] = 0x42455000;
    s->hpsys_periph.regs[EFUSEC_BANK0_DATA1 / 4] = 0xa575524c;
    s->hpsys_periph.regs[RTC_ISR / 4] =
        RTC_ISR_ALRMWF | RTC_ISR_WUTWF | RTC_ISR_RSF;
    if (!s->rtc_initialized) {
        qemu_get_timedate(&tm, 0);
        s->rtc_base_seconds = mktimegm(&tm);
        s->rtc_base_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        s->rtc_initialized = true;
    }
    s->lptim1_start_ns = s->rtc_base_ns;
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
    DeviceState *flash_dev;
    DeviceState *usart;
    SysBusDevice *usart_sbd;
    DriveInfo *dinfo;
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
    dinfo = drive_get(IF_MTD, 0, 0);
    if (dinfo) {
        int64_t length;
        uint64_t perm = BLK_PERM_CONSISTENT_READ;

        s->flash_blk = blk_by_legacy_dinfo(dinfo);
        flash_dev = qdev_new(TYPE_SF32LB52_FLASH);
        qdev_prop_set_drive(flash_dev, "drive", s->flash_blk);
        qdev_realize_and_unref(flash_dev, NULL, &error_fatal);
        if (blk_supports_write_perm(s->flash_blk)) {
            perm |= BLK_PERM_WRITE;
        }
        if (blk_set_perm(s->flash_blk, perm, BLK_PERM_ALL,
                         &error_fatal) < 0) {
            exit(EXIT_FAILURE);
        }
        length = blk_getlength(s->flash_blk);
        if (length < 0) {
            error_report("sf32lb52: failed to get external flash size");
            exit(EXIT_FAILURE);
        }
        s->flash_backed_size = MIN(length, SF32LB52_FLASH_SIZE);
        if (s->flash_backed_size &&
            blk_pread(s->flash_blk, 0, s->flash_backed_size,
                      memory_region_get_ram_ptr(&s->flash), 0) < 0) {
            error_report("sf32lb52: failed to load external flash image");
            exit(EXIT_FAILURE);
        }
    }
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
    qdev_prop_set_bit(armv7m, "systick-wakes-deep-sleep", true);
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
    s->gpio1_irq = qdev_get_gpio_in(armv7m, SF32LB52_GPIO1_IRQ);
    s->lcdc_irq = qdev_get_gpio_in(armv7m, SF32LB52_LCDC1_IRQ);
    s->gptim2_irq = qdev_get_gpio_in(armv7m, SF32LB52_GPTIM2_IRQ);
    s->lptim1_irq = qdev_get_gpio_in(armv7m, SF32LB52_LPTIM1_IRQ);
    s->rtc_irq = qdev_get_gpio_in(armv7m, SF32LB52_RTC_IRQ);
    s->lcpu2hcpu_irq = qdev_get_gpio_in(armv7m,
                                        SF32LB52_LCPU2HCPU_IRQ);
    s->lcdc_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL,
                                 sf32lb52_lcdc_timer, s);
    s->gptim2_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL,
                                   sf32lb52_gptim2_expire, s);
    s->lptim1_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL,
                                   sf32lb52_lptim1_expire, s);
    s->mpi2_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL,
                                 sf32lb52_mpi2_complete, s);
    s->rtc_alarm_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL,
                                      sf32lb52_rtc_alarm, s);
    s->lsm6dso_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL,
                                    sf32lb52_lsm6dso_sample, s);
    s->hci_ready_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL,
                                      sf32lb52_hci_ready, s);
    for (int channel = 0; channel < DMAC_CHANNEL_COUNT; channel++) {
        s->dmac1_irq[channel] = qdev_get_gpio_in(
            armv7m, SF32LB52_DMAC1_CH1_IRQ + channel);
        s->dmac_timer_context[channel].machine = s;
        s->dmac_timer_context[channel].channel = channel;
        s->dmac_timer[channel] = timer_new_ns(
            QEMU_CLOCK_VIRTUAL, sf32lb52_dmac_stream,
            &s->dmac_timer_context[channel]);
    }

    s->display = qdev_new(TYPE_PEBBLE_DISPLAY);
    qdev_prop_set_uint32(s->display, "width", 200);
    qdev_prop_set_uint32(s->display, "height", 228);
    qdev_prop_set_uint32(s->display, "format",
                         PEBBLE_DISPLAY_FORMAT_RGB332);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(s->display), &error_fatal);

    s->buttons = qdev_new(TYPE_PEBBLE_GPIO);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(s->buttons), &error_fatal);
    pbl_gpio_set_callback(s->buttons, sf32lb52_button_state, s);

    s->touch = qdev_new(TYPE_PEBBLE_TOUCH);
    qdev_prop_set_uint32(s->touch, "display-width", 200);
    qdev_prop_set_uint32(s->touch, "display-height", 228);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(s->touch), &error_fatal);
    pbl_touch_set_callback(s->touch, sf32lb52_touch_state, s);

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

static const Property sf32lb52_flash_properties[] = {
    DEFINE_PROP_DRIVE("drive", SF32LB52FlashState, blk),
};

static void sf32lb52_flash_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    device_class_set_props(dc, sf32lb52_flash_properties);
    set_bit(DEVICE_CATEGORY_STORAGE, dc->categories);
    dc->desc = "SiFli SF32LB52 external flash backing";
}

static const TypeInfo sf32lb52_flash_info = {
    .name = TYPE_SF32LB52_FLASH,
    .parent = TYPE_DEVICE,
    .instance_size = sizeof(SF32LB52FlashState),
    .class_init = sf32lb52_flash_class_init,
};

static const TypeInfo sf32lb52_machine_info = {
    .name = TYPE_SF32LB52_MACHINE,
    .parent = TYPE_MACHINE,
    .instance_size = sizeof(SF32LB52MachineState),
    .class_init = sf32lb52_machine_class_init,
};

static void sf32lb52_machine_register_types(void)
{
    type_register_static(&sf32lb52_flash_info);
    type_register_static(&sf32lb52_machine_info);
}

type_init(sf32lb52_machine_register_types)
