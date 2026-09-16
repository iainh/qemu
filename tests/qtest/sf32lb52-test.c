/*
 * QTest tests for the SiFli SF32LB52 machine
 *
 * Copyright (c) 2026 Core Devices LLC
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/units.h"
#include "libqtest.h"

#define HPSYS_RAM_BASE     0x20000000
#define LPSYS_RAM_BASE     0x20400000
#define HPSYS_RCC_DLL1CR   0x5000002c
#define HPSYS_RCC_HRCCAL1  0x50000034
#define EFUSEC_CR          0x5000c000
#define EFUSEC_SR          0x5000c008
#define EFUSEC_BANK0_DATA0 0x5000c030
#define EFUSEC_BANK0_DATA1 0x5000c034
#define MPI2_CR            0x50042000
#define MPI2_DR            0x50042004
#define MPI2_SR            0x50042010
#define MPI2_SCR           0x50042014
#define MPI2_CMDR1         0x50042018
#define MPI2_AR1           0x5004201c
#define TRNG_CTRL          0x5000f000
#define TRNG_STAT          0x5000f004
#define TRNG_RAND_NUM0     0x5000f030
#define LCDC1_IRQ          0x50008008
#define LCDC1_SETTING      0x5000800c
#define LCDC1_CANVAS_TL    0x50008010
#define LCDC1_CANVAS_BR    0x50008014
#define LCDC1_LAYER0_SRC   0x5000802c
#define LCDC1_JDI_CTRL     0x500080ec
#define WDT1_CCR           0x5009400c
#define WDT1_SR            0x50094014
#define DMAC1_ISR          0x50081000
#define DMAC1_IFCR         0x50081004
#define DMAC1_CCR2         0x5008101c
#define DMAC1_CNDTR2       0x50081020
#define DMAC1_CPAR2        0x50081024
#define DMAC1_CM0AR2       0x50081028
#define DMAC1_CCR5         0x50081058
#define DMAC1_CNDTR5       0x5008105c
#define DMAC1_CPAR5        0x50081060
#define DMAC1_CM0AR5       0x50081064
#define PDM1_DATA_L        0x5009a040
#define I2C2_CR            0x5009d000
#define I2C1_BASE          0x5009c000
#define I2C2_BASE          0x5009d000
#define I2C3_BASE          0x5009e000
#define I2C_TCR            0x04
#define I2C_IER            0x08
#define I2C_SR             0x0c
#define I2C_DBR            0x10
#define GPIO1_BANK0        0x500a0000
#define GPIO1_BANK1        0x500a0080
#define GPIO_DIR           0x00
#define GPIO_IER           0x1c
#define GPIO_IESR          0x20
#define GPIO_ITR           0x28
#define GPIO_ITSR          0x2c
#define GPIO_IPHR          0x34
#define GPIO_IPHSR         0x38
#define GPIO_IPLR          0x40
#define GPIO_IPLSR         0x44
#define GPIO_ISR           0x4c
#define AUDCODEC_PLL_CFG0   0x50088080
#define AUDCODEC_PLL_CAL_CFG 0x500880a4
#define AUDCODEC_PLL_CAL_RESULT 0x500880a8
#define HPSYS_AON_ACR      0x500c0010
#define HPSYS_AON_GTIMR    0x500c0034
#define PMUC_LRC32_CR      0x500ca01c
#define HPSYS_CFG_RTC_TR   0x5000b014
#define HPSYS_CFG_RTC_DR   0x5000b018
#define RTC_TR             0x500cb000
#define RTC_DR             0x500cb004
#define RTC_CR             0x500cb008
#define RTC_ISR            0x500cb00c
#define RTC_ALRMTR         0x500cb018
#define RTC_ALRMDR         0x500cb01c
#define RTC_BKP2R          0x500cb038
#define PMUC_CR            0x500ca000
#define LPTIM1_ISR         0x500c1000
#define LPTIM1_ICR         0x500c1004
#define LPTIM1_IER         0x500c1008
#define LPTIM1_CR          0x500c1010
#define LPTIM1_ARR         0x500c1018
#define LPTIM1_CNT         0x500c101c
#define GPTIM2_CR1         0x500b0000
#define GPTIM2_DIER        0x500b000c
#define GPTIM2_SR          0x500b0010
#define GPTIM2_PSC         0x500b0028
#define GPTIM2_ARR         0x500b002c
#define MAILBOX1_C1IER     0x50082000
#define MAILBOX1_C1ITR     0x50082004
#define MAILBOX2_C1ICR     0x40002008
#define MAILBOX2_C1MISR    0x40002010
#define BT_RFC_VCO_REG1    0x40082800
#define BT_RFC_VCO_REG3    0x40082808
#define BT_RFC_FBDV_REG1   0x40082814
#define BT_RFC_FBDV_REG2   0x40082818
#define BT_RFC_EDR_CAL_REG1 0x40082824
#define BT_RFC_ROSCAL_REG2 0x40082880
#define BT_RFC_RCROSCAL_REG 0x40082884
#define BT_RFC_PACAL_REG   0x40082888
#define BT_PHY_TX_HFP_CFG  0x40084118
#define GPADC_IRQ           0x50087044
#define DMAC2_ISR           0x40001000
#define LPSYS_AON_ACR       0x40040010
#define BT_MAC_RCCAL_CTRL   0x40090114
#define BT_MAC_RCCAL_RESULT 0x40090118
#define HCPU2LCPU_RING     0x2007fe00
#define LCPU2HCPU_RING     0x20405c00
#define IPC_RING_HEADER    20
#define USART1_BASE        0x50084000
#define DWT_BASE           0xe0001000
#define DWT_CTRL           0x00
#define DWT_CYCCNT         0x04
#define USART_CR1          0x00
#define USART_ISR          0x1c
#define USART_TDR          0x28

#define HRCCAL1_CAL_EN     (1U << 30)
#define HRCCAL1_CAL_DONE   (1U << 31)
#define DLLCR_EN           (1U << 0)
#define DLLCR_READY        (1U << 31)
#define EFUSEC_CR_EN       (1U << 0)
#define EFUSEC_SR_DONE     (1U << 0)
#define MPI_SR_TCF         (1U << 0)
#define MPI_SR_SMF         (1U << 3)
#define MPI_CR_CMD2E       (1U << 16)
#define MPI_CR_SME2        (1U << 18)
#define SPI_FLASH_RDID     0x9f
#define SPI_FLASH_ID       0x1940c8
#define SPI_FLASH_QPP      0x32
#define SPI_FLASH_SE       0x20
#define TRNG_GEN_RAND      (1U << 1)
#define TRNG_RAND_VALID    (1U << 3)
#define WDT_CMD_STOP       0x34
#define WDT_CMD_START      0x76
#define WDT_SR_ACTIVE      (1U << 1)
#define DMAC_CCR_EN        (1U << 0)
#define DMAC_CCR_TCIE      (1U << 1)
#define DMAC_CCR_HTIE      (1U << 2)
#define DMAC_CCR_CIRC      (1U << 5)
#define DMAC_CCR_MINC      (1U << 7)
#define DMAC_ISR_GIF2      (1U << 4)
#define DMAC_ISR_TCIF2     (1U << 5)
#define DMAC_ISR_GIF5      (1U << 16)
#define DMAC_ISR_TCIF5     (1U << 17)
#define DMAC_ISR_HTIF5     (1U << 18)
#define I2C_CR_RSTREQ      (1U << 30)
#define I2C_TCR_TB         (1U << 0)
#define I2C_TCR_START      (1U << 1)
#define I2C_TCR_STOP       (1U << 2)
#define I2C_SR_TE          (1U << 6)
#define I2C_SR_RF          (1U << 7)
#define I2C_SR_MSD         (1U << 12)
#define PLL_CFG0_FC_VCO_SHIFT 17
#define PLL_CAL_EN         (1U << 0)
#define PLL_CAL_DONE       (1U << 1)
#define PLL_CAL_LEN        (2000U << 16)
#define RTC_ISR_RSF        (1U << 7)
#define RTC_ISR_INITF      (1U << 9)
#define RTC_ISR_INIT       (1U << 10)
#define RTC_CR_ALRME       (1U << 8)
#define RTC_CR_ALRMIE      (1U << 11)
#define RTC_ISR_ALRMF      (1U << 1)
#define PMUC_CR_REBOOT     (1U << 2)
#define RTC_ALRMMASK_DATE  (1U << 27)
#define RTC_ALRMMASK_MONTH (1U << 28)
#define RTC_ALRMMASK_WDAY  (1U << 29)
#define LPTIM_ISR_OF       (1U << 1)
#define LPTIM_ISR_OFWKUP   (1U << 9)
#define LPTIM_IER_OFIE     (1U << 1)
#define LPTIM_CR_ENABLE    (1U << 0)
#define LPTIM_CR_SNGSTRT   (1U << 1)
#define GPTIM_CR1_CEN      (1U << 0)
#define GPTIM_DIER_UIE     (1U << 0)
#define GPTIM_SR_UIF       (1U << 0)
#define ACR_HXT48_REQ      (1U << 1)
#define ACR_HXT48_RDY      (1U << 31)
#define LRC32_EN           (1U << 0)
#define LRC32_RDY          (1U << 31)
#define DWT_CYCCNTENA      (1U << 0)
#define USART_CR1_UE       (1U << 0)
#define USART_CR1_RE       (1U << 2)
#define USART_CR1_TE       (1U << 3)
#define USART_CR1_TXEIE    (1U << 7)
#define USART_ISR_TC       (1U << 6)
#define USART_ISR_TXE      (1U << 7)
#define USART_ISR_TEACK    (1U << 21)
#define USART_ISR_REACK    (1U << 22)

static QTestState *sf32lb52_start(void)
{
    return qtest_init("-machine sf32lb52 -display none -serial null");
}

static QTestState *sf32lb52_start_with_flash(const char *path)
{
    return qtest_initf("-machine sf32lb52 -display none -serial null "
                       "-drive if=mtd,format=raw,file=%s", path);
}

static void test_memory(void)
{
    QTestState *qts = sf32lb52_start();

    qtest_writel(qts, HPSYS_RAM_BASE, 0x12345678);
    g_assert_cmphex(qtest_readl(qts, HPSYS_RAM_BASE), ==, 0x12345678);
    qtest_writel(qts, HPSYS_RAM_BASE + 0x7fffc, 0x87654321);
    g_assert_cmphex(qtest_readl(qts, HPSYS_RAM_BASE + 0x7fffc), ==,
                    0x87654321);
    qtest_writel(qts, LPSYS_RAM_BASE, 0xa5a55a5a);
    g_assert_cmphex(qtest_readl(qts, LPSYS_RAM_BASE), ==, 0xa5a55a5a);

    qtest_quit(qts);
}

static void test_startup_handshakes(void)
{
    QTestState *qts = sf32lb52_start();

    qtest_writel(qts, LPSYS_AON_ACR, ACR_HXT48_REQ);
    g_assert_cmphex(qtest_readl(qts, LPSYS_AON_ACR) & ACR_HXT48_RDY, ==,
                    ACR_HXT48_RDY);

    qtest_writel(qts, HPSYS_AON_ACR, ACR_HXT48_REQ);
    g_assert_cmphex(qtest_readl(qts, HPSYS_AON_ACR) & ACR_HXT48_RDY, ==,
                    ACR_HXT48_RDY);

    qtest_writel(qts, PMUC_LRC32_CR, LRC32_EN);
    g_assert_cmphex(qtest_readl(qts, PMUC_LRC32_CR) & LRC32_RDY, ==,
                    LRC32_RDY);

    qtest_writel(qts, HPSYS_RCC_HRCCAL1, HRCCAL1_CAL_EN);
    g_assert_cmphex(qtest_readl(qts, HPSYS_RCC_HRCCAL1) & HRCCAL1_CAL_DONE,
                    ==, HRCCAL1_CAL_DONE);

    qtest_writel(qts, HPSYS_RCC_DLL1CR, DLLCR_EN);
    g_assert_cmphex(qtest_readl(qts, HPSYS_RCC_DLL1CR) & DLLCR_READY, ==,
                    DLLCR_READY);

    qtest_writel(qts, EFUSEC_CR, EFUSEC_CR_EN);
    g_assert_cmphex(qtest_readl(qts, EFUSEC_SR) & EFUSEC_SR_DONE, ==,
                    EFUSEC_SR_DONE);
    g_assert_cmphex(qtest_readl(qts, EFUSEC_BANK0_DATA0), ==, 0x42455000);
    g_assert_cmphex(qtest_readl(qts, EFUSEC_BANK0_DATA1), ==, 0xa575524c);

    qtest_writel(qts, MPI2_CMDR1, SPI_FLASH_RDID);
    g_assert_cmphex(qtest_readl(qts, MPI2_SR) & MPI_SR_TCF, ==, MPI_SR_TCF);
    g_assert_cmphex(qtest_readl(qts, MPI2_DR), ==, SPI_FLASH_ID);
    qtest_writel(qts, MPI2_SCR, MPI_SR_TCF);
    g_assert_cmphex(qtest_readl(qts, MPI2_SR) & MPI_SR_TCF, ==, 0);
    qtest_writel(qts, MPI2_CR, MPI_CR_CMD2E | MPI_CR_SME2);
    qtest_writel(qts, MPI2_CMDR1, 0x20);
    g_assert_cmphex(qtest_readl(qts, MPI2_SR) & MPI_SR_SMF, ==, MPI_SR_SMF);
    qtest_writel(qts, MPI2_SCR, MPI_SR_SMF);
    g_assert_cmphex(qtest_readl(qts, MPI2_SR) & MPI_SR_SMF, ==, 0);

    qtest_writel(qts, RTC_ISR, RTC_ISR_INIT);
    g_assert_cmphex(qtest_readl(qts, RTC_ISR) &
                    (RTC_ISR_RSF | RTC_ISR_INITF), ==,
                    RTC_ISR_RSF | RTC_ISR_INITF);

    qtest_writel(qts, TRNG_CTRL, TRNG_GEN_RAND);
    g_assert_cmphex(qtest_readl(qts, TRNG_STAT) & TRNG_RAND_VALID, ==,
                    TRNG_RAND_VALID);
    g_assert_cmphex(qtest_readl(qts, TRNG_RAND_NUM0), !=, 0);

    qtest_writel(qts, WDT1_CCR, WDT_CMD_START);
    g_assert_cmphex(qtest_readl(qts, WDT1_SR) & WDT_SR_ACTIVE, ==,
                    WDT_SR_ACTIVE);
    qtest_writel(qts, WDT1_CCR, WDT_CMD_STOP);
    g_assert_cmphex(qtest_readl(qts, WDT1_SR) & WDT_SR_ACTIVE, ==, 0);

    qtest_writel(qts, I2C2_CR, I2C_CR_RSTREQ);
    g_assert_cmphex(qtest_readl(qts, I2C2_CR) & I2C_CR_RSTREQ, ==, 0);

    qtest_writel(qts, AUDCODEC_PLL_CFG0, 16 << PLL_CFG0_FC_VCO_SHIFT);
    qtest_writel(qts, AUDCODEC_PLL_CAL_CFG, PLL_CAL_LEN | PLL_CAL_EN);
    g_assert_cmphex(qtest_readl(qts, AUDCODEC_PLL_CAL_CFG) & PLL_CAL_DONE,
                    ==, PLL_CAL_DONE);
    g_assert_cmphex(qtest_readl(qts, AUDCODEC_PLL_CAL_RESULT), ==,
                    (2000U << 16) | 2000);
    qtest_writel(qts, AUDCODEC_PLL_CAL_CFG, 0);
    g_assert_cmphex(qtest_readl(qts, AUDCODEC_PLL_CAL_CFG) & PLL_CAL_DONE,
                    ==, 0);
    qtest_writel(qts, AUDCODEC_PLL_CFG0, 8 << PLL_CFG0_FC_VCO_SHIFT);
    qtest_writel(qts, AUDCODEC_PLL_CAL_CFG, PLL_CAL_LEN | PLL_CAL_EN);
    g_assert_cmphex(qtest_readl(qts, AUDCODEC_PLL_CAL_RESULT), ==,
                    (1800U << 16) | 2000);

    qtest_quit(qts);
}

static void test_dwt_cycle_counter(void)
{
    QTestState *qts = sf32lb52_start();

    qtest_writel(qts, DWT_BASE + DWT_CYCCNT, 100);
    qtest_writel(qts, DWT_BASE + DWT_CTRL, DWT_CYCCNTENA);
    qtest_clock_step(qts, 1000);
    g_assert_cmphex(qtest_readl(qts, DWT_BASE + DWT_CYCCNT), ==, 340);

    qtest_quit(qts);
}

static uint32_t rtc_time(unsigned int hour, unsigned int minute,
                         unsigned int second)
{
    return ((hour / 10 << 4 | hour % 10) << 25) |
           ((minute / 10 << 4 | minute % 10) << 18) |
           ((second / 10 << 4 | second % 10) << 11);
}

static uint32_t rtc_date(unsigned int year, unsigned int month,
                         unsigned int day)
{
    return (1U << 24) | ((year / 10 << 4 | year % 10) << 16) |
           ((month / 10 << 4 | month % 10) << 8) |
           (day / 10 << 4 | day % 10);
}

static void test_rtc_and_low_power_timer(void)
{
    QTestState *qts = sf32lb52_start();
    uint32_t gtimer;

    qtest_writel(qts, RTC_TR, rtc_time(12, 34, 58));
    qtest_writel(qts, RTC_DR, rtc_date(26, 9, 16));
    qtest_clock_step(qts, 3 * G_TIME_SPAN_SECOND * 1000);
    g_assert_cmphex(qtest_readl(qts, RTC_TR) & ~0x3ff, ==,
                    rtc_time(12, 35, 1));
    g_assert_cmphex(qtest_readl(qts, RTC_DR) & 0x01ff1f3f, ==,
                    rtc_date(26, 9, 16));
    g_assert_cmphex(qtest_readl(qts, HPSYS_CFG_RTC_TR) & ~0x3ff, ==,
                    rtc_time(12, 35, 1));
    g_assert_cmphex(qtest_readl(qts, HPSYS_CFG_RTC_DR) & 0x01ff1f3f, ==,
                    rtc_date(26, 9, 16));

    qtest_writel(qts, RTC_ALRMTR, rtc_time(12, 35, 2));
    qtest_writel(qts, RTC_ALRMDR,
                 RTC_ALRMMASK_DATE | RTC_ALRMMASK_MONTH |
                 RTC_ALRMMASK_WDAY);
    qtest_writel(qts, RTC_CR, RTC_CR_ALRME | RTC_CR_ALRMIE);
    qtest_clock_step(qts, G_TIME_SPAN_SECOND * 1000);
    g_assert_cmphex(qtest_readl(qts, RTC_ISR) & RTC_ISR_ALRMF, ==,
                    RTC_ISR_ALRMF);
    qtest_writel(qts, RTC_ISR, 0);
    g_assert_cmphex(qtest_readl(qts, RTC_ISR) & RTC_ISR_ALRMF, ==, 0);

    qtest_writel(qts, LPTIM1_ARR, 100);
    qtest_writel(qts, LPTIM1_IER, LPTIM_IER_OFIE);
    qtest_writel(qts, LPTIM1_CR, LPTIM_CR_ENABLE | LPTIM_CR_SNGSTRT);
    qtest_clock_step(qts, 9 * G_TIME_SPAN_MILLISECOND * 1000);
    g_assert_cmpuint(qtest_readl(qts, LPTIM1_CNT), ==, 90);
    g_assert_cmphex(qtest_readl(qts, LPTIM1_ISR), ==, 0);
    qtest_clock_step(qts, 2 * G_TIME_SPAN_MILLISECOND * 1000);
    g_assert_cmphex(qtest_readl(qts, LPTIM1_ISR), ==,
                    LPTIM_ISR_OF | LPTIM_ISR_OFWKUP);
    qtest_writel(qts, LPTIM1_ICR, LPTIM_ISR_OF);
    g_assert_cmphex(qtest_readl(qts, LPTIM1_ISR), ==, LPTIM_ISR_OFWKUP);

    gtimer = qtest_readl(qts, HPSYS_AON_GTIMR);
    qtest_clock_step(qts, G_TIME_SPAN_MILLISECOND * 1000);
    g_assert_cmpuint(qtest_readl(qts, HPSYS_AON_GTIMR) - gtimer, >=, 32);

    qtest_quit(qts);
}

static void test_gptim2(void)
{
    QTestState *qts = sf32lb52_start();

    qtest_writel(qts, GPTIM2_PSC, 2399);
    qtest_writel(qts, GPTIM2_ARR, 19);
    qtest_writel(qts, GPTIM2_DIER, GPTIM_DIER_UIE);
    qtest_writel(qts, GPTIM2_CR1, GPTIM_CR1_CEN);
    qtest_clock_step(qts, G_TIME_SPAN_MILLISECOND * 1000);
    g_assert_cmphex(qtest_readl(qts, GPTIM2_SR) & GPTIM_SR_UIF, ==, 0);
    qtest_clock_step(qts, G_TIME_SPAN_MILLISECOND * 1000);
    g_assert_cmphex(qtest_readl(qts, GPTIM2_SR) & GPTIM_SR_UIF, ==,
                    GPTIM_SR_UIF);

    qtest_writel(qts, GPTIM2_SR, ~GPTIM_SR_UIF);
    g_assert_cmphex(qtest_readl(qts, GPTIM2_SR) & GPTIM_SR_UIF, ==, 0);
    qtest_clock_step(qts, 2 * G_TIME_SPAN_MILLISECOND * 1000);
    g_assert_cmphex(qtest_readl(qts, GPTIM2_SR) & GPTIM_SR_UIF, ==,
                    GPTIM_SR_UIF);

    qtest_writel(qts, GPTIM2_CR1, 0);
    qtest_writel(qts, GPTIM2_SR, ~GPTIM_SR_UIF);
    qtest_clock_step(qts, 2 * G_TIME_SPAN_MILLISECOND * 1000);
    g_assert_cmphex(qtest_readl(qts, GPTIM2_SR) & GPTIM_SR_UIF, ==, 0);

    qtest_quit(qts);
}

static void test_pmu_reboot(void)
{
    QTestState *qts = sf32lb52_start();

    qtest_writel(qts, RTC_TR, rtc_time(12, 34, 58));
    qtest_writel(qts, RTC_DR, rtc_date(26, 9, 16));
    qtest_writel(qts, RTC_BKP2R, 0x12345678);
    qtest_writel(qts, GPTIM2_PSC, 2399);
    qtest_clock_step(qts, 2 * G_TIME_SPAN_SECOND * 1000);

    qtest_writel(qts, PMUC_CR, PMUC_CR_REBOOT);
    qtest_qmp_eventwait(qts, "RESET");

    g_assert_cmphex(qtest_readl(qts, RTC_BKP2R), ==, 0x12345678);
    g_assert_cmphex(qtest_readl(qts, RTC_TR) & ~0x3ff, ==,
                    rtc_time(12, 35, 0));
    g_assert_cmphex(qtest_readl(qts, RTC_DR) & 0x01ff1f3f, ==,
                    rtc_date(26, 9, 16));
    g_assert_cmphex(qtest_readl(qts, GPTIM2_PSC), ==, 0);

    qtest_quit(qts);
}

static void test_dmac1_channel2(void)
{
    QTestState *qts = sf32lb52_start();

    qtest_writel(qts, DMAC1_CNDTR2, 37);
    qtest_writel(qts, DMAC1_CCR2, DMAC_CCR_EN);
    g_assert_cmphex(qtest_readl(qts, DMAC1_CNDTR2), ==, 0);
    g_assert_cmphex(qtest_readl(qts, DMAC1_CCR2) & DMAC_CCR_EN, ==, 0);
    g_assert_cmphex(qtest_readl(qts, DMAC1_ISR), ==,
                    DMAC_ISR_GIF2 | DMAC_ISR_TCIF2);

    qtest_writel(qts, DMAC1_IFCR, DMAC_ISR_TCIF2);
    g_assert_cmphex(qtest_readl(qts, DMAC1_ISR), ==, 0);

    qtest_quit(qts);
}

static void test_audio_dma(void)
{
    QTestState *qts = sf32lb52_start();
    uint8_t samples[16];

    qtest_memset(qts, HPSYS_RAM_BASE, 0xff, sizeof(samples));
    qtest_writel(qts, DMAC1_CNDTR5, 8);
    qtest_writel(qts, DMAC1_CPAR5, PDM1_DATA_L);
    qtest_writel(qts, DMAC1_CM0AR5, HPSYS_RAM_BASE);
    qtest_writel(qts, DMAC1_CCR5,
                 DMAC_CCR_EN | DMAC_CCR_TCIE | DMAC_CCR_HTIE |
                 DMAC_CCR_CIRC | DMAC_CCR_MINC);
    qtest_clock_step(qts, G_TIME_SPAN_MILLISECOND * 1000);
    g_assert_cmphex(qtest_readl(qts, DMAC1_ISR) &
                    (DMAC_ISR_GIF5 | DMAC_ISR_HTIF5), ==,
                    DMAC_ISR_GIF5 | DMAC_ISR_HTIF5);
    qtest_memread(qts, HPSYS_RAM_BASE, samples, sizeof(samples));
    g_assert_cmphex(samples[0], ==, 0x00);
    g_assert_cmphex(samples[1], ==, 0xfc);
    qtest_writel(qts, DMAC1_IFCR, DMAC_ISR_GIF5);
    qtest_clock_step(qts, G_TIME_SPAN_MILLISECOND * 1000);
    g_assert_cmphex(qtest_readl(qts, DMAC1_ISR) &
                    (DMAC_ISR_GIF5 | DMAC_ISR_TCIF5), ==,
                    DMAC_ISR_GIF5 | DMAC_ISR_TCIF5);
    g_assert_cmphex(qtest_readl(qts, DMAC1_CCR5) & DMAC_CCR_EN, ==,
                    DMAC_CCR_EN);

    qtest_quit(qts);
}

static void test_flash_program_and_erase(void)
{
    QTestState *qts = sf32lb52_start();
    const uint32_t flash_offset = 0x1000000;

    qtest_writel(qts, MPI2_AR1, flash_offset);
    qtest_writel(qts, MPI2_CMDR1, SPI_FLASH_SE);
    g_assert_cmphex(qtest_readl(qts, 0x12000000 + flash_offset), ==,
                    0xffffffff);

    qtest_writel(qts, HPSYS_RAM_BASE, 0x5aa53cc3);
    qtest_writel(qts, DMAC1_CNDTR2, 4);
    qtest_writel(qts, DMAC1_CPAR2, MPI2_DR);
    qtest_writel(qts, DMAC1_CM0AR2, HPSYS_RAM_BASE);
    qtest_writel(qts, DMAC1_CCR2, DMAC_CCR_EN);
    qtest_writel(qts, MPI2_CMDR1, SPI_FLASH_QPP);
    g_assert_cmphex(qtest_readl(qts, 0x12000000 + flash_offset), ==,
                    0x5aa53cc3);

    qtest_writel(qts, MPI2_CMDR1, SPI_FLASH_SE);
    g_assert_cmphex(qtest_readl(qts, 0x12000000 + flash_offset), ==,
                    0xffffffff);

    qtest_quit(qts);
}

static void test_flash_backing(void)
{
    const uint32_t flash_offset = 0x1000000;
    const uint32_t initial = 0x12345678;
    const uint32_t programmed = 0xa55ac33c;
    g_autofree char *path = NULL;
    QTestState *qts;
    uint32_t persisted;
    int fd;

    fd = g_file_open_tmp("sf32lb52-flash-XXXXXX", &path, NULL);
    g_assert_cmpint(fd, >=, 0);
    g_assert_cmpint(ftruncate(fd, 32 * MiB), ==, 0);
    g_assert_cmpint(pwrite(fd, &initial, sizeof(initial), flash_offset), ==,
                    sizeof(initial));
    close(fd);

    qts = sf32lb52_start_with_flash(path);
    g_assert_cmphex(qtest_readl(qts, 0x12000000 + flash_offset), ==,
                    initial);
    qtest_writel(qts, MPI2_AR1, flash_offset);
    qtest_writel(qts, MPI2_CMDR1, SPI_FLASH_SE);
    qtest_writel(qts, HPSYS_RAM_BASE, programmed);
    qtest_writel(qts, DMAC1_CNDTR2, sizeof(programmed));
    qtest_writel(qts, DMAC1_CPAR2, MPI2_DR);
    qtest_writel(qts, DMAC1_CM0AR2, HPSYS_RAM_BASE);
    qtest_writel(qts, DMAC1_CCR2, DMAC_CCR_EN);
    qtest_writel(qts, MPI2_CMDR1, SPI_FLASH_QPP);
    qtest_quit(qts);

    fd = open(path, O_RDONLY);
    g_assert_cmpint(fd, >=, 0);
    g_assert_cmpint(pread(fd, &persisted, sizeof(persisted), flash_offset), ==,
                    sizeof(persisted));
    close(fd);
    g_assert_cmphex(persisted, ==, programmed);
    unlink(path);
}

static void test_lcdc_jdi(void)
{
    const uint32_t jdi_irq = (1U << 20) | (1U << 4);
    const uint32_t eof_irq = (1U << 16) | (1U << 0);
    QTestState *qts = sf32lb52_start();

    qtest_memset(qts, HPSYS_RAM_BASE, 0xe0, 200 * 2);
    qtest_writel(qts, LCDC1_CANVAS_TL, 0);
    qtest_writel(qts, LCDC1_CANVAS_BR, (3U << 16) | 199);
    qtest_writel(qts, LCDC1_LAYER0_SRC, HPSYS_RAM_BASE);
    qtest_writel(qts, LCDC1_SETTING, (1U << 4) | (1U << 0));
    qtest_writel(qts, LCDC1_JDI_CTRL, 1);

    qtest_clock_step(qts, 1000);
    g_assert_cmphex(qtest_readl(qts, LCDC1_IRQ), ==, jdi_irq);
    qtest_writel(qts, LCDC1_IRQ, jdi_irq);
    qtest_clock_step(qts, 1000);
    g_assert_cmphex(qtest_readl(qts, LCDC1_IRQ), ==, jdi_irq);
    qtest_writel(qts, LCDC1_IRQ, jdi_irq);
    qtest_clock_step(qts, 1000);
    g_assert_cmphex(qtest_readl(qts, LCDC1_IRQ), ==, eof_irq);
    qtest_writel(qts, LCDC1_IRQ, eof_irq);
    g_assert_cmphex(qtest_readl(qts, LCDC1_IRQ), ==, 0);

    qtest_quit(qts);
}

static void test_i2c1(void)
{
    QTestState *qts = sf32lb52_start();
    const uint32_t address = 0x58;

    qtest_writel(qts, I2C1_BASE + I2C_DBR, address << 1);
    qtest_writel(qts, I2C1_BASE + I2C_TCR, I2C_TCR_START | I2C_TCR_TB);
    g_assert_cmphex(qtest_readl(qts, I2C1_BASE + I2C_SR), ==, I2C_SR_TE);
    qtest_writel(qts, I2C1_BASE + I2C_SR, I2C_SR_TE);

    qtest_writel(qts, I2C1_BASE + I2C_DBR, 0x20);
    qtest_writel(qts, I2C1_BASE + I2C_TCR, I2C_TCR_TB);
    qtest_writel(qts, I2C1_BASE + I2C_SR, I2C_SR_TE);
    qtest_writel(qts, I2C1_BASE + I2C_DBR, 0xa5);
    qtest_writel(qts, I2C1_BASE + I2C_TCR, I2C_TCR_TB | I2C_TCR_STOP);
    g_assert_cmphex(qtest_readl(qts, I2C1_BASE + I2C_SR), ==,
                    I2C_SR_TE | I2C_SR_MSD);
    qtest_writel(qts, I2C1_BASE + I2C_SR, I2C_SR_TE | I2C_SR_MSD);

    qtest_writel(qts, I2C1_BASE + I2C_DBR, address << 1);
    qtest_writel(qts, I2C1_BASE + I2C_TCR, I2C_TCR_START | I2C_TCR_TB);
    qtest_writel(qts, I2C1_BASE + I2C_SR, I2C_SR_TE);
    qtest_writel(qts, I2C1_BASE + I2C_DBR, 0x20);
    qtest_writel(qts, I2C1_BASE + I2C_TCR, I2C_TCR_TB);
    qtest_writel(qts, I2C1_BASE + I2C_SR, I2C_SR_TE);
    qtest_writel(qts, I2C1_BASE + I2C_DBR, (address << 1) | 1);
    qtest_writel(qts, I2C1_BASE + I2C_TCR, I2C_TCR_START | I2C_TCR_TB);
    qtest_writel(qts, I2C1_BASE + I2C_SR, I2C_SR_TE);
    qtest_writel(qts, I2C1_BASE + I2C_TCR, I2C_TCR_TB | I2C_TCR_STOP);
    g_assert_cmphex(qtest_readl(qts, I2C1_BASE + I2C_SR), ==,
                    I2C_SR_RF | I2C_SR_MSD);
    g_assert_cmphex(qtest_readl(qts, I2C1_BASE + I2C_DBR), ==, 0xa5);

    qtest_quit(qts);
}

static uint8_t i2c_read_register(QTestState *qts, uint32_t base,
                                 uint8_t address, uint8_t reg)
{
    qtest_writel(qts, base + I2C_DBR, address << 1);
    qtest_writel(qts, base + I2C_TCR, I2C_TCR_START | I2C_TCR_TB);
    qtest_writel(qts, base + I2C_SR, I2C_SR_TE);
    qtest_writel(qts, base + I2C_DBR, reg);
    qtest_writel(qts, base + I2C_TCR, I2C_TCR_TB);
    qtest_writel(qts, base + I2C_SR, I2C_SR_TE);
    qtest_writel(qts, base + I2C_DBR, (address << 1) | 1);
    qtest_writel(qts, base + I2C_TCR, I2C_TCR_START | I2C_TCR_TB);
    qtest_writel(qts, base + I2C_SR, I2C_SR_TE);
    qtest_writel(qts, base, 0x10d);
    qtest_writel(qts, base + I2C_IER, I2C_SR_RF | I2C_SR_MSD);
    qtest_writel(qts, base + I2C_TCR, I2C_TCR_TB | I2C_TCR_STOP);

    return qtest_readl(qts, base + I2C_DBR);
}

static uint8_t i2c_read_register16(QTestState *qts, uint32_t base,
                                   uint8_t address, uint16_t reg)
{
    qtest_writel(qts, base + I2C_DBR, address << 1);
    qtest_writel(qts, base + I2C_TCR, I2C_TCR_START | I2C_TCR_TB);
    qtest_writel(qts, base + I2C_SR, I2C_SR_TE);
    qtest_writel(qts, base + I2C_DBR, reg >> 8);
    qtest_writel(qts, base + I2C_TCR, I2C_TCR_TB);
    qtest_writel(qts, base + I2C_SR, I2C_SR_TE);
    qtest_writel(qts, base + I2C_DBR, reg);
    qtest_writel(qts, base + I2C_TCR, I2C_TCR_TB | I2C_TCR_STOP);
    qtest_writel(qts, base + I2C_SR, I2C_SR_TE | I2C_SR_MSD);
    qtest_writel(qts, base + I2C_DBR, (address << 1) | 1);
    qtest_writel(qts, base + I2C_TCR, I2C_TCR_START | I2C_TCR_TB);
    qtest_writel(qts, base + I2C_SR, I2C_SR_TE);
    qtest_writel(qts, base + I2C_TCR, I2C_TCR_TB | I2C_TCR_STOP);

    return qtest_readl(qts, base + I2C_DBR);
}

static void i2c_write_register(QTestState *qts, uint32_t base,
                               uint8_t address, uint8_t reg, uint8_t data)
{
    qtest_writel(qts, base + I2C_DBR, address << 1);
    qtest_writel(qts, base + I2C_TCR, I2C_TCR_START | I2C_TCR_TB);
    qtest_writel(qts, base + I2C_SR, I2C_SR_TE);
    qtest_writel(qts, base + I2C_DBR, reg);
    qtest_writel(qts, base + I2C_TCR, I2C_TCR_TB);
    qtest_writel(qts, base + I2C_SR, I2C_SR_TE);
    qtest_writel(qts, base + I2C_DBR, data);
    qtest_writel(qts, base + I2C_TCR, I2C_TCR_TB | I2C_TCR_STOP);
}

static void i2c_read_block(QTestState *qts, uint32_t base, uint8_t address,
                           uint8_t reg, uint8_t *data, size_t length)
{
    qtest_writel(qts, base + I2C_DBR, address << 1);
    qtest_writel(qts, base + I2C_TCR, I2C_TCR_START | I2C_TCR_TB);
    qtest_writel(qts, base + I2C_DBR, reg);
    qtest_writel(qts, base + I2C_TCR, I2C_TCR_TB);
    qtest_writel(qts, base + I2C_DBR, (address << 1) | 1);
    qtest_writel(qts, base + I2C_TCR, I2C_TCR_START | I2C_TCR_TB);
    for (size_t i = 0; i < length; i++) {
        qtest_writel(qts, base + I2C_TCR,
                     I2C_TCR_TB | (i == length - 1 ? I2C_TCR_STOP : 0));
        data[i] = qtest_readl(qts, base + I2C_DBR);
    }
}

static void test_obelix_motion_samples(void)
{
    QTestState *qts = sf32lb52_start();
    uint8_t samples[140];

    i2c_write_register(qts, I2C2_BASE, 0x6a, 0x10, 0x30);
    i2c_write_register(qts, I2C2_BASE, 0x6a, 0x07, 2);
    i2c_write_register(qts, I2C2_BASE, 0x6a, 0x09, 3);
    i2c_write_register(qts, I2C2_BASE, 0x6a, 0x0a, 6);
    qtest_clock_step(qts, 390 * G_TIME_SPAN_MILLISECOND * 1000);
    g_assert_cmphex(i2c_read_register(qts, I2C2_BASE, 0x6a, 0x3a), ==, 20);
    g_assert_cmphex(i2c_read_register(qts, I2C2_BASE, 0x6a, 0x3b) & 0x80,
                    ==, 0x80);
    i2c_read_block(qts, I2C2_BASE, 0x6a, 0x78,
                   samples, sizeof(samples));
    g_assert_cmphex(samples[0], ==, 0x10);
    g_assert_cmphex(samples[138], ==, 0x00);
    g_assert_cmphex(samples[139], ==, 0x40);
    g_assert_cmphex(i2c_read_register(qts, I2C2_BASE, 0x6a, 0x3a), ==, 0);

    qtest_quit(qts);
}

static void test_lcpu_hci(void)
{
    QTestState *qts = sf32lb52_start();
    const uint8_t reset[] = { 0x01, 0x03, 0x0c, 0x00 };
    const uint8_t ready_event[] = {
        0x04, 0x0e, 0x04, 0x01, 0x11, 0xfc, 0x00,
    };
    const uint8_t reset_event[] = {
        0x04, 0x0e, 0x04, 0x01, 0x03, 0x0c, 0x00,
    };
    uint8_t event[7];

    qtest_writel(qts, MAILBOX1_C1IER, 1);
    qtest_clock_step(qts, G_TIME_SPAN_MILLISECOND * 1000);
    g_assert_cmphex(qtest_readl(qts, MAILBOX2_C1MISR), ==, 1);
    g_assert_cmphex(qtest_readl(qts, LCPU2HCPU_RING + 12), ==, 7U << 16);
    qtest_memread(qts, LCPU2HCPU_RING + IPC_RING_HEADER, event,
                  sizeof(event));
    g_assert_cmpmem(event, sizeof(event), ready_event, sizeof(ready_event));
    qtest_writel(qts, MAILBOX2_C1ICR, 1);
    qtest_writel(qts, LCPU2HCPU_RING + 8, 7U << 16);

    qtest_writel(qts, HCPU2LCPU_RING + 8, 0);
    qtest_writel(qts, HCPU2LCPU_RING + 12, sizeof(reset) << 16);
    qtest_writew(qts, HCPU2LCPU_RING + 16, 512 - IPC_RING_HEADER);
    qtest_memwrite(qts, HCPU2LCPU_RING + IPC_RING_HEADER,
                   reset, sizeof(reset));
    qtest_writel(qts, MAILBOX1_C1ITR, 1);

    g_assert_cmphex(qtest_readl(qts, LCPU2HCPU_RING + 12), ==, 14U << 16);
    qtest_memread(qts, LCPU2HCPU_RING + IPC_RING_HEADER + 7,
                  event, sizeof(event));
    g_assert_cmpmem(event, sizeof(event), reset_event, sizeof(reset_event));

    qtest_quit(qts);
}

static void test_bt_rf_calibration(void)
{
    QTestState *qts = sf32lb52_start();

    qtest_writel(qts, BT_RFC_VCO_REG3, 128);
    qtest_writel(qts, BT_RFC_FBDV_REG1, 1U << 2);
    g_assert_cmphex(qtest_readl(qts, BT_RFC_FBDV_REG1) & 1, ==, 1);
    g_assert_cmpuint(qtest_readl(qts, BT_RFC_FBDV_REG2) >> 16, ==, 33880);

    qtest_writel(qts, BT_PHY_TX_HFP_CFG, 63U << 22);
    g_assert_cmpuint(qtest_readl(qts, BT_RFC_FBDV_REG2) >> 16, ==, 35140);
    qtest_writel(qts, BT_RFC_VCO_REG1, 1U << 13);
    qtest_writel(qts, BT_RFC_EDR_CAL_REG1, 128);
    g_assert_cmpuint(qtest_readl(qts, BT_RFC_FBDV_REG2) >> 16, ==, 34940);

    g_assert_cmphex(qtest_readl(qts, BT_RFC_PACAL_REG) & (1U << 1), !=, 0);
    g_assert_cmphex(qtest_readl(qts, BT_RFC_ROSCAL_REG2) & 1, !=, 0);
    g_assert_cmphex(qtest_readl(qts, BT_RFC_RCROSCAL_REG) & (1U << 20),
                    !=, 0);
    g_assert_cmphex(qtest_readl(qts, GPADC_IRQ) & (1U << 2), !=, 0);
    g_assert_cmphex(qtest_readl(qts, DMAC2_ISR) & (1U << 29), !=, 0);
    qtest_writel(qts, BT_MAC_RCCAL_CTRL, 20);
    g_assert_cmphex(qtest_readl(qts, BT_MAC_RCCAL_RESULT), ==,
                    (1U << 31) | 96000);

    qtest_quit(qts);
}

static void test_obelix_i2c_devices(void)
{
    QTestState *qts = sf32lb52_start();

    g_assert_cmphex(i2c_read_register(qts, I2C1_BASE, 0x64, 0x00), ==,
                    0x09);
    g_assert_cmphex(i2c_read_register(qts, I2C1_BASE, 0x48, 0x3e), ==,
                    0xe5);
    g_assert_cmphex(i2c_read_register(qts, I2C2_BASE, 0x6a, 0x0f), ==,
                    0x6c);
    g_assert_cmphex(i2c_read_register(qts, I2C2_BASE, 0x30, 0x39), ==,
                    0x10);
    g_assert_cmphex(i2c_read_register(qts, I2C3_BASE, 0x15, 0xa9), ==,
                    0x07);
    g_assert_cmphex(i2c_read_register16(qts, I2C1_BASE, 0x6b, 0x0003), ==,
                    0x43);
    g_assert_cmphex(i2c_read_register16(qts, I2C1_BASE, 0x6b, 0x0510), ==,
                    0x04);
    g_assert_cmphex(i2c_read_register16(qts, I2C1_BASE, 0x6b, 0x0511), ==,
                    0xc7);
    g_assert_cmphex(i2c_read_register16(qts, I2C1_BASE, 0x6b, 0x0512), ==,
                    0x80);
    g_assert_cmphex(i2c_read_register16(qts, I2C1_BASE, 0x6b, 0x0515), ==,
                    0x02);

    i2c_write_register(qts, I2C2_BASE, 0x6a, 0x12, 0x01);
    g_assert_cmphex(i2c_read_register(qts, I2C2_BASE, 0x6a, 0x12), ==, 0);

    qtest_quit(qts);
}

static void gpio_enable_both_edges(QTestState *qts, uint32_t base,
                                   uint32_t mask)
{
    qtest_writel(qts, base + GPIO_ITSR, mask);
    qtest_writel(qts, base + GPIO_IPHSR, mask);
    qtest_writel(qts, base + GPIO_IPLSR, mask);
    qtest_writel(qts, base + GPIO_IESR, mask);

    g_assert_cmphex(qtest_readl(qts, base + GPIO_ITR) & mask, ==, mask);
    g_assert_cmphex(qtest_readl(qts, base + GPIO_IPHR) & mask, ==, mask);
    g_assert_cmphex(qtest_readl(qts, base + GPIO_IPLR) & mask, ==, mask);
    g_assert_cmphex(qtest_readl(qts, base + GPIO_IER) & mask, ==, mask);
}

static void test_obelix_buttons(void)
{
    QTestState *qts = sf32lb52_start();
    uint32_t back = 1U << (34 - 32);

    gpio_enable_both_edges(qts, GPIO1_BANK1, back);
    g_assert_cmphex(qtest_readl(qts, GPIO1_BANK1 + GPIO_DIR) & back, ==, 0);

    qtest_qmp_assert_success(qts,
        "{'execute': 'input-send-event', 'arguments': {'events': ["
        "{'type': 'key', 'data': {'down': true, "
        "'key': {'type': 'qcode', 'data': 'q'}}}]}}");

    g_assert_cmphex(qtest_readl(qts, GPIO1_BANK1 + GPIO_DIR) & back, ==,
                    back);
    g_assert_cmphex(qtest_readl(qts, GPIO1_BANK1 + GPIO_ISR) & back, ==,
                    back);
    qtest_writel(qts, GPIO1_BANK1 + GPIO_ISR, back);
    g_assert_cmphex(qtest_readl(qts, GPIO1_BANK1 + GPIO_ISR) & back, ==, 0);

    qtest_quit(qts);
}

static void test_obelix_touch(void)
{
    QTestState *qts = sf32lb52_start();
    uint32_t touch_int = 1U << 27;

    gpio_enable_both_edges(qts, GPIO1_BANK0, touch_int);
    qtest_qmp_assert_success(qts,
        "{'execute': 'input-send-event', 'arguments': {'events': ["
        "{'type': 'abs', 'data': {'axis': 'x', 'value': 16384}},"
        "{'type': 'abs', 'data': {'axis': 'y', 'value': 8192}},"
        "{'type': 'btn', 'data': {'down': true, 'button': 'left'}}]}}");

    g_assert_cmphex(qtest_readl(qts, GPIO1_BANK0 + GPIO_ISR) & touch_int, ==,
                    touch_int);
    g_assert_cmphex(i2c_read_register(qts, I2C3_BASE, 0x15, 0x02), ==, 1);
    g_assert_cmphex(i2c_read_register(qts, I2C3_BASE, 0x15, 0x03), ==, 0);
    g_assert_cmphex(i2c_read_register(qts, I2C3_BASE, 0x15, 0x04), ==, 100);
    g_assert_cmphex(i2c_read_register(qts, I2C3_BASE, 0x15, 0x05), ==, 0);
    g_assert_cmphex(i2c_read_register(qts, I2C3_BASE, 0x15, 0x06), ==, 57);

    qtest_quit(qts);
}

static void test_usart1(void)
{
    QTestState *qts = sf32lb52_start();

    g_assert_cmphex(qtest_readl(qts, USART1_BASE + USART_ISR), ==,
                    USART_ISR_TXE | USART_ISR_TC);
    qtest_writel(qts, USART1_BASE + USART_TDR, 'P');
    g_assert_cmphex(qtest_readl(qts, USART1_BASE + USART_ISR), ==,
                    USART_ISR_TXE | USART_ISR_TC);

    qtest_writel(qts, USART1_BASE + USART_CR1,
                 USART_CR1_UE | USART_CR1_RE | USART_CR1_TE | USART_CR1_TXEIE);
    g_assert_cmphex(qtest_readl(qts, USART1_BASE + USART_CR1), ==,
                    USART_CR1_UE | USART_CR1_RE | USART_CR1_TE |
                    USART_CR1_TXEIE);
    g_assert_cmphex(qtest_readl(qts, USART1_BASE + USART_ISR), ==,
                    USART_ISR_TXE | USART_ISR_TC | USART_ISR_TEACK |
                    USART_ISR_REACK);

    qtest_quit(qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    qtest_add_func("sf32lb52/memory", test_memory);
    qtest_add_func("sf32lb52/startup-handshakes", test_startup_handshakes);
    qtest_add_func("sf32lb52/dwt-cycle-counter", test_dwt_cycle_counter);
    qtest_add_func("sf32lb52/rtc-low-power-timer",
                   test_rtc_and_low_power_timer);
    qtest_add_func("sf32lb52/gptim2", test_gptim2);
    qtest_add_func("sf32lb52/pmu-reboot", test_pmu_reboot);
    qtest_add_func("sf32lb52/dmac1-channel2", test_dmac1_channel2);
    qtest_add_func("sf32lb52/audio-dma", test_audio_dma);
    qtest_add_func("sf32lb52/flash-program-erase",
                   test_flash_program_and_erase);
    qtest_add_func("sf32lb52/flash-backing", test_flash_backing);
    qtest_add_func("sf32lb52/lcdc-jdi", test_lcdc_jdi);
    qtest_add_func("sf32lb52/i2c1", test_i2c1);
    qtest_add_func("sf32lb52/obelix-i2c-devices", test_obelix_i2c_devices);
    qtest_add_func("sf32lb52/obelix-motion-samples",
                   test_obelix_motion_samples);
    qtest_add_func("sf32lb52/lcpu-hci", test_lcpu_hci);
    qtest_add_func("sf32lb52/bt-rf-calibration", test_bt_rf_calibration);
    qtest_add_func("sf32lb52/obelix-buttons", test_obelix_buttons);
    qtest_add_func("sf32lb52/obelix-touch", test_obelix_touch);
    qtest_add_func("sf32lb52/usart1", test_usart1);

    return g_test_run();
}
