/*
 * QTest tests for the SiFli SF32LB52 machine
 *
 * Copyright (c) 2026 Core Devices LLC
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "libqtest.h"

#define HPSYS_RAM_BASE     0x20000000
#define LPSYS_RAM_BASE     0x20400000
#define HPSYS_RCC_DLL1CR   0x5000002c
#define HPSYS_RCC_HRCCAL1  0x50000034
#define EFUSEC_CR          0x5000c000
#define EFUSEC_SR          0x5000c008
#define MPI2_CR            0x50042000
#define MPI2_DR            0x50042004
#define MPI2_SR            0x50042010
#define MPI2_SCR           0x50042014
#define MPI2_CMDR1         0x50042018
#define MPI2_AR1           0x5004201c
#define TRNG_CTRL          0x5000f000
#define TRNG_STAT          0x5000f004
#define TRNG_RAND_NUM0     0x5000f030
#define WDT1_CCR           0x5009400c
#define WDT1_SR            0x50094014
#define DMAC1_ISR          0x50081000
#define DMAC1_IFCR         0x50081004
#define DMAC1_CCR2         0x5008101c
#define DMAC1_CNDTR2       0x50081020
#define DMAC1_CPAR2        0x50081024
#define DMAC1_CM0AR2       0x50081028
#define I2C2_CR            0x5009d000
#define I2C1_BASE          0x5009c000
#define I2C2_BASE          0x5009d000
#define I2C3_BASE          0x5009e000
#define I2C_TCR            0x04
#define I2C_IER            0x08
#define I2C_SR             0x0c
#define I2C_DBR            0x10
#define AUDCODEC_PLL_CFG0   0x50088080
#define AUDCODEC_PLL_CAL_CFG 0x500880a4
#define AUDCODEC_PLL_CAL_RESULT 0x500880a8
#define HPSYS_AON_ACR      0x500c0010
#define PMUC_LRC32_CR      0x500ca01c
#define RTC_ISR            0x500cb00c
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
#define DMAC_ISR_GIF2      (1U << 4)
#define DMAC_ISR_TCIF2     (1U << 5)
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

    i2c_write_register(qts, I2C2_BASE, 0x6a, 0x12, 0x01);
    g_assert_cmphex(i2c_read_register(qts, I2C2_BASE, 0x6a, 0x12), ==, 0);

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
    qtest_add_func("sf32lb52/dmac1-channel2", test_dmac1_channel2);
    qtest_add_func("sf32lb52/flash-program-erase",
                   test_flash_program_and_erase);
    qtest_add_func("sf32lb52/i2c1", test_i2c1);
    qtest_add_func("sf32lb52/obelix-i2c-devices", test_obelix_i2c_devices);
    qtest_add_func("sf32lb52/usart1", test_usart1);

    return g_test_run();
}
