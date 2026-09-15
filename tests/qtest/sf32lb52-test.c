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
#define MPI2_DR            0x50042004
#define MPI2_SR            0x50042010
#define MPI2_SCR           0x50042014
#define MPI2_CMDR1         0x50042018
#define TRNG_CTRL          0x5000f000
#define TRNG_STAT          0x5000f004
#define TRNG_RAND_NUM0     0x5000f030
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
#define SPI_FLASH_RDID     0x9f
#define SPI_FLASH_ID       0x1840ef
#define TRNG_GEN_RAND      (1U << 1)
#define TRNG_RAND_VALID    (1U << 3)
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

    qtest_writel(qts, RTC_ISR, RTC_ISR_INIT);
    g_assert_cmphex(qtest_readl(qts, RTC_ISR) &
                    (RTC_ISR_RSF | RTC_ISR_INITF), ==,
                    RTC_ISR_RSF | RTC_ISR_INITF);

    qtest_writel(qts, TRNG_CTRL, TRNG_GEN_RAND);
    g_assert_cmphex(qtest_readl(qts, TRNG_STAT) & TRNG_RAND_VALID, ==,
                    TRNG_RAND_VALID);
    g_assert_cmphex(qtest_readl(qts, TRNG_RAND_NUM0), !=, 0);

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
    qtest_add_func("sf32lb52/usart1", test_usart1);

    return g_test_run();
}
