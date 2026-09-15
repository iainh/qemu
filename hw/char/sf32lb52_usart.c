/*
 * SiFli SF32LB52 USART
 *
 * Copyright (c) 2026 Core Devices LLC
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "hw/irq.h"
#include "hw/sysbus.h"
#include "hw/qdev-properties-system.h"
#include "chardev/char-fe.h"
#include "qom/object.h"

#define TYPE_SF32LB52_USART "sf32lb52-usart"
OBJECT_DECLARE_SIMPLE_TYPE(SF32LB52UsartState, SF32LB52_USART)

#define USART_CR1  0x00
#define USART_CR2  0x04
#define USART_CR3  0x08
#define USART_BRR  0x0c
#define USART_GTPR 0x10
#define USART_RTOR 0x14
#define USART_RQR  0x18
#define USART_ISR  0x1c
#define USART_ICR  0x20
#define USART_RDR  0x24
#define USART_TDR  0x28
#define USART_SIZE 0x40

#define CR1_RXNEIE (1U << 5)
#define CR1_TCIE   (1U << 6)
#define CR1_TXEIE  (1U << 7)
#define CR1_UE      (1U << 0)
#define CR1_RE      (1U << 2)
#define CR1_TE      (1U << 3)

#define ISR_RXNE   (1U << 5)
#define ISR_TC     (1U << 6)
#define ISR_TXE    (1U << 7)
#define ISR_TEACK  (1U << 21)
#define ISR_REACK  (1U << 22)

#define RQR_RXFRQ  (1U << 3)

#define RX_FIFO_SIZE 256

struct SF32LB52UsartState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    CharBackend chr;
    qemu_irq irq;
    uint32_t regs[USART_SIZE / sizeof(uint32_t)];
    uint8_t rx_fifo[RX_FIFO_SIZE];
    unsigned int rx_head;
    unsigned int rx_count;
};

static void sf32lb52_usart_update(SF32LB52UsartState *s)
{
    uint32_t isr = ISR_TXE | ISR_TC;

    if (s->rx_count) {
        isr |= ISR_RXNE;
    }
    if (s->regs[USART_CR1 / 4] & CR1_UE) {
        if (s->regs[USART_CR1 / 4] & CR1_TE) {
            isr |= ISR_TEACK;
        }
        if (s->regs[USART_CR1 / 4] & CR1_RE) {
            isr |= ISR_REACK;
        }
    }
    s->regs[USART_ISR / 4] = isr;
    qemu_set_irq(s->irq,
                 ((s->regs[USART_CR1 / 4] & CR1_RXNEIE) && s->rx_count) ||
                 (s->regs[USART_CR1 / 4] & (CR1_TCIE | CR1_TXEIE)));
}

static uint64_t sf32lb52_usart_read(void *opaque, hwaddr offset,
                                    unsigned int size)
{
    SF32LB52UsartState *s = opaque;

    if (offset == USART_RDR) {
        uint8_t value = 0;

        if (s->rx_count) {
            value = s->rx_fifo[s->rx_head];
            s->rx_head = (s->rx_head + 1) % RX_FIFO_SIZE;
            s->rx_count--;
            sf32lb52_usart_update(s);
        }
        return value;
    }
    return s->regs[offset / 4];
}

static void sf32lb52_usart_write(void *opaque, hwaddr offset,
                                 uint64_t value, unsigned int size)
{
    SF32LB52UsartState *s = opaque;

    switch (offset) {
    case USART_TDR: {
        uint8_t byte = value;
        qemu_chr_fe_write(&s->chr, &byte, 1);
        break;
    }
    case USART_RQR:
        if (value & RQR_RXFRQ) {
            s->rx_head = 0;
            s->rx_count = 0;
        }
        break;
    case USART_ICR:
        break;
    case USART_ISR:
    case USART_RDR:
        return;
    default:
        s->regs[offset / 4] = value;
        break;
    }
    sf32lb52_usart_update(s);
}

static const MemoryRegionOps sf32lb52_usart_ops = {
    .read = sf32lb52_usart_read,
    .write = sf32lb52_usart_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static int sf32lb52_usart_can_receive(void *opaque)
{
    SF32LB52UsartState *s = opaque;

    return RX_FIFO_SIZE - s->rx_count;
}

static void sf32lb52_usart_receive(void *opaque, const uint8_t *buf, int size)
{
    SF32LB52UsartState *s = opaque;

    size = MIN(size, RX_FIFO_SIZE - s->rx_count);
    for (int i = 0; i < size; i++) {
        unsigned int tail = (s->rx_head + s->rx_count) % RX_FIFO_SIZE;

        s->rx_fifo[tail] = buf[i];
        s->rx_count++;
    }
    sf32lb52_usart_update(s);
}

static void sf32lb52_usart_reset(DeviceState *dev)
{
    SF32LB52UsartState *s = SF32LB52_USART(dev);

    memset(s->regs, 0, sizeof(s->regs));
    s->rx_head = 0;
    s->rx_count = 0;
    sf32lb52_usart_update(s);
}

static void sf32lb52_usart_realize(DeviceState *dev, Error **errp)
{
    SF32LB52UsartState *s = SF32LB52_USART(dev);

    qemu_chr_fe_set_handlers(&s->chr, sf32lb52_usart_can_receive,
                             sf32lb52_usart_receive, NULL, NULL,
                             s, NULL, true);
}

static void sf32lb52_usart_init(Object *obj)
{
    SF32LB52UsartState *s = SF32LB52_USART(obj);

    memory_region_init_io(&s->iomem, obj, &sf32lb52_usart_ops, s,
                          TYPE_SF32LB52_USART, USART_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
}

static const Property sf32lb52_usart_properties[] = {
    DEFINE_PROP_CHR("chardev", SF32LB52UsartState, chr),
};

static void sf32lb52_usart_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = sf32lb52_usart_realize;
    device_class_set_legacy_reset(dc, sf32lb52_usart_reset);
    device_class_set_props(dc, sf32lb52_usart_properties);
}

static const TypeInfo sf32lb52_usart_info = {
    .name = TYPE_SF32LB52_USART,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(SF32LB52UsartState),
    .instance_init = sf32lb52_usart_init,
    .class_init = sf32lb52_usart_class_init,
};

static void sf32lb52_usart_register_types(void)
{
    type_register_static(&sf32lb52_usart_info);
}

type_init(sf32lb52_usart_register_types)
