// samc21 serial port
//
// Copyright (C) 2018-2019  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "board/armcm_boot.h" // armcm_enable_irq
#include "board/serial_irq.h" // serial_rx_data
#include "command.h" // DECL_CONSTANT_STR
#include "internal.h" // enable_pclock
#include "sched.h" // DECL_INIT

#ifdef BUTTON_SERIAL
#define SERCOM_NUM SERCOM5
#else
#define SERCOM_NUM SERCOM2
#endif

void
serial_enable_tx_irq(void)
{
    SERCOM_NUM->USART.INTENSET.reg = SERCOM_USART_INTENSET_RXC | SERCOM_USART_INTENSET_DRE;
}

#ifdef BUTTON_SERIAL
void
SERCOM5_Handler(void)
{
    uint32_t status = SERCOM5->USART.INTFLAG.reg;
    if (status & SERCOM_USART_INTFLAG_RXC) {
        serial_rx_byte(SERCOM5->USART.DATA.reg);
    }
    if (status & SERCOM_USART_INTFLAG_DRE) {
        uint8_t data;
        int ret = serial_get_tx_byte(&data);
        if (ret)
          SERCOM5->USART.INTENCLR.reg = SERCOM_USART_INTENSET_DRE;
        else
          SERCOM5->USART.DATA.reg = data;
    }
}
#else
void
SERCOM2_Handler(void)
{
    uint32_t status = SERCOM2->USART.INTFLAG.reg;
    if (status & SERCOM_USART_INTFLAG_RXC) {
        serial_rx_byte(SERCOM2->USART.DATA.reg);
    }
    if (status & SERCOM_USART_INTFLAG_DRE) {
        uint8_t data;
        int ret = serial_get_tx_byte(&data);
        if (ret)
          SERCOM2->USART.INTENCLR.reg = SERCOM_USART_INTENSET_DRE;
        else
          SERCOM2->USART.DATA.reg = data;
    }
}
#endif

#ifdef BUTTON_SERIAL
DECL_CONSTANT_STR("RESERVE_PINS_serial", "PB23,PB22");
#else
DECL_CONSTANT_STR("RESERVE_PINS_serial", "PA13,PA12");
#endif

void
serial_init(void)
{
#ifdef BUTTON_SERIAL
    uint32_t freq = get_pclock_frequency(SERCOM5_GCLK_ID_CORE);
#else
    uint32_t freq = get_pclock_frequency(SERCOM2_GCLK_ID_CORE);
#endif
    uint64_t br = (uint64_t)65536 * (freq - 16 * CONFIG_SERIAL_BAUD) / freq;

    // Enable pins
#ifdef BUTTON_SERIAL
    gpio_peripheral(GPIO('B', 23), 'D', 0);
    gpio_peripheral(GPIO('B', 22), 'D', 0);
#else
    gpio_peripheral(GPIO('A', 13), 'C', 0);
    gpio_peripheral(GPIO('A', 12), 'C', 0);
#endif

    // Enable serial clock
#ifdef BUTTON_SERIAL
    enable_pclock(SERCOM5_GCLK_ID_CORE, MCLK_APBCMASK_SERCOM5);
#else
    enable_pclock(SERCOM2_GCLK_ID_CORE, MCLK_APBCMASK_SERCOM2);
#endif

    SERCOM_NUM->USART.CTRLA.reg =
      SERCOM_USART_CTRLA_DORD | SERCOM_USART_CTRLA_MODE(1/*USART_INT_CLK*/) |
#ifdef BUTTON_SERIAL
      SERCOM_USART_CTRLA_RXPO(3/*PAD3*/) | SERCOM_USART_CTRLA_TXPO(1/*PAD2*/);
#else
      SERCOM_USART_CTRLA_RXPO(1/*PAD1*/) | SERCOM_USART_CTRLA_TXPO(0/*PAD0*/);
#endif

    SERCOM_NUM->USART.CTRLB.reg = SERCOM_USART_CTRLB_RXEN | SERCOM_USART_CTRLB_TXEN |
      SERCOM_USART_CTRLB_CHSIZE(0/*8 bits*/);

    SERCOM_NUM->USART.BAUD.reg = (uint16_t)br;

    SERCOM_NUM->USART.CTRLA.reg |= SERCOM_USART_CTRLA_ENABLE;
    SERCOM_NUM->USART.INTENSET.reg = SERCOM_USART_INTENSET_RXC | SERCOM_USART_INTENSET_DRE;
#ifdef BUTTON_SERIAL
    armcm_enable_irq(SERCOM5_Handler, SERCOM5_IRQn, 0);
#else
    armcm_enable_irq(SERCOM2_Handler, SERCOM2_IRQn, 0);
#endif
}
DECL_INIT(serial_init);
