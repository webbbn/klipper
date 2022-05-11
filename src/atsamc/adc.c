// Analog to Digital Converter support
//
// Copyright (C) 2018-2020  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "command.h" // shutdown
#include "gpio.h" // gpio_adc_read
#include "internal.h" // GPIO
#include "sched.h" // sched_shutdown

#define ADC_TEMPERATURE_PIN 0xfe
DECL_ENUMERATION("pin", "ADC_TEMPERATURE", ADC_TEMPERATURE_PIN);

#define SAMC51_ADC_SYNC(ADC, BIT) \
    while(ADC->SYNCBUSY.reg & ADC_SYNCBUSY_ ## BIT)
static const uint8_t adc_pins[] = {
    /* ADC0 */
    GPIO('A', 2), GPIO('A', 3), GPIO('B', 8), GPIO('B', 9), GPIO('A', 4),
    GPIO('A', 5), GPIO('A', 6), GPIO('A', 7), GPIO('A', 8), GPIO('A', 9),
    GPIO('A', 10), GPIO('A', 11),
    /* ADC1 */
    GPIO('B', 0), GPIO('B', 1), GPIO('B', 2), GPIO('B', 3), GPIO('B', 8),
    GPIO('B', 9), GPIO('B', 4), GPIO('B', 5), GPIO('B', 6), GPIO('B', 7),
    GPIO('A', 8), GPIO('A', 9)
};

DECL_CONSTANT("ADC_MAX", 4095);

static struct gpio_adc gpio_adc_pin_to_struct(uint8_t pin)
{
    // Find pin in adc_pins table
    uint8_t chan;
    for (chan=0; ; chan++) {
        if (chan >= ARRAY_SIZE(adc_pins))
            shutdown("Not a valid ADC pin");
        if (adc_pins[chan] == pin)
            break;
    }
    Adc* reg = (chan < 12 ? ADC0 : ADC1);
    chan %= 12;
    return (struct gpio_adc){ .regs=reg, .chan=chan };
}


static void
adc_init(void)
{
    static uint8_t have_run_init;
    if (have_run_init)
        return;
    have_run_init = 1;

    // Enable adc clock
    MCLK->APBCMASK.reg |= MCLK_APBCMASK_ADC0 | MCLK_APBCMASK_ADC1;
    enable_pclock(ADC0_GCLK_ID, GCLK_PCHCTRL_CHEN);
    enable_pclock(ADC1_GCLK_ID, GCLK_PCHCTRL_CHEN);

    // Set the AREF pin in ADC mode
    gpio_peripheral(GPIO('A', 3), 'A', 3);

    // Load calibration info
    // ADC0
    uint32_t refbuf = GET_FUSE(ADC0_FUSES_BIASREFBUF);
    uint32_t comp = GET_FUSE(ADC0_FUSES_BIASCOMP);
    ADC0->CALIB.reg = (ADC0_FUSES_BIASREFBUF(refbuf) | ADC0_FUSES_BIASCOMP(comp));

    // ADC1
    refbuf = GET_FUSE(ADC1_FUSES_BIASREFBUF);
    comp = GET_FUSE(ADC1_FUSES_BIASCOMP);
    ADC1->CALIB.reg = (ADC0_FUSES_BIASREFBUF(refbuf) | ADC0_FUSES_BIASCOMP(comp));

    // Reset
    ADC0->CTRLA.reg = ADC_CTRLA_SWRST;
    while (ADC0->CTRLA.reg & ADC_CTRLA_SWRST);
    ADC1->CTRLA.reg = ADC_CTRLA_SWRST;
    while (ADC1->CTRLA.reg & ADC_CTRLA_SWRST);

    // Setup and enable
    // ADC0
    ADC0->REFCTRL.reg = ADC_REFCTRL_REFSEL_AREFA | ADC_REFCTRL_REFCOMP;
    ADC0->CTRLB.reg = ADC_CTRLB_PRESCALER_DIV128;
    ADC0->CTRLC.reg = ADC_CTRLC_RESSEL_12BIT;
    ADC0->AVGCTRL.reg = ADC_AVGCTRL_SAMPLENUM_1;
    ADC0->SAMPCTRL.reg = ADC_SAMPCTRL_SAMPLEN(63);
    ADC0->INPUTCTRL.reg = ADC_INPUTCTRL_MUXPOS_AIN0 | ADC_INPUTCTRL_MUXNEG(0x18/*GND*/);
    ADC0->CTRLA.reg = ADC_CTRLA_ENABLE;

    // ADC1
    ADC1->REFCTRL.reg = ADC_REFCTRL_REFSEL_AREFA | ADC_REFCTRL_REFCOMP;
    ADC1->CTRLB.reg = ADC_CTRLB_PRESCALER_DIV128;
    ADC1->CTRLC.reg = ADC_CTRLC_RESSEL_12BIT;
    ADC1->AVGCTRL.reg = ADC_AVGCTRL_SAMPLENUM_1;
    ADC1->SAMPCTRL.reg = ADC_SAMPCTRL_SAMPLEN(63);
    ADC1->INPUTCTRL.reg = ADC_INPUTCTRL_MUXPOS_AIN1 | ADC_INPUTCTRL_MUXNEG(0x18/*GND*/);
    ADC1->CTRLA.reg = ADC_CTRLA_ENABLE;
}

struct gpio_adc
gpio_adc_setup(uint8_t pin)
{
    // Enable ADC
    adc_init();

/*
    if (pin == ADC_TEMPERATURE_PIN) {
        SUPC->VREF.reg |= SUPC_VREF_TSEN;
        return (struct gpio_adc){ .regs=ADC0,
                .chan=ADC_INPUTCTRL_MUXPOS_TEMP_Val };
    }

    // Set pin in ADC mode
    gpio_peripheral(pin, 'B', 0);
*/

    return gpio_adc_pin_to_struct(pin);
}

enum { ADC_DUMMY=0xff };
static uint8_t last_analog_read = ADC_DUMMY;

// Try to sample a value. Returns zero if sample ready, otherwise
// returns the number of clock ticks the caller should wait before
// retrying this function.
uint32_t
gpio_adc_sample(struct gpio_adc g)
{
    Adc *reg = g.regs;
    if (last_analog_read == g.chan) {
        if (reg->INTFLAG.reg & ADC_INTFLAG_RESRDY)
            // Sample now ready
            return 0;
        // ADC is still busy
        goto need_delay;
    }
    if (last_analog_read != ADC_DUMMY)
        // Sample on another channel in progress
        goto need_delay;
    last_analog_read = g.chan;

    // Set the channel to sample
    reg->INPUTCTRL.reg = (ADC_INPUTCTRL_MUXPOS(g.chan)
                          | ADC_INPUTCTRL_MUXNEG(0x18/*GND*/)
                          );

    SAMC51_ADC_SYNC(reg, INPUTCTRL);
    // Start the sample
    reg->SWTRIG.reg = ADC_SWTRIG_START;
    SAMC51_ADC_SYNC(reg, SWTRIG);

    // Schedule next attempt after sample is likely to be complete
need_delay:
    return 42 * 128 + 200; // 42 == 1 + (63+1)/2 + 1 + 12/2 + 1.5
}

// Read a value; use only after gpio_adc_sample() returns zero
uint16_t
gpio_adc_read(struct gpio_adc g)
{
    last_analog_read = ADC_DUMMY;
    return ((Adc *)g.regs)->RESULT.reg;
}

// Cancel a sample that may have been started with gpio_adc_sample()
void
gpio_adc_cancel_sample(struct gpio_adc g)
{
    Adc * reg = g.regs;
    if (last_analog_read == g.chan) {
        reg->SWTRIG.reg = ADC_SWTRIG_FLUSH;
        SAMC51_ADC_SYNC(reg, SWTRIG);
        reg->INTFLAG.reg = ADC_INTFLAG_RESRDY;
        last_analog_read = ADC_DUMMY;
    }
}
