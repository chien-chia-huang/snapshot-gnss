/****************************************************************************
 * softwareUart.c
 * SnapperGPS
 * July 2026
 *****************************************************************************/

#include "em_cmu.h"
#include "em_gpio.h"
#include "em_timer.h"

#include "softwareUart.h"
#include "pinouts.h"

/* Bit-banged UART settings */

#define SOFTWAREUART_BAUDRATE          115200

#define SOFTWAREUART_BITS_PER_BYTE     8

/* Use GPIO1 as the TX pin. There is no RX side: this is a transmit-only
 * link used to stream data off the device in place of the USB interface. */

#define SOFTWAREUART_TX_PORT           GPIO1_PORT
#define SOFTWAREUART_TX_PIN            GPIO1_PIN

/* TIMER0 is used as a free-running tick source for bit timing. It is not
 * used by any other module in this firmware version (unlike the standard
 * firmware, which reserves it for the USB stack). */

#define SOFTWAREUART_TIMER             TIMER0

/* Private function */

/* Busy-wait until the timer's free-running 16-bit counter reaches
 * targetTicks. Comparing with a signed 16-bit difference makes this
 * correct across counter wraparound, as long as targetTicks is never
 * more than half a counter period (32768 ticks) ahead of the actual
 * counter value, which holds here since callers only ever look one bit
 * period ahead. */

static void waitUntilTick(uint16_t targetTicks) {

    while ((int16_t)(targetTicks - (uint16_t)TIMER_CounterGet(SOFTWAREUART_TIMER)) > 0) { }

}

/* Public functions */

void SoftwareUart_init() {

    // Configure the TX pin as a push-pull output, idling high

    GPIO_PinModeSet(SOFTWAREUART_TX_PORT, SOFTWAREUART_TX_PIN, gpioModePushPull, 1);

}

void SoftwareUart_enable() {

    CMU_ClockEnable(cmuClock_TIMER0, true);

    TIMER_Init_TypeDef init = TIMER_INIT_DEFAULT;

    init.enable = false;

    TIMER_Init(SOFTWAREUART_TIMER, &init);

}

void SoftwareUart_disable() {

    TIMER_Enable(SOFTWAREUART_TIMER, false);

    CMU_ClockEnable(cmuClock_TIMER0, false);

}

void SoftwareUart_transmit(const uint8_t *data, uint32_t length) {

    // Read the live peripheral clock frequency so that timing stays correct
    // regardless of whether the core clock is currently HFRCO or HFXO

    uint32_t timerFrequency = CMU_ClockFreqGet(cmuClock_TIMER0);

    uint16_t ticksPerBit = (uint16_t)(timerFrequency / SOFTWAREUART_BAUDRATE);

    TIMER_CounterSet(SOFTWAREUART_TIMER, 0);

    TIMER_Enable(SOFTWAREUART_TIMER, true);

    uint16_t target = ticksPerBit;

    for (uint32_t i = 0; i < length; ++i) {

        uint8_t byte = data[i];

        // Start bit (low)

        GPIO_PinOutClear(SOFTWAREUART_TX_PORT, SOFTWAREUART_TX_PIN);

        waitUntilTick(target);

        target += ticksPerBit;

        // 8 data bits, least significant bit first

        for (uint8_t bit = 0; bit < SOFTWAREUART_BITS_PER_BYTE; ++bit) {

            if (byte & (1 << bit)) {

                GPIO_PinOutSet(SOFTWAREUART_TX_PORT, SOFTWAREUART_TX_PIN);

            } else {

                GPIO_PinOutClear(SOFTWAREUART_TX_PORT, SOFTWAREUART_TX_PIN);

            }

            waitUntilTick(target);

            target += ticksPerBit;

        }

        // Stop bit (high)

        GPIO_PinOutSet(SOFTWAREUART_TX_PORT, SOFTWAREUART_TX_PIN);

        waitUntilTick(target);

        target += ticksPerBit;

    }

    TIMER_Enable(SOFTWAREUART_TIMER, false);

}
