#include <time.h>
#include <string.h>

#include "em_cmu.h"
#include "em_emu.h"
#include "em_rmu.h"
#include "em_rtc.h"
#include "em_chip.h"
#include "em_device.h"
#include "em_system.h"

#include "radio.h"
#include "timer.h"
#include "pinouts.h"
#include "softwareUart.h"
#include "analogToDigitalConverter.h"

/* LED pattern hyperparameters */

// Flash interval before the first snapshot has been captured

#define LED_INTERVAL_SECONDS                    5

/* Useful time constants */

#define MILLISECONDS_IN_SECOND                  1000

#define SECONDS_IN_MINUTE                       60
#define SECONDS_IN_HOUR                         (60 * SECONDS_IN_MINUTE)
#define SECONDS_IN_DAY                          (24 * SECONDS_IN_HOUR)

/* Clock frequencies */

#define HFXO_FREQ                               16368000
#define LFXO_FREQ                               32768

#define HFXO_FREQ_MARGIN                        10000

#define LFXO_CAL_COUNTS                         32

/* Hardware constants */

#define LFXO_TICKS_PER_SECOND                   1024

#define RTC_OVERFLOW                            (0x01 << 24)
#define RTC_DIV32                               5
#define RTC_COMP0                               0
#define RTC_COMP1                               1

#define MINIMUM_TICKS_TO_SLEEP                  128

#define TICKS_TO_CAPTURE                        13

#define DC_BOOST_BATTERY_THRESHOLD              350

/* Firmware description constants */

#define FIRMWARE_VERSION_LENGTH                 3
#define FIRMWARE_DESCRIPTION_LENGTH             32

/* Snapshot constants */

#define SNAPSHOT_BUFFER_LOCATION                0x20000800
#define SNAPSHOT_BUFFER_SIZE                    0x1800

/* Useful macros */

#define MAX(a,b)                                (((a) > (b)) ? (a) : (b))

#define ROUNDED_UP_DIV(a, b)                    (((a) + (b) - 1) / (b))

/*
 * Recording configuration.
 *
 * There is no USB host to configure these at runtime any more, so they are
 * compile-time constants instead. Edit them and rebuild if you need
 * different values.
 */

// Time between two snapshots, in seconds

#define MEASUREMENT_INTERVAL_SECONDS            20

// Unix timestamp of the first snapshot. 0 means "as soon as possible",
// i.e. at the next measurement interval boundary after boot.

#define RECORDING_START_TIME                    0

// Unix timestamp of the last snapshot. 0x7FFFFFFF effectively means
// "never", i.e. record until the device is reset or loses power.

#define RECORDING_END_TIME                      0x7FFFFFFF

/*
 * There is no USB host to set the clock any more either, so the device
 * clock is seeded once at boot from a compile-time Unix timestamp
 * instead. The Makefile regenerates buildtime.h with the current time
 * on every build (see $(GENDIR)buildtime.h in build/Makefile), so this
 * happens automatically; just build immediately before flashing. The
 * clock will free-run from that value afterwards; the SnapperGPS
 * post-processing method tolerates the resulting error of a few tens of
 * seconds without difficulty.
 */

#include "buildtime.h"

/* Software UART protocol constants */

#define UART_SYNC_WORD_LENGTH                   4

#define UART_CRC_POLY                           0x1021

typedef enum {
    UART_FRAME_INFO = 0x01,
    UART_FRAME_SNAPSHOT = 0x02
} uartFrameType_t;

/* Device state enumerations */

typedef enum {
    STATE_WILL_RECORD,                            // Device is recording snapshots
    STATE_WILL_SHUTDOWN                           // Recording has finished; device will enter deep sleep
} deviceState_t;

/* UART frame data structures */

#pragma pack(push, 1)

// Sent once at boot, before recording starts

typedef struct {
    uint8_t syncWord[UART_SYNC_WORD_LENGTH];
    uint8_t frameType;
    uint64_t deviceID;
    uint8_t firmwareVersion[FIRMWARE_VERSION_LENGTH];
    uint8_t firmwareDescription[FIRMWARE_DESCRIPTION_LENGTH];
    uint32_t measurementInterval;                 // Time between 2 snapshots in seconds
    uint32_t startTime;                            // Unix timestamp of first snapshot
    uint32_t endTime;                              // Unix timestamp of last snapshot
    uint32_t resetCause;                           // RMU_RSTCAUSE bits from the reset that preceded this boot
    uint16_t crc;                                  // CRC-16/CCITT over the bytes after syncWord
} uartInfoFrame_t;

// Sent once per captured snapshot. The raw snapshot bytes (snapshotLength
// bytes, read directly out of the snapshot buffer) follow this header on
// the wire, and a trailing uint16_t CRC-16/CCITT follows the snapshot
// bytes, covering everything from frameType onwards.

typedef struct {
    uint8_t syncWord[UART_SYNC_WORD_LENGTH];
    uint8_t frameType;
    uint32_t time;                                 // Unix timestamp (snapshot capture)
    uint16_t ticks;                                // Clock ticks (snapshot capture) [0-1023]
    int16_t temperature;                           // Tenths of a degree Celsius
    uint16_t batteryVoltage;                       // Hundredths of a volt
    uint16_t snapshotLength;                       // Number of raw snapshot bytes that follow
} uartSnapshotHeader_t;

#pragma pack(pop)

static const uint8_t uartSyncWord[UART_SYNC_WORD_LENGTH] = {0xAA, 0x55, 0xAA, 0x55};

/* Global device state variable */

static uint64_t timeOffset;

static deviceState_t state = STATE_WILL_RECORD;

static uint32_t measurementInterval = MEASUREMENT_INTERVAL_SECONDS;

static uint32_t startTime = RECORDING_START_TIME;

static uint32_t endTime = RECORDING_END_TIME;

/* Global volatile event flags */

static volatile bool eventRTC_Comp0;

static volatile bool eventRTC_Comp1;

/* Firmware version */

static uint8_t firmwareVersion[FIRMWARE_VERSION_LENGTH] = {0, 1, 0};

static uint8_t firmwareDescription[FIRMWARE_DESCRIPTION_LENGTH] = "SnapperGPS-UART";

/* Board version */

static bool legacyBoard = false;

/* Cause of the reset that preceded this boot (read before anything can
 * clear it), for diagnosing unexpected resets since there is no debugger
 * attached in normal use. */

static uint32_t resetCause;

/* Interrupt handlers */

void RTC_IRQHandler(void) {

    // Get interrupt cause

    uint32_t interruptMask = RTC_IntGet();

    // Handle interrupt

    if (interruptMask & RTC_IFC_COMP0) eventRTC_Comp0 = true;

    if (interruptMask & RTC_IFC_COMP1) eventRTC_Comp1 = true;

    if (interruptMask & RTC_IFC_OF) timeOffset += RTC_OVERFLOW;

    // Clear the RTC interrupt flag

    RTC_IntClear(interruptMask);

}

/* LED functions */

static void enableRedLED(bool enable) {

    GPIO_PinModeSet(RED_LED_PORT, RED_LED_PIN, gpioModePushPull, enable);

}

static void enableGreenLED(bool enable) {

    GPIO_PinModeSet(GREEN_LED_PORT, GREEN_LED_PIN, gpioModePushPull, enable);

}

/* DC boost function */

void enableDCBoost() {

    GPIO_PinModeSet(DC_BOOST_EN_PORT, DC_BOOST_EN_PIN, gpioModeInputPull, 1);

}

void disableDCBoost() {

    GPIO_PinModeSet(DC_BOOST_EN_PORT, DC_BOOST_EN_PIN, gpioModeInputPull, 0);

}

/* Time handling functions */

static void getTime(uint32_t *time, uint32_t *ticks, uint64_t *counter) {

    uint64_t rawCounter = RTC_CounterGet();

    uint64_t updatedCounter = rawCounter + timeOffset;

    if (time) *time = (uint32_t)(updatedCounter / LFXO_TICKS_PER_SECOND);

    if (ticks) *ticks = (uint32_t)(updatedCounter % LFXO_TICKS_PER_SECOND);

    if (counter) *counter = rawCounter;

}

static void setTime(uint32_t time, uint32_t ticks) {

    uint64_t requiredCounter = (uint64_t)time * LFXO_TICKS_PER_SECOND + (uint64_t)ticks;

    timeOffset = requiredCounter - (uint64_t)RTC_CounterGet();

}

/* CRC-16/CCITT helper functions */

static uint16_t updateCRC(uint16_t crc, int incr) {

    uint16_t xor = crc >> 15;

    uint16_t out = crc << 1;

    if (incr) out++;

    if (xor) out ^= UART_CRC_POLY;

    return out;

}

static uint16_t crcUpdateBytes(uint16_t crc, const uint8_t *data, uint32_t length) {

    for (uint32_t i = 0; i < length; ++i) {

        uint8_t byte = data[i];

        for (uint16_t mask = 0x80; mask > 0; mask >>= 1) {
            crc = updateCRC(crc, byte & mask);
        }

    }

    return crc;

}

static uint16_t crcFinalize(uint16_t crc) {

    for (uint16_t i = 0; i < 16; ++i) crc = updateCRC(crc, 0);

    return crc;

}

/* Software UART frame functions */

static void sendInfoFrame() {

    uartInfoFrame_t frame;

    memcpy(frame.syncWord, uartSyncWord, UART_SYNC_WORD_LENGTH);
    frame.frameType = UART_FRAME_INFO;
    frame.deviceID = SYSTEM_GetUnique();
    memcpy(frame.firmwareVersion, firmwareVersion, FIRMWARE_VERSION_LENGTH);
    memcpy(frame.firmwareDescription, firmwareDescription, FIRMWARE_DESCRIPTION_LENGTH);
    frame.measurementInterval = measurementInterval;
    frame.startTime = startTime;
    frame.endTime = endTime;
    frame.resetCause = resetCause;

    uint16_t crc = 0;

    crc = crcUpdateBytes(crc, (uint8_t*)&frame + UART_SYNC_WORD_LENGTH,
                          sizeof(frame) - UART_SYNC_WORD_LENGTH - sizeof(frame.crc));

    frame.crc = crcFinalize(crc);

    SoftwareUart_enable();

    SoftwareUart_transmit((uint8_t*)&frame, sizeof(frame));

    SoftwareUart_disable();

}

static void sendSnapshotFrame(uint32_t time, uint32_t ticks, int32_t temperature, uint32_t batteryVoltage) {

    uartSnapshotHeader_t header;

    memcpy(header.syncWord, uartSyncWord, UART_SYNC_WORD_LENGTH);
    header.frameType = UART_FRAME_SNAPSHOT;
    header.time = time;
    header.ticks = (uint16_t)ticks;
    header.temperature = (int16_t)temperature;
    header.batteryVoltage = (uint16_t)batteryVoltage;
    header.snapshotLength = SNAPSHOT_BUFFER_SIZE;

    uint16_t crc = 0;

    crc = crcUpdateBytes(crc, (uint8_t*)&header + UART_SYNC_WORD_LENGTH,
                          sizeof(header) - UART_SYNC_WORD_LENGTH);

    crc = crcUpdateBytes(crc, (uint8_t*)SNAPSHOT_BUFFER_LOCATION, SNAPSHOT_BUFFER_SIZE);

    crc = crcFinalize(crc);

    // Little-endian, like every other multi-byte field in this protocol

    uint8_t crcBytes[2] = {(uint8_t)crc, (uint8_t)(crc >> 8)};

    SoftwareUart_enable();

    SoftwareUart_transmit((uint8_t*)&header, sizeof(header));

    SoftwareUart_transmit((uint8_t*)SNAPSHOT_BUFFER_LOCATION, SNAPSHOT_BUFFER_SIZE);

    SoftwareUart_transmit(crcBytes, sizeof(crcBytes));

    SoftwareUart_disable();

}

/* HFXO measurement function */

static uint32_t measureHFXO() {

    CMU->CALCNT = LFXO_CAL_COUNTS - 1;

    CMU->CALCTRL = CMU_CALCTRL_UPSEL_HFXO | CMU_CALCTRL_DOWNSEL_LFXO;

    CMU->CMD |= CMU_CMD_CALSTART;

    while (CMU->STATUS & CMU_STATUS_CALBSY);

    return LFXO_FREQ / LFXO_CAL_COUNTS * CMU->CALCNT;

}

/* Main function */

int main(void) {

    // Capture and clear the reset cause before anything else can touch it

    resetCause = RMU_ResetCauseGet();

    RMU_ResetCauseClear();

    CHIP_Init();

    // Enable high frequency peripherals

    CMU_ClockEnable(cmuClock_HFPER, true);

    // Enable GPIO

    CMU_ClockEnable(cmuClock_GPIO, true);

    // Enable LFXO and low energy domain

    CMU_ClockEnable(cmuClock_CORELE, true);

    CMU_OscillatorEnable(cmuOsc_LFXO, true, true);

    CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_LFXO);

    // Enable DC boost

    enableDCBoost();

    // Enable timer

    Timer_enable();

    // Enable LED pins

    GPIO->ROUTE &= ~(GPIO_ROUTE_SWDIOPEN | GPIO_ROUTE_SWCLKPEN);

    // Enable RTC

    CMU_ClockEnable(cmuClock_RTC, true);

    CMU->LFAPRESC0 = RTC_DIV32;

    // Initialise real-time counter

    RTC_Init_TypeDef rtcInit = RTC_INIT_DEFAULT;

    rtcInit.comp0Top = false;

    RTC_Init(&rtcInit);

    // Check board version (battery voltage measurement circuit differs)

    AnalogToDigitalConverter_enable();
    // Measure voltage without battery voltage measurement enabled
    uint32_t voltageDisabled = AnalogToDigitalConverter_measureBatteryVoltage();
    // Enable battery voltage measurement assuming legacy board
    AnalogToDigitalConverter_enableBatteryMeasurement(PULL_LOW);
    // Check if voltage measurement increases (by more than threshold)
    uint32_t legacyBoardThreshold = 15;  // Centi-volts
    legacyBoard = AnalogToDigitalConverter_measureBatteryVoltage() >= voltageDisabled + legacyBoardThreshold;
    AnalogToDigitalConverter_disableBatteryMeasurement();
    AnalogToDigitalConverter_disable();

    // Seed the clock, since there is no USB host to set it at runtime

    setTime(INITIAL_UNIX_TIME, 0);

    // Set up the software UART TX pin

    SoftwareUart_init();

    // Announce the device and its recording configuration once

    sendInfoFrame();

    // Flash green LED to indicate initialized recording

    for (uint32_t i = 0; i < 10; i += 1) {
        enableGreenLED(true);
        Timer_delayMilliseconds(100);
        enableGreenLED(false);
        Timer_delayMilliseconds(100);
    }

    // Get the current time and the current real-time counter compare register value

    uint64_t counter;

    uint32_t time, ticks;

    getTime(&time, &ticks, &counter);

    // Calculate the current time rounded up to the next integer measurement interval

    time_t rawTime = time;

    struct tm *tm = gmtime(&rawTime);

    uint32_t secondsSinceStartOfDay = SECONDS_IN_HOUR * tm->tm_hour + SECONDS_IN_MINUTE * tm->tm_min + tm->tm_sec;

    int64_t currentTimeRoundedUpToMeasurementInterval = (int64_t)time - (int64_t)secondsSinceStartOfDay + (int64_t)measurementInterval * (int64_t)ROUNDED_UP_DIV(secondsSinceStartOfDay, measurementInterval);

    // Calculate the number of ticks until the later of the start time and the current time rounded up to the next integer measurement interval

    int64_t ticksUntilFirstInterrupt = MAX(currentTimeRoundedUpToMeasurementInterval, (int64_t)startTime) * LFXO_TICKS_PER_SECOND - (int64_t)time * LFXO_TICKS_PER_SECOND - (int64_t)ticks - TICKS_TO_CAPTURE;

    // Wait an extra measurement interval if there is not enough time to sleep

    if (ticksUntilFirstInterrupt < MINIMUM_TICKS_TO_SLEEP) ticksUntilFirstInterrupt += (int64_t)measurementInterval * LFXO_TICKS_PER_SECOND;

    // Increment the real-time counter compare register

    uint64_t compare = counter + ticksUntilFirstInterrupt;

    // Remember higher bits of the 24-bit real-time counter compare register to count down to the real start time

    uint64_t delayedStartCount = compare >> 24;

    // Enable interrupt using real-time counter with compare register 0

    RTC_CompareSet(RTC_COMP0, compare);

    RTC_IntEnable(RTC_IEN_COMP0);

    // Enable interrupt to regularly flash LED while waiting using real-time counter with compare register 1

    RTC_CompareSet(RTC_COMP1, counter + LED_INTERVAL_SECONDS * LFXO_TICKS_PER_SECOND);

    RTC_IntEnable(RTC_IEN_COMP1);

    NVIC_ClearPendingIRQ(RTC_IRQn);

    NVIC_EnableIRQ(RTC_IRQn);

    // Initialize real-time counter interrupt event flags

    eventRTC_Comp0 = false;

    eventRTC_Comp1 = false;

    // Disable timer

    Timer_disable();

    // Remember if recording has started already
    bool started = false;

    // Record snapshots until the end date is reached

    while (state == STATE_WILL_RECORD) {

        if (eventRTC_Comp0) {

            // Time interval since last measurement has passed

            if (delayedStartCount == 0) {

                getTime(&time, &ticks, NULL);

                if (!started) {

                    // Disable interrupts; so, LED does not flash every 5 s anymore

                    RTC_IntDisable(RTC_IEN_COMP1);

                    // Remember actual start time

                    startTime = time;

                    // Remember that recording started

                    started = true;

                }

                // Check if end time is reached

                if (time + measurementInterval >= endTime) {

                    state = STATE_WILL_SHUTDOWN;

                    // Disable interrupts so no more snapshots are acquired

                    RTC_IntDisable(RTC_IEN_COMP0);

                    NVIC_ClearPendingIRQ(RTC_IRQn);

                    NVIC_DisableIRQ(RTC_IRQn);

                } else {

                    // Update the real-time counter compare register for the next interrupt

                    uint32_t compare = RTC_CompareGet(RTC_COMP0) + measurementInterval * LFXO_TICKS_PER_SECOND;

                    RTC_CompareSet(RTC_COMP0, compare);

                }

                /* Enable DC boost */

                enableDCBoost();

                // Acquire snapshot using GNSS front-end

                Timer_enable();

                // Enable the ADC

                AnalogToDigitalConverter_enable();

                // Read ambient temperature in tenths of a degree Celsius

                int32_t temperature = AnalogToDigitalConverter_measureTemperature();

                AnalogToDigitalConverter_enableBatteryMeasurement(legacyBoard ? PULL_LOW : PULL_HIGH);

                // Read battery voltage in hundredths of a volt

                uint32_t batteryVoltage = AnalogToDigitalConverter_measureBatteryVoltage();

                AnalogToDigitalConverter_disableBatteryMeasurement();

                // Disable the ADC

                AnalogToDigitalConverter_disable();

                // Power up radio

                Radio_powerOn();

                Radio_enableHFXOInput();

                // Wait short period for radio to stabilize

                Timer_delayMilliseconds(10);

                // Check that the HFXO is running

                uint32_t frequency =  measureHFXO();

                bool frequencyGood = frequency > HFXO_FREQ - HFXO_FREQ_MARGIN && frequency < HFXO_FREQ + HFXO_FREQ_MARGIN;

                if (frequencyGood) {

                    // Switch HF clock to HFXO

                    CMU_ClockSelectSet(cmuClock_HF, cmuSelect_HFXO);

                    // Get timestamp for snapshot

                    getTime(&time, &ticks, NULL);

                    // Capture snapshot

                    Radio_captureSnapshot((uint8_t*)SNAPSHOT_BUFFER_LOCATION, SNAPSHOT_BUFFER_SIZE);

                    // Switch HF clock to HFRCO

                    CMU_ClockSelectSet(cmuClock_HF, cmuSelect_HFRCO);

                } else {

                    // Set snapshot to default value 0

                    memset((uint8_t*)SNAPSHOT_BUFFER_LOCATION, 0, SNAPSHOT_BUFFER_SIZE);

                    // Get timestamp

                    getTime(&time, &ticks, NULL);

                }

                // Power off the radio

                Radio_disableHFXOInput();

                Radio_powerOff();

                if (frequencyGood) {

                    // Flash green LED once

                    enableGreenLED(true);
                    Timer_delayMilliseconds(10);
                    enableGreenLED(false);

                } else {

                    // Flash red LED once

                    enableRedLED(true);
                    Timer_delayMilliseconds(10);
                    enableRedLED(false);

                }

                // Stream the snapshot and its metadata out over the software UART

                sendSnapshotFrame(time, ticks, temperature, batteryVoltage);

                // Disable DC boost

                if (batteryVoltage < DC_BOOST_BATTERY_THRESHOLD) Timer_delayMilliseconds(20);

                disableDCBoost();

                // Disable timer if another snapshot will be recorded

                if (state == STATE_WILL_RECORD) Timer_disable();

            } else {

                --delayedStartCount;

            }

            // Reset interrupt event flag

            eventRTC_Comp0 = false;

        } else {

            if (eventRTC_Comp1) {

                // Still waiting to record first snapshot

                // Update the real-time counter compare register for the next interrupt

                RTC_CompareSet(RTC_COMP1, RTC_CompareGet(RTC_COMP1)
                                          + LED_INTERVAL_SECONDS * LFXO_TICKS_PER_SECOND);

                // Flash LEDs

                Timer_enable();

                enableGreenLED(true);
                enableRedLED(true);
                Timer_delayMilliseconds(10);
                enableGreenLED(false);
                enableRedLED(false);

                Timer_disable();

                // Reset interrupt flag

                eventRTC_Comp1 = false;

            }

        }

        // Enter EM3 if recording more snapshots

        if (state == STATE_WILL_RECORD)  EMU_EnterEM3(false);

    }

    if (state == STATE_WILL_SHUTDOWN) {

        // Flash red LED to indicate power down

        for (uint32_t i = 0; i < 10; i += 1) {
            enableRedLED(true);
            Timer_delayMilliseconds(100);
            enableRedLED(false);
            Timer_delayMilliseconds(100);
        }

        // Disable timer

        Timer_disable();

        // Enter EM4. There is no wake pin configured, so only a physical
        // reset or power cycle will bring the device back.

        EMU_EnterEM4();

        // Enter EM3 if something goes wrong while entering EM4

        CMU_ClockEnable(cmuClock_CORELE, false);

        CMU_OscillatorEnable(cmuOsc_LFXO, false, false);

        while (true) EMU_EnterEM3(false);

    }

    NVIC_SystemReset();

}
