#include <time.h>
#include <string.h>

#include "em_cmu.h"
#include "em_emu.h"
#include "em_rmu.h"
#include "em_rtc.h"
#include "em_gpio.h"
#include "em_chip.h"
#include "em_device.h"
#include "em_leuart.h"
#include "em_system.h"

#include "radio.h"
#include "timer.h"
#include "pinouts.h"
#include "analogToDigitalConverter.h"

/*
 * NOT for real deployment - power efficiency is deliberately out of scope
 * for now. Set to 1 to keep the HF oscillator running (EM1 instead of EM3)
 * between measurement intervals, so LEUART0 never loses its clock.
 *
 * Originally added to test an EM3-wake-race theory for why CMD_START never
 * got ACKed - that theory turned out to be wrong (root cause was a missing
 * GPIO_PinModeSet() for LEUART0's TX pin in Leuart_init(), unrelated to
 * sleep state), but EM1 has been left on since real EM3 sleep hasn't been
 * re-validated against the fixed firmware yet. Set to 0 once that's done -
 * EM1 draws meaningfully more power than EM3 for no benefit in real use.
 */
#define DEBUG_STAY_IN_EM1_NOT_EM3 1

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

/* Software UART protocol constants */

#define UART_SYNC_WORD_LENGTH                   4

#define UART_CRC_POLY                           0x1021

typedef enum {
    UART_FRAME_INFO = 0x01,
    UART_FRAME_SNAPSHOT = 0x02,
    UART_FRAME_CMD_START = 0x10,                  // nRF9151 -> SnapperGPS: start auto-capturing
    UART_FRAME_CMD_STOP = 0x11,                    // nRF9151 -> SnapperGPS: stop auto-capturing
    UART_FRAME_CMD_SET_TIME = 0x12,                // nRF9151 -> SnapperGPS: set RTC clock.
                                                    // 4-byte little-endian Unix timestamp
                                                    // payload (seconds only)
    UART_FRAME_ACK = 0x20                          // SnapperGPS -> nRF9151: 1-byte payload = resulting capturingEnabled
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

/* Gates whether the RTC-driven measurement-interval handler actually
 * captures and sends a snapshot, or just no-ops and reschedules. Does not
 * touch the RTC cadence itself - only what the nRF9151 command channel
 * (LEUART0_IRQHandler, below) is allowed to turn on/off. Starts false: the
 * device boots idle and waits for an explicit CMD_START. */

static volatile bool capturingEnabled = false;

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

/* GPIO_ODD_IRQHandler's only job is to end EM2/EM3 on an edge from the
 * nRF9151 starting a command frame on PC15/USB_DP - LEUART0 is clocked
 * from cmuSelect_HFCLKLE (see Leuart_init()), which is inert while the HF
 * oscillator is stopped in EM2/EM3, so a plain, clock-independent GPIO
 * edge is what actually wakes the core. Once awake, main()'s loop resumes
 * and LEUART0 (already configured) takes over receiving the byte stream
 * for real - this handler does nothing else. */

void GPIO_ODD_IRQHandler(void) {

    uint32_t interruptMask = GPIO_IntGet();

    GPIO_IntClear(interruptMask);

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

/* LEUART0 functions - PC14 (TX, sync+info+snapshot+ACK) / PC15 (RX,
 * commands from the nRF9151), both at 115200 baud, replacing the old
 * bit-banged softwareUart.c/GPIO1 link this firmware version used to use. */

static void Leuart_transmit(const uint8_t *data, size_t len) {

    for (size_t i = 0; i < len; ++i) {
        LEUART_Tx(LEUART0, data[i]);   // blocks until TX buffer has room; hardware paces the bits
    }

}

static void sendAckFrame(bool enabled) {

    uint8_t frame[UART_SYNC_WORD_LENGTH + 1 /* type */ + 1 /* payload */ + 2 /* crc */];

    memcpy(frame, uartSyncWord, UART_SYNC_WORD_LENGTH);
    frame[UART_SYNC_WORD_LENGTH] = UART_FRAME_ACK;
    frame[UART_SYNC_WORD_LENGTH + 1] = enabled ? 1 : 0;

    uint16_t crc = crcFinalize(crcUpdateBytes(0, &frame[UART_SYNC_WORD_LENGTH], 2));

    frame[UART_SYNC_WORD_LENGTH + 2] = (uint8_t)crc;
    frame[UART_SYNC_WORD_LENGTH + 3] = (uint8_t)(crc >> 8);

    Leuart_transmit(frame, sizeof(frame));

}

/* Incoming command parser state, fed one byte at a time from
 * LEUART0_IRQHandler. Unlike the old bit-banged design, each byte arrives
 * via its own hardware interrupt rather than a busy-wait loop, so there's
 * no risk of this handler hanging - it always processes exactly one byte
 * and returns. */

typedef enum {
    CMD_STATE_SYNC,
    CMD_STATE_TYPE,
    CMD_STATE_PAYLOAD,
    CMD_STATE_CRC
} cmdParserState_t;

static cmdParserState_t cmdState = CMD_STATE_SYNC;

static uint8_t cmdSyncWindow[UART_SYNC_WORD_LENGTH];
static uint8_t cmdSyncWindowLen;

static uint8_t cmdType;

// Only CMD_SET_TIME carries a payload today (a 4-byte little-endian Unix
// timestamp) - sized for that, not a general per-type length table.
static uint8_t cmdPayloadBytes[4];
static uint8_t cmdPayloadLen;
static uint8_t cmdPayloadIndex;

static uint8_t cmdCrcBytes[2];
static uint8_t cmdCrcIndex;

static void handleCommandByte(uint8_t byte) {

    switch (cmdState) {

    case CMD_STATE_SYNC:

        if (cmdSyncWindowLen < UART_SYNC_WORD_LENGTH) {
            cmdSyncWindow[cmdSyncWindowLen++] = byte;
        } else {
            memmove(cmdSyncWindow, cmdSyncWindow + 1, UART_SYNC_WORD_LENGTH - 1);
            cmdSyncWindow[UART_SYNC_WORD_LENGTH - 1] = byte;
        }

        if (cmdSyncWindowLen == UART_SYNC_WORD_LENGTH &&
            memcmp(cmdSyncWindow, uartSyncWord, UART_SYNC_WORD_LENGTH) == 0) {
            cmdState = CMD_STATE_TYPE;
        }

        break;

    case CMD_STATE_TYPE:

        cmdType = byte;
        cmdPayloadIndex = 0;
        cmdCrcIndex = 0;

        if (cmdType == UART_FRAME_CMD_SET_TIME) {
            cmdPayloadLen = sizeof(cmdPayloadBytes);
            cmdState = CMD_STATE_PAYLOAD;
        } else {
            cmdPayloadLen = 0;
            cmdState = CMD_STATE_CRC;
        }

        break;

    case CMD_STATE_PAYLOAD:

        cmdPayloadBytes[cmdPayloadIndex++] = byte;

        if (cmdPayloadIndex == cmdPayloadLen) {
            cmdState = CMD_STATE_CRC;
        }

        break;

    case CMD_STATE_CRC:

        cmdCrcBytes[cmdCrcIndex++] = byte;

        if (cmdCrcIndex == 2) {

            uint16_t crcCalc = crcUpdateBytes(0, &cmdType, 1);

            if (cmdPayloadLen > 0) {
                crcCalc = crcUpdateBytes(crcCalc, cmdPayloadBytes, cmdPayloadLen);
            }

            crcCalc = crcFinalize(crcCalc);

            uint16_t crcRecv = (uint16_t)cmdCrcBytes[0] | ((uint16_t)cmdCrcBytes[1] << 8);

            if (crcCalc == crcRecv) {

                if (cmdType == UART_FRAME_CMD_START || cmdType == UART_FRAME_CMD_STOP) {

                    capturingEnabled = (cmdType == UART_FRAME_CMD_START);

                    // Steady-state LED: green while running, red while
                    // stopped - not a flash, a persistent indicator.
                    enableGreenLED(capturingEnabled);
                    enableRedLED(!capturingEnabled);

                    sendAckFrame(capturingEnabled);

                } else if (cmdType == UART_FRAME_CMD_SET_TIME) {

                    uint32_t newTime = (uint32_t)cmdPayloadBytes[0]
                                      | ((uint32_t)cmdPayloadBytes[1] << 8)
                                      | ((uint32_t)cmdPayloadBytes[2] << 16)
                                      | ((uint32_t)cmdPayloadBytes[3] << 24);

                    setTime(newTime, 0);
                    sendAckFrame(capturingEnabled); // unchanged by CMD_SET_TIME

                }

            }
            // CRC mismatch or unknown type: silently dropped, no ACK - the
            // nRF9151 retries on a missing ACK.

            cmdState = CMD_STATE_SYNC;
            cmdSyncWindowLen = 0;

        }

        break;

    }

}

void LEUART0_IRQHandler(void) {

    uint32_t flags = LEUART_IntGet(LEUART0);

    LEUART_IntClear(LEUART0, flags);

    if (!(flags & LEUART_IF_RXDATAV)) return;

    handleCommandByte(LEUART_RxDataGet(LEUART0));

}

static void Leuart_init(void) {

    // The LEUART0->ROUTE assignment below only tells the peripheral which
    // pins to use - it does NOT configure the GPIO pad itself. Without this,
    // PC14 stays in its post-reset disabled/tristate state and LEUART_Tx()
    // silently does nothing externally visible, even though the peripheral
    // and its internal shift register work fine. Initial output value 1
    // matches UART's idle-high convention.
    //
    // PC15 (RX) doesn't need an equivalent call here: main() already puts it
    // in gpioModeInputPullFilter for the GPIO-wake interrupt, and that mode
    // happens to also be exactly what LEUART0's RX function needs.

    GPIO_PinModeSet(USB_DM_PORT, USB_DM_PIN, gpioModePushPull, 1);

    CMU_ClockEnable(cmuClock_LEUART0, true);

    // HFCLKLE-sourced (a divided tap off the main HF clock) -> supports
    // 115200 baud, but only while the HF oscillator is running (EM0/EM1).
    // LEUART0 is inert in EM2/EM3; wake is handled separately by the plain
    // GPIO edge interrupt on PC15 (GPIO_ODD_IRQHandler, above), not by this
    // peripheral. cmuClock_LFB is the RTC's cmuClock_LFA's independent
    // sibling branch, so this doesn't disturb RTC timing.

    CMU_ClockSelectSet(cmuClock_LFB, cmuSelect_HFCLKLE);

    LEUART_Init_TypeDef init = LEUART_INIT_DEFAULT;
    init.baudrate = 115200;

    LEUART_Init(LEUART0, &init);

    LEUART0->ROUTE = LEUART_ROUTE_TXPEN | LEUART_ROUTE_RXPEN | LEUART_ROUTE_LOCATION_LOC5; // PC14 TX / PC15 RX

    LEUART_IntEnable(LEUART0, LEUART_IEN_RXDATAV);

    NVIC_ClearPendingIRQ(LEUART0_IRQn);
    NVIC_EnableIRQ(LEUART0_IRQn);

}

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

    Leuart_transmit((uint8_t*)&frame, sizeof(frame));

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

    Leuart_transmit((uint8_t*)&header, sizeof(header));

    Leuart_transmit((uint8_t*)SNAPSHOT_BUFFER_LOCATION, SNAPSHOT_BUFFER_SIZE);

    Leuart_transmit(crcBytes, sizeof(crcBytes));

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

    // Clock starts at epoch 0 here; it's seeded for real once the nRF9151 sends
    // a CMD_SET_TIME command (see handleCommandByte()) - harmless in the
    // meantime, since capturingEnabled stays false (no snapshots captured)
    // until CMD_START, and CMD_SET_TIME follows immediately after that.

    // Set up LEUART0 (PC14 TX / PC15 RX) for the nRF9151 link

    Leuart_init();

    // Wake from EM2/EM3 on an edge from the nRF9151 starting a command
    // frame on PC15/USB_DP - mirrors the USB_SENSE GPIO-wake pattern from
    // firmware_versions/snapper/src/main.c, the only existing precedent in
    // this codebase for waking on a GPIO edge rather than the RTC.

    GPIO_PinModeSet(USB_DP_PORT, USB_DP_PIN, gpioModeInputPullFilter, 1);

    GPIO_IntConfig(USB_DP_PORT, USB_DP_PIN, true, true, true);

    NVIC_ClearPendingIRQ(GPIO_ODD_IRQn);
    NVIC_EnableIRQ(GPIO_ODD_IRQn);

    // Announce the device and its recording configuration once

    sendInfoFrame();

    // Flash green LED to indicate initialized recording

    for (uint32_t i = 0; i < 10; i += 1) {
        enableGreenLED(true);
        Timer_delayMilliseconds(100);
        enableGreenLED(false);
        Timer_delayMilliseconds(100);
    }

    // Steady-state LED baseline at boot: red (stopped), matching
    // capturingEnabled's false default - see handleCommandByte(), which
    // swaps this to green on CMD_START and back to red on CMD_STOP.

    enableRedLED(true);

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

                if (capturingEnabled) {

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

                if (!frequencyGood) {

                    // Flash red LED once - a diagnostic blip on top of the
                    // steady green "running" state (see handleCommandByte()),
                    // returning to red-off afterwards since that's the
                    // correct baseline while capturingEnabled is true. No
                    // equivalent flash on success: the steady green already
                    // conveys "running and fine," and briefly toggling it
                    // off here would kill that steady indicator instead of
                    // just blipping on top of it.

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

                } // if (capturingEnabled)

            } else {

                --delayedStartCount;

            }

            // Reset interrupt event flag

            eventRTC_Comp0 = false;

        } else {

            if (eventRTC_Comp1) {

                // Still waiting to record first snapshot. No LED flash here
                // any more - it used to briefly flash both LEDs, but that
                // unconditionally turned them both off afterwards, which
                // would fight the steady green/red run-state indicator (see
                // handleCommandByte()) every LED_INTERVAL_SECONDS until the
                // first RTC_COMP0 tick disables this handler for good.

                // Update the real-time counter compare register for the next interrupt

                RTC_CompareSet(RTC_COMP1, RTC_CompareGet(RTC_COMP1)
                                          + LED_INTERVAL_SECONDS * LFXO_TICKS_PER_SECOND);

                // Reset interrupt flag

                eventRTC_Comp1 = false;

            }

        }

        // Enter EM3 if recording more snapshots

        if (state == STATE_WILL_RECORD) {
#if DEBUG_STAY_IN_EM1_NOT_EM3
            EMU_EnterEM1();
#else
            EMU_EnterEM3(false);
#endif
        }

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
