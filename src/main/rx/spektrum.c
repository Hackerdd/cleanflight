/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "platform.h"

#ifdef SERIAL_RX

#include "build/debug.h"

#include "drivers/io.h"
#include "drivers/io_impl.h"
#include "drivers/system.h"
#include "drivers/light_led.h"

#include "io/serial.h"

#include "fc/config.h"

#ifdef TELEMETRY
#include "telemetry/telemetry.h"
#endif

#include "rx/rx.h"
#include "rx/spektrum.h"

// driver for spektrum satellite receiver / sbus

#define SPEKTRUM_MAX_SUPPORTED_CHANNEL_COUNT 12
#define SPEKTRUM_2048_CHANNEL_COUNT 12
#define SPEKTRUM_1024_CHANNEL_COUNT 7

#define SPEK_FRAME_SIZE 16
#define SPEKTRUM_NEEDED_FRAME_INTERVAL 5000

#define SPEKTRUM_BAUDRATE 115200

#define SPEKTRUM_MAX_FADE_PER_SEC 40
#define SPEKTRUM_FADE_REPORTS_PER_SEC 2

static uint8_t spek_chan_shift;
static uint8_t spek_chan_mask;
static bool rcFrameComplete = false;
static bool spekHiRes = false;

// Variables used for calculating a signal strength from satellite fade.
//  This is time-variant and computed every second based on the fade
//  count over the last second.
static uint32_t spek_fade_last_sec = 0; // Stores the timestamp of the last second.
static uint16_t spek_fade_last_sec_count = 0; // Stores the fade count at the last second.
static uint8_t rssi_channel; // Stores the RX RSSI channel.

static volatile uint8_t spekFrame[SPEK_FRAME_SIZE];

static rxRuntimeConfig_t *rxRuntimeConfigPtr;

#ifdef SPEKTRUM_BIND
static IO_t BindPin = DEFIO_IO(NONE);
#endif
#ifdef HARDWARE_BIND_PLUG
static IO_t BindPlug = DEFIO_IO(NONE);
#endif


// Receive ISR callback
static void spektrumDataReceive(uint16_t c)
{
    uint32_t spekTime, spekTimeInterval;
    static uint32_t spekTimeLast = 0;
    static uint8_t spekFramePosition = 0;

    spekTime = micros();
    spekTimeInterval = spekTime - spekTimeLast;
    spekTimeLast = spekTime;

    if (spekTimeInterval > SPEKTRUM_NEEDED_FRAME_INTERVAL) {
        spekFramePosition = 0;
    }

    if (spekFramePosition < SPEK_FRAME_SIZE) {
        spekFrame[spekFramePosition++] = (uint8_t)c;
        if (spekFramePosition < SPEK_FRAME_SIZE) {
            rcFrameComplete = false;
        } else {
            rcFrameComplete = true;
        }
    }
}

static uint32_t spekChannelData[SPEKTRUM_MAX_SUPPORTED_CHANNEL_COUNT];

static uint8_t spektrumFrameStatus(void)
{
    if (!rcFrameComplete) {
        return RX_FRAME_PENDING;
    }

    rcFrameComplete = false;

    // Fetch the fade count
    const uint16_t fade = (spekFrame[0] << 8) + spekFrame[1];
    const uint32_t current_secs = micros() / 1000 / (1000 / SPEKTRUM_FADE_REPORTS_PER_SEC);

    if (spek_fade_last_sec == 0) {
        // This is the first frame status received.
        spek_fade_last_sec_count = fade;
        spek_fade_last_sec = current_secs;
    } else if(spek_fade_last_sec != current_secs) {
        // If the difference is > 1, then we missed several seconds worth of frames and
        // should just throw out the fade calc (as it's likely a full signal loss).
        if((current_secs - spek_fade_last_sec) == 1) {
            if(rssi_channel != 0) {
                if (spekHiRes)
                    spekChannelData[rssi_channel] = 2048 - ((fade - spek_fade_last_sec_count) * 2048 / (SPEKTRUM_MAX_FADE_PER_SEC / SPEKTRUM_FADE_REPORTS_PER_SEC));
                else
                    spekChannelData[rssi_channel] = 1024 - ((fade - spek_fade_last_sec_count) * 1024 / (SPEKTRUM_MAX_FADE_PER_SEC / SPEKTRUM_FADE_REPORTS_PER_SEC));
            }
        }
        spek_fade_last_sec_count = fade;
        spek_fade_last_sec = current_secs;
    }

    for (int b = 3; b < SPEK_FRAME_SIZE; b += 2) {
        const uint8_t spekChannel = 0x0F & (spekFrame[b - 1] >> spek_chan_shift);
        if (spekChannel < rxRuntimeConfigPtr->channelCount && spekChannel < SPEKTRUM_MAX_SUPPORTED_CHANNEL_COUNT) {
            if(rssi_channel == 0 || spekChannel != rssi_channel) {
                spekChannelData[spekChannel] = ((uint32_t)(spekFrame[b - 1] & spek_chan_mask) << 8) + spekFrame[b];
            }
        }
    }

    return RX_FRAME_COMPLETE;
}

static uint16_t spektrumReadRawRC(const rxRuntimeConfig_t *rxRuntimeConfig, uint8_t chan)
{
    uint16_t data;

    if (chan >= rxRuntimeConfig->channelCount) {
        return 0;
    }

    if (spekHiRes)
        data = 988 + (spekChannelData[chan] >> 1);   // 2048 mode
    else
        data = 988 + spekChannelData[chan];          // 1024 mode

    return data;
}

#ifdef SPEKTRUM_BIND

bool spekShouldBind(uint8_t spektrum_sat_bind)
{
#ifdef HARDWARE_BIND_PLUG
    BindPlug = IOGetByTag(IO_TAG(BINDPLUG_PIN));
    IOInit(BindPlug, OWNER_RX_BIND, 0);
    IOConfigGPIO(BindPlug, IOCFG_IPU);

    // Check status of bind plug and exit if not active
    delayMicroseconds(10);  // allow configuration to settle
    if (IORead(BindPlug)) {
        return false;
    }
#endif

    return !(
        isMPUSoftReset() ||
        spektrum_sat_bind == SPEKTRUM_SAT_BIND_DISABLED ||
        spektrum_sat_bind > SPEKTRUM_SAT_BIND_MAX
    );
}
/* spektrumBind function ported from Baseflight. It's used to bind satellite receiver to TX.
 * Function must be called immediately after startup so that we don't miss satellite bind window.
 * Known parameters. Tested with DSMX satellite and DX8 radio. Framerate (11ms or 22ms) must be selected from TX.
 * 9 = DSMX 11ms / DSMX 22ms
 * 5 = DSM2 11ms 2048 / DSM2 22ms 1024
 */
void spektrumBind(rxConfig_t *rxConfig)
{
    int i;
    if (!spekShouldBind(rxConfig->spektrum_sat_bind)) {
        return;
    }

    LED1_ON;

    //find bind pin automatically
    const serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_RX_SERIAL);
    if (!portConfig) {
        return;
    }

    switch(portConfig->identifier){
#ifdef USE_UART1
		case SERIAL_PORT_USART1:
			BindPin = IOGetByTag(IO_TAG(UART1_RX_PIN));
        break;
#endif
#ifdef USE_UART2
		case SERIAL_PORT_USART2:
        	BindPin = IOGetByTag(IO_TAG(UART2_RX_PIN));
        break;
#endif
#ifdef USE_UART3
		case SERIAL_PORT_USART3:
        	BindPin = IOGetByTag(IO_TAG(UART3_RX_PIN));
        break;
#endif
#ifdef USE_UART4
		case SERIAL_PORT_USART4:
        	BindPin = IOGetByTag(IO_TAG(UART4_RX_PIN));
        break;
#endif
#ifdef USE_UART5
		case SERIAL_PORT_USART5:
        	BindPin = IOGetByTag(IO_TAG(UART5_RX_PIN));
        break;
#endif
#ifdef USE_UART6
		case SERIAL_PORT_USART6:
        	BindPin = IOGetByTag(IO_TAG(UART6_RX_PIN));
        break;
#endif
#ifdef USE_UART7
		case SERIAL_PORT_USART7:
        	BindPin = IOGetByTag(IO_TAG(UART7_RX_PIN));
        break;
#endif
#ifdef USE_UART8
		case SERIAL_PORT_USART8:
        	BindPin = IOGetByTag(IO_TAG(UART8_RX_PIN));
        break;
#endif
      default:
        return;
    }

    IOInit(BindPin, OWNER_RX_BIND, 0);
    IOConfigGPIO(BindPin, IOCFG_OUT_PP);

    // RX line, set high
    IOWrite(BindPin, true);

    // Bind window is around 20-140ms after powerup
    delay(60);
    LED1_OFF;

    for (i = 0; i < rxConfig->spektrum_sat_bind; i++) {

        LED0_OFF;
        LED2_OFF;
        // RX line, drive low for 120us
        IOWrite(BindPin, false);
        delayMicroseconds(120);

        LED0_ON;
        LED2_ON;
        // RX line, drive high for 120us
        IOWrite(BindPin, true);
        delayMicroseconds(120);

    }

#ifndef HARDWARE_BIND_PLUG
    // If we came here as a result of hard  reset (power up, with spektrum_sat_bind set), then reset it back to zero and write config
    // Don't reset if hardware bind plug is present
    // Reset only when autoreset is enabled
    if (rxConfig->spektrum_sat_bind_autoreset == 1 && !isMPUSoftReset()) {
        rxConfig->spektrum_sat_bind = 0;
        saveConfigAndNotify();
    }
#endif

}
#endif // SPEKTRUM_BIND

bool spektrumInit(const rxConfig_t *rxConfig, rxRuntimeConfig_t *rxRuntimeConfig)
{
    rxRuntimeConfigPtr = rxRuntimeConfig;

    switch (rxConfig->serialrx_provider) {
    case SERIALRX_SPEKTRUM2048:
        // 11 bit frames
        spek_chan_shift = 3;
        spek_chan_mask = 0x07;
        spekHiRes = true;
        rxRuntimeConfig->channelCount = SPEKTRUM_2048_CHANNEL_COUNT;
        rxRuntimeConfig->rxRefreshRate = 11000;
        break;
    case SERIALRX_SPEKTRUM1024:
        // 10 bit frames
        spek_chan_shift = 2;
        spek_chan_mask = 0x03;
        spekHiRes = false;
        rxRuntimeConfig->channelCount = SPEKTRUM_1024_CHANNEL_COUNT;
        rxRuntimeConfig->rxRefreshRate = 22000;
        break;
    }

    rxRuntimeConfig->rcReadRawFn = spektrumReadRawRC;
    rxRuntimeConfig->rcFrameStatusFn = spektrumFrameStatus;

    const serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_RX_SERIAL);
    if (!portConfig) {
        return false;
    }

#ifdef TELEMETRY
    bool portShared = telemetryCheckRxPortShared(portConfig);
#else
    bool portShared = false;
#endif

    serialPort_t *spektrumPort = openSerialPort(portConfig->identifier, FUNCTION_RX_SERIAL, spektrumDataReceive, SPEKTRUM_BAUDRATE, portShared ? MODE_RXTX : MODE_RX, SERIAL_NOT_INVERTED);

#ifdef TELEMETRY
    if (portShared) {
        telemetrySharedPort = spektrumPort;
    }
#endif

    rssi_channel = rxConfig->rssi_channel - 1; // -1 because rxConfig->rssi_channel is 1-based and rssi_channel is 0-based.
    if (rssi_channel >= rxRuntimeConfig->channelCount) {
        rssi_channel = 0;
    }

    return spektrumPort != NULL;
}
#endif // SERIAL_RX
