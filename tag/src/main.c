/**
 * Copyright (c) 2019 - Frederic Mes, RTLOC
 * Copyright (c) 2015 - Decawave Ltd, Dublin, Ireland.
 *
 * This file is part of Zephyr-DWM1001.
 *
 *   Zephyr-DWM1001 is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *
 *   Zephyr-DWM1001 is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with Zephyr-DWM1001.  If not, see <https://www.gnu.org/licenses/>.
 *
 */

/*! ----------------------------------------------------------------------------
 *  @file    ex_05a_main.c
 *  @brief   Double-sided two-way ranging (DS TWR) initiator example code
 *
 *           This is a simple code example which acts as the initiator in a DS TWR distance measurement exchange. This application sends a "poll"
 *           frame (recording the TX time-stamp of the poll), and then waits for a "response" message expected from the "DS TWR responder" example
 *           code (companion to this application). When the response is received its RX time-stamp is recorded and we send a "final" message to
 *           complete the exchange. The final message contains all the time-stamps recorded by this application, including the calculated/predicted TX
 *           time-stamp for the final message itself. The companion "DS TWR responder" example application works out the time-of-flight over-the-air
 *           and, thus, the estimated distance between the two devices.
 *
 *
 * All rights reserved.
 *
 * @author Decawave, RTLOC
 */

#include <drivers/dw1000/deca_device_api.h>
#include <drivers/dw1000/deca_regs.h>
#include <drivers/dw1000/deca_spi.h>
#include <drivers/dw1000/port.h>

#include <zephyr/drivers/hwinfo.h>
#include <zephyr/timing/timing.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);

/* Example application name and version to display on console. */
#define APP_HEADER "DWM1001 & Zephyr"
#define APP_NAME "Example 5a - DS TWR INIT"
#define APP_VERSION "Version - 1.4.0"
#define APP_VERSION_NUM 0x010400
#define APP_LINE "================="

/* Inter-ranging delay period, in milliseconds. */
#define LOOP_DELAY_MS 500

/* Default communication configuration. */
static dwt_config_t config = {
    5,               /* Channel number. */
    DWT_PRF_64M,     /* Pulse repetition frequency. */
    DWT_PLEN_128,    /* Preamble length. Used in TX only. */
    DWT_PAC8,        /* Preamble acquisition chunk size. Used in RX only. */
    9,               /* TX preamble code. Used in TX only. */
    9,               /* RX preamble code. Used in RX only. */
    1,               /* 0 to use standard SFD, 1 to use non-standard SFD. */
    DWT_BR_6M8,      /* Data rate. */
    DWT_PHRMODE_EXT, /* PHY header mode. */
    (129)            /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
};

/* Default antenna delay values for 64 MHz PRF. See NOTE 1 below. */
#define TX_ANT_DLY 16436
#define RX_ANT_DLY 16436

/* Frames used in the ranging process. See NOTE 2 below. */
static uint8 ranging_request_msg[] = {
    0, 0, 0, 0, 0, 0, 0, 0, /* Tag device id */
    0, 0, 0, 0, 0, 0, 0, 0, /* Anchor device id */
    0x52, 0x00, /* Type */
    0, 0 /* Checksum*/
};

static uint8 ranging_request_response_msg[] = {
    0, 0, 0, 0, 0, 0, 0, 0, /* Tag device id */
    0, 0, 0, 0, 0, 0, 0, 0, /* Anchor device id */
    0x52, 0x01, /* Type */
    0, 0 /* Checksum*/  
};

static uint8 ranging_final_msg[] = {
    0, 0, 0, 0, 0, 0, 0, 0, /* Tag device id */
    0, 0, 0, 0, 0, 0, 0, 0, /* Anchor device id */
    0x52, 0x02, /* Type */
    0, 0, 0, 0, /* Ranging request transmission timestamp */
    0, 0, 0, 0, /* Ranging request response reception timestamp */
    0, 0, 0, 0, /* Final ranging transmission timestamp */
    0, 0 /* Checksum*/  
};

static uint8 distance_relay_msg[] = {
    0, 0, 0, 0, 0, 0, 0, 0, /* Tag device id */
    0, 0, 0, 0, 0, 0, 0, 0, /* Anchor device id */
    0x52, 0x03, /* Type */
    0, 0, 0, 0, /* Distance (mm)*/
    0, 0 /* Checksum*/
};

#define DEVICE_UUID_LEN 8

#define ALL_MSG_COMMON_LEN 16
#define FINAL_MSG_POLL_TX_TS_IDX 18
#define FINAL_MSG_RESP_RX_TS_IDX 22
#define FINAL_MSG_FINAL_TX_TS_IDX 26
#define FINAL_MSG_TS_LEN 4
#define RELAY_MSG_DISTANCE_IDX 18
#define TAG_UUID_IDX 0
#define ANCHOR_UUID_IDX 8


/* Buffer to store received response message.
 * Its size is adjusted to longest frame that this example code is supposed to handle. */
#define RX_BUF_LEN 32
static uint8 rx_buffer[RX_BUF_LEN];

/* Hold copy of status register state here for reference so that it can be examined at a debug breakpoint. */
static uint32 status_reg = 0;

/* UWB microsecond (uus) to device time unit (dtu, around 15.65 ps) conversion factor.
 * 1 uus = 512 / 499.2 �s and 1 �s = 499.2 * 128 dtu. */
#define UUS_TO_DWT_TIME 65536

/* Delay between frames, in UWB microseconds. See NOTE 4 below. */
/* This is the delay from the end of the frame transmission to the enable of the receiver, as programmed for the DW1000's wait for response feature. */
#define POLL_TX_TO_RESP_RX_DLY_UUS 300
/* This is the delay from Frame RX timestamp to TX reply timestamp used for calculating/setting the DW1000's delayed TX function. This includes the
 * frame length of approximately 2.66 ms with above configuration. */
#define RESP_RX_TO_FINAL_TX_DLY_UUS 4000
/* Receive response timeout. See NOTE 5 below. */
#define RESP_RX_TIMEOUT_UUS 6000
/* Preamble timeout, in multiple of PAC size. See NOTE 6 below. */
#define PRE_TIMEOUT 40

#define DUMMY_BUFFER_LEN 1024
static uint8 dummy_buffer[DUMMY_BUFFER_LEN];

/* Time-stamps of frames transmission/reception, expressed in device time units.
 * As they are 40-bit wide, we need to define a 64-bit int type to handle them. */
typedef unsigned long long uint64;
static uint64 poll_tx_ts;
static uint64 resp_rx_ts;
static uint64 final_tx_ts;

/* Declaration of static functions. */
static uint64 get_tx_timestamp_u64(void);
static uint64 get_rx_timestamp_u64(void);
static void final_msg_set_ts(uint8 *ts_field, uint64 ts);

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn main()
 *
 * @brief Application entry point.
 *
 * @param  none
 *
 * @return none
 */
int main(void)
{
    /* Display application name on console. */
    LOG_INF(APP_HEADER);
    LOG_INF(APP_NAME);
    LOG_INF(APP_VERSION);
    LOG_INF(APP_LINE);

    /* Configure DW1000 SPI */
    openspi();

    /* Reset and initialise DW1000.
     * For initialisation, DW1000 clocks must be temporarily set to crystal speed. After initialisation SPI rate can be increased for optimum
     * performance. */
    port_set_dw1000_slowrate();
    if (dwt_initialise(DWT_LOADUCODE) == DWT_ERROR)
    {
        LOG_ERR("err - init failed");
        while (1)
        {
        };
    }
    port_set_dw1000_fastrate();

    /* Configure DW1000. See NOTE 7 below. */
    dwt_configure(&config);

    /* Apply default antenna delay value. See NOTE 1 below. */
    dwt_setrxantennadelay(RX_ANT_DLY);
    dwt_settxantennadelay(TX_ANT_DLY);

    /* Set expected response's delay and timeout. See NOTE 4, 5 and 6 below.
     * As this example only handles one incoming frame with always the same delay and timeout, those values can be set here once for all. */
    dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
    dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);

    // NOTE: no use of preamble tmo's yet
    //  dwt_setpreambledetecttimeout(PRE_TIMEOUT);

    /* Configure DW1000 LEDs */
    dwt_setleds(1);

    /* Add anchor UUID to ranging messages */
    uint8_t hwid_buffer[8];
    hwinfo_get_device_id(hwid_buffer, 8);

    memcpy(&ranging_request_msg[TAG_UUID_IDX], hwid_buffer, sizeof(hwid_buffer));
    memcpy(&ranging_request_response_msg[TAG_UUID_IDX], hwid_buffer, sizeof(hwid_buffer));
    memcpy(&ranging_final_msg[TAG_UUID_IDX], hwid_buffer, sizeof(hwid_buffer));
    memcpy(&distance_relay_msg[TAG_UUID_IDX], hwid_buffer, sizeof(hwid_buffer));

    LOG_HEXDUMP_INF(hwid_buffer, sizeof(hwid_buffer), "Device ID");
    
    // uint8 test_anchor_uuid[] = {0xb9, 0x1c, 0x2e, 0x64, 0xe3, 0x31, 0x82, 0x70};
    // uint8 test_anchor_uuid[] = {0x6e, 0x18, 0x72, 0x70, 0x49, 0x09, 0xa4, 0x69};

    uint8 anchor_uuids[7][8] = {
        {0xb9, 0x1c, 0x2e, 0x64, 0xe3, 0x31, 0x82, 0x70}, 
        {0x6e, 0x18, 0x72, 0x70, 0x49, 0x09, 0xa4, 0x69},
        {0x39, 0x39, 0xcf, 0x15, 0x64, 0x6b, 0xec, 0xd5},
        {0xfe, 0xfd, 0x56, 0x80, 0x3a, 0x55, 0x67, 0xa9},
        {0x3e, 0x04, 0xeb, 0x49, 0xd1, 0x4b, 0x3f, 0x7a},
        {0x73, 0x0f, 0x83, 0x7b, 0xa0, 0x79, 0x88, 0x6a},
        {0x3a, 0x2e, 0x08, 0x8a, 0x50, 0xb2, 0xfc, 0x3f},


    };

    int curr_anchor_uuid = 0;

    // memcpy(&ranging_request_msg[ANCHOR_UUID_IDX], test_anchor_uuid, sizeof(test_anchor_uuid));
    // memcpy(&ranging_request_response_msg[ANCHOR_UUID_IDX], test_anchor_uuid, sizeof(test_anchor_uuid));
    // memcpy(&ranging_final_msg[ANCHOR_UUID_IDX], test_anchor_uuid, sizeof(test_anchor_uuid));
    // memcpy(&distance_relay_msg[ANCHOR_UUID_IDX], test_anchor_uuid, sizeof(test_anchor_uuid));



    /* Loop forever initiating ranging exchanges. */
    while (1)
    {
        ++curr_anchor_uuid;
        if (curr_anchor_uuid > 6)
            curr_anchor_uuid = 0;


        memcpy(&ranging_request_msg[ANCHOR_UUID_IDX], anchor_uuids[curr_anchor_uuid], sizeof(anchor_uuids[curr_anchor_uuid]));
        memcpy(&ranging_request_response_msg[ANCHOR_UUID_IDX], anchor_uuids[curr_anchor_uuid], sizeof(anchor_uuids[curr_anchor_uuid]));
        memcpy(&ranging_final_msg[ANCHOR_UUID_IDX], anchor_uuids[curr_anchor_uuid], sizeof(anchor_uuids[curr_anchor_uuid]));
        memcpy(&distance_relay_msg[ANCHOR_UUID_IDX], anchor_uuids[curr_anchor_uuid], sizeof(anchor_uuids[curr_anchor_uuid]));



        /* Ranging routine timing */
        timing_t start_time, end_time;
        uint64_t total_cycles;
        uint64_t total_ns;

        timing_init();
        timing_start();

        start_time = timing_counter_get();

        /* Write frame data to DW1000 and prepare transmission. See NOTE 8 below. */
        dwt_writetxdata(sizeof(ranging_request_msg), ranging_request_msg, 0); /* Zero offset in TX buffer. */
        dwt_writetxfctrl(sizeof(ranging_request_msg), 0, 1);          /* Zero offset in TX buffer, ranging. */

        /* Start transmission, indicating that a response is expected so that reception is enabled automatically after the frame is sent and the delay
         * set by dwt_setrxaftertxdelay() has elapsed. */
        dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

        /* We assume that the transmission is achieved correctly, poll for reception of a frame or error/timeout. See NOTE 9 below. */
        while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)))
        {
        };

        if (status_reg & SYS_STATUS_RXFCG)
        {
            uint32 frame_len;

            /* Clear good RX frame event and TX frame sent in the DW1000 status register. */
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG | SYS_STATUS_TXFRS);

            /* A frame has been received, read it into the local buffer. */
            frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
            if (frame_len <= RX_BUF_LEN)
            {
                dwt_readrxdata(rx_buffer, frame_len, 0);
            }

            /* Check that the frame is the expected response from the companion "DS TWR responder" example.
             * As the sequence number field of the frame is not relevant, it is cleared to simplify the validation of the frame. */
            if (memcmp(rx_buffer, ranging_request_response_msg, ALL_MSG_COMMON_LEN) == 0)
            {
                uint32 final_tx_time;
                int ret;

                /* Retrieve poll transmission and response reception timestamp. */
                poll_tx_ts = get_tx_timestamp_u64();
                resp_rx_ts = get_rx_timestamp_u64();

                /* Compute final message transmission time. See NOTE 10 below. */
                final_tx_time = (resp_rx_ts + (RESP_RX_TO_FINAL_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
                dwt_setdelayedtrxtime(final_tx_time);

                /* Final TX timestamp is the transmission time we programmed plus the TX antenna delay. */
                final_tx_ts = (((uint64)(final_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;

                /* Write all timestamps in the final message. See NOTE 11 below. */
                final_msg_set_ts(&ranging_final_msg[FINAL_MSG_POLL_TX_TS_IDX], poll_tx_ts);
                final_msg_set_ts(&ranging_final_msg[FINAL_MSG_RESP_RX_TS_IDX], resp_rx_ts);
                final_msg_set_ts(&ranging_final_msg[FINAL_MSG_FINAL_TX_TS_IDX], final_tx_ts);

                /* Write and send final message. See NOTE 8 below. */
                dwt_writetxdata(sizeof(ranging_final_msg), ranging_final_msg, 0); /* Zero offset in TX buffer. */
                dwt_writetxfctrl(sizeof(ranging_final_msg), 0, 0);           /* Zero offset in TX buffer, non-ranging. */
                ret = dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED);

                if (ret == DWT_SUCCESS)
                {
                    /* We assume that the transmission is achieved correctly, poll for reception of a frame or error/timeout. See NOTE 9 below. */
                    while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)))
                    {
                    };

                    if (status_reg & SYS_STATUS_RXFCG)
                    {
                        uint32 frame_len;

                        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG | SYS_STATUS_TXFRS);

                        frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
                        if (frame_len <= RX_BUF_LEN)
                        {
                            dwt_readrxdata(rx_buffer, frame_len, 0);
                        }

                        uint32_t distance_mm = (uint32_t)rx_buffer[RELAY_MSG_DISTANCE_IDX + 3] << 24 |
                                               (uint32_t)rx_buffer[RELAY_MSG_DISTANCE_IDX + 2] << 16 |
                                               (uint32_t)rx_buffer[RELAY_MSG_DISTANCE_IDX + 1] << 8 |
                                               (uint32_t)rx_buffer[RELAY_MSG_DISTANCE_IDX];

                        double distance = distance_mm / 1000000.0;

                        LOG_INF("Distance (m) from anchor %d: %f", curr_anchor_uuid, distance);
                    }

                    /* Clear TXFRS event. */
                    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);
                }

                else
                {
                    LOG_ERR("err - tx failed");
                }
            }
        }
        else
        {
            LOG_ERR("err - tmo1 (rx failed)");
            /* Clear RX error/timeout events in the DW1000 status register. */
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);

            /* Reset RX to properly reinitialise LDE operation. */
            dwt_rxreset();
        }

        end_time = timing_counter_get();

        total_cycles = timing_cycles_get(&start_time, &end_time);
        total_ns = timing_cycles_to_ns(total_cycles);

        timing_stop();

        // LOG_INF("Ranging time (ms): %llu", total_ns / 1000000UL);

        /* Execute a delay between ranging exchanges. */
        k_msleep(LOOP_DELAY_MS);
    }
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn get_tx_timestamp_u64()
 *
 * @brief Get the TX time-stamp in a 64-bit variable.
 *        /!\ This function assumes that length of time-stamps is 40 bits, for both TX and RX!
 *
 * @param  none
 *
 * @return  64-bit value of the read time-stamp.
 */
static uint64 get_tx_timestamp_u64(void)
{
    uint8 ts_tab[5];
    uint64 ts = 0;
    int i;
    dwt_readtxtimestamp(ts_tab);
    for (i = 4; i >= 0; i--)
    {
        ts <<= 8;
        ts |= ts_tab[i];
    }
    return ts;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn get_rx_timestamp_u64()
 *
 * @brief Get the RX time-stamp in a 64-bit variable.
 *        /!\ This function assumes that length of time-stamps is 40 bits, for both TX and RX!
 *
 * @param  none
 *
 * @return  64-bit value of the read time-stamp.
 */
static uint64 get_rx_timestamp_u64(void)
{
    uint8 ts_tab[5];
    uint64 ts = 0;
    int i;
    dwt_readrxtimestamp(ts_tab);
    for (i = 4; i >= 0; i--)
    {
        ts <<= 8;
        ts |= ts_tab[i];
    }
    return ts;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn final_msg_set_ts()
 *
 * @brief Fill a given timestamp field in the final message with the given value. In the timestamp fields of the final
 *        message, the least significant byte is at the lower address.
 *
 * @param  ts_field  pointer on the first byte of the timestamp field to fill
 *         ts  timestamp value
 *
 * @return none
 */
static void final_msg_set_ts(uint8 *ts_field, uint64 ts)
{
    int i;
    for (i = 0; i < FINAL_MSG_TS_LEN; i++)
    {
        ts_field[i] = (uint8)ts;
        ts >>= 8;
    }
}
/*****************************************************************************************************************************************************
 * NOTES:
 *
 * 1. The sum of the values is the TX to RX antenna delay, experimentally determined by a calibration process. Here we use a hard coded typical value
 *    but, in a real application, each device should have its own antenna delay properly calibrated to get the best possible precision when performing
 *    range measurements.
 * 2. The messages here are similar to those used in the DecaRanging ARM application (shipped with EVK1000 kit). They comply with the IEEE
 *    802.15.4 standard MAC data frame encoding and they are following the ISO/IEC:24730-62:2013 standard. The messages used are:
 *     - a poll message sent by the initiator to trigger the ranging exchange.
 *     - a response message sent by the responder allowing the initiator to go on with the process
 *     - a final message sent by the initiator to complete the exchange and provide all information needed by the responder to compute the
 *       time-of-flight (distance) estimate.
 *    The first 10 bytes of those frame are common and are composed of the following fields:
 *     - byte 0/1: frame control (0x8841 to indicate a data frame using 16-bit addressing).
 *     - byte 2: sequence number, incremented for each new frame.
 *     - byte 3/4: PAN ID (0xDECA).
 *     - byte 5/6: destination address, see NOTE 3 below.
 *     - byte 7/8: source address, see NOTE 3 below.
 *     - byte 9: function code (specific values to indicate which message it is in the ranging process).
 *    The remaining bytes are specific to each message as follows:
 *    Poll message:
 *     - no more data
 *    Response message:
 *     - byte 10: activity code (0x02 to tell the initiator to go on with the ranging exchange).
 *     - byte 11/12: activity parameter, not used here for activity code 0x02.
 *    Final message:
 *     - byte 10 -> 13: poll message transmission timestamp.
 *     - byte 14 -> 17: response message reception timestamp.
 *     - byte 18 -> 21: final message transmission timestamp.
 *    All messages end with a 2-byte checksum automatically set by DW1000.
 * 3. Source and destination addresses are hard coded constants in this example to keep it simple but for a real product every device should have a
 *    unique ID. Here, 16-bit addressing is used to keep the messages as short as possible but, in an actual application, this should be done only
 *    after an exchange of specific messages used to define those short addresses for each device participating to the ranging exchange.
 * 4. Delays between frames have been chosen here to ensure proper synchronisation of transmission and reception of the frames between the initiator
 *    and the responder and to ensure a correct accuracy of the computed distance. The user is referred to DecaRanging ARM Source Code Guide for more
 *    details about the timings involved in the ranging process.
 * 5. This timeout is for complete reception of a frame, i.e. timeout duration must take into account the length of the expected frame. Here the value
 *    is arbitrary but chosen large enough to make sure that there is enough time to receive the complete response frame sent by the responder at the
 *    110k data rate used (around 3 ms).
 * 6. The preamble timeout allows the receiver to stop listening in situations where preamble is not starting (which might be because the responder is
 *    out of range or did not receive the message to respond to). This saves the power waste of listening for a message that is not coming. We
 *    recommend a minimum preamble timeout of 5 PACs for short range applications and a larger value (e.g. in the range of 50% to 80% of the preamble
 *    length) for more challenging longer range, NLOS or noisy environments.
 * 7. In a real application, for optimum performance within regulatory limits, it may be necessary to set TX pulse bandwidth and TX power, (using
 *    the dwt_configuretxrf API call) to per device calibrated values saved in the target system or the DW1000 OTP memory.
 * 8. dwt_writetxdata() takes the full size of the message as a parameter but only copies (size - 2) bytes as the check-sum at the end of the frame is
 *    automatically appended by the DW1000. This means that our variable could be two bytes shorter without losing any data (but the sizeof would not
 *    work anymore then as we would still have to indicate the full length of the frame to dwt_writetxdata()).
 * 9. We use polled mode of operation here to keep the example as simple as possible but all status events can be used to generate interrupts. Please
 *    refer to DW1000 User Manual for more details on "interrupts". It is also to be noted that STATUS register is 5 bytes long but, as the event we
 *    use are all in the first bytes of the register, we can use the simple dwt_read32bitreg() API call to access it instead of reading the whole 5
 *    bytes.
 * 10. As we want to send final TX timestamp in the final message, we have to compute it in advance instead of relying on the reading of DW1000
 *     register. Timestamps and delayed transmission time are both expressed in device time units so we just have to add the desired response delay to
 *     response RX timestamp to get final transmission time. The delayed transmission time resolution is 512 device time units which means that the
 *     lower 9 bits of the obtained value must be zeroed. This also allows to encode the 40-bit value in a 32-bit words by shifting the all-zero lower
 *     8 bits.
 * 11. In this operation, the high order byte of each 40-bit timestamps is discarded. This is acceptable as those time-stamps are not separated by
 *     more than 2**32 device time units (which is around 67 ms) which means that the calculation of the round-trip delays (needed in the
 *     time-of-flight computation) can be handled by a 32-bit subtraction.
 * 12. When running this example on the EVB1000 platform with the RESP_RX_TO_FINAL_TX_DLY response delay provided, the dwt_starttx() is always
 *     successful. However, in cases where the delay is too short (or something else interrupts the code flow), then the dwt_starttx() might be issued
 *     too late for the configured start time. The code below provides an example of how to handle this condition: In this case it abandons the
 *     ranging exchange to try another one after 1 second. If this error handling code was not here, a late dwt_starttx() would result in the code
 *     flow getting stuck waiting for a TX frame sent event that will never come. The companion "responder" example (ex_05b) should timeout from
 *     awaiting the "final" and proceed to have its receiver on ready to poll of the following exchange.
 * 13. The user is referred to DecaRanging ARM application (distributed with EVK1000 product) for additional practical example of usage, and to the
 *     DW1000 API Guide for more details on the DW1000 driver functions.
 ****************************************************************************************************************************************************/