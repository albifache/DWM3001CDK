/*! ----------------------------------------------------------------------------
 * @file    config_options.h
 * @brief   Configuration options are selected here.
 *
 * @author Decawave
 *
 * @copyright SPDX-FileCopyrightText: Copyright (c) 2024 Qorvo US, Inc.
 *            SPDX-License-Identifier: LicenseRef-QORVO-2
 *
 */

#include <deca_device_api.h>
#include "example_selection.h"

#ifndef EXAMPLES_CONFIG_OPTIONS_H_
#define EXAMPLES_CONFIG_OPTIONS_H_

/* Index values for errors in array */
#define CRC_ERR_IDX                0
#define RSE_ERR_IDX                1
#define PHE_ERR_IDX                2
#define SFDTO_ERR_IDX              3
#define PTO_ERR_IDX                4
#define RTO_ERR_IDX                5
#define SPICRC_ERR_IDX             6
#define TXTO_ERR_IDX               7
#define ARFE_ERR_IDX               8
#define TS_MISMATCH_ERR_IDX        9
#define BAD_FRAME_ERR_IDX          10
#define PREAMBLE_COUNT_ERR_IDX     11
#define CP_QUAL_ERR_IDX            12
#define STS_PREAMBLE_ERR           13
#define STS_PEAK_GROWTH_RATE_ERR   14
#define STS_ADC_COUNT_ERR          15
#define STS_SFD_COUNT_ERR          16
#define STS_LATE_FIRST_PATH_ERR    17
#define STS_LATE_COARSE_EST_ERR    18
#define STS_COARSE_EST_EMPTY_ERR   19
#define STS_HIGH_NOISE_THREASH_ERR 20
#define STS_NON_TRIANGLE_ERR       21
#define STS_LOG_REG_FAILED_ERR     22

/*
 * Number of ranges to attempt in test
 */
#define RANGE_COUNT 200

/* Compensation value for CPU
 * The time taken to receive the poll frame, check for errors,
 * calculate length, read content, get poll timestamp,
 * calculate response timestamp and send delayed response with timestamp will
 * be different for each device.
 * Adjusting this value lower and lower until dwt_starttx() starts returning
 * DWT_ERROR status allows the user to tweak their system to calculate the
 * shortest turn-around time for messages.
 */
#define CPU_PROCESSING_TIME 400

/*
 * SPI Rate Configuration Settings
 */
#define CONFIG_SPI_FAST_RATE
//#define CONFIG_SPI_SLOW_RATE

/*
 * Changing threshold to 5ns for DW3000 B0 red board devices.
 * ~10% of ranging attempts have a larger than usual difference between Ipatov and STS.
 * A larger threshold allows for better coverage with this coverage.
 * This should be fixed for DW3000 C0 devices.
 */
#define TS_MISMATCH_THRESHOLD 5 * 64 /* 64 = 1 ns --> 5 ns */

/*
 * Please note that a PRF of 16 MHz and a STS PRF of 64 MHz will not be supported for the DW3000.
 */
/* When no UWB configuration is needed we select this option */
#define CONFIG_OPTION_NONE 0

/* Configuration option 01.
 * Channel 5, PRF 64M, Preamble Length 64, PAC 8, Preamble code 9, Data Rate 850k, STS Length 64
 */
#define CONFIG_OPTION_01 1

/* Configuration option 02.
 * Channel 9, PRF 64M, Preamble Length 64, PAC 8, Preamble code 9, Data Rate 850k, STS Length 64
 */
#define CONFIG_OPTION_02 2

/* Configuration option 03.
 * Channel 5, PRF 64M, Preamble Length 128, PAC 8, Preamble code 9, Data Rate 850k, STS Length 64
 */
#define CONFIG_OPTION_03 3

/* Configuration option 04.
 * Channel 9, PRF 64M, Preamble Length 128, PAC 8, Preamble code 9, Data Rate 850k, STS Length 64
 */
#define CONFIG_OPTION_04 4

/* Configuration option 05.
 * Channel 5, PRF 64M, Preamble Length 512, PAC 8, Preamble code 9, Data Rate 850k, STS Length 64
 */
#define CONFIG_OPTION_05 5

/* Configuration option 06.
 * Channel 9, PRF 64M, Preamble Length 512, PAC 8, Preamble code 9, Data Rate 850k, STS Length 64
 */
#define CONFIG_OPTION_06 6

/* Configuration option 07.
 * Channel 5, PRF 64M, Preamble Length 1024, PAC 8, Preamble code 9, Data Rate 850k, STS Length 64
 */
#define CONFIG_OPTION_07 7  

/* Configuration option 08.
 * Channel 9, PRF 64M, Preamble Length 1024, PAC 8, Preamble code 9, Data Rate 850k, STS Length 64
 */
#define CONFIG_OPTION_08 8

/* Configuration option 09.
 * Channel 5, PRF 64M, Preamble Length 64, PAC 8, Preamble code 10, Data Rate 850k, STS Length 64
 */
#define CONFIG_OPTION_09 9

/* Configuration option 10.
 * Channel 9, PRF 64M, Preamble Length 64, PAC 8, Preamble code 10, Data Rate 850k, STS Length 64
 */
#define CONFIG_OPTION_10 10

/* Configuration option 11.
 * Channel 5, PRF 64M, Preamble Length 128, PAC 8, Preamble code 10, Data Rate 850k, STS Length 64
 */
#define CONFIG_OPTION_11 11

/* Configuration option 12.
 * Channel 9, PRF 64M, Preamble Length 128, PAC 8, Preamble code 10, Data Rate 850k, STS Length 64
 */
#define CONFIG_OPTION_12 12

/* Configuration option 13.
 * Channel 5, PRF 64M, Preamble Length 512, PAC 8, Preamble code 10, Data Rate 850k, STS Length 64
 */
#define CONFIG_OPTION_13 13

/* Configuration option 14.
 * Channel 9, PRF 64M, Preamble Length 512, PAC 8, Preamble code 10, Data Rate 850k, STS Length 64
 */
#define CONFIG_OPTION_14 14

/* Configuration option 15.
 * Channel 5, PRF 64M, Preamble Length 1024, PAC 8, Preamble code 10, Data Rate 850k, STS Length 64
 */
#define CONFIG_OPTION_15 15

/* Configuration option 16.
 * Channel 9, PRF 64M, Preamble Length 1024, PAC 8, Preamble code 10, Data Rate 850k, STS Length 64
 */
#define CONFIG_OPTION_16 16

/* Configuration option 17.
 * Channel 5, PRF 64M, Preamble Length 64, PAC 8, Preamble code 9, Data Rate 6.8M, STS Length 64
 */
#define CONFIG_OPTION_17 17

/* Configuration option 18.
 * Channel 9, PRF 64M, Preamble Length 64, PAC 8, Preamble code 9, Data Rate 6.8M, STS Length 64
 */
#define CONFIG_OPTION_18 18

/* Configuration option 19.
 * Channel 5, PRF 64M, Preamble Length 128, PAC 8, Preamble code 9, Data Rate 6.8M, STS Length 64
 */
#define CONFIG_OPTION_19 19

/* Configuration option 20.
 * Channel 9, PRF 64M, Preamble Length 128, PAC 8, Preamble code 9, Data Rate 6.8M, STS Length 64
 */
#define CONFIG_OPTION_20 20

/* Configuration option 21.
 * Channel 5, PRF 64M, Preamble Length 512, PAC 8, Preamble code 9, Data Rate 850k, STS Length 64
 */
#define CONFIG_OPTION_21 21

/* Configuration option 22.
 * Channel 9, PRF 64M, Preamble Length 512, PAC 8, Preamble code 9, Data Rate 6.8M, STS Length 64
 */
#define CONFIG_OPTION_22 22

/* Configuration option 23.
 * Channel 5, PRF 64M, Preamble Length 1024, PAC 8, Preamble code 9, Data Rate 6.8M, STS Length 64
 */
#define CONFIG_OPTION_23 23

/* Configuration option 24.
 * Channel 9, PRF 64M, Preamble Length 1024, PAC 8, Preamble code 9, Data Rate 6.8M, STS Length 64
 */
#define CONFIG_OPTION_24 24

/* Configuration option 25.
 * Channel 5, PRF 64M, Preamble Length 64, PAC 8, Preamble code 10, Data Rate 6.8M, STS Length 64
 */
#define CONFIG_OPTION_25 25

/* Configuration option 26.
 * Channel 9, PRF 64M, Preamble Length 64, PAC 8, Preamble code 10, Data Rate 6.8M, STS Length 64
 */
#define CONFIG_OPTION_26 26

/* Configuration option 27.
 * Channel 5, PRF 64M, Preamble Length 128, PAC 8, Preamble code 10, Data Rate 6.8M, STS Length 64
 */
#define CONFIG_OPTION_27 27

/* Configuration option 28.
 * Channel 9, PRF 64M, Preamble Length 128, PAC 8, Preamble code 10, Data Rate 6.8M, STS Length 64
 */
#define CONFIG_OPTION_28 28

/* Configuration option 29.
 * Channel 5, PRF 64M, Preamble Length 512, PAC 8, Preamble code 10, Data Rate 6.8M, STS Length 64
 */
#define CONFIG_OPTION_29 29

/* Configuration option 30.
 * Channel 9, PRF 64M, Preamble Length 512, PAC 8, Preamble code 10, Data Rate 6.8M, STS Length 64
 */
#define CONFIG_OPTION_30 30

/* Configuration option 31.
 * Channel 5, PRF 64M, Preamble Length 1024, PAC 8, Preamble code 10, Data Rate 6.8M, STS Length 64
 */
#define CONFIG_OPTION_31 31

/* Configuration option 32.
 * Channel 9, PRF 64M, Preamble Length 1024, PAC 8, Preamble code 10, Data Rate 6.8M, STS Length 64
 */
#define CONFIG_OPTION_32 32

/* Configuration option 33.
 * Channel 5, PRF 64M, Preamble Length 128, PAC 8, Preamble code 9, Data Rate 6.8M, STS Length 128
 */
#define CONFIG_OPTION_33 33

/* Configuration option 34.
 * Channel 9, PRF 64M, Preamble Length 128, PAC 8, Preamble code 9, Data Rate 6.8M, STS Length 128
 */
#define CONFIG_OPTION_34 34

/* Configuration option 35.
 * Channel 5, PRF 64M, Preamble Length 128, PAC 8, Preamble code 9, Data Rate 6.8M, STS Length 64
 */
#define CONFIG_OPTION_35 35

/* Configuration option 36.
 * Channel 9, PRF 64M, Preamble Length 128, PAC 8, Preamble code 9, Data Rate 6.8M, STS Length 64
 */
#define CONFIG_OPTION_36 36

/* Configuration option 37.
 * Channel 5, PRF 64M, Preamble Length 128, PAC 8, Preamble code 9, Data Rate 6.8M, STS Length 256
 */
#define CONFIG_OPTION_37 37

/* Configuration option 38.
 * Channel 5, PRF 64M, Preamble Length 128, PAC 8, Preamble code 9, Data Rate 6.8M, STS Length 64
 */
#define CONFIG_OPTION_38 38

/* Configuration option 39.
 * Channel 5, PRF 64M, Preamble Length 128, PAC 8, Preamble code 9, Data Rate 6.8M, STS Length 128
 */
#define CONFIG_OPTION_39 39

/* Configuration option 40.
 * Channel 5, PRF 64M, Preamble Length 1024, PAC 32, Preamble code 9, Data Rate 850K, STS Length 64
 */
#define CONFIG_OPTION_40 40

/* Configuration option 41.
 * Channel 5, PRF 64M, Preamble Length 64, PAC 8, Preamble code 9, Data Rate 6.8M, STS Length 64
 */
#define CONFIG_OPTION_41 41

/* If CMake provides a specific configuration than we select that one
 * otherwise we select the configuration based on the example being run.
 */
#ifndef CMAKE_UWB_CONFIG_OPTION
#if defined(TEST_DS_TWR_INITIATOR_STS) || defined(TEST_DS_TWR_RESPONDER_STS) \
    || defined(TEST_SS_TWR_INITIATOR_STS) || defined(TEST_SS_TWR_RESPONDER_STS) \
    || defined(TEST_SS_TWR_INITIATOR_STS_NO_DATA) || defined(TEST_SS_TWR_RESPONDER_STS_NO_DATA) 
    #define CONFIG_OPTION CONFIG_OPTION_33
#elif defined(TEST_SIMPLE_TX) || defined(TEST_TX_SLEEP_IDLE_RC) || defined(TEST_TX_SLEEP) \
    || defined(TEST_TX_SLEEP_AUTO) || defined(TEST_TX_SLEEP_TIMED) || defined(TEST_TX_WITH_CCA) \
    || defined(TEST_SIMPLE_TX_AUTOMOTIVE) || defined(TEST_SIMPLE_TX_AES) || defined(TEST_SIMPLE_RX) \
    || defined(TEST_RX_DIAG) || defined(TEST_RX_SNIFF) || defined(TEST_DOUBLE_BUFFER_RX) \
    || defined(TEST_RX_TRIM) || defined(TEST_SIMPLE_RX_CIR) || defined(TEST_SIMPLE_RX_AES) \
    || defined(TEST_TX_WAIT_RESP) || defined(TEST_TX_WAIT_RESP_INT) || defined(TEST_RX_SEND_RESP) \
    || defined(TEST_RX_ADC_CAPTURE) || defined(TEST_CONTINUOUS_FRAME) || defined(TEST_DS_TWR_INITIATOR) \
    || defined(TEST_DS_TWR_RESPONDER) || defined(TEST_SS_TWR_INITIATOR) || defined(TEST_SS_TWR_RESPONDER) \
    || defined(TEST_AES_SS_TWR_INITIATOR) || defined(TEST_AES_SS_TWR_RESPONDER) || defined(TEST_ACK_DATA_TX) \
    || defined(TEST_ACK_DATA_RX) || defined(TEST_LE_PEND_RX) || defined(TEST_LE_PEND_TX) \
    || defined(TEST_BW_CAL) || defined(TEST_TX_POWER_ADJUSTMENT)
    #define CONFIG_OPTION CONFIG_OPTION_35
#elif defined(TEST_PLL_CAL) || defined(TEST_LINEAR_TX_POWER)
    #define CONFIG_OPTION CONFIG_OPTION_36
#elif defined(TEST_SIMPLE_RX_PDOA) || defined(TEST_SIMPLE_TX_PDOA)
    #define CONFIG_OPTION CONFIG_OPTION_37
#elif defined(TEST_SIMPLE_TX_STS_SDC) || defined(TEST_SIMPLE_RX_STS_SDC)
    #define CONFIG_OPTION CONFIG_OPTION_38
#elif defined(TEST_SIMPLE_RX_NLOS)
    #define CONFIG_OPTION CONFIG_OPTION_39
#elif defined(TEST_CONTINUOUS_WAVE)
    #define CONFIG_OPTION CONFIG_OPTION_40
#elif defined(TEST_DS_TWR_STS_SDC_INITIATOR) || defined(TEST_DS_TWR_STS_SDC_RESPONDER)
    #define CONFIG_OPTION CONFIG_OPTION_41
#else
    #define CONFIG_OPTION CONFIG_OPTION_NONE
#endif
#else
    #define CONFIG_OPTION CMAKE_UWB_CONFIG_OPTION
#endif

#if CONFIG_OPTION < 0 || CONFIG_OPTION > 41
    #error "Configuration option not valid"
#endif

extern char dist_str[64];

#endif /* EXAMPLES_CONFIG_OPTIONS_H_ */
