/*! ----------------------------------------------------------------------------
 *  @file    simple_rx.c
 *  @brief   Simple RX example code
 *
 * @author Decawave
 *
 * @copyright SPDX-FileCopyrightText: Copyright (c) 2024 Qorvo US, Inc.
 *            SPDX-License-Identifier: LicenseRef-QORVO-2
 *
 */

#include "deca_probe_interface.h"
#include <config_options.h>
#include <deca_device_api.h>
#include <example_selection.h>
#include <port.h>
#include <shared_defines.h>
#include <shared_functions.h>
#include <string.h>
#include <zephyr/sys/printk.h>

#if defined(TEST_SIMPLE_RX)

extern void test_run_info(unsigned char *data);

/* Example application name */
#define APP_NAME "SIMPLE RX v1.0"

/* Buffer to store received frame. See NOTE 1 below. */
static uint8_t rx_buffer[FRAME_LEN_MAX];

extern dwt_config_t config_options;

/**
 * Application entry point.
 */
int simple_rx(void)
{
    /* Hold copy of status register state here for reference so that it can be examined at a debug breakpoint. */
    uint32_t status_reg;
    /* Hold copy of frame length of frame received (if good) so that it can be examined at a debug breakpoint. */
    uint16_t frame_len;

    /* Print application name on the console. */
    test_run_info((unsigned char *)APP_NAME);

    //----------------- DW3001C specific initialization -----------------
    int16_t ret;
    ret = led_gpio_init();
    if (ret != PORT_SUCCESS)
    {
        printk("\n[ERROR] LEDs initialization failed.\n");
        while(1);
    }
    
    // Initialize DW3000 IC
    ret = deca_init();
    if (ret != PORT_SUCCESS)
    {
        printk("\n[ERROR] DW3000 initialization failed.\n");
        while(1);
    }
    //----------------- DW3001C specific initialization -----------------

    /* Configure DW IC. */
    /* if the dwt_configure returns DWT_ERROR either the PLL or RX calibration has failed the host should reset the device */
    if (dwt_configure(&config_options))
    {
        test_run_info((unsigned char *)"CONFIG FAILED     ");
        while (1) { };
    }

    /* Loop forever receiving frames. */
    while (1)
    {
        /* TESTING BREAKPOINT LOCATION #1 */

        /* Clear local RX buffer to avoid having leftovers from previous receptions  This is not necessary but is included here to aid reading
         * the RX buffer.
         * This is a good place to put a breakpoint. Here (after first time through the loop) the local status register will be set for last event
         * and if a good receive has happened the data buffer will have the data in it, and frame_len will be set to the length of the RX frame. */
        memset(rx_buffer, 0, sizeof(rx_buffer));

        /* Activate reception immediately. See NOTE 2 below. */
        dwt_rxenable(DWT_START_RX_IMMEDIATE);

        /* Poll until a frame is properly received or an error/timeout occurs. See NOTE 3 below.
         * STATUS register is 5 bytes long but, as the event we are looking at is in the first byte of the register, we can use this simplest API
         * function to access it. */
        waitforsysstatus(&status_reg, NULL, (DWT_INT_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_ERR), 0);

        if (status_reg & DWT_INT_RXFCG_BIT_MASK)
        {
            /* A frame has been received, copy it to our local buffer. */
            frame_len = dwt_getframelength(0);
            if (frame_len <= FRAME_LEN_MAX)
            {
                dwt_readrxdata(rx_buffer, frame_len - FCS_LEN, 0); /* No need to read the FCS/CRC. */
            }

            /* Clear good RX frame event in the DW IC status register. */
            dwt_writesysstatuslo(DWT_INT_RXFCG_BIT_MASK);

            test_run_info((unsigned char *)"Frame Received");
        }
        else
        {
            /* Clear RX error events in the DW IC status register. */
            dwt_writesysstatuslo(SYS_STATUS_ALL_RX_ERR);
        }
    }
}
#endif
/*****************************************************************************************************************************************************
 * NOTES:
 *
 * 1. In this example, maximum frame length is set to 127 bytes which is 802.15.4 UWB standard maximum frame length. DW IC supports an extended
 *    frame length (up to 1023 bytes long) mode which is not used in this example.
 * 2. Manual reception activation is performed here but DW IC offers several features that can be used to handle more complex scenarios or to
 *    optimise system's overall performance (e.g. timeout after a given time, automatic re-enabling of reception in case of errors, etc.).
 * 3. We use polled mode of operation here to keep the example as simple as possible, but RXFCG and error/timeout status events can be used to generate
 *    interrupts. Please refer to DW IC User Manual for more details on "interrupts".
 ****************************************************************************************************************************************************/
