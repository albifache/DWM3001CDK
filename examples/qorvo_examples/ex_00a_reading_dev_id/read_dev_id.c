/*! ----------------------------------------------------------------------------
 *  @file    read_dev_id.c
 *  @brief   This example just read DW IC's device ID. It can be used to verify
 *           the SPI comms are working correctly.
 *
 *
 * @author Decawave
 *
 * @copyright SPDX-FileCopyrightText: Copyright (c) 2024 Qorvo US, Inc.
 *            SPDX-License-Identifier: LicenseRef-QORVO-2
 * 
 */

#include "deca_probe_interface.h"
#include <deca_device_api.h>
#include <example_selection.h>
#include <port.h>
#include <stdio.h>
#include <zephyr/sys/printk.h>

#if defined(TEST_READING_DEV_ID)

extern void test_run_info(unsigned char *data);

/* Example application name and version to print to the console. */
#define APP_NAME "READ DEV ID      "

// Don't use for DWM3001C target
// #define USE_SPI2 1 // set this to 1 to use DW37X0 SPI2

/**
 * Application entry point.
 */
int read_dev_id(void)
{

#if USE_SPI2
    uint8_t sema_res;
#endif
    int err;
    uint32_t dev_id;
    
    // /* Print application name on the console. */
    test_run_info((unsigned char *)APP_NAME);

    //----------------- DWM3001C specific initialization -----------------
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
    //----------------- DWM3001C specific initialization -----------------

    dev_id = dwt_readdevid();
    if ((dev_id == (uint32_t)DWT_DW3720_PDOA_DEV_ID))
    {
        /* If host is using SPI 2 to connect to DW3000 the code in the USE_SPI2 above should be set to 1 */
#if USE_SPI2

        change_SPI(SPI_2);

        /* Configure SPI rate, DW3000 supports up to 38 MHz */
        port_set_dw_ic_spi_fastrate();

        /* Reset DW IC */
        reset_DWIC(); /* Target specific drive of RSTn line into DW IC low for a period. */

        Sleep(2); // Time needed for DW3000 to start up (transition from INIT_RC to IDLE_RC, or could wait for SPIRDY event)

        /* If host is using SPI 2 to connect to DW3000 the it needs to request access or force access */
        sema_res = dwt_ds_sema_status();

        if ((sema_res & (0x2)) == 0) // the SPI2 is free
        {
            dwt_ds_sema_request();
        }
        else
        {
            test_run_info((unsigned char *)"SPI2 IS NOT FREE"); // If SPI2 is not free the host can force access
            while (1) { };
        }
#endif
    }

    /* Reads and validate device ID returns DWT_ERROR if it does not match expected else DWT_SUCCESS */
    if ((err = dwt_check_dev_id()) == DWT_SUCCESS)
    {
        test_run_info((unsigned char *)"DEV ID OK");
    }
    else
    {
        test_run_info((unsigned char *)"DEV ID FAILED");
    }

    return err;
}

#endif
/*****************************************************************************************************************************************************
 * NOTES:
 ****************************************************************************************************************************************************/
