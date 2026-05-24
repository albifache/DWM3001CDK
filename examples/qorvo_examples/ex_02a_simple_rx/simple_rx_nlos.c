/*! ----------------------------------------------------------------------------
 *  @file    simple_rx_nlos.c
 *  @brief   Simple RX NLOS example code
 *           This is a simple code example that turns on the DW IC receiver to receive a frame, (expecting the frame as sent by the companion simple
 *           example "Simple TX example code"). When a frame is received and validated the diagnostic register values are read and calculations
 *           for First Path Power based on the section 4.7.1 and Estimating the receive signal power based on 4.7.2 of the User Manual.
 *           The probability of signal being Line of Sight or Non-Line of Sight is calculated based on the "APS006 Part 3 DW1000 Diagnostics for NLOS Channels".
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
#include <math.h>
#include <port.h>
#include <shared_defines.h>
#include <shared_functions.h>
#include <string.h>
#include <zephyr/sys/printk.h>

#if defined(TEST_SIMPLE_RX_NLOS)

extern void test_run_info(unsigned char *data);

/* Example application name */
#define APP_NAME "SIMPLE RX_NLOS v1.0"

#define SIG_LVL_FACTOR     0.4     // Factor between 0 and 1; default 0.4 from experiments and simulations.
#define SIG_LVL_THRESHOLD  12      // Threshold unit is dB; default 12 dB from experiments and simulations.
#define ALPHA_PRF_16       113.8   // Constant A for PRF of 16 MHz. See User Manual for more information.
#define ALPHA_PRF_64       120.7   // Constant A for PRF of 64 MHz. See User Manual for more information.
#define RX_CODE_THRESHOLD  8       // For 64 MHz PRF the RX code is 9.
#define LOG_CONSTANT_C0    63.2    // 10log10(2^21) = 63.2    // See User Manual for more information.
#define LOG_CONSTANT_D0_E0 51.175  // 10log10(2^17) = 51.175  // See User Manual for more information.
#define IP_MIN_THRESHOLD   3.3     // The minimum difference between PP and FP indices. Please see Application Note "APS006 Part 3", NOTE 10
#define IP_MAX_THRESHOLD   6.0     // The maximum difference between PP and FP indices. Please see Application Note "APS006 Part 3", NOTE 10
#define CONSTANT_PR_IP_A   0.39178 // Constant from simulations on DW device accumulator, please see Application Note "APS006 Part 3"
#define CONSTANT_PR_IP_B   1.31719 // Constant from simulations on DW device accumulator, please see Application Note "APS006 Part 3"

/* Buffer to store received frame. See NOTE 1 below. */
static uint8_t rx_buffer[FRAME_LEN_MAX];

char prob_str[30] = { 0 };

extern dwt_config_t config_options;

/**
 * Application entry point.
 */
int simple_rx_nlos(void)
{
    /* Hold copy of status register state here for reference so that it can be examined at a debug breakpoint. */
    uint32_t status_reg = 0;
    /* Hold copy of frame length of frame received (if good) so that it can be examined at a debug breakpoint. */
    uint16_t frame_len = 0;

    /* Line-of-sight / Non-line-of-sight Variables */
    uint32_t dev_id;
    uint8_t D;
    dwt_nlos_alldiag_t all_diag;
    dwt_nlos_ipdiag_t index;

    // All float variables used for recording different diagnostic results and probability.
    float ip_f1, ip_f2, ip_f3, sts1_f1, sts1_f2, sts1_f3, sts2_f1, sts2_f2, sts2_f3 = 0;
    float ip_n, sts1_n, sts2_n, ip_cp, sts1_cp, sts2_cp, ip_rsl, ip_fsl, sts1_rsl, sts1_fsl, sts2_rsl, sts2_fsl = 0;
    float pr_nlos, sl_diff_ip, sl_diff_sts1, sl_diff_sts2, sl_diff, index_diff = 0;
    float alpha, ip_alpha, log_constant = 0;

    /* Print application name on the console. */
    test_run_info((unsigned char *)APP_NAME);

    //----------------- DW3001C specificc initialization -----------------
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
    //----------------- DW3001Cc specific initialization -----------------

    dev_id = dwt_readdevid();

    /* Configure DW IC. */
    /* if the dwt_configure returns DWT_ERROR either the PLL or RX calibration has failed the host should reset the device */
    if (dwt_configure(&config_options))
    {
        test_run_info((unsigned char *)"CONFIG FAILED     ");
        while (1) { };
    }

    /* See NOTE 4 below. */
    dwt_configciadiag(DW_CIA_DIAG_LOG_ALL);
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

        // Check if the received frame is good.
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

            if ((dev_id == (uint32_t)DWT_DW3000_DEV_ID) || (dev_id == (uint32_t)DWT_DW3000_PDOA_DEV_ID))
            {
                log_constant = LOG_CONSTANT_C0;
            }
            else
            {
                log_constant = LOG_CONSTANT_D0_E0;
            }

            // Select IPATOV to read Ipatov diagnostic registers from API function dwt_nlos_alldiag()
            all_diag.diag_type = IPATOV;
            dwt_nlos_alldiag(&all_diag);
            ip_alpha = (config_options.rxCode > RX_CODE_THRESHOLD) ? (-ALPHA_PRF_64) : -(ALPHA_PRF_16);
            ip_n = all_diag.accumCount; // The number of preamble symbols accumulated
            ip_f1 = all_diag.F1 / 4;    // The First Path Amplitude (point 1) magnitude value (it has 2 fractional bits),
            ip_f2 = all_diag.F2 / 4;    // The First Path Amplitude (point 2) magnitude value (it has 2 fractional bits),
            ip_f3 = all_diag.F3 / 4;    // The First Path Amplitude (point 3) magnitude value (it has 2 fractional bits),
            ip_cp = all_diag.cir_power;

            // Select STS1 to read STS1 diagnostic registers from API function dwt_nlos_alldiag()
            all_diag.diag_type = STS1;
            dwt_nlos_alldiag(&all_diag);
            alpha = -(ALPHA_PRF_64 + 1);  // for STS alpha = ALPHA_PRF_64 + 1
            sts1_n = all_diag.accumCount; // The number of preamble symbols accumulated
            sts1_f1 = all_diag.F1 / 4;    // The First Path Amplitude (point 1) magnitude value (it has 2 fractional bits),
            sts1_f2 = all_diag.F2 / 4;    // The First Path Amplitude (point 2) magnitude value (it has 2 fractional bits),
            sts1_f3 = all_diag.F3 / 4;    // The First Path Amplitude (point 3) magnitude value (it has 2 fractional bits),
            sts1_cp = all_diag.cir_power;

            // Select STS2 to read STS2 diagnostic registers from API function dwt_nlos_alldiag()
            all_diag.diag_type = STS2;
            dwt_nlos_alldiag(&all_diag);
            sts2_n = all_diag.accumCount; // The number of preamble symbols accumulated
            sts2_f1 = all_diag.F1 / 4;    // The First Path Amplitude (point 1) magnitude value (it has 2 fractional bits),
            sts2_f2 = all_diag.F2 / 4;    // The First Path Amplitude (point 2) magnitude value (it has 2 fractional bits),
            sts2_f3 = all_diag.F3 / 4;    // The First Path Amplitude (point 3) magnitude value (it has 2 fractional bits),
            sts2_cp = all_diag.cir_power;

            // The calculation of First Path Power Level(FSL) and Receive Signal Power Level(RSL) is taken from
            // DW3000 User Manual section 4.7.1 & 4.7.2

            // For IPATOV
            ip_n *= ip_n;
            ip_f1 *= ip_f1;
            ip_f2 *= ip_f2;
            ip_f3 *= ip_f3;

            // For STS1
            sts1_n *= sts1_n;
            sts1_f1 *= sts1_f1;
            sts1_f2 *= sts1_f2;
            sts1_f3 *= sts1_f3;

            // For STS2
            sts2_n *= sts2_n;
            sts2_f1 *= sts2_f1;
            sts2_f2 *= sts2_f2;
            sts2_f3 *= sts2_f3;

            D = all_diag.D * 6;

            // Calculate the First Signal Level(FSL) and Receive Signal Level(RSL) then subtract FSL from RSL
            // to find out Signal Level Difference which is compared to defined Signal Threshold.

            // For Ipatov CIR
            ip_rsl = 10 * log10f((float)ip_cp / ip_n) + ip_alpha + log_constant + D;
            ip_fsl = 10 * log10f(((ip_f1 + ip_f2 + ip_f3) / ip_n)) + ip_alpha + D;

            // Signal Level Difference value for Ipatov.
            sl_diff_ip = ip_rsl - ip_fsl;

            // For the STS1 CIR
            sts1_rsl = 10 * log10f((float)sts1_cp / sts1_n) + alpha + log_constant + D;
            sts1_fsl = 10 * log10f(((sts1_f1 + sts1_f2 + sts1_f3) / sts1_n)) + alpha + D;

            // For the STS2 CIR
            sts2_rsl = 10 * log10f((float)sts2_cp / sts2_n) + alpha + log_constant + D;
            sts2_fsl = 10 * log10f(((sts2_f1 + sts2_f2 + sts2_f3) / sts2_n)) + alpha + D;

            // STS Mode OFF, Signal Level Difference of STS1 and STS2 is set to zero, as these values are not used
            if (config_options.stsMode == DWT_STS_MODE_OFF)
            {
                sl_diff_sts1 = 0;
                sl_diff_sts2 = 0;
            }
            else // If PDOA Mode 3 is enabled then there's Signal Level Difference value for all three CIRs: Ipatov, STS1 and STS2.
            {
                sl_diff_sts1 = sts1_rsl - sts1_fsl;

                // IF PDOA Mode 3 is not enabled then Signal Level Difference of STS2 is zero.
                if (config_options.pdoaMode != DWT_PDOA_M3)
                {
                    sl_diff_sts2 = 0;
                }
                else
                {
                    sl_diff_sts2 = sts2_rsl - sts2_fsl;
                }
            }

            /* Check for Line-of-sight or Non-line-of-sight */
            // If the received signal power difference is above 12 dB then the signal is considered Non Line of Sight.
            // Otherwise a probability value of non-line of sight is calculated.
            //
            // Calculating the probability of NLOS.
            // 1. If the signal level difference of IPATOV, STS1 or STS2 is greater than 12 dB then the signal is Non Line of sight.
            if ((sl_diff_ip > SIG_LVL_THRESHOLD) || (sl_diff_sts1 > SIG_LVL_THRESHOLD) || (sl_diff_sts2 > SIG_LVL_THRESHOLD))
            {
                pr_nlos = 100;
                test_run_info((unsigned char *)"Non-Line of sight");
            }

            // If the difference in received signal power is between 4.8 dB and 12 dB then there's a possibility that the signal is
            // Non Line of Sight and probability of it being Non Line of Sight signal is calculated.
            //
            // 2. If any the signal level difference of IPATOV, STS1 or STS2 is greater than
            //    (Signal Level Threshold(12) * Signal Level Factor(0.4)) = 4.8 dB but less than 12 dB, then calculate the
            //    probability of non-line of sight based on the signal level difference which is above the threshold (IPATOV, STS1 or STS2).

            else if ((sl_diff_ip > SIG_LVL_THRESHOLD * SIG_LVL_FACTOR) || (sl_diff_sts1 > SIG_LVL_THRESHOLD * SIG_LVL_FACTOR)
                     || (sl_diff_sts2 > SIG_LVL_THRESHOLD * SIG_LVL_FACTOR))
            {
                if (sl_diff_ip > SIG_LVL_THRESHOLD * SIG_LVL_FACTOR)
                {
                    sl_diff = sl_diff_ip;
                }
                else if (sl_diff_sts1 > SIG_LVL_THRESHOLD * SIG_LVL_FACTOR)
                {
                    sl_diff = sl_diff_sts1;
                }
                else
                {
                    sl_diff = sl_diff_sts2;
                }

                pr_nlos = 100 * ((sl_diff / SIG_LVL_THRESHOLD - SIG_LVL_FACTOR) / (1 - SIG_LVL_FACTOR));
                snprintf(prob_str, sizeof(prob_str), "Probability of NLOS: %3.2f", (double)fabsf(pr_nlos));
                test_run_info((unsigned char *)prob_str);
            }

            // 3. If the signal is less than the Combined Threshold (Signal Level Threshold(12) * Signal Level Factor(0.4)) for all three
            //    IPATOV, STS1 and STS2 reported through dwt_nlos_alldiag() then check the IPATOV Diagnostic First Path and Peak Path Index
            //    through dwt_nlos_ipdiag().
            //    NOTE 10 below:
            //    3.a. If the index difference is less than 3.3 then it's considered a Line of Sight signal.
            //    3.b. If the index difference is greater than 3.3 and less than 6 then the probability of Non Line of Sight
            //         is calculated.
            //    3.c. If the index level is greater than 6 dB then it's considered a Non Line of Sight signal.
            else
            {
                dwt_nlos_ipdiag(&index); //NOTE the PP and FP indices in the dwt_nlos_ipdiag_t structure are in [9:-6] format
                                         //thus we divide the difference by 64 below
                index_diff = ((float)index.index_pp_u32 - (float)index.index_fp_u32) / 64;

                if (index_diff <= IP_MIN_THRESHOLD)
                {
                    pr_nlos = 0;
                    test_run_info((unsigned char *)"Line of Sight");
                }
                else if ((index_diff > IP_MIN_THRESHOLD) && (index_diff < IP_MAX_THRESHOLD))
                {
                    pr_nlos = 100 * ((CONSTANT_PR_IP_A * index_diff) - CONSTANT_PR_IP_B);
                    snprintf(prob_str, sizeof(prob_str), "**Probability of NLOS: %3.2f", (double)fabsf(pr_nlos));
                    test_run_info((unsigned char *)prob_str);
                }
                else
                {
                    pr_nlos = 100;
                    test_run_info((unsigned char *)"Non-Line of Sight");
                }
            }
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
 * 1.  In this example, maximum frame length is set to 127 bytes which is 802.15.4 UWB standard maximum frame length. DW IC supports an extended
 *     frame length (up to 1023 bytes long) mode which is not used in this example.
 * 2.  Manual reception activation is performed here but DW IC offers several features that can be used to handle more complex scenarios or to
 *     optimise system's overall performance (e.g. timeout after a given time, automatic re-enabling of reception in case of errors, etc.).
 * 3.  We use polled mode of operation here to keep the example as simple as possible, but RXFCG and error/timeout status events can be used to
 *     generate interrupts. Please refer to DW IC User Manual for more details on "interrupts".
 * 4.  Enable the CIA Diagnostics "dwt_configciadiag()" before RX ENABLE.
 *
 * Please see defines at the beginning of the file for detailed explanation of the threshold values.
 * 5.  The Signal Level Threshold is 12 dB and Signal Level Factor 0.4 dB.
 * 6.  If PDOA Mode 3 is enabled then all three IPATOV, STS1 and STS2 will report a Signal Level difference.
 * 7.  If the signal level difference of IPATOV, STS1 or STS2 is greater than 12 dB then the signal is Non Line of sight.
 * 8.  If the signal level difference of IPATOV, STS1 or STS2 is greater than Signal Level Threshold * Signal Level Factor(12*0.4) = 4.8 dB then
 *     calculate the "Probability of Non Line of Sight" based on the signal which has a greater signal level difference(IPATOV, STS1 or STS2).
 * 9.  If the signal is less than the Combined Threshold (Signal Level Threshold * Signal Level Factor) for all three - IPATOV, STS1 and STS2
 *     reported through "dwt_nlos_alldiag()" then check the IPATOV Diagnostic First Path and Peak Path Index through "dwt_nlos_ipdiag()".
 * 10. When STS is OFF and the index difference is less than 3.3 then it's a Line of Sight signal. See "APS006 Part 3 DW1000 Diagnostics for NLOS Channels"
 *     When STS is OFF and the index difference is greater than 3.3 and less than 6 then the probability of Non Line of Sight
 *     is calculated. See "APS006 Part 3 DW1000 Diagnostics for NLOS Channels"
 *     When STS is OFF and the index difference is greater than 6 then it's a Non Line of Sight signal. See "APS006 Part 3 DW1000 Diagnostics for NLOS Channels"
 ****************************************************************************************************************************************************/
