/*! ----------------------------------------------------------------------------
 * @file    utils.c
 * @brief   Various utility functions are kept here. Functions are not to be released to customers.
 *
 * @author Decawave
 *
 * @copyright SPDX-FileCopyrightText: Copyright (c) 2024 Qorvo US, Inc.
 *            SPDX-License-Identifier: LicenseRef-QORVO-2
 *
 */

#include <stdio.h>
/* Used for printf() code */
#include <math.h>
#include <stdio.h>

#include "config_options.h"
#include <deca_device_api.h>

#if 0 // FIXME: need to update reg dump when the reduced regs.h file is used
#define DIAG0_ID   0x00060018
#define DIAG1_ID   0x0006001C
#define DIAG2_ID   0x00060020
#define DIAG3_ID   0x00060024
#define DIAG4_ID   0x00060028
#define DIAG5_ID   0x0006002C
#define DIAG6_ID   0x00060030
#define DIG_CFG_ID 0x000A0000
#define CTRL_ID    0x000A0004
#define WDATA_ID   0x000B0000
#define ADDR_ID    0x000B0004
#define CFG_ID     0x000B0008
#define STATUS_ID  0x000B000C
#define RDATA_ID   0x000B0010
#define SRDATA_ID  0x000B0014
#define CR0_ID     0x00100000
#define CR1_ID     0x00100004
#define CR2_ID     0x00100008
#define CR3_ID     0x0010000C
#define CR4_ID     0x00100010
#define CR5_ID     0x00100014
#define CR6_ID     0x00100018
#define CR7_ID     0x0010001C
#define CR8_ID     0x00100020
#define CR9_ID     0x00100024
#define CR10_ID    0x00100028

void dump_registers(void)
{
    int i = 0;

    printf("Reg[%08X]=TxBuffer\n", TX_BUFFER_ID);
    for (i = 0 ; i < 1024 ; i++)
    {
        printf("%02X", dwt_read8bitoffsetreg(TX_BUFFER_ID, i));
    }
    printf("\n");

    printf("Reg[%08X]=RxBuffer 0\n", RX_BUFFER_0_ID);
    for (i = 0 ; i < 1024 ; i++)
    {
        printf("%02X", dwt_read8bitoffsetreg(RX_BUFFER_0_ID, i));
    }
    printf("\n");

    printf("Reg[%08X]=RxBuffer 1\n", RX_BUFFER_1_ID);
    for (i = 0 ; i < 1024 ; i++)
    {
        printf("%02X", dwt_read8bitoffsetreg(RX_BUFFER_1_ID, i));
    }
    printf("\n");

    // TODO: This does not contain all the required registers for DW3000 C0 - Will need to update this.
    printf("DEV_ID\tReg[%08X]=\t%08X\n", (unsigned int)DEV_ID_ID, (unsigned int)dwt_read32bitreg(DEV_ID_ID));
    printf("EUI_64_LO\tReg[%08X]=\t%08X\n", (unsigned int)EUI_64_LO_ID, (unsigned int)dwt_read32bitreg(EUI_64_LO_ID));
    printf("EUI_64_HI\tReg[%08X]=\t%08X\n", (unsigned int)EUI_64_HI_ID, (unsigned int)dwt_read32bitreg(EUI_64_HI_ID));
    printf("PANADR\tReg[%08X]=\t%08X\n", (unsigned int)PANADR_ID, (unsigned int)dwt_read32bitreg(PANADR_ID));
    printf("SYS_CFG\tReg[%08X]=\t%08X\n", (unsigned int)SYS_CFG_ID, (unsigned int)dwt_read32bitreg(SYS_CFG_ID));
    printf("ADR_FILT_CFG\tReg[%08X]=\t%08X\n", (unsigned int)ADR_FILT_CFG_ID, (unsigned int)dwt_read32bitreg(ADR_FILT_CFG_ID));
    printf("SPICRC_CFG\tReg[%08X]=\t%08X\n", (unsigned int)SPICRC_CFG_ID, (unsigned int)dwt_read32bitreg(SPICRC_CFG_ID));
    printf("SYS_TIME\tReg[%08X]=\t%08X\n", (unsigned int)SYS_TIME_ID, (unsigned int)dwt_read32bitreg(SYS_TIME_ID));
    printf("TX_FCTRL_LO\tReg[%08X]=\t%08X\n", (unsigned int)TX_FCTRL_ID, (unsigned int)dwt_read32bitreg(TX_FCTRL_ID));
    printf("TX_FCTRL_HI\tReg[%08X]=\t%08X\n", (unsigned int)TX_FCTRL_HI_ID, (unsigned int)dwt_read32bitreg(TX_FCTRL_HI_ID));
    printf("DX_TIME\tReg[%08X]=\t%08X\n", (unsigned int)DX_TIME_ID, (unsigned int)dwt_read32bitreg(DX_TIME_ID));
    printf("DREF_TIME\tReg[%08X]=\t%08X\n", (unsigned int)DREF_TIME_ID, (unsigned int)dwt_read32bitreg(DREF_TIME_ID));
    printf("RX_FWTO\tReg[%08X]=\t%08X\n", (unsigned int)RX_FWTO_ID, (unsigned int)dwt_read32bitreg(RX_FWTO_ID));
    printf("SYS_CTRL\tReg[%08X]=\t%08X\n", (unsigned int)SYS_CTRL_ID, (unsigned int)dwt_read32bitreg(SYS_CTRL_ID));
    printf("SYS_ENABLE_LO\tReg[%08X]=\t%08X\n", (unsigned int)SYS_ENABLE_LO_ID, (unsigned int)dwt_read32bitreg(SYS_ENABLE_LO_ID));
    printf("SYS_ENABLE_HI\tReg[%08X]=\t%08X\n", (unsigned int)SYS_ENABLE_HI_ID, (unsigned int)dwt_read32bitreg(SYS_ENABLE_HI_ID));
    printf("SYS_STATUS_LO\tReg[%08X]=\t%08X\n", (unsigned int)SYS_STATUS_ID, (unsigned int)dwt_read32bitreg(SYS_STATUS_ID));
    printf("SYS_STATUS_HI\tReg[%08X]=\t%08X\n", (unsigned int)SYS_STATUS_HI_ID, (unsigned int)dwt_read32bitreg(SYS_STATUS_HI_ID));
    printf("RX_FINFO\tReg[%08X]=\t%08X\n", (unsigned int)RX_FINFO_ID, (unsigned int)dwt_read32bitreg(RX_FINFO_ID));
    printf("RX_FQUAL_LO\tReg[%08X]=\t%08X\n", (unsigned int)RX_FQUAL_LO_ID, (unsigned int)dwt_read32bitreg(RX_FQUAL_LO_ID));
    printf("RX_FQUAL_HI\tReg[%08X]=\t%08X\n", (unsigned int)RX_FQUAL_HI_ID, (unsigned int)dwt_read32bitreg(RX_FQUAL_HI_ID));
    printf("RX_TTCKO_LO\tReg[%08X]=\t%08X\n", (unsigned int)RX_TTCKO_LO_ID, (unsigned int)dwt_read32bitreg(RX_TTCKO_LO_ID));
    printf("RX_TTCKO_HI\tReg[%08X]=\t%08X\n", (unsigned int)RX_TTCKO_HI_ID, (unsigned int)dwt_read32bitreg(RX_TTCKO_HI_ID));
    printf("RX_TIME_0\tReg[%08X]=\t%08X\n", (unsigned int)RX_TIME_0_ID, (unsigned int)dwt_read32bitreg(RX_TIME_0_ID));
    printf("RX_TIME_1\tReg[%08X]=\t%08X\n", (unsigned int)RX_TIME_1_ID, (unsigned int)dwt_read32bitreg(RX_TIME_1_ID));
    printf("RX_TIME_2\tReg[%08X]=\t%08X\n", (unsigned int)RX_TIME_2_ID, (unsigned int)dwt_read32bitreg(RX_TIME_2_ID));
    printf("RX_TIME_3\tReg[%08X]=\t%08X\n", (unsigned int)RX_TIME_3_ID, (unsigned int)dwt_read32bitreg(RX_TIME_3_ID));
    printf("TX_TIME_LO\tReg[%08X]=\t%08X\n", (unsigned int)TX_TIME_LO_ID, (unsigned int)dwt_read32bitreg(TX_TIME_LO_ID));
    printf("TX_TIME_HI\tReg[%08X]=\t%08X\n", (unsigned int)TX_TIME_HI_ID, (unsigned int)dwt_read32bitreg(TX_TIME_HI_ID));
    printf("TX_TIME_2\tReg[%08X]=\t%08X\n", (unsigned int)TX_TIME_2_ID, (unsigned int)dwt_read32bitreg(TX_TIME_2_ID));
    printf("TX_ANTD\tReg[%08X]=\t%08X\n", (unsigned int)TX_ANTD_ID, (unsigned int)dwt_read32bitreg(TX_ANTD_ID));
    printf("ACK_RESP\tReg[%08X]=\t%08X\n", (unsigned int)ACK_RESP_ID, (unsigned int)dwt_read32bitreg(ACK_RESP_ID));
    printf("TX_POWER\tReg[%08X]=\t%08X\n", (unsigned int)TX_POWER_ID, (unsigned int)dwt_read32bitreg(TX_POWER_ID));
    printf("CHAN_CTRL\tReg[%08X]=\t%08X\n", (unsigned int)CHAN_CTRL_ID, (unsigned int)dwt_read32bitreg(CHAN_CTRL_ID));
    printf("LE_PEND_01\tReg[%08X]=\t%08X\n", (unsigned int)LE_PEND_01_ID, (unsigned int)dwt_read32bitreg(LE_PEND_01_ID));
    printf("LE_PEND_23\tReg[%08X]=\t%08X\n", (unsigned int)LE_PEND_23_ID, (unsigned int)dwt_read32bitreg(LE_PEND_23_ID));
    printf("SPI_COLLISION_STATUS\tReg[%08X]=\t%08X\n", (unsigned int)SPI_COLLISION_STATUS_ID, (unsigned int)dwt_read32bitreg(SPI_COLLISION_STATUS_ID));
    printf("REGMAP_VER\tReg[%08X]=\t%08X\n", (unsigned int)REGMAP_VER_ID, (unsigned int)dwt_read32bitreg(REGMAP_VER_ID));
    printf("AES_CFG\tReg[%08X]=\t%08X\n", (unsigned int)AES_CFG_ID, (unsigned int)dwt_read32bitreg(AES_CFG_ID));
    printf("AES_IV0\tReg[%08X]=\t%08X\n", (unsigned int)AES_IV0_ID, (unsigned int)dwt_read32bitreg(AES_IV0_ID));
    printf("AES_IV1\tReg[%08X]=\t%08X\n", (unsigned int)AES_IV1_ID, (unsigned int)dwt_read32bitreg(AES_IV1_ID));
    printf("AES_IV2\tReg[%08X]=\t%08X\n", (unsigned int)AES_IV2_ID, (unsigned int)dwt_read32bitreg(AES_IV2_ID));
    printf("DMA_CFG0\tReg[%08X]=\t%08X\n", (unsigned int)DMA_CFG0_ID, (unsigned int)dwt_read32bitreg(DMA_CFG0_ID));
    printf("DMA_CFG1\tReg[%08X]=\t%08X\n", (unsigned int)DMA_CFG1_ID, (unsigned int)dwt_read32bitreg(DMA_CFG1_ID));
    printf("AES_START\tReg[%08X]=\t%08X\n", (unsigned int)AES_START_ID, (unsigned int)dwt_read32bitreg(AES_START_ID));
    printf("AES_STS\tReg[%08X]=\t%08X\n", (unsigned int)AES_STS_ID, (unsigned int)dwt_read32bitreg(AES_STS_ID));
    printf("AES_KEY0\tReg[%08X]=\t%08X\n", (unsigned int)AES_KEY0_ID, (unsigned int)dwt_read32bitreg(AES_KEY0_ID));
    printf("AES_KEY1\tReg[%08X]=\t%08X\n", (unsigned int)AES_KEY1_ID, (unsigned int)dwt_read32bitreg(AES_KEY1_ID));
    printf("AES_KEY2\tReg[%08X]=\t%08X\n", (unsigned int)AES_KEY2_ID, (unsigned int)dwt_read32bitreg(AES_KEY2_ID));
    printf("AES_KEY3\tReg[%08X]=\t%08X\n", (unsigned int)AES_KEY3_ID, (unsigned int)dwt_read32bitreg(AES_KEY3_ID));
    printf("CP_CFG0\tReg[%08X]=\t%08X\n", (unsigned int)CP_CFG0_ID, (unsigned int)dwt_read32bitreg(CP_CFG0_ID));
    printf("CP_STS\tReg[%08X]=\t%08X\n", (unsigned int)CP_STS_ID, (unsigned int)dwt_read32bitreg(CP_STS_ID));
    printf("CP_KEY0\tReg[%08X]=\t%08X\n", (unsigned int)CP_KEY0_ID, (unsigned int)dwt_read32bitreg(CP_KEY0_ID));
    printf("CP_KEY1\tReg[%08X]=\t%08X\n", (unsigned int)CP_KEY1_ID, (unsigned int)dwt_read32bitreg(CP_KEY1_ID));
    printf("CP_KEY2\tReg[%08X]=\t%08X\n", (unsigned int)CP_KEY2_ID, (unsigned int)dwt_read32bitreg(CP_KEY2_ID));
    printf("CP_KEY3\tReg[%08X]=\t%08X\n", (unsigned int)CP_KEY3_ID, (unsigned int)dwt_read32bitreg(CP_KEY3_ID));
    printf("CP_IV0\tReg[%08X]=\t%08X\n", (unsigned int)CP_IV0_ID, (unsigned int)dwt_read32bitreg(CP_IV0_ID));
    printf("CP_IV1\tReg[%08X]=\t%08X\n", (unsigned int)CP_IV1_ID, (unsigned int)dwt_read32bitreg(CP_IV1_ID));
    printf("CP_IV2\tReg[%08X]=\t%08X\n", (unsigned int)CP_IV2_ID, (unsigned int)dwt_read32bitreg(CP_IV2_ID));
    printf("CP_IV3\tReg[%08X]=\t%08X\n", (unsigned int)CP_IV3_ID, (unsigned int)dwt_read32bitreg(CP_IV3_ID));
    printf("USER_MASK0\tReg[%08X]=\t%08X\n", (unsigned int)USER_MASK0_ID, (unsigned int)dwt_read32bitreg(USER_MASK0_ID));
    printf("USER_MASK1\tReg[%08X]=\t%08X\n", (unsigned int)USER_MASK1_ID, (unsigned int)dwt_read32bitreg(USER_MASK1_ID));
    printf("LCSS_MARGIN\tReg[%08X]=\t%08X\n", (unsigned int)LCSS_MARGIN_ID, (unsigned int)dwt_read32bitreg(LCSS_MARGIN_ID));
    printf("MRX\tReg[%08X]=\t%08X\n", (unsigned int)MRX_ID, (unsigned int)dwt_read32bitreg(MRX_ID));
    printf("MRX_PGF_LUT\tReg[%08X]=\t%08X\n", (unsigned int)MRX_PGF_LUT_ID, (unsigned int)dwt_read32bitreg(MRX_PGF_LUT_ID));
    printf("ADC_CFG\tReg[%08X]=\t%08X\n", (unsigned int)ADC_CFG_ID, (unsigned int)dwt_read32bitreg(ADC_CFG_ID));
    printf("ADC_ZERO_THRESH_CFG\tReg[%08X]=\t%08X\n", (unsigned int)ADC_ZERO_THRESH_CFG_ID, (unsigned int)dwt_read32bitreg(ADC_ZERO_THRESH_CFG_ID));
    printf("ADC_THRESH_CFG\tReg[%08X]=\t%08X\n", (unsigned int)ADC_THRESH_CFG_ID, (unsigned int)dwt_read32bitreg(ADC_THRESH_CFG_ID));
    printf("AGC_CFG\tReg[%08X]=\t%08X\n", (unsigned int)AGC_CFG_ID, (unsigned int)dwt_read32bitreg(AGC_CFG_ID));
    printf("DGC_CFG\tReg[%08X]=\t%08X\n", (unsigned int)DGC_CFG_ID, (unsigned int)dwt_read32bitreg(DGC_CFG_ID));
    printf("DGC_ACC_CORR_LUT_CFG\tReg[%08X]=\t%08X\n", (unsigned int)DGC_ACC_CORR_LUT_CFG_ID, (unsigned int)dwt_read32bitreg(DGC_ACC_CORR_LUT_CFG_ID));
    printf("DGC_CORR_LUT_7_16_CFG\tReg[%08X]=\t%08X\n", (unsigned int)DGC_CORR_LUT_7_16_CFG_ID, (unsigned int)dwt_read32bitreg(DGC_CORR_LUT_7_16_CFG_ID));
    printf("DGC_CORR_LUT_17_26_CFG\tReg[%08X]=\t%08X\n", (unsigned int)DGC_CORR_LUT_17_26_CFG_ID, (unsigned int)dwt_read32bitreg(DGC_CORR_LUT_17_26_CFG_ID));
    printf("DGC_CORR_NRG_LUT_CFG\tReg[%08X]=\t%08X\n", (unsigned int)DGC_CORR_NRG_LUT_CFG_ID, (unsigned int)dwt_read32bitreg(DGC_CORR_NRG_LUT_CFG_ID));
    printf("DGC_NRG_LUT_5_14_CFG\tReg[%08X]=\t%08X\n", (unsigned int)DGC_NRG_LUT_5_14_CFG_ID, (unsigned int)dwt_read32bitreg(DGC_NRG_LUT_5_14_CFG_ID));
    printf("DGC_NRG_LUT_15_24_CFG\tReg[%08X]=\t%08X\n", (unsigned int)DGC_NRG_LUT_15_24_CFG_ID, (unsigned int)dwt_read32bitreg(DGC_NRG_LUT_15_24_CFG_ID));
    printf("DGC_NRG_LUT_CFG\tReg[%08X]=\t%08X\n", (unsigned int)DGC_NRG_LUT_CFG_ID, (unsigned int)dwt_read32bitreg(DGC_NRG_LUT_CFG_ID));
    printf("DGC_DGC_LUT_0_CFG\tReg[%08X]=\t%08X\n", (unsigned int)DGC_DGC_LUT_0_CFG_ID, (unsigned int)dwt_read32bitreg(DGC_DGC_LUT_0_CFG_ID));
    printf("DGC_DGC_LUT_1_CFG\tReg[%08X]=\t%08X\n", (unsigned int)DGC_DGC_LUT_1_CFG_ID, (unsigned int)dwt_read32bitreg(DGC_DGC_LUT_1_CFG_ID));
    printf("DGC_DGC_LUT_2_CFG\tReg[%08X]=\t%08X\n", (unsigned int)DGC_DGC_LUT_2_CFG_ID, (unsigned int)dwt_read32bitreg(DGC_DGC_LUT_2_CFG_ID));
    printf("DGC_DGC_LUT_3_CFG\tReg[%08X]=\t%08X\n", (unsigned int)DGC_DGC_LUT_3_CFG_ID, (unsigned int)dwt_read32bitreg(DGC_DGC_LUT_3_CFG_ID));
    printf("DGC_DGC_LUT_4_CFG\tReg[%08X]=\t%08X\n", (unsigned int)DGC_DGC_LUT_4_CFG_ID, (unsigned int)dwt_read32bitreg(DGC_DGC_LUT_4_CFG_ID));
    printf("DGC_DGC_LUT_5_CFG\tReg[%08X]=\t%08X\n", (unsigned int)DGC_DGC_LUT_5_CFG_ID, (unsigned int)dwt_read32bitreg(DGC_DGC_LUT_5_CFG_ID));
    printf("DGC_DGC_LUT_6_CFG\tReg[%08X]=\t%08X\n", (unsigned int)DGC_DGC_LUT_6_CFG_ID, (unsigned int)dwt_read32bitreg(DGC_DGC_LUT_6_CFG_ID));
    printf("DGC_MIN_METRIC_LIM_WNDW_CFG\tReg[%08X]=\t%08X\n", (unsigned int)DGC_MIN_METRIC_LIM_WNDW_CFG_ID, (unsigned int)dwt_read32bitreg(DGC_MIN_METRIC_LIM_WNDW_CFG_ID));
    printf("ADC_THRESH_DBG\tReg[%08X]=\t%08X\n", (unsigned int)ADC_THRESH_DBG_ID, (unsigned int)dwt_read32bitreg(ADC_THRESH_DBG_ID));
    printf("ADC_DBG\tReg[%08X]=\t%08X\n", (unsigned int)ADC_DBG_ID, (unsigned int)dwt_read32bitreg(ADC_DBG_ID));
    printf("ADC_DGC_DBG\tReg[%08X]=\t%08X\n", (unsigned int)ADC_DGC_DBG_ID, (unsigned int)dwt_read32bitreg(ADC_DGC_DBG_ID));
    printf("DGC_DBG\tReg[%08X]=\t%08X\n", (unsigned int)DGC_DBG_ID, (unsigned int)dwt_read32bitreg(DGC_DBG_ID));
    printf("EC_CTRL\tReg[%08X]=\t%08X\n", (unsigned int)EC_CTRL_ID, (unsigned int)dwt_read32bitreg(EC_CTRL_ID));
    printf("EC_RXTC\tReg[%08X]=\t%08X\n", (unsigned int)EC_RXTC_ID, (unsigned int)dwt_read32bitreg(EC_RXTC_ID));
    printf("EC_GOLP\tReg[%08X]=\t%08X\n", (unsigned int)EC_GOLP_ID, (unsigned int)dwt_read32bitreg(EC_GOLP_ID));
    printf("PGF_CAL_CFG\tReg[%08X]=\t%08X\n", (unsigned int)PGF_CAL_CFG_ID, (unsigned int)dwt_read32bitreg(PGF_CAL_CFG_ID));
    printf("PGF_I_CTRL0\tReg[%08X]=\t%08X\n", (unsigned int)PGF_I_CTRL0_ID, (unsigned int)dwt_read32bitreg(PGF_I_CTRL0_ID));
    printf("PGF_I_CTRL1\tReg[%08X]=\t%08X\n", (unsigned int)PGF_I_CTRL1_ID, (unsigned int)dwt_read32bitreg(PGF_I_CTRL1_ID));
    printf("PGF_Q_CTRL0\tReg[%08X]=\t%08X\n", (unsigned int)PGF_Q_CTRL0_ID, (unsigned int)dwt_read32bitreg(PGF_Q_CTRL0_ID));
    printf("PGF_Q_CTRL1\tReg[%08X]=\t%08X\n", (unsigned int)PGF_Q_CTRL1_ID, (unsigned int)dwt_read32bitreg(PGF_Q_CTRL1_ID));
    printf("PGF_CAL_STS\tReg[%08X]=\t%08X\n", (unsigned int)PGF_CAL_STS_ID, (unsigned int)dwt_read32bitreg(PGF_CAL_STS_ID));
    printf("PGF_COMP_STS\tReg[%08X]=\t%08X\n", (unsigned int)PGF_COMP_STS_ID, (unsigned int)dwt_read32bitreg(PGF_COMP_STS_ID));
    printf("MFIO_MODE\tReg[%08X]=\t%08X\n", (unsigned int)MFIO_MODE_ID, (unsigned int)dwt_read32bitreg(MFIO_MODE_ID));
    printf("GPIO_PULL_EN\tReg[%08X]=\t%08X\n", (unsigned int)GPIO_PULL_EN_ID, (unsigned int)dwt_read32bitreg(GPIO_PULL_EN_ID));
    printf("GPIO_DIR\tReg[%08X]=\t%08X\n", (unsigned int)GPIO_DIR_ID, (unsigned int)dwt_read32bitreg(GPIO_DIR_ID));
    printf("GPIO_OUT\tReg[%08X]=\t%08X\n", (unsigned int)GPIO_OUT_ID, (unsigned int)dwt_read32bitreg(GPIO_OUT_ID));
    printf("GPIO_INT_EN\tReg[%08X]=\t%08X\n", (unsigned int)GPIO_INT_EN_ID, (unsigned int)dwt_read32bitreg(GPIO_INT_EN_ID));
    printf("GPIO_INT_STATUS\tReg[%08X]=\t%08X\n", (unsigned int)GPIO_INT_STATUS_ID, (unsigned int)dwt_read32bitreg(GPIO_INT_STATUS_ID));
    printf("GPIO_INT_EDGE\tReg[%08X]=\t%08X\n", (unsigned int)GPIO_INT_EDGE_ID, (unsigned int)dwt_read32bitreg(GPIO_INT_EDGE_ID));
    printf("GPIO_INT_TYPE\tReg[%08X]=\t%08X\n", (unsigned int)GPIO_INT_TYPE_ID, (unsigned int)dwt_read32bitreg(GPIO_INT_TYPE_ID));
    printf("GPIO_INT_TYPE2\tReg[%08X]=\t%08X\n", (unsigned int)GPIO_INT_TYPE2_ID, (unsigned int)dwt_read32bitreg(GPIO_INT_TYPE2_ID));
    printf("GPIO_INT_CLR\tReg[%08X]=\t%08X\n", (unsigned int)GPIO_INT_CLR_ID, (unsigned int)dwt_read32bitreg(GPIO_INT_CLR_ID));
    printf("GPIO_DBNC_EN\tReg[%08X]=\t%08X\n", (unsigned int)GPIO_DBNC_EN_ID, (unsigned int)dwt_read32bitreg(GPIO_DBNC_EN_ID));
    printf("GPIO_RAW_DATA\tReg[%08X]=\t%08X\n", (unsigned int)GPIO_RAW_DATA_ID, (unsigned int)dwt_read32bitreg(GPIO_RAW_DATA_ID));
    printf("DTUNE0\tReg[%08X]=\t%08X\n", (unsigned int)DTUNE0_ID, (unsigned int)dwt_read32bitreg(DTUNE0_ID));
    printf("DTUNE1\tReg[%08X]=\t%08X\n", (unsigned int)DTUNE1_ID, (unsigned int)dwt_read32bitreg(DTUNE1_ID));
    printf("DTUNE2\tReg[%08X]=\t%08X\n", (unsigned int)DTUNE2_ID, (unsigned int)dwt_read32bitreg(DTUNE2_ID));
    printf("DTUNE3\tReg[%08X]=\t%08X\n", (unsigned int)DTUNE3_ID, (unsigned int)dwt_read32bitreg(DTUNE3_ID));
    printf("DTUNE4\tReg[%08X]=\t%08X\n", (unsigned int)DTUNE4_ID, (unsigned int)dwt_read32bitreg(DTUNE4_ID));
    printf("DTUNE5\tReg[%08X]=\t%08X\n", (unsigned int)DTUNE5_ID, (unsigned int)dwt_read32bitreg(DTUNE5_ID));
    printf("DIAG0\tReg[%08X]=\t%08X\n", (unsigned int)DIAG0_ID, (unsigned int)dwt_read32bitreg(DIAG0_ID));
    printf("DIAG1\tReg[%08X]=\t%08X\n", (unsigned int)DIAG1_ID, (unsigned int)dwt_read32bitreg(DIAG1_ID));
    printf("DIAG2\tReg[%08X]=\t%08X\n", (unsigned int)DIAG2_ID, (unsigned int)dwt_read32bitreg(DIAG2_ID));
    printf("DIAG3\tReg[%08X]=\t%08X\n", (unsigned int)DIAG3_ID, (unsigned int)dwt_read32bitreg(DIAG3_ID));
    printf("DIAG4\tReg[%08X]=\t%08X\n", (unsigned int)DIAG4_ID, (unsigned int)dwt_read32bitreg(DIAG4_ID));
    printf("DIAG5\tReg[%08X]=\t%08X\n", (unsigned int)DIAG5_ID, (unsigned int)dwt_read32bitreg(DIAG5_ID));
    printf("DIAG6\tReg[%08X]=\t%08X\n", (unsigned int)DIAG6_ID, (unsigned int)dwt_read32bitreg(DIAG6_ID));
    printf("DSTAT\tReg[%08X]=\t%08X\n", (unsigned int)DSTAT_ID, (unsigned int)dwt_read32bitreg(DSTAT_ID));
    printf("RF_OVR\tReg[%08X]=\t%08X\n", (unsigned int)RF_OVR_ID, (unsigned int)dwt_read32bitreg(RF_OVR_ID));
    printf("RF_CTRL_MASK\tReg[%08X]=\t%08X\n", (unsigned int)RF_CTRL_MASK_ID, (unsigned int)dwt_read32bitreg(RF_CTRL_MASK_ID));
    printf("RX_CTRL_LO\tReg[%08X]=\t%08X\n", (unsigned int)RX_CTRL_LO_ID, (unsigned int)dwt_read32bitreg(RX_CTRL_LO_ID));
    printf("RX_CTRL_MID\tReg[%08X]=\t%08X\n", (unsigned int)RX_CTRL_MID_ID, (unsigned int)dwt_read32bitreg(RX_CTRL_MID_ID));
    printf("RX_CTRL_HI\tReg[%08X]=\t%08X\n", (unsigned int)RX_CTRL_HI_ID, (unsigned int)dwt_read32bitreg(RX_CTRL_HI_ID));
    printf("PDOA_CTRL\tReg[%08X]=\t%08X\n", (unsigned int)PDOA_CTRL_ID, (unsigned int)dwt_read32bitreg(PDOA_CTRL_ID));
    printf("TX_CTRL_LO\tReg[%08X]=\t%08X\n", (unsigned int)TX_CTRL_LO_ID, (unsigned int)dwt_read32bitreg(TX_CTRL_LO_ID));
    printf("TX_CTRL_HI\tReg[%08X]=\t%08X\n", (unsigned int)TX_CTRL_HI_ID, (unsigned int)dwt_read32bitreg(TX_CTRL_HI_ID));
    printf("ED_CTRL\tReg[%08X]=\t%08X\n", (unsigned int)ED_CTRL_ID, (unsigned int)dwt_read32bitreg(ED_CTRL_ID));
    printf("RX_TEST\tReg[%08X]=\t%08X\n", (unsigned int)RX_TEST_ID, (unsigned int)dwt_read32bitreg(RX_TEST_ID));
    printf("TX_TEST\tReg[%08X]=\t%08X\n", (unsigned int)TX_TEST_ID, (unsigned int)dwt_read32bitreg(TX_TEST_ID));
    printf("SPARE_IO\tReg[%08X]=\t%08X\n", (unsigned int)SPARE_IO_ID, (unsigned int)dwt_read32bitreg(SPARE_IO_ID));
    printf("TESTMUX\tReg[%08X]=\t%08X\n", (unsigned int)TESTMUX_ID, (unsigned int)dwt_read32bitreg(TESTMUX_ID));
    printf("SAR_TEST\tReg[%08X]=\t%08X\n", (unsigned int)SAR_TEST_ID, (unsigned int)dwt_read32bitreg(SAR_TEST_ID));
    printf("PG_TST_DATA\tReg[%08X]=\t%08X\n", (unsigned int)PG_TST_DATA_ID, (unsigned int)dwt_read32bitreg(PG_TST_DATA_ID));
    printf("RF_STATUS\tReg[%08X]=\t%08X\n", (unsigned int)RF_STATUS_ID, (unsigned int)dwt_read32bitreg(RF_STATUS_ID));
    printf("LDO_TUNE_LO\tReg[%08X]=\t%08X\n", (unsigned int)LDO_TUNE_LO_ID, (unsigned int)dwt_read32bitreg(LDO_TUNE_LO_ID));
    printf("LDO_TUNE_HI\tReg[%08X]=\t%08X\n", (unsigned int)LDO_TUNE_HI_ID, (unsigned int)dwt_read32bitreg(LDO_TUNE_HI_ID));
    printf("LDO_CTRL\tReg[%08X]=\t%08X\n", (unsigned int)LDO_CTRL_ID, (unsigned int)dwt_read32bitreg(LDO_CTRL_ID));
    printf("LDO_VOUT\tReg[%08X]=\t%08X\n", (unsigned int)LDO_VOUT_ID, (unsigned int)dwt_read32bitreg(LDO_VOUT_ID));
    printf("LDO_RLOAD\tReg[%08X]=\t%08X\n", (unsigned int)LDO_RLOAD_ID, (unsigned int)dwt_read32bitreg(LDO_RLOAD_ID));
    printf("LDO_BYPASS\tReg[%08X]=\t%08X\n", (unsigned int)LDO_BYPASS_ID, (unsigned int)dwt_read32bitreg(LDO_BYPASS_ID));
    printf("LDO_DC_TST\tReg[%08X]=\t%08X\n", (unsigned int)LDO_DC_TST_ID, (unsigned int)dwt_read32bitreg(LDO_DC_TST_ID));
    printf("SAR_CTRL\tReg[%08X]=\t%08X\n", (unsigned int)SAR_CTRL_ID, (unsigned int)dwt_read32bitreg(SAR_CTRL_ID));
    printf("SAR_STATUS\tReg[%08X]=\t%08X\n", (unsigned int)SAR_STATUS_ID, (unsigned int)dwt_read32bitreg(SAR_STATUS_ID));
    printf("SAR_READING\tReg[%08X]=\t%08X\n", (unsigned int)SAR_READING_ID, (unsigned int)dwt_read32bitreg(SAR_READING_ID));
    printf("SAR_OLD_READ\tReg[%08X]=\t%08X\n", (unsigned int)SAR_OLD_READ_ID, (unsigned int)dwt_read32bitreg(SAR_OLD_READ_ID));
    printf("PGC_CTRL\tReg[%08X]=\t%08X\n", (unsigned int)PGC_CTRL_ID, (unsigned int)dwt_read32bitreg(PGC_CTRL_ID));
    printf("PGC_STATUS\tReg[%08X]=\t%08X\n", (unsigned int)PGC_STATUS_ID, (unsigned int)dwt_read32bitreg(PGC_STATUS_ID));
    printf("PG_TEST\tReg[%08X]=\t%08X\n", (unsigned int)PG_TEST_ID, (unsigned int)dwt_read32bitreg(PG_TEST_ID));
    printf("PG_CAL_TARGET\tReg[%08X]=\t%08X\n", (unsigned int)PG_CAL_TARGET_ID, (unsigned int)dwt_read32bitreg(PG_CAL_TARGET_ID));
    printf("DELTA_VBAT\tReg[%08X]=\t%08X\n", (unsigned int)DELTA_VBAT_ID, (unsigned int)dwt_read32bitreg(DELTA_VBAT_ID));
    printf("DELTA_TEMP\tReg[%08X]=\t%08X\n", (unsigned int)DELTA_TEMP_ID, (unsigned int)dwt_read32bitreg(DELTA_TEMP_ID));
    printf("DELTA_IRQ\tReg[%08X]=\t%08X\n", (unsigned int)DELTA_IRQ_ID, (unsigned int)dwt_read32bitreg(DELTA_IRQ_ID));
    printf("FOSC_CAL\tReg[%08X]=\t%08X\n", (unsigned int)FOSC_CAL_ID, (unsigned int)dwt_read32bitreg(FOSC_CAL_ID));
    printf("PLL_CFG\tReg[%08X]=\t%08X\n", (unsigned int)PLL_CFG_ID, (unsigned int)dwt_read32bitreg(PLL_CFG_ID));
    printf("PLL_COARSE_CODE\tReg[%08X]=\t%08X\n", (unsigned int)PLL_COARSE_CODE_ID, (unsigned int)dwt_read32bitreg(PLL_COARSE_CODE_ID));
    printf("PLL_CAL\tReg[%08X]=\t%08X\n", (unsigned int)PLL_CAL_ID, (unsigned int)dwt_read32bitreg(PLL_CAL_ID));
    printf("PLL_STATUS\tReg[%08X]=\t%08X\n", (unsigned int)PLL_STATUS_ID, (unsigned int)dwt_read32bitreg(PLL_STATUS_ID));
    printf("PLL_COMMON\tReg[%08X]=\t%08X\n", (unsigned int)PLL_COMMON_ID, (unsigned int)dwt_read32bitreg(PLL_COMMON_ID));
    printf("XTAL\tReg[%08X]=\t%08X\n", (unsigned int)XTAL_ID, (unsigned int)dwt_read32bitreg(XTAL_ID));
    printf("PLL_LOCK_TIME_DBG\tReg[%08X]=\t%08X\n", (unsigned int)PLL_LOCK_TIME_DBG_ID, (unsigned int)dwt_read32bitreg(PLL_LOCK_TIME_DBG_ID));
    printf("DIG_CFG\tReg[%08X]=\t%08X\n", (unsigned int)DIG_CFG_ID, (unsigned int)dwt_read32bitreg(DIG_CFG_ID));
    printf("CTRL\tReg[%08X]=\t%08X\n", (unsigned int)CTRL_ID, (unsigned int)dwt_read32bitreg(CTRL_ID));
    printf("AON_RDATA\tReg[%08X]=\t%08X\n", (unsigned int)AON_RDATA_ID, (unsigned int)dwt_read32bitreg(AON_RDATA_ID));
    printf("AON_ADDR\tReg[%08X]=\t%08X\n", (unsigned int)AON_ADDR_ID, (unsigned int)dwt_read32bitreg(AON_ADDR_ID));
    printf("AON_WDATA\tReg[%08X]=\t%08X\n", (unsigned int)AON_WDATA_ID, (unsigned int)dwt_read32bitreg(AON_WDATA_ID));
    printf("ANA_CFG\tReg[%08X]=\t%08X\n", (unsigned int)ANA_CFG_ID, (unsigned int)dwt_read32bitreg(ANA_CFG_ID));
    printf("WDATA\tReg[%08X]=\t%08X\n", (unsigned int)WDATA_ID, (unsigned int)dwt_read32bitreg(WDATA_ID));
    printf("ADDR\tReg[%08X]=\t%08X\n", (unsigned int)ADDR_ID, (unsigned int)dwt_read32bitreg(ADDR_ID));
    printf("CFG\tReg[%08X]=\t%08X\n", (unsigned int)CFG_ID, (unsigned int)dwt_read32bitreg(CFG_ID));
    printf("STATUS\tReg[%08X]=\t%08X\n", (unsigned int)STATUS_ID, (unsigned int)dwt_read32bitreg(STATUS_ID));
    printf("RDATA\tReg[%08X]=\t%08X\n", (unsigned int)RDATA_ID, (unsigned int)dwt_read32bitreg(RDATA_ID));
    printf("SRDATA\tReg[%08X]=\t%08X\n", (unsigned int)SRDATA_ID, (unsigned int)dwt_read32bitreg(SRDATA_ID));
    printf("IP_TOA_LO\tReg[%08X]=\t%08X\n", (unsigned int)IP_TOA_LO_ID, (unsigned int)dwt_read32bitreg(IP_TOA_LO_ID));
    printf("IP_TOA_HI\tReg[%08X]=\t%08X\n", (unsigned int)IP_TOA_HI_ID, (unsigned int)dwt_read32bitreg(IP_TOA_HI_ID));
    printf("CY0_TOA_LO\tReg[%08X]=\t%08X\n", (unsigned int)CY0_TOA_LO_ID, (unsigned int)dwt_read32bitreg(CY0_TOA_LO_ID));
    printf("CY0_TOA_HI\tReg[%08X]=\t%08X\n", (unsigned int)CY0_TOA_HI_ID, (unsigned int)dwt_read32bitreg(CY0_TOA_HI_ID));
    printf("CY1_TOA_LO\tReg[%08X]=\t%08X\n", (unsigned int)CY1_TOA_LO_ID, (unsigned int)dwt_read32bitreg(CY1_TOA_LO_ID));
    printf("CY1_TOA_HI\tReg[%08X]=\t%08X\n", (unsigned int)CY1_TOA_HI_ID, (unsigned int)dwt_read32bitreg(CY1_TOA_HI_ID));
    printf("CIA_TDOA_0\tReg[%08X]=\t%08X\n", (unsigned int)CIA_TDOA_0_ID, (unsigned int)dwt_read32bitreg(CIA_TDOA_0_ID));
    printf("CIA_TDOA_1_PDOA\tReg[%08X]=\t%08X\n", (unsigned int)CIA_TDOA_1_PDOA_ID, (unsigned int)dwt_read32bitreg(CIA_TDOA_1_PDOA_ID));
    printf("CIA_DIAG_0\tReg[%08X]=\t%08X\n", (unsigned int)CIA_DIAG_0_ID, (unsigned int)dwt_read32bitreg(CIA_DIAG_0_ID));
    printf("CIA_DIAG_1\tReg[%08X]=\t%08X\n", (unsigned int)CIA_DIAG_1_ID, (unsigned int)dwt_read32bitreg(CIA_DIAG_1_ID));
    printf("IP_DIAG_0\tReg[%08X]=\t%08X\n", (unsigned int)IP_DIAG_0_ID, (unsigned int)dwt_read32bitreg(IP_DIAG_0_ID));
    printf("IP_DIAG_1\tReg[%08X]=\t%08X\n", (unsigned int)IP_DIAG_1_ID, (unsigned int)dwt_read32bitreg(IP_DIAG_1_ID));
    printf("IP_DIAG_2\tReg[%08X]=\t%08X\n", (unsigned int)IP_DIAG_2_ID, (unsigned int)dwt_read32bitreg(IP_DIAG_2_ID));
    printf("IP_DIAG_3\tReg[%08X]=\t%08X\n", (unsigned int)IP_DIAG_3_ID, (unsigned int)dwt_read32bitreg(IP_DIAG_3_ID));
    printf("IP_DIAG_4\tReg[%08X]=\t%08X\n", (unsigned int)IP_DIAG_4_ID, (unsigned int)dwt_read32bitreg(IP_DIAG_4_ID));
    printf("IP_DIAG_5\tReg[%08X]=\t%08X\n", (unsigned int)IP_DIAG_5_ID, (unsigned int)dwt_read32bitreg(IP_DIAG_5_ID));
    printf("IP_DIAG_6\tReg[%08X]=\t%08X\n", (unsigned int)IP_DIAG_6_ID, (unsigned int)dwt_read32bitreg(IP_DIAG_6_ID));
    printf("IP_DIAG_7\tReg[%08X]=\t%08X\n", (unsigned int)IP_DIAG_7_ID, (unsigned int)dwt_read32bitreg(IP_DIAG_7_ID));
    printf("IP_DIAG_8\tReg[%08X]=\t%08X\n", (unsigned int)IP_DIAG_8_ID, (unsigned int)dwt_read32bitreg(IP_DIAG_8_ID));
    printf("IP_DIAG_9\tReg[%08X]=\t%08X\n", (unsigned int)IP_DIAG_9_ID, (unsigned int)dwt_read32bitreg(IP_DIAG_9_ID));
    printf("IP_DIAG_10\tReg[%08X]=\t%08X\n", (unsigned int)IP_DIAG_10_ID, (unsigned int)dwt_read32bitreg(IP_DIAG_10_ID));
    printf("IP_DIAG_11\tReg[%08X]=\t%08X\n", (unsigned int)IP_DIAG_11_ID, (unsigned int)dwt_read32bitreg(IP_DIAG_11_ID));
    printf("IP_DIAG_12\tReg[%08X]=\t%08X\n", (unsigned int)IP_DIAG_12_ID, (unsigned int)dwt_read32bitreg(IP_DIAG_12_ID));
    printf("CY0_DIAG_0\tReg[%08X]=\t%08X\n", (unsigned int)CY0_DIAG_0_ID, (unsigned int)dwt_read32bitreg(CY0_DIAG_0_ID));
    printf("CY0_DIAG_1\tReg[%08X]=\t%08X\n", (unsigned int)CY0_DIAG_1_ID, (unsigned int)dwt_read32bitreg(CY0_DIAG_1_ID));
    printf("CY0_DIAG_2\tReg[%08X]=\t%08X\n", (unsigned int)CY0_DIAG_2_ID, (unsigned int)dwt_read32bitreg(CY0_DIAG_2_ID));
    printf("CY0_DIAG_3\tReg[%08X]=\t%08X\n", (unsigned int)CY0_DIAG_3_ID, (unsigned int)dwt_read32bitreg(CY0_DIAG_3_ID));
    printf("CY0_DIAG_4\tReg[%08X]=\t%08X\n", (unsigned int)CY0_DIAG_4_ID, (unsigned int)dwt_read32bitreg(CY0_DIAG_4_ID));
    printf("CY0_DIAG_5\tReg[%08X]=\t%08X\n", (unsigned int)CY0_DIAG_5_ID, (unsigned int)dwt_read32bitreg(CY0_DIAG_5_ID));
    printf("CY0_DIAG_6\tReg[%08X]=\t%08X\n", (unsigned int)CY0_DIAG_6_ID, (unsigned int)dwt_read32bitreg(CY0_DIAG_6_ID));
    printf("CY0_DIAG_7\tReg[%08X]=\t%08X\n", (unsigned int)CY0_DIAG_7_ID, (unsigned int)dwt_read32bitreg(CY0_DIAG_7_ID));
    printf("CY0_DIAG_8\tReg[%08X]=\t%08X\n", (unsigned int)CY0_DIAG_8_ID, (unsigned int)dwt_read32bitreg(CY0_DIAG_8_ID));
    printf("CY0_DIAG_9\tReg[%08X]=\t%08X\n", (unsigned int)CY0_DIAG_9_ID, (unsigned int)dwt_read32bitreg(CY0_DIAG_9_ID));
    printf("CY0_DIAG_10\tReg[%08X]=\t%08X\n", (unsigned int)CY0_DIAG_10_ID, (unsigned int)dwt_read32bitreg(CY0_DIAG_10_ID));
    printf("CY0_DIAG_11\tReg[%08X]=\t%08X\n", (unsigned int)CY0_DIAG_11_ID, (unsigned int)dwt_read32bitreg(CY0_DIAG_11_ID));
    printf("CY0_DIAG_12\tReg[%08X]=\t%08X\n", (unsigned int)CY0_DIAG_12_ID, (unsigned int)dwt_read32bitreg(CY0_DIAG_12_ID));
    printf("CY0_DIAG_13\tReg[%08X]=\t%08X\n", (unsigned int)CY0_DIAG_13_ID, (unsigned int)dwt_read32bitreg(CY0_DIAG_13_ID));
    printf("CY0_DIAG_14\tReg[%08X]=\t%08X\n", (unsigned int)CY0_DIAG_14_ID, (unsigned int)dwt_read32bitreg(CY0_DIAG_14_ID));
    printf("CY0_DIAG_15\tReg[%08X]=\t%08X\n", (unsigned int)CY0_DIAG_15_ID, (unsigned int)dwt_read32bitreg(CY0_DIAG_15_ID));
    printf("CY0_DIAG_16\tReg[%08X]=\t%08X\n", (unsigned int)CY0_DIAG_16_ID, (unsigned int)dwt_read32bitreg(CY0_DIAG_16_ID));
    printf("CY0_DIAG_17\tReg[%08X]=\t%08X\n", (unsigned int)CY0_DIAG_17_ID, (unsigned int)dwt_read32bitreg(CY0_DIAG_17_ID));
    printf("CY1_DIAG_0\tReg[%08X]=\t%08X\n", (unsigned int)CY1_DIAG_0_ID, (unsigned int)dwt_read32bitreg(CY1_DIAG_0_ID));
    printf("CY1_DIAG_1\tReg[%08X]=\t%08X\n", (unsigned int)CY1_DIAG_1_ID, (unsigned int)dwt_read32bitreg(CY1_DIAG_1_ID));
    printf("CY1_DIAG_2\tReg[%08X]=\t%08X\n", (unsigned int)CY1_DIAG_2_ID, (unsigned int)dwt_read32bitreg(CY1_DIAG_2_ID));
    printf("CY1_DIAG_3\tReg[%08X]=\t%08X\n", (unsigned int)CY1_DIAG_3_ID, (unsigned int)dwt_read32bitreg(CY1_DIAG_3_ID));
    printf("CY1_DIAG_4\tReg[%08X]=\t%08X\n", (unsigned int)CY1_DIAG_4_ID, (unsigned int)dwt_read32bitreg(CY1_DIAG_4_ID));
    printf("CY1_DIAG_5\tReg[%08X]=\t%08X\n", (unsigned int)CY1_DIAG_5_ID, (unsigned int)dwt_read32bitreg(CY1_DIAG_5_ID));
    printf("CY1_DIAG_6\tReg[%08X]=\t%08X\n", (unsigned int)CY1_DIAG_6_ID, (unsigned int)dwt_read32bitreg(CY1_DIAG_6_ID));
    printf("CY1_DIAG_7\tReg[%08X]=\t%08X\n", (unsigned int)CY1_DIAG_7_ID, (unsigned int)dwt_read32bitreg(CY1_DIAG_7_ID));
    printf("CY1_DIAG_8\tReg[%08X]=\t%08X\n", (unsigned int)CY1_DIAG_8_ID, (unsigned int)dwt_read32bitreg(CY1_DIAG_8_ID));
    printf("CY1_DIAG_9\tReg[%08X]=\t%08X\n", (unsigned int)CY1_DIAG_9_ID, (unsigned int)dwt_read32bitreg(CY1_DIAG_9_ID));
    printf("CY1_DIAG_10\tReg[%08X]=\t%08X\n", (unsigned int)CY1_DIAG_10_ID, (unsigned int)dwt_read32bitreg(CY1_DIAG_10_ID));
    printf("CY1_DIAG_11\tReg[%08X]=\t%08X\n", (unsigned int)CY1_DIAG_11_ID, (unsigned int)dwt_read32bitreg(CY1_DIAG_11_ID));
    printf("CY1_DIAG_12\tReg[%08X]=\t%08X\n", (unsigned int)CY1_DIAG_12_ID, (unsigned int)dwt_read32bitreg(CY1_DIAG_12_ID));
    printf("RX_ANTENNA_DELAY\tReg[%08X]=\t%08X\n", (unsigned int)RX_ANTENNA_DELAY_ID, (unsigned int)dwt_read32bitreg(RX_ANTENNA_DELAY_ID));
    printf("FP_CONFIDENCE_LIMIT\tReg[%08X]=\t%08X\n", (unsigned int)FP_CONFIDENCE_LIMIT_ID, (unsigned int)dwt_read32bitreg(FP_CONFIDENCE_LIMIT_ID));
    printf("IP_CONFIG_LO\tReg[%08X]=\t%08X\n", (unsigned int)IP_CONFIG_LO_ID, (unsigned int)dwt_read32bitreg(IP_CONFIG_LO_ID));
    printf("IP_CONFIG_HI\tReg[%08X]=\t%08X\n", (unsigned int)IP_CONFIG_HI_ID, (unsigned int)dwt_read32bitreg(IP_CONFIG_HI_ID));
    printf("CY_CONFIG_LO\tReg[%08X]=\t%08X\n", (unsigned int)CY_CONFIG_LO_ID, (unsigned int)dwt_read32bitreg(CY_CONFIG_LO_ID));
    printf("CY_CONFIG_HI\tReg[%08X]=\t%08X\n", (unsigned int)CY_CONFIG_HI_ID, (unsigned int)dwt_read32bitreg(CY_CONFIG_HI_ID));
    printf("PGF_DELAY_COMP_LO\tReg[%08X]=\t%08X\n", (unsigned int)PGF_DELAY_COMP_LO_ID, (unsigned int)dwt_read32bitreg(PGF_DELAY_COMP_LO_ID));
    printf("PGF_DELAY_COMP_HI\tReg[%08X]=\t%08X\n", (unsigned int)PGF_DELAY_COMP_HI_ID, (unsigned int)dwt_read32bitreg(PGF_DELAY_COMP_HI_ID));
    printf("EVENT_CTRL\tReg[%08X]=\t%08X\n", (unsigned int)EVENT_CTRL_ID, (unsigned int)dwt_read32bitreg(EVENT_CTRL_ID));
    printf("EVENT_COUNT0\tReg[%08X]=\t%08X\n", (unsigned int)EVENT_COUNT0_ID, (unsigned int)dwt_read32bitreg(EVENT_COUNT0_ID));
    printf("EVENT_COUNT1\tReg[%08X]=\t%08X\n", (unsigned int)EVENT_COUNT1_ID, (unsigned int)dwt_read32bitreg(EVENT_COUNT1_ID));
    printf("EVENT_COUNT2\tReg[%08X]=\t%08X\n", (unsigned int)EVENT_COUNT2_ID, (unsigned int)dwt_read32bitreg(EVENT_COUNT2_ID));
    printf("EVENT_COUNT3\tReg[%08X]=\t%08X\n", (unsigned int)EVENT_COUNT3_ID, (unsigned int)dwt_read32bitreg(EVENT_COUNT3_ID));
    printf("EVENT_COUNT4\tReg[%08X]=\t%08X\n", (unsigned int)EVENT_COUNT4_ID, (unsigned int)dwt_read32bitreg(EVENT_COUNT4_ID));
    printf("EVENT_COUNT5\tReg[%08X]=\t%08X\n", (unsigned int)EVENT_COUNT5_ID, (unsigned int)dwt_read32bitreg(EVENT_COUNT5_ID));
    printf("EVENT_COUNT6\tReg[%08X]=\t%08X\n", (unsigned int)EVENT_COUNT6_ID, (unsigned int)dwt_read32bitreg(EVENT_COUNT6_ID));
    printf("ADC_MEM_PTR\tReg[%08X]=\t%08X\n", (unsigned int)ADC_MEM_PTR_ID, (unsigned int)dwt_read32bitreg(ADC_MEM_PTR_ID));
    printf("TEST_CTRL0\tReg[%08X]=\t%08X\n", (unsigned int)TEST_CTRL0_ID, (unsigned int)dwt_read32bitreg(TEST_CTRL0_ID));
    printf("EVENT_COUNT7\tReg[%08X]=\t%08X\n", (unsigned int)EVENT_COUNT7_ID, (unsigned int)dwt_read32bitreg(EVENT_COUNT7_ID));
    printf("SPI_MODE\tReg[%08X]=\t%08X\n", (unsigned int)SPI_MODE_ID, (unsigned int)dwt_read32bitreg(SPI_MODE_ID));
    printf("SYS_STATE_LO\tReg[%08X]=\t%08X\n", (unsigned int)SYS_STATE_LO_ID, (unsigned int)dwt_read32bitreg(SYS_STATE_LO_ID));
    printf("BIST_CTRL\tReg[%08X]=\t%08X\n", (unsigned int)BIST_CTRL_ID, (unsigned int)dwt_read32bitreg(BIST_CTRL_ID));
    printf("TST_DBG\tReg[%08X]=\t%08X\n", (unsigned int)TST_DBG_ID, (unsigned int)dwt_read32bitreg(TST_DBG_ID));
    printf("FCMD_STATUS\tReg[%08X]=\t%08X\n", (unsigned int)FCMD_STATUS_ID, (unsigned int)dwt_read32bitreg(FCMD_STATUS_ID));
    printf("TEST_LOGGING\tReg[%08X]=\t%08X\n", (unsigned int)TEST_LOGGING_ID, (unsigned int)dwt_read32bitreg(TEST_LOGGING_ID));
    printf("STATUS_LOGGING\tReg[%08X]=\t%08X\n", (unsigned int)STATUS_LOGGING_ID, (unsigned int)dwt_read32bitreg(STATUS_LOGGING_ID));
    printf("CR0\tReg[%08X]=\t%08X\n", (unsigned int)CR0_ID, (unsigned int)dwt_read32bitreg(CR0_ID));
    printf("CR1\tReg[%08X]=\t%08X\n", (unsigned int)CR1_ID, (unsigned int)dwt_read32bitreg(CR1_ID));
    printf("CR2\tReg[%08X]=\t%08X\n", (unsigned int)CR2_ID, (unsigned int)dwt_read32bitreg(CR2_ID));
    printf("CR3\tReg[%08X]=\t%08X\n", (unsigned int)CR3_ID, (unsigned int)dwt_read32bitreg(CR3_ID));
    printf("CR4\tReg[%08X]=\t%08X\n", (unsigned int)CR4_ID, (unsigned int)dwt_read32bitreg(CR4_ID));
    printf("CR5\tReg[%08X]=\t%08X\n", (unsigned int)CR5_ID, (unsigned int)dwt_read32bitreg(CR5_ID));
    printf("CR6\tReg[%08X]=\t%08X\n", (unsigned int)CR6_ID, (unsigned int)dwt_read32bitreg(CR6_ID));
    printf("CR7\tReg[%08X]=\t%08X\n", (unsigned int)CR7_ID, (unsigned int)dwt_read32bitreg(CR7_ID));
    printf("CR8\tReg[%08X]=\t%08X\n", (unsigned int)CR8_ID, (unsigned int)dwt_read32bitreg(CR8_ID));
    printf("CR9\tReg[%08X]=\t%08X\n", (unsigned int)CR9_ID, (unsigned int)dwt_read32bitreg(CR9_ID));
    printf("CR10\tReg[%08X]=\t%08X\n", (unsigned int)CR10_ID, (unsigned int)dwt_read32bitreg(CR10_ID));
    printf("SOFT_RST\tReg[%08X]=\t%08X\n", (unsigned int)SOFT_RST_ID, (unsigned int)dwt_read32bitreg(SOFT_RST_ID));
    printf("CLK_CTRL\tReg[%08X]=\t%08X\n", (unsigned int)CLK_CTRL_ID, (unsigned int)dwt_read32bitreg(CLK_CTRL_ID));
    printf("SEQ_CTRL\tReg[%08X]=\t%08X\n", (unsigned int)SEQ_CTRL_ID, (unsigned int)dwt_read32bitreg(SEQ_CTRL_ID));
    printf("SNOOZE_CNT\tReg[%08X]=\t%08X\n", (unsigned int)SNOOZE_CNT_ID, (unsigned int)dwt_read32bitreg(SNOOZE_CNT_ID));
    printf("PWR_UP_TIMES_LO\tReg[%08X]=\t%08X\n", (unsigned int)PWR_UP_TIMES_LO_ID, (unsigned int)dwt_read32bitreg(PWR_UP_TIMES_LO_ID));
    printf("PWR_UP_TIMES_HI\tReg[%08X]=\t%08X\n", (unsigned int)PWR_UP_TIMES_HI_ID, (unsigned int)dwt_read32bitreg(PWR_UP_TIMES_HI_ID));
    printf("LED_CTRL\tReg[%08X]=\t%08X\n", (unsigned int)LED_CTRL_ID, (unsigned int)dwt_read32bitreg(LED_CTRL_ID));
    printf("RX_PPM\tReg[%08X]=\t%08X\n", (unsigned int)RX_PPM_ID, (unsigned int)dwt_read32bitreg(RX_PPM_ID));
    printf("FOSC_CTRL\tReg[%08X]=\t%08X\n", (unsigned int)FOSC_CTRL_ID, (unsigned int)dwt_read32bitreg(FOSC_CTRL_ID));
    printf("BIAS_CTRL\tReg[%08X]=\t%08X\n", (unsigned int)BIAS_CTRL_ID, (unsigned int)dwt_read32bitreg(BIAS_CTRL_ID));
    printf("INDIRECT_ADDR_A\tReg[%08X]=\t%08X\n", (unsigned int)INDIRECT_ADDR_A_ID, (unsigned int)dwt_read32bitreg(INDIRECT_ADDR_A_ID));
    printf("ADDR_OFFSET_A\tReg[%08X]=\t%08X\n", (unsigned int)ADDR_OFFSET_A_ID, (unsigned int)dwt_read32bitreg(ADDR_OFFSET_A_ID));
    printf("INDIRECT_ADDR_B\tReg[%08X]=\t%08X\n", (unsigned int)INDIRECT_ADDR_B_ID, (unsigned int)dwt_read32bitreg(INDIRECT_ADDR_B_ID));
    printf("ADDR_OFFSET_B\tReg[%08X]=\t%08X\n", (unsigned int)ADDR_OFFSET_B_ID, (unsigned int)dwt_read32bitreg(ADDR_OFFSET_B_ID));
}

#endif
/*
 * Code used for printing out accumulator data
 */
#define ACCLOC   (1024 + 512 + 512)
#define NUMBYTES (((ACCLOC)*6) + 1)

void print_acc_data(void)
{
    uint8_t buff[NUMBYTES];
    int32_t real[ACCLOC];
    int32_t imag[ACCLOC];
    int32_t temp;
    uint16_t i, y;

    dwt_readaccdata(buff, NUMBYTES, 0);

    printf("Processing Accumulator\n");

    y = 0;
    printf("Processing %d bytes to %d locations\n", NUMBYTES, ACCLOC);
    // Skip byte[0] as its a dummy byte
    for (i = 1; i < NUMBYTES; i = i + 6)
    {

        temp = (int32_t)((buff[i + 2]) << 16) + (int32_t)((buff[i + 1]) << 8) + (int32_t)(buff[i]);
        if (temp >= 0x20000)
            real[y] = temp - 0x1000000;
        else
            real[y] = temp;

        temp = (int32_t)((buff[i + 5]) << 16) + (int32_t)((buff[i + 4]) << 8) + (int32_t)(buff[i + 3]);
        if (temp >= 0x20000)
            imag[y] = temp - 0x1000000;
        else
            imag[y] = temp;
        y++;
    }

    for (i = 0; i < ACCLOC; i = i + 1)
    {
        printf("%d,%d\n", (int)real[i], (int)imag[i]);
    }
}

/*
 * Code used to print out information at end of test.
 */
void end_of_test_info(dwt_config_t config_options, uint32_t rangeCount, double *distance_array, uint32_t *errors)
{
    int successful_range_count = 0;
    uint32_t i = 0;
    float percentage_successful = 0;
    double sum, average_range, std_dev = 0;

    printf("Ranging Test.\n");

    switch (config_options.dataRate)
    {
    case DWT_BR_850K:
        printf("Data Rate: 850kpbs\n");
        break;
    case DWT_BR_6M8:
        printf("Data Rate: 6.8Mbps\n");
        break;
    default:
        break;
    }

    switch (config_options.txPreambLength)
    {
    case DWT_PLEN_64:
        printf("Preamble Length: 64\n");
        break;
    case DWT_PLEN_128:
        printf("Preamble Length: 128\n");
        break;
    case DWT_PLEN_512:
        printf("Preamble Length: 512\n");
        break;
    case DWT_PLEN_1024:
        printf("Preamble Length: 1024\n");
        break;
    default:
        break;
    }

    printf("Preamble Code: %d\n", config_options.txCode);

    printf("SFD Type: 4z 8-bit\n");

    printf("PAC: 8\n");

    printf("PHR Mode: Standard\n");

    printf("CP Mode: M1_4Z\n");

    printf("CP Length: 64\n");

    printf("CP PRF: 64 MHz\n");

    printf("PDOA Mode: Off\n");

    printf("Channel: %d\n", config_options.chan);

#ifdef CONFIG_MAX_TX_POWER
    printf("TX Power Setting: Max\n");
#endif

#ifdef CONFIG_NOM_TX_POWER
    printf("TX Power Setting: Nominal\n");
#endif

    printf("Ranges Attempted: %d\n", (int)rangeCount);

    for (i = 0; i < rangeCount; i++)
    {
        if (distance_array[i] != 0)
        {
            successful_range_count++;
            sum += distance_array[i];
        }
    }

    printf("Ranges Successful: %d\n", successful_range_count);

    percentage_successful = (float)successful_range_count / rangeCount * 100.0;

    printf("Percentage Successful: %.2f%%\n", percentage_successful);

    average_range = sum / successful_range_count;

    printf("Average Range: %.8f\n", average_range);

    for (i = 0; i < rangeCount; i++)
    {
        if (distance_array[i] != 0)
        {
            std_dev += pow(distance_array[i] - average_range, 2);
        }
    }

    printf("Stdev Range: %.8f\n", sqrt(std_dev / successful_range_count));

    printf("CRC Errors: %d\n", (int)errors[CRC_ERR_IDX]);

    printf("RSE Errors: %d\n", (int)errors[RSE_ERR_IDX]);

    printf("PHE Errors: %d\n", (int)errors[PHE_ERR_IDX]);

    printf("SFDTO Errors: %d\n", (int)errors[SFDTO_ERR_IDX]);

    printf("PTO Errors: %d\n", (int)errors[PTO_ERR_IDX]);

    printf("RTO Errors: %d\n", (int)errors[RTO_ERR_IDX]);

    printf("SPICRC Errors: %d\n", (int)errors[SPICRC_ERR_IDX]);

    printf("TXTO Errors: %d\n", (int)errors[TXTO_ERR_IDX]);

    printf("ARFE Errors: %d\n", (int)errors[ARFE_ERR_IDX]);

    printf("TS MISMATCH Errors: %d\n", (int)errors[TS_MISMATCH_ERR_IDX]);

    printf("BAD FRAME Errors: %d\n", (int)errors[BAD_FRAME_ERR_IDX]);

    printf("PREAMBLE COUNT Errors: %d\n", (int)errors[PREAMBLE_COUNT_ERR_IDX]);

    printf("CP QUAL Errors: %d\n", (int)errors[CP_QUAL_ERR_IDX]);
}
