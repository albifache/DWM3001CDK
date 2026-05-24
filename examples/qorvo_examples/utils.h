/*! ----------------------------------------------------------------------------
 * @file    utils.h
 * @brief   Various utility functions are kept here. Functions are not to be released to customers.
 *
 * @author Decawave
 *
 * @copyright SPDX-FileCopyrightText: Copyright (c) 2024 Qorvo US, Inc.
 *            SPDX-License-Identifier: LicenseRef-QORVO-2
 *
 */

#ifndef UTILS_H_
#define UTILS_H_

void dump_registers(void);
void print_acc_data(void);
void end_of_test_info(dwt_config_t config_options, uint32_t rangeCount, double *distance_array, uint32_t *errors);

#endif /* UTILS_H_ */
