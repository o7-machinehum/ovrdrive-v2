/* scsi_tables.h - SCSI response data tables
 *
 * Fixed response arrays for INQUIRY, READ FORMAT CAPACITIES,
 * READ CAPACITY, and MODE SENSE commands.
 *
 * [ref/SCSI_Block_Commands_SBC3_r25.pdf]
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef SCSI_TABLES_H_
#define SCSI_TABLES_H_

#include "bot_state.h"

extern uint8_t        g_inquiry_response[];
extern uint8_t const  g_format_capacity_response[];
extern uint8_t const  g_read_capacity_response[];
extern uint8_t const  g_mode_sense6_response[];
extern uint8_t const  g_mode_sense10_response[];

#endif /* SCSI_TABLES_H_ */
