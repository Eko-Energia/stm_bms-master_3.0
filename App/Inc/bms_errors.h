#ifndef BMS_ERRORS_H
#define BMS_ERRORS_H

/*
 * Error codes for BMSMaster_NODE. Codes 1 and 2 come from the team's CSV
 * registry; 3 to 11 are allocated here and must be added to it. Severity comes
 * from errorSeverity_e in error_handler.h - no fault here uses severity 0,
 * which would command vehicle-wide safe state.
 *
 * Error_Specific_Data layouts are documented in docs/firmwareSpec.md
 * section 10.
 */
#define BMS_ERR_TEMP_HIGH           (1u)  /* on-board NTC above 60 degC       */
#define BMS_ERR_CAN2_TEMP_HIGH      (2u)  /* a module thermistor above 60 degC */
#define BMS_ERR_CAN2_MODULE_SILENT  (3u)  /* PCBCells module missed 3 periods  */
#define BMS_ERR_JK_COMMS_TIMEOUT    (4u)  /* no valid JK response             */
#define BMS_ERR_JK_FRAME_INVALID    (5u)  /* JK frame failed validation       */
#define BMS_ERR_PACK_VOLT_RANGE     (6u)  /* pack voltage outside 63-87 V     */
#define BMS_ERR_PACK_CURRENT_HIGH   (7u)  /* current magnitude above 300 A    */
#define BMS_ERR_TEMP_SENSOR_FAULT   (8u)  /* on-board NTC open or shorted     */
#define BMS_ERR_CAN1_TX_FAIL        (9u)  /* TX mailboxes blocked             */
#define BMS_ERR_FATAL_INIT          (10u) /* peripheral init failed; Error_Handler reached */
#define BMS_ERR_ADC_STALLED         (11u) /* no completed ADC scan for 100 ms  */
#define BMS_ERR_CAN2_THERM_SATURATED (12u) /* a thermistor pinned at 0 or 100 degC */

#endif /* BMS_ERRORS_H */
