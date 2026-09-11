#ifndef APP_CAN_H
#define APP_CAN_H
#include "can_driver.h"
#include "error_handler.h"

/**
 * @brief Bring up both buses and register the 20 application TX frames.
 *        Leaves frame 128 to EH_init, which registers it itself.
 *        Drives both transceivers out of standby before starting either bus.
 */
void CAN_App_Init(CAN_HandleTypeDef *hcan1, CAN_HandleTypeDef *hcan2, EH_HandleTypeDef *eh);

/** @brief Service the CAN1 transmit scheduler. Call last in the loop. */
void CAN_App_Task(void);

/** @brief CAN1 RX FIFO0: SafeState IDs 1 and 3. ISR context. */
void CAN_App_OnRx1(CAN_HandleTypeDef *hcan);

/** @brief CAN2 RX FIFO0: PCBCells thermistors. ISR context. */
void CAN_App_OnRx2(CAN_HandleTypeDef *hcan);

/** @brief The CAN1 scheduler, so EH_init can add the node frame to it. */
struct CAN_scheduledMsgList *CAN_App_Scheduler(void);

#endif /* APP_CAN_H */
