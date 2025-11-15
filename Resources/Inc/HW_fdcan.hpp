#ifndef _HW_FDCAN_H_
#define _HW_FDCAN_H_
/* ------------------------------ Include ------------------------------ */
#include "stm32h723xx.h"
#include "stm32h7xx.h"
#include "system_user.hpp"

#include "fdcan.h"
/* ------------------------------ Macro Definition
 * ------------------------------ */

/* ------------------------------ Type Definition ------------------------------
 */

/* ------------------------------ Extern Global Variable
 * ------------------------------ */

/* ------------------------------ Function Declaration (used in other .c files)
 * ------------------------------ */

void FdcanFilter_Init(FDCAN_HandleTypeDef *hfdcan);

void FDCAN_Send_Msg(FDCAN_HandleTypeDef *hfdcan, uint8_t *msg, uint32_t id,
                    uint8_t len);

#endif
