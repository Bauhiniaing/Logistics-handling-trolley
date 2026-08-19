#ifndef __X_V2_CAN_H
#define __X_V2_CAN_H

#include "can.h"
#include "stdbool.h"

typedef struct {
	__IO CAN_RxHeaderTypeDef CAN_RxMsg;
	__IO uint8_t rxData[32];
	
	__IO CAN_TxHeaderTypeDef CAN_TxMsg;
	__IO uint8_t txData[32];

	__IO bool rxFrameFlag;
}CAN_t;

extern __IO CAN_t can;

void X_V2_SelectCAN(CAN_HandleTypeDef *hcan);
void can_SendCmd(__IO uint8_t *cmd, uint8_t len);

#endif

