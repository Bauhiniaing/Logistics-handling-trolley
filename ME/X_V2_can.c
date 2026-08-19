#include "X_V2_can.h"

////////////////////////X_V2 闭环步进电机 CAN 控制库///////////////////
///////////////////////Emm固件和X固件均可使用/////////////////////////
////////////////////////////////////////////////////////////////////
__IO CAN_t can = {0};
static CAN_HandleTypeDef *x42_hcan = &hcan1;//在不调用初始化选择函数时，也会有一个默认的通道can1

/**
* @brief   在初始化的时候调用可以改变步进电机的通信通道
	* @param   无
	* @retval  无
	*/
void X_V2_SelectCAN(CAN_HandleTypeDef *hcan)
{
	if((hcan == &hcan1) || (hcan == &hcan2))
	{
		x42_hcan = hcan;
	}
}

/**
	* @brief   CAN发送多个字节
	* @param   无
	* @retval  无
	*/
void can_SendCmd(__IO uint8_t *cmd, uint8_t len)
{
	static uint32_t TxMailbox; __IO uint8_t i = 0, j = 0, k = 0, l = 0, packNum = 0;
	CAN_HandleTypeDef *send_hcan = x42_hcan;

	// 除去ID地址和功能码后的数据长度
	j = len - 2;

	// 发送数据
	while(i < j)
	{
		// 数据个数
		k = j - i;

		// 填充缓存
		can.CAN_TxMsg.StdId = 0x00;
		can.CAN_TxMsg.ExtId = ((uint32_t)cmd[0] << 8) | (uint32_t)packNum;
		can.txData[0] = cmd[1];
		can.CAN_TxMsg.IDE = CAN_ID_EXT;
		can.CAN_TxMsg.RTR = CAN_RTR_DATA;

		// 小于8字节命令
		if(k < 8)
		{
			for(l=0; l < k; l++,i++) { can.txData[l + 1] = cmd[i + 2]; } can.CAN_TxMsg.DLC = k + 1;
		}
		// 大于8字节命令，分包发送，每包数据最多发送8个字节
		else
		{
			for(l=0; l < 7; l++,i++) { can.txData[l + 1] = cmd[i + 2]; } can.CAN_TxMsg.DLC = 8;
		}

		// 发送数据
		while(HAL_CAN_AddTxMessage(send_hcan, (CAN_TxHeaderTypeDef *)(&can.CAN_TxMsg), (uint8_t *)(&can.txData), (&TxMailbox)) != HAL_OK);

		// 记录发送的第几包的数据
		++packNum;
	}
}

