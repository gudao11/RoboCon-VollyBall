#include "motor_can.h"
#include "can.h"
#include "main.h"
#include "pid_tim.h"
#include "stm32f4xx_hal_can.h"
#include <stdint.h>
#include <sys/_intsup.h>
motor_info_t C620_1,C620_2,C620_3,C620_4;
motor_info_t C6xx_1,C6xx_2,C6xx_3,C6xx_4;
#define CanRxGetU16(canRxMsg, num) (((uint16_t)canRxMsg.Data[num * 2] << 8) | (uint16_t)canRxMsg.Data[num * 2 + 1])
HAL_CAN_RxMsgTypedef can1RxMsg,can2RxMsg; //接受消息结构体
uint8_t can1RxData[8],can2RxData[8];     //接受数据缓存
uint8_t isRcan1Started=0,isRcan2Started=0; //标志位，表示 CAN1 和 CAN2 是否已启动接收
CAN_DATA_t sendData[4], receiveData[4];
uint8_t maxSendSize = 4;  //定义了发送和接收数据的数组,并设置最大发送大小
uint8_t can1_update = 1;
uint8_t can2_update = 1; //标志位，表示 CAN1 和 CAN2 是否有新的数据需要发送
extern uint16_t PID_Calc_Flag;

/********************CAN发送*****************************/
//CAN数据标记发送，保证发送资源正常
void HAL_CAN_TxCpltCallback(CAN_HandleTypeDef* hcan){
	if(hcan == &hcan1){
		can1_update = 1;
	}
//	else if(hcan == &hcan2){
		can2_update = 1;
	}
}

/*滤波器配置及can初始化*/
void can1_filter_init(void)
{ 	
	CAN_FilterTypeDef can1_filter_structure;
	can1_filter_structure.FilterActivation = ENABLE;//使能滤波器
	can1_filter_structure.FilterMode = CAN_FILTERMODE_IDMASK;//掩码模式
	can1_filter_structure.FilterScale = CAN_FILTERSCALE_32BIT;
	can1_filter_structure.FilterIdHigh = 0x0000;//下面配置则不筛选ID
	can1_filter_structure.FilterIdLow = 0x0000;
	can1_filter_structure.FilterMaskIdHigh = 0x0000;
	can1_filter_structure.FilterMaskIdLow = 0x0000;
	can1_filter_structure.FilterBank = 0;
	can1_filter_structure.FilterFIFOAssignment = CAN_RX_FIFO0;//使用FIFO0
	HAL_CAN_ConfigFilter(&hcan1, &can1_filter_structure);
	
	HAL_CAN_Start(&hcan1);
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);//使能中断
    isRcan1Started=1;
}


void can2_fliter_init(void)
{
	CAN_FilterTypeDef can2_filter_structure;
	can2_filter_structure.FilterActivation = ENABLE;//使能滤波器
	can2_filter_structure.FilterMode = CAN_FILTERMODE_IDMASK;//掩码模式
	can2_filter_structure.FilterScale = CAN_FILTERSCALE_32BIT;
	can2_filter_structure.FilterIdHigh = 0x0000;//下面配置则不筛选ID
	can2_filter_structure.FilterIdLow = 0x0000;
	can2_filter_structure.FilterMaskIdHigh = 0x0000;
	can2_filter_structure.FilterMaskIdLow = 0x0000;
	can2_filter_structure.FilterBank = 0;
	can2_filter_structure.FilterFIFOAssignment = CAN_RX_FIFO0;//使用FIFO0
	//HAL_CAN_ConfigFilter(&hcan2, &can2_filter_structure);
	
	//HAL_CAN_Start(&hcan2);
	//HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);//使能中断
    isRcan2Started=1;
}
/*设置电机电压*/
void Set_voltagec1(CAN_HandleTypeDef* hcan,int16_t voltage1,int16_t voltage2,int16_t voltage3,int16_t voltage4)
{
  CAN_TxHeaderTypeDef can1TxMsg;
  uint8_t             can1TxData[8] = {0};
  int16_t voltage[]={voltage1,voltage2,voltage3,voltage4};
  can1TxMsg.StdId = 0x1ff;
  can1TxMsg.IDE   = CAN_ID_STD;//标准ID
  can1TxMsg.RTR   = CAN_RTR_DATA;//数据帧
  can1TxMsg.DLC   = 8;//数据长度
  for(int8_t i=0;i<4;i++)
  {
   can1TxData[2*i]=(voltage[i]>>8)&0xff;
   can1TxData[2*i+1]=(voltage[i])&0xff;
  }
  HAL_CAN_AddTxMessage(&hcan1, &can1TxMsg, can1TxData,(uint32_t*)CAN_TX_MAILBOX0);//发送报文
}
{
  CAN_TxHeaderTypeDef can1TxMsg;
  uint8_t             can1TxData[8] = {0};
  int16_t voltage[]={voltage1,voltage2,voltage3,voltage4};
  can1TxMsg.StdId = 0x1ff;
  can1TxMsg.IDE   = CAN_ID_STD;//标准ID
  can1TxMsg.RTR   = CAN_RTR_DATA;//数据帧
  can1TxMsg.DLC   = 8;//数据长度
  for(int8_t i=0;i<4;i++)
  {
   can1TxData[2*i]=(voltage[i]>>8)&0xff;
   can1TxData[2*i+1]=(voltage[i])&0xff;
  }
  HAL_CAN_AddTxMessage(&hcan1, &can1TxMsg, can1TxData,(uint32_t*)CAN_TX_MAILBOX0);//发送报文
}

void Set_voltagec2(CAN_HandleTypeDef* hcan,int16_t voltage1,int16_t voltage2,int16_t voltage3,int16_t voltage4)
{
  CAN_TxHeaderTypeDef can2TxMsg;
  uint8_t             can2TxData[8] = {0};
  int16_t voltage[]={voltage1,voltage2,voltage3,voltage4};
  can2TxMsg.StdId = 0x1ff;
  can2TxMsg.IDE   = CAN_ID_STD;//标准ID
  can2TxMsg.RTR   = CAN_RTR_DATA;//数据帧
  can2TxMsg.DLC   = 8;//数据长度
  for(int8_t i=0;i<4;i++)
  {
   can2TxData[2*i]=(voltage[i]>>8)&0xff;
   can2TxData[2*i+1]=(voltage[i])&0xff;
  }
//  HAL_CAN_AddTxMessage(&hcan2, &can2TxMsg, can2TxData,(uint32_t*)CAN_TX_MAILBOX0);//发送报文
}
//接收中断回调函数
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  uint8_t flag=0;
  CAN_RxHeaderTypeDef can1RxMsg;
  CAN_RxHeaderTypeDef can2RxMsg;
  if(hcan==&hcan1)
  {
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &can1RxMsg, can1RxData);
	if(can1RxMsg.StdId==0x201){
		C620_1.Rxmsg.Angle= ((can1RxData[0] << 8) | can1RxData[1])*360/8192.0f;
		C620_1.Rxmsg.Speed= ((can1RxData[2] << 8) | can1RxData[3]);
		C620_1.Rxmsg.Torque=((can1RxData[4] << 8) | can1RxData[5]);
		C620_1.Rxmsg.Temp=can1RxData[6];
		flag=1;
	}
	if(can1RxMsg.StdId==0x202){
		C620_2.Rxmsg.Angle= ((can1RxData[0] << 8) | can1RxData[1])*360/8192.0f;
		C620_2.Rxmsg.Speed= ((can1RxData[2] << 8) | can1RxData[3]);
		C620_2.Rxmsg.Torque=((can1RxData[4] << 8) | can1RxData[5]);
		C620_2.Rxmsg.Temp=can1RxData[6];
		flag=1;
	}
	if(can1RxMsg.StdId==0x203){
		C620_3.Rxmsg.Angle= ((can1RxData[0] << 8) | can1RxData[1])*360/8192.0f;
		C620_3.Rxmsg.Speed= ((can1RxData[2] << 8) | can1RxData[3]);
		C620_3.Rxmsg.Torque=((can1RxData[4] << 8) | can1RxData[5]);
		C620_3.Rxmsg.Temp=can1RxData[6];
		flag=1;
	}
	if(can1RxMsg.StdId==0x204){
		C620_4.Rxmsg.Angle= ((can1RxData[0] << 8) | can1RxData[1])*360/8192.0f;
		C620_4.Rxmsg.Speed= ((can1RxData[2] << 8) | can1RxData[3]);
		C620_4.Rxmsg.Torque=((can1RxData[4] << 8) | can1RxData[5]);
		C620_4.Rxmsg.Temp=can1RxData[6];
		flag=1;
	} 

  //  if(hcan==&hcan2)
		{
	     HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &can2RxMsg, can2RxData);
		 if(can2RxMsg.StdId==0x201){
		C620_1.Rxmsg.Angle= ((can2RxData[0] << 8) | can2RxData[1])*360/8192.0f;
		C620_1.Rxmsg.Speed= ((can2RxData[2] << 8) | can2RxData[3]);
		C620_1.Rxmsg.Torque=((can2RxData[4] << 8) | can2RxData[5]);
		C620_1.Rxmsg.Temp=can2RxData[6];
		flag=1;
	}
	if(can2RxMsg.StdId==0x202){
		C620_2.Rxmsg.Angle= ((can2RxData[0] << 8) | can2RxData[1])*360/8192.0f;
		C620_2.Rxmsg.Speed= ((can2RxData[2] << 8) | can2RxData[3]);
		C620_2.Rxmsg.Torque=((can2RxData[4] << 8) | can2RxData[5]);
		C620_2.Rxmsg.Temp=can2RxData[6];
		flag=1;
	}
	if(can2RxMsg.StdId==0x203){
		C620_3.Rxmsg.Angle= ((can2RxData[0] << 8) | can2RxData[1])*360/8192.0f;
		C620_3.Rxmsg.Speed= ((can2RxData[2] << 8) | can2RxData[3]);
		C620_3.Rxmsg.Torque=((can2RxData[4] << 8) | can2RxData[5]);
		C620_3.Rxmsg.Temp=can2RxData[6];
		flag=1;
	}
	if(can2RxMsg.StdId==0x204){
		C620_4.Rxmsg.Angle= ((can2RxData[0] << 8) | can2RxData[1])*360/8192.0f;
		C620_4.Rxmsg.Speed= ((can2RxData[2] << 8) | can2RxData[3]);
		C620_4.Rxmsg.Torque=((can2RxData[4] << 8) | can2RxData[5]);
		C620_4.Rxmsg.Temp=can2RxData[6];
		flag=1;
	} 
		}


	
	
}
