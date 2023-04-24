/**
  ******************************************************************************
  * @file      stm32_canfestival.c
  * @brief     CANopen回调函数
  * @author    lxtqyh
  * @date      2022-9-1
  * @version   V1.0
  * @details   这个文件提供了canfestival运行所需的底层接口函数，主要分为两类，一
  *            类是CAN的收发函数，另一类是时间计算相关函数，函数列表如下：
  *            + canSend()                CANopen发送函数
  *            + canReceive_Callback()    CANopen接收回调函数
  *            + setTimer()               计算下一次调用TimeDispatch()时间
  *            + getElapsedTime()         用来查询距离下一个定时触发还有多少时间
  *            + timerForCan()            CANopen定时器回调函数
  * @copyright santiqyh@qq.com
  ******************************************************************************
  * @attention
  * 硬件平台：STM32F1 \n
  * 软件平台：MDK5.31  CUBEMX6.5.0  Firmware Package 1.8.4
  * @par      修改日志
  * <table>
  * <tr><th>Data       <th>Version  <th>Author   <th>Description
  * <tr><td>2022.9.1   <td>1.0      <td>lxtqyh   <td>创建文件
  * </table>
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "canfestival.h"
#include "CANopen_Master_M200.h"
#include "can.h"
#include "tim.h"
#include "cmsis_os.h"

/* Private variables ---------------------------------------------------------*/
static uint32_t TimeCNT=0;                     /*时间计数*/
static uint32_t NextTime=0;                    /*下一次触发时间计数*/
static uint32_t TIMER_MAX_COUNT=10000;         /*最大时间计数*/
static TIMEVAL last_time_set = TIMEVAL_MAX;    /*上一次的时间计数*/


uint8_t heartID = 0;
uint8_t heartState = 0;
//UNS8 res = SDO_UPLOAD_IN_PROGRESS;
extern CO_Data CANopen_Master_M200_Data;
/* Exported functions --------------------------------------------------------*/
/**
  * @brief   计算下一次调用TimeDispatch()时间
  */
void setTimer(TIMEVAL value)
{
	//mlh
   NextTime=(TimeCNT+value)%TIMER_MAX_COUNT;
	//mlh
	
//	uint16_t capture = 0;

//	capture = __HAL_TIM_GET_COMPARE(&htim3,TIM_CHANNEL_1);
//	__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1, capture + value);
}

/**
  * @brief   用来查询距离下一个定时触发还有多少时间
  */
TIMEVAL getElapsedTime(void)
{
	// MLH
  int ret=0;
  ret = TimeCNT> last_time_set ? TimeCNT - last_time_set : TimeCNT + TIMER_MAX_COUNT - last_time_set;
  //last_time_set = TimeCNT;
  return ret;
	// MLH
//	uint16_t timer = __HAL_TIM_GET_COUNTER(&htim3);
//	return timer > last_time_set ? timer - last_time_set : last_time_set - timer; 
	
}

/**
  * @brief   CANopen发送函数 
  */
UNS8 canSend(CAN_PORT notused, Message *m)
{
  CAN_TxHeaderTypeDef   TxHeader;       /*发送句柄*/
	uint32_t TxMailbox;                   /*储存发送数据的发送邮箱编号,没有用到*/
	uint8_t message[8];                   /*发送数据缓冲*/  

	TxHeader.StdId=m->cob_id;             /*标准标识符*/
	TxHeader.ExtId=0x00;                  /*扩展标识符(29位)*/
	TxHeader.IDE=CAN_ID_STD;              /*使用标准帧*/
	TxHeader.DLC=m->len;                  /*数据长度*/
	TxHeader.TransmitGlobalTime = DISABLE;/*时间戳不开启 */  
  if(m->rtr)
  {TxHeader.RTR=CAN_RTR_REMOTE;}        /*远程帧*/  
  else
  {TxHeader.RTR=CAN_RTR_DATA;}          /*数据帧*/    

	for(uint8_t i=0;i<m->len;i++)
	{
		message[i] = m->data[i];            /*数据搬移*/
	}
	while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0);/*判断发送队列是否已满*/
	if(HAL_CAN_AddTxMessage(&hcan1, &TxHeader, message, &TxMailbox) == HAL_OK)/* 添加发送数据至发送邮箱并发送出去 */
	{
//		return 0xff; MLH
			return 0x00;
	}  
  else
  {
//    return 0x00; MLH
			return 0xff;
  }
}

/**
  * @brief   CANopen接收回调函数
  * @note    该函数需要在CAN外设接收中断函数中被调用 
  */
void canReceive_Callback(CAN_RxHeaderTypeDef *pHeader,uint8_t* msg)
{
  Message m;
  m.cob_id = pHeader->StdId;            /*标准标识符*/
  if(pHeader->RTR == CAN_RTR_REMOTE)    /*远程帧*/  
  {m.rtr=1;}
  else if(pHeader->RTR  == CAN_RTR_DATA)/*数据帧*/
  {m.rtr=0;}
  m.len=pHeader->DLC;                   /*数据长度*/
	for(uint8_t i=0;i<m.len;i++)
	{
		m.data[i] = msg[i];                 /*数据搬移*/
	}
  canDispatch(&CANopen_Master_M200_Data, &m);/* CANopen协议解析函数 */  
}

/**
  * @brief   CANopen定时器回调函数
  * @note    该函数需要在定时器中断函数中周期性调用，定时器周期需要设置为1ms
  */
void timerForCan(void)
{
	//mlh
  last_time_set = TimeCNT;
  TimeCNT++;
  if (TimeCNT>=TIMER_MAX_COUNT)
  {
    TimeCNT=0;
  }
  if (TimeCNT==NextTime)
  {
    TimeDispatch();
  }
	//mlh
//		last_time_set = __HAL_TIM_GET_COUNTER(&htim3);
//		TimeDispatch();
		sendPDOevent (&CANopen_Master_M200_Data);
}

// 设备心跳超时回调函数
void heartbeat_error(CO_Data* d, UNS8 heartbeatID)
{
		heartID = heartbeatID;
		heartState = 0;
}

// 设备上线回调函数
void post_SlaveBootup(CO_Data* d, UNS8 heartbeatID)
{
		heartID = heartbeatID;
		heartState = 1;
}

//SDO读canopen从站数据
UNS8 ReadSDO(CO_Data* d, UNS8 nodeId, UNS16 index, UNS8 subIndex, UNS8 dataType, void* data, UNS32* size, UNS8 useBlockMode)
{
	UNS32 abortCode = 0;
	UNS8 res = SDO_UPLOAD_IN_PROGRESS;
	// Read SDO
	UNS8 err = readNetworkDict(d, nodeId, index, subIndex, dataType, useBlockMode);
	if (err)
			return 0xFF;
	for (uint8_t i =0; i <= 5; i++)
	{
			res = getReadResultNetworkDict(d, nodeId, data, size, &abortCode);
					//printf("sendsdo res = %x\n", res);
			if (res != SDO_UPLOAD_IN_PROGRESS)
					break;
			HAL_Delay(2);
			continue;
	}
	closeSDOtransfer(d, nodeId, SDO_CLIENT);
	if (res == SDO_FINISHED)
			return 0;
	return 0xFE;
}

// SDO写canopen从站数据
UNS8 WriteSDO(CO_Data* d, UNS8 nodeId, UNS16 index, UNS8 subIndex, UNS32 count, UNS8 dataType, void* data, UNS8 useBlockMode)
{
    UNS32 abortCode = 0;
    UNS8 res = SDO_DOWNLOAD_IN_PROGRESS;
    // Write SDO
    UNS8 err = writeNetworkDict(d, nodeId, index, subIndex, count, dataType, data, useBlockMode);
    if (err)
        return 0xFF;
    for (uint8_t i =0; i <= 3; i++)
    {
        res = getWriteResultNetworkDict(d, nodeId, &abortCode);
        //printf("write res = %x\n", res);
        if (res != SDO_DOWNLOAD_IN_PROGRESS)
            break;
        HAL_Delay(1);
				continue;
    }
    closeSDOtransfer(d, nodeId, SDO_CLIENT);
    
    if (res == SDO_FINISHED)
        return 0;
    return 0xFF;
}



// 车辆设备启动
void carDeviceStart(void)
{
		masterSendNMTstateChange(&CANopen_Master_M200_Data, 0, 0x01);
}

// 车辆设备检查
UNS8 carDeviceCheck(void)
{
		return 0;
}

/********************************END OF FILE***********************************/
