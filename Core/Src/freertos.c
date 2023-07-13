/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "handle.h"
#include "can.h"
#include "adc_read.h"
#include "can_device.h"
#include "iwdg.h"
#include "air820.h"
#include "canfestival.h"
#include "CANopen_Master_M200.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
//int fan_num1 = 0;
//int fan_num2 = 0;

extern CO_Data CANopen_Master_M200_Data;
//uint8_t INDECATOR_LIGHT_IDLE = 0;			// 同步帧标志位
//uint16_t INDECATOR_LIGHT_IDLE_NUM = 0;

//UNS8 data = 0;
//UNS32 size = sizeof(data);

/* USER CODE END Variables */
osThreadId Task01Handle;
osThreadId Task02Handle;
osThreadId Task03Handle;
osThreadId Task04Handle;
osTimerId myTimer01Handle;
osSemaphoreId BinarySem01Handle;
osSemaphoreId BinarySem02Handle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartTask01(void const * argument);
void StartTask02(void const * argument);
void StartTask03(void const * argument);
void StartTask04(void const * argument);
void Callback01(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* GetTimerTaskMemory prototype (linked to static allocation support) */
void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/* USER CODE BEGIN GET_TIMER_TASK_MEMORY */
static StaticTask_t xTimerTaskTCBBuffer;
static StackType_t xTimerStack[configTIMER_TASK_STACK_DEPTH];

void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize )
{
  *ppxTimerTaskTCBBuffer = &xTimerTaskTCBBuffer;
  *ppxTimerTaskStackBuffer = &xTimerStack[0];
  *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
  /* place for user code */
}
/* USER CODE END GET_TIMER_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of BinarySem01 */
  osSemaphoreDef(BinarySem01);
  BinarySem01Handle = osSemaphoreCreate(osSemaphore(BinarySem01), 1);

  /* definition and creation of BinarySem02 */
  osSemaphoreDef(BinarySem02);
  BinarySem02Handle = osSemaphoreCreate(osSemaphore(BinarySem02), 1);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* definition and creation of myTimer01 */
  osTimerDef(myTimer01, Callback01);
  myTimer01Handle = osTimerCreate(osTimer(myTimer01), osTimerPeriodic, NULL);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
	osTimerStart(myTimer01Handle, 100);
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of Task01 */
  osThreadDef(Task01, StartTask01, osPriorityBelowNormal, 0, 512);
  Task01Handle = osThreadCreate(osThread(Task01), NULL);

  /* definition and creation of Task02 */
  osThreadDef(Task02, StartTask02, osPriorityHigh, 0, 128);
  Task02Handle = osThreadCreate(osThread(Task02), NULL);

  /* definition and creation of Task03 */
  osThreadDef(Task03, StartTask03, osPriorityNormal, 0, 128);
  Task03Handle = osThreadCreate(osThread(Task03), NULL);

  /* definition and creation of Task04 */
  osThreadDef(Task04, StartTask04, osPriorityAboveNormal, 0, 128);
  Task04Handle = osThreadCreate(osThread(Task04), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartTask01 */
/**
  * @brief  Function implementing the Task01 thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartTask01 */
void StartTask01(void const * argument)
{
  /* USER CODE BEGIN StartTask01 */
  /* Infinite loop *//*------------------------------4G--------------------------------*/
  for(;;)
  {

        if(control_flag.sTimer_ms_1000 >= Count_1s)
        {
            control_flag.sTimer_ms_1000 = 0;
            if (gps_info.gpsCheckcount < 120)
            {
              gps_info.gpsCheckcount++;
            }
            //运行状态下每120s检查一次GPS连接
            if(gps_info.gpsCheckcount == 120)
            {
              if(control_flag.Car_State == RUN)
              {
                if(gps_info.gps_signal_flag != 0x31)
                {
                  gpsReset(air_4g_flag.vTa_delay);
                  gps_info.gpsCheckcount = 0;
                  gps_info.gpsUrc =10;
                  gps_info.gpsUrcSet = 1;
                }
              }
            }

            if(control_flag.Car_State == RUN)
            {
                // air_4g_MPUB(control_flag.Car_State);    // 运行状态下发送
                air_4g_MPUB(RUN);    // 运行状态下发送
            }
            else if(control_flag.Car_State == CLOSE && control_flag.sTimer_ms_10000 == Count_10s)	//运行状态跳回睡眠状态、待机状态下需等待运行记录上报事件完毕
            {
                control_flag.sTimer_ms_10000 = 0;
                // air_4g_MPUB(control_flag.Car_State);    // 睡眠状态下发送
                air_4g_MPUB(CLOSE);    // 睡眠状态下发送
            }
            else if(control_flag.Car_State == OPEN && control_flag.sTimer_ms_5000 == Count_5s)
            {
                control_flag.sTimer_ms_5000 = 0;
                // air_4g_MPUB(control_flag.Car_State);    // 待机状态下发送
                air_4g_MPUB(OPEN);    // 待机状态下发送
            }
            else if(control_flag.Car_State == FAULT && control_flag.sTimer_ms_5000 == Count_5s)
            {
                control_flag.sTimer_ms_5000 = 0;
                // air_4g_MPUB(control_flag.Car_State);    //异常状态下发送
                air_4g_MPUB(FAULT);    //异常状态下发送
            }
            else if(control_flag.Car_State == RECHARGE && control_flag.sTimer_ms_10000 == Count_10s)
            {
                control_flag.sTimer_ms_10000 = 0;
                // air_4g_MPUB(control_flag.Car_State);    // 充电状态下发送
                air_4g_MPUB(RECHARGE);    // 充电状态下发送
            }

						usart1_sbus_tx();// 运行状态下数传发送
						while (huart1.gState != HAL_UART_STATE_READY){}
						memset(&sbus_pack_data,0,sizeof(sbus_pack_data));
        }

        // 调整GPS上报频率
        if(gps_info.gpsUrc != gps_info.gpsUrcSet)
        {
            char gpsBuf[20];
            sprintf(gpsBuf, "AT+CGNSURC=%d\r", gps_info.gpsUrcSet);
            air4g_send_cmd("ATE0\r", "OK", 50, air_4g_flag.vTa_delay);
            air4g_send_cmd(gpsBuf, "OK", 100, air_4g_flag.vTa_delay);
            gps_info.gpsUrc = gps_info.gpsUrcSet;
        }
        if(air_4g_flag.MQTT_flag == FALSE)
        {
            air_4g_flag.MQTT_flag = TRUE;
						vTaskDelay(5000);
            // 开启主动上报
            air_4g_openURC(1);
        }
		osDelay(1);
  }
  /* USER CODE END StartTask01 */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the Task02 thread.
* @param argument: Not used
* @retval None
*/
uint8_t Check1=0;
uint8_t Check2=0;
/* USER CODE END Header_StartTask02 */
void StartTask02(void const * argument)
{
  /* USER CODE BEGIN StartTask02 */
  /* Infinite loop *//*------------------------------遥控器--------------------------------*/
  for(;;)
  {
        // 100ms逻辑：车辆控制
        if(control_flag.cTimer_ms_100 == 1)
        {
            control_flag.cTimer_ms_100=0;

            // 车辆控制状态机
            car_state_trans();
        }

        // 1s逻辑：散热风扇控制，主控盒开盖检测
        if(control_flag.cTimer_ms_1000 == 10)
        {
            control_flag.cTimer_ms_1000=0;

            // 散热风扇控制
            Boxfan_control();
            Carfan_control();

            // 开盖检测
            Check1 = HAL_GPIO_ReadPin(GPIOE,Light_Check1_Pin);
            Check2 = HAL_GPIO_ReadPin(GPIOE,Light_Check2_Pin);
            control_flag.main_kaihe_flag = (!Check1) | (!Check2);

            // 辅助S48100电池进行充电
            if(battery_data.battType == S48100)
            {
                S48100B_TEMP_CHECK();
            }
        }

        // 3s逻辑：读取电池数据，低电量判定
        if(control_flag.cTimer_ms_3000 == 30)
        {
            control_flag.cTimer_ms_3000 = 0;

            // 读取电池数据
            batteryReadData();

            // 判断风机水泵禁止逻辑
            // if(control_flag.Car_State == RUN)
            // {
            //     if(battery_data.soc > BatteryLowSOC_Level1)			// 电量高于一级电量报警值
            //     {
            //         battery_data.socState = SOC_ENOUGH;
            //         indicatorLight(LED12VR_LED12VRState);
            //         control_flag.DraughtPumpEnable = TRUE;
            //     }
            //     else if(battery_data.soc <= BatteryLowSOC_Level1 && battery_data.soc > BatteryLowSOC_Level2)		// 电量低于一级电量报警值，高于二级电量报警值
            //     {
            //         control_flag.DraughtPumpEnable = TRUE;
            //         battery_data.socState = SOC_LOW_LEVEL_1;
            //         LED12VR_LED12VRState = INDECATOR_LIGHT_ITEM_SOC_LOW_LEVEL_2;
            //         indicatorLight(LED12VL_LED12VLState);
            //     }
            //     else if(battery_data.soc <= BatteryLowSOC_Level2)	// 电量低于二级电量报警值
            //     {
            //         control_flag.DraughtPumpEnable = FALSE;
            //         battery_data.socState = SOC_LOW_LEVEL_2;
            //         LED12VL_LED12VLState = INDECATOR_LIGHT_ITEM_SOC_LOW_LEVEL_1;
            //         indicatorLight(LED12VR_LED12VRState);
            //     }
            //     else
            //     {
            //         control_flag.DraughtPumpEnable = TRUE;
            //         battery_data.socState = SOC_ENOUGH;
            //         indicatorLight(LED12VR_LED12VRState);
            //         control_flag.DraughtPumpEnable = TRUE;
            //     }
            // }
        }

        // 5s逻辑：读取电机数据，读取小电池数据
        if(control_flag.cTimer_ms_5000 == 50)
        {
            control_flag.cTimer_ms_5000 = 0;

            //读取电机驱动器数据
            read_moto_data();

            //按顺序读取ADC通道值
            adc_batt_read();
            adc_intem_read();

        }

        // 120s逻辑：软开关打开时进行低电量播报
        if(control_flag.cTimer_ms_60000 == 1200)
        {
            control_flag.cTimer_ms_60000 = 0;

            if(softSwitch_switchFlag == TRUE)
            {
                if(battery_data.soc <= BatteryLowSOC_Level1 && battery_data.soc > BatteryLowSOC_Level2)
                {
                    speakItem(SPEAK_ITEM_SOC(battery_data.soc));
                }
                else if(battery_data.soc <= BatteryLowSOC_Level2)
                {
                    speakItem(SPEAK_ITEM_WORK_BAN);
                    speakItem(SPEAK_ITEM_CHARGE_REMIND);
                }
            }
        }

        // 计时周期
        control_flag.cTimer_ms_100++;
        control_flag.cTimer_ms_1000++;
        control_flag.cTimer_ms_3000++;
        control_flag.cTimer_ms_5000++;
        control_flag.cTimer_ms_60000++;
        osDelay(90);
    }
  /* USER CODE END StartTask02 */
}

/* USER CODE BEGIN Header_StartTask03 */
/**
* @brief Function implementing the Task03 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask03 */
void StartTask03(void const * argument)
{
  /* USER CODE BEGIN StartTask03 */
  /* Infinite loop *//*--------------------------数据接收处理TASK--------------------------------*/
  for(;;)
  {
        if(osOK == osSemaphoreWait(BinarySem01Handle,osWaitForever))		// 收到串口接收信号量
        {
            //if(usart3_handle_4g.rx_len > SHORT_MESG)
            //{
            // 如果判断是GPS自动上报的数据，并且GNSS run status正常，则接收并更新经纬度信息，这里存在一个问题，就是GNSS run status异常，就不会更新GPS信息，这里需要做处理
            if(strstr((char*)usart3_handle_4g.report_buf,"+UGNSINF: 1") != NULL )
            {
                char *token;

                // strtok两次直接取第二个","之前的字符串
                token = strtok((char*)usart3_handle_4g.report_buf, ",");

                // 获取GPS信号
                gps_info.gps_signal_flag = *strtok(NULL, ",");	// 这个标志是GPS是否获取到了，实时更新


                // 获取GPS
								token = strtok(NULL,",");//分割一次逗号，但时间不在此获取
                if(gps_info.gps_signal_flag == 0x31 )		    // 第二个逗号到第一个逗号之间的值表示GPS有无获取   0x31：有
                {
                    token = strtok(NULL, ",");//获取纬度

                    //if(token[0] > 0x2f && token[0] < 0x3a)
                    if(token[0] > 0x30 && token[0] < 0x3a)
                    {
                        strcpy((char *)gps_info.Lat_nowstr, token);//拷贝纬度

                        token = strtok(NULL, ",");//获取纬度
                        strcpy((char *)gps_info.Lon_nowstr,token);//拷贝纬度

                        token = strtok(NULL, ".");//获取高程
                        gps_info.MSL_Altitude = atoi(token);
                        // strcpy((char *)gps_info.Msl_nowstr,token);//拷贝高程

                        if(control_flag.save_turn_flag == SET)
                        {
                            control_flag.save_turn_flag = RESET;
                            if(sing_work_event.save_track_count < SING_TRACK_MAX)
                            {
                                strcat(sing_work_event.trackData,(const char *)gps_info.Lon_nowstr);
                                strcat(sing_work_event.trackData,",");
                                strcat(sing_work_event.trackData,(const char *)gps_info.Lat_nowstr);
                                strcat(sing_work_event.trackData,";");
                                sing_work_event.save_track_count++;
                            }
                        }
                    }
                }
                else
                {
                    gps_info.gps_signal_flag=0;	//这个标志是GPS是否获取到了
                }
            }

			// 上报出错次数累计 58不识别指令允许100次
            else if(strstr((char*)usart3_handle_4g.report_buf,"ERROR: 58")!=NULL)
            {
                if(control_flag.error_res_count<100) {
                    control_flag.error_res_count++;
                }
                else{
                    control_flag.error_res_flag = TRUE;
                }
            }

            memset(usart3_handle_4g.report_buf,0,SAVE_SIZE);
        }
        osDelay(1);
  }
  /* USER CODE END StartTask03 */
}

/* USER CODE BEGIN Header_StartTask04 */
/**
* @brief Function implementing the Task04 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask04 */
void StartTask04(void const * argument)
{
  /* USER CODE BEGIN StartTask04 */
  /* Infinite loop */
	//-----------------------------------定时属性上报&数据处理TASK-----------------------------------//
	uint32_t Prewaketime = osKernelSysTick();

  for(;;)
    {
        // 注意1s计算一次霍尔差
        send_moto_cmd(MOTO1_HALL, MOTO_REGIS_NUM2, 0, MOTO_ReadREQ_ID);

        // 依据状态计算汇总时间
        if(control_flag.Car_State > CLOSE)
        {
            count_time.countRunTime++;
        }
        if(control_flag.Car_State == RUN)
        {
            if(control_flag.Draught_swith == TRUE)
            {
                count_time.countFanMachineryTime++;
            }
            if(control_flag.Pump_swith != G_0)
            {
                count_time.countWaterPumpTime++;
            }
        }



       // 绝对延时下看门狗4S喂狗
        if (control_flag.Iwdg_count == 4)
        {
            control_flag.Iwdg_count = 0;
            HAL_IWDG_Refresh(&hiwdg);
        }
        else
        {
            control_flag.Iwdg_count++;
        }


        osDelayUntil(&Prewaketime,1000);
    }
  /* USER CODE END StartTask04 */
}

/* Callback01 function */
void Callback01(void const * argument)
{
  /* USER CODE BEGIN Callback01 */
//	control_flag.sTimer_ms_100++;
//	control_flag.sTimer_ms_500++;

    // 1S计时
    if(control_flag.sTimer_ms_1000 < Count_1s)
    {
		control_flag.sTimer_ms_1000++;
	}

    // 5S计时
	if(control_flag.sTimer_ms_5000 < Count_5s)
    {
		control_flag.sTimer_ms_5000++;
	}

    // 10S计时
	if(control_flag.sTimer_ms_10000 < Count_10s)
    {
		control_flag.sTimer_ms_10000++;
	}

    // 60S计时
    if(control_flag.sTimer_ms_60000 < Count_60s)
    {
		control_flag.sTimer_ms_60000++;
	}

  /* USER CODE END Callback01 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
