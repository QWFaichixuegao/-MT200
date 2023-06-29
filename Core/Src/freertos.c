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
            air_4g_connect.air820Count++;
            if (gps_info.gpsCheckcount < 120)
            {
              gps_info.gpsCheckcount++;
            }
            // 车辆运行数据上报,目前每次慢300ms
            if(air_4g_flag.MQTT_flag == TRUE /* && control_flag.event_mpub_mutex == FALSE && gps_info.LBSRxFlag == FALSE*/)
            {
                if(control_flag.event_mpub_single_flag == TRUE)
                {
                  air_4g_MPUB_event(WORK_EVENT_MAIN);
                  air_4g_MPUB_event(WORK_EVENT_TRACK);
                  while (huart3.gState != HAL_UART_STATE_READY){}
                  memset(&sing_work_event,0,sizeof(sing_work_event));//上报完后清空运行记录结构体数据
                  control_flag.event_mpub_single_flag = FALSE;
                }
                else
                {
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

                    while (huart3.gState != HAL_UART_STATE_READY){}
                    memset(mqtt_pub_inform.PubBuf,0,sizeof(mqtt_pub_inform.PubBuf));

                    //else if(control_flag.Car_State == RECHARGE && control_flag.sTimer_ms_60000 == Count_60s)
                    //{
                    //    control_flag.sTimer_ms_60000 = 0;
                    //    air_4g_MPUB(control_flag.Car_State);    // 充电状态下发送
                    //}
                }
                // vTaskDelay(10);
            }

            //复位后每秒发送一次获取实时时间指令直到时间校准成功
            if(air_4g_flag.get_realtime_flag == FALSE)
            {
              get_real_time(1);
            }

            //每20s读取一次4G当前信息
            if(air_4g_connect.air820Count == 10)
            {
              air_4g_connect.air820Count = 0;
              get_4G_msg(1);// 获取4G当前信息(MQTT连接状态、4G信号质量)
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

        // 开启蓝牙
        if(ble_inform.ble_switch_state != ble_inform.ble_switch)
        {
            if(ble_inform.ble_switch == TRUE)
            {
                ble_swit_on(1);
            }
            else if(ble_inform.ble_switch == FALSE)
            {
                ble_swit_off(1);
            }
            ble_inform.ble_switch_state = ble_inform.ble_switch;
        }

        // 如果在绑定模式下接收到了绑定请求，就关闭绑定模式，并通过蓝牙给手机发送三元组信息
        if(control_flag.Car_State == OPEN && control_flag.Bound_flag == TRUE)
        {
            control_flag.Bound_flag 	= FALSE;

            /*
            发送三元组信息及其他传输
            AT+BLECOMM=SENDDATA,fee2,21,7172735F7465737436246768333750317A51646F47
            */
            air4g_send_cmd(ble_inform.dev_send_cmd, "OK", 100, air_4g_flag.vTa_delay);
        }

        // 车辆和MQTT服务器断开连接
		if(air_4g_flag.MQTT_flag == FALSE)
		{

            // 回显模式关闭
            for(uint8_t i = 0; i < 5; i++)
            {
                vTaskDelay (1000);
                if(!air4g_send_cmd("ATE0\r", "OK", 500, air_4g_flag.vTa_delay))
                    break;
            }
            air4g_send_cmd("AT+MQTTSTATU\r", "+MQTTSTATU :1", 200, air_4g_flag.vTa_delay);
            // 再次检查车辆和MQTT服务器连接是否正常，这里可以检查3次
            if(air_4g_flag.MQTT_flag == TRUE)
            {
                air_4g_flag.MQTT_flag = TRUE;
                air_4g_flag.SIM_flag = TRUE;
            }
            else
            {
                /******检查SIM卡连接是否正常*******/

                // 如果车辆和接蜂窝网络连接正常，这里可以检查3次

                for(uint8_t i = 0; i <= 3; i++)
                {
                    air_4g_closeURC(1);
                    // 如果车辆和接蜂窝网络连接不正常
                    if(i >= 3)
                    {
                        air_4g_flag.SIM_flag = FALSE;
                        // 蜂窝网络连接不正常，就让4G模块重启
                        air_4g_restar(air_4g_flag.vTa_delay);
                        break;
                    }
                    if(air_4g_net_check(1) == SIM_OK)
                    {
                        air_4g_flag.SIM_flag = TRUE;
                        if(air_4g_connect_server(0,1) == MQTT_CONNECT_OK)   // MQTT断连后自动关闭MQTT
                        {
                            // 如果车辆和MQTT服务器连接成功
                            air_4g_flag.MQTT_flag = TRUE;
                            air_4g_flag.SIM_flag = TRUE;
                            break;
                        }
                    }
                    vTaskDelay(1000);

                }

            }
            // 检查车辆和MQTT服务器是否连接，首次上报初次上线需要上报的信息
            if(air_4g_flag.MQTT_flag == TRUE)
            {
                air_4g_OTAMPUB(1);
                air_4g_OTASUB(1);
                topic_sub(1);

                // 开启主动上报
                air_4g_openURC(1);
            }
		}

		// 车辆和MQTT服务器连接正常
		else
		{
			vTaskDelay(10);
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

						// 更新MQTT连接状态
            else if(strstr((char*)usart3_handle_4g.report_buf,"+MQTTSTATU :1")!=NULL)
            {
							air_4g_flag.MQTT_flag = TRUE;
            }
            else if(strstr((char*)usart3_handle_4g.report_buf,"+MQTTSTATU :0")!=NULL)
            {
							air_4g_flag.MQTT_flag = FALSE;
            }

						// 更新SIM卡信号质量
            //else if(strstr((char*)usart3_handle_4g.report_buf,"+CSQ:")!=NULL)
            else if(strstr((char*)usart3_handle_4g.report_buf,"+CSQ:")!=NULL)
            {
                char *token;
                token=strtok((char*)usart3_handle_4g.report_buf,":");
                token=strtok(NULL,",");
                air_4g_connect.sim_CSQ = atoi(token);
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

            // 在绑定模式模式下
            else if((control_flag.Match_flag == TRUE) && (strstr((char*)usart3_handle_4g.report_buf,"+BLEIND=DATA")!=NULL))
            {
                /*
                连接状态上报
                +BLEIND=CONNECT,"73:80:b7:3f:3c:12"
                接收到数据
                +BLEIND=DATA,"fee1",3,"303124"
                接收到数据
                +BLEIND=DATA,"fee1",3,"303324"
                */
                if(strstr((char*)usart3_handle_4g.report_buf,"303124")!=NULL )
                {
                    control_flag.Bound_flag = TRUE;
                }
                else if(strstr((char*)usart3_handle_4g.report_buf,"303224")!=NULL )
                {
                    speakItem(SPEAK_ITEM_BIND_FAIL);			//语音播报“绑定失败”
                }
                else if(strstr((char*)usart3_handle_4g.report_buf,"303324")!=NULL )
                {
                    speakItem(SPEAK_ITEM_BIND_SECCEEED);		//语音播报“绑定成功”
                }
            }
            //开机时校准一次实时时间
            else if(strstr((char*)usart3_handle_4g.report_buf,"+CCLK:")!=NULL)
            {
							if(strlen((char*)usart3_handle_4g.report_buf) >= 31)
							{
                char *token;
								memset(timer_info.Utc_nowstr,0,16);
								if(strlen((char*)timer_info.Utc_nowstr) == 0)
								{
									token=strtok((char*)usart3_handle_4g.report_buf,"/");
									strcat((char *)timer_info.Utc_nowstr,"20");
									strcat((char *)timer_info.Utc_nowstr,token+10);
									token=strtok(NULL,"/");
									strcat((char *)timer_info.Utc_nowstr,token);
									token=strtok(NULL,",");
									strcat((char *)timer_info.Utc_nowstr,token);
									token=strtok(NULL,":");
									strcat((char *)timer_info.Utc_nowstr,token);
									token=strtok(NULL,":");
									strcat((char *)timer_info.Utc_nowstr,token);
									token=strtok(NULL,"+");
									strcat((char *)timer_info.Utc_nowstr,token);
									timer_info.timestamp = (uint32_t)StringToTimeStamp(timer_info.Utc_nowstr);
									air_4g_flag.get_realtime_flag = TRUE;
								}
							}
            }

            else if(strstr((char*)usart3_handle_4g.report_buf,"restTask\":1")!=NULL)
            {
                control_flag.restTask_flag = TRUE;
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
