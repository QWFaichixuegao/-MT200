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
//uint8_t INDECATOR_LIGHT_IDLE = 0;			// ͬ��֡��־λ
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
            // �������������ϱ�,Ŀǰÿ����300ms
            if(air_4g_flag.MQTT_flag == TRUE /* && control_flag.event_mpub_mutex == FALSE && gps_info.LBSRxFlag == FALSE*/)
            {
                if(control_flag.event_mpub_single_flag == TRUE)
                {
                  air_4g_MPUB_event(WORK_EVENT_MAIN);
                  air_4g_MPUB_event(WORK_EVENT_TRACK);
                  while (huart3.gState != HAL_UART_STATE_READY){}
                  memset(&sing_work_event,0,sizeof(sing_work_event));//�ϱ����������м�¼�ṹ������
                  control_flag.event_mpub_single_flag = FALSE;
                }
                else
                {
                    if(control_flag.Car_State == RUN)
                    {
                        // air_4g_MPUB(control_flag.Car_State);    // ����״̬�·���
                        air_4g_MPUB(RUN);    // ����״̬�·���
                    }
                    else if(control_flag.Car_State == CLOSE && control_flag.sTimer_ms_10000 == Count_10s)	//����״̬����˯��״̬������״̬����ȴ����м�¼�ϱ��¼����
                    {
                        control_flag.sTimer_ms_10000 = 0;
                        // air_4g_MPUB(control_flag.Car_State);    // ˯��״̬�·���
                        air_4g_MPUB(CLOSE);    // ˯��״̬�·���
                    }
                    else if(control_flag.Car_State == OPEN && control_flag.sTimer_ms_5000 == Count_5s)
                    {
                        control_flag.sTimer_ms_5000 = 0;
                        // air_4g_MPUB(control_flag.Car_State);    // ����״̬�·���
                        air_4g_MPUB(OPEN);    // ����״̬�·���
                    }
                    else if(control_flag.Car_State == FAULT && control_flag.sTimer_ms_5000 == Count_5s)
                    {
                        control_flag.sTimer_ms_5000 = 0;
                        // air_4g_MPUB(control_flag.Car_State);    //�쳣״̬�·���
                        air_4g_MPUB(FAULT);    //�쳣״̬�·���
                    }
                    else if(control_flag.Car_State == RECHARGE && control_flag.sTimer_ms_10000 == Count_10s)
                    {
                        control_flag.sTimer_ms_10000 = 0;
                        // air_4g_MPUB(control_flag.Car_State);    // ���״̬�·���
                        air_4g_MPUB(RECHARGE);    // ���״̬�·���
                    }
                    usart1_sbus_tx();// ����״̬����������
                    while (huart1.gState != HAL_UART_STATE_READY){}
                    memset(&sbus_pack_data,0,sizeof(sbus_pack_data));

                    while (huart3.gState != HAL_UART_STATE_READY){}
                    memset(mqtt_pub_inform.PubBuf,0,sizeof(mqtt_pub_inform.PubBuf));

                    //else if(control_flag.Car_State == RECHARGE && control_flag.sTimer_ms_60000 == Count_60s)
                    //{
                    //    control_flag.sTimer_ms_60000 = 0;
                    //    air_4g_MPUB(control_flag.Car_State);    // ���״̬�·���
                    //}
                }
                // vTaskDelay(10);
            }

            //��λ��ÿ�뷢��һ�λ�ȡʵʱʱ��ָ��ֱ��ʱ��У׼�ɹ�
            if(air_4g_flag.get_realtime_flag == FALSE)
            {
              get_real_time(1);
            }

            //ÿ20s��ȡһ��4G��ǰ��Ϣ
            if(air_4g_connect.air820Count == 10)
            {
              air_4g_connect.air820Count = 0;
              get_4G_msg(1);// ��ȡ4G��ǰ��Ϣ(MQTT����״̬��4G�ź�����)
            }

            //����״̬��ÿ120s���һ��GPS����
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

        // ����GPS�ϱ�Ƶ��
        if(gps_info.gpsUrc != gps_info.gpsUrcSet)
        {
            char gpsBuf[20];
            sprintf(gpsBuf, "AT+CGNSURC=%d\r", gps_info.gpsUrcSet);
            air4g_send_cmd("ATE0\r", "OK", 50, air_4g_flag.vTa_delay);
            air4g_send_cmd(gpsBuf, "OK", 100, air_4g_flag.vTa_delay);
            gps_info.gpsUrc = gps_info.gpsUrcSet;
        }

        // ��������
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

        // ����ڰ�ģʽ�½��յ��˰����󣬾͹رհ�ģʽ����ͨ���������ֻ�������Ԫ����Ϣ
        if(control_flag.Car_State == OPEN && control_flag.Bound_flag == TRUE)
        {
            control_flag.Bound_flag 	= FALSE;

            /*
            ������Ԫ����Ϣ����������
            AT+BLECOMM=SENDDATA,fee2,21,7172735F7465737436246768333750317A51646F47
            */
            air4g_send_cmd(ble_inform.dev_send_cmd, "OK", 100, air_4g_flag.vTa_delay);
        }

        // ������MQTT�������Ͽ�����
		if(air_4g_flag.MQTT_flag == FALSE)
		{

            // ����ģʽ�ر�
            for(uint8_t i = 0; i < 5; i++)
            {
                vTaskDelay (1000);
                if(!air4g_send_cmd("ATE0\r", "OK", 500, air_4g_flag.vTa_delay))
                    break;
            }
            air4g_send_cmd("AT+MQTTSTATU\r", "+MQTTSTATU :1", 200, air_4g_flag.vTa_delay);
            // �ٴμ�鳵����MQTT�����������Ƿ�������������Լ��3��
            if(air_4g_flag.MQTT_flag == TRUE)
            {
                air_4g_flag.MQTT_flag = TRUE;
                air_4g_flag.SIM_flag = TRUE;
            }
            else
            {
                /******���SIM�������Ƿ�����*******/

                // ��������ͽӷ�����������������������Լ��3��

                for(uint8_t i = 0; i <= 3; i++)
                {
                    air_4g_closeURC(1);
                    // ��������ͽӷ����������Ӳ�����
                    if(i >= 3)
                    {
                        air_4g_flag.SIM_flag = FALSE;
                        // �����������Ӳ�����������4Gģ������
                        air_4g_restar(air_4g_flag.vTa_delay);
                        break;
                    }
                    if(air_4g_net_check(1) == SIM_OK)
                    {
                        air_4g_flag.SIM_flag = TRUE;
                        if(air_4g_connect_server(0,1) == MQTT_CONNECT_OK)   // MQTT�������Զ��ر�MQTT
                        {
                            // ���������MQTT���������ӳɹ�
                            air_4g_flag.MQTT_flag = TRUE;
                            air_4g_flag.SIM_flag = TRUE;
                            break;
                        }
                    }
                    vTaskDelay(1000);

                }

            }
            // ��鳵����MQTT�������Ƿ����ӣ��״��ϱ�����������Ҫ�ϱ�����Ϣ
            if(air_4g_flag.MQTT_flag == TRUE)
            {
                air_4g_OTAMPUB(1);
                air_4g_OTASUB(1);
                topic_sub(1);

                // ���������ϱ�
                air_4g_openURC(1);
            }
		}

		// ������MQTT��������������
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
  /* Infinite loop *//*------------------------------ң����--------------------------------*/
  for(;;)
  {
        // 100ms�߼�����������
        if(control_flag.cTimer_ms_100 == 1)
        {
            control_flag.cTimer_ms_100=0;

            // ��������״̬��
            car_state_trans();
        }

        // 1s�߼���ɢ�ȷ��ȿ��ƣ����غп��Ǽ��
        if(control_flag.cTimer_ms_1000 == 10)
        {
            control_flag.cTimer_ms_1000=0;

            // ɢ�ȷ��ȿ���
            Boxfan_control();
            Carfan_control();

            // ���Ǽ��
            Check1 = HAL_GPIO_ReadPin(GPIOE,Light_Check1_Pin);
            Check2 = HAL_GPIO_ReadPin(GPIOE,Light_Check2_Pin);
            control_flag.main_kaihe_flag = (!Check1) | (!Check2);

            // ����S48100��ؽ��г��
            if(battery_data.battType == S48100)
            {
                S48100B_TEMP_CHECK();
            }
        }

        // 3s�߼�����ȡ������ݣ��͵����ж�
        if(control_flag.cTimer_ms_3000 == 30)
        {
            control_flag.cTimer_ms_3000 = 0;

            // ��ȡ�������
            batteryReadData();

            // �жϷ��ˮ�ý�ֹ�߼�
            // if(control_flag.Car_State == RUN)
            // {
            //     if(battery_data.soc > BatteryLowSOC_Level1)			// ��������һ����������ֵ
            //     {
            //         battery_data.socState = SOC_ENOUGH;
            //         indicatorLight(LED12VR_LED12VRState);
            //         control_flag.DraughtPumpEnable = TRUE;
            //     }
            //     else if(battery_data.soc <= BatteryLowSOC_Level1 && battery_data.soc > BatteryLowSOC_Level2)		// ��������һ����������ֵ�����ڶ�����������ֵ
            //     {
            //         control_flag.DraughtPumpEnable = TRUE;
            //         battery_data.socState = SOC_LOW_LEVEL_1;
            //         LED12VR_LED12VRState = INDECATOR_LIGHT_ITEM_SOC_LOW_LEVEL_2;
            //         indicatorLight(LED12VL_LED12VLState);
            //     }
            //     else if(battery_data.soc <= BatteryLowSOC_Level2)	// �������ڶ�����������ֵ
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

        // 5s�߼�����ȡ������ݣ���ȡС�������
        if(control_flag.cTimer_ms_5000 == 50)
        {
            control_flag.cTimer_ms_5000 = 0;

            //��ȡ�������������
            read_moto_data();

            //��˳���ȡADCͨ��ֵ
            adc_batt_read();
            adc_intem_read();

        }

        // 120s�߼������ش�ʱ���е͵�������
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

        // ��ʱ����
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
  /* Infinite loop *//*--------------------------���ݽ��մ���TASK--------------------------------*/
  for(;;)
  {
        if(osOK == osSemaphoreWait(BinarySem01Handle,osWaitForever))		// �յ����ڽ����ź���
        {
            //if(usart3_handle_4g.rx_len > SHORT_MESG)
            //{
            // ����ж���GPS�Զ��ϱ������ݣ�����GNSS run status����������ղ����¾�γ����Ϣ���������һ�����⣬����GNSS run status�쳣���Ͳ������GPS��Ϣ��������Ҫ������
            if(strstr((char*)usart3_handle_4g.report_buf,"+UGNSINF: 1") != NULL )
            {
                char *token;

                // strtok����ֱ��ȡ�ڶ���","֮ǰ���ַ���
                token = strtok((char*)usart3_handle_4g.report_buf, ",");

                // ��ȡGPS�ź�
                gps_info.gps_signal_flag = *strtok(NULL, ",");	// �����־��GPS�Ƿ��ȡ���ˣ�ʵʱ����


                // ��ȡGPS
								token = strtok(NULL,",");//�ָ�һ�ζ��ţ���ʱ�䲻�ڴ˻�ȡ
                if(gps_info.gps_signal_flag == 0x31 )		    // �ڶ������ŵ���һ������֮���ֵ��ʾGPS���޻�ȡ   0x31����
                {
                    token = strtok(NULL, ",");//��ȡγ��

                    //if(token[0] > 0x2f && token[0] < 0x3a)
                    if(token[0] > 0x30 && token[0] < 0x3a)
                    {
                        strcpy((char *)gps_info.Lat_nowstr, token);//����γ��

                        token = strtok(NULL, ",");//��ȡγ��
                        strcpy((char *)gps_info.Lon_nowstr,token);//����γ��

                        token = strtok(NULL, ".");//��ȡ�߳�
                        gps_info.MSL_Altitude = atoi(token);
                        // strcpy((char *)gps_info.Msl_nowstr,token);//�����߳�

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
                    gps_info.gps_signal_flag=0;	//�����־��GPS�Ƿ��ȡ����
                }
            }

						// ����MQTT����״̬
            else if(strstr((char*)usart3_handle_4g.report_buf,"+MQTTSTATU :1")!=NULL)
            {
							air_4g_flag.MQTT_flag = TRUE;
            }
            else if(strstr((char*)usart3_handle_4g.report_buf,"+MQTTSTATU :0")!=NULL)
            {
							air_4g_flag.MQTT_flag = FALSE;
            }

						// ����SIM���ź�����
            //else if(strstr((char*)usart3_handle_4g.report_buf,"+CSQ:")!=NULL)
            else if(strstr((char*)usart3_handle_4g.report_buf,"+CSQ:")!=NULL)
            {
                char *token;
                token=strtok((char*)usart3_handle_4g.report_buf,":");
                token=strtok(NULL,",");
                air_4g_connect.sim_CSQ = atoi(token);
            }

			// �ϱ���������ۼ� 58��ʶ��ָ������100��
            else if(strstr((char*)usart3_handle_4g.report_buf,"ERROR: 58")!=NULL)
            {
				if(control_flag.error_res_count<100) {
                    control_flag.error_res_count++;
                }
                else{
                    control_flag.error_res_flag = TRUE;
                }
            }

            // �ڰ�ģʽģʽ��
            else if((control_flag.Match_flag == TRUE) && (strstr((char*)usart3_handle_4g.report_buf,"+BLEIND=DATA")!=NULL))
            {
                /*
                ����״̬�ϱ�
                +BLEIND=CONNECT,"73:80:b7:3f:3c:12"
                ���յ�����
                +BLEIND=DATA,"fee1",3,"303124"
                ���յ�����
                +BLEIND=DATA,"fee1",3,"303324"
                */
                if(strstr((char*)usart3_handle_4g.report_buf,"303124")!=NULL )
                {
                    control_flag.Bound_flag = TRUE;
                }
                else if(strstr((char*)usart3_handle_4g.report_buf,"303224")!=NULL )
                {
                    speakItem(SPEAK_ITEM_BIND_FAIL);			//������������ʧ�ܡ�
                }
                else if(strstr((char*)usart3_handle_4g.report_buf,"303324")!=NULL )
                {
                    speakItem(SPEAK_ITEM_BIND_SECCEEED);		//�����������󶨳ɹ���
                }
            }
            //����ʱУ׼һ��ʵʱʱ��
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
	//-----------------------------------��ʱ�����ϱ�&���ݴ���TASK-----------------------------------//
	uint32_t Prewaketime = osKernelSysTick();

  for(;;)
    {
        // ע��1s����һ�λ�����
        send_moto_cmd(MOTO1_HALL, MOTO_REGIS_NUM2, 0, MOTO_ReadREQ_ID);

        // ����״̬�������ʱ��
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



       // ������ʱ�¿��Ź�4Sι��
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

    // 1S��ʱ
    if(control_flag.sTimer_ms_1000 < Count_1s)
    {
		control_flag.sTimer_ms_1000++;
	}

    // 5S��ʱ
	if(control_flag.sTimer_ms_5000 < Count_5s)
    {
		control_flag.sTimer_ms_5000++;
	}

    // 10S��ʱ
	if(control_flag.sTimer_ms_10000 < Count_10s)
    {
		control_flag.sTimer_ms_10000++;
	}

    // 60S��ʱ
    if(control_flag.sTimer_ms_60000 < Count_60s)
    {
		control_flag.sTimer_ms_60000++;
	}

  /* USER CODE END Callback01 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
