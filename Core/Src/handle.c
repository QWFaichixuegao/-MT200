#include "handle.h"
#include "air820.h"
//#include "rtc.h"
#include "cmsis_os.h"
#include "can.h"
#include "at24cxx.h"
#include "can_device.h"
#include "at24cxx.h"
#include "CANopen_Master_M200.h"
#include "canfestival.h"

//#define QRS_TEST_INVERT1_6     //���Գ�1��ʶ��

// CANopen�����ֵ䶨��
extern CO_Data CANopen_Master_M200_Data;

// ң��������
USARTX_SBUS 		usart6_sbus;

// �������ݶ���
CONTROL_FLAG 		control_flag;

// CANͨѶ����
CAN_READ_DATA		can_read_data;
CAN_HANDLE			can_handle;
CAR_MOTO 			car_moto;

// ��������
SUMMARY_DATA 		summary_data;
COUNT_TIME 			count_time;
EEPROM_PACK 		eeprom_packet;

// ң��������
volatile    uint16_t Remote[16];
static      uint8_t subs2dbus 	 = 0;


uint16_t BLElen;//Ҫ�����������ݵĳ���(����С��244)    		// DeviceName;ProductKey


// ��ȡ��λԴ
void getSysReseySource(void)
{
    // �����ѹ���ڷ�ֵ�����ĸ�λ   POR/PDR or BOR reset  //Power-on/power-down reset (POR/PDR reset) or brownout (BOR)
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_BORRST)== SET)
    {
        SETBIT(control_flag.resetSource, 0);
    }

    // RESET�ܽŲ����ĸ�λ
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_PINRST)== SET)
    {
        SETBIT(control_flag.resetSource, 1);
    }

    // �ϵ縴λ����������
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_PORRST)== SET)
    {
        SETBIT(control_flag.resetSource, 2);
    }

    // ������������ĸ�λ
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST)== SET)
    {
        SETBIT(control_flag.resetSource, 3);
    }

    // �������Ź������ĸ�λ
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST)== SET)
    {
        SETBIT(control_flag.resetSource, 4);
    }

    // ���ڿ��Ź������ĸ�λ
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_WWDGRST)!= RESET)
    {
        SETBIT(control_flag.resetSource, 5);
    }

    // �͹��Ĳ����ĸ�λ
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_LPWRRST )!= RESET)
    {
        SETBIT(control_flag.resetSource, 6);
    }
}

// �������Ʋ�����ʼ��
void control_init(void)
{
    // ����״̬
    control_flag.Car_State    			        = CLOSE;
    control_flag.Init_flag                      = TRUE;

    // ������
    control_flag.Bound_flag					    = FALSE;
    control_flag.Match_flag					    = FALSE;

    // ң����
    control_flag.Sbus_lock_flag			        = FALSE;		// ң����������־λ
    control_flag.Sbus_connect_flag              = FALSE;		// ң�������ӱ�־λ


    // �����豸��ʼ��
    control_flag.Led48v_swith				    = FALSE;		// ����ǰ��ƿ��ƿ���
    control_flag.Boxfan_swith				    = FALSE;		// ����������ȿ��ƿ���
    control_flag.Carfan_swith				    = FALSE;		// ����������ȿ��ƿ���
    softSwitch_switchFlag 			     		= FALSE;

    // ������ҵ
    control_flag.Draught_swith			        = FALSE;		// ������ƿ���
    control_flag.Pump_swith			  	        = FALSE;
    control_flag.DraughtPumpEnable				= TRUE;
    control_flag.speed_notific_flag	            = FALSE;
    control_flag.Auto_spray_swith		        = FALSE;
    control_flag.Auto_swith					    = FALSE;
    control_flag.Auto_gear_swith		        = G_0;

    driverBoard_draughtFanspeed					= 0;
    driverBoard_pumpSpeed						= 0;


    // �˶�����
    control_flag.up_gear_flag				    = FALSE;
    control_flag.down_gear_flag			    = FALSE;
		control_flag.Auto_backflag1					= FALSE;
		control_flag.Auto_backflag2					= FALSE;
		control_flag.lock_check_again 			= FALSE;

    // 4Gģ��
    //control_flag.Usart3_handle_flag             = TRUE;       // ����3����4Gģ���·�����Ƭ��������
    control_flag.event_mpub_mutex               = FALSE;        // �������ϱ��������ر�
    control_flag.event_mpub_single_flag         = FALSE;        // �������ϱ����ι���ʱ���־λ�ڹرպ���Ϣ�ϱ���ѭ�����ϴ�һ��

    gps_info.gps_signal_flag				    = FALSE;

    air_4g_flag.hal_delay 					    = (bool)FALSE;
		air_4g_flag.vTa_delay					    = (bool)TRUE;
		air_4g_flag.SIM_flag 					    = (bool)FALSE;
		air_4g_flag.MQTT_flag	 				    = (bool)FALSE;
		air_4g_flag.send_version_flag 	            = (bool)TRUE;
		air_4g_flag.get_realtime_flag			= FALSE;												//������ʼУ׼ʵʱʱ��
		air_4g_connect.sim_ccount 	 			    = 0;
		air_4g_connect.mqtt_ccount	 			    = 0;



    // ��Դ
    battery_data.charge_flag				    = FALSE;        // ����ʶ
    battery_data.charge_overflag                = DISABLE;
    driverBoard_scramStop						= FALSE;        // ��ͣ��ʶ
    driverBoard_contactorState					= 0;		    // �Ӵ���Ĭ�϶Ͽ�

    gps_info.gpsUrc                             = 10;
    gps_info.gpsUrcSet                          = 10;

    ble_inform.ble_switch_state                 = FALSE;
    ble_inform.ble_switch                       = FALSE;


    can_handle.RxFlag							= FALSE;	    // CAN���ձ�־λ
    control_flag.Iwdg_count						= 0;
    control_flag.error_res_count				= 0;
    control_flag.error_res_flag                 = FALSE;




    //control_flag.DraughtPumpEnableRecover     = FALSE;	    // ����״̬��û�й��͵�����λ
    //control_flag.Highest_autho_flag           = TRUE; 		// ����״̬�£��ȴ����Ȩ��
    //control_flag.Sbus_midlock_flag	        = FALSE;		// ң�����������б�־λ

    //driverBoard_carLight						= 0;
    //driverBoard_carFanSpeed					= 0;
    //driverBoard_boxFanSpeed					= 0;
}

/*************************************************************�������м�¼����***********************************************************/

// �����������ݳ�ʼ��
void device_init(void)
 {

	strcpy(device_inform.version,"1.1.7");//д��İ汾��
	AT24CXX_WriteData(0x0180, (uint8_t*)device_inform.version, 5);	//���汾��д��EEPROM



	/*��EEPROM�ж�ȡ�豸��Ϣ*/
	HAL_I2C_Mem_Read_DMA(&hi2c1, R_IIC_ADDR_AT24CXX, 0x0000, I2C_MEMADD_SIZE_16BIT, (uint8_t*)device_inform.DeviceName, 32);
	while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY){}

	HAL_I2C_Mem_Read_DMA(&hi2c1, R_IIC_ADDR_AT24CXX, 0x0020, I2C_MEMADD_SIZE_16BIT, (uint8_t*)device_inform.ProductKey, 32);
	while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY){}

	HAL_I2C_Mem_Read_DMA(&hi2c1, R_IIC_ADDR_AT24CXX, 0x0080, I2C_MEMADD_SIZE_16BIT, (uint8_t*)device_inform.Password, 	128);
	while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY){}

	HAL_I2C_Mem_Read_DMA(&hi2c1, R_IIC_ADDR_AT24CXX, 0x0100, I2C_MEMADD_SIZE_16BIT, (uint8_t*)device_inform.server_host,64);
	while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY){}

	HAL_I2C_Mem_Read_DMA(&hi2c1, R_IIC_ADDR_AT24CXX, 0x0140, I2C_MEMADD_SIZE_16BIT, (uint8_t*)device_inform.server_port,8);
	while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY){}

	HAL_I2C_Mem_Read_DMA(&hi2c1, R_IIC_ADDR_AT24CXX, 0x0148, I2C_MEMADD_SIZE_16BIT, (uint8_t*)device_inform.timestamp,  16);
	while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY){}

	HAL_I2C_Mem_Read_DMA(&hi2c1, R_IIC_ADDR_AT24CXX, 0x0185, I2C_MEMADD_SIZE_16BIT, &device_inform.upgrade_flag,  		1);
	while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY){}

	sprintf(mqtt_pub_inform.theme_str,"/sys/%s/%s/thing/event/property/post",		        //�豸�����ϱ�����ģ��
	        device_inform.ProductKey, device_inform.DeviceName);

	sprintf(mqtt_pub_inform.work_event_Main,"/sys/%s/%s/thing/event/runRecordMain/post",	//�豸�¼��ϱ�
	        device_inform.ProductKey,device_inform.DeviceName);

	sprintf(mqtt_pub_inform.work_event_Track,"/sys/%s/%s/thing/event/runRecordTrack/post",	//�豸�¼��ϱ�
	        device_inform.ProductKey,device_inform.DeviceName);

	sprintf(mqtt_ota_inform.version_theme_str,"/ota/device/inform/%s/%s",	                //�豸�ϱ��̼�������Ϣ
	        device_inform.ProductKey, device_inform.DeviceName);

	sprintf(mqtt_ota_inform.request_theme_str,"/sys/%s/%s/thing/ota/firmware/get",	        //�豸������ȡ�̼�������Ϣ
	        device_inform.ProductKey, device_inform.DeviceName);

	sprintf(ota_inform.msubOTA_theme_str,"/sys/%s/%s/thing/ota/firmware/get_reply",
					device_inform.ProductKey, device_inform.DeviceName);

	mqtt_pub_inform.liquidLevelSensor	= 10;
	mqtt_pub_inform.vehicleSpeed		= 123;
	mqtt_pub_inform.fanMachinery		= 10;
	mqtt_pub_inform.motorWaterPump		= 6;
	mqtt_pub_inform.rundata_turn_count	= 0;
	mqtt_pub_inform.start_summary_flag  = DISABLE;		// ����Ĭ�ϲ����ϱ�������Ϣ
	mqtt_pub_inform.start_charge_flag   = DISABLE;

	sing_work_event.power				= 0;//--
	sing_work_event.drug				= 0;//--
	sing_work_event.save_track_count	= 0;
	sing_work_event.mileage				= 0;

	battery_init();


	/*-BLE-*/

	sprintf(ble_inform.ble_name, "AT+BLECOMM=NAME,MQ_%s\r", device_inform.DeviceName);
	char data4[2];//ÿ��ת��16���Ƶ��ַ� ����λ�͵���λ�ֱ�ռ�����ֽ�
	sprintf(ble_inform.dev_bound_data, "%s$%s", device_inform.DeviceName,device_inform.ProductKey);
	BLElen=strlen(ble_inform.dev_bound_data);
	for(uint8_t i = 0; i<BLElen; i++)
		{
			sprintf(data4, "%2X", ble_inform.dev_bound_data[i]);//��ÿ���ַ�ת��16���Ƹ���λ�͵���λ�ֱ�ռ�����ֽ�(%2X)
			//strncat(data6, data4, 2);
			strcat (ble_inform.dev_data_To16, data4);
		}
	sprintf(ble_inform.dev_send_cmd, "AT+BLECOMM=SENDDATA,fee2,%d,%s\r",BLElen , ble_inform.dev_data_To16);
}

// ��������ʱ���¼
void counttime_handle(void)
{
	summary_data.totalRunTime += count_time.countRunTime;
	summary_data.totalWaterPumpTime += count_time.countWaterPumpTime;
	summary_data.totalFanMachineryTime += count_time.countFanMachineryTime;

	for(uint8_t i=0;i<4;i++)
	{
		eeprom_packet.RunTime[i]				  = 0xff&(summary_data.totalRunTime>>(i*8));
		eeprom_packet.WaterPumpTime[i] 		= 0xff&(summary_data.totalWaterPumpTime>>(i*8));
		eeprom_packet.FanMachineryTime[i] = 0xff&(summary_data.totalFanMachineryTime>>(i*8));
	}

	AT24CXX_WriteData(0x0200, eeprom_packet.RunTime,	4);
	AT24CXX_WriteData(0x0208, eeprom_packet.WaterPumpTime,  	4);
	AT24CXX_WriteData(0x020c, eeprom_packet.FanMachineryTime,  		4);

	count_time.countRunTime = 0;
	count_time.countWaterPumpTime = 0;
	count_time.countFanMachineryTime = 0;
}

// ����������̼�¼
void countdata_handle(void)
{
	summary_data.totalMileage += sing_work_event.mileage;
	for(uint8_t i=0;i<4;i++)
	{
		eeprom_packet.Mileage[i]				  = 0xff&(summary_data.totalMileage>>(i*8));
	}
	AT24CXX_WriteData(0x0204, eeprom_packet.Mileage,	4);
}


// ����������ȡ��������
void countdata_read(void)
{
	HAL_I2C_Mem_Read_DMA(&hi2c1, R_IIC_ADDR_AT24CXX, 0x0200, I2C_MEMADD_SIZE_16BIT, (uint8_t*)device_inform.RunTime,					4);
	while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY){}

	HAL_I2C_Mem_Read_DMA(&hi2c1, R_IIC_ADDR_AT24CXX, 0x0204, I2C_MEMADD_SIZE_16BIT, (uint8_t*)device_inform.Mileage,					4);
	while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY){}

	HAL_I2C_Mem_Read_DMA(&hi2c1, R_IIC_ADDR_AT24CXX, 0x0208, I2C_MEMADD_SIZE_16BIT, (uint8_t*)device_inform.WaterPumpTime,  	4);
	while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY){}

	HAL_I2C_Mem_Read_DMA(&hi2c1, R_IIC_ADDR_AT24CXX, 0x020c, I2C_MEMADD_SIZE_16BIT, (uint8_t*)device_inform.FanMachineryTime, 4);
	while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY){}

	summary_data.totalRunTime				 		= (device_inform.RunTime[3]<<24)|(device_inform.RunTime[2]<<16)|(device_inform.RunTime[1]<<8)|device_inform.RunTime[0];
	summary_data.totalMileage 					= (device_inform.Mileage[3]<<24)|(device_inform.Mileage[2]<<16)|(device_inform.Mileage[1]<<8)|device_inform.Mileage[0];
	summary_data.totalWaterPumpTime 		= (device_inform.WaterPumpTime[3]<<24)|(device_inform.WaterPumpTime[2]<<16)|(device_inform.WaterPumpTime[1]<<8)|device_inform.WaterPumpTime[0];
	summary_data.totalFanMachineryTime 	= (device_inform.FanMachineryTime[3]<<24)|(device_inform.FanMachineryTime[2]<<16)|(device_inform.FanMachineryTime[1]<<8)|device_inform.FanMachineryTime[0];
}

/*************************************************************ң�������ݽ���**************************************************************/

//ң�������ݽ���
void subus_read(void)
{
    subs2dbus=1;
    Remote[SBUS_X2] = ((int16_t)usart6_sbus.rx_buf[ 0+subs2dbus] >> 0 | ((int16_t)usart6_sbus.rx_buf[ 1+subs2dbus] << 8 )) & 0x07FF;
    Remote[SBUS_Y2] = ((int16_t)usart6_sbus.rx_buf[ 1+subs2dbus] >> 3 | ((int16_t)usart6_sbus.rx_buf[ 2+subs2dbus] << 5 )) & 0x07FF;
    Remote[SBUS_Y2] = SBUS_zhongzhi+SBUS_zhongzhi-Remote[SBUS_Y2];
    Remote[SBUS_Y1] = ((int16_t)usart6_sbus.rx_buf[ 2+subs2dbus] >> 6 | ((int16_t)usart6_sbus.rx_buf[ 3+subs2dbus] << 2 )  | (int16_t)usart6_sbus.rx_buf[ 4+subs2dbus] << 10 ) & 0x07FF;
    Remote[SBUS_X1] = ((int16_t)usart6_sbus.rx_buf[ 4+subs2dbus] >> 1 | ((int16_t)usart6_sbus.rx_buf[ 5+subs2dbus] << 7 )) & 0x07FF;
    Remote[SBUS_E]  = ((int16_t)usart6_sbus.rx_buf[ 5+subs2dbus] >> 4 | ((int16_t)usart6_sbus.rx_buf[ 6+subs2dbus] << 4 )) & 0x07FF;
    Remote[SBUS_F]  = ((int16_t)usart6_sbus.rx_buf[ 6+subs2dbus] >> 7 | ((int16_t)usart6_sbus.rx_buf[ 7+subs2dbus] << 1 )  | (int16_t)usart6_sbus.rx_buf[8+subs2dbus] <<  9 ) & 0x07FF;
    Remote[SBUS_A]  = ((int16_t)usart6_sbus.rx_buf[ 8+subs2dbus] >> 2 | ((int16_t)usart6_sbus.rx_buf[ 9+subs2dbus] << 6 )) & 0x07FF;
    Remote[SBUS_B]  = ((int16_t)usart6_sbus.rx_buf[ 9+subs2dbus] >> 5 | ((int16_t)usart6_sbus.rx_buf[10+subs2dbus] << 3 )) & 0x07FF;
    Remote[SBUS_C]  = ((int16_t)usart6_sbus.rx_buf[11+subs2dbus] >> 0 | ((int16_t)usart6_sbus.rx_buf[12+subs2dbus] << 8 )) & 0x07FF;
    Remote[SBUS_D]  = ((int16_t)usart6_sbus.rx_buf[12+subs2dbus] >> 3 | ((int16_t)usart6_sbus.rx_buf[13+subs2dbus] << 5 )) & 0x07FF; 		// 282-1722
    // T10 10ͨ��
     Remote[10] = ((int16_t)usart6_sbus.rx_buf[13+subs2dbus] >> 6 | ((int16_t)usart6_sbus.rx_buf[14+subs2dbus] << 2 )  | (int16_t)usart6_sbus.rx_buf[15+subs2dbus] <<  10 ) & 0x07FF;
     Remote[11] = ((int16_t)usart6_sbus.rx_buf[15+subs2dbus] >> 1 | ((int16_t)usart6_sbus.rx_buf[16+subs2dbus] << 7 )) & 0x07FF;
    // Remote[12] = ((int16_t)usart6_sbus.rx_buf[16+subs2dbus] >> 4 | ((int16_t)usart6_sbus.rx_buf[17+subs2dbus] << 4 )) & 0x07FF;
    // Remote[13] = ((int16_t)usart6_sbus.rx_buf[17+subs2dbus] >> 7 | ((int16_t)usart6_sbus.rx_buf[18+subs2dbus] << 1 )  | (int16_t)usart6_sbus.rx_buf[19+subs2dbus] <<  9 ) & 0x07FF;
    // Remote[14] = ((int16_t)usart6_sbus.rx_buf[19+subs2dbus] >> 2 | ((int16_t)usart6_sbus.rx_buf[20+subs2dbus] << 6 )) & 0x07FF;
    // Remote[15] = ((int16_t)usart6_sbus.rx_buf[20+subs2dbus] >> 5 | ((int16_t)usart6_sbus.rx_buf[21+subs2dbus] << 3 )) & 0x07FF;

    //�жϽ��ջ��Ƿ���������
    if(usart6_sbus.rx_buf[23] != 0x00)
    {
        control_flag.Sbus_connect_flag = FALSE;
    }
    else
    {
        control_flag.Sbus_connect_flag = TRUE;
    }

    // ����Զ���ģʽ����
    if(control_flag.Auto_swith == ENABLE)		//A�������Զ���
    {
			if(Remote[SBUS_Y2]>SBUS_up_onofflimit && control_flag.up_gear_flag == FALSE)
			{
				control_flag.up_gear_flag = TRUE;			// ����
				if(control_flag.Auto_gear_swith < G_3)
				{
						control_flag.Auto_gear_swith++;
				}
				control_flag.Auto_backflag2	= FALSE;      //�Զ����״ι��㵵����������
			}
			else if(Remote[SBUS_Y2]<SBUS_lw_onofflimit)
			{
				control_flag.down_gear_flag = TRUE;		    // ����
			}


			if(Remote[SBUS_Y2] < SBUS_mid_R_limit && Remote[SBUS_Y2] > SBUS_mid_L_limit)	// ң����λ����50������гɹ�
			{
				control_flag.up_gear_flag 		= FALSE;
				if(control_flag.Auto_backflag1 == TRUE)
				{
					control_flag.Auto_backflag2	= TRUE;
					control_flag.Auto_backflag1 = FALSE;
				}
			}

			if(control_flag.down_gear_flag == TRUE)	// ��������λֱ�ӽ�ΪG_0��
			{
				control_flag.down_gear_flag 		= FALSE;
				control_flag.Auto_gear_swith		= G_0;
				control_flag.Auto_backflag1			= TRUE;
			}
    }

    /******����Ѿ�����******/
    if(control_flag.Sbus_lock_flag == TRUE)
    {
        // Sbus������Ʊ�־λ
        if(Remote[SBUS_C] >= SBUS_up_limit | Remote[SBUS_F] <= SBUS_lw_limit)
        {
            // ����ٶȹر�
            control_flag.Draught_swith = FALSE;
        }
        else if(Remote[SBUS_C] <= SBUS_up_limit && Remote[SBUS_F] >= SBUS_lw_limit)
        {
            // �����
            control_flag.Draught_swith = TRUE;
        }

        // Sbusˮ�ÿ��Ʊ�־λ
        if(Remote[SBUS_F] <= SBUS_lw_limit)
        {
            // ˮ�ùر�
            control_flag.Pump_swith = G_0;
        }
        else if(Remote[SBUS_F] == SBUS_zhongzhi)
        {
            // ˮ��1��
            control_flag.Pump_swith = G_1;
        }
        else if(Remote[SBUS_F] >= SBUS_up_limit)
        {
            // ˮ��2��
            control_flag.Pump_swith = G_2;
        }
    }

		// Sbus��ƿ��Ʊ�־λ
		if(Remote[SBUS_Y1]>SBUS_up_onofflimit && Remote[SBUS_X1]<SBUS_mid_R_limit && Remote[SBUS_X1]>SBUS_mid_L_limit)
		{
				// �򿪴��
				control_flag.Led48v_swith=TRUE;
		}
		else if(Remote[SBUS_Y1]<SBUS_lw_onofflimit && Remote[SBUS_X1]<SBUS_mid_R_limit && Remote[SBUS_X1]>SBUS_mid_L_limit)
		{
				// �رմ��
				control_flag.Led48v_swith=FALSE;
		}

    // �����󶨱�־λ
    if(Remote[SBUS_E] >= SBUS_up_limit )
    {
        // �ر������Ӧ
        control_flag.Match_flag     = FALSE;

        // ң��������
        control_flag.Sbus_lock_flag = FALSE;
        control_flag.lock_check_again = TRUE;
    }
    else if(Remote[SBUS_E] == SBUS_zhongzhi)
    {
        // �������Ӧ
        control_flag.Match_flag     = TRUE;

        // ң��������
        control_flag.Sbus_lock_flag = FALSE;

    }
    else if(Remote[SBUS_E] <= SBUS_lw_limit)
    {
        // �ر������Ӧ
        control_flag.Match_flag = FALSE;

				if(control_flag.lock_check_flag ==TRUE)
				{
					if(Remote[SBUS_F] <= SBUS_lw_limit)//ˮ�ÿ��عرպ���ܽ���
					{
						if(control_flag.lock_check_again ==TRUE)
						{
							control_flag.lock_check_flag = FALSE;
							// ң��������
							control_flag.Sbus_lock_flag = TRUE;
                            control_flag.lock_check_again = FALSE;
						}
					}
					else
					{
						control_flag.lock_check_again = FALSE;//��������Ҫ���²�������
					}

				}

    }

    // Sbus�Զ������Ʊ�־λ
    if(Remote[SBUS_A] >= SBUS_up_limit)
    {
        control_flag.Auto_swith = ENABLE;       //����ģʽ
        CLRBIT(mqtt_pub_inform.runvehicleStatus,4);
        SETBIT(mqtt_pub_inform.runvehicleStatus,5);
    }
    else if(Remote[SBUS_A] <= SBUS_lw_limit)
    {
        control_flag.Auto_swith = DISABLE;      //�ֶ�ģʽ
        CLRBIT(mqtt_pub_inform.runvehicleStatus,5);
        SETBIT(mqtt_pub_inform.runvehicleStatus,4);
    }


    // Sbus�����Զ���ͣ���Ʊ�־λ
    if(Remote[SBUS_B] >= SBUS_up_limit)
    {
        // �Զ���ͣ���ܴ�
        control_flag.Auto_spray_swith = TRUE;
        CLRBIT(mqtt_pub_inform.runvehicleStatus,9);
        SETBIT(mqtt_pub_inform.runvehicleStatus,8);
    }
    else if(Remote[SBUS_B] <= SBUS_lw_limit)
    {
        control_flag.Auto_spray_swith = FALSE;
        CLRBIT(mqtt_pub_inform.runvehicleStatus,8);
        SETBIT(mqtt_pub_inform.runvehicleStatus,9);
    }
}

/*************************************************************����ִ��������**************************************************************/

// 48V��ƿ���
void Led48v_control(void)
{
    // �����Ȳ��ɿأ���д���������
    if(control_flag.Led48v_swith==TRUE)
    {
        driverBoard_carLight = 100;

        // ��λAPP�������
        SETBIT(mqtt_pub_inform.runvehicleStatus,15);
    }
    else
    {
        driverBoard_carLight = 0;

        // ���APP�������
        CLRBIT(mqtt_pub_inform.runvehicleStatus,15);
    }
    WriteSDO(&CANopen_Master_M200_Data, CANDevice_NodeID_driver, DRIVER_INDEX_CAR_LIGHT, DRIVER_SUB_INDEX_DEFAULT, 1, uint8, &driverBoard_carLight, 0);
}

// ������ȿ���
void Boxfan_control(void)
{
    if(control_flag.Boxfan_swith==TRUE)
    {
        driverBoard_boxFanSpeed = 100;
    }
    else
    {
        driverBoard_boxFanSpeed = 0;
    }
    WriteSDO(&CANopen_Master_M200_Data, CANDevice_NodeID_driver, DRIVER_INDEX_BOX_FAN_SPEED, DRIVER_SUB_INDEX_DEFAULT, 1, uint8, &driverBoard_boxFanSpeed, 0);
}

// ������ȿ���
void Carfan_control(void)
{
    if(control_flag.Carfan_swith==TRUE)
    {
        driverBoard_carFanSpeed = 100;
    }
    else
    {
        driverBoard_carFanSpeed = 0;
    }
    WriteSDO(&CANopen_Master_M200_Data, CANDevice_NodeID_driver, DRIVER_INDEX_CAR_FAN_SPEED, DRIVER_SUB_INDEX_DEFAULT, 1, uint8, &driverBoard_carFanSpeed, 0);
}

// �������
void Draught_control(void)
{
    uint16_t draughtSpeedVar;
    //if(control_flag.Draught_swith == FALSE | control_flag.Draught_open_flag == FALSE | control_flag.DraughtPumpEnable == FALSE)
    if(control_flag.Draught_swith == FALSE | control_flag.DraughtPumpEnable == FALSE)
    {

        mqtt_pub_inform.fanMachinery    = 0;
        driverBoard_draughtFanspeed     = 0;

        // ���APP�������
        CLRBIT(mqtt_pub_inform.runvehicleStatus,13);
    }
    else
    {
        draughtSpeedVar = 29 + (SBUS_zhongzhi-Remote[SBUS_C]) * 0.04;			//		0~57
        mqtt_pub_inform.fanMachinery    =   (uint8_t)(draughtSpeedVar / 57.0 * 100.0);
        driverBoard_draughtFanspeed     =   draughtSpeedVar;

        // ��λAPP�������
        SETBIT(mqtt_pub_inform.runvehicleStatus,13);
    }
    WriteSDO(&CANopen_Master_M200_Data, CANDevice_NodeID_driver, DRIVER_INDEX_DRAUGHT_FAN_SPEED, DRIVER_SUB_INDEX_DEFAULT, 1, uint8, &driverBoard_draughtFanspeed, 0);
}

// ˮ�ÿ���
void Pump_control(void)
{
    uint16_t G_1_var,G_2_var;
    if(control_flag.DraughtPumpEnable == FALSE | Remote[SBUS_D] <= 290)
    {
        driverBoard_pumpSpeed =0;

        // ���APP����ˮ��
        CLRBIT(mqtt_pub_inform.runvehicleStatus,12);
    }
    else
    {
        switch(control_flag.Pump_swith)
        {
            case G_1:
                G_1_var = (uint16_t)((Remote[SBUS_D]-SBUS_MIN) * 39 / (SBUS_MAX - SBUS_MIN) + 20);	// 20-59
                driverBoard_pumpSpeed = G_1_var;

                // ��λAPP����ˮ��
                SETBIT(mqtt_pub_inform.runvehicleStatus,12);
                break;

            case G_2:
                G_2_var = (uint16_t)((Remote[SBUS_D]-SBUS_MIN) * 59 / (SBUS_MAX - SBUS_MIN) + 40);	// 40-99
                driverBoard_pumpSpeed = G_2_var;

                // ��λAPP����ˮ��
                SETBIT(mqtt_pub_inform.runvehicleStatus,12);
                break;

            case G_0:
                driverBoard_pumpSpeed = 0;

                // ���APP����ˮ��
                CLRBIT(mqtt_pub_inform.runvehicleStatus,12);
                break;
        }
    }
    WriteSDO(&CANopen_Master_M200_Data, CANDevice_NodeID_driver, DRIVER_INDEX_PUMP_SPEED, DRIVER_SUB_INDEX_DEFAULT, 1, uint8, &driverBoard_pumpSpeed, 0);
}

/*************************************************************�����˶�����**************************************************************/

// ���������ֶ�����ģʽ
void hunkong(uint16_t X2, uint16_t Y2)
{
    // ��λAPP����ͼ�Ĵ��˶�
    SETBIT(mqtt_pub_inform.runvehicleStatus,14);

    control_flag.speed_notific_flag = TRUE;
    //��
    car_moto.x = X2-SBUS_zhongzhi;
    car_moto.y = Y2-SBUS_zhongzhi;
    //
    car_moto.cos = car_moto.x / sqrt(car_moto.x*car_moto.x+car_moto.y*car_moto.y);
    car_moto.sin = car_moto.y / sqrt(car_moto.x*car_moto.x+car_moto.y*car_moto.y);

    if(car_moto.cos<0)
    {
        car_moto.cos=-car_moto.cos;
    }
    if(car_moto.sin<0)
    {
        car_moto.sin=-car_moto.sin;
    }

    // ǰ������
    if(car_moto.x==0&&car_moto.y!=0)
    {
        #ifdef QRS_TEST_INVERT1_6			                // ���Գ�
            car_moto.car_left = car_moto.y;
            car_moto.car_right= -car_moto.y;
        #else												// ��ʽ��
            car_moto.car_left = car_moto.y;
            car_moto.car_right= -car_moto.y;
        #endif
    }

    // ԭ��ת

    else if(car_moto.x!=0&&car_moto.y>-SBUS_siqu&&car_moto.y<SBUS_siqu)
    {
        #ifdef QRS_TEST_INVERT1_6			                // ���Գ�
            car_moto.car_left = car_moto.x*TURN_VAR;
            car_moto.car_right= car_moto.x*TURN_VAR;
        #else												// ��ʽ��
            car_moto.car_left = -car_moto.x*TURN_VAR;
            car_moto.car_right= -car_moto.x*TURN_VAR;
        #endif
        control_flag.speed_notific_flag = FALSE;
    }

    // ��ת
    else if(car_moto.x>0&&(car_moto.y<-SBUS_siqu|car_moto.y>SBUS_siqu))
    {
        #ifdef QRS_TEST_INVERT1_6			                // ���Գ�
            car_moto.car_left = car_moto.y;
            car_moto.car_right= -car_moto.y*car_moto.sin;
        #else												// ��ʽ��
            car_moto.car_left = car_moto.y*car_moto.sin;
            car_moto.car_right= -car_moto.y;
        #endif
    }

    // ��ת
    else if(car_moto.x<0&&(car_moto.y<-SBUS_siqu|car_moto.y>SBUS_siqu))
    {
        #ifdef QRS_TEST_INVERT1_6			                // ���Գ�
            car_moto.car_left = car_moto.y*car_moto.sin;
            car_moto.car_right=	-car_moto.y;
        #else												// ��ʽ��
            car_moto.car_left = car_moto.y;
            car_moto.car_right=	-car_moto.y*car_moto.sin;
        #endif
    }

    // ֹͣ
    else
    {
        car_moto.car_left  = 0;
        car_moto.car_right = 0;

        // ���APP����ͼ�Ĵ��˶�
        CLRBIT(mqtt_pub_inform.runvehicleStatus,14);
        control_flag.speed_notific_flag = FALSE;
    }

    // �л�ָʾ����ʾ״̬
    indicatorLight(INDECATOR_LIGHT_ITEM_MANUAL);

    // �������ٶ�ָ��
    send_moto_cmd(MOTO1_Velocity, MOTO_REGIS_NUM1, -car_moto.car_left	* ROL_VAR, MOTO_Control_ID);
    send_moto_cmd(MOTO2_Velocity, MOTO_REGIS_NUM1, -car_moto.car_right * ROL_VAR, MOTO_Control_ID);
}

// �������ж��ٿ���ģʽ
void Auto_control(void)
{
    // ��λAPP����ͼ�Ĵ��˶�
    SETBIT(mqtt_pub_inform.runvehicleStatus,14);

    control_flag.speed_notific_flag = TRUE;
    car_moto.x = Remote[SBUS_X2]-SBUS_zhongzhi;
    car_moto.y = Remote[SBUS_Y2]-SBUS_zhongzhi;

    if(control_flag.Auto_gear_swith != G_0) 			            // �Ƕ���G_0��λ���������ҵ���ٶ�
    {
        if(control_flag.Auto_gear_swith == G_1)
        {
            #ifdef QRS_TEST_INVERT1_6	                        // ���Գ�
                    car_moto.car_left =  GEAR1_SPEED;
                    car_moto.car_right=  -GEAR1_SPEED;
            #else											    // ��ʽ��
                    car_moto.car_left =  GEAR1_SPEED;
                    car_moto.car_right= -GEAR1_SPEED;
            #endif
        }
        else if(control_flag.Auto_gear_swith == G_2)
        {
            #ifdef QRS_TEST_INVERT1_6			                // ���Գ�
                    car_moto.car_left =  GEAR2_SPEED;
                    car_moto.car_right=  -GEAR2_SPEED;
            #else											    // ��ʽ��
                    car_moto.car_left =  GEAR2_SPEED;
                    car_moto.car_right= -GEAR2_SPEED;
            #endif
        }
        else if(control_flag.Auto_gear_swith == G_3)
        {
            #ifdef QRS_TEST_INVERT1_6			                // ���Գ�
                car_moto.car_left =  GEAR3_SPEED;
                car_moto.car_right=  -GEAR3_SPEED;
            #else												// ��ʽ��
                car_moto.car_left =  GEAR3_SPEED;
                car_moto.car_right= -GEAR3_SPEED;
            #endif
        }

        if(car_moto.x > 0&&car_moto.y>-SBUS_siqu&&car_moto.y<SBUS_siqu)					// ��ת
        {
            #ifdef QRS_TEST_INVERT1_6		                    // ���Գ�
                car_moto.car_left = car_moto.car_left;
                car_moto.car_right=	car_moto.car_right * (1 - car_moto.x * 0.00138 * 0.5);
            #else												// ��ʽ��
                car_moto.car_left = car_moto.car_left * (1 - car_moto.x * 0.00138 * 0.5);//1/720=0.0013
                car_moto.car_right=	car_moto.car_right;
            #endif
        }
        else if(car_moto.x < 0&&car_moto.y>-SBUS_siqu&&car_moto.y<SBUS_siqu)			// ��ת
        {


            #ifdef QRS_TEST_INVERT1_6		                    // ���Գ�
                car_moto.car_left = car_moto.car_left * (1 + car_moto.x * 0.00138 * 0.5);         //  250/750=0.333����ģʽ�²���ת������250����
                car_moto.car_right=	car_moto.car_right;
            #else												// ��ʽ��
                car_moto.car_left = car_moto.car_left;
                car_moto.car_right= car_moto.car_right * (1 + car_moto.x * 0.00138 * 0.5);//1/720=0.0013
            #endif
        }
    }
    else if(control_flag.Auto_gear_swith == G_0) 	                // ����G_0��λ���������ҵ���ٶ�
    {
				if(control_flag.Auto_backflag2 == TRUE)
				{
					car_moto.cos = car_moto.x / sqrt(car_moto.x*car_moto.x+car_moto.y*car_moto.y);
					car_moto.sin = car_moto.y / sqrt(car_moto.x*car_moto.x+car_moto.y*car_moto.y);

					if(car_moto.cos<0)
					{
							car_moto.cos=-car_moto.cos;
					}
					if(car_moto.sin<0)
					{
							car_moto.sin=-car_moto.sin;
					}

//					if(car_moto.x<-SBUS_siqu|car_moto.x>SBUS_siqu && car_moto.y>-SBUS_siqu && car_moto.y<SBUS_siqu)				//ԭ��ת
//					{
//							#ifdef QRS_TEST_INVERT1_6	                            // ���Գ�
//									car_moto.car_left 	=	car_moto.x * TURN_VAR;
//									car_moto.car_right	= 	car_moto.x * TURN_VAR;
//							#else											        // ��ʽ��
//									car_moto.car_left 	=	-car_moto.x * TURN_VAR;
//									car_moto.car_right	= 	-car_moto.x * TURN_VAR;
//							#endif
//					}
//					else if(car_moto.x>-SBUS_siqu && car_moto.x<SBUS_siqu && car_moto.y<0)																		// ����
//					{
//							#ifdef QRS_TEST_INVERT1_6			                // ���Գ�
//									car_moto.car_left = car_moto.y;
//									car_moto.car_right= -car_moto.y;
//							#else												// ��ʽ��
//									car_moto.car_left = car_moto.y;
//									car_moto.car_right= -car_moto.y;
//							#endif
//					}

					// ǰ������
					if(car_moto.x==0&&car_moto.y<0)
					{
							#ifdef QRS_TEST_INVERT1_6			                // ���Գ�
									car_moto.car_left = car_moto.y;
									car_moto.car_right= -car_moto.y;
							#else												// ��ʽ��
									car_moto.car_left = car_moto.y;
									car_moto.car_right= -car_moto.y;
							#endif
					}

					// ԭ��ת

					else if(car_moto.x!=0&&car_moto.y>-SBUS_siqu&&car_moto.y<SBUS_siqu)
					{
							#ifdef QRS_TEST_INVERT1_6			                // ���Գ�
									car_moto.car_left = car_moto.x*TURN_VAR;
									car_moto.car_right= car_moto.x*TURN_VAR;
							#else												// ��ʽ��
									car_moto.car_left = -car_moto.x*TURN_VAR;
									car_moto.car_right= -car_moto.x*TURN_VAR;
							#endif
							control_flag.speed_notific_flag = FALSE;
					}

					// ��ת
					else if(car_moto.x>0&&car_moto.y<-SBUS_siqu)
					{
							#ifdef QRS_TEST_INVERT1_6			                // ���Գ�
									car_moto.car_left = car_moto.y;
									car_moto.car_right= -car_moto.y*car_moto.sin;
							#else												// ��ʽ��
									car_moto.car_left = car_moto.y*car_moto.sin;
									car_moto.car_right= -car_moto.y;
							#endif
					}

					// ��ת
					else if(car_moto.x<0&&car_moto.y<-SBUS_siqu)
					{
							#ifdef QRS_TEST_INVERT1_6			                // ���Գ�
									car_moto.car_left = car_moto.y*car_moto.sin;
									car_moto.car_right=	-car_moto.y;
							#else												// ��ʽ��
									car_moto.car_left = car_moto.y;
									car_moto.car_right=	-car_moto.y*car_moto.sin;
							#endif
					}
					else
					{
							car_moto.car_left =  0;
							car_moto.car_right=  0;

							// ���APP����ͼ�Ĵ��˶�
							CLRBIT(mqtt_pub_inform.runvehicleStatus,14);
							control_flag.speed_notific_flag = FALSE;
					}

				}
				else
				{
					car_moto.car_left =  0;
					car_moto.car_right=  0;

					// ���APP����ͼ�Ĵ��˶�
					CLRBIT(mqtt_pub_inform.runvehicleStatus,14);
					control_flag.speed_notific_flag = FALSE;
				}

		}

    // �л�ָʾ����ʾ״̬
    switch(control_flag.Auto_gear_swith)
    {
        case G_0:
            indicatorLight(INDECATOR_LIGHT_ITEM_AUTO);
            break;

        case G_1:
            indicatorLight(INDECATOR_LIGHT_ITEM_GEAR_1);
            break;

        case G_2:
            indicatorLight(INDECATOR_LIGHT_ITEM_GEAR_2);
            break;

        case G_3:
            indicatorLight(INDECATOR_LIGHT_ITEM_GEAR_3);
            break;
    }


    // �������ٶ�ָ��
    send_moto_cmd(MOTO1_Velocity, MOTO_REGIS_NUM1, -car_moto.car_left  * ROL_VAR, MOTO_Control_ID);
    send_moto_cmd(MOTO2_Velocity, MOTO_REGIS_NUM1, -car_moto.car_right * ROL_VAR, MOTO_Control_ID);
}



/*************************************************************�������ƶ���**************************************************************/

// ң������������
void sbus_unlock(void)
{
    // �����������λ
	send_moto_cmd(MOTO_RESET_STATE, MOTO_REGIS_NUM1, 0xFF00, MOTO_Control_ID);  //ң������������λ�������������״̬

//    // ������������������ƶ�
//	send_moto_cmd(MOTO_SysBitM, MOTO_REGIS_NUM1, 0x2000, MOTO_Control_ID);  //ң������������λ�������������״̬

    // �˶�����
    control_flag.Auto_gear_swith 				= G_0;			                // ����ģʽ���㵵����ֹ�������Զ���ģʽ������

	// �������������ͣ�Ĵ������������1��2��
    send_moto_cmd(MOTO1_Launch, MOTO_REGIS_NUM1, 0xFF00, MOTO_Control_ID);
	send_moto_cmd(MOTO2_Launch, MOTO_REGIS_NUM1, 0xFF00, MOTO_Control_ID);

	// ������˲���¼���м�¼�Ŀ�ʼ����
	time_t changeUTC = timer_info.timestamp;//��+28800��IOTĿǰ��UTC
	memcpy(sing_work_event.start_date, TimeStampToString(&changeUTC),14);// �Ӳ����ۼƵ�ʱ���ת����ʱ���ַ�����¼��ʼʱ��

	sing_work_event.start_power = battery_data.soc;						   		// ��¼��ʼ����
	sing_work_event.mileage = 0;												// ����ǰ����һ�����

    // �л�GPS�ϱ�Ƶ��
    gps_info.gpsUrcSet = 1;

    // ����������������
	speakItem(SPEAK_ITEM_SBUS_UNLOCK);

    // �л�ָʾ����ʾ״̬
	indicatorLight(INDECATOR_LIGHT_ITEM_MANUAL);
}

// ң������������
void sbus_lock(void)
{
    // ������������ٶȼĴ�������
    send_moto_cmd(MOTO1_Velocity, MOTO_REGIS_NUM1, 0, MOTO_Control_ID);
    send_moto_cmd(MOTO2_Velocity, MOTO_REGIS_NUM1, 0, MOTO_Control_ID);

    // �������������ͣ�Ĵ���ֹͣ�����1��2��
    send_moto_cmd(MOTO1_Launch, MOTO_REGIS_NUM1, 0x0000, MOTO_Control_ID);
    send_moto_cmd(MOTO2_Launch, MOTO_REGIS_NUM1, 0x0000, MOTO_Control_ID);

    // �Զ�����λ����
    control_flag.Auto_gear_swith 		= G_0;

    // д�������
    countdata_handle();

    // ����������������
    speakItem(SPEAK_ITEM_SBUS_LOCK);

    // �л�ָʾ����ʾ״̬
    indicatorLight(INDECATOR_LIGHT_ITEM_OPEN);

    // �رմ��
//    control_flag.Led48v_swith       = FALSE;
//    Led48v_control();

    // �رշ��
    control_flag.Draught_swith      = FALSE;
    Draught_control();

    // �ر�ˮ��
    control_flag.Pump_swith         = G_0;
    Pump_control();

    // �л�GPS�ϱ�Ƶ��
    gps_info.gpsUrcSet = 10;
}

// ң������������
void sbus_unconnect(void)
{
    // ������������ٶȼĴ�������
    send_moto_cmd(MOTO1_Velocity, MOTO_REGIS_NUM1, 0, MOTO_Control_ID);
    send_moto_cmd(MOTO2_Velocity, MOTO_REGIS_NUM1, 0, MOTO_Control_ID);

    // �Զ�����λ����
    control_flag.Auto_gear_swith 	= G_0;

    // ң�����ָ�����״̬
    control_flag.Sbus_lock_flag     = FALSE;

    // �رմ��
    control_flag.Led48v_swith       = FALSE;
    Led48v_control();

    // �رշ��
    control_flag.Draught_swith      = FALSE;
    Draught_control();

    // �ر�ˮ��
    control_flag.Pump_swith         = G_0;
    Pump_control();

    // �л�GPS�ϱ�Ƶ��
    gps_info.gpsUrcSet = 10;
}

// ���ؿ�������
void soft_sw_clickON(void)
{

    // �Ӵ�������
    driverBoard_contactorState = 1;
    WriteSDO(&CANopen_Master_M200_Data, CANDevice_NodeID_driver, DRIVER_INDEX_CONTACTOE, DRIVER_SUB_INDEX_DEFAULT, 1, uint8, &driverBoard_contactorState, 0);

    // ����ɢ�ȷ���
    control_flag.Boxfan_swith       	 = ENABLE;		// ����������ȿ���ʹ��
    control_flag.Carfan_swith      		 = ENABLE;		// ����������ȿ���ʹ��

    // ��ȡ������Ϣ
    countdata_read();

    // �򿪽��ջ�����
    HAL_GPIO_WritePin(SBUS_PWR_GPIO_Port, SBUS_PWR_Pin, GPIO_PIN_RESET);

    // �л�ָʾ����ʾ״̬
    indicatorLight(INDECATOR_LIGHT_ITEM_OPEN);

    // ������������
    speakItem(SPEAK_ITEM_OPEN);
    batteryTypeRead();
    speakItem(SPEAK_ITEM_SOC(battery_data.soc));
    if(battery_data.soc < BatteryLowSOC_Level1 && battery_data.soc > BatteryLowSOC_Level2)
    {
        speakItem(SPEAK_ITEM_WORKTIME_REMIND);
    }
    if(battery_data.soc <= BatteryLowSOC_Level2)
    {
        speakItem(SPEAK_ITEM_WORK_BAN);
        speakItem(SPEAK_ITEM_CHARGE_REMIND);
    }

    // ��������
    ble_inform.ble_switch = TRUE;

		//QC150��ؽӴ������ϵ���ת������״̬����ƿ����ź�֮ǰ������ʱ�������������ر���
		vTaskDelay(1000);
}

// ���عػ�����
void soft_sw_clickOFF(void)
{
    // д����ʱ��
    counttime_handle();

    // �Զ�����λ����
    control_flag.Auto_gear_swith 		= G_0;

    // ң�����ָ�����״̬
    control_flag.Sbus_lock_flag = FALSE;

    // �ر�ɢ�ȷ���
    control_flag.Boxfan_swith           = DISABLE;
    control_flag.Carfan_swith      	    = DISABLE;

    // �رմ��
    control_flag.Led48v_swith       = FALSE;
    Led48v_control();

    // �رշ��
    control_flag.Draught_swith      = FALSE;
    Draught_control();

    // �ر�ˮ��
    control_flag.Pump_swith         = G_0;
    Pump_control();

    // �رս��ջ�����
    HAL_GPIO_WritePin(SBUS_PWR_GPIO_Port, SBUS_PWR_Pin, GPIO_PIN_SET);

    // �л�ָʾ����ʾ״̬
    battery_data.socState = SOC_ENOUGH;
    indicatorLight(INDECATOR_LIGHT_ITEM_CLOSE);

    // �����ػ���ʣ�����
    speakItem(SPEAK_ITEM_CLOSE);
    speakItem(SPEAK_ITEM_SOC(battery_data.soc));
    if(battery_data.soc < BatteryHighSOC)
    {
        speakItem(SPEAK_ITEM_CHARGE_REMIND);		//�����������뼰ʱ��硱
    }

    // �ر�����
    ble_inform.ble_switch = FALSE;

    osDelay(1000);

    // �Ӵ����Ͽ�
    driverBoard_contactorState = 0;
    WriteSDO(&CANopen_Master_M200_Data, CANDevice_NodeID_driver, DRIVER_INDEX_CONTACTOE, DRIVER_SUB_INDEX_DEFAULT, 1, uint8, &driverBoard_contactorState, 0);

    // �л�GPS�ϱ�Ƶ��
    gps_info.gpsUrcSet = 10;
}

// ��ͣ�򿪶���
void scram_sw_clickON(void)
{

    // �Ͽ��Ӵ���
    driverBoard_contactorState = 0;
    WriteSDO(&CANopen_Master_M200_Data, CANDevice_NodeID_driver, DRIVER_INDEX_CONTACTOE, DRIVER_SUB_INDEX_DEFAULT, 1, uint8, &driverBoard_contactorState, 0);

    // д�������
	counttime_handle();

    // �л�ָʾ����ʾ״̬
	indicatorLight(INDECATOR_LIGHT_ITEM_FAULT);

    // �رմ��
    control_flag.Led48v_swith       = FALSE;
    Led48v_control();

    // �رշ��
    control_flag.Draught_swith      = FALSE;
    Draught_control();

    // �ر�ˮ��
    control_flag.Pump_swith         = G_0;
    Pump_control();

    // �Զ�����λ����
    control_flag.Auto_gear_swith 	= G_0;

    // �л�GPS�ϱ�Ƶ��
    gps_info.gpsUrcSet = 10;
}

// �����ϵ��ʼ������
void car_state_Init(void)
{
    // �رմ��
    control_flag.Led48v_swith       = FALSE;
    Led48v_control();

    // �رշ��
    control_flag.Draught_swith      = FALSE;
    Draught_control();

    // �ر�ˮ��
    control_flag.Pump_swith         = G_0;
    Pump_control();

    //��ͣ������������Ĵ��������1��2�� ֹͣ
    send_moto_cmd(MOTO1_Launch, MOTO_REGIS_NUM1, 0x0000, MOTO_Control_ID);
    send_moto_cmd(MOTO2_Launch, MOTO_REGIS_NUM1, 0x0000, MOTO_Control_ID);
}

// ��������Ӷ���
void power_connect(void)
{
    // �л�ָʾ����ʾģʽ
	battery_data.socState = SOC_ENOUGH;
	indicatorLight(INDECATOR_LIGHT_ITEM_RECHARGE);

    // ������������Դ�����ӡ�
	speakItem(SPEAK_ITEM_POWER_CONNECT);

    // ��ɢ�ȷ���
    control_flag.Boxfan_swith           = ENABLE;
    control_flag.Carfan_swith      	    = ENABLE;
}

// �����϶���
void charge_reset(void)
{
    // �������ָ�
    battery_data.spentChargeTime  = 0;		//�����������绨��ʱ��
	battery_data.chargeTimeRemain = 0;		//������������ʣ��ʱ��

	// �л�ָʾ����ʾģʽ
	indicatorLight(INDECATOR_LIGHT_ITEM_CLOSE);

    // �ر�ɢ�ȷ���
    control_flag.Carfan_swith      		 	= DISABLE;
    control_flag.Boxfan_swith      		 	= DISABLE;
}

/*************************************************************��������״̬��***********************************************************/

// ��������״̬��
void car_state_trans(void)
{
	switch(control_flag.Car_State)
	{
        //------------------------------------�ػ�״̬--------------------------------------------------------�ػ�״̬------------------------------------------------------�ػ�״̬-----------------------------------//
        case CLOSE:
            {
                // ������ʼ��
                if(control_flag.Init_flag == TRUE)
                {
                    control_flag.Init_flag             = FALSE;
                    car_state_Init();
                }

                // ���������
                if(battery_data.charge_flag == TRUE)                    // CAN���ռ�������ش��ڳ��״̬������־λλ��
                {
                    mqtt_pub_inform.start_charge_flag   = ENABLE;		// ��⵽����Դ�����ϱ�һ�������ʱ��
                    power_connect();
                    control_flag.Car_State 				= RECHARGE;	    // ��ת�����״̬
                }

                // ���ش�
                if(softSwitch_switchFlag == TRUE)
                {
                    soft_sw_clickON();
										control_flag.lock_check_flag = TRUE;//����ʱ���һ��ˮ���Ƿ�ر�
                    control_flag.Car_State 				 = OPEN;		// ��ת����������״̬
                }

                //�ϱ�����λ
				if(control_flag.error_res_flag == TRUE) {
					HAL_NVIC_SystemReset();
				}

            }
            break;


        //------------------------------------����״̬--------------------------------------------------------����״̬------------------------------------------------------����״̬-----------------------------------//
        case OPEN:
            {
                // ң��������
                if(control_flag.Sbus_lock_flag == TRUE && control_flag.Sbus_connect_flag == TRUE)
                {
                    sbus_unlock();
                    mqtt_pub_inform.runvehicleStatus = 0xffff&0x0213;   // ����ģʽ�������ó�ȫ�أ��Զ������أ��ֶ�
                    sing_work_event.send_count = FALSE;
                    control_flag.Car_State = RUN;
                }

                // ���عر�
                if(softSwitch_switchFlag == FALSE)
                {
                    soft_sw_clickOFF();
                    control_flag.Car_State 				= CLOSE;	    // ��ת���ػ����͹��ģ�״̬
                    mqtt_pub_inform.start_summary_flag  = ENABLE;		// ���عػ�ʱ�ϱ�һ�λ�������
                }

//							// �����豸����
								Led48v_control();
            }
            break;


        //------------------------------------����״̬--------------------------------------------------------����״̬------------------------------------------------------����״̬-----------------------------------//
        case RUN:
            {
                /**************************��ת����*************************/
                if(control_flag.save_turn_flag == RESET)
                {
                    if(Remote[SBUS_X2] < SBUS_mid_L_limit | Remote[SBUS_X2] > SBUS_mid_R_limit)
                    {
                        control_flag.count_turn++;				// ң�������ܿ����ж������ۼ�10�δ���һ��
                        if(control_flag.count_turn > 20)	    // ң�д��򳬹�����洢һ������
                        {
                            control_flag.save_turn_flag = SET;  // ң����ң��ת���ж����洢����ʹ��
                            control_flag.count_turn = 0;
                        }
                    }
                    else
                    {
                        control_flag.count_turn = 0;
                    }
                }
                control_flag.Draught_open_flag = TRUE;

                /**************************ң��������*************************/
                if(control_flag.Sbus_lock_flag == TRUE)
                {

                    // �����豸����
                    Led48v_control();

                    // ����ϵͳ����
                    if(control_flag.Auto_spray_swith == TRUE && control_flag.speed_notific_flag == FALSE)
                    {
                        control_flag.Pump_swith			= G_0;
                        control_flag.Draught_swith 	    = FALSE;
                    }
                    Pump_control();
                    Draught_control();

                    // �����˶�����
                    if(control_flag.Auto_swith== DISABLE)
                    {
                        hunkong(Remote[SBUS_X2], Remote[SBUS_Y2]);	// ������ʻ����
                        control_flag.Auto_gear_swith = G_0;			// ����Ѳ���ٶ�����ΪG0
                    }
                    else if(control_flag.Auto_swith == ENABLE)
                    {
                        Auto_control();								// ����Ѳ����ʻ����
                    }
                }

                /**************************ң��������*************************/
                else
                {
                    sbus_lock();												// ң����������ң����ʧ�ر���ֵ�ᵼ��ң��������

                    sing_work_event.send_count 					= TRUE;
                    control_flag.event_mpub_single_flag 	    = TRUE;         // �ϱ��¼�

										control_flag.lock_check_flag = TRUE;//�´ν���ʱ���һ��ˮ�ÿ����Ƿ�ر�

                    control_flag.Car_State 					    = OPEN;			// ��ת���ػ����͹��ģ�״̬
                }

                /**************************ң��������*************************/
                if(control_flag.Sbus_connect_flag == FALSE)
                {
                    // ִ��ң������������
                    sbus_lock();

                    //ң���������Ϻ���´���ǰ���һ��ˮ���Ƿ�ر�
					control_flag.lock_check_flag = TRUE;
					control_flag.Sbus_lock_flag  = FAULT;
                    // ״̬�л�
                    control_flag.Car_State 			= OPEN;
                }

                /**************************��ͣ����*************************/
                if(driverBoard_scramStop == TRUE)
                {
                    // �ϱ�һ�λ�������
                    mqtt_pub_inform.start_summary_flag          = ENABLE;

                    // ִ�м�ͣ���²���
                    scram_sw_clickON();

					// ״̬�л�
                    control_flag.Car_State 						= FAULT;

                    // �ϱ��¼�
                    sing_work_event.send_count 					= TRUE;
                    control_flag.event_mpub_single_flag 	    = TRUE;

                }

                /**************************���عر�*************************/
                if(softSwitch_switchFlag == FALSE)
                {
                    // �ϱ�һ�λ�������
                    mqtt_pub_inform.start_summary_flag          = ENABLE;

                    // ��¼���������
                    countdata_handle();

                    // ���عػ�����
                    soft_sw_clickOFF();

                    // �ϱ��¼�
                    sing_work_event.send_count 					= TRUE;			// ң���������ϱ��¼�2��
                    control_flag.event_mpub_single_flag 	    = TRUE;			// ����״̬��������״̬���û��ػ������ϱ�һ�����м�¼�¼�

                    // ״̬�л�
                    control_flag.Car_State 						= CLOSE;		// ��ת���ػ����͹��ģ�״̬
                }
            }
            break;


        //------------------------------------���״̬--------------------------------------------------------���״̬------------------------------------------------------���״̬-----------------------------------//
        case RECHARGE:
            {
                /**************************���ѳ���*************************/
                if(battery_data.charge_flag == FALSE)
                {
                    charge_reset();
                    battery_data.charge_overflag       	    = ENABLE;		// �����Ϸ��ͳ��ʣ��ʱ��0
                    control_flag.Car_State 					= CLOSE;		// ��ת���ػ�״̬
                }

                /**************************ң��������*************************/
                //if(control_flag.Sbus_connect_flag == FALSE)
                //{
                //    // ִ��ң������������
                //    sbus_unconnect();
                //}
            }
            break;


        //------------------------------------����״̬--------------------------------------------------------����״̬------------------------------------------------------����״̬-----------------------------------//
        case FAULT:
            {
                /**************************���عػ�*************************/
                if(softSwitch_switchFlag == FALSE)
                {
                    soft_sw_clickOFF();
                    control_flag.Car_State 				 	= CLOSE;
                }
            }
            break;


        //------------------------------------Ĭ��״̬--------------------------------------------------------Ĭ��״̬------------------------------------------------------Ĭ��״̬-----------------------------------//
        default:
            break;
		}
}


































/************************************δʹ�õĺ���****************************************/



///*************��⵽ң�������м�λ�Ƿ�ԭ*************/
//uint8_t sbus_check(void)
//{
//	if(control_flag.Sbus_connect_flag == TRUE    &&     //ң����������
//		control_flag.Pump_swith         == G_0     &&     //ˮ�õ�λΪ0
//		 Remote[SBUS_C]  >=  SBUS_up_limit         &&     //����ر�
//	   control_flag.Auto_spray_swith  == FALSE   &&     //�Զ�������ͣ�ر�
//	   control_flag.Auto_swith        == DISABLE )      //�Զ����ر�
//		return TRUE;
//	else
//		return FALSE;
//}

//uint8_t aTxBuffer[WR_DATA_LEN] = "0123456789ABCDEF";
//uint8_t aRxBuffer[WR_DATA_LEN] = {0};

