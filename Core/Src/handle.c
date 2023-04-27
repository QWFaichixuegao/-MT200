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

//#define QRS_TEST_INVERT1_6     //测试车1标识符

// CANopen对象字典定义
extern CO_Data CANopen_Master_M200_Data;

// 遥控器数据
USARTX_SBUS 		usart6_sbus;

// 控制数据定义
CONTROL_FLAG 		control_flag;

// CAN通讯数据
CAN_READ_DATA		can_read_data;
CAN_HANDLE			can_handle;
CAR_MOTO 			car_moto;

// 汇总数据
SUMMARY_DATA 		summary_data;
COUNT_TIME 			count_time;
EEPROM_PACK 		eeprom_packet;

// 遥控器数据
volatile    uint16_t Remote[16];
static      uint8_t subs2dbus 	 = 0;


uint16_t BLElen;//要发送蓝牙数据的长度(单次小于244)    		// DeviceName;ProductKey


// 获取复位源
void getSysReseySource(void)
{
    // 供电电压低于阀值产生的复位   POR/PDR or BOR reset  //Power-on/power-down reset (POR/PDR reset) or brownout (BOR)
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_BORRST)== SET)
    {
        SETBIT(control_flag.resetSource, 0);
    }

    // RESET管脚产生的复位
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_PINRST)== SET)
    {
        SETBIT(control_flag.resetSource, 1);
    }

    // 上电复位（冷启动）
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_PORRST)== SET)
    {
        SETBIT(control_flag.resetSource, 2);
    }

    // 软件重启产生的复位
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST)== SET)
    {
        SETBIT(control_flag.resetSource, 3);
    }

    // 独立看门狗产生的复位
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST)== SET)
    {
        SETBIT(control_flag.resetSource, 4);
    }

    // 窗口看门狗产生的复位
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_WWDGRST)!= RESET)
    {
        SETBIT(control_flag.resetSource, 5);
    }

    // 低功耗产生的复位
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_LPWRRST )!= RESET)
    {
        SETBIT(control_flag.resetSource, 6);
    }
}

// 车辆控制参数初始化
void control_init(void)
{
    // 车辆状态
    control_flag.Car_State    			        = CLOSE;
    control_flag.Init_flag                      = TRUE;

    // 车辆绑定
    control_flag.Bound_flag					    = FALSE;
    control_flag.Match_flag					    = FALSE;

    // 遥控器
    control_flag.Sbus_lock_flag			        = FALSE;		// 遥控器解锁标志位
    control_flag.Sbus_connect_flag              = FALSE;		// 遥控器连接标志位


    // 车身设备初始化
    control_flag.Led48v_swith				    = FALSE;		// 车辆前大灯控制开关
    control_flag.Boxfan_swith				    = FALSE;		// 车辆箱体风扇控制开关
    control_flag.Carfan_swith				    = FALSE;		// 车辆车体风扇控制开关
    softSwitch_switchFlag 			     		= FALSE;

    // 喷洒作业
    control_flag.Draught_swith			        = FALSE;		// 风机控制开关
    control_flag.Pump_swith			  	        = FALSE;
    control_flag.DraughtPumpEnable				= TRUE;
    control_flag.speed_notific_flag	            = FALSE;
    control_flag.Auto_spray_swith		        = FALSE;
    control_flag.Auto_swith					    = FALSE;
    control_flag.Auto_gear_swith		        = G_0;

    driverBoard_draughtFanspeed					= 0;
    driverBoard_pumpSpeed						= 0;


    // 运动控制
    control_flag.up_gear_flag				    = FALSE;
    control_flag.down_gear_flag			    = FALSE;
		control_flag.Auto_backflag1					= FALSE;
		control_flag.Auto_backflag2					= FALSE;
		control_flag.lock_check_again 			= FALSE;

    // 4G模块
    //control_flag.Usart3_handle_flag             = TRUE;       // 串口3处理4G模块下发给单片机的数据
    control_flag.event_mpub_mutex               = FALSE;        // 服务器上报互斥锁关闭
    control_flag.event_mpub_single_flag         = FALSE;        // 服务器上报单次工作时间标志位在关闭后消息上报主循环中上传一次

    gps_info.gps_signal_flag				    = FALSE;

    air_4g_flag.hal_delay 					    = (bool)FALSE;
		air_4g_flag.vTa_delay					    = (bool)TRUE;
		air_4g_flag.SIM_flag 					    = (bool)FALSE;
		air_4g_flag.MQTT_flag	 				    = (bool)FALSE;
		air_4g_flag.send_version_flag 	            = (bool)TRUE;
		air_4g_flag.get_realtime_flag			= FALSE;												//开机开始校准实时时间
		air_4g_connect.sim_ccount 	 			    = 0;
		air_4g_connect.mqtt_ccount	 			    = 0;



    // 电源
    battery_data.charge_flag				    = FALSE;        // 充电标识
    battery_data.charge_overflag                = DISABLE;
    driverBoard_scramStop						= FALSE;        // 急停标识
    driverBoard_contactorState					= 0;		    // 接触器默认断开

    gps_info.gpsUrc                             = 10;
    gps_info.gpsUrcSet                          = 10;

    ble_inform.ble_switch_state                 = FALSE;
    ble_inform.ble_switch                       = FALSE;


    can_handle.RxFlag							= FALSE;	    // CAN接收标志位
    control_flag.Iwdg_count						= 0;
    control_flag.error_res_count				= 0;
    control_flag.error_res_flag                 = FALSE;




    //control_flag.DraughtPumpEnableRecover     = FALSE;	    // 开机状态下没有过低电量复位
    //control_flag.Highest_autho_flag           = TRUE; 		// 出厂状态下，先打开最高权限
    //control_flag.Sbus_midlock_flag	        = FALSE;		// 遥控器解锁回中标志位

    //driverBoard_carLight						= 0;
    //driverBoard_carFanSpeed					= 0;
    //driverBoard_boxFanSpeed					= 0;
}

/*************************************************************车辆运行记录汇总***********************************************************/

// 车辆运行数据初始化
void device_init(void)
 {

	strcpy(device_inform.version,"1.1.7");//写入的版本号
	AT24CXX_WriteData(0x0180, (uint8_t*)device_inform.version, 5);	//将版本号写入EEPROM



	/*从EEPROM中读取设备信息*/
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

	sprintf(mqtt_pub_inform.theme_str,"/sys/%s/%s/thing/event/property/post",		        //设备属性上报，物模型
	        device_inform.ProductKey, device_inform.DeviceName);

	sprintf(mqtt_pub_inform.work_event_Main,"/sys/%s/%s/thing/event/runRecordMain/post",	//设备事件上报
	        device_inform.ProductKey,device_inform.DeviceName);

	sprintf(mqtt_pub_inform.work_event_Track,"/sys/%s/%s/thing/event/runRecordTrack/post",	//设备事件上报
	        device_inform.ProductKey,device_inform.DeviceName);

	sprintf(mqtt_ota_inform.version_theme_str,"/ota/device/inform/%s/%s",	                //设备上报固件升级信息
	        device_inform.ProductKey, device_inform.DeviceName);

	sprintf(mqtt_ota_inform.request_theme_str,"/sys/%s/%s/thing/ota/firmware/get",	        //设备主动拉取固件升级信息
	        device_inform.ProductKey, device_inform.DeviceName);

	sprintf(ota_inform.msubOTA_theme_str,"/sys/%s/%s/thing/ota/firmware/get_reply",
					device_inform.ProductKey, device_inform.DeviceName);

	mqtt_pub_inform.liquidLevelSensor	= 10;
	mqtt_pub_inform.vehicleSpeed		= 123;
	mqtt_pub_inform.fanMachinery		= 10;
	mqtt_pub_inform.motorWaterPump		= 6;
	mqtt_pub_inform.rundata_turn_count	= 0;
	mqtt_pub_inform.start_summary_flag  = DISABLE;		// 开机默认不会上报汇总信息
	mqtt_pub_inform.start_charge_flag   = DISABLE;

	sing_work_event.power				= 0;//--
	sing_work_event.drug				= 0;//--
	sing_work_event.save_track_count	= 0;
	sing_work_event.mileage				= 0;

	battery_init();


	/*-BLE-*/

	sprintf(ble_inform.ble_name, "AT+BLECOMM=NAME,MQ_%s\r", device_inform.DeviceName);
	char data4[2];//每个转成16进制的字符 高四位和低四位分别占两个字节
	sprintf(ble_inform.dev_bound_data, "%s$%s", device_inform.DeviceName,device_inform.ProductKey);
	BLElen=strlen(ble_inform.dev_bound_data);
	for(uint8_t i = 0; i<BLElen; i++)
		{
			sprintf(data4, "%2X", ble_inform.dev_bound_data[i]);//将每个字符转成16进制高四位和低四位分别占两个字节(%2X)
			//strncat(data6, data4, 2);
			strcat (ble_inform.dev_data_To16, data4);
		}
	sprintf(ble_inform.dev_send_cmd, "AT+BLECOMM=SENDDATA,fee2,%d,%s\r",BLElen , ble_inform.dev_data_To16);
}

// 车辆运行时间记录
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

// 车辆运行里程记录
void countdata_handle(void)
{
	summary_data.totalMileage += sing_work_event.mileage;
	for(uint8_t i=0;i<4;i++)
	{
		eeprom_packet.Mileage[i]				  = 0xff&(summary_data.totalMileage>>(i*8));
	}
	AT24CXX_WriteData(0x0204, eeprom_packet.Mileage,	4);
}


// 车辆开机读取运行数据
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

/*************************************************************遥控器数据解析**************************************************************/

//遥控器数据解析
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
    // T10 10通道
     Remote[10] = ((int16_t)usart6_sbus.rx_buf[13+subs2dbus] >> 6 | ((int16_t)usart6_sbus.rx_buf[14+subs2dbus] << 2 )  | (int16_t)usart6_sbus.rx_buf[15+subs2dbus] <<  10 ) & 0x07FF;
     Remote[11] = ((int16_t)usart6_sbus.rx_buf[15+subs2dbus] >> 1 | ((int16_t)usart6_sbus.rx_buf[16+subs2dbus] << 7 )) & 0x07FF;
    // Remote[12] = ((int16_t)usart6_sbus.rx_buf[16+subs2dbus] >> 4 | ((int16_t)usart6_sbus.rx_buf[17+subs2dbus] << 4 )) & 0x07FF;
    // Remote[13] = ((int16_t)usart6_sbus.rx_buf[17+subs2dbus] >> 7 | ((int16_t)usart6_sbus.rx_buf[18+subs2dbus] << 1 )  | (int16_t)usart6_sbus.rx_buf[19+subs2dbus] <<  9 ) & 0x07FF;
    // Remote[14] = ((int16_t)usart6_sbus.rx_buf[19+subs2dbus] >> 2 | ((int16_t)usart6_sbus.rx_buf[20+subs2dbus] << 6 )) & 0x07FF;
    // Remote[15] = ((int16_t)usart6_sbus.rx_buf[20+subs2dbus] >> 5 | ((int16_t)usart6_sbus.rx_buf[21+subs2dbus] << 3 )) & 0x07FF;

    //判断接收机是否连接正常
    if(usart6_sbus.rx_buf[23] != 0x00)
    {
        control_flag.Sbus_connect_flag = FALSE;
    }
    else
    {
        control_flag.Sbus_connect_flag = TRUE;
    }

    // 如果自动挡模式开启
    if(control_flag.Auto_swith == ENABLE)		//A键开启自动挡
    {
			if(Remote[SBUS_Y2]>SBUS_up_onofflimit && control_flag.up_gear_flag == FALSE)
			{
				control_flag.up_gear_flag = TRUE;			// 升档
				if(control_flag.Auto_gear_swith < G_3)
				{
						control_flag.Auto_gear_swith++;
				}
				control_flag.Auto_backflag2	= FALSE;      //自动挡首次归零档比例后退锁
			}
			else if(Remote[SBUS_Y2]<SBUS_lw_onofflimit)
			{
				control_flag.down_gear_flag = TRUE;		    // 降档
			}


			if(Remote[SBUS_Y2] < SBUS_mid_R_limit && Remote[SBUS_Y2] > SBUS_mid_L_limit)	// 遥感中位上下50内算回中成功
			{
				control_flag.up_gear_flag 		= FALSE;
				if(control_flag.Auto_backflag1 == TRUE)
				{
					control_flag.Auto_backflag2	= TRUE;
					control_flag.Auto_backflag1 = FALSE;
				}
			}

			if(control_flag.down_gear_flag == TRUE)	// 降档，档位直接将为G_0档
			{
				control_flag.down_gear_flag 		= FALSE;
				control_flag.Auto_gear_swith		= G_0;
				control_flag.Auto_backflag1			= TRUE;
			}
    }

    /******如果已经解锁******/
    if(control_flag.Sbus_lock_flag == TRUE)
    {
        // Sbus风机控制标志位
        if(Remote[SBUS_C] >= SBUS_up_limit | Remote[SBUS_F] <= SBUS_lw_limit)
        {
            // 风机速度关闭
            control_flag.Draught_swith = FALSE;
        }
        else if(Remote[SBUS_C] <= SBUS_up_limit && Remote[SBUS_F] >= SBUS_lw_limit)
        {
            // 风机打开
            control_flag.Draught_swith = TRUE;
        }

        // Sbus水泵控制标志位
        if(Remote[SBUS_F] <= SBUS_lw_limit)
        {
            // 水泵关闭
            control_flag.Pump_swith = G_0;
        }
        else if(Remote[SBUS_F] == SBUS_zhongzhi)
        {
            // 水泵1档
            control_flag.Pump_swith = G_1;
        }
        else if(Remote[SBUS_F] >= SBUS_up_limit)
        {
            // 水泵2档
            control_flag.Pump_swith = G_2;
        }
    }

		// Sbus大灯控制标志位
		if(Remote[SBUS_Y1]>SBUS_up_onofflimit && Remote[SBUS_X1]<SBUS_mid_R_limit && Remote[SBUS_X1]>SBUS_mid_L_limit)
		{
				// 打开大灯
				control_flag.Led48v_swith=TRUE;
		}
		else if(Remote[SBUS_Y1]<SBUS_lw_onofflimit && Remote[SBUS_X1]<SBUS_mid_R_limit && Remote[SBUS_X1]>SBUS_mid_L_limit)
		{
				// 关闭大灯
				control_flag.Led48v_swith=FALSE;
		}

    // 蓝牙绑定标志位
    if(Remote[SBUS_E] >= SBUS_up_limit )
    {
        // 关闭配对响应
        control_flag.Match_flag     = FALSE;

        // 遥控器上锁
        control_flag.Sbus_lock_flag = FALSE;
        control_flag.lock_check_again = TRUE;
    }
    else if(Remote[SBUS_E] == SBUS_zhongzhi)
    {
        // 打开配对响应
        control_flag.Match_flag     = TRUE;

        // 遥控器上锁
        control_flag.Sbus_lock_flag = FALSE;

    }
    else if(Remote[SBUS_E] <= SBUS_lw_limit)
    {
        // 关闭配对响应
        control_flag.Match_flag = FALSE;

				if(control_flag.lock_check_flag ==TRUE)
				{
					if(Remote[SBUS_F] <= SBUS_lw_limit)//水泵开关关闭后才能解锁
					{
						if(control_flag.lock_check_again ==TRUE)
						{
							control_flag.lock_check_flag = FALSE;
							// 遥控器解锁
							control_flag.Sbus_lock_flag = TRUE;
                            control_flag.lock_check_again = FALSE;
						}
					}
					else
					{
						control_flag.lock_check_again = FALSE;//解锁键需要重新波动解锁
					}

				}

    }

    // Sbus自动挡控制标志位
    if(Remote[SBUS_A] >= SBUS_up_limit)
    {
        control_flag.Auto_swith = ENABLE;       //定速模式
        CLRBIT(mqtt_pub_inform.runvehicleStatus,4);
        SETBIT(mqtt_pub_inform.runvehicleStatus,5);
    }
    else if(Remote[SBUS_A] <= SBUS_lw_limit)
    {
        control_flag.Auto_swith = DISABLE;      //手动模式
        CLRBIT(mqtt_pub_inform.runvehicleStatus,5);
        SETBIT(mqtt_pub_inform.runvehicleStatus,4);
    }


    // Sbus喷洒自动启停控制标志位
    if(Remote[SBUS_B] >= SBUS_up_limit)
    {
        // 自动启停功能打开
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

/*************************************************************车辆执行器控制**************************************************************/

// 48V大灯控制
void Led48v_control(void)
{
    // 灯亮度不可控，先写死最亮与最暗
    if(control_flag.Led48v_swith==TRUE)
    {
        driverBoard_carLight = 100;

        // 置位APP孪生大灯
        SETBIT(mqtt_pub_inform.runvehicleStatus,15);
    }
    else
    {
        driverBoard_carLight = 0;

        // 清除APP孪生大灯
        CLRBIT(mqtt_pub_inform.runvehicleStatus,15);
    }
    WriteSDO(&CANopen_Master_M200_Data, CANDevice_NodeID_driver, DRIVER_INDEX_CAR_LIGHT, DRIVER_SUB_INDEX_DEFAULT, 1, uint8, &driverBoard_carLight, 0);
}

// 箱体风扇控制
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

// 车体风扇控制
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

// 风机控制
void Draught_control(void)
{
    uint16_t draughtSpeedVar;
    //if(control_flag.Draught_swith == FALSE | control_flag.Draught_open_flag == FALSE | control_flag.DraughtPumpEnable == FALSE)
    if(control_flag.Draught_swith == FALSE | control_flag.DraughtPumpEnable == FALSE)
    {

        mqtt_pub_inform.fanMachinery    = 0;
        driverBoard_draughtFanspeed     = 0;

        // 清除APP孪生风机
        CLRBIT(mqtt_pub_inform.runvehicleStatus,13);
    }
    else
    {
        draughtSpeedVar = 29 + (SBUS_zhongzhi-Remote[SBUS_C]) * 0.04;			//		0~57
        mqtt_pub_inform.fanMachinery    =   (uint8_t)(draughtSpeedVar / 57.0 * 100.0);
        driverBoard_draughtFanspeed     =   draughtSpeedVar;

        // 置位APP孪生风机
        SETBIT(mqtt_pub_inform.runvehicleStatus,13);
    }
    WriteSDO(&CANopen_Master_M200_Data, CANDevice_NodeID_driver, DRIVER_INDEX_DRAUGHT_FAN_SPEED, DRIVER_SUB_INDEX_DEFAULT, 1, uint8, &driverBoard_draughtFanspeed, 0);
}

// 水泵控制
void Pump_control(void)
{
    uint16_t G_1_var,G_2_var;
    if(control_flag.DraughtPumpEnable == FALSE | Remote[SBUS_D] <= 290)
    {
        driverBoard_pumpSpeed =0;

        // 清除APP孪生水泵
        CLRBIT(mqtt_pub_inform.runvehicleStatus,12);
    }
    else
    {
        switch(control_flag.Pump_swith)
        {
            case G_1:
                G_1_var = (uint16_t)((Remote[SBUS_D]-SBUS_MIN) * 39 / (SBUS_MAX - SBUS_MIN) + 20);	// 20-59
                driverBoard_pumpSpeed = G_1_var;

                // 置位APP孪生水泵
                SETBIT(mqtt_pub_inform.runvehicleStatus,12);
                break;

            case G_2:
                G_2_var = (uint16_t)((Remote[SBUS_D]-SBUS_MIN) * 59 / (SBUS_MAX - SBUS_MIN) + 40);	// 40-99
                driverBoard_pumpSpeed = G_2_var;

                // 置位APP孪生水泵
                SETBIT(mqtt_pub_inform.runvehicleStatus,12);
                break;

            case G_0:
                driverBoard_pumpSpeed = 0;

                // 清除APP孪生水泵
                CLRBIT(mqtt_pub_inform.runvehicleStatus,12);
                break;
        }
    }
    WriteSDO(&CANopen_Master_M200_Data, CANDevice_NodeID_driver, DRIVER_INDEX_PUMP_SPEED, DRIVER_SUB_INDEX_DEFAULT, 1, uint8, &driverBoard_pumpSpeed, 0);
}

/*************************************************************车辆运动控制**************************************************************/

// 车辆运行手动控制模式
void hunkong(uint16_t X2, uint16_t Y2)
{
    // 置位APP孪生图履带运动
    SETBIT(mqtt_pub_inform.runvehicleStatus,14);

    control_flag.speed_notific_flag = TRUE;
    //新
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

    // 前进后退
    if(car_moto.x==0&&car_moto.y!=0)
    {
        #ifdef QRS_TEST_INVERT1_6			                // 测试车
            car_moto.car_left = car_moto.y;
            car_moto.car_right= -car_moto.y;
        #else												// 正式车
            car_moto.car_left = car_moto.y;
            car_moto.car_right= -car_moto.y;
        #endif
    }

    // 原地转

    else if(car_moto.x!=0&&car_moto.y>-SBUS_siqu&&car_moto.y<SBUS_siqu)
    {
        #ifdef QRS_TEST_INVERT1_6			                // 测试车
            car_moto.car_left = car_moto.x*TURN_VAR;
            car_moto.car_right= car_moto.x*TURN_VAR;
        #else												// 正式车
            car_moto.car_left = -car_moto.x*TURN_VAR;
            car_moto.car_right= -car_moto.x*TURN_VAR;
        #endif
        control_flag.speed_notific_flag = FALSE;
    }

    // 右转
    else if(car_moto.x>0&&(car_moto.y<-SBUS_siqu|car_moto.y>SBUS_siqu))
    {
        #ifdef QRS_TEST_INVERT1_6			                // 测试车
            car_moto.car_left = car_moto.y;
            car_moto.car_right= -car_moto.y*car_moto.sin;
        #else												// 正式车
            car_moto.car_left = car_moto.y*car_moto.sin;
            car_moto.car_right= -car_moto.y;
        #endif
    }

    // 左转
    else if(car_moto.x<0&&(car_moto.y<-SBUS_siqu|car_moto.y>SBUS_siqu))
    {
        #ifdef QRS_TEST_INVERT1_6			                // 测试车
            car_moto.car_left = car_moto.y*car_moto.sin;
            car_moto.car_right=	-car_moto.y;
        #else												// 正式车
            car_moto.car_left = car_moto.y;
            car_moto.car_right=	-car_moto.y*car_moto.sin;
        #endif
    }

    // 停止
    else
    {
        car_moto.car_left  = 0;
        car_moto.car_right = 0;

        // 清除APP孪生图履带运动
        CLRBIT(mqtt_pub_inform.runvehicleStatus,14);
        control_flag.speed_notific_flag = FALSE;
    }

    // 切换指示灯显示状态
    indicatorLight(INDECATOR_LIGHT_ITEM_MANUAL);

    // 输出电机速度指令
    send_moto_cmd(MOTO1_Velocity, MOTO_REGIS_NUM1, -car_moto.car_left	* ROL_VAR, MOTO_Control_ID);
    send_moto_cmd(MOTO2_Velocity, MOTO_REGIS_NUM1, -car_moto.car_right * ROL_VAR, MOTO_Control_ID);
}

// 车辆运行定速控制模式
void Auto_control(void)
{
    // 置位APP孪生图履带运动
    SETBIT(mqtt_pub_inform.runvehicleStatus,14);

    control_flag.speed_notific_flag = TRUE;
    car_moto.x = Remote[SBUS_X2]-SBUS_zhongzhi;
    car_moto.y = Remote[SBUS_Y2]-SBUS_zhongzhi;

    if(control_flag.Auto_gear_swith != G_0) 			            // 非定速G_0档位，计算左右电机速度
    {
        if(control_flag.Auto_gear_swith == G_1)
        {
            #ifdef QRS_TEST_INVERT1_6	                        // 测试车
                    car_moto.car_left =  GEAR1_SPEED;
                    car_moto.car_right=  -GEAR1_SPEED;
            #else											    // 正式车
                    car_moto.car_left =  GEAR1_SPEED;
                    car_moto.car_right= -GEAR1_SPEED;
            #endif
        }
        else if(control_flag.Auto_gear_swith == G_2)
        {
            #ifdef QRS_TEST_INVERT1_6			                // 测试车
                    car_moto.car_left =  GEAR2_SPEED;
                    car_moto.car_right=  -GEAR2_SPEED;
            #else											    // 正式车
                    car_moto.car_left =  GEAR2_SPEED;
                    car_moto.car_right= -GEAR2_SPEED;
            #endif
        }
        else if(control_flag.Auto_gear_swith == G_3)
        {
            #ifdef QRS_TEST_INVERT1_6			                // 测试车
                car_moto.car_left =  GEAR3_SPEED;
                car_moto.car_right=  -GEAR3_SPEED;
            #else												// 正式车
                car_moto.car_left =  GEAR3_SPEED;
                car_moto.car_right= -GEAR3_SPEED;
            #endif
        }

        if(car_moto.x > 0&&car_moto.y>-SBUS_siqu&&car_moto.y<SBUS_siqu)					// 右转
        {
            #ifdef QRS_TEST_INVERT1_6		                    // 测试车
                car_moto.car_left = car_moto.car_left;
                car_moto.car_right=	car_moto.car_right * (1 - car_moto.x * 0.00138 * 0.5);
            #else												// 正式车
                car_moto.car_left = car_moto.car_left * (1 - car_moto.x * 0.00138 * 0.5);//1/720=0.0013
                car_moto.car_right=	car_moto.car_right;
            #endif
        }
        else if(car_moto.x < 0&&car_moto.y>-SBUS_siqu&&car_moto.y<SBUS_siqu)			// 左转
        {


            #ifdef QRS_TEST_INVERT1_6		                    // 测试车
                car_moto.car_left = car_moto.car_left * (1 + car_moto.x * 0.00138 * 0.5);         //  250/750=0.333定速模式下差速转向最大差250的量
                car_moto.car_right=	car_moto.car_right;
            #else												// 正式车
                car_moto.car_left = car_moto.car_left;
                car_moto.car_right= car_moto.car_right * (1 + car_moto.x * 0.00138 * 0.5);//1/720=0.0013
            #endif
        }
    }
    else if(control_flag.Auto_gear_swith == G_0) 	                // 定速G_0档位，计算左右电机速度
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

//					if(car_moto.x<-SBUS_siqu|car_moto.x>SBUS_siqu && car_moto.y>-SBUS_siqu && car_moto.y<SBUS_siqu)				//原地转
//					{
//							#ifdef QRS_TEST_INVERT1_6	                            // 测试车
//									car_moto.car_left 	=	car_moto.x * TURN_VAR;
//									car_moto.car_right	= 	car_moto.x * TURN_VAR;
//							#else											        // 正式车
//									car_moto.car_left 	=	-car_moto.x * TURN_VAR;
//									car_moto.car_right	= 	-car_moto.x * TURN_VAR;
//							#endif
//					}
//					else if(car_moto.x>-SBUS_siqu && car_moto.x<SBUS_siqu && car_moto.y<0)																		// 后退
//					{
//							#ifdef QRS_TEST_INVERT1_6			                // 测试车
//									car_moto.car_left = car_moto.y;
//									car_moto.car_right= -car_moto.y;
//							#else												// 正式车
//									car_moto.car_left = car_moto.y;
//									car_moto.car_right= -car_moto.y;
//							#endif
//					}

					// 前进后退
					if(car_moto.x==0&&car_moto.y<0)
					{
							#ifdef QRS_TEST_INVERT1_6			                // 测试车
									car_moto.car_left = car_moto.y;
									car_moto.car_right= -car_moto.y;
							#else												// 正式车
									car_moto.car_left = car_moto.y;
									car_moto.car_right= -car_moto.y;
							#endif
					}

					// 原地转

					else if(car_moto.x!=0&&car_moto.y>-SBUS_siqu&&car_moto.y<SBUS_siqu)
					{
							#ifdef QRS_TEST_INVERT1_6			                // 测试车
									car_moto.car_left = car_moto.x*TURN_VAR;
									car_moto.car_right= car_moto.x*TURN_VAR;
							#else												// 正式车
									car_moto.car_left = -car_moto.x*TURN_VAR;
									car_moto.car_right= -car_moto.x*TURN_VAR;
							#endif
							control_flag.speed_notific_flag = FALSE;
					}

					// 右转
					else if(car_moto.x>0&&car_moto.y<-SBUS_siqu)
					{
							#ifdef QRS_TEST_INVERT1_6			                // 测试车
									car_moto.car_left = car_moto.y;
									car_moto.car_right= -car_moto.y*car_moto.sin;
							#else												// 正式车
									car_moto.car_left = car_moto.y*car_moto.sin;
									car_moto.car_right= -car_moto.y;
							#endif
					}

					// 左转
					else if(car_moto.x<0&&car_moto.y<-SBUS_siqu)
					{
							#ifdef QRS_TEST_INVERT1_6			                // 测试车
									car_moto.car_left = car_moto.y*car_moto.sin;
									car_moto.car_right=	-car_moto.y;
							#else												// 正式车
									car_moto.car_left = car_moto.y;
									car_moto.car_right=	-car_moto.y*car_moto.sin;
							#endif
					}
					else
					{
							car_moto.car_left =  0;
							car_moto.car_right=  0;

							// 清除APP孪生图履带运动
							CLRBIT(mqtt_pub_inform.runvehicleStatus,14);
							control_flag.speed_notific_flag = FALSE;
					}

				}
				else
				{
					car_moto.car_left =  0;
					car_moto.car_right=  0;

					// 清除APP孪生图履带运动
					CLRBIT(mqtt_pub_inform.runvehicleStatus,14);
					control_flag.speed_notific_flag = FALSE;
				}

		}

    // 切换指示灯显示状态
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


    // 输出电机速度指令
    send_moto_cmd(MOTO1_Velocity, MOTO_REGIS_NUM1, -car_moto.car_left  * ROL_VAR, MOTO_Control_ID);
    send_moto_cmd(MOTO2_Velocity, MOTO_REGIS_NUM1, -car_moto.car_right * ROL_VAR, MOTO_Control_ID);
}



/*************************************************************交互控制动作**************************************************************/

// 遥控器解锁动作
void sbus_unlock(void)
{
    // 电机驱动器复位
	send_moto_cmd(MOTO_RESET_STATE, MOTO_REGIS_NUM1, 0xFF00, MOTO_Control_ID);  //遥控器上锁来复位电机驱动器故障状态

//    // 电机驱动器开启零速制动
//	send_moto_cmd(MOTO_SysBitM, MOTO_REGIS_NUM1, 0x2000, MOTO_Control_ID);  //遥控器上锁来复位电机驱动器故障状态

    // 运动控制
    control_flag.Auto_gear_swith 				= G_0;			                // 定速模式归零档，防止上锁后自动挡模式下升档

	// 控制两个电机启停寄存器启动（电机1、2）
    send_moto_cmd(MOTO1_Launch, MOTO_REGIS_NUM1, 0xFF00, MOTO_Control_ID);
	send_moto_cmd(MOTO2_Launch, MOTO_REGIS_NUM1, 0xFF00, MOTO_Control_ID);

	// 解锁的瞬间记录运行记录的开始数据
	time_t changeUTC = timer_info.timestamp;//不+28800，IOT目前用UTC
	memcpy(sing_work_event.start_date, TimeStampToString(&changeUTC),14);// 从不断累计得时间戳转化成时间字符串记录开始时间

	sing_work_event.start_power = battery_data.soc;						   		// 记录开始电量
	sing_work_event.mileage = 0;												// 解锁前清零一次里程

    // 切换GPS上报频率
    gps_info.gpsUrcSet = 1;

    // 语音播报“解锁”
	speakItem(SPEAK_ITEM_SBUS_UNLOCK);

    // 切换指示灯显示状态
	indicatorLight(INDECATOR_LIGHT_ITEM_MANUAL);
}

// 遥控器上锁操作
void sbus_lock(void)
{
    // 控制两个电机速度寄存器归零
    send_moto_cmd(MOTO1_Velocity, MOTO_REGIS_NUM1, 0, MOTO_Control_ID);
    send_moto_cmd(MOTO2_Velocity, MOTO_REGIS_NUM1, 0, MOTO_Control_ID);

    // 控制两个电机启停寄存器停止（电机1、2）
    send_moto_cmd(MOTO1_Launch, MOTO_REGIS_NUM1, 0x0000, MOTO_Control_ID);
    send_moto_cmd(MOTO2_Launch, MOTO_REGIS_NUM1, 0x0000, MOTO_Control_ID);

    // 自动档档位归零
    control_flag.Auto_gear_swith 		= G_0;

    // 写运行里程
    countdata_handle();

    // 语音播报“上锁”
    speakItem(SPEAK_ITEM_SBUS_LOCK);

    // 切换指示灯显示状态
    indicatorLight(INDECATOR_LIGHT_ITEM_OPEN);

    // 关闭大灯
//    control_flag.Led48v_swith       = FALSE;
//    Led48v_control();

    // 关闭风机
    control_flag.Draught_swith      = FALSE;
    Draught_control();

    // 关闭水泵
    control_flag.Pump_swith         = G_0;
    Pump_control();

    // 切换GPS上报频率
    gps_info.gpsUrcSet = 10;
}

// 遥控器断连操作
void sbus_unconnect(void)
{
    // 控制两个电机速度寄存器归零
    send_moto_cmd(MOTO1_Velocity, MOTO_REGIS_NUM1, 0, MOTO_Control_ID);
    send_moto_cmd(MOTO2_Velocity, MOTO_REGIS_NUM1, 0, MOTO_Control_ID);

    // 自动档档位归零
    control_flag.Auto_gear_swith 	= G_0;

    // 遥控器恢复上锁状态
    control_flag.Sbus_lock_flag     = FALSE;

    // 关闭大灯
    control_flag.Led48v_swith       = FALSE;
    Led48v_control();

    // 关闭风机
    control_flag.Draught_swith      = FALSE;
    Draught_control();

    // 关闭水泵
    control_flag.Pump_swith         = G_0;
    Pump_control();

    // 切换GPS上报频率
    gps_info.gpsUrcSet = 10;
}

// 软开关开机动作
void soft_sw_clickON(void)
{

    // 接触器连接
    driverBoard_contactorState = 1;
    WriteSDO(&CANopen_Master_M200_Data, CANDevice_NodeID_driver, DRIVER_INDEX_CONTACTOE, DRIVER_SUB_INDEX_DEFAULT, 1, uint8, &driverBoard_contactorState, 0);

    // 开启散热风扇
    control_flag.Boxfan_swith       	 = ENABLE;		// 开机箱体风扇开关使能
    control_flag.Carfan_swith      		 = ENABLE;		// 开机车体风扇开关使能

    // 读取汇总信息
    countdata_read();

    // 打开接收机供电
    HAL_GPIO_WritePin(SBUS_PWR_GPIO_Port, SBUS_PWR_Pin, GPIO_PIN_RESET);

    // 切换指示灯显示状态
    indicatorLight(INDECATOR_LIGHT_ITEM_OPEN);

    // 开机电量播报
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

    // 开启蓝牙
    ble_inform.ble_switch = TRUE;

		//QC150电池接触器吸合到跳转到开机状态给大灯控制信号之前不加延时导致驱动器过载保护
		vTaskDelay(1000);
}

// 软开关关机动作
void soft_sw_clickOFF(void)
{
    // 写运行时间
    counttime_handle();

    // 自动档档位归零
    control_flag.Auto_gear_swith 		= G_0;

    // 遥控器恢复上锁状态
    control_flag.Sbus_lock_flag = FALSE;

    // 关闭散热风扇
    control_flag.Boxfan_swith           = DISABLE;
    control_flag.Carfan_swith      	    = DISABLE;

    // 关闭大灯
    control_flag.Led48v_swith       = FALSE;
    Led48v_control();

    // 关闭风机
    control_flag.Draught_swith      = FALSE;
    Draught_control();

    // 关闭水泵
    control_flag.Pump_swith         = G_0;
    Pump_control();

    // 关闭接收机供电
    HAL_GPIO_WritePin(SBUS_PWR_GPIO_Port, SBUS_PWR_Pin, GPIO_PIN_SET);

    // 切换指示灯显示状态
    battery_data.socState = SOC_ENOUGH;
    indicatorLight(INDECATOR_LIGHT_ITEM_CLOSE);

    // 播报关机和剩余电量
    speakItem(SPEAK_ITEM_CLOSE);
    speakItem(SPEAK_ITEM_SOC(battery_data.soc));
    if(battery_data.soc < BatteryHighSOC)
    {
        speakItem(SPEAK_ITEM_CHARGE_REMIND);		//语音播报“请及时充电”
    }

    // 关闭蓝牙
    ble_inform.ble_switch = FALSE;

    osDelay(1000);

    // 接触器断开
    driverBoard_contactorState = 0;
    WriteSDO(&CANopen_Master_M200_Data, CANDevice_NodeID_driver, DRIVER_INDEX_CONTACTOE, DRIVER_SUB_INDEX_DEFAULT, 1, uint8, &driverBoard_contactorState, 0);

    // 切换GPS上报频率
    gps_info.gpsUrcSet = 10;
}

// 急停打开动作
void scram_sw_clickON(void)
{

    // 断开接触器
    driverBoard_contactorState = 0;
    WriteSDO(&CANopen_Master_M200_Data, CANDevice_NodeID_driver, DRIVER_INDEX_CONTACTOE, DRIVER_SUB_INDEX_DEFAULT, 1, uint8, &driverBoard_contactorState, 0);

    // 写运行里程
	counttime_handle();

    // 切换指示灯显示状态
	indicatorLight(INDECATOR_LIGHT_ITEM_FAULT);

    // 关闭大灯
    control_flag.Led48v_swith       = FALSE;
    Led48v_control();

    // 关闭风机
    control_flag.Draught_swith      = FALSE;
    Draught_control();

    // 关闭水泵
    control_flag.Pump_swith         = G_0;
    Pump_control();

    // 自动档档位归零
    control_flag.Auto_gear_swith 	= G_0;

    // 切换GPS上报频率
    gps_info.gpsUrcSet = 10;
}

// 车辆上电初始化动作
void car_state_Init(void)
{
    // 关闭大灯
    control_flag.Led48v_swith       = FALSE;
    Led48v_control();

    // 关闭风机
    control_flag.Draught_swith      = FALSE;
    Draught_control();

    // 关闭水泵
    control_flag.Pump_swith         = G_0;
    Pump_control();

    //启停命令操作两个寄存器（电机1、2） 停止
    send_moto_cmd(MOTO1_Launch, MOTO_REGIS_NUM1, 0x0000, MOTO_Control_ID);
    send_moto_cmd(MOTO2_Launch, MOTO_REGIS_NUM1, 0x0000, MOTO_Control_ID);
}

// 充电器连接动作
void power_connect(void)
{
    // 切换指示灯显示模式
	battery_data.socState = SOC_ENOUGH;
	indicatorLight(INDECATOR_LIGHT_ITEM_RECHARGE);

    // 语音播报“电源已连接”
	speakItem(SPEAK_ITEM_POWER_CONNECT);

    // 打开散热风扇
    control_flag.Boxfan_swith           = ENABLE;
    control_flag.Carfan_swith      	    = ENABLE;
}

// 充电完毕动作
void charge_reset(void)
{
    // 充电参数恢复
    battery_data.spentChargeTime  = 0;		//充电完毕清零充电花费时间
	battery_data.chargeTimeRemain = 0;		//充电完毕清零充电剩余时间

	// 切换指示灯显示模式
	indicatorLight(INDECATOR_LIGHT_ITEM_CLOSE);

    // 关闭散热风扇
    control_flag.Carfan_swith      		 	= DISABLE;
    control_flag.Boxfan_swith      		 	= DISABLE;
}

/*************************************************************车辆运行状态机***********************************************************/

// 车辆运行状态机
void car_state_trans(void)
{
	switch(control_flag.Car_State)
	{
        //------------------------------------关机状态--------------------------------------------------------关机状态------------------------------------------------------关机状态-----------------------------------//
        case CLOSE:
            {
                // 开机初始化
                if(control_flag.Init_flag == TRUE)
                {
                    control_flag.Init_flag             = FALSE;
                    car_state_Init();
                }

                // 充电器连接
                if(battery_data.charge_flag == TRUE)                    // CAN接收监听到电池处于充电状态，充电标志位位真
                {
                    mqtt_pub_inform.start_charge_flag   = ENABLE;		// 检测到充电电源连接上报一次最后充电时间
                    power_connect();
                    control_flag.Car_State 				= RECHARGE;	    // 跳转至充电状态
                }

                // 软开关打开
                if(softSwitch_switchFlag == TRUE)
                {
                    soft_sw_clickON();
										control_flag.lock_check_flag = TRUE;//开机时检查一次水泵是否关闭
                    control_flag.Car_State 				 = OPEN;		// 跳转至就绪待机状态
                }

                //上报错误复位
				if(control_flag.error_res_flag == TRUE) {
					HAL_NVIC_SystemReset();
				}

            }
            break;


        //------------------------------------空闲状态--------------------------------------------------------空闲状态------------------------------------------------------空闲状态-----------------------------------//
        case OPEN:
            {
                // 遥控器解锁
                if(control_flag.Sbus_lock_flag == TRUE && control_flag.Sbus_connect_flag == TRUE)
                {
                    sbus_unlock();
                    mqtt_pub_inform.runvehicleStatus = 0xffff&0x0213;   // 运行模式下先设置成全关，自动喷洒关，手动
                    sing_work_event.send_count = FALSE;
                    control_flag.Car_State = RUN;
                }

                // 软开关关闭
                if(softSwitch_switchFlag == FALSE)
                {
                    soft_sw_clickOFF();
                    control_flag.Car_State 				= CLOSE;	    // 跳转至关机（低功耗）状态
                    mqtt_pub_inform.start_summary_flag  = ENABLE;		// 软开关关机时上报一次汇总数据
                }

//							// 车身设备控制
								Led48v_control();
            }
            break;


        //------------------------------------运行状态--------------------------------------------------------运行状态------------------------------------------------------运行状态-----------------------------------//
        case RUN:
            {
                /**************************有转向动作*************************/
                if(control_flag.save_turn_flag == RESET)
                {
                    if(Remote[SBUS_X2] < SBUS_mid_L_limit | Remote[SBUS_X2] > SBUS_mid_R_limit)
                    {
                        control_flag.count_turn++;				// 遥控器接受空闲中断里已累计10次处理一次
                        if(control_flag.count_turn > 20)	    // 遥感打方向超过两秒存储一次坐标
                        {
                            control_flag.save_turn_flag = SET;  // 遥控器遥感转向判定，存储坐标使能
                            control_flag.count_turn = 0;
                        }
                    }
                    else
                    {
                        control_flag.count_turn = 0;
                    }
                }
                control_flag.Draught_open_flag = TRUE;

                /**************************遥控器解锁*************************/
                if(control_flag.Sbus_lock_flag == TRUE)
                {

                    // 车身设备控制
                    Led48v_control();

                    // 喷洒系统控制
                    if(control_flag.Auto_spray_swith == TRUE && control_flag.speed_notific_flag == FALSE)
                    {
                        control_flag.Pump_swith			= G_0;
                        control_flag.Draught_swith 	    = FALSE;
                    }
                    Pump_control();
                    Draught_control();

                    // 车辆运动控制
                    if(control_flag.Auto_swith== DISABLE)
                    {
                        hunkong(Remote[SBUS_X2], Remote[SBUS_Y2]);	// 正常行驶控制
                        control_flag.Auto_gear_swith = G_0;			// 定速巡航速度设置为G0
                    }
                    else if(control_flag.Auto_swith == ENABLE)
                    {
                        Auto_control();								// 定速巡航行驶控制
                    }
                }

                /**************************遥控器上锁*************************/
                else
                {
                    sbus_lock();												// 遥控器上锁，遥控器失控保护值会导致遥控器上锁

                    sing_work_event.send_count 					= TRUE;
                    control_flag.event_mpub_single_flag 	    = TRUE;         // 上报事件

										control_flag.lock_check_flag = TRUE;//下次解锁时检查一次水泵开关是否关闭

                    control_flag.Car_State 					    = OPEN;			// 跳转至关机（低功耗）状态
                }

                /**************************遥控器断连*************************/
                if(control_flag.Sbus_connect_flag == FALSE)
                {
                    // 执行遥控器断连操作
                    sbus_lock();

                    //遥控器连接上后解下次锁前检查一次水泵是否关闭
					control_flag.lock_check_flag = TRUE;
					control_flag.Sbus_lock_flag  = FAULT;
                    // 状态切换
                    control_flag.Car_State 			= OPEN;
                }

                /**************************急停拍下*************************/
                if(driverBoard_scramStop == TRUE)
                {
                    // 上报一次汇总数据
                    mqtt_pub_inform.start_summary_flag          = ENABLE;

                    // 执行急停按下操作
                    scram_sw_clickON();

					// 状态切换
                    control_flag.Car_State 						= FAULT;

                    // 上报事件
                    sing_work_event.send_count 					= TRUE;
                    control_flag.event_mpub_single_flag 	    = TRUE;

                }

                /**************************软开关关闭*************************/
                if(softSwitch_switchFlag == FALSE)
                {
                    // 上报一次汇总数据
                    mqtt_pub_inform.start_summary_flag          = ENABLE;

                    // 记录运行总里程
                    countdata_handle();

                    // 软开关关机操作
                    soft_sw_clickOFF();

                    // 上报事件
                    sing_work_event.send_count 					= TRUE;			// 遥控器上锁上报事件2个
                    control_flag.event_mpub_single_flag 	    = TRUE;			// 运行状态跳到休眠状态（用户关机）后上报一次运行记录事件

                    // 状态切换
                    control_flag.Car_State 						= CLOSE;		// 跳转至关机（低功耗）状态
                }
            }
            break;


        //------------------------------------充电状态--------------------------------------------------------充电状态------------------------------------------------------充电状态-----------------------------------//
        case RECHARGE:
            {
                /**************************电已充满*************************/
                if(battery_data.charge_flag == FALSE)
                {
                    charge_reset();
                    battery_data.charge_overflag       	    = ENABLE;		// 充电完毕发送充电剩余时间0
                    control_flag.Car_State 					= CLOSE;		// 跳转至关机状态
                }

                /**************************遥控器断连*************************/
                //if(control_flag.Sbus_connect_flag == FALSE)
                //{
                //    // 执行遥控器断连操作
                //    sbus_unconnect();
                //}
            }
            break;


        //------------------------------------故障状态--------------------------------------------------------故障状态------------------------------------------------------故障状态-----------------------------------//
        case FAULT:
            {
                /**************************软开关关机*************************/
                if(softSwitch_switchFlag == FALSE)
                {
                    soft_sw_clickOFF();
                    control_flag.Car_State 				 	= CLOSE;
                }
            }
            break;


        //------------------------------------默认状态--------------------------------------------------------默认状态------------------------------------------------------默认状态-----------------------------------//
        default:
            break;
		}
}


































/************************************未使用的函数****************************************/



///*************检测到遥控器所有键位是否复原*************/
//uint8_t sbus_check(void)
//{
//	if(control_flag.Sbus_connect_flag == TRUE    &&     //遥控器已连接
//		control_flag.Pump_swith         == G_0     &&     //水泵档位为0
//		 Remote[SBUS_C]  >=  SBUS_up_limit         &&     //风机关闭
//	   control_flag.Auto_spray_swith  == FALSE   &&     //自动喷洒启停关闭
//	   control_flag.Auto_swith        == DISABLE )      //自动档关闭
//		return TRUE;
//	else
//		return FALSE;
//}

//uint8_t aTxBuffer[WR_DATA_LEN] = "0123456789ABCDEF";
//uint8_t aRxBuffer[WR_DATA_LEN] = {0};

