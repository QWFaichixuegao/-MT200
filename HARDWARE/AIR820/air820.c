#include "air820.h"
#include "handle.h"
#include "math.h"
#include "stdlib.h"
#include "CANopen_Master_M200.h"



extern osSemaphoreId BinarySem01Handle;
OTA_INFORM                  ota_inform;
AIR_4g_FLAG 				air_4g_flag;       //4G模块标志位
AIR_4g_CONNECT 				air_4g_connect;    //4G模块连接计数
USARTX_HANDLE               usart3_handle_4g;
DEVICE_INFORM 				device_inform;
MQTT_PUB_INFORM  			mqtt_pub_inform;
MQTT_OTA_INFORM 			mqtt_ota_inform;
SING_WORK_EVENT			    sing_work_event;
GPS_HANDLE 					gps_info;
BLE_INFORM 					ble_inform;
TIMER_HANDLE1 		        timer_info;
DESIRED_INFORM             desired_inform;
SBUS_PACK 					sbus_pack_data;


// 字符串转时间戳函数
time_t StringToTimeStamp(uint8_t* timeStr)
{
	struct tm tm_;
	int year, month,day,hour,minute,second;
	sscanf((const  char*)timeStr, "%4d%2d%2d%2d%2d%2d", &year, &month, &day, &hour, &minute, &second);
	tm_.tm_year  = year-1900;
	tm_.tm_mon   = month-1;
	tm_.tm_mday  = day;
	tm_.tm_hour  = hour;
	tm_.tm_min   = minute;
	tm_.tm_sec   = second;
	tm_.tm_isdst = 0;

	time_t timeStamp = mktime(&tm_);
	return timeStamp-28800;//目前时间获取变为北京时区，mktime转换的是utc时间所以减掉多的8小时秒数
}

// 时间戳转字符串函数
char *TimeStampToString(time_t* timeStamp)
{
	struct tm *info;
	char* buffer_add;
	char 	buffer[16];
	info = localtime(timeStamp);//目前时间戳获取变为北京时区，localtime转换的是utc时间所以减掉多的8小时秒数
	strftime(buffer, 16, "%Y%m%d%H%M%S", info);
	buffer_add = buffer;
	return buffer_add;
}


// 4G模块初始化
void air_4g_init(uint8_t connectMode, uint8_t delay_way)
{
    // 4G重启时不发送串口3处理信号量
    //control_flag.Usart3_handle_flag = FALSE;

    // 等待4g模块启动，13S
    if(delay_way)  vTaskDelay (20000); else  HAL_Delay(20000);

    // 关闭主动上报的信息，可以关3次，确保关闭
    air4g_send_cmd("AT*CSQ=0\r", "OK", 20, delay_way);
    air4g_send_cmd("AT+CGNSURC=0\r" , "OK" , 20 , delay_way);//关闭处理后的GPS信息周期上报
		air4g_send_cmd("AT+CGNSPWR=0\r" , "OK" , 20,delay_way);//关闭GPS

    // 回显模式关闭
    for(uint8_t i = 0; i < 5; i++)
    {
        if(delay_way)  vTaskDelay (1000); else  HAL_Delay(1000);
        if(!air4g_send_cmd("ATE0\r", "OK", 500, delay_way))
            break;
    }

    // 检查车辆和MQTT服务器是否连接
    if(!air4g_send_cmd("AT+MQTTSTATU\r", "+MQTTSTATU :1", 150, delay_way))
    {
        air_4g_flag.MQTT_flag = TRUE;
        air_4g_flag.SIM_flag = TRUE;
    }

    // 检查蜂窝网络连接
    else if(air_4g_net_check(delay_way) == SIM_OK)
    {
        air_4g_flag.SIM_flag = TRUE;
    }

    // 车辆连接MQTT服务器
    if((air_4g_flag.SIM_flag == TRUE) && (air_4g_flag.MQTT_flag == FALSE))
    {
        if(air_4g_connect_server(connectMode,delay_way) == MQTT_CONNECT_OK)
        {
            // 检查MQTT连接状态
            // air4g_send_cmd("AT+MQTTSTATU\r", "OK", 500, delay_way);
            air_4g_flag.MQTT_flag = TRUE;
            //control_flag.Usart3_handle_flag = TRUE;
        }
    }

    // 检查车辆和MQTT服务器是否连接
    if(!air4g_send_cmd("AT+MQTTSTATU\r", "+MQTTSTATU :1", 150, delay_way))
    {
        air_4g_OTAMPUB(delay_way);
        air_4g_OTASUB(delay_way);
        topic_sub(delay_way);
        air_4g_flag.MQTT_flag = TRUE;
    }

    // 打开主动上报的GPS和CSQ
    air_4g_openURC(delay_way);

    //air4g_send_cmd("AT+ICCID\r", "OK" , 100, air_4g_flag.vTa_delay);            // 获取物联网卡ID
}

// 打开主动上报
uint8_t air_4g_openURC(uint8_t delay_way)
{
    // 打开GPS
    air4g_send_cmd("AT+CGNSPWR=1\r", "OK" , 100, delay_way);
    // 使能辅助定位
		air4g_send_cmd("AT+CGNSAID=31,1,1,1\r", "OK", 100, delay_way);
    // 设置GPS上报频率为10秒1次
    air4g_send_cmd("AT+CGNSURC=10\r", "OK", 100, delay_way);
    //当前为10S一次  当其与设定值(gpsUrcSet)不一致时，会在任务一中紧接着调整GPS频率
    gps_info.gpsUrc                             = 10;
    return 0;
}

// 关闭主动上报
uint8_t air_4g_closeURC(uint8_t delay_way)
{
    // 设置GPS上报频率为1秒0次，关闭
    air4g_send_cmd("AT+CGNSURC=0\r", "OK", 100, delay_way);
    return 0;
}


// 4G模块重启
uint8_t air_4g_restar(uint8_t delay_way)
{
    // 4G模块复位，要不要采用带接收的方式？
    char buf[11];
    sprintf(buf,"AT+RSTSET\r");
    HAL_UART_Transmit_DMA(&huart3,(uint8_t*)buf,strlen(buf));

    air_4g_init(0,1);
    return	0;
}

// 4G模块检查网络状态
uint8_t air_4g_net_check(uint8_t delay_way)
{
	// 回显模式关闭
    // air4g_send_cmd("ATE0\r" , "OK" , 500,delay_way);

    // 查询PIN码，返回OK正常，返回err:10未检测到卡，查询是否有SIM插入
    if(air4g_send_cmd("AT+CPIN?\r" , "READY" , 500,delay_way))
        return SIM_CPIN_ERR;

    // 查询网络注册状态，暂时没有对结果做处理
    if(air4g_send_cmd("AT+CEREG?\r", "+CEREG: 0,1", 500, delay_way))
        return SIM_CREG_ERR;

    // 查询GPRS附着状态，+CGATT:1正常附着网络 +CGATT:0 未附着网络，如果未附着，返回附着错误
    if(air4g_send_cmd("AT+CGATT?\r", "+CGATT: 1", 500, delay_way))
        return SIM_CGATT_ERR;
    return SIM_OK;
}

// 4G模块连接到MQTT服务器
uint8_t air_4g_connect_server(uint8_t connectMode, uint8_t delay_way)
{

  if(connectMode)
  {
    // 关闭MQTT连接，刚开机本条指令会返回767错误
    if(air4g_send_cmd("AT+MDISCONNECT\r", "OK" , 50,delay_way))
        return MQTT_MDISCONNECT_ERR;

    // 关闭TCP连接,刚开机本条指令会返回767错误
    if(air4g_send_cmd("AT+MIPCLOSE\r", "OK" , 50,delay_way))
        return MQTT_MIPCLOSE_ERR;

    // IP应用设置，IP，本条指令会返回3错误
    air4g_send_cmd("AT+SAPBR=0,1\r", "OK" , 50,delay_way);
	}

	// 设置承载类型为GPRS
	air4g_send_cmd("AT+SAPBR=3,1,\"CONTYPE\",\"GPRS\"\r" , "OK" , 500,delay_way);

	// 设置PDP承载的APN参数
	air4g_send_cmd("AT+SAPBR=3,1,\"APN\",\"\"\r" , "OK" , 500,delay_way);

	// 发起PDP激活的请求
	air4g_send_cmd("AT+SAPBR=1,1\r" , "OK" , 500,delay_way);

    // 查询PDP地址，即4G入网后获得IP，暂时没有解析IP，返回第1个1是承载上下文表示，返回第2个1是表示承载已经连接
	if(air4g_send_cmd("AT+SAPBR=2,1\r" , "+SAPBR: 1,1" , 200,delay_way))
        return PDP_ACTIVE_ERR;

	// 设置MQTT相关参数
	sprintf(device_inform.AtStrBuf,"AT+MCONFIG=\"%s.%s|securemode=2,signmethod=hmacsha256,timestamp=%s|\",\"%s&%s\",\"%s\"\r",
			device_inform.ProductKey, device_inform.DeviceName, device_inform.timestamp, device_inform.DeviceName, device_inform.ProductKey, device_inform.Password);
	if(air4g_send_cmd(device_inform.AtStrBuf , "OK" , 500,delay_way))
		return	MQTT_MCONFIG_ERR;

	// 建立TCP连接
	sprintf(device_inform.AtStrBuf,"AT+MIPSTART=\"%s\",%s\r",
			device_inform.server_host,device_inform.server_port);
	if(air4g_send_cmd_mqtt(device_inform.AtStrBuf , "CONNECT" , "OK" , 500,delay_way))
		return	MQTT_MIPSTART_ERR;

	// MQTT连接成功分两段会送，第一个OK是回应发送指令，第二个CONNECT OK才是连接成功，未连接成功发送心跳保活会失败
	// 客户端向服务器请求会话连接	AT+MCONNECT=<clean_session>, <keepalive>
	if(air4g_send_cmd_mqtt("AT+MCONNECT=0,60\r" , "CONNACK" , "OK" , 400,delay_way))
		return	MQTT_MCONNECT_ERR;

	return	MQTT_CONNECT_OK;
}


// 4G模块指令发送函数: sendcmd:待发送指令   checkbuf:校对指令  waitsec:等待时间   delay_mode:延时函数使用模式
uint8_t air4g_send_cmd(char* sendcmd, char* checkbuf, uint16_t waitsec, uint8_t delay_mode )
{
	uint8_t len_scmd;

    // 获取数据长度及发送数据
	len_scmd = strlen(sendcmd);
	HAL_UART_Transmit_DMA(&huart3, (uint8_t*)sendcmd, len_scmd);

    // 延时
	if(delay_mode)  vTaskDelay (waitsec); else  HAL_Delay(waitsec);

    // 判断AT指令返回结果
	if(strstr((char*)usart3_handle_4g.save_buf, checkbuf) == NULL )
	{
		memset(usart3_handle_4g.save_buf,0,SAVE_SIZE);
		return  1;
	}

    // 清空串口接收缓冲区
	memset(usart3_handle_4g.save_buf,0,SAVE_SIZE);
	return  0;
}


// 4G模块MQTT指令发送函数:sendcmd:待发送指令   checkbuf:校对指令  checkbu2:校对指令  waitsec:等待时间   delay_mode:延时函数使用模式
uint8_t air4g_send_cmd_mqtt(char* sendcmd, char* checkbuf, char* checkbu2, uint16_t waitsec, uint8_t delay_mode )
{
	uint8_t len_scmd;

    // 获取数据长度及发送数据
	len_scmd = strlen(sendcmd);
	HAL_UART_Transmit_DMA(&huart3,(uint8_t*)sendcmd,len_scmd);

    // 延时
	if(delay_mode)  vTaskDelay(waitsec);    else  HAL_Delay  (waitsec);

    // 判断AT指令返回结果
	if((strstr((char*)usart3_handle_4g.save_buf,checkbuf) || strstr((char*)usart3_handle_4g.save_buf,checkbu2))==NULL)
	{
		memset(usart3_handle_4g.save_buf,0,SAVE_SIZE);
		return 1;
	}

    // 清空串口接收缓冲区
	memset(usart3_handle_4g.save_buf,0,SAVE_SIZE);

	return 0;
}

// 获取4G实时时间
void get_real_time(uint8_t delay_way)
{
	air4g_send_cmd("AT+CCLK?\r", "OK", 100, delay_way);
}

// 获取4G当前信息
void get_4G_msg(uint8_t delay_way)
{
	air4g_send_cmd("ATE0\r", "OK", 50, delay_way);
  air4g_send_cmd("AT+CSQ\r", "OK", 50, delay_way);		//查询4G信号质量
	air4g_send_cmd("AT+MQTTSTATU\r", "+MQTTSTATU", 50, delay_way);	//查询MQTT连接状态
	//..
}

//GPS关闭并重新打开
void gpsReset(uint8_t delay_way)
{
  air4g_send_cmd("ATE0\r" , "OK" , 20,delay_way);
  air4g_send_cmd("AT+CGNSURC=0\r" , "OK" , 100 , delay_way);//关闭处理后的GPS信息周期上报
  air4g_send_cmd("AT+CGNSPWR=0\r" , "OK" , 100,delay_way);//关闭GPS

  air4g_send_cmd("AT+CGNSPWR=1\r" , "OK" , 100 , delay_way);//打开GPS
  air4g_send_cmd("AT+CGNSAID=31,1,1,1\r" , "OK" , 1000,delay_way);//使能辅助定位
  gps_info.gpsUrc = 10;//GPS重新打开后默认当前GPS上报频率为10秒1次
}

// 打开蓝牙
void ble_swit_on(uint8_t delay_way)
{
    // 打开蓝牙及从模式
	air4g_send_cmd("AT+BTCOMM=ENABLE,1,0\r" , "OK" , 1800 , delay_way);

    // 设置蓝牙名称
	air4g_send_cmd(ble_inform.ble_name, "OK" , 200, delay_way);

    // 设置广播包数据，0x02 0x01 0x06 Header固定 0x03(三位) 0xff(厂家自定义的type) 0x00 0x01 (可用于指示256*256=65536台设备)
	air4g_send_cmd("AT+BLEADV=ADVDATA,7,02010603ff0001\r" , "OK" , 200, delay_way);

    // 设置响应包数据
    air4g_send_cmd("AT+BLEADV=SCANRSPDATA,4,03ff0001\r" , "OK" , 200, delay_way);

    // 打开广播等待连接成功
    air4g_send_cmd("AT+BLEADV=ENABLE,1\r" , "OK" , 300, delay_way);
}

// 关闭蓝牙
void ble_swit_off(bool delay_way)
{
    // 关闭广播
	air4g_send_cmd("AT+BLEADV=ENABLE,0\r" , "OK" , 300 , delay_way);

    // 关闭蓝牙及从模式
	air4g_send_cmd("AT+BTCOMM=ENABLE,0,0\r" , "OK" , 200,delay_way);
}


// 检查4G模块发送的信息是否为自动解析的数据
void air4gRecCheck(void)
{
    if((strstr((char*)usart3_handle_4g.rx_buf,"+UGNSINF: 1")!=NULL)||
		(strstr(usart3_handle_4g.rx_buf,"+CSQ:")!=NULL)||
		(strstr(usart3_handle_4g.rx_buf,"+BLEIND=DATA")!=NULL)||
		(strstr(usart3_handle_4g.rx_buf,"+MQTTSTATU")!=NULL)||
		(strstr(usart3_handle_4g.rx_buf,"+CCLK:")!=NULL)||
        (strstr(usart3_handle_4g.rx_buf,"ERROR")!=NULL)||
        (strstr(usart3_handle_4g.rx_buf,"property/set")!=NULL)
        //(strstr(usart3_handle_4g.rx_buf,"303324")!=NULL)
        )
    {
        //if(control_flag.Init_flag == FALSE)
        //{
            memcpy(usart3_handle_4g.report_buf,usart3_handle_4g.rx_buf,usart3_handle_4g.rx_len);
            osSemaphoreRelease(BinarySem01Handle);
        //}
    }
    else
    {
		memcpy(usart3_handle_4g.save_buf,usart3_handle_4g.rx_buf,usart3_handle_4g.rx_len);
    }
}


// 4G模块上报当前固件信息
void air_4g_OTAMPUB(uint8_t delay_way)
{

    // 上报主控固件版本
	sprintf(mqtt_ota_inform.PubBuf,"AT+MPUB=\"%s\",0,0,\"{\\22id\\22:203%d,\\22params\\22:{\\22version\\22:\\22%s\\22}}\"\r"
								,mqtt_ota_inform.version_theme_str
								,device_inform.upgrade_flag
								,device_inform.version
							);
    for(uint8_t i = 0; i < 5; i++)
    {
        if(!air4g_send_cmd(mqtt_ota_inform.PubBuf , "OK" , 200 , delay_way))
        {
            break;
        }
        else
        {
            if(delay_way)  vTaskDelay (500); else  HAL_Delay(500);
        }
    }
	//HAL_UART_Transmit_DMA(&huart3, (uint8_t*)mqtt_ota_inform.PubBuf, strlen(mqtt_ota_inform.PubBuf));
}

// 订阅主题设备拉取OTA固件升级
void air_4g_OTASUB(uint8_t delay_way)
{
    sprintf(ota_inform.PubBuf,"AT+MSUB=\"%s\",1\r"
                                ,ota_inform.msubOTA_theme_str
                            );
    air4g_send_cmd(ota_inform.PubBuf , "OK" , 200 , delay_way);
    //HAL_UART_Transmit_DMA(&huart3, (uint8_t*)ota_inform.PubBuf, strlen(ota_inform.PubBuf));
}

// 4G发起HTTP的GET请求
void air_4g_http_otarequest(void)
{
    // 终止HTTP服务
    air4g_send_cmd("AT+HTTPTERM\r" , "OK" , 100, 1);

    // 初始化HTTP服务
    air4g_send_cmd("AT+HTTPINIT\r" , "OK" , 100, 1);

    // 设置会话参数CID
    air4g_send_cmd("AT+HTTPPARA=\"CID\",1\r" , "OK" , 100, 1);

    // 设置会话参数 URL
    sprintf(ota_inform.ota_http_urlATcmd,"AT+HTTPPARA=\"URL\",\"%s\"\r",ota_inform.ota_http_url);
    air4g_send_cmd(ota_inform.ota_http_urlATcmd, "OK", 100, 1);

    //发起HTTP GET 请求
    air4g_send_cmd("AT+HTTPACTION=0\r", "OK" , 200, 1);

    //sprintf(buf,"AT+HTTPACTION=0\r");
    //HAL_UART_Transmit_DMA(&huart3,(uint8_t*)buf,strlen(buf));
}

// 设备主动拉去固件升级信息
void air_4g_OTAREQUEST(void)
{
    sprintf(mqtt_ota_inform.PubBuf,"AT+MPUB=\"%s\",0,0,\"{\\22id\\22:\\22202\\22,\\22params\\22:{\\22version\\22:\\22%s\\22}}\"\r"
                                ,mqtt_ota_inform.request_theme_str
                                ,device_inform.version
                            );
    HAL_UART_Transmit_DMA(&huart3, (uint8_t*)mqtt_ota_inform.PubBuf, strlen(mqtt_ota_inform.PubBuf));
}

// 订阅主题
void topic_sub(uint8_t delay_way)
{
    sprintf(desired_inform.PubBuf,"AT+MSUB=\"%s\",1\r"
                                ,desired_inform.desired_reply_theme_str
                            );
    air4g_send_cmd(desired_inform.PubBuf , "OK" , 200 , delay_way);
}

//static uint32_t countid = 1213;
// 4G模块上报物模型信息
void air_4g_MPUB(uint8_t car_state)
{
    switch(car_state)
    {
        case CLOSE:
            {
                mqtt_pub_inform.vehicleStatus = 0x01;
                {
                    sprintf(mqtt_pub_inform.PubBuf,"AT+MPUB=\"%s\",0,0,\"{\\22ID\\22:%d,\\22params\\22:{},\\22method\\22:\\22thing.event.property.post\\22}\"\r"
                    ,mqtt_pub_inform.theme_str
                    ,0
                    );
                }
            }
            break;

        case OPEN:
            {
                mqtt_pub_inform.vehicleStatus = 0x02;
            }
            break;

        case RUN:
            {
                mqtt_pub_inform.vehicleStatus = mqtt_pub_inform.runvehicleStatus;   // 运行状态下上传的车辆状态为可解析数据
                mqtt_pub_inform.vehicleSpeed  = can_read_data.Speed;
            }
            break;

        case RECHARGE:
            {
                mqtt_pub_inform.vehicleStatus = 0x05;
                {
                    //发送一次最后电源连接的时间和充电剩余时间
                    mqtt_pub_inform.start_charge_flag = DISABLE;
                    time_t changeUTC = timer_info.timestamp;//不+28800，IOT目前用UTC
                    battery_data.timestamp = changeUTC;

                    //sprintf(mqtt_pub_inform.PubBuf,"AT+MPUB=\"%s\",0,0,\"{\\22ID\\22:%d,\\22params\\22:{\\22battery:lastChargeTime\\22:\\22%lld\\22,\\22battery:chargeTimeRemain\\22:%d},\\22method\\22:\\22thing.event.property.post\\22}\"\r"
                    sprintf(mqtt_pub_inform.PubBuf,"AT+MPUB=\"%s\",0,0,\"{\\22ID\\22:%d,\\22params\\22:{\\22battery:lastChargeTime\\22:\\22%lld\\22,\\22battery:chargeTimeRemain\\22:%d},\\22method\\22:\\22thing.event.property.post\\22}\"\r"
                                    ,mqtt_pub_inform.theme_str
                                    ,106
                                    ,battery_data.timestamp * 1000
                                    ,battery_data.chargeTimeRemain * 60000
                                    );
                }
            }
            break;

        case FAULT:
            {
                mqtt_pub_inform.vehicleStatus = 0x24;
						}
            break;
    }
    vTaskDelay(10);
}
/*-------------------------------------------------------------事件-------------------------------------------------------------------*/
// 4G模块上报当次运行记录数据
void air_4g_MPUB_event(uint8_t eventid)
{
    while (huart3.gState != HAL_UART_STATE_READY){}
    memset(mqtt_pub_inform.PubBuf,0,sizeof(mqtt_pub_inform.Pub_work_event_Buf));

		time_t *changeUTC = &timer_info.timestamp;//不+28800，IOT目前用UTC
		switch(eventid)
		{
				case WORK_EVENT_MAIN:
						strcpy((char *)sing_work_event.end_date, (const char *)TimeStampToString(changeUTC));// 从不断累计的时间戳转化成时间字符串记录结束时间

						strcpy((char *)sing_work_event.latitude, (const char *)gps_info.Lat_nowstr);

						strcpy((char *)sing_work_event.longitude, (const char *)gps_info.Lon_nowstr);

						sprintf((char *)sing_work_event.recordID,"%s%s"
									,sing_work_event.start_date
									,sing_work_event.end_date+9
								);

						sing_work_event.power = sing_work_event.start_power - battery_data.soc;                      //运行记录耗电量计算

            if (sing_work_event.start_drug > spraySensor_waterLevel)                                 //运行记录耗药量计算
            {
                sing_work_event.drug = (sing_work_event.start_drug - spraySensor_waterLevel)*10;
            }
            else
            {
                sing_work_event.drug = 0;
            }

						sprintf(mqtt_pub_inform.Pub_work_event_Buf,"AT+MPUB=\"%s\",0,0,\"{\\22params\\22:{\\22startTime\\22:\\22%s\\22,\\22endTime\\22:\\22%s\\22,\\22power\\22:%d,\\22drug\\22:%d,\\22mileage\\22:%d,\\22longitude\\22:\\22%s\\22,\\22latitude\\22:\\22%s\\22,\\22recordId\\22:\\22%s\\22},\\22method\\22:\\22thing.event.runRecordMain.post\\22}\"\r"
										,mqtt_pub_inform.work_event_Main
										,sing_work_event.start_date
										,sing_work_event.end_date
										,sing_work_event.power
										,sing_work_event.drug
										,sing_work_event.mileage
										,sing_work_event.longitude
										,sing_work_event.latitude
										,sing_work_event.recordID
									);
						break;

				case WORK_EVENT_TRACK:
						sprintf(mqtt_pub_inform.Pub_work_event_Buf,"AT+MPUB=\"%s\",0,0,\"{\\22params\\22:{\\22recordId\\22:\\22%s\\22,\\22trackData\\22:\\22%s\\22},\\22method\\22:\\22thing.event.runRecordTrack.post\\22}\"\r"
									,mqtt_pub_inform.work_event_Track
									,sing_work_event.recordID//以结束时间作为运行记录两条主从数据的标识
									,sing_work_event.trackData
								);
						break;

				case DEFAUT_EVENT:
						break;
		}
    vTaskDelay(10);
		HAL_UART_Transmit_DMA(&huart3, (uint8_t*)mqtt_pub_inform.Pub_work_event_Buf, strlen(mqtt_pub_inform.Pub_work_event_Buf));
}

void usart1_sbus_tx(void)
{
                                                    //{前的\"和}后的\"方便Android端处理这里去调
		sprintf(sbus_pack_data.tx_buf,"AT+MPUB=\"%s\",0,0,{\\22ID\\22:%d,\\22params\\22:{\\22senser:waterPressureSensor\\22:%d,\\22senser:flowSensor\\22:%d,\\22senser:liquidLevelSensor\\22:%d,\\22motorWaterPump\\22:%d,\\22fanMachinery\\22:%d,\\22battery:capacitySoc\\22:%d,\\22battery:realTimeCurrent\\22:%d,\\22battery:dischargeTimeRemain\\22:%d,\\22basic:longitude\\22:\\22%s\\22,\\22basic:latitude\\22:\\22%s\\22,\\22basic:vehicleStatus\\22:%d,\\22basic:vehicleSpeed\\22:%d},\\22method\\22:\\22thing.event.property.post\\22}\r"
										,mqtt_pub_inform.theme_str
										,200
										// 传感器模块
										,spraySensor_waterPressure                  // 水压 每KPa
										,spraySensor_waterFlow/1000                 // 流量 每1ml
										,spraySensor_waterLevel/100                 // 水位 每10ml

										// 默认模块
										,mqtt_pub_inform.motorWaterPump             // 水泵功率
										,mqtt_pub_inform.fanMachinery               // 风机

										// 电池模块
										,battery_data.soc                            // 电池SOC
                    ,battery_data.currentCurrent                 // 实时电流 充电为＋ 放电为-
                    ,battery_data.dischargeTimeRemain            // 放电剩余时间

										// 主电机驱动器

										// 主控模块
										,gps_info.Lon_nowstr                        // 经度
										,gps_info.Lat_nowstr                        // 纬度
										,mqtt_pub_inform.vehicleStatus              // 车辆状态
										,mqtt_pub_inform.vehicleSpeed				// 车速
						);
    vTaskDelay(10);
    HAL_UART_Transmit_DMA(&huart1, (uint8_t*)sbus_pack_data.tx_buf, strlen(sbus_pack_data.tx_buf));
}

