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

    // 设置GPS上报频率为1秒1次
    air4g_send_cmd("AT+CGNSURC=10\r", "OK", 100, delay_way);
    gps_info.gpsUrc                             = 1;

    // 打开CSQ主动上报
//    air4g_send_cmd("AT*CSQ=1\r", "OK", 100, delay_way);

    return 0;
}

// 关闭主动上报
uint8_t air_4g_closeURC(uint8_t delay_way)
{
    // 打开GPS
    // air4g_send_cmd("AT+CGNSPWR=1\r", "OK" , 100, delay_way);

    // 使能辅助定位
	// air4g_send_cmd("AT+CGNSAID=31,1,1,1\r", "OK", 100, delay_way);

    // 设置GPS上报频率为1秒0次，关闭
    air4g_send_cmd("AT+CGNSURC=0\r", "OK", 100, delay_way);
    gps_info.gpsUrc                             = 1;

//    // 关闭CSQ主动上报
//    air4g_send_cmd("AT*CSQ=0\r", "OK", 100, delay_way);


    return 0;
}


// 4G模块重启
uint8_t air_4g_restar(bool delay_way)
{

    // 复位前关闭主动上报的GPS和CSQ


    // 4G模块复位，要不要采用带接收的方式？
    char buf[11];
    sprintf(buf,"AT+RSTSET\r");
    HAL_UART_Transmit_DMA(&huart3,(uint8_t*)buf,strlen(buf));

    air_4g_init(0,1);
    return	0;
}

// 4G模块检查网络状态
uint8_t air_4g_net_check(bool delay_way)
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

    // 查询网络质量，暂时没有对结果做处理
    //uint8_t simCSQ = 0;
    //simCSQ = air4gCSQCheck(0);
    //if(simCSQ < 5)
    //    return SIM_CSQ_LOW;

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


// 蜂窝信号质量查询
uint8_t air4gCSQCheck(uint8_t delay_mode)
{
    char sendcmd[10];
    uint8_t len_scmd;
    uint8_t simCSQ = 0;
    sprintf(sendcmd,"AT+CSQ\r");

    // 发送指令
    len_scmd = strlen(sendcmd);
	HAL_UART_Transmit_DMA(&huart3, (uint8_t*)sendcmd, len_scmd);

    // 延时
	if(delay_mode)  vTaskDelay (400); else  HAL_Delay(400);

    // 判断AT指令返回结果
    if(strstr((char*)usart3_handle_4g.save_buf,"+CSQ:")!=NULL)
    {
        char *token;
        token=strtok((char*)usart3_handle_4g.save_buf,":");
        token=strtok(NULL,",");
        simCSQ = atoi(token);

        memset(usart3_handle_4g.save_buf,0,SAVE_SIZE);
        return simCSQ;
    }

    // 清空串口接收缓冲区
	memset(usart3_handle_4g.save_buf,0,SAVE_SIZE);

	return  0;
}

// 基站定位信息查询
uint8_t lbsInfoRead(uint8_t delay_mode)
{
    char sendcmd[20];
    uint8_t len_scmd;
    sprintf(sendcmd,"AT+CIPGSMLOC=1,1\r");

    // 发送指令
    len_scmd = strlen(sendcmd);
    HAL_UART_Transmit_DMA(&huart3, (uint8_t*)sendcmd, len_scmd);

    // 延时等待消息
    for(uint8_t i = 0; i <= 15; i++)
    {
        if(delay_mode)  vTaskDelay (400); else  HAL_Delay(400);

        // 判断AT指令返回结果
        if(strstr((char*)usart3_handle_4g.save_buf,"+CIPGSMLOC: 0")!=NULL)
        {
            char *token;
            token=strtok((char*)usart3_handle_4g.save_buf,",");
            token=strtok(NULL,",");
            strcpy((char *)gps_info.LBSlat_nowstr,token);
            token=strtok(NULL,",");
            strcpy((char *)gps_info.LBSlon_nowstr,token);

            memset(usart3_handle_4g.save_buf,0,SAVE_SIZE);
            return  1;
        }

        // 清空串口接收缓冲区
        memset(usart3_handle_4g.save_buf,0,SAVE_SIZE);
    }

    // 清空串口接收缓冲区
    memset(usart3_handle_4g.save_buf,0,SAVE_SIZE);
    return  0;

}

// 获取4G实时时间
void get_real_time(bool delay_way)
{
	air4g_send_cmd("AT+CCLK?\r", "OK", 100, delay_way);
}

// 获取4G当前信息
void get_4G_msg(bool delay_way)
{
	air4g_send_cmd("ATE0\r", "OK", 50, delay_way);
	air4g_send_cmd("AT+MQTTSTATU\r", "+MQTTSTATU", 50, delay_way);	//查询MQTT连接状态
	air4g_send_cmd("AT+CSQ\r", "OK", 50, delay_way);		//查询4G信号质量
	//..
}


/*************计算坐标间距离***************/
void BLTOXY(CRDCARTESIAN * pcc, CRDGEODETIC * pcg, int Datum, int zonewide)
{
    double B = pcg->latitude; //纬度
    double L = pcg->longitude; //经度//纬度度数

    double L0; //中央经线度数
    double N; //卯酉圈曲率半径
    double q2;
    double x; //高斯平面纵坐标
    double y; //高斯平面横坐标
    double s; //赤道至纬度B的经线弧长
    double f; //参考椭球体扁率
    double e1; //椭球第一偏心率
    double a; //参考椭球体长半轴
    //double b;    //参考椭球体短半轴
    double a1, a2, a3, a4;
    double b1, b2, b3, b4;
    double c0, c1, c2, c3;

    const double IPI = 0.0174532925199433333333; //3.1415926535898/180.0

    int prjno = 0; //投影带号
    // zonewide 投影带宽带， 3 或者是 6
    if (zonewide == 6)
    {
        prjno = (int) (L / zonewide) + 1;
        L0 = prjno * zonewide - 3;
    }
    else
    {
        prjno = (int) ((L - 1.5) / 3) + 1;
        L0 = prjno * 3;
    }

    /*
     * 北京 54
     * 长半轴a=6378245m
     * 短半轴b=6356863.0188m
     * 扁率α=1/298.3
     * 第一偏心率平方 =0.006693421622966
     * 第二偏心率平方 =0.006738525414683
     *
     * 西安80
     * 长半轴a=6378140±5（m）
     * 短半轴b=6356755.2882m
     * 扁率α=1/298.257
     * 第一偏心率平方 =0.00669438499959
     * 第二偏心率平方=0.00673950181947
     *
     * WGS84
     * 长半轴a=6378137± 2（m）
     * 短半轴b=6356752.3142m
     * 扁率α=1/298.257223563
     * 第一偏心率平方 =0.00669437999013
     * 第二偏心率平方 =0.00673949674223
     *
     */

    //Datum 投影基准面类型：北京54基准面为54，西安80基准面为80，WGS84基准面为84
    if (Datum == 84)
    {
        a = 6378137;
        f = 1 / 298.257223563;
    }
    else if (Datum == 54)
    {
        a = 6378245;
        f = 1 / 298.3;
    }
    else if (Datum == 80)
    {
        a = 6378140;
        f = 1 / 298.257;
    }

    e1 = 2 * f - f*f; //(a*a-b*b)/(a*a) 椭球第一偏心率

    L0 = L0*IPI; // 转为弧度
    L = L*IPI; // 转为弧度
    B = B*IPI; // 转为弧度

    double sinB = sin(B); //sinB
    double cosB = cos(B); //cosB
    double tanB = tan(B); //tanB

    double l = L - L0; //L-L0l
    double m = l * cosB; //ltanB

    N = a / sqrt(1 - e1 * pow(sinB, 2));
    q2 = e1 / (1 - e1) * pow(cosB, 2);

    a1 = 1 + 3.0 / 4.0 * e1 + 45.0 / 64.0 * pow(e1, 2) + 175.0 / 256.0 * pow(e1, 3)
            + 11025.0 / 16384.0 * pow(e1, 4) + 43659.0 / 65536.0 * pow(e1, 5);

    a2 = 3.0 / 4.0 * e1 + 15.0 / 16.0 * pow(e1, 2) + 525.0 / 512.0 * pow(e1, 3)
            + 2205.0 / 2048.0 * pow(e1, 4) + 72765.0 / 65536.0 * pow(e1, 5);

    a3 = 15.0 / 64.0 * pow(e1, 2) + 105.0 / 256.0 * pow(e1, 3) + 2205.0 / 4096.0
            * pow(e1, 4) + 10359.0 / 16384.0 * pow(e1, 5);

    a4 = 35.0 / 512.0 * pow(e1, 3) + 315.0 / 2048.0 * pow(e1, 4) + 31185.0 / 13072.0
            * pow(e1, 5);
    b1 = a1 * a * (1 - e1);
    b2 = -1.0 / 2.0 * a2 * a * (1 - e1);
    b3 = 1.0 / 4.0 * a3 * a * (1 - e1);
    b4 = -1.0 / 6.0 * a4 * a * (1 - e1);
    c0 = b1;
    c1 = 2 * b2 + 4 * b3 + 6 * b4;
    c2 = -(8 * b3 + 32 * b4);
    c3 = 32 * b4;
    s = c0 * B + cosB * (c1 * sinB + c2 * pow(sinB, 3) + c3 * pow(sinB, 5));

    x = s + 0.5 * N * tanB * pow(m, 2) + 1.0 / 24.0 * (5 - pow(tanB, 2) + 9 * q2 + 4
            * pow(q2, 2)) * N * tanB * pow(m, 4) + 1.0 / 720.0 * (61 - 58 * pow(tanB, 6))
            * N * tanB * pow(m, 6);

    y = N * m + 1.0 / 6.0 * (1 - pow(tanB, 2) + q2) * N * pow(m, 3) + 1.0 / 120.0
            * (5 - 18 * tanB * tanB + pow(tanB, 4) - 14 * q2 - 58 * q2 * pow(tanB, 2)) * N * pow(m, 5);

    y = y + 1000000 * prjno + 500000;
    pcc->x = x;
    pcc->y = y - 38000000;
    pcc->z = 0;
}


// 返回值单位是米
double BLDistance(CRDGEODETIC *pcg1, CRDGEODETIC *pcg2, int Datum, int zonewide)
{
    CRDCARTESIAN pcc1, pcc2;
    BLTOXY(&pcc1, pcg1, Datum, zonewide);
    BLTOXY(&pcc2, pcg2, Datum, zonewide);

    double xdes = fabs(pcc1.x - pcc2.x);
    double ydes = fabs(pcc1.y - pcc2.y);
    double des = sqrt(xdes * xdes + ydes * ydes);

    return des;
}

// 计算两个经纬度坐标之间的距离，返回值单位为米
uint16_t BLDistanceDiff(uint8_t *Lon_nowstr1, uint8_t *Lat_nowstr1, uint8_t *Lon_nowstr2, uint8_t *Lat_nowstr2)
{
    CRDGEODETIC pcg1 = {atof((char *)Lon_nowstr1), atof((char *)Lat_nowstr1), 0};
    CRDGEODETIC pcg2 = {atof((char *)Lon_nowstr2), atof((char *)Lat_nowstr2), 0};
    return (uint16_t)BLDistance(&pcg1, &pcg2, WGS84, ZONEWIDE6);
}

// 打开蓝牙
void ble_swit_on(bool delay_way)
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



//    else if(
//    //if( //(strstr(usart3_handle_4g.rx_buf,"+UGNSINF: 1")!=NULL) ||        // GPS定位信息
//        //(strstr(usart3_handle_4g.rx_buf,"+CIPGSMLOC: 0")!=NULL) ||      // 基站定位信息
//        //(strstr(usart3_handle_4g.rx_buf,"+WIFILOC: 0")!=NULL) ||        // WIFI定位信息
//        (strstr(usart3_handle_4g.rx_buf,"+CSQ:")!=NULL)  ||
//        (strstr(usart3_handle_4g.rx_buf,"303124")!=NULL) ||
//        (strstr(usart3_handle_4g.rx_buf,"303224")!=NULL) ||
//        (strstr(usart3_handle_4g.rx_buf,"303324")!=NULL) ||
//        (strstr(usart3_handle_4g.rx_buf,"upgrade")!=NULL)||
//        (strstr(usart3_handle_4g.rx_buf,"+HTTPACTION: 0,200")!=NULL)
//        )
//    {
//        //if(control_flag.Init_flag == FALSE)
//        //{
//            memcpy(usart3_handle_4g.report_buf,usart3_handle_4g.rx_buf,usart3_handle_4g.rx_len);
//            osSemaphoreRelease(BinarySem01Handle);
//        //}
//    }
//    if(strstr((char*)usart3_handle_4g.rx_buf,"+UGNSINF: 1")!=NULL )		// GPS定位信息
//    {
//        memcpy(gps_info.GPSRx,usart3_handle_4g.rx_buf,usart3_handle_4g.rx_len);
//        gps_info.GPSRxFlag = TRUE;
//    }
    //else if( strstr(usart3_handle_4g.rx_buf,"+CIPGSMLOC: 0")!=NULL)     // 基站定位信息
    //{
    //    memcpy(gps_info.LBSRx,usart3_handle_4g.rx_buf,usart3_handle_4g.rx_len);
    //    gps_info.LBSRxFlag = TRUE;
    //}
    if((strstr((char*)usart3_handle_4g.rx_buf,"+UGNSINF: 1")!=NULL)||
		(strstr(usart3_handle_4g.rx_buf,"+CSQ:")!=NULL)||
		(strstr(usart3_handle_4g.rx_buf,"+BLEIND=DATA")!=NULL)||
		(strstr(usart3_handle_4g.rx_buf,"+MQTTSTATU")!=NULL)||
		(strstr(usart3_handle_4g.rx_buf,"+CCLK:")!=NULL)||
        (strstr(usart3_handle_4g.rx_buf,"ERROR")!=NULL)
        //(strstr(usart3_handle_4g.rx_buf,"303224")!=NULL) ||
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

//static uint32_t countid = 1213;
// 4G模块上报物模型信息
void air_4g_MPUB(uint8_t car_state)
{
    switch(car_state)
    {
        case CLOSE:
            {
                mqtt_pub_inform.vehicleStatus = 0x01;
                if(battery_data.charge_overflag == DISABLE && mqtt_pub_inform.start_summary_flag == DISABLE)
                {
                    sprintf(mqtt_pub_inform.PubBuf,"AT+MPUB=\"%s\",0,0,\"{\\22ID\\22:%d,\\22params\\22:{\\22senser:temperatureHumiditySensor1Temp\\22:%d,\\22senser:temperatureHumiditySensor1Humi\\22:%d,\\22senser:temperatureHumiditySensor2Temp\\22:%d,\\22senser:temperatureHumiditySensor2Humi\\22:%d,\\22senser:waterPressureSensor\\22:%d,\\22motorWaterPump\\22:%d,\\22senser:liquidLevelSensor\\22:%d,\\22driverMpuTemp\\22:%d,\\22driverBoxTemp1\\22:%d,\\22driverBoxTemp2\\22:%d,\\22driverMcuTemp\\22:%d,\\22battery:realTimeCurrent\\22:%d,\\22battery:batteryWarningStatus\\22:%d,\\22battery:capacitySoc\\22:%d,\\22battery:batteryVoltageTotal\\22:%d,\\22battery:electricCoreVoltageMax\\22:%d,\\22battery:electricCoreVoltageMin\\22:%d,\\22battery:batteryEnvironmentTemp\\22:%d,\\22battery:batteryTemp1\\22:%d,\\22battery:batteryTemp2\\22:%d,\\22battery:batteryTemp3\\22:%d,\\22battery:batteryTemp4\\22:%d,\\22basic:longitude\\22:\\22%s\\22,\\22basic:latitude\\22:\\22%s\\22,\\22basic:vehicleStatus\\22:%d,\\22debug:openMainBoxCheck\\22:%d,\\22debug:openDriverBoxCheck\\22:%d,\\22debug:gpsSignal\\22:%d,\\22debug:cellularSignalQuality\\22:%d,},\\22method\\22:\\22thing.event.property.post\\22}\"\r"
                                ,mqtt_pub_inform.theme_str
                                ,100

                                // 传感器模块
                                ,SHT30Environment_temperature / 10				// SHT30环境温度， / 10满足APP暂时显示正常
                                ,SHT30Environment_humidity						// SHT30环境湿度
                                ,SHT30CarBox_temperature      					// SHT30箱体温度
                                ,SHT30CarBox_humidity							// SHT30箱体湿度

                                ,spraySensor_waterPressure                      // 压力
                                ,spraySensor_waterFlow / 1000                   // 流量
                                ,spraySensor_waterLevel / 100                   // 液位

                                // 默认模块
                                ,driverBoard_mpu6050Temp						// 电驱板MPU6050温度
                                ,driverBoard_PT100Temp1							// 驱动盒温度1
                                ,driverBoard_PT100Temp2							// 驱动盒温度2
                                ,driverBoard_mcuTempRise						// 电驱板MCU温度

                                // 电池模块
                                ,battery_data.currentCurrent					// 实时电流
                                ,battery_data.warningState						// 告警状态
                                ,battery_data.soc											// 电池SOC
                                ,battery_data.voitageCurrent					// 电池总电压
                                ,battery_data.cellVoitageMax					// 电池最高电芯电压
                                ,battery_data.cellVoitageMin					// 电池最低电芯电压
                                ,battery_data.batt_temp[0]						// 电池环境温度
                                ,battery_data.batt_temp[0]						// 电池温度1
                                ,battery_data.batt_temp[1]						// 电池温度2
                                ,battery_data.batt_temp[2]						// 电池温度3
                                ,battery_data.batt_temp[3]						// 电池温度4
                                //,battery_data.capDesign							// 电池设计容量
                                //,battery_data.capOverall						// 电池满容量

                                // 主电机驱动器

                                // 主控模块
                                ,gps_info.Lon_nowstr                            // 经度
                                ,gps_info.Lat_nowstr                            // 纬度
                                ,mqtt_pub_inform.vehicleStatus                  // 车辆状态01；关机

                                // 汇总数据

                                // 调试模块
                                ,control_flag.main_kaihe_flag                   // 主控开盒标志位
                                ,driverBoard_uncoverFlag     				    // 电驱板开盒标志位
                                ,gps_info.gps_signal_flag				    // GPS信号标志位，上报值第二位
                                ,air_4g_connect.sim_CSQ							// SIM卡蜂窝信号质量

                            );
                }
                else if(battery_data.charge_overflag == ENABLE)		// 充电结束上报最后一次充电时间？
                {
                    battery_data.charge_overflag = DISABLE;
                    sprintf(mqtt_pub_inform.PubBuf,"AT+MPUB=\"%s\",0,0,\"{\\22params\\22:{\\22battery:chargeTimeRemain\\22:%d},\\22method\\22:\\22thing.event.property.post\\22}\"\r"
                    ,mqtt_pub_inform.theme_str
                    ,0);
                }
                else if(mqtt_pub_inform.start_summary_flag == ENABLE)		// 关机上报汇总数据
                {
                    mqtt_pub_inform.start_summary_flag = DISABLE;			// 软开关开机时上报一次汇总数据
                    if(battery_data.circle > 300)
                    {
                        battery_data.soh =90;
                    }
                    sprintf(mqtt_pub_inform.PubBuf,"AT+MPUB=\"%s\",0,0,\"{\\22ID\\22:%d,\\22params\\22:{\\22summary:totalRunTime\\22:%d,\\22summary:totalFanMachineryTime\\22:%d,\\22summary:totalWaterPumpTime\\22:%d,\\22summary:totalMileage\\22:%d,\\22battery:batteryLoop\\22:%d,\\22battery:batteryCapacityPlan\\22:%1f,\\22battery:batteryCapacityFull\\22:%1f,\\22battery:hardwareVersion\\22:\\22%s\\22,\\22battery:healthStatus\\22:%d,\\22battery:batteryType\\22:\\22%s\\22,\\22battery:softwareVersion\\22:\\22%s\\22,\\22battery:batterySN\\22:\\22%s\\22,\\22debug:appRestartReason\\22:%d,},\\22method\\22:\\22thing.event.property.post\\22}\"\r"
                    ,mqtt_pub_inform.theme_str
                    ,102

                    // 汇总数据
                    ,summary_data.totalRunTime*1000                     // 车辆总运行时间
                    ,summary_data.totalFanMachineryTime*1000            // 车辆风机总运行时间
                    ,summary_data.totalWaterPumpTime*1000               // 车辆水泵总运行时间
                    ,summary_data.totalMileage						    // 车辆总里程

                    // 电池模块
                    ,battery_data.circle								// 电池循环次数
                    ,(double)battery_data.capDesign						// 电池设计容量
                    ,(double)battery_data.capOverall					// 电池满容量
                    ,battery_data.battHarewareVersion					// 电池硬件版本号
                    ,battery_data.soh									// 电池健康状况SOH
                    ,battery_data.battTypeName							// 电池型号名称
                    ,battery_data.battSoftwareVersion					// 电池软件版本号
                    ,battery_data.battSN								// 电池SN码

                    // APP上次复位原因
                    ,control_flag.resetSource
                    );
                }
                else
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
                sprintf(mqtt_pub_inform.PubBuf,"AT+MPUB=\"%s\",0,0,\"{\\22ID\\22:%d,\\22params\\22:{\\22senser:temperatureHumiditySensor1Temp\\22:%d,\\22senser:temperatureHumiditySensor1Humi\\22:%d,\\22senser:temperatureHumiditySensor2Temp\\22:%d,\\22senser:temperatureHumiditySensor2Humi\\22:%d,\\22senser:waterPressureSensor\\22:%d,\\22motorWaterPump\\22:%d,\\22senser:liquidLevelSensor\\22:%d,\\22battery:realTimeCurrent\\22:%d,\\22battery:batteryWarningStatus\\22:%d,\\22battery:capacitySoc\\22:%d,\\22battery:batteryVoltageTotal\\22:%d,\\22battery:electricCoreVoltageMax\\22:%d,\\22battery:electricCoreVoltageMin\\22:%d,\\22battery:batteryEnvironmentTemp\\22:%d,\\22battery:batteryTemp1\\22:%d,\\22battery:batteryTemp2\\22:%d,\\22battery:batteryTemp3\\22:%d,\\22battery:batteryTemp4\\22:%d,\\22basic:longitude\\22:\\22%s\\22,\\22basic:latitude\\22:\\22%s\\22,\\22basic:vehicleStatus\\22:%d,\\22debug:openMainBoxCheck\\22:%d,\\22debug:openDriverBoxCheck\\22:%d,\\22debug:gpsSignal\\22:%d,\\22debug:cellularSignalQuality\\22:%d,},\\22method\\22:\\22thing.event.property.post\\22}\"\r"
                                ,mqtt_pub_inform.theme_str					        // TOPIC
                                ,101												// ID

                                // 传感器模块
                                ,SHT30Environment_temperature / 10				    // SHT30环境温度， / 10满足APP暂时显示正常
                                ,SHT30Environment_humidity							// SHT30环境湿度
                                ,SHT30CarBox_temperature      						// SHT30箱体温度
                                ,SHT30CarBox_humidity								// SHT30箱体湿度
                                ,spraySensor_waterPressure                          // 压力
                                ,spraySensor_waterFlow / 1000                       // 流量
                                ,spraySensor_waterLevel / 100                       // 液位

                                // 默认模块

                                // 电池模块
                                ,battery_data.currentCurrent						// 实时电流
                                ,battery_data.warningState							// 告警状态
                                ,battery_data.soc												// 电池SOC
                                ,battery_data.voitageCurrent						// 电池总电压
                                ,battery_data.cellVoitageMax						// 电池最高电芯电压
                                ,battery_data.cellVoitageMin						// 电池最低电芯电压
                                ,battery_data.batt_temp[0]							// 电池环境温度
                                ,battery_data.batt_temp[0]							// 电池温度1
                                ,battery_data.batt_temp[1]							// 电池温度2
                                ,battery_data.batt_temp[2]							// 电池温度3
                                ,battery_data.batt_temp[3]							// 电池温度4

                                // 主电机驱动器

                                // 主控模块
                                ,gps_info.Lon_nowstr                			    // 经度
                                ,gps_info.Lat_nowstr                			    // 纬度
                                ,mqtt_pub_inform.vehicleStatus      			    // 车辆状态

                                // 汇总数据

                                // 调试模块
                                ,control_flag.main_kaihe_flag       			    // 主控开盒标志位
                                ,driverBoard_uncoverFlag     						// 电驱板开盒标志位
                                ,gps_info.gps_signal_flag						// GPS信号标志位，上报值第二位
                                ,air_4g_connect.sim_CSQ								// SIM卡蜂窝信号质量
                            );
                }
                break;

        case RUN:
            {
                if(mqtt_pub_inform.rundata_turn_count<5)		// 发送实时地图页面数据
                {
                    mqtt_pub_inform.rundata_turn_count++;
                    mqtt_pub_inform.vehicleStatus = mqtt_pub_inform.runvehicleStatus;   // 运行状态下上传的车辆状态为可解析数据
                    mqtt_pub_inform.vehicleSpeed  = can_read_data.Speed;
                    sprintf(mqtt_pub_inform.PubBuf,"AT+MPUB=\"%s\",0,0,\"{\\22ID\\22:%d,\\22params\\22:{\\22senser:waterPressureSensor\\22:%d,\\22motorWaterPump\\22:%d,\\22senser:liquidLevelSensor\\22:%d,\\22fanMachinery\\22:%d,\\22anglePitch\\22:%d,\\22angleRoll\\22:%d,\\22battery:realTimeCurrent\\22:%d,\\22battery:capacitySoc\\22:%d,\\22basic:longitude\\22:\\22%s\\22,\\22basic:latitude\\22:\\22%s\\22,\\22basic:vehicleStatus\\22:%d,\\22basic:vehicleSpeed\\22:%d,},\\22method\\22:\\22thing.event.property.post\\22}\"\r"
                                    ,mqtt_pub_inform.theme_str
                                    ,103
                                    // 传感器模块
                                    ,spraySensor_waterPressure                    // 流量和液位
                                    ,spraySensor_waterFlow / 1000                 // 流量
                                    ,spraySensor_waterLevel / 100

                                    // 默认模块
                                    ,mqtt_pub_inform.fanMachinery				// 风机
                                    ,driverBoard_carPitchAngle					// 倾角
                                    ,driverBoard_carRollAngle					// 翻滚角

                                    // 电池模块
                                    ,battery_data.currentCurrent				// 实时电流
                                    ,battery_data.soc							// 电池SOC

                                    // 主电机驱动器

                                    // 主控模块
                                    ,gps_info.Lon_nowstr                        // 经度
                                    ,gps_info.Lat_nowstr                        // 纬度
                                    ,mqtt_pub_inform.vehicleStatus              // 车辆状态
                                    ,mqtt_pub_inform.vehicleSpeed				// 车速

                                    // 汇总数据

                                    // 调试模块

                                );
                }
                else
                {
                    mqtt_pub_inform.rundata_turn_count=0;
                    sprintf(mqtt_pub_inform.PubBuf,"AT+MPUB=\"%s\",0,0,\"{\\22ID\\22:%d,\\22params\\22:{\\22senser:temperatureHumiditySensor1Temp\\22:%d,\\22senser:temperatureHumiditySensor1Humi\\22:%d,\\22senser:temperatureHumiditySensor2Temp\\22:%d,\\22senser:temperatureHumiditySensor2Humi\\22:%d,\\22driverMpuTemp\\22:%d,\\22driverBoxTemp1\\22:%d,\\22driverBoxTemp2\\22:%d,\\22driverMcuTemp\\22:%d,\\22battery:batteryWarningStatus\\22:%d,\\22battery:batteryVoltageTotal\\22:%d,\\22battery:electricCoreVoltageMax\\22:%d,\\22battery:electricCoreVoltageMin\\22:%d,\\22battery:batteryEnvironmentTemp\\22:%d,\\22battery:batteryTemp1\\22:%d,\\22battery:batteryTemp2\\22:%d,\\22battery:batteryTemp3\\22:%d,\\22battery:batteryTemp4\\22:%d,\\22motor:state\\22:%d,\\22motor:temp1\\22:%d,\\22motor:temp2\\22:%d,\\22motor:temp3\\22:%d,\\22motor:temp4\\22:%d,\\22debug:gpsSignal\\22:%d,\\22debug:cellularSignalQuality\\22:%d,},\\22method\\22:\\22thing.event.property.post\\22}\"\r"
                                    ,mqtt_pub_inform.theme_str
                                    ,104

                                    // 传感器模块
                                    ,SHT30Environment_temperature / 10				// SHT30环境温度， / 10满足APP暂时显示正常
                                    ,SHT30Environment_humidity						// SHT30环境湿度
                                    ,SHT30CarBox_temperature      					// SHT30箱体温度
                                    ,SHT30CarBox_humidity							// SHT30箱体湿度

                                    // 默认模块
                                    ,driverBoard_mpu6050Temp						// 电驱板MPU6050温度
                                    ,driverBoard_PT100Temp1							// 驱动盒温度1
                                    ,driverBoard_PT100Temp2							// 驱动盒温度2
                                    ,driverBoard_mcuTempRise						// 电驱板MCU温度

                                    // 电池模块
                                    ,battery_data.warningState						// 告警状态
                                    ,battery_data.voitageCurrent					// 电池总电压
                                    ,battery_data.cellVoitageMax					// 电池最高电芯电压
                                    ,battery_data.cellVoitageMin					// 电池最低电芯电压
                                    ,battery_data.batt_temp[0]						// 电池环境温度
                                    ,battery_data.batt_temp[0]						// 电池温度1
                                    ,battery_data.batt_temp[1]						// 电池温度2
                                    ,battery_data.batt_temp[2]						// 电池温度3
                                    ,battery_data.batt_temp[3]						// 电池温度4

                                    // 主电机驱动器
                                    ,can_read_data.moto_state						// 电机驱动器异常状态字
                                    ,can_read_data.moto_temp1						// 电机驱动器温度1
                                    ,can_read_data.moto_temp2						// 电机驱动器温度2
                                    ,can_read_data.moto_temp3						// 电机驱动器温度3
                                    ,can_read_data.moto_temp4						// 电机驱动器温度4

                                    // 调试模块
                                    ,gps_info.gps_signal_flag				        // GPS信号标志位，上报值第二位
                                    ,air_4g_connect.sim_CSQ							// SIM卡蜂窝信号质量
                                );
                }
            }
            break;

        case RECHARGE:
            {
                mqtt_pub_inform.vehicleStatus = 0x05;
                if(mqtt_pub_inform.start_charge_flag == DISABLE)
                {
                    sprintf(mqtt_pub_inform.PubBuf,"AT+MPUB=\"%s\",0,0,\"{\\22ID\\22:%d,\\22params\\22:{\\22senser:temperatureHumiditySensor1Temp\\22:%d,\\22senser:temperatureHumiditySensor1Humi\\22:%d,\\22senser:temperatureHumiditySensor2Temp\\22:%d,\\22senser:temperatureHumiditySensor2Humi\\22:%d,\\22senser:waterPressureSensor\\22:%d,\\22motorWaterPump\\22:%d,\\22senser:liquidLevelSensor\\22:%d,\\22battery:realTimeCurrent\\22:%d,\\22battery:batteryProtectStatus\\22:%d,\\22battery:batteryWarningStatus\\22:%d,\\22battery:capacitySoc\\22:%d,\\22battery:chargeTimeRemain\\22:%d,\\22battery:batteryVoltageTotal\\22:%d,\\22battery:electricCoreVoltageMax\\22:%d,\\22battery:electricCoreVoltageMin\\22:%d,\\22battery:batteryEnvironmentTemp\\22:%d,\\22battery:batteryTemp1\\22:%d,\\22battery:batteryTemp2\\22:%d,\\22battery:batteryTemp3\\22:%d,\\22battery:batteryTemp4\\22:%d,\\22battery:spentChargeTime\\22:%d,\\22basic:longitude\\22:\\22%s\\22,\\22basic:latitude\\22:\\22%s\\22,\\22basic:vehicleStatus\\22:%d,\\22debug:openMainBoxCheck\\22:%d,\\22debug:openDriverBoxCheck\\22:%d,\\22debug:gpsSignal\\22:%d,\\22debug:cellularSignalQuality\\22:%d,},\\22method\\22:\\22thing.event.property.post\\22}\"\r"
                                    ,mqtt_pub_inform.theme_str
                                    ,105

                                    // 传感器模块
                                    ,SHT30Environment_temperature / 10				// SHT30环境温度， / 10满足APP暂时显示正常
                                    ,SHT30Environment_humidity						// SHT30环境湿度
                                    ,SHT30CarBox_temperature      					// SHT30箱体温度
                                    ,SHT30CarBox_humidity							// SHT30箱体湿度

                                    ,spraySensor_waterPressure                      // 压力
                                    ,spraySensor_waterFlow / 1000                   // 流量
                                    ,spraySensor_waterLevel / 100                   // 液位

                                    // 默认模块

                                    // 电池模块
                                    ,battery_data.currentCurrent					// 实时电流
                                    ,battery_data.protect_state						// 电池保护状态
                                    ,battery_data.warningState						// 告警状态
                                    ,battery_data.soc								// 电池SOC
                                    ,battery_data.chargeTimeRemain*60000			// 充电剩余时间
                                    ,battery_data.voitageCurrent					// 电池总电压
                                    ,battery_data.cellVoitageMax					// 电池最高电芯电压
                                    ,battery_data.cellVoitageMin					// 电池最低电芯电压
                                    ,battery_data.batt_temp[0]						// 电池环境温度
                                    ,battery_data.batt_temp[0]						// 电池温度1
                                    ,battery_data.batt_temp[1]						// 电池温度2
                                    ,battery_data.batt_temp[2]						// 电池温度3
                                    ,battery_data.batt_temp[3]						// 电池温度4
                                    ,battery_data.spentChargeTime*10000			    // 已充电时间
                                    //,battery_data.spentChargeTime*60000			// 已充电时间

                                    // 主电机驱动器

                                    // 主控模块
                                    ,gps_info.Lon_nowstr                			// 经度
                                    ,gps_info.Lat_nowstr                			// 纬度
                                    ,mqtt_pub_inform.vehicleStatus      			// 车辆状态

                                    // 汇总数据

                                    // 调试模块
                                    ,control_flag.main_kaihe_flag       			// 主控开盒标志位
                                    ,driverBoard_uncoverFlag     					// 电驱板开盒标志位
                                    ,gps_info.gps_signal_flag					// GPS信号标志位，上报值第二位
                                    ,air_4g_connect.sim_CSQ							// SIM卡蜂窝信号质量
                                );
                    battery_data.spentChargeTime++;
                }
                else
                {
                    //发送一次最后电源连接的时间和充电剩余时间
                    mqtt_pub_inform.start_charge_flag = DISABLE;
                    strcpy((char *)battery_data.lastChargeTime, (const char *)timer_info.Utc_nowstr);
                    battery_data.timestamp = (uint64_t)StringToTimeStamp(battery_data.lastChargeTime);

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
								sprintf(mqtt_pub_inform.PubBuf,"AT+MPUB=\"%s\",0,0,\"{\\22ID\\22:%d,\\22params\\22:{\\22senser:temperatureHumiditySensor1Temp\\22:%d,\\22senser:temperatureHumiditySensor1Humi\\22:%d,\\22senser:temperatureHumiditySensor2Temp\\22:%d,\\22senser:temperatureHumiditySensor2Humi\\22:%d,\\22senser:waterPressureSensor\\22:%d,\\22motorWaterPump\\22:%d,\\22senser:liquidLevelSensor\\22:%d,\\22driverMpuTemp\\22:%d,\\22emergencyStopStatus\\22:%d,\\22driverBoxTemp1\\22:%d,\\22driverBoxTemp2\\22:%d,\\22driverMcuTemp\\22:%d,\\22battery:realTimeCurrent\\22:%d,\\22battery:batteryProtectStatus\\22:%d,\\22battery:batteryWarningStatus\\22:%d,\\22battery:capacitySoc\\22:%d,\\22battery:batteryVoltageTotal\\22:%d,\\22battery:electricCoreVoltageMax\\22:%d,\\22battery:electricCoreVoltageMin\\22:%d,\\22battery:batteryEnvironmentTemp\\22:%d,\\22battery:batteryTemp1\\22:%d,\\22battery:batteryTemp2\\22:%d,\\22battery:batteryTemp3\\22:%d,\\22battery:batteryTemp4\\22:%d,\\22motor:state\\22:%d,\\22basic:longitude\\22:\\22%s\\22,\\22basic:latitude\\22:\\22%s\\22,\\22basic:vehicleStatus\\22:%d,\\22debug:openMainBoxCheck\\22:%d,\\22debug:openDriverBoxCheck\\22:%d,\\22debug:gpsSignal\\22:%d,\\22debug:cellularSignalQuality\\22:%d,},\\22method\\22:\\22thing.event.property.post\\22}\"\r"
																		,mqtt_pub_inform.theme_str
																		,107

																		// 传感器模块
																		,SHT30Environment_temperature / 10				// SHT30环境温度， / 10满足APP暂时显示正常
																		,SHT30Environment_humidity						// SHT30环境湿度
																		,SHT30CarBox_temperature      					// SHT30箱体温度
																		,SHT30CarBox_humidity							// SHT30箱体湿度

																		,spraySensor_waterPressure                      // 压力
																		,spraySensor_waterFlow / 1000                   // 流量
																		,spraySensor_waterLevel / 100                   // 液位

																		// 默认模块
																		,driverBoard_mpu6050Temp						// 电驱板MPU6050温度
																		,driverBoard_scramStop							// 急停开关状态
																		,driverBoard_PT100Temp1							// 驱动盒温度1
																		,driverBoard_PT100Temp2							// 驱动盒温度2
																		,driverBoard_mcuTempRise						// 电驱板MCU温度

																		// 电池模块
																		,battery_data.currentCurrent					// 实时电流
																		,battery_data.protect_state						// 电池保护状态
																		,battery_data.warningState						// 告警状态
																		,battery_data.soc											// 电池SOC
																		,battery_data.voitageCurrent					// 电池总电压
																		,battery_data.cellVoitageMax					// 电池最高电芯电压
																		,battery_data.cellVoitageMin					// 电池最低电芯电压
																		,battery_data.batt_temp[0]						// 电池环境温度
																		,battery_data.batt_temp[0]						// 电池温度1
																		,battery_data.batt_temp[1]						// 电池温度2
																		,battery_data.batt_temp[2]						// 电池温度3
																		,battery_data.batt_temp[3]						// 电池温度4

																		// 主电机驱动器
																		,can_read_data.moto_state						// 电机驱动器异常状态字

																		// 主控模块
																		,gps_info.Lon_nowstr                			// 经度
																		,gps_info.Lat_nowstr                			// 纬度
																		,mqtt_pub_inform.vehicleStatus      			// 车辆状态01；关机

																		// 调试模块
																		,control_flag.main_kaihe_flag       			// 主控开盒标志位
																		,driverBoard_uncoverFlag     					// 电驱板开盒标志位
																		,gps_info.gps_signal_flag					// GPS信号标志位，上报值第二位
																		,air_4g_connect.sim_CSQ							// SIM卡蜂窝信号质量
																		);
						}
            break;
    }
	char lenbuf[16];
	sprintf(lenbuf,"mqttlen:%d",strlen(mqtt_pub_inform.PubBuf));
    HAL_UART_Transmit_DMA(&huart3,(uint8_t*)lenbuf,strlen(lenbuf));
    vTaskDelay(10);

    HAL_UART_Transmit_DMA(&huart3, (uint8_t*)mqtt_pub_inform.PubBuf, strlen(mqtt_pub_inform.PubBuf));

	// for(int i=0;i<strlen(mqtt_pub_inform.PubBuf);i++)//循环发送数据
	// 	{
	// 		while((USART3->SR&0X40)==0){;}//循环发送,直到发送完毕
	// 		USART3->DR =mqtt_pub_inform.PubBuf[i];
	// 	}
	vTaskDelay(100);
	memset(mqtt_pub_inform.PubBuf,0,sizeof(mqtt_pub_inform.PubBuf));
}
/*-------------------------------------------------------------事件-------------------------------------------------------------------*/
// 4G模块上报当次运行记录数据
void air_4g_MPUB_event(uint8_t eventid)
{
		time_t changeUTC = timer_info.timestamp;//不+28800，IOT目前用UTC
		switch(eventid)
		{
				case WORK_EVENT_MAIN:
						memcpy(timer_info.Utc_nowstr, TimeStampToString(&changeUTC),14);// 从不断累计得时间戳转化成时间字符串记录结束时间

						strcpy((char *)sing_work_event.end_date, (const char *)timer_info.Utc_nowstr);

						strcpy((char *)sing_work_event.latitude, (const char *)gps_info.Lat_nowstr);

						strcpy((char *)sing_work_event.longitude, (const char *)gps_info.Lon_nowstr);

						sprintf((char *)sing_work_event.recordID,"%s%s"
									,sing_work_event.start_date
									,sing_work_event.end_date+9
								);

						sing_work_event.power = sing_work_event.start_power - battery_data.soc;

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
		vTaskDelay(100);
		memset(mqtt_pub_inform.PubBuf,0,sizeof(mqtt_pub_inform.Pub_work_event_Buf));
}


/************************************未使用的函数****************************************/


//YY_HANDLE 			yy_module;

//// 打开GPS
//void gps_swit_on(bool delay_way)
//{
//	air4g_send_cmd("ATE0\r" , "OK" , 20,delay_way);
//	air4g_send_cmd("ATE0\r" , "OK" , 20,delay_way);
//
//	air4g_send_cmd("AT+CGNSPWR=1\r" , "OK" , 100 , delay_way);			 		//打开GPS
//	air4g_send_cmd("AT+CGNSAID=31,1,1,1\r" , "OK" , 1000,delay_way);	 		//使能辅助定位
//	switch(control_flag.Car_State)
//		{
//			case CLOSE:
//				air4g_send_cmd("AT+CGNSURC=10\r" , "OK" , 100,delay_way);       //设置处理后的GPS信息周期上报0.1hz
//			break;
//			case OPEN:
//				air4g_send_cmd("AT+CGNSURC=5\r" , "OK" , 100,delay_way);        //设置处理后的GPS信息周期上报0.2hz
//			break;
//			case RUN:
//				air4g_send_cmd("AT+CGNSURC=1\r" , "OK" , 100,delay_way);        //设置处理后的GPS信息周期上报  1hz
//			break;
//			default:
//				air4g_send_cmd("AT+CGNSURC=10\r" , "OK" , 100,delay_way);       //设置处理后的GPS信息周期上报0.1hz
//		}
//}

//// 关闭GPS
//void gps_swit_off(bool delay_way)
//{
//	air4g_send_cmd("ATE0\r" , "OK" , 20,delay_way);
//	air4g_send_cmd("ATE0\r" , "OK" , 20,delay_way);
//	air4g_send_cmd("AT+CGNSURC=0\r" , "OK" , 100 , delay_way);//关闭处理后的GPS信息周期上报
//	air4g_send_cmd("AT+CGNSPWR=0\r" , "OK" , 100,delay_way);//关闭GPS
//}

//// 4G模块进行语音播报，暂未使用
//void SPEAK(char* conten)
//{
//	sprintf(yy_module.buf_spk,"AT+CTTS=2,\"%s\"\r",conten);
//	HAL_UART_Transmit_DMA(&huart3,(uint8_t*)yy_module.buf_spk,strlen(yy_module.buf_spk));
////	memset(yy_module.buf_spk,0,30);
//}

