#include "air820.h"
#include "handle.h"
#include "math.h"
#include "stdlib.h"
#include "CANopen_Master_M200.h"



extern osSemaphoreId BinarySem01Handle;
OTA_INFORM                  ota_inform;
AIR_4g_FLAG 				air_4g_flag;       //4Gģ���־λ
AIR_4g_CONNECT 				air_4g_connect;    //4Gģ�����Ӽ���
USARTX_HANDLE               usart3_handle_4g;
DEVICE_INFORM 				device_inform;
MQTT_PUB_INFORM  			mqtt_pub_inform;
MQTT_OTA_INFORM 			mqtt_ota_inform;
SING_WORK_EVENT			    sing_work_event;
GPS_HANDLE 					gps_info;
BLE_INFORM 					ble_inform;
TIMER_HANDLE1 		        timer_info;
DESIRED_INFORM             desired_inform;


// �ַ���תʱ�������
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
	return timeStamp-28800;//Ŀǰʱ���ȡ��Ϊ����ʱ����mktimeת������utcʱ�����Լ������8Сʱ����
}

// ʱ���ת�ַ�������
char *TimeStampToString(time_t* timeStamp)
{
	struct tm *info;
	char* buffer_add;
	char 	buffer[16];
	info = localtime(timeStamp);//Ŀǰʱ�����ȡ��Ϊ����ʱ����localtimeת������utcʱ�����Լ������8Сʱ����
	strftime(buffer, 16, "%Y%m%d%H%M%S", info);
	buffer_add = buffer;
	return buffer_add;
}


// 4Gģ���ʼ��
void air_4g_init(uint8_t connectMode, uint8_t delay_way)
{
    // 4G����ʱ�����ʹ���3�����ź���
    //control_flag.Usart3_handle_flag = FALSE;

    // �ȴ�4gģ��������13S
    if(delay_way)  vTaskDelay (20000); else  HAL_Delay(20000);

    // �ر������ϱ�����Ϣ�����Թ�3�Σ�ȷ���ر�
    air4g_send_cmd("AT*CSQ=0\r", "OK", 20, delay_way);
    air4g_send_cmd("AT+CGNSURC=0\r" , "OK" , 20 , delay_way);//�رմ�����GPS��Ϣ�����ϱ�
		air4g_send_cmd("AT+CGNSPWR=0\r" , "OK" , 20,delay_way);//�ر�GPS

    // ����ģʽ�ر�
    for(uint8_t i = 0; i < 5; i++)
    {
        if(delay_way)  vTaskDelay (1000); else  HAL_Delay(1000);
        if(!air4g_send_cmd("ATE0\r", "OK", 500, delay_way))
            break;
    }

    // ��鳵����MQTT�������Ƿ�����
    if(!air4g_send_cmd("AT+MQTTSTATU\r", "+MQTTSTATU :1", 150, delay_way))
    {
        air_4g_flag.MQTT_flag = TRUE;
        air_4g_flag.SIM_flag = TRUE;
    }

    // ��������������
    else if(air_4g_net_check(delay_way) == SIM_OK)
    {
        air_4g_flag.SIM_flag = TRUE;
    }

    // ��������MQTT������
    if((air_4g_flag.SIM_flag == TRUE) && (air_4g_flag.MQTT_flag == FALSE))
    {
        if(air_4g_connect_server(connectMode,delay_way) == MQTT_CONNECT_OK)
        {
            // ���MQTT����״̬
            // air4g_send_cmd("AT+MQTTSTATU\r", "OK", 500, delay_way);
            air_4g_flag.MQTT_flag = TRUE;
            //control_flag.Usart3_handle_flag = TRUE;
        }
    }

    // ��鳵����MQTT�������Ƿ�����
    if(!air4g_send_cmd("AT+MQTTSTATU\r", "+MQTTSTATU :1", 150, delay_way))
    {
        air_4g_OTAMPUB(delay_way);
        air_4g_OTASUB(delay_way);
		topic_sub(delay_way);
        air_4g_flag.MQTT_flag = TRUE;
    }

    // �������ϱ���GPS��CSQ
    air_4g_openURC(delay_way);

    //air4g_send_cmd("AT+ICCID\r", "OK" , 100, air_4g_flag.vTa_delay);            // ��ȡ��������ID
}

// �������ϱ�
uint8_t air_4g_openURC(uint8_t delay_way)
{
    // ��GPS
    air4g_send_cmd("AT+CGNSPWR=1\r", "OK" , 100, delay_way);

    // ʹ�ܸ�����λ
		air4g_send_cmd("AT+CGNSAID=31,1,1,1\r", "OK", 100, delay_way);

    // ����GPS�ϱ�Ƶ��Ϊ10��1��
    air4g_send_cmd("AT+CGNSURC=10\r", "OK", 100, delay_way);
    gps_info.gpsUrc                             = 1;

    // ��CSQ�����ϱ�
//    air4g_send_cmd("AT*CSQ=1\r", "OK", 100, delay_way);

    return 0;
}

// �ر������ϱ�
uint8_t air_4g_closeURC(uint8_t delay_way)
{
    // ��GPS
    // air4g_send_cmd("AT+CGNSPWR=1\r", "OK" , 100, delay_way);

    // ʹ�ܸ�����λ
	// air4g_send_cmd("AT+CGNSAID=31,1,1,1\r", "OK", 100, delay_way);

    // ����GPS�ϱ�Ƶ��Ϊ1��0�Σ��ر�
    air4g_send_cmd("AT+CGNSURC=0\r", "OK", 100, delay_way);
    gps_info.gpsUrc                             = 1;

//    // �ر�CSQ�����ϱ�
//    air4g_send_cmd("AT*CSQ=0\r", "OK", 100, delay_way);


    return 0;
}


// 4Gģ������
uint8_t air_4g_restar(bool delay_way)
{

    // ��λǰ�ر������ϱ���GPS��CSQ


    // 4Gģ�鸴λ��Ҫ��Ҫ���ô����յķ�ʽ��
    char buf[11];
    sprintf(buf,"AT+RSTSET\r");
    HAL_UART_Transmit_DMA(&huart3,(uint8_t*)buf,strlen(buf));

    air_4g_init(0,1);
    return	0;
}

// 4Gģ��������״̬
uint8_t air_4g_net_check(bool delay_way)
{
	// ����ģʽ�ر�
    // air4g_send_cmd("ATE0\r" , "OK" , 500,delay_way);

    // ��ѯPIN�룬����OK����������err:10δ��⵽������ѯ�Ƿ���SIM����
    if(air4g_send_cmd("AT+CPIN?\r" , "READY" , 500,delay_way))
        return SIM_CPIN_ERR;

    // ��ѯ����ע��״̬����ʱû�жԽ��������
    if(air4g_send_cmd("AT+CEREG?\r", "+CEREG: 0,1", 500, delay_way))
        return SIM_CREG_ERR;

    // ��ѯGPRS����״̬��+CGATT:1������������ +CGATT:0 δ�������磬���δ���ţ����ظ��Ŵ���
    if(air4g_send_cmd("AT+CGATT?\r", "+CGATT: 1", 500, delay_way))
        return SIM_CGATT_ERR;

    // ��ѯ������������ʱû�жԽ��������
    //uint8_t simCSQ = 0;
    //simCSQ = air4gCSQCheck(0);
    //if(simCSQ < 5)
    //    return SIM_CSQ_LOW;

    return SIM_OK;
}

// 4Gģ�����ӵ�MQTT������
uint8_t air_4g_connect_server(uint8_t connectMode, uint8_t delay_way)
{

    if(connectMode)
    {
        // �ر�MQTT���ӣ��տ�������ָ��᷵��767����
        if(air4g_send_cmd("AT+MDISCONNECT\r", "OK" , 50,delay_way))
            return MQTT_MDISCONNECT_ERR;

        // �ر�TCP����,�տ�������ָ��᷵��767����
        if(air4g_send_cmd("AT+MIPCLOSE\r", "OK" , 50,delay_way))
            return MQTT_MIPCLOSE_ERR;

        // IPӦ�����ã�IP������ָ��᷵��3����
        air4g_send_cmd("AT+SAPBR=0,1\r", "OK" , 50,delay_way);
	}

	// ���ó�������ΪGPRS
	air4g_send_cmd("AT+SAPBR=3,1,\"CONTYPE\",\"GPRS\"\r" , "OK" , 500,delay_way);

	// ����PDP���ص�APN����
	air4g_send_cmd("AT+SAPBR=3,1,\"APN\",\"\"\r" , "OK" , 500,delay_way);

	// ����PDP���������
	air4g_send_cmd("AT+SAPBR=1,1\r" , "OK" , 500,delay_way);

    // ��ѯPDP��ַ����4G��������IP����ʱû�н���IP�����ص�1��1�ǳ��������ı�ʾ�����ص�2��1�Ǳ�ʾ�����Ѿ�����
	if(air4g_send_cmd("AT+SAPBR=2,1\r" , "+SAPBR: 1,1" , 200,delay_way))
        return PDP_ACTIVE_ERR;

	// ����MQTT��ز���
	sprintf(device_inform.AtStrBuf,"AT+MCONFIG=\"%s.%s|securemode=2,signmethod=hmacsha256,timestamp=%s|\",\"%s&%s\",\"%s\"\r",
			device_inform.ProductKey, device_inform.DeviceName, device_inform.timestamp, device_inform.DeviceName, device_inform.ProductKey, device_inform.Password);
	if(air4g_send_cmd(device_inform.AtStrBuf , "OK" , 500,delay_way))
		return	MQTT_MCONFIG_ERR;

	// ����TCP����
	sprintf(device_inform.AtStrBuf,"AT+MIPSTART=\"%s\",%s\r",
			device_inform.server_host,device_inform.server_port);
	if(air4g_send_cmd_mqtt(device_inform.AtStrBuf , "CONNECT" , "OK" , 500,delay_way))
		return	MQTT_MIPSTART_ERR;

	// MQTT���ӳɹ������λ��ͣ���һ��OK�ǻ�Ӧ����ָ��ڶ���CONNECT OK�������ӳɹ���δ���ӳɹ��������������ʧ��
	// �ͻ��������������Ự����	AT+MCONNECT=<clean_session>, <keepalive>
	if(air4g_send_cmd_mqtt("AT+MCONNECT=0,60\r" , "CONNACK" , "OK" , 400,delay_way))
		return	MQTT_MCONNECT_ERR;

	return	MQTT_CONNECT_OK;
}


// 4Gģ��ָ��ͺ���: sendcmd:������ָ��   checkbuf:У��ָ��  waitsec:�ȴ�ʱ��   delay_mode:��ʱ����ʹ��ģʽ
uint8_t air4g_send_cmd(char* sendcmd, char* checkbuf, uint16_t waitsec, uint8_t delay_mode )
{
	uint8_t len_scmd;

    // ��ȡ���ݳ��ȼ���������
	len_scmd = strlen(sendcmd);
	HAL_UART_Transmit_DMA(&huart3, (uint8_t*)sendcmd, len_scmd);

    // ��ʱ
	if(delay_mode)  vTaskDelay (waitsec); else  HAL_Delay(waitsec);

    // �ж�ATָ��ؽ��
	if(strstr((char*)usart3_handle_4g.save_buf, checkbuf) == NULL )
	{
		memset(usart3_handle_4g.save_buf,0,SAVE_SIZE);
		return  1;
	}

    // ��մ��ڽ��ջ�����
	memset(usart3_handle_4g.save_buf,0,SAVE_SIZE);
	return  0;
}


// 4Gģ��MQTTָ��ͺ���:sendcmd:������ָ��   checkbuf:У��ָ��  checkbu2:У��ָ��  waitsec:�ȴ�ʱ��   delay_mode:��ʱ����ʹ��ģʽ
uint8_t air4g_send_cmd_mqtt(char* sendcmd, char* checkbuf, char* checkbu2, uint16_t waitsec, uint8_t delay_mode )
{
	uint8_t len_scmd;

    // ��ȡ���ݳ��ȼ���������
	len_scmd = strlen(sendcmd);
	HAL_UART_Transmit_DMA(&huart3,(uint8_t*)sendcmd,len_scmd);

    // ��ʱ
	if(delay_mode)  vTaskDelay(waitsec);    else  HAL_Delay  (waitsec);

    // �ж�ATָ��ؽ��
	if((strstr((char*)usart3_handle_4g.save_buf,checkbuf) || strstr((char*)usart3_handle_4g.save_buf,checkbu2))==NULL)
	{
		memset(usart3_handle_4g.save_buf,0,SAVE_SIZE);
		return 1;
	}

    // ��մ��ڽ��ջ�����
	memset(usart3_handle_4g.save_buf,0,SAVE_SIZE);

	return 0;
}


// �����ź�������ѯ
uint8_t air4gCSQCheck(uint8_t delay_mode)
{
    char sendcmd[10];
    uint8_t len_scmd;
    uint8_t simCSQ = 0;
    sprintf(sendcmd,"AT+CSQ\r");

    // ����ָ��
    len_scmd = strlen(sendcmd);
	HAL_UART_Transmit_DMA(&huart3, (uint8_t*)sendcmd, len_scmd);

    // ��ʱ
	if(delay_mode)  vTaskDelay (400); else  HAL_Delay(400);

    // �ж�ATָ��ؽ��
    if(strstr((char*)usart3_handle_4g.save_buf,"+CSQ:")!=NULL)
    {
        char *token;
        token=strtok((char*)usart3_handle_4g.save_buf,":");
        token=strtok(NULL,",");
        simCSQ = atoi(token);

        memset(usart3_handle_4g.save_buf,0,SAVE_SIZE);
        return simCSQ;
    }

    // ��մ��ڽ��ջ�����
	memset(usart3_handle_4g.save_buf,0,SAVE_SIZE);

	return  0;
}

// ��վ��λ��Ϣ��ѯ
uint8_t lbsInfoRead(uint8_t delay_mode)
{
    char sendcmd[20];
    uint8_t len_scmd;
    sprintf(sendcmd,"AT+CIPGSMLOC=1,1\r");

    // ����ָ��
    len_scmd = strlen(sendcmd);
    HAL_UART_Transmit_DMA(&huart3, (uint8_t*)sendcmd, len_scmd);

    // ��ʱ�ȴ���Ϣ
    for(uint8_t i = 0; i <= 15; i++)
    {
        if(delay_mode)  vTaskDelay (400); else  HAL_Delay(400);

        // �ж�ATָ��ؽ��
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

        // ��մ��ڽ��ջ�����
        memset(usart3_handle_4g.save_buf,0,SAVE_SIZE);
    }

    // ��մ��ڽ��ջ�����
    memset(usart3_handle_4g.save_buf,0,SAVE_SIZE);
    return  0;

}

// ��ȡ4Gʵʱʱ��
void get_real_time(bool delay_way)
{
	air4g_send_cmd("AT+CCLK?\r", "OK", 100, delay_way);
}

// ��ȡ4G��ǰ��Ϣ
void get_4G_msg(bool delay_way)
{
	air4g_send_cmd("ATE0\r", "OK", 50, delay_way);
	air4g_send_cmd("AT+MQTTSTATU\r", "+MQTTSTATU", 50, delay_way);	//��ѯMQTT����״̬
	air4g_send_cmd("AT+CSQ\r", "OK", 50, delay_way);		//��ѯ4G�ź�����
	//..
}

//gps���Ӽ�� ����ģʽ��
void gpsCheck(bool delay_way)
{   //���GPS��ʧ�رղ����´�
    if(gps_info.gps_signal_flag != 0x31)
    {
        air4g_send_cmd("ATE0\r" , "OK" , 20,delay_way);
        air4g_send_cmd("AT+CGNSURC=0\r" , "OK" , 100 , delay_way);//�رմ�����GPS��Ϣ�����ϱ�
        air4g_send_cmd("AT+CGNSPWR=0\r" , "OK" , 100,delay_way);//�ر�GPS

        air4g_send_cmd("AT+CGNSPWR=1\r" , "OK" , 100 , delay_way);//��GPS
        air4g_send_cmd("AT+CGNSAID=31,1,1,1\r" , "OK" , 1000,delay_way);//ʹ�ܸ�����λ
		air4g_send_cmd("AT+CGNSURC=1\r" , "OK" , 100,delay_way);//���ô�����GPS��Ϣ�����ϱ�  1hz
    }
}

/*************������������***************/
void BLTOXY(CRDCARTESIAN * pcc, CRDGEODETIC * pcg, int Datum, int zonewide)
{
    double B = pcg->latitude; //γ��
    double L = pcg->longitude; //����//γ�ȶ���

    double L0; //���뾭�߶���
    double N; //î��Ȧ���ʰ뾶
    double q2;
    double x; //��˹ƽ��������
    double y; //��˹ƽ�������
    double s; //�����γ��B�ľ��߻���
    double f; //�ο����������
    double e1; //�����һƫ����
    double a; //�ο������峤����
    //double b;    //�ο�������̰���
    double a1, a2, a3, a4;
    double b1, b2, b3, b4;
    double c0, c1, c2, c3;

    const double IPI = 0.0174532925199433333333; //3.1415926535898/180.0

    int prjno = 0; //ͶӰ����
    // zonewide ͶӰ������� 3 ������ 6
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
     * ���� 54
     * ������a=6378245m
     * �̰���b=6356863.0188m
     * ���ʦ�=1/298.3
     * ��һƫ����ƽ�� =0.006693421622966
     * �ڶ�ƫ����ƽ�� =0.006738525414683
     *
     * ����80
     * ������a=6378140��5��m��
     * �̰���b=6356755.2882m
     * ���ʦ�=1/298.257
     * ��һƫ����ƽ�� =0.00669438499959
     * �ڶ�ƫ����ƽ��=0.00673950181947
     *
     * WGS84
     * ������a=6378137�� 2��m��
     * �̰���b=6356752.3142m
     * ���ʦ�=1/298.257223563
     * ��һƫ����ƽ�� =0.00669437999013
     * �ڶ�ƫ����ƽ�� =0.00673949674223
     *
     */

    //Datum ͶӰ��׼�����ͣ�����54��׼��Ϊ54������80��׼��Ϊ80��WGS84��׼��Ϊ84
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

    e1 = 2 * f - f*f; //(a*a-b*b)/(a*a) �����һƫ����

    L0 = L0*IPI; // תΪ����
    L = L*IPI; // תΪ����
    B = B*IPI; // תΪ����

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


// ����ֵ��λ����
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

// ����������γ������֮��ľ��룬����ֵ��λΪ��
uint16_t BLDistanceDiff(uint8_t *Lon_nowstr1, uint8_t *Lat_nowstr1, uint8_t *Lon_nowstr2, uint8_t *Lat_nowstr2)
{
    CRDGEODETIC pcg1 = {atof((char *)Lon_nowstr1), atof((char *)Lat_nowstr1), 0};
    CRDGEODETIC pcg2 = {atof((char *)Lon_nowstr2), atof((char *)Lat_nowstr2), 0};
    return (uint16_t)BLDistance(&pcg1, &pcg2, WGS84, ZONEWIDE6);
}

// ������
void ble_swit_on(bool delay_way)
{
    // ����������ģʽ
	air4g_send_cmd("AT+BTCOMM=ENABLE,1,0\r" , "OK" , 1800 , delay_way);

    // ������������
	air4g_send_cmd(ble_inform.ble_name, "OK" , 200, delay_way);

    // ���ù㲥�����ݣ�0x02 0x01 0x06 Header�̶� 0x03(��λ) 0xff(�����Զ����type) 0x00 0x01 (������ָʾ256*256=65536̨�豸)
	air4g_send_cmd("AT+BLEADV=ADVDATA,7,02010603ff0001\r" , "OK" , 200, delay_way);

    // ������Ӧ������
    air4g_send_cmd("AT+BLEADV=SCANRSPDATA,4,03ff0001\r" , "OK" , 200, delay_way);

    // �򿪹㲥�ȴ����ӳɹ�
    air4g_send_cmd("AT+BLEADV=ENABLE,1\r" , "OK" , 300, delay_way);
}

// �ر�����
void ble_swit_off(bool delay_way)
{
    // �رչ㲥
	air4g_send_cmd("AT+BLEADV=ENABLE,0\r" , "OK" , 300 , delay_way);

    // �ر���������ģʽ
	air4g_send_cmd("AT+BTCOMM=ENABLE,0,0\r" , "OK" , 200,delay_way);
}


// ���4Gģ�鷢�͵���Ϣ�Ƿ�Ϊ�Զ�����������
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


// 4Gģ���ϱ���ǰ�̼���Ϣ
void air_4g_OTAMPUB(uint8_t delay_way)
{

    // �ϱ����ع̼��汾
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

// ���������豸��ȡOTA�̼�����
void air_4g_OTASUB(uint8_t delay_way)
{
    sprintf(ota_inform.PubBuf,"AT+MSUB=\"%s\",1\r"
                                ,ota_inform.msubOTA_theme_str
                            );
    air4g_send_cmd(ota_inform.PubBuf , "OK" , 200 , delay_way);
    //HAL_UART_Transmit_DMA(&huart3, (uint8_t*)ota_inform.PubBuf, strlen(ota_inform.PubBuf));
}

// 4G����HTTP��GET����
void air_4g_http_otarequest(void)
{
    // ��ֹHTTP����
    air4g_send_cmd("AT+HTTPTERM\r" , "OK" , 100, 1);

    // ��ʼ��HTTP����
    air4g_send_cmd("AT+HTTPINIT\r" , "OK" , 100, 1);

    // ���ûỰ����CID
    air4g_send_cmd("AT+HTTPPARA=\"CID\",1\r" , "OK" , 100, 1);

    // ���ûỰ���� URL
    sprintf(ota_inform.ota_http_urlATcmd,"AT+HTTPPARA=\"URL\",\"%s\"\r",ota_inform.ota_http_url);
    air4g_send_cmd(ota_inform.ota_http_urlATcmd, "OK", 100, 1);

    //����HTTP GET ����
    air4g_send_cmd("AT+HTTPACTION=0\r", "OK" , 200, 1);

    //sprintf(buf,"AT+HTTPACTION=0\r");
    //HAL_UART_Transmit_DMA(&huart3,(uint8_t*)buf,strlen(buf));
}

// �豸������ȥ�̼�������Ϣ
void air_4g_OTAREQUEST(void)
{
    sprintf(mqtt_ota_inform.PubBuf,"AT+MPUB=\"%s\",0,0,\"{\\22id\\22:\\22202\\22,\\22params\\22:{\\22version\\22:\\22%s\\22}}\"\r"
                                ,mqtt_ota_inform.request_theme_str
                                ,device_inform.version
                            );
    HAL_UART_Transmit_DMA(&huart3, (uint8_t*)mqtt_ota_inform.PubBuf, strlen(mqtt_ota_inform.PubBuf));
}

// ��������
void topic_sub(uint8_t delay_way)
{
    sprintf(desired_inform.PubBuf,"AT+MSUB=\"%s\",1\r"
                                ,desired_inform.desired_reply_theme_str
                            );
    air4g_send_cmd(desired_inform.PubBuf , "OK" , 200 , delay_way);
}

//static uint32_t countid = 1213;
// 4Gģ���ϱ���ģ����Ϣ
void air_4g_MPUB(uint8_t car_state)
{
    switch(car_state)
    {
        case CLOSE:
            {
                mqtt_pub_inform.vehicleStatus = 0x01;
                if(battery_data.charge_overflag == DISABLE && mqtt_pub_inform.start_summary_flag == DISABLE)
                {
                    sprintf(mqtt_pub_inform.PubBuf,"AT+MPUB=\"%s\",0,0,\"{\\22ID\\22:%d,\\22params\\22:{\\22senser:temperatureHumiditySensor1Temp\\22:%d,\\22senser:temperatureHumiditySensor1Humi\\22:%d,\\22senser:temperatureHumiditySensor2Temp\\22:%d,\\22senser:temperatureHumiditySensor2Humi\\22:%d,\\22senser:liquidLevelSensor\\22:%d,\\22driverMpuTemp\\22:%d,\\22driverBoxTemp1\\22:%d,\\22driverBoxTemp2\\22:%d,\\22driverMcuTemp\\22:%d,\\22battery:realTimeCurrent\\22:%d,\\22battery:batteryWarningStatus\\22:%d,\\22battery:capacitySoc\\22:%d,\\22battery:batteryVoltageTotal\\22:%d,\\22battery:electricCoreVoltageMax\\22:%d,\\22battery:electricCoreVoltageMin\\22:%d,\\22battery:batteryEnvironmentTemp\\22:%d,\\22battery:batteryTemp1\\22:%d,\\22battery:batteryTemp2\\22:%d,\\22battery:batteryTemp3\\22:%d,\\22battery:batteryTemp4\\22:%d,\\22basic:longitude\\22:\\22%s\\22,\\22basic:latitude\\22:\\22%s\\22,\\22basic:vehicleStatus\\22:%d,\\22debug:openMainBoxCheck\\22:%d,\\22debug:openDriverBoxCheck\\22:%d,\\22debug:gpsSignal\\22:%d,\\22debug:cellularSignalQuality\\22:%d,},\\22method\\22:\\22thing.event.property.post\\22}\"\r"
                                ,mqtt_pub_inform.theme_str
                                ,100

                                // ������ģ��
                                ,SHT30Environment_temperature / 10				// SHT30�����¶ȣ� / 10����APP��ʱ��ʾ����
                                ,SHT30Environment_humidity						// SHT30����ʪ��
                                ,SHT30CarBox_temperature      					// SHT30�����¶�
                                ,SHT30CarBox_humidity							// SHT30����ʪ��
                                ,spraySensor_waterLevel / 100                   // Һλ

                                // Ĭ��ģ��
                                ,driverBoard_mpu6050Temp						// ������MPU6050�¶�
                                ,driverBoard_PT100Temp1							// �������¶�1
                                ,driverBoard_PT100Temp2							// �������¶�2
                                ,driverBoard_mcuTempRise						// ������MCU�¶�

                                // ���ģ��
                                ,battery_data.currentCurrent					// ʵʱ����
                                ,battery_data.warningState						// �澯״̬
                                ,battery_data.soc											// ���SOC
                                ,battery_data.voitageCurrent					// ����ܵ�ѹ
                                ,battery_data.cellVoitageMax					// �����ߵ�о��ѹ
                                ,battery_data.cellVoitageMin					// �����͵�о��ѹ
                                ,battery_data.batt_temp[0]						// ��ػ����¶�
                                ,battery_data.batt_temp[0]						// ����¶�1
                                ,battery_data.batt_temp[1]						// ����¶�2
                                ,battery_data.batt_temp[2]						// ����¶�3
                                ,battery_data.batt_temp[3]						// ����¶�4
                                //,battery_data.capDesign							// ����������
                                //,battery_data.capOverall						// ���������

                                // �����������

                                // ����ģ��
                                ,gps_info.Lon_nowstr                            // ����
                                ,gps_info.Lat_nowstr                            // γ��
                                ,mqtt_pub_inform.vehicleStatus                  // ����״̬01���ػ�

                                // ��������

                                // ����ģ��
                                ,control_flag.main_kaihe_flag                   // ���ؿ��б�־λ
                                ,driverBoard_uncoverFlag     				    // �����忪�б�־λ
                                ,gps_info.gps_signal_flag				    // GPS�źű�־λ���ϱ�ֵ�ڶ�λ
                                ,air_4g_connect.sim_CSQ							// SIM�������ź�����

                            );
                }
                else if(battery_data.charge_overflag == ENABLE)		// �������ϱ����һ�γ��ʱ�䣿
                {
                    battery_data.charge_overflag = DISABLE;
                    sprintf(mqtt_pub_inform.PubBuf,"AT+MPUB=\"%s\",0,0,\"{\\22params\\22:{\\22battery:chargeTimeRemain\\22:%d},\\22method\\22:\\22thing.event.property.post\\22}\"\r"
                    ,mqtt_pub_inform.theme_str
                    ,0);
                }
                else if(mqtt_pub_inform.start_summary_flag == ENABLE)		// �ػ��ϱ���������
                {
                    mqtt_pub_inform.start_summary_flag = DISABLE;			// ���ؿ���ʱ�ϱ�һ�λ�������
                    if(battery_data.circle > 300)
                    {
                        battery_data.soh =90;
                    }
                    sprintf(mqtt_pub_inform.PubBuf,"AT+MPUB=\"%s\",0,0,\"{\\22ID\\22:%d,\\22params\\22:{\\22summary:totalRunTime\\22:%d,\\22summary:totalFanMachineryTime\\22:%d,\\22summary:totalWaterPumpTime\\22:%d,\\22summary:totalMileage\\22:%d,\\22battery:batteryLoop\\22:%d,\\22battery:batteryCapacityPlan\\22:%1f,\\22battery:batteryCapacityFull\\22:%1f,\\22battery:hardwareVersion\\22:\\22%s\\22,\\22battery:healthStatus\\22:%d,\\22battery:batteryType\\22:\\22%s\\22,\\22battery:softwareVersion\\22:\\22%s\\22,\\22battery:batterySN\\22:\\22%s\\22,\\22debug:appRestartReason\\22:%d,},\\22method\\22:\\22thing.event.property.post\\22}\"\r"
                    ,mqtt_pub_inform.theme_str
                    ,102

                    // ��������
                    ,summary_data.totalRunTime*1000                     // ����������ʱ��
                    ,summary_data.totalFanMachineryTime*1000            // �������������ʱ��
                    ,summary_data.totalWaterPumpTime*1000               // ����ˮ��������ʱ��
                    ,summary_data.totalMileage						    // ���������

                    // ���ģ��
                    ,battery_data.circle								// ���ѭ������
                    ,(double)battery_data.capDesign						// ����������
                    ,(double)battery_data.capOverall					// ���������
                    ,battery_data.battHarewareVersion					// ���Ӳ���汾��
                    ,battery_data.soh									// ��ؽ���״��SOH
                    ,battery_data.battTypeName							// ����ͺ�����
                    ,battery_data.battSoftwareVersion					// �������汾��
                    ,battery_data.battSN								// ���SN��

                    // APP�ϴθ�λԭ��
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

                                // ������ģ��
                                ,SHT30Environment_temperature / 10				    // SHT30�����¶ȣ� / 10����APP��ʱ��ʾ����
                                ,SHT30Environment_humidity							// SHT30����ʪ��
                                ,SHT30CarBox_temperature      						// SHT30�����¶�
                                ,SHT30CarBox_humidity								// SHT30����ʪ��
                                ,spraySensor_waterPressure                          // ѹ��
                                ,spraySensor_waterFlow                      // ����
                                ,spraySensor_waterLevel / 100                       // Һλ

                                // Ĭ��ģ��

                                // ���ģ��
                                ,battery_data.currentCurrent						// ʵʱ����
                                ,battery_data.warningState							// �澯״̬
                                ,battery_data.soc												// ���SOC
                                ,battery_data.voitageCurrent						// ����ܵ�ѹ
                                ,battery_data.cellVoitageMax						// �����ߵ�о��ѹ
                                ,battery_data.cellVoitageMin						// �����͵�о��ѹ
                                ,battery_data.batt_temp[0]							// ��ػ����¶�
                                ,battery_data.batt_temp[0]							// ����¶�1
                                ,battery_data.batt_temp[1]							// ����¶�2
                                ,battery_data.batt_temp[2]							// ����¶�3
                                ,battery_data.batt_temp[3]							// ����¶�4

                                // �����������

                                // ����ģ��
                                ,gps_info.Lon_nowstr                			    // ����
                                ,gps_info.Lat_nowstr                			    // γ��
                                ,mqtt_pub_inform.vehicleStatus      			    // ����״̬

                                // ��������

                                // ����ģ��
                                ,control_flag.main_kaihe_flag       			    // ���ؿ��б�־λ
                                ,driverBoard_uncoverFlag     						// �����忪�б�־λ
                                ,gps_info.gps_signal_flag						// GPS�źű�־λ���ϱ�ֵ�ڶ�λ
                                ,air_4g_connect.sim_CSQ								// SIM�������ź�����
                            );
                }
                break;

        case RUN:
            {
                if(mqtt_pub_inform.rundata_turn_count<5)		// ����ʵʱ��ͼҳ������
                {
                    mqtt_pub_inform.rundata_turn_count++;
                    mqtt_pub_inform.vehicleStatus = mqtt_pub_inform.runvehicleStatus;   // ����״̬���ϴ��ĳ���״̬Ϊ�ɽ�������
                    mqtt_pub_inform.vehicleSpeed  = can_read_data.Speed;
                    sprintf(mqtt_pub_inform.PubBuf,"AT+MPUB=\"%s\",0,0,\"{\\22ID\\22:%d,\\22params\\22:{\\22senser:waterPressureSensor\\22:%d,\\22senser:flowSensor\\22:%d,\\22senser:liquidLevelSensor\\22:%d,\\22motorWaterPump\\22:%d,\\22fanMachinery\\22:%d,\\22anglePitch\\22:%d,\\22angleRoll\\22:%d,\\22battery:realTimeCurrent\\22:%d,\\22battery:capacitySoc\\22:%d,\\22basic:longitude\\22:\\22%s\\22,\\22basic:latitude\\22:\\22%s\\22,\\22basic:vehicleStatus\\22:%d,\\22basic:vehicleSpeed\\22:%d,},\\22method\\22:\\22thing.event.property.post\\22}\"\r"
                                    ,mqtt_pub_inform.theme_str
                                    ,103
                                    // ������ģ��
                                    ,spraySensor_waterPressure                  // ˮѹ ÿKPa
                                    ,spraySensor_waterFlow/1000                 // ���� ÿ1ml
                                    ,spraySensor_waterLevel/100                 // ˮλ ÿ10ml

                                    // Ĭ��ģ��
                                    ,mqtt_pub_inform.motorWaterPump             // ˮ�ù���
                                    ,mqtt_pub_inform.fanMachinery				// ���
                                    ,driverBoard_carPitchAngle					// ���
                                    ,driverBoard_carRollAngle						// ������

                                    // ���ģ��
                                    ,battery_data.currentCurrent				// ʵʱ����
                                    ,battery_data.soc							// ���SOC

                                    // �����������

                                    // ����ģ��
                                    ,gps_info.Lon_nowstr                        // ����
                                    ,gps_info.Lat_nowstr                        // γ��
                                    ,mqtt_pub_inform.vehicleStatus              // ����״̬
                                    ,mqtt_pub_inform.vehicleSpeed				// ����

                                    // ��������

                                    // ����ģ��

                                );
                }
                else
                {
                    mqtt_pub_inform.rundata_turn_count=0;
                    sprintf(mqtt_pub_inform.PubBuf,"AT+MPUB=\"%s\",0,0,\"{\\22ID\\22:%d,\\22params\\22:{\\22senser:temperatureHumiditySensor1Temp\\22:%d,\\22senser:temperatureHumiditySensor1Humi\\22:%d,\\22senser:temperatureHumiditySensor2Temp\\22:%d,\\22senser:temperatureHumiditySensor2Humi\\22:%d,\\22driverMpuTemp\\22:%d,\\22driverBoxTemp1\\22:%d,\\22driverBoxTemp2\\22:%d,\\22driverMcuTemp\\22:%d,\\22battery:batteryWarningStatus\\22:%d,\\22battery:batteryVoltageTotal\\22:%d,\\22battery:electricCoreVoltageMax\\22:%d,\\22battery:electricCoreVoltageMin\\22:%d,\\22battery:batteryEnvironmentTemp\\22:%d,\\22battery:batteryTemp1\\22:%d,\\22battery:batteryTemp2\\22:%d,\\22battery:batteryTemp3\\22:%d,\\22battery:batteryTemp4\\22:%d,\\22motor:state\\22:%d,\\22motor:temp1\\22:%d,\\22motor:temp2\\22:%d,\\22motor:temp3\\22:%d,\\22motor:temp4\\22:%d,\\22debug:gpsSignal\\22:%d,\\22debug:cellularSignalQuality\\22:%d,},\\22method\\22:\\22thing.event.property.post\\22}\"\r"
                                    ,mqtt_pub_inform.theme_str
                                    ,104

                                    // ������ģ��
                                    ,SHT30Environment_temperature / 10				// SHT30�����¶ȣ� / 10����APP��ʱ��ʾ����
                                    ,SHT30Environment_humidity						// SHT30����ʪ��
                                    ,SHT30CarBox_temperature      					// SHT30�����¶�
                                    ,SHT30CarBox_humidity							// SHT30����ʪ��

                                    // Ĭ��ģ��
                                    ,driverBoard_mpu6050Temp						// ������MPU6050�¶�
                                    ,driverBoard_PT100Temp1							// �������¶�1
                                    ,driverBoard_PT100Temp2							// �������¶�2
                                    ,driverBoard_mcuTempRise						// ������MCU�¶�

                                    // ���ģ��
                                    ,battery_data.warningState						// �澯״̬
                                    ,battery_data.voitageCurrent					// ����ܵ�ѹ
                                    ,battery_data.cellVoitageMax					// �����ߵ�о��ѹ
                                    ,battery_data.cellVoitageMin					// �����͵�о��ѹ
                                    ,battery_data.batt_temp[0]						// ��ػ����¶�
                                    ,battery_data.batt_temp[0]						// ����¶�1
                                    ,battery_data.batt_temp[1]						// ����¶�2
                                    ,battery_data.batt_temp[2]						// ����¶�3
                                    ,battery_data.batt_temp[3]						// ����¶�4

                                    // �����������
                                    ,can_read_data.moto_state						// ����������쳣״̬��
                                    ,can_read_data.moto_temp1						// ����������¶�1
                                    ,can_read_data.moto_temp2						// ����������¶�2
                                    ,can_read_data.moto_temp3						// ����������¶�3
                                    ,can_read_data.moto_temp4						// ����������¶�4

                                    // ����ģ��
                                    ,gps_info.gps_signal_flag				        // GPS�źű�־λ���ϱ�ֵ�ڶ�λ
                                    ,air_4g_connect.sim_CSQ							// SIM�������ź�����
                                );
                }
            }
            break;

        case RECHARGE:
            {
                mqtt_pub_inform.vehicleStatus = 0x05;
                if(mqtt_pub_inform.start_charge_flag == DISABLE)
                {
                    sprintf(mqtt_pub_inform.PubBuf,"AT+MPUB=\"%s\",0,0,\"{\\22ID\\22:%d,\\22params\\22:{\\22senser:temperatureHumiditySensor1Temp\\22:%d,\\22senser:temperatureHumiditySensor1Humi\\22:%d,\\22senser:temperatureHumiditySensor2Temp\\22:%d,\\22senser:temperatureHumiditySensor2Humi\\22:%d,\\22senser:liquidLevelSensor\\22:%d,\\22battery:realTimeCurrent\\22:%d,\\22battery:batteryProtectStatus\\22:%d,\\22battery:batteryWarningStatus\\22:%d,\\22battery:capacitySoc\\22:%d,\\22battery:chargeTimeRemain\\22:%d,\\22battery:batteryVoltageTotal\\22:%d,\\22battery:electricCoreVoltageMax\\22:%d,\\22battery:electricCoreVoltageMin\\22:%d,\\22battery:batteryEnvironmentTemp\\22:%d,\\22battery:batteryTemp1\\22:%d,\\22battery:batteryTemp2\\22:%d,\\22battery:batteryTemp3\\22:%d,\\22battery:batteryTemp4\\22:%d,\\22battery:spentChargeTime\\22:%d,\\22basic:longitude\\22:\\22%s\\22,\\22basic:latitude\\22:\\22%s\\22,\\22basic:vehicleStatus\\22:%d,\\22debug:openMainBoxCheck\\22:%d,\\22debug:openDriverBoxCheck\\22:%d,\\22debug:gpsSignal\\22:%d,\\22debug:cellularSignalQuality\\22:%d,},\\22method\\22:\\22thing.event.property.post\\22}\"\r"
                                    ,mqtt_pub_inform.theme_str
                                    ,105

                                    // ������ģ��
                                    ,SHT30Environment_temperature / 10				// SHT30�����¶ȣ� / 10����APP��ʱ��ʾ����
                                    ,SHT30Environment_humidity						// SHT30����ʪ��
                                    ,SHT30CarBox_temperature      					// SHT30�����¶�
                                    ,SHT30CarBox_humidity							// SHT30����ʪ��

                                    ,spraySensor_waterLevel / 100                   // Һλ

                                    // Ĭ��ģ��

                                    // ���ģ��
                                    ,battery_data.currentCurrent					// ʵʱ����
                                    ,battery_data.protect_state						// ��ر���״̬
                                    ,battery_data.warningState						// �澯״̬
                                    ,battery_data.soc								// ���SOC
                                    ,battery_data.chargeTimeRemain*60000			// ���ʣ��ʱ��
                                    ,battery_data.voitageCurrent					// ����ܵ�ѹ
                                    ,battery_data.cellVoitageMax					// �����ߵ�о��ѹ
                                    ,battery_data.cellVoitageMin					// �����͵�о��ѹ
                                    ,battery_data.batt_temp[0]						// ��ػ����¶�
                                    ,battery_data.batt_temp[0]						// ����¶�1
                                    ,battery_data.batt_temp[1]						// ����¶�2
                                    ,battery_data.batt_temp[2]						// ����¶�3
                                    ,battery_data.batt_temp[3]						// ����¶�4
                                    ,battery_data.spentChargeTime*10000			    // �ѳ��ʱ��
                                    //,battery_data.spentChargeTime*60000			// �ѳ��ʱ��

                                    // �����������

                                    // ����ģ��
                                    ,gps_info.Lon_nowstr                			// ����
                                    ,gps_info.Lat_nowstr                			// γ��
                                    ,mqtt_pub_inform.vehicleStatus      			// ����״̬

                                    // ��������

                                    // ����ģ��
                                    ,control_flag.main_kaihe_flag       			// ���ؿ��б�־λ
                                    ,driverBoard_uncoverFlag     					// �����忪�б�־λ
                                    ,gps_info.gps_signal_flag					// GPS�źű�־λ���ϱ�ֵ�ڶ�λ
                                    ,air_4g_connect.sim_CSQ							// SIM�������ź�����
                                );
                    battery_data.spentChargeTime++;
                }
                else
                {
                    //����һ������Դ���ӵ�ʱ��ͳ��ʣ��ʱ��
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
								sprintf(mqtt_pub_inform.PubBuf,"AT+MPUB=\"%s\",0,0,\"{\\22ID\\22:%d,\\22params\\22:{\\22senser:temperatureHumiditySensor1Temp\\22:%d,\\22senser:temperatureHumiditySensor1Humi\\22:%d,\\22senser:temperatureHumiditySensor2Temp\\22:%d,\\22senser:temperatureHumiditySensor2Humi\\22:%d,\\22senser:liquidLevelSensor\\22:%d,\\22driverMpuTemp\\22:%d,\\22emergencyStopStatus\\22:%d,\\22driverBoxTemp1\\22:%d,\\22driverBoxTemp2\\22:%d,\\22driverMcuTemp\\22:%d,\\22battery:realTimeCurrent\\22:%d,\\22battery:batteryProtectStatus\\22:%d,\\22battery:batteryWarningStatus\\22:%d,\\22battery:capacitySoc\\22:%d,\\22battery:batteryVoltageTotal\\22:%d,\\22battery:electricCoreVoltageMax\\22:%d,\\22battery:electricCoreVoltageMin\\22:%d,\\22battery:batteryEnvironmentTemp\\22:%d,\\22battery:batteryTemp1\\22:%d,\\22battery:batteryTemp2\\22:%d,\\22battery:batteryTemp3\\22:%d,\\22battery:batteryTemp4\\22:%d,\\22motor:state\\22:%d,\\22basic:longitude\\22:\\22%s\\22,\\22basic:latitude\\22:\\22%s\\22,\\22basic:vehicleStatus\\22:%d,\\22debug:openMainBoxCheck\\22:%d,\\22debug:openDriverBoxCheck\\22:%d,\\22debug:gpsSignal\\22:%d,\\22debug:cellularSignalQuality\\22:%d,},\\22method\\22:\\22thing.event.property.post\\22}\"\r"
																		,mqtt_pub_inform.theme_str
																		,107

																		// ������ģ��
																		,SHT30Environment_temperature / 10				// SHT30�����¶ȣ� / 10����APP��ʱ��ʾ����
																		,SHT30Environment_humidity						// SHT30����ʪ��
																		,SHT30CarBox_temperature      					// SHT30�����¶�
																		,SHT30CarBox_humidity							// SHT30����ʪ��

																		,spraySensor_waterLevel / 100                   // Һλ

																		// Ĭ��ģ��
																		,driverBoard_mpu6050Temp						// ������MPU6050�¶�
																		,driverBoard_scramStop							// ��ͣ����״̬
																		,driverBoard_PT100Temp1							// �������¶�1
																		,driverBoard_PT100Temp2							// �������¶�2
																		,driverBoard_mcuTempRise						// ������MCU�¶�

																		// ���ģ��
																		,battery_data.currentCurrent					// ʵʱ����
																		,battery_data.protect_state						// ��ر���״̬
																		,battery_data.warningState						// �澯״̬
																		,battery_data.soc											// ���SOC
																		,battery_data.voitageCurrent					// ����ܵ�ѹ
																		,battery_data.cellVoitageMax					// �����ߵ�о��ѹ
																		,battery_data.cellVoitageMin					// �����͵�о��ѹ
																		,battery_data.batt_temp[0]						// ��ػ����¶�
																		,battery_data.batt_temp[0]						// ����¶�1
																		,battery_data.batt_temp[1]						// ����¶�2
																		,battery_data.batt_temp[2]						// ����¶�3
																		,battery_data.batt_temp[3]						// ����¶�4

																		// �����������
																		,can_read_data.moto_state						// ����������쳣״̬��

																		// ����ģ��
																		,gps_info.Lon_nowstr                			// ����
																		,gps_info.Lat_nowstr                			// γ��
																		,mqtt_pub_inform.vehicleStatus      			// ����״̬01���ػ�

																		// ����ģ��
																		,control_flag.main_kaihe_flag       			// ���ؿ��б�־λ
																		,driverBoard_uncoverFlag     					// �����忪�б�־λ
																		,gps_info.gps_signal_flag					// GPS�źű�־λ���ϱ�ֵ�ڶ�λ
																		,air_4g_connect.sim_CSQ							// SIM�������ź�����
																		);
						}
            break;
    }
	char lenbuf[16];
	sprintf(lenbuf,"mqttlen:%d",strlen(mqtt_pub_inform.PubBuf));
    HAL_UART_Transmit_DMA(&huart3,(uint8_t*)lenbuf,strlen(lenbuf));
    vTaskDelay(10);

    HAL_UART_Transmit_DMA(&huart3, (uint8_t*)mqtt_pub_inform.PubBuf, strlen(mqtt_pub_inform.PubBuf));

	// for(int i=0;i<strlen(mqtt_pub_inform.PubBuf);i++)//ѭ����������
	// 	{
	// 		while((USART3->SR&0X40)==0){;}//ѭ������,ֱ���������
	// 		USART3->DR =mqtt_pub_inform.PubBuf[i];
	// 	}
	vTaskDelay(100);
	memset(mqtt_pub_inform.PubBuf,0,sizeof(mqtt_pub_inform.PubBuf));
}
/*-------------------------------------------------------------�¼�-------------------------------------------------------------------*/
// 4Gģ���ϱ��������м�¼����
void air_4g_MPUB_event(uint8_t eventid)
{
		time_t changeUTC = timer_info.timestamp;//��+28800��IOTĿǰ��UTC
		switch(eventid)
		{
				case WORK_EVENT_MAIN:
						memcpy(timer_info.Utc_nowstr, TimeStampToString(&changeUTC),14);// �Ӳ����ۼƵ�ʱ���ת����ʱ���ַ�����¼����ʱ��

						strcpy((char *)sing_work_event.end_date, (const char *)timer_info.Utc_nowstr);

						strcpy((char *)sing_work_event.latitude, (const char *)gps_info.Lat_nowstr);

						strcpy((char *)sing_work_event.longitude, (const char *)gps_info.Lon_nowstr);

						sprintf((char *)sing_work_event.recordID,"%s%s"
									,sing_work_event.start_date
									,sing_work_event.end_date+9
								);

						sing_work_event.power = sing_work_event.start_power - battery_data.soc;                      //���м�¼�ĵ�������

                        if (sing_work_event.start_drug - spraySensor_waterLevel > 0)                                 //���м�¼��ҩ������
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
									,sing_work_event.recordID//�Խ���ʱ����Ϊ���м�¼�����������ݵı�ʶ
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


/************************************δʹ�õĺ���****************************************/


//YY_HANDLE 			yy_module;


//// ��GPS
//void gps_swit_on(bool delay_way)
//{
//	air4g_send_cmd("ATE0\r" , "OK" , 20,delay_way);
//	air4g_send_cmd("ATE0\r" , "OK" , 20,delay_way);
//
//	air4g_send_cmd("AT+CGNSPWR=1\r" , "OK" , 100 , delay_way);			 		//��GPS
//	air4g_send_cmd("AT+CGNSAID=31,1,1,1\r" , "OK" , 1000,delay_way);	 		//ʹ�ܸ�����λ
//	switch(control_flag.Car_State)
//		{
//			case CLOSE:
//				air4g_send_cmd("AT+CGNSURC=10\r" , "OK" , 100,delay_way);       //���ô�����GPS��Ϣ�����ϱ�0.1hz
//			break;
//			case OPEN:
//				air4g_send_cmd("AT+CGNSURC=5\r" , "OK" , 100,delay_way);        //���ô�����GPS��Ϣ�����ϱ�0.2hz
//			break;
//			case RUN:
//				air4g_send_cmd("AT+CGNSURC=1\r" , "OK" , 100,delay_way);        //���ô�����GPS��Ϣ�����ϱ�  1hz
//			break;
//			default:
//				air4g_send_cmd("AT+CGNSURC=10\r" , "OK" , 100,delay_way);       //���ô�����GPS��Ϣ�����ϱ�0.1hz
//		}
//}

//// �ر�GPS
//void gps_swit_off(bool delay_way)
//{
//	air4g_send_cmd("ATE0\r" , "OK" , 20,delay_way);
//	air4g_send_cmd("ATE0\r" , "OK" , 20,delay_way);
//	air4g_send_cmd("AT+CGNSURC=0\r" , "OK" , 100 , delay_way);//�رմ�����GPS��Ϣ�����ϱ�
//	air4g_send_cmd("AT+CGNSPWR=0\r" , "OK" , 100,delay_way);//�ر�GPS
//}

//// 4Gģ�����������������δʹ��
//void SPEAK(char* conten)
//{
//	sprintf(yy_module.buf_spk,"AT+CTTS=2,\"%s\"\r",conten);
//	HAL_UART_Transmit_DMA(&huart3,(uint8_t*)yy_module.buf_spk,strlen(yy_module.buf_spk));
////	memset(yy_module.buf_spk,0,30);
//}

