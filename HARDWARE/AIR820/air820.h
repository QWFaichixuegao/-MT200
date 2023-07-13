#ifndef __AIR820_H_
#define __AIR820_H_
#include "handle.h"
#include "cmsis_os.h"
#include "can.h"
#include "at24cxx.h"
#include "can_device.h"
#include "at24cxx.h"
#include "time.h"
#include "adc_read.h"

/*************************************************************����ͨ��******************************************************************/

#define AIR4GCHECKTIME 		2000
#define RSTSET_COUNT 		5
#define SHORT_MESG 			7		// usart3����Ϣ����
//#define LONG_MESG 		150		// usart3����Ϣ����
#define LONG_MESG 			850		// usart3����Ϣ����   debug



// 4Gģ��UARTͨѶ��������
#define RX_SIZE     1024
#define TX_SIZE  	128
#define SAVE_SIZE   1024

typedef struct
{
    char        rx_buf[RX_SIZE];
    char        save_buf[SAVE_SIZE];
    char        report_buf[SAVE_SIZE];
    char        tx_buf[TX_SIZE];
    uint16_t    rx_count;
    uint16_t    rx_len;
    uint16_t    tx_len;
}USARTX_HANDLE;
extern USARTX_HANDLE usart3_handle_4g;


/*************************************************************״̬��Ϣ�ϱ�******************************************************************/

// ��ҵ�ϱ���Ϣ�����
typedef enum
{
    WORK_EVENT_MAIN    		= 0,
    WORK_EVENT_TRACK		= 1,
    DEFAUT_EVENT    		= 2,
}MPUB_EVENT;

// ������ʾ��APP�ϵ�ʵʱ״̬�ϱ�
typedef struct
{
    uint16_t    vehicleStatus;
    uint16_t    runvehicleStatus;	//��������״̬  4λ5λ�ֶ������л�  8λ9λ�Զ���ͣ �Զ���ͣ  12λˮ��  13λ���  14λԭ��ֹͣ 15λ���
    uint16_t    vehicleSpeed;
    uint8_t     liquidLevelSensor;
    uint8_t     fanMachinery;
    uint8_t     motorWaterPump;
    uint8_t     start_summary_flag;
    uint8_t     start_charge_flag;          // ��ʼ����־����������IOT����
    uint8_t	    rundata_turn_count;
    char        theme_str[128];
    char        work_event_Main[128];
    char        work_event_Track[128];
    char        PubBuf[2048];				// ��512��Ϊ2048
    char        Pub_work_event_Buf[2048];

} MQTT_PUB_INFORM;
extern MQTT_PUB_INFORM mqtt_pub_inform;

// IOTƽ̨�ϱ�״̬���л�����Ŀ��ֵ
typedef enum
{
    Count_1s	=10,
    Count_5s  	=50,
    Count_10s  	=100,
    Count_12s  	=120,
    Count_60s  	=600,
} TIME_COUNT_MS;

/*************************************************************״̬��Ϣ�ϱ�(����)******************************************************************/
// ������ʾ��APP�ϵ�ʵʱ״̬�ϱ�
typedef struct
{
    char       tx_buf[2048];
} SBUS_PACK;
extern SBUS_PACK sbus_pack_data;

/*************************************************************mpu��λ������******************************************************************/
#pragma pack(push,1)
typedef struct
{
    uint8_t   header1;
    uint8_t   header2;
    uint16_t  soc;
    uint16_t  speed;
    uint16_t  medi;
    uint16_t  flow_rate;
    uint16_t  pressure;
    uint16_t  airflow;
    uint16_t  pump;
    uint8_t   control_mode;
    uint8_t   checksum;
} MPU_MSG_PACK;
#pragma pack()
extern MPU_MSG_PACK mpu_msg_pack;

#pragma pack(push,1)
typedef struct
{
    uint8_t   header1;
    uint8_t   header2;
    int16_t   linear_speed;//-1500~1500 0.001m/s
    int16_t   angular_speed;//��Ϊ�� ��Ϊ��
    uint8_t   airflow;
    uint8_t   pump;
    uint8_t   light;
    uint8_t   checksum;
} MPU_REC_PACK;
#pragma pack()
extern MPU_REC_PACK mpu_rec_pack;



/*************************************************************����******************************************************************/

// ������Ϣ
typedef struct
{
    char    dev_send_cmd[244];	        // 244BLE�������η��͵��ֽڳ���
    char 	dev_data_To16[244];	        // 244BLE�������η��͵��ֽڳ���
    char 	dev_bound_data[48];         // ����Ϣ[DeviceName]$[ProductKey]
    char 	ble_name[48]; 			    // ����������MQZN_DeviceName

    uint8_t ble_switch_state;           // ������ǰ����״̬
    uint8_t ble_switch;                 // ������������״̬
} BLE_INFORM;
extern BLE_INFORM ble_inform;


/*************************************************************��������******************************************************************/
// ��������״̬
typedef enum
{
    SIM_OK					=0,
    SIM_COMMUNTION_ERR  	=1,
    SIM_CPIN_ERR  		   	=2,
    SIM_CREG_ERR        	=3,
    SIM_CGATT_ERR           =4,
    SIM_CSQ_LOW        		=5,
} SIM_STATE;


// 4Gģ��״̬
typedef struct
{
    uint8_t vTa_delay;
    uint8_t hal_delay;
    uint8_t SIM_flag;
    uint8_t MQTT_flag;
		uint8_t MQTT_flag2;
    uint8_t send_version_flag;
	  uint8_t get_realtime_flag;
	  uint8_t addbuf1;//������
}AIR_4g_FLAG;
extern AIR_4g_FLAG air_4g_flag;


// ���ӷ��������IOTƽ̨����
typedef struct
{
	  int			sim_CSQ;
    uint8_t sim_ccount;
    uint8_t mqtt_ccount;
		uint8_t mqtt_state;
		uint8_t air820Count;
//	  uint8_t addbuf1;//������


}AIR_4g_CONNECT;
extern AIR_4g_CONNECT air_4g_connect;

// MQTT���ӷ�����״̬
typedef enum
{
    MQTT_CONNECT_OK    		= 0,
    MQTT_CGATT_ERR     		= 1,
    PDP_ACTIVE_ERR	 		= 2,
    MQTT_MCONFIG_ERR	 	= 3,
    MQTT_MIPSTART_ERR	 	= 4,
    MQTT_MCONNECT_ERR	 	= 5,
    MQTT_MIPCLOSE_ERR       = 6,
    MQTT_MDISCONNECT_ERR    = 7,

} MQTT_SERVER_STATE;


/*************************************************************GPS******************************************************************/

// GPS���ݽ����洢��
typedef struct
{
    uint16_t    distanceDif;
    int         MSL_Altitude;
    uint8_t	    Lon_nowstr[16];
    uint8_t	    Lat_nowstr[16];
	  uint8_t 		gps_signal_flag;	    // GPS�źŻ�ȡ��־λ
    uint8_t 		lbs_signal_flag;	    // GPS�źŻ�ȡ��־λ
		uint8_t     gpsUrc;                 // GPS��ǰ�ϱ������1��5
    uint8_t     gpsUrcSet;              // GPS�ϱ��������ֵ
    uint8_t     gpsCheckcount;              // ����״̬��GPS��������ۼ�
}GPS_HANDLE;
extern GPS_HANDLE gps_info;

// GPS���ں�ʱ������洢��
typedef struct
{
    uint8_t		Utc_nowstr[16];
		uint32_t  timestamp;

}TIMER_HANDLE1;
extern TIMER_HANDLE1 timer_info;

/*************������������***************/
#define     WGS84       84          // WGS84����ϵ��GPS ���꣩
#define     BJ54        54          // ����54����ϵ
#define     XIAN80      80          // ����80����ϵ
#define     ZONEWIDE3   3           // ͶӰ����� 3
#define     ZONEWIDE6   6           // ͶӰ����� 6

//��˹ƽ������ϵ
typedef struct
{
    double x;
    double y;
    double z;
} CRDCARTESIAN;

//�������ϵ�������� ����54����ϵ������80����ϵ��WGS84����ϵ��GPS ���꣩��
typedef struct
{
    double longitude; //����
    double latitude; //γ��
    double height; //��ظ�,����Ϊ0
} CRDGEODETIC;

/*************************************************************OTA*************************************************************/

// �ϱ�IOTƽ̨��OTA��Ϣ
typedef struct
{
    char    PubBuf[512];
    char    version_theme_str[64];
    char    request_theme_str[64];

} MQTT_OTA_INFORM;
extern MQTT_OTA_INFORM mqtt_ota_inform;



typedef struct
{
	uint32_t	ota_data_len;
	uint32_t 	ota_data_count;
	uint8_t 	ota_pack[1024];	   			 // ����IOT���͵� OTA ��Ϣ
	uint8_t 	ota_http_url[256]; 			 // ��ȡ������OTA��http url
	uint8_t		ota_data_md5[64];			 // ��ȡota�̼����ݵ�MD5У׼
	uint8_t		ota_version[8];				 // ��ȡ�������ص�OTA�汾��
	uint8_t		ota_data_lenstr[8];			 // ��ȡ�������ص�OTA�汾��
	uint8_t		get_version_flag;		     // ��ȡ�汾���Ƿ�ɹ�
	uint8_t		get_fireware_flag;			 // ��ȡ�¹̼��Ƿ�ɹ�
	char        PubBuf[512];
	char 	      ota_http_urlATcmd[512];      // ���ATָ�������http url
	char        version_theme_str[64];
	char        request_theme_str[64];
	char        msubOTA_theme_str[64];
} OTA_INFORM;
extern OTA_INFORM ota_inform;

// ����ֵ��Ϣ
typedef struct
{
	char       PubBuf[256];
    char       desired_reply_theme_str[80];
} DESIRED_INFORM;
extern DESIRED_INFORM desired_inform;


/*************************************************************��������*************************************************************/

uint8_t air4g_send_cmd(char* sendcmd, char* checkbuf, uint16_t waitsec, uint8_t delay_mode );
uint8_t air4g_send_cmd_mqtt(char* sendcmd, char* checkbuf, char* checkbu2, uint16_t waitsec, uint8_t delay_mode );

void air_4g_init(uint8_t connectMode, uint8_t delay_way);
uint8_t air_4g_openURC(uint8_t delay_way);
uint8_t air_4g_closeURC(uint8_t delay_way);
uint8_t air_4g_restar(uint8_t delay_way);
uint8_t air_4g_net_check(uint8_t delay_way);
uint8_t air_4g_connect_server(uint8_t connectMode, uint8_t delay_way);
void air4gRecCheck(void);
uint8_t lbsInfoRead(uint8_t delay_mode);
uint16_t BLDistanceDiff(uint8_t *Lon_nowstr1, uint8_t *Lat_nowstr1, uint8_t *Lon_nowstr2, uint8_t *Lat_nowstr2);

void ble_swit_on(uint8_t delay_way);
void ble_swit_off(uint8_t delay_way);

void air_4g_MPUB(uint8_t car_state);
void air_4g_MPUB_event(uint8_t eventid);
void air_4g_OTAMPUB(uint8_t delay_way);
void air_4g_OTAREQUEST(void);
void air_4g_OTASUB(uint8_t delay_way);
void air_4g_http_otarequest(void);

time_t StringToTimeStamp(uint8_t* timeStr);
char *TimeStampToString(time_t* timeStamp);
void get_real_time(uint8_t delay_way);
void get_4G_msg(uint8_t delay_way);
void gpsReset(uint8_t delay_way);
void topic_sub(uint8_t delay_way);

void usart1_sbus_tx(void);

void mpu_msg_tx(void);

void calculateChecksum(MPU_MSG_PACK* packet);
// δʹ��


//void gps_swit_on(bool delay_way);           // ��GPS
//void gps_swit_off(bool delay_way);          // �ر�GPS



//void air_4g_init(bool delay_way);
//void Task_4G(void);
//uint8_t air_4g_restar(bool delay_way);
#endif
