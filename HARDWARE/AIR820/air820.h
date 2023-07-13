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

/*************************************************************串口通信******************************************************************/

#define AIR4GCHECKTIME 		2000
#define RSTSET_COUNT 		5
#define SHORT_MESG 			7		// usart3短消息过滤
//#define LONG_MESG 		150		// usart3长消息过滤
#define LONG_MESG 			850		// usart3长消息过滤   debug



// 4G模块UART通讯基础定义
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


/*************************************************************状态信息上报******************************************************************/

// 作业上报信息类别定义
typedef enum
{
    WORK_EVENT_MAIN    		= 0,
    WORK_EVENT_TRACK		= 1,
    DEFAUT_EVENT    		= 2,
}MPUB_EVENT;

// 车辆显示到APP上的实时状态上报
typedef struct
{
    uint16_t    vehicleStatus;
    uint16_t    runvehicleStatus;	//车辆运行状态  4位5位手动定速切换  8位9位自动启停 自动启停  12位水泵  13位风机  14位原地停止 15位大灯
    uint16_t    vehicleSpeed;
    uint8_t     liquidLevelSensor;
    uint8_t     fanMachinery;
    uint8_t     motorWaterPump;
    uint8_t     start_summary_flag;
    uint8_t     start_charge_flag;          // 开始充电标志，用于区别IOT发送
    uint8_t	    rundata_turn_count;
    char        theme_str[128];
    char        work_event_Main[128];
    char        work_event_Track[128];
    char        PubBuf[2048];				// 由512改为2048
    char        Pub_work_event_Buf[2048];

} MQTT_PUB_INFORM;
extern MQTT_PUB_INFORM mqtt_pub_inform;

// IOT平台上报状态机切换计数目标值
typedef enum
{
    Count_1s	=10,
    Count_5s  	=50,
    Count_10s  	=100,
    Count_12s  	=120,
    Count_60s  	=600,
} TIME_COUNT_MS;

/*************************************************************状态信息上报(数传)******************************************************************/
// 车辆显示到APP上的实时状态上报
typedef struct
{
    char       tx_buf[2048];
} SBUS_PACK;
extern SBUS_PACK sbus_pack_data;

/*************************************************************mpu上位机交互******************************************************************/
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
    int16_t   angular_speed;//正为左 负为右
    uint8_t   airflow;
    uint8_t   pump;
    uint8_t   light;
    uint8_t   checksum;
} MPU_REC_PACK;
#pragma pack()
extern MPU_REC_PACK mpu_rec_pack;



/*************************************************************蓝牙******************************************************************/

// 蓝牙信息
typedef struct
{
    char    dev_send_cmd[244];	        // 244BLE蓝牙单次发送的字节长度
    char 	dev_data_To16[244];	        // 244BLE蓝牙单次发送的字节长度
    char 	dev_bound_data[48];         // 绑定信息[DeviceName]$[ProductKey]
    char 	ble_name[48]; 			    // 搜索蓝牙名MQZN_DeviceName

    uint8_t ble_switch_state;           // 蓝牙当前开关状态
    uint8_t ble_switch;                 // 蓝牙开关设置状态
} BLE_INFORM;
extern BLE_INFORM ble_inform;


/*************************************************************网络连接******************************************************************/
// 蜂窝连接状态
typedef enum
{
    SIM_OK					=0,
    SIM_COMMUNTION_ERR  	=1,
    SIM_CPIN_ERR  		   	=2,
    SIM_CREG_ERR        	=3,
    SIM_CGATT_ERR           =4,
    SIM_CSQ_LOW        		=5,
} SIM_STATE;


// 4G模块状态
typedef struct
{
    uint8_t vTa_delay;
    uint8_t hal_delay;
    uint8_t SIM_flag;
    uint8_t MQTT_flag;
		uint8_t MQTT_flag2;
    uint8_t send_version_flag;
	  uint8_t get_realtime_flag;
	  uint8_t addbuf1;//对齐用
}AIR_4g_FLAG;
extern AIR_4g_FLAG air_4g_flag;


// 连接蜂窝网络和IOT平台次数
typedef struct
{
	  int			sim_CSQ;
    uint8_t sim_ccount;
    uint8_t mqtt_ccount;
		uint8_t mqtt_state;
		uint8_t air820Count;
//	  uint8_t addbuf1;//对齐用


}AIR_4g_CONNECT;
extern AIR_4g_CONNECT air_4g_connect;

// MQTT连接服务器状态
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

// GPS数据解析存储区
typedef struct
{
    uint16_t    distanceDif;
    int         MSL_Altitude;
    uint8_t	    Lon_nowstr[16];
    uint8_t	    Lat_nowstr[16];
	  uint8_t 		gps_signal_flag;	    // GPS信号获取标志位
    uint8_t 		lbs_signal_flag;	    // GPS信号获取标志位
		uint8_t     gpsUrc;                 // GPS当前上报间隔，1和5
    uint8_t     gpsUrcSet;              // GPS上报间隔设置值
    uint8_t     gpsCheckcount;              // 运行状态下GPS计数检查累计
}GPS_HANDLE;
extern GPS_HANDLE gps_info;

// GPS日期和时间解析存储区
typedef struct
{
    uint8_t		Utc_nowstr[16];
		uint32_t  timestamp;

}TIMER_HANDLE1;
extern TIMER_HANDLE1 timer_info;

/*************计算坐标间距离***************/
#define     WGS84       84          // WGS84坐标系（GPS 坐标）
#define     BJ54        54          // 北京54坐标系
#define     XIAN80      80          // 西安80坐标系
#define     ZONEWIDE3   3           // 投影带宽度 3
#define     ZONEWIDE6   6           // 投影带宽度 6

//高斯平面坐标系
typedef struct
{
    double x;
    double y;
    double z;
} CRDCARTESIAN;

//大地坐标系（可以是 北京54坐标系，西安80坐标系，WGS84坐标系（GPS 坐标））
typedef struct
{
    double longitude; //经度
    double latitude; //纬度
    double height; //大地高,可设为0
} CRDGEODETIC;

/*************************************************************OTA*************************************************************/

// 上报IOT平台的OTA信息
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
	uint8_t 	ota_pack[1024];	   			 // 拷贝IOT发送的 OTA 信息
	uint8_t 	ota_http_url[256]; 			 // 获取到下载OTA的http url
	uint8_t		ota_data_md5[64];			 // 获取ota固件数据的MD5校准
	uint8_t		ota_version[8];				 // 获取到可下载的OTA版本号
	uint8_t		ota_data_lenstr[8];			 // 获取到可下载的OTA版本号
	uint8_t		get_version_flag;		     // 获取版本号是否成功
	uint8_t		get_fireware_flag;			 // 获取新固件是否成功
	char        PubBuf[512];
	char 	      ota_http_urlATcmd[512];      // 组合AT指令请求的http url
	char        version_theme_str[64];
	char        request_theme_str[64];
	char        msubOTA_theme_str[64];
} OTA_INFORM;
extern OTA_INFORM ota_inform;

// 期望值消息
typedef struct
{
	char       PubBuf[256];
    char       desired_reply_theme_str[80];
} DESIRED_INFORM;
extern DESIRED_INFORM desired_inform;


/*************************************************************函数声明*************************************************************/

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
// 未使用


//void gps_swit_on(bool delay_way);           // 打开GPS
//void gps_swit_off(bool delay_way);          // 关闭GPS



//void air_4g_init(bool delay_way);
//void Task_4G(void);
//uint8_t air_4g_restar(bool delay_way);
#endif
