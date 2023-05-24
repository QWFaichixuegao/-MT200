#ifndef __CAN_DEVICE_H_
#define __CAN_DEVICE_H_

#include "can.h"
#include "time.h"



/*********************************CAN传感器板管理**************************************/


//uint32_t canDeviceOnLineState;		// CAN传感器在线状态
#define CANDevice_NodeID_driver						0x01
#define CANDevice_NodeID_softWare					0x02
#define CANDevice_NodeID_speak						0x03
#define CANDevice_NodeID_indecatorLightL			0x07
#define CANDevice_NodeID_indecatorLightR			0x08
#define CANDevice_NodeID_sht30CarBox				0x0A
#define CANDevice_NodeID_sht30Environment			0x0B

#define CANDevice_SUB_INDEX							0x00

#define CANDevice_INDEX_Name						0x1008
#define CANDevice_INDEX_hardwareVersion				0x1009
#define CANDevice_INDEX_softwareVersion				0x100A


/*********************************QC105电池**************************************/

// SPB21-SW20-008-A01电池通讯参数
#define BAT_STAR_StdId						0x0001
#define BAT_DATA_StdId						0x0002
#define BAT_OVER_StdId						0x0003
static uint8_t  CAN_TX_Data1[8]         = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};  // 第一帧数据
static uint8_t  CAN_TX_Data2_current[8] = {0xEA,0xD1,0x01,0x04,0xFF,0x03,0xF8,0xF5};	 // 第二帧电池电流指令
static uint8_t  CAN_TX_Data2_energy[8]  = {0xEA,0xD1,0x01,0x04,0xFF,0x04,0xFF,0xF5};  // 第二帧电池电量指令
static uint8_t  CAN_TX_Data3[8]         = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};  // 第三帧数据

// 电池SPB21-SW20-008-A01读取状态
typedef enum
{
     BAT_ENERGY			= 0,
     BAT_CURRENT 		= 1,
     BAT_VOLTAGE  		= 2,
} BAT_READ_STATE;


/*************************************S48100电池***********************************************/

// S48100电池通讯协议定义
#define S48100_NODE_ID								0x04
#define S48100_INDEX								0x6300

#define S48100_SUB_INDEX_batteryFaultState			0x01
#define S48100_SUB_INDEX_BMSStatus					0x02
#define S48100_SUB_INDEX_batteryCloseControl		0x03
#define S48100_SUB_INDEX_chargeDischarge			0x04
#define S48100_SUB_INDEX_batterySOC					0x05
#define S48100_SUB_INDEX_batterySOH					0x06
#define S48100_SUB_INDEX_capRemain					0x07
#define S48100_SUB_INDEX_cycleIndex					0x08
#define S48100_SUB_INDEX_currentCurrent				0x09
#define S48100_SUB_INDEX_currentVoltage				0x0A
#define S48100_SUB_INDEX_packVoltage				0x0B
#define S48100_SUB_INDEX_cellTempMax				0x0C
#define S48100_SUB_INDEX_cellTempMin				0x0D
#define S48100_SUB_INDEX_cellVoltageMax				0x0E
#define S48100_SUB_INDEX_cellVoltageMin				0x0F
#define S48100_SUB_INDEX_cellVoltageDiffMax			0x10
#define S48100_SUB_INDEX_chargeVoltageAllow			0x11
#define S48100_SUB_INDEX_chargeCurrentAllow			0x12
#define S48100_SUB_INDEX_nominalVoltage				0x13
#define S48100_SUB_INDEX_capDesign					0x14
#define S48100_SUB_INDEX_manufacturer				0x15
#define S48100_SUB_INDEX_seriesParallelNum			0x16
#define S48100_SUB_INDEX_softwareVersion			0x17
#define S48100_SUB_INDEX_hardwareVersion			0x18
#define S48100_SUB_INDEX_batterySN					0x19
#define S48100_SUB_INDEX_cellSN						0x1A
#define S48100_SUB_INDEX_warningState				0x1C
#define S48100_SUB_INDEX_protectState				0x1D
#define S48100_SUB_INDEX_cellEquilibriumState		0x1E
#define S48100_SUB_INDEX_battTempPoint				0x20
#define S48100_SUB_INDEX_battTemp1					0x31
#define S48100_SUB_INDEX_battTemp2					0x32
#define S48100_SUB_INDEX_battTemp3					0x33
#define S48100_SUB_INDEX_battTemp4					0x34




/************************************电池汇总数据***********************************************/

// 低电量提醒
#define BatteryLowSOC_Level1    25		//25
#define BatteryLowSOC_Level2    10
#define BatteryHighSOC		    80

// 电池种类  0.QC105		1.S48100
typedef enum
{
    QC105				= 0,
    S48100				= 1,
} BATTERY_SOC_STATE;


typedef enum
{
    SOC_ENOUGH			= 0,
    SOC_LOW_LEVEL_1 	= 1,
    SOC_LOW_LEVEL_2		= 2,
} BATTERY_TYPE;

// 电池数据
typedef struct {

    uint64_t 	timestamp;          			// 电时间时间戳形式，uint64_t
    // 固定参数
    uint8_t		battType;						// 电池种类
    char		battTypeName[16];				// 电池种类名称
    char  		battHarewareVersion[16];		// 电池硬件版本号；单位无，INT32，取值无限制
    char  		battSoftwareVersion[16];		// 电池软件版本号；
    char		battSN[16];						// 电池SN码


    // QC105通讯参数
    uint8_t  	cur_count;						// 电池读取数据包切换
    uint8_t  	energy_rx_count;		    	// 接收帧计数
    uint8_t  	energy_rx_flag;					// 接收帧计数

    // 电池温度
    uint8_t		battTempPoint;					// 温度采集点数量
    int16_t 	batt_temp[8]; 					// 电池温度；IOT单位1摄氏度，取值范围-100~100；老电池只用前4位：0-1是电芯0-1温度，2是MOS温度，3是MOS温度；新电池用8位：0-7是电芯0-7的温度


    // 充电相关参数
    uint8_t 	charge_state;       			// 电量包充放电状态
    uint8_t 	charge_flag;                    // 电池充电标志位
    uint8_t 	charge_overflag;                // 电池充电结束标志位
    uint16_t 	spentChargeTime;    			// 已经充电时间（分钟）；IOT无单位，INT32，取值为正；
    uint16_t 	circle;             			// 循环次数；IOT无单位，INT32，取值为正；

    uint16_t 	chargeTimeRemain;   			// 充电剩余时间（分钟）；IOT无单位，INT32，取值为正；
    uint16_t 	dischargeTimeRemain;   		    // 放电剩余时间（分钟）；IOT无单位，INT32，取值为正；
    uint8_t  	lastChargeTime[16];      	    // 最后一次充电时间（时间格式），IOT单位UTC时间戳

    // 电压电流参数
    uint8_t 	voitageCurrentH;				// 电池当前电压高字节
    uint16_t 	voitageCurrent;					// 电池当前电压；IOT单位mV，INT32，取值无限制
    uint16_t 	cellVoitageMax;					// 电池最高电芯电压；IOT单位mV，INT32，取值无限制
    uint16_t 	cellVoitageMin;					// 电池最低电芯电压；IOT单位mV，INT32，取值无限制
    uint8_t 	currentCurrentH;				// 电池当前电流高字节
    int16_t 	currentCurrent;					// 电池当前电流；IOT单位mA，INT32，取值无限制；

    // 容量和健康度参数
    uint8_t  	soc;							// 电池电量剩余百分比；IOT单位无，INT32，取值0-100；
    uint32_t 	capDesignH;						// 电池设计容量高
    uint32_t 	capDesign;						// 电池设计容量；IOT无单位mAH，double，取值为正；
    uint32_t 	capOverall;						// 电池满容量；IOT无单位mAH，double，取值为正；
    uint32_t 	capRemain;						// 电池当前的剩余容量mAH；IOT无单位，double，取值为正；
    uint8_t  	soh;							// 电池健康百分比；IOT单位无，INT32，取值0-100；

    // 电池状态
    uint16_t    protect_state;					// 电池保护状态；IOT单位无，INT32，
    uint16_t	warningState;					// 电池告警状态；IOT单位无，INT32，
    uint16_t	faultState;						// 电池故障状态；
    uint16_t	bmsState;						// BMS状态
    uint16_t    cellEquilibriumState;			// 电芯均衡状态
    uint8_t		socState;						// SOC状态


}BATTERY_DATA;
extern BATTERY_DATA battery_data;




/**********************************电机驱动器**************************************************/

// 电机通信标识符
#define MOTO_Control_ID			0X609	// 电机通信控制ID 11bit 高四位功能码+低四位设备ID 1100 0001001
#define MOTO_ReadREQ_ID			0X209	// 电机通信读取请求ID
#define MOTO_ReadACK_ID			0X189	// 电机通信读取应答ID
#define MOTO_DLC				0X08	// 电机通信数据长度

// 电机指令寄存器地址
#define MOTO_STATE				0X79	// 电机状态寄存器
#define MOTO_RESET_STATE		0XA0	// 电机状态复位寄存器

#define MOTO_SysBitM            0X0A    // 系统位掩码设置

#define MOTO_TEMP1				0X92	// 驱动单元1温度
#define MOTO_TEMP2				0X93	// 驱动单元2温度
#define MOTO_TEMP3				0X94	// 驱动单元3温度
#define MOTO_TEMP4				0X95	// 驱动单元4温度

// 电机控制指令
#define MOTO1_Velocity			0X5A	// 电机1速度指令寄存器
#define MOTO1_Launch			0XA4	// 电机1启动指令寄存器
#define MOTO1_HALL				0X8A	// 电机1霍尔读取寄存器
#define MOTO2_Velocity 			0X5B
#define MOTO2_Launch 			0XA5
#define MOTO2_HALL				0X8C

//#define MOTO1_HALL_speed		0X84	// 电机1霍尔速度读取


//#define MOTO2_HALL_speed		0X85
#define MOTO_REGIS_NUM1 		0X01    // 单次操作的寄存器数
#define MOTO_REGIS_NUM2 		0X02
#define MOTO_auto_ask			0XA7

// 电机驱动器数据
typedef struct {
    uint32_t   	Hall_new;			    // 读取到的当前霍尔位置
    uint32_t   	Hall_old;			    // 读取到的上次霍尔位置
    uint32_t   	Hall_add;			    // 霍尔位置增量，未使用
    int16_t  	  l_rate;		        // 根据霍尔位置差计算出的霍尔差值
    uint16_t  	Speed;
    uint16_t  	moto_state;		        // 电机驱动器系统状态字2
    int16_t     moto_temp1;		        // 散热片监测点1温度值
    int16_t     moto_temp2;		        // 散热片监测点2温度值
    int16_t     moto_temp3;		        // 散热片监测点3温度值
    int16_t     moto_temp4;		        // 散热片监测点4温度值

    uint8_t 		Cmd_addr;
}CAN_READ_DATA;
extern CAN_READ_DATA can_read_data;


/***********************************语音播放模块*******************************************/

// 语音播放模块通讯协议定义
#define SPEAK_INDEX_ITEM						0x2001
#define SPEAK_SUB_INDEX_ITEM					0x00

#define SPEAK_ITEM_OPEN							0x01		// 开机
#define SPEAK_ITEM_CLOSE						0x00		// 关机
#define SPEAK_ITEM_POWER_CONNECT				0x03		// 电源已连接
#define SPEAK_ITEM_BIND_SECCEEED				0x0C		// 绑定成功
#define SPEAK_ITEM_BIND_FAIL					0x0D		// 绑定失败
#define SPEAK_ITEM_SBUS_LOCK					0x0F		// 已上锁
#define SPEAK_ITEM_SBUS_UNLOCK					0x0E		// 已解锁
#define SPEAK_ITEM_CHARGE_REMIND				0x11		// 请及时充电
#define SPEAK_ITEM_WORK_BAN						0x12		// 功能受限
#define SPEAK_ITEM_WORKTIME_REMIND				0x13		// 请注意作业时间
#define SPEAK_ITEM_SOC(x)						0x80|x		// 电量剩余[]%

/***********************************指示灯模块*******************************************/

// 指示灯模块通讯协议定义

#define INDECATOR_LIGHT_INDEX_ITEM				0x2001
#define INDECATOR_LIGHT_SUB_INDEX_ITEM			0x00

// rear indicator light 后指示灯
#define INDECATOR_LIGHT_ITEM_CLOSE				0x00		// 关机
#define INDECATOR_LIGHT_ITEM_OPEN				0x01		// 开机
#define INDECATOR_LIGHT_ITEM_UNLOCK				0x02		// 解锁
//#define SPEAK_ITEM_LOCK						0x01		// 上锁
#define INDECATOR_LIGHT_ITEM_MANUAL				0x03		// 手动模式
#define INDECATOR_LIGHT_ITEM_AUTO				0x11		// 定速模式
#define INDECATOR_LIGHT_ITEM_GEAR_1				0x16		// 定速一档
#define INDECATOR_LIGHT_ITEM_GEAR_2				0x17		// 定速二档
#define INDECATOR_LIGHT_ITEM_GEAR_3				0x18		// 定速三档
#define INDECATOR_LIGHT_ITEM_RECHARGE			0x50		// 充电
#define INDECATOR_LIGHT_ITEM_SOC_LOW_LEVEL_1	0x60		// 低电量一级报警
#define INDECATOR_LIGHT_ITEM_SOC_LOW_LEVEL_2	0x61		// 低电量二级报警
#define INDECATOR_LIGHT_ITEM_SCRAMSTOP			0x62		// 急停
#define INDECATOR_LIGHT_ITEM_FAULT				0x63		// 故障


//extern uint8_t INDECATOR_LIGHT_IDLE;						// 同步帧标志位
//extern uint16_t INDECATOR_LIGHT_IDLE_NUM;
/***********************************电驱板参数定义*******************************************/

// 电驱板通讯协议定义

#define DRIVER_SUB_INDEX_DEFAULT				0x00
#define DRIVER_INDEX_CAR_LIGHT					0x2002
#define DRIVER_INDEX_CAR_FAN_SPEED				0x2007
#define DRIVER_INDEX_BOX_FAN_SPEED				0x2008
#define DRIVER_INDEX_DRAUGHT_FAN_SPEED			0x2009
#define DRIVER_INDEX_PUMP_SPEED					0x200A
#define DRIVER_INDEX_CONTACTOE					0x200D


/***********************************软开关参数定义*******************************************/




//char softWareName[16];


/*********************************CAN通讯基础结构体定义*************************************/

#define DATADLC					0x08

typedef struct {
    uint8_t   				TxData[8];
    uint8_t  			  	RxData[8];
    uint32_t			 	TxMailbox;
    //CAN_SEND_CMD 			can_Scmd;
    CAN_TxHeaderTypeDef     TxHeader;
    CAN_RxHeaderTypeDef     RxHeader;

    uint8_t				    RxFlag;

}CAN_HANDLE;
extern CAN_HANDLE can_handle;

/***********************************作废定义***********************************************/

// 电驱板状态数据
//typedef struct {
//	 uint8_t  driver_RxData[8];
//	 uint8_t  device_id;
//	 int8_t  	boxPT100_1;			//###对象字典替换###
//	 int8_t  	boxPT100_2;			//###对象字典替换###
//	 int8_t  	temp_mcu;			//###对象字典替换###
//	 int8_t  	temp_6050;			//###对象字典替换###
//	 int8_t  	pitch;				//###对象字典替换###
//	 int8_t  	roll;				//###对象字典替换###
//	 int8_t  	yaw;				//###对象字典替换###
//	 int8_t  	humiture_wendu1;	//###对象字典替换###
//	 int8_t  	humiture_shidu1;	//###对象字典替换###
//	 int8_t  	humiture_wendu2;	//###对象字典替换###
//	 int8_t  	humiture_shidu2;	//###对象字典替换###
//}DRIVER_SENSOR;
//extern DRIVER_SENSOR driver_sensor;

//void read_driver_sensor(void);		//CANopen取代
//void driver_sensor_ack_handle(void);	//CANopen取代



/***********************************函数声明***********************************************/
void canDeviceVersionRead(void);
void indicatorLight(uint8_t item);
void speakItem(uint8_t item);
void battery_init(void);
void batteryTypeRead(void);
void batteryReadData(void);
void batteryS48100ReadParameter(void);
void batteryS48100PdoDataCopy(void);
void batteryS48100ReadData(void);
void read_energy_data(void);
void read_current_data(void);
void read_moto_data(void);
void moto_ack_handle(void);
void battery_ack_handle(void);
void S48100B_TEMP_CHECK(void);

#endif

