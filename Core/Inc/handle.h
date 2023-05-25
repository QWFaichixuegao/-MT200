#ifndef __HANDLE_H__
#define __HANDLE_H__

#ifdef __cplusplus
extern "C" {
#endif
#include "main.h"
#include "math.h"
#include "string.h"
#include "stdio.h"
#include "stdlib.h"
#include "stm32f4xx_hal.h"
#include "usart.h"


#define SETBIT(x,y) x|=(1<<y)  			//将X的第Y位置1
#define CLRBIT(x,y) x&=~(1<<y) 			//将X的第Y位清0

/* enum:枚举类型 */
typedef enum
{
    FALSE = 0,
    TRUE  = !FALSE
}
bool;

/*********************************************整车控制数据*************************************************/

// 设备信息固化参数
typedef struct
{
    char AtStrBuf[384];
    char Password[128];
    char server_host[64];
    char ProductKey[32];
    char DeviceName[32];
    char timestamp[16];
    char server_port[8];
    char version[8];
    char RunTime[4];
    char Mileage[4];
    char WaterPumpTime[4];
    char FanMachineryTime[4];
    uint8_t upgrade_flag;						//升级标志位
} DEVICE_INFORM;
extern DEVICE_INFORM device_inform;

// 整车状态
typedef enum
{
    CLOSE			   	= 0x00,
    OPEN 			    = 0x01,
    RUN  				= 0x02,
    RECHARGE     		= 0x03,
    FAULT      			= 0x04,

    //MANUAL_RUNNING 	= 0x03,
    //AUTO_RUNNING		= 0x11,
    //BOUND_SUCC		= 0x12,
    //BOUND_FAIL        = 0x13,
    //UNLOCK			= 0x14,
    //LOCKC				= 0x15,
    //GEAR_1			= 0x16,
    //GEAR_2			= 0x17,
    //GEAR_3			= 0x18,
} CAR_STATE;

// 控制参数定义
typedef struct
{

	  uint16_t 		cTimer_ms_60000;
		uint16_t 		realTimer_ms_1000;   				//	实时时间计时标志
    uint16_t 		sTimer_ms_60000;
    uint16_t 		Sbus_connect_flag;       		// 遥控器连接标志位

    // 车辆状态
    uint8_t 		Car_State;                  // 车辆状态
    uint8_t 		Init_flag;                  // 整车初始化标志位

    // 车辆绑定
    uint8_t 		Bound_flag;                 // 遥控器绑定标志位
    uint8_t 		Match_flag;	                // 遥控器匹配标志位

    // 遥控器
    uint8_t 		Sbus_lock_flag;             // 遥控器解锁标志
    //uint8_t 		Sbus_midlock_flag;		    // 遥控器E键中位标志



    // 权限控制
    //uint8_t 		Highest_autho_flag;         //

    // 行驶轨迹记录
    //uint8_t 		count_lock;                 //
    uint8_t 		count_turn;                 //
    uint8_t 		save_turn_flag;             //


    // 调试数据
    uint8_t 		main_kaihe_flag;            //
    uint8_t			resetSource;				// 获取复位源


    // 车身设备控制
    uint8_t 		Led48v_swith;               // 大灯开关
    uint8_t 		Boxfan_swith;               // 箱体风扇开关
    uint8_t 		Carfan_swith;               // 车体风扇开关
    //uint8_t 		Auto_turnled_flag;	        // 自动档档位变化标志，用来切换指示灯显示状态
    //uint8_t 		Ledsor_model;               // 传感器灯模式标志位，未使用

    // 喷洒作业
    uint8_t 		Draught_swith;              // 风机开关（解锁后解析遥控器）
    uint8_t 		Draught_open_flag;          // 风机启停开关（未使用，复位启动时使用）

    uint8_t 		Pump_swith;                 // 水泵档位开关（解锁后解析遥控器）

    uint8_t 		DraughtPumpEnable;		    // 风机水泵禁用标志，低电量使用

    uint8_t 		Auto_spray_swith;	        // 喷撒自动启停开关
    uint8_t 		speed_notific_flag;	        // 喷洒自动启停速度感知标志
    uint8_t 		lock_check_flag;	        // 喷洒作业开机解锁键检查标志
		uint8_t 		lock_check_again;	        // 喷洒作业开机重解锁标志

    //uint8_t 		DraughtPumpEnableRecover;   // 水泵风机禁用恢复

    // 运动控制
    uint8_t 		up_gear_flag;               // 定速模式升档标志
    uint8_t 		down_gear_flag;             // 定速模式降档标志
    uint8_t 		Auto_swith;                 // 定速模式开关
    uint8_t 		Auto_gear_swith;            // 定速模式档位选择


    // 4G模块
    uint8_t 		Usart3_handle_flag;
    uint8_t 		event_mpub_mutex;
    uint8_t 		event_mpub_single_flag;     // 时间上报信号


    // 电池读取控制
    uint8_t 		bat_read_flag;


    // 看门狗计数
    uint8_t 		Iwdg_count;

    // 车辆控制时间片计时，Task2
    uint8_t 		cTimer_ms_100;
    uint8_t 		cTimer_ms_1000;
    uint8_t 		cTimer_ms_3000;
    uint8_t 		cTimer_ms_5000;

    // 上报IOT时间片计时，Task1
    uint8_t 		sTimer_ms_100;
    uint8_t 		sTimer_ms_500;
    uint8_t 		sTimer_ms_1000;
    uint8_t 		sTimer_ms_5000;
    uint8_t 		sTimer_ms_10000;
    uint8_t 		sTimer_ms_12000;



//	uint8_t 		scram_stop_flag;          //###对象字典替换###							//急停标志位
//	uint8_t 		Soft_sw_flag;			  //###对象字典替换###
//	uint8_t 		driver_kaihe_flag;        //###对象字典替换###

		uint8_t 		Auto_backflag1;            	// 定速模式零档比例后退标志1
		uint8_t 		Auto_backflag2;            	// 定速模式零档比例后退标志2

    uint8_t         error_res_count;            // 上报出错次数累计
    uint8_t         error_res_flag;             // 上报出错复位标志
    uint8_t         restTask_flag;              // 复位任务标志
}CONTROL_FLAG;
extern CONTROL_FLAG control_flag;


/*********************************************车辆运行数据*************************************************/
typedef struct
{
    int32_t		totalRunTime;					//总运行时长
    int32_t		totalMileage;					//总里程
    int32_t  	totalWaterPumpTime;				//水泵运行总时长
    int32_t  	totalFanMachineryTime;			//风机运行总时长

} SUMMARY_DATA;
extern SUMMARY_DATA summary_data;

typedef struct
{
    uint32_t    countRunTime;					//总运行时长
    uint32_t    countWaterPumpTime;				//水泵运行总时长
    uint32_t    countFanMachineryTime;			//风机运行总时长

} COUNT_TIME;
extern COUNT_TIME count_time;


/*********************************************车辆运动控制参数*************************************************/

// 遥控器按键枚举
typedef enum
{
    SBUS_X2		=1,
    SBUS_Y2  	=2,
    SBUS_Y1  	=3,
    SBUS_X1     =4,
    SBUS_E      =5,
    SBUS_F      =6,
    SBUS_A      =7,
    SBUS_B      =8,
    SBUS_C      =9,
    SBUS_D     	=10,
    SBUS_G     	=11,
    SBUS_H     	=12,

} REMOTE_SBUS;

/****************遥控器拨杆和旋钮位置定义*******************/


#define SBUS_SIZE 				30 							// sbus串口数据长度

#define SBUS_VAR  				1440 						// 遥控器SBUS信号量程
#define SBUS_HALFVAR  			720 						// 遥控器SBUS信号半量程

#define SBUS_up_limit  			SBUS_MAX - SBUS_pianyi
#define SBUS_lw_limit  			SBUS_MIN + SBUS_pianyi

#define SBUS_mid_L_limit  	    SBUS_zhongzhi - SBUS_pianyi
#define SBUS_mid_R_limit  	    SBUS_zhongzhi + SBUS_pianyi


#define GEAR1_SPEED  			250  						// 自动1档速度
#define GEAR2_SPEED  			500  						// 自动2档速度
#define GEAR3_SPEED  			700 						// 自动3档速度
#define TURN_VAR  				0.4   					    // 原地转速度比例
#define ROL_VAR  				13 							// 10000÷720=13.88 峰值9360 电机调速峰值10000 单位0.01% 即拉满93.60%的速度(3000*93.60%)

#define SBUS_MAX  				1713                        // 表示最大值
#define SBUS_MIN  				273                         // 表示最小值
#define SBUS_zhongzhi  			993                         // 表示中间值
#define SBUS_siqu  				200                         // 原地转锥形大小   则差速锥形大小为 720 - SBUS_siqu
#define SBUS_pianyi  	 		50                          // 偏移大小
#define SBUS_pianyi2  	        300                         // 用作开关的摇杆偏移大小

#define SBUS_up_onofflimit  			SBUS_MAX - SBUS_pianyi2      // 1722-300=1422
#define SBUS_lw_onofflimit  			SBUS_MIN + SBUS_pianyi2      // 282 +300=582

#define WATER_PUMP_DEAD         275                         //水泵有效值死区

// 遥控器UART通讯结构体
typedef struct {
	uint16_t    rx_count;
	uint8_t     rx_sbus_flag;
	uint8_t     rx_buf[SBUS_SIZE];
	uint8_t 	rx_len;
	uint8_t		cro_sensitiv;
}USARTX_SBUS;
extern USARTX_SBUS usart6_sbus;


// 自动模式下车速档位
typedef enum
{
    G_0		=0,
    G_1     =1,
    G_2     =2,
    G_3     =3,
} CAR_GEAR;

// 电机转动结构体
typedef struct
{
    int16_t 	x;
    int16_t 	y;
    int16_t  	car_left;
    int16_t  	car_right;
    float 		cos;
    float 	   	sin;
}CAR_MOTO;
extern CAR_MOTO car_moto;


/*************************************************单次运行记录****************************************************/

#define SING_TRACK_MAX			90			//单次运行记录最大存储90个点

typedef struct {
uint32_t    mileage;
uint32_t    start_drug;
uint32_t    drug;
uint32_t    coverbuf;               //补位
uint8_t     recordID[64];
uint8_t     start_date[16];         // 开始数据
uint8_t	    end_date[16];           // 结束数据
uint8_t	    latitude[16];
uint8_t	    longitude[16];
uint8_t	    start_power;
uint8_t	    power;
uint8_t	    send_count;
uint8_t	    save_track_count;
char  	    trackData[2048];	    // 轨迹数据
}SING_WORK_EVENT;
extern SING_WORK_EVENT sing_work_event;


/********************************************函数声明*************************************************/

void getSysReseySource(void);               // 获取本次启动的复位源
void control_init(void);                    // 控制参数初始化
void device_init(void);                     // 设备数据初始化
void subus_read(void);                      // 遥控器数据解析
void Boxfan_control(void);                  // 箱体风扇控制
void Carfan_control(void);                  // 车体风扇控制
void car_state_trans(void);                 // 车辆控制状态机
void hunkong(uint16_t X2, uint16_t Y2);     // 手动控制
void soft_sw_clickON(void);                 // 软开关打开任务
void soft_sw_clickOFF(void);                // 软开关关闭任务



#ifdef __cplusplus
}
#endif












































/********************************************作废定义*************************************************/
void SPEAK(char* conten);

typedef struct {
char buf_spk[30];

}YY_HANDLE;
extern YY_HANDLE yy_module;

#define DRIVERID						0x700 //电驱板ID从0x600改为了0x700
#define SOFTSWID						0x601
#define LEDSERSON1ID				0x602
#define LEDSERSON2ID				0x603
#define YYID								0x604
#define HUMITURE1ID					0x605
#define HUMITURE2ID					0x606

#define DATADLC							0x08

#define SCRAM								0x01
#define BATTERY							0x02
#define LED48V							0x03
#define BOXFAN							0x04
#define CARFAN							0x05
#define TEMP								0x06
#define KAIHE_DRIVER				0x07
#define ANGLE								0x08
#define DRAUGHT							0x10
#define MOTOR								0x11
#define PUMP								0x12
#define LED_SER							0x13
#define SOFT_SW							0x14
#define SPEECH							0x15
#define HUMITURE1						0x16
#define HUMITURE2						0x17
#define ALLDEV							0xFF

#define HEADER  						0xAA
#define CHECK_DATA  					2
#define PARAMS_DATA  					2
#define CHECK_DATA  					2

#endif


