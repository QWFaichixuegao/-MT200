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


#define SETBIT(x,y) x|=(1<<y)  			//��X�ĵ�Yλ��1
#define CLRBIT(x,y) x&=~(1<<y) 			//��X�ĵ�Yλ��0

/* enum:ö������ */
typedef enum
{
    FALSE = 0,
    TRUE  = !FALSE
}
bool;

/*********************************************������������*************************************************/

// �豸��Ϣ�̻�����
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
    uint8_t upgrade_flag;						//������־λ
} DEVICE_INFORM;
extern DEVICE_INFORM device_inform;

// ����״̬
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

// ���Ʋ�������
typedef struct
{

	  uint16_t 		cTimer_ms_60000;
		uint16_t 		realTimer_ms_1000;   				//	ʵʱʱ���ʱ��־
    uint16_t 		sTimer_ms_60000;
    uint16_t 		Sbus_connect_flag;       		// ң�������ӱ�־λ

    // ����״̬
    uint8_t 		Car_State;                  // ����״̬
    uint8_t 		Init_flag;                  // ������ʼ����־λ

    // ������
    uint8_t 		Bound_flag;                 // ң�����󶨱�־λ
    uint8_t 		Match_flag;	                // ң����ƥ���־λ

    // ң����
    uint8_t 		Sbus_lock_flag;             // ң����������־
    //uint8_t 		Sbus_midlock_flag;		    // ң����E����λ��־



    // Ȩ�޿���
    //uint8_t 		Highest_autho_flag;         //

    // ��ʻ�켣��¼
    //uint8_t 		count_lock;                 //
    uint8_t 		count_turn;                 //
    uint8_t 		save_turn_flag;             //


    // ��������
    uint8_t 		main_kaihe_flag;            //
    uint8_t			resetSource;				// ��ȡ��λԴ


    // �����豸����
    uint8_t 		Led48v_swith;               // ��ƿ���
    uint8_t 		Boxfan_swith;               // ������ȿ���
    uint8_t 		Carfan_swith;               // ������ȿ���
    //uint8_t 		Auto_turnled_flag;	        // �Զ�����λ�仯��־�������л�ָʾ����ʾ״̬
    //uint8_t 		Ledsor_model;               // ��������ģʽ��־λ��δʹ��

    // ������ҵ
    uint8_t 		Draught_swith;              // ������أ����������ң������
    uint8_t 		Draught_open_flag;          // �����ͣ���أ�δʹ�ã���λ����ʱʹ�ã�

    uint8_t 		Pump_swith;                 // ˮ�õ�λ���أ����������ң������

    uint8_t 		DraughtPumpEnable;		    // ���ˮ�ý��ñ�־���͵���ʹ��

    uint8_t 		Auto_spray_swith;	        // �����Զ���ͣ����
    uint8_t 		speed_notific_flag;	        // �����Զ���ͣ�ٶȸ�֪��־
    uint8_t 		lock_check_flag;	        // ������ҵ��������������־
		uint8_t 		lock_check_again;	        // ������ҵ�����ؽ�����־

    //uint8_t 		DraughtPumpEnableRecover;   // ˮ�÷�����ûָ�

    // �˶�����
    uint8_t 		up_gear_flag;               // ����ģʽ������־
    uint8_t 		down_gear_flag;             // ����ģʽ������־
    uint8_t 		Auto_swith;                 // ����ģʽ����
    uint8_t 		Auto_gear_swith;            // ����ģʽ��λѡ��


    // 4Gģ��
    uint8_t 		Usart3_handle_flag;
    uint8_t 		event_mpub_mutex;
    uint8_t 		event_mpub_single_flag;     // ʱ���ϱ��ź�


    // ��ض�ȡ����
    uint8_t 		bat_read_flag;


    // ���Ź�����
    uint8_t 		Iwdg_count;

    // ��������ʱ��Ƭ��ʱ��Task2
    uint8_t 		cTimer_ms_100;
    uint8_t 		cTimer_ms_1000;
    uint8_t 		cTimer_ms_3000;
    uint8_t 		cTimer_ms_5000;

    // �ϱ�IOTʱ��Ƭ��ʱ��Task1
    uint8_t 		sTimer_ms_100;
    uint8_t 		sTimer_ms_500;
    uint8_t 		sTimer_ms_1000;
    uint8_t 		sTimer_ms_5000;
    uint8_t 		sTimer_ms_10000;
    uint8_t 		sTimer_ms_12000;



//	uint8_t 		scram_stop_flag;          //###�����ֵ��滻###							//��ͣ��־λ
//	uint8_t 		Soft_sw_flag;			  //###�����ֵ��滻###
//	uint8_t 		driver_kaihe_flag;        //###�����ֵ��滻###

		uint8_t 		Auto_backflag1;            	// ����ģʽ�㵵�������˱�־1
		uint8_t 		Auto_backflag2;            	// ����ģʽ�㵵�������˱�־2

    uint8_t         error_res_count;            // �ϱ���������ۼ�
    uint8_t         error_res_flag;             // �ϱ�����λ��־
    uint8_t         restTask_flag;              // ��λ�����־
}CONTROL_FLAG;
extern CONTROL_FLAG control_flag;


/*********************************************������������*************************************************/
typedef struct
{
    int32_t		totalRunTime;					//������ʱ��
    int32_t		totalMileage;					//�����
    int32_t  	totalWaterPumpTime;				//ˮ��������ʱ��
    int32_t  	totalFanMachineryTime;			//���������ʱ��

} SUMMARY_DATA;
extern SUMMARY_DATA summary_data;

typedef struct
{
    uint32_t    countRunTime;					//������ʱ��
    uint32_t    countWaterPumpTime;				//ˮ��������ʱ��
    uint32_t    countFanMachineryTime;			//���������ʱ��

} COUNT_TIME;
extern COUNT_TIME count_time;


/*********************************************�����˶����Ʋ���*************************************************/

// ң��������ö��
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

/****************ң�������˺���ťλ�ö���*******************/


#define SBUS_SIZE 				30 							// sbus�������ݳ���

#define SBUS_VAR  				1440 						// ң����SBUS�ź�����
#define SBUS_HALFVAR  			720 						// ң����SBUS�źŰ�����

#define SBUS_up_limit  			SBUS_MAX - SBUS_pianyi
#define SBUS_lw_limit  			SBUS_MIN + SBUS_pianyi

#define SBUS_mid_L_limit  	    SBUS_zhongzhi - SBUS_pianyi
#define SBUS_mid_R_limit  	    SBUS_zhongzhi + SBUS_pianyi


#define GEAR1_SPEED  			250  						// �Զ�1���ٶ�
#define GEAR2_SPEED  			500  						// �Զ�2���ٶ�
#define GEAR3_SPEED  			700 						// �Զ�3���ٶ�
#define TURN_VAR  				0.4   					    // ԭ��ת�ٶȱ���
#define ROL_VAR  				13 							// 10000��720=13.88 ��ֵ9360 ������ٷ�ֵ10000 ��λ0.01% ������93.60%���ٶ�(3000*93.60%)

#define SBUS_MAX  				1713                        // ��ʾ���ֵ
#define SBUS_MIN  				273                         // ��ʾ��Сֵ
#define SBUS_zhongzhi  			993                         // ��ʾ�м�ֵ
#define SBUS_siqu  				200                         // ԭ��ת׶�δ�С   �����׶�δ�СΪ 720 - SBUS_siqu
#define SBUS_pianyi  	 		50                          // ƫ�ƴ�С
#define SBUS_pianyi2  	        300                         // �������ص�ҡ��ƫ�ƴ�С

#define SBUS_up_onofflimit  			SBUS_MAX - SBUS_pianyi2      // 1722-300=1422
#define SBUS_lw_onofflimit  			SBUS_MIN + SBUS_pianyi2      // 282 +300=582

#define WATER_PUMP_DEAD         275                         //ˮ����Чֵ����

// ң����UARTͨѶ�ṹ��
typedef struct {
	uint16_t    rx_count;
	uint8_t     rx_sbus_flag;
	uint8_t     rx_buf[SBUS_SIZE];
	uint8_t 	rx_len;
	uint8_t		cro_sensitiv;
}USARTX_SBUS;
extern USARTX_SBUS usart6_sbus;


// �Զ�ģʽ�³��ٵ�λ
typedef enum
{
    G_0		=0,
    G_1     =1,
    G_2     =2,
    G_3     =3,
} CAR_GEAR;

// ���ת���ṹ��
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


/*************************************************�������м�¼****************************************************/

#define SING_TRACK_MAX			90			//�������м�¼���洢90����

typedef struct {
uint32_t    mileage;
uint32_t    start_drug;
uint32_t    drug;
uint32_t    coverbuf;               //��λ
uint8_t     recordID[64];
uint8_t     start_date[16];         // ��ʼ����
uint8_t	    end_date[16];           // ��������
uint8_t	    latitude[16];
uint8_t	    longitude[16];
uint8_t	    start_power;
uint8_t	    power;
uint8_t	    send_count;
uint8_t	    save_track_count;
char  	    trackData[2048];	    // �켣����
}SING_WORK_EVENT;
extern SING_WORK_EVENT sing_work_event;


/********************************************��������*************************************************/

void getSysReseySource(void);               // ��ȡ���������ĸ�λԴ
void control_init(void);                    // ���Ʋ�����ʼ��
void device_init(void);                     // �豸���ݳ�ʼ��
void subus_read(void);                      // ң�������ݽ���
void Boxfan_control(void);                  // ������ȿ���
void Carfan_control(void);                  // ������ȿ���
void car_state_trans(void);                 // ��������״̬��
void hunkong(uint16_t X2, uint16_t Y2);     // �ֶ�����
void soft_sw_clickON(void);                 // ���ش�����
void soft_sw_clickOFF(void);                // ���عر�����



#ifdef __cplusplus
}
#endif












































/********************************************���϶���*************************************************/
void SPEAK(char* conten);

typedef struct {
char buf_spk[30];

}YY_HANDLE;
extern YY_HANDLE yy_module;

#define DRIVERID						0x700 //������ID��0x600��Ϊ��0x700
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


