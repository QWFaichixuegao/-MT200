#ifndef __CAN_DEVICE_H_
#define __CAN_DEVICE_H_

#include "can.h"
#include "time.h"



/*********************************CAN�����������**************************************/


//uint32_t canDeviceOnLineState;		// CAN����������״̬
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


/*********************************QC105���**************************************/

// SPB21-SW20-008-A01���ͨѶ����
#define BAT_STAR_StdId						0x0001
#define BAT_DATA_StdId						0x0002
#define BAT_OVER_StdId						0x0003
static uint8_t  CAN_TX_Data1[8]         = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};  // ��һ֡����
static uint8_t  CAN_TX_Data2_current[8] = {0xEA,0xD1,0x01,0x04,0xFF,0x03,0xF8,0xF5};	 // �ڶ�֡��ص���ָ��
static uint8_t  CAN_TX_Data2_energy[8]  = {0xEA,0xD1,0x01,0x04,0xFF,0x04,0xFF,0xF5};  // �ڶ�֡��ص���ָ��
static uint8_t  CAN_TX_Data3[8]         = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};  // ����֡����

// ���SPB21-SW20-008-A01��ȡ״̬
typedef enum
{
     BAT_ENERGY			= 0,
     BAT_CURRENT 		= 1,
     BAT_VOLTAGE  		= 2,
} BAT_READ_STATE;


/*************************************S48100���***********************************************/

// S48100���ͨѶЭ�鶨��
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




/************************************��ػ�������***********************************************/

// �͵�������
#define BatteryLowSOC_Level1    25		//25
#define BatteryLowSOC_Level2    10
#define BatteryHighSOC		    80

// �������  0.QC105		1.S48100
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

// �������
typedef struct {

    uint64_t 	timestamp;          			// ��ʱ��ʱ�����ʽ��uint64_t
    // �̶�����
    uint8_t		battType;						// �������
    char		battTypeName[16];				// �����������
    char  		battHarewareVersion[16];		// ���Ӳ���汾�ţ���λ�ޣ�INT32��ȡֵ������
    char  		battSoftwareVersion[16];		// �������汾�ţ�
    char		battSN[16];						// ���SN��


    // QC105ͨѶ����
    uint8_t  	cur_count;						// ��ض�ȡ���ݰ��л�
    uint8_t  	energy_rx_count;		    	// ����֡����
    uint8_t  	energy_rx_flag;					// ����֡����

    // ����¶�
    uint8_t		battTempPoint;					// �¶Ȳɼ�������
    int16_t 	batt_temp[8]; 					// ����¶ȣ�IOT��λ1���϶ȣ�ȡֵ��Χ-100~100���ϵ��ֻ��ǰ4λ��0-1�ǵ�о0-1�¶ȣ�2��MOS�¶ȣ�3��MOS�¶ȣ��µ����8λ��0-7�ǵ�о0-7���¶�


    // �����ز���
    uint8_t 	charge_state;       			// ��������ŵ�״̬
    uint8_t 	charge_flag;                    // ��س���־λ
    uint8_t 	charge_overflag;                // ��س�������־λ
    uint16_t 	spentChargeTime;    			// �Ѿ����ʱ�䣨���ӣ���IOT�޵�λ��INT32��ȡֵΪ����
    uint16_t 	circle;             			// ѭ��������IOT�޵�λ��INT32��ȡֵΪ����

    uint16_t 	chargeTimeRemain;   			// ���ʣ��ʱ�䣨���ӣ���IOT�޵�λ��INT32��ȡֵΪ����
    uint16_t 	dischargeTimeRemain;   		    // �ŵ�ʣ��ʱ�䣨���ӣ���IOT�޵�λ��INT32��ȡֵΪ����
    uint8_t  	lastChargeTime[16];      	    // ���һ�γ��ʱ�䣨ʱ���ʽ����IOT��λUTCʱ���

    // ��ѹ��������
    uint8_t 	voitageCurrentH;				// ��ص�ǰ��ѹ���ֽ�
    uint16_t 	voitageCurrent;					// ��ص�ǰ��ѹ��IOT��λmV��INT32��ȡֵ������
    uint16_t 	cellVoitageMax;					// �����ߵ�о��ѹ��IOT��λmV��INT32��ȡֵ������
    uint16_t 	cellVoitageMin;					// �����͵�о��ѹ��IOT��λmV��INT32��ȡֵ������
    uint8_t 	currentCurrentH;				// ��ص�ǰ�������ֽ�
    int16_t 	currentCurrent;					// ��ص�ǰ������IOT��λmA��INT32��ȡֵ�����ƣ�

    // �����ͽ����Ȳ���
    uint8_t  	soc;							// ��ص���ʣ��ٷֱȣ�IOT��λ�ޣ�INT32��ȡֵ0-100��
    uint32_t 	capDesignH;						// ������������
    uint32_t 	capDesign;						// ������������IOT�޵�λmAH��double��ȡֵΪ����
    uint32_t 	capOverall;						// �����������IOT�޵�λmAH��double��ȡֵΪ����
    uint32_t 	capRemain;						// ��ص�ǰ��ʣ������mAH��IOT�޵�λ��double��ȡֵΪ����
    uint8_t  	soh;							// ��ؽ����ٷֱȣ�IOT��λ�ޣ�INT32��ȡֵ0-100��

    // ���״̬
    uint16_t    protect_state;					// ��ر���״̬��IOT��λ�ޣ�INT32��
    uint16_t	warningState;					// ��ظ澯״̬��IOT��λ�ޣ�INT32��
    uint16_t	faultState;						// ��ع���״̬��
    uint16_t	bmsState;						// BMS״̬
    uint16_t    cellEquilibriumState;			// ��о����״̬
    uint8_t		socState;						// SOC״̬


}BATTERY_DATA;
extern BATTERY_DATA battery_data;




/**********************************���������**************************************************/

// ���ͨ�ű�ʶ��
#define MOTO_Control_ID			0X609	// ���ͨ�ſ���ID 11bit ����λ������+����λ�豸ID 1100 0001001
#define MOTO_ReadREQ_ID			0X209	// ���ͨ�Ŷ�ȡ����ID
#define MOTO_ReadACK_ID			0X189	// ���ͨ�Ŷ�ȡӦ��ID
#define MOTO_DLC				0X08	// ���ͨ�����ݳ���

// ���ָ��Ĵ�����ַ
#define MOTO_STATE				0X79	// ���״̬�Ĵ���
#define MOTO_RESET_STATE		0XA0	// ���״̬��λ�Ĵ���

#define MOTO_SysBitM            0X0A    // ϵͳλ��������

#define MOTO_TEMP1				0X92	// ������Ԫ1�¶�
#define MOTO_TEMP2				0X93	// ������Ԫ2�¶�
#define MOTO_TEMP3				0X94	// ������Ԫ3�¶�
#define MOTO_TEMP4				0X95	// ������Ԫ4�¶�

// �������ָ��
#define MOTO1_Velocity			0X5A	// ���1�ٶ�ָ��Ĵ���
#define MOTO1_Launch			0XA4	// ���1����ָ��Ĵ���
#define MOTO1_HALL				0X8A	// ���1������ȡ�Ĵ���
#define MOTO2_Velocity 			0X5B
#define MOTO2_Launch 			0XA5
#define MOTO2_HALL				0X8C

//#define MOTO1_HALL_speed		0X84	// ���1�����ٶȶ�ȡ


//#define MOTO2_HALL_speed		0X85
#define MOTO_REGIS_NUM1 		0X01    // ���β����ļĴ�����
#define MOTO_REGIS_NUM2 		0X02
#define MOTO_auto_ask			0XA7

// �������������
typedef struct {
    uint32_t   	Hall_new;			    // ��ȡ���ĵ�ǰ����λ��
    uint32_t   	Hall_old;			    // ��ȡ�����ϴλ���λ��
    uint32_t   	Hall_add;			    // ����λ��������δʹ��
    int16_t  	  l_rate;		        // ���ݻ���λ�ò������Ļ�����ֵ
    uint16_t  	Speed;
    uint16_t  	moto_state;		        // ���������ϵͳ״̬��2
    int16_t     moto_temp1;		        // ɢ��Ƭ����1�¶�ֵ
    int16_t     moto_temp2;		        // ɢ��Ƭ����2�¶�ֵ
    int16_t     moto_temp3;		        // ɢ��Ƭ����3�¶�ֵ
    int16_t     moto_temp4;		        // ɢ��Ƭ����4�¶�ֵ

    uint8_t 		Cmd_addr;
}CAN_READ_DATA;
extern CAN_READ_DATA can_read_data;


/***********************************��������ģ��*******************************************/

// ��������ģ��ͨѶЭ�鶨��
#define SPEAK_INDEX_ITEM						0x2001
#define SPEAK_SUB_INDEX_ITEM					0x00

#define SPEAK_ITEM_OPEN							0x01		// ����
#define SPEAK_ITEM_CLOSE						0x00		// �ػ�
#define SPEAK_ITEM_POWER_CONNECT				0x03		// ��Դ������
#define SPEAK_ITEM_BIND_SECCEEED				0x0C		// �󶨳ɹ�
#define SPEAK_ITEM_BIND_FAIL					0x0D		// ��ʧ��
#define SPEAK_ITEM_SBUS_LOCK					0x0F		// ������
#define SPEAK_ITEM_SBUS_UNLOCK					0x0E		// �ѽ���
#define SPEAK_ITEM_CHARGE_REMIND				0x11		// �뼰ʱ���
#define SPEAK_ITEM_WORK_BAN						0x12		// ��������
#define SPEAK_ITEM_WORKTIME_REMIND				0x13		// ��ע����ҵʱ��
#define SPEAK_ITEM_SOC(x)						0x80|x		// ����ʣ��[]%

/***********************************ָʾ��ģ��*******************************************/

// ָʾ��ģ��ͨѶЭ�鶨��

#define INDECATOR_LIGHT_INDEX_ITEM				0x2001
#define INDECATOR_LIGHT_SUB_INDEX_ITEM			0x00

// rear indicator light ��ָʾ��
#define INDECATOR_LIGHT_ITEM_CLOSE				0x00		// �ػ�
#define INDECATOR_LIGHT_ITEM_OPEN				0x01		// ����
#define INDECATOR_LIGHT_ITEM_UNLOCK				0x02		// ����
//#define SPEAK_ITEM_LOCK						0x01		// ����
#define INDECATOR_LIGHT_ITEM_MANUAL				0x03		// �ֶ�ģʽ
#define INDECATOR_LIGHT_ITEM_AUTO				0x11		// ����ģʽ
#define INDECATOR_LIGHT_ITEM_GEAR_1				0x16		// ����һ��
#define INDECATOR_LIGHT_ITEM_GEAR_2				0x17		// ���ٶ���
#define INDECATOR_LIGHT_ITEM_GEAR_3				0x18		// ��������
#define INDECATOR_LIGHT_ITEM_RECHARGE			0x50		// ���
#define INDECATOR_LIGHT_ITEM_SOC_LOW_LEVEL_1	0x60		// �͵���һ������
#define INDECATOR_LIGHT_ITEM_SOC_LOW_LEVEL_2	0x61		// �͵�����������
#define INDECATOR_LIGHT_ITEM_SCRAMSTOP			0x62		// ��ͣ
#define INDECATOR_LIGHT_ITEM_FAULT				0x63		// ����


//extern uint8_t INDECATOR_LIGHT_IDLE;						// ͬ��֡��־λ
//extern uint16_t INDECATOR_LIGHT_IDLE_NUM;
/***********************************�������������*******************************************/

// ������ͨѶЭ�鶨��

#define DRIVER_SUB_INDEX_DEFAULT				0x00
#define DRIVER_INDEX_CAR_LIGHT					0x2002
#define DRIVER_INDEX_CAR_FAN_SPEED				0x2007
#define DRIVER_INDEX_BOX_FAN_SPEED				0x2008
#define DRIVER_INDEX_DRAUGHT_FAN_SPEED			0x2009
#define DRIVER_INDEX_PUMP_SPEED					0x200A
#define DRIVER_INDEX_CONTACTOE					0x200D


/***********************************���ز�������*******************************************/




//char softWareName[16];


/*********************************CANͨѶ�����ṹ�嶨��*************************************/

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

/***********************************���϶���***********************************************/

// ������״̬����
//typedef struct {
//	 uint8_t  driver_RxData[8];
//	 uint8_t  device_id;
//	 int8_t  	boxPT100_1;			//###�����ֵ��滻###
//	 int8_t  	boxPT100_2;			//###�����ֵ��滻###
//	 int8_t  	temp_mcu;			//###�����ֵ��滻###
//	 int8_t  	temp_6050;			//###�����ֵ��滻###
//	 int8_t  	pitch;				//###�����ֵ��滻###
//	 int8_t  	roll;				//###�����ֵ��滻###
//	 int8_t  	yaw;				//###�����ֵ��滻###
//	 int8_t  	humiture_wendu1;	//###�����ֵ��滻###
//	 int8_t  	humiture_shidu1;	//###�����ֵ��滻###
//	 int8_t  	humiture_wendu2;	//###�����ֵ��滻###
//	 int8_t  	humiture_shidu2;	//###�����ֵ��滻###
//}DRIVER_SENSOR;
//extern DRIVER_SENSOR driver_sensor;

//void read_driver_sensor(void);		//CANopenȡ��
//void driver_sensor_ack_handle(void);	//CANopenȡ��



/***********************************��������***********************************************/
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

