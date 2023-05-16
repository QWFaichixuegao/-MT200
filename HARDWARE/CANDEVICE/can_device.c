#include "can_device.h"
#include "cmsis_os.h"
#include "CANopen_Master_M200.h"
#include "canfestival.h"
BATTERY_DATA			 battery_data;
//DRIVER_SENSOR 		 driver_sensor;

extern CO_Data CANopen_Master_M200_Data;

// CAN传感器版本号读取
void canDeviceVersionRead(void)
{
    UNS32 size;
    // 读电驱板版本号
    size = sizeof(driverBoard_softwareVersion);
    ReadSDO(&CANopen_Master_M200_Data, CANDevice_NodeID_driver, CANDevice_INDEX_softwareVersion, CANDevice_SUB_INDEX, uint8, &driverBoard_softwareVersion, &size, 0);

    // 读软开关版本号
    size = sizeof(softSwitch_softwareVersion);
    ReadSDO(&CANopen_Master_M200_Data, CANDevice_NodeID_softWare, CANDevice_INDEX_softwareVersion, CANDevice_SUB_INDEX, uint8, &softSwitch_softwareVersion, &size, 0);

    // 读语音播放器版本号
    size = sizeof(speaker_softwareVersion);
    ReadSDO(&CANopen_Master_M200_Data, CANDevice_NodeID_speak, CANDevice_INDEX_softwareVersion, CANDevice_SUB_INDEX, uint8, &speaker_softwareVersion, &size, 0);

    // 读指示灯左版本号
    size = sizeof(LED12VL_softwareVersion);
    ReadSDO(&CANopen_Master_M200_Data, CANDevice_NodeID_indecatorLightL, CANDevice_INDEX_softwareVersion, CANDevice_SUB_INDEX, uint8, &LED12VL_softwareVersion, &size, 0);

    // 读指示灯右版本号
    size = sizeof(LED12VR_softwareVersion);
    ReadSDO(&CANopen_Master_M200_Data, CANDevice_NodeID_indecatorLightR, CANDevice_INDEX_softwareVersion, CANDevice_SUB_INDEX, uint8, &LED12VR_softwareVersion, &size, 0);

    // 读温湿度车厢版本号
    size = sizeof(SHT30CarBox_softwareVersion);
    ReadSDO(&CANopen_Master_M200_Data, CANDevice_NodeID_sht30CarBox, CANDevice_INDEX_softwareVersion, CANDevice_SUB_INDEX, uint8, &SHT30CarBox_softwareVersion, &size, 0);

    // 读温湿度环境版本号
    size = sizeof(SHT30Environment_softwareVersion);
    ReadSDO(&CANopen_Master_M200_Data, CANDevice_NodeID_sht30Environment, CANDevice_INDEX_softwareVersion, CANDevice_SUB_INDEX, uint8, &SHT30Environment_softwareVersion, &size, 0);
}

// 指示灯按预设条目闪烁
void indicatorLight(uint8_t item)
{
    if(battery_data.socState == SOC_ENOUGH)
    {
        LED12VL_LED12VLState = 	LED12VR_LED12VRState = item;
    }
    else if(battery_data.socState == SOC_LOW_LEVEL_1)
    {
        //LED12VL_LED12VLState = INDECATOR_LIGHT_ITEM_SOC_LOW_LEVEL_1;
        LED12VL_LED12VLState = item;

    }
    else if(battery_data.socState == SOC_LOW_LEVEL_2)
    {
        LED12VR_LED12VRState = item;
        //LED12VR_LED12VRState = INDECATOR_LIGHT_ITEM_SOC_LOW_LEVEL_2;
    }
    //if (INDECATOR_LIGHT_IDLE == FALSE)
    //{
    WriteSDO(&CANopen_Master_M200_Data, CANDevice_NodeID_indecatorLightL, INDECATOR_LIGHT_INDEX_ITEM, INDECATOR_LIGHT_SUB_INDEX_ITEM, 1, uint8, &LED12VL_LED12VLState, 0);
    WriteSDO(&CANopen_Master_M200_Data, CANDevice_NodeID_indecatorLightR, INDECATOR_LIGHT_INDEX_ITEM, INDECATOR_LIGHT_SUB_INDEX_ITEM, 1, uint8, &LED12VR_LED12VRState, 0);
    //}
}

// 语音模块播放预设条目
void speakItem(uint8_t item)
{
    WriteSDO(&CANopen_Master_M200_Data, CANDevice_NodeID_speak, SPEAK_INDEX_ITEM, SPEAK_SUB_INDEX_ITEM, 1, uint8, &item, 0);
}

// 电池数据初始化
void battery_init(void)
{

		// 固定参数
		//battery_data.battHarewareVersion			= 0;		// 电池硬件版本号
		//battery_data.battSoftwareVersion			= 0;		// 电池硬件版本号

		// SPB21-SW20-008-A01通讯参数
		battery_data.cur_count 					    = 1;		// 1读取电量包 2读取电流包
		battery_data.energy_rx_count				= 0;		// 接收帧计数
		battery_data.energy_rx_flag					= FALSE;	// 接收帧计数

		// 电池温度
		battery_data.battTempPoint					= 4;		// 温度采集点个数
		for(int i=0; i<8; i++)
        {
				battery_data.batt_temp[i] = 0;				    // 电池温度：老电池只用前4位：0-1是电芯0-1温度，2是MOS温度，3是MOS温度；新电池用8位：0-7是电芯0-7的温度
		}
		// 充电相关参数
		battery_data.charge_state					= 0;       	// 电量包充放电状态
		battery_data.charge_flag					= FALSE;    // 电池充电标志位
		battery_data.charge_overflag				= 0;        // 电池过充电标志位
		battery_data.spentChargeTime				= 0;    	// 已经充电时间（分钟）
		battery_data.circle							= 0;        // 循环次数
		battery_data.timestamp						= 0;        // 已充电时间时间戳形式
		battery_data.chargeTimeRemain				= 0;   		// 充电剩余时间（分钟）
		battery_data.dischargeTimeRemain		    = 0;   		// 放电剩余时间（分钟）
		//battery_data.lastChargeTime[16];      				// 最后一次充电时间（时间格式）

		// 电压电流参数
		battery_data.voitageCurrentH				= 0;		// 电池当前电压高字节
		battery_data.voitageCurrent					= 0;		// 电池当前电压
		battery_data.cellVoitageMax					= 0;		// 电池最高电芯电压
		battery_data.cellVoitageMin					= 0;		// 电池最低电芯电压
		battery_data.currentCurrentH				= 0;		// 电池当前电流高字节
		battery_data.currentCurrent					= 0;		// 电池当前电流

		// 容量和健康度参数
		battery_data.soc							= 0;		// 电池电量剩余百分比
		battery_data.capDesignH						= 0;		// 电池设计容量高
		battery_data.capDesign						= 0;		// 电池设计容量
		battery_data.capOverall						= 0;		// 电池满容量
		battery_data.capRemain						= 0;		// 电池当前的剩余容量
		battery_data.soh							= 100;		// 电池健康百分比

		// 电池状态
		battery_data.protect_state					= 0;		// 电池保护状态
		battery_data.warningState					= 0;		// 电池告警状态
		battery_data.faultState						= 0;		// 电池故障状态
		battery_data.bmsState						= 0;		// BMS状态
		battery_data.socState						= 0;		// SOC状态



		// 确定电池种类
		batteryTypeRead();
}

// 确定电池种类
void batteryTypeRead(void)
{
    // 确定电池种类
    UNS32 size;
    size = sizeof(s48100Battery_batterySOC);
    for(uint8_t i=0; i<=3; i++ )
    {
        if(ReadSDO(&CANopen_Master_M200_Data, 0x04, 0x6300, 0x05, uint16, &s48100Battery_batterySOC, &size, 0) == 0)
        {
            battery_data.battType			= S48100;								//电池种类
            battery_data.soc = s48100Battery_batterySOC / 10;
            strcpy(battery_data.battTypeName, "S48100");
        }
        else
        {
            battery_data.battType			= QC105;								//电池种类,默认
            strcpy(battery_data.battTypeName, "QC105");
        }
    }

    // 上电读取电池SOC数据
    if (battery_data.battType == QC105)
    {
        control_flag.bat_read_flag = BAT_ENERGY;
        read_energy_data();		//读取初始电量
    }
    else if(battery_data.battType == s48100Battery_batterySOC)
    {
        batteryS48100ReadParameter();
    }
}


// 电池读取数据（兼容）
void batteryReadData(void)
{
    if(battery_data.battType == QC105)
    {
        switch(battery_data.cur_count)
        {
            case 1://读取电量数据包
                battery_data.cur_count=2;
                control_flag.bat_read_flag = BAT_ENERGY;
                read_energy_data();
                break;

            case 2://读取电流数据包
                battery_data.cur_count=1;
                control_flag.bat_read_flag = BAT_CURRENT;
                read_current_data();
                break;
        }
    }
    else if(battery_data.battType == S48100)
    {
        batteryS48100ReadData();
        batteryS48100PdoDataCopy();
    }
    else
    {

    }
    softSwitch_switchBatteySOC = battery_data.soc;
}

// 电池S48100电池数据拷贝
void batteryS48100PdoDataCopy(void)
{
    // 充电相关参数
    battery_data.charge_state 				= s48100Battery_chargeDischargeControl;     //电量包充放电状态

    if (battery_data.charge_state  == 0x31 | battery_data.charge_state  == 0x01)	    //电池充电标志位
        battery_data.charge_flag 			= TRUE;
    else
        battery_data.charge_flag 			= FALSE;

    // 容量和健康度参数
    battery_data.soc 						= s48100Battery_batterySOC / 10;			// 电池电量剩余百分比SOC
    battery_data.capRemain 					= s48100Battery_capRemain * 1000;			// 电池当前的剩余容量
    battery_data.soh 						= s48100Battery_batterySOH;					// 电池健康百分比SOH

    battery_data.chargeTimeRemain 		    = (1000-s48100Battery_batterySOC)*120/1000;                 // 充电剩余时间（分钟）

    // 电压电流参数
    battery_data.voitageCurrent 			= s48100Battery_currentVoltage * 10;		// 电池当前电压
    battery_data.currentCurrent 			= (int16_t)s48100Battery_currentCurrent;	// 电池当前电流

    // 电池状态
    battery_data.protect_state 				= s48100Battery_protectState;				// 保护状态
    battery_data.warningState				= s48100Battery_warningState;				// 告警状态
    battery_data.faultState					= s48100Battery_batteryFaultState;			// 故障状态
    battery_data.bmsState					= s48100Battery_BMSStatus;					// BMS状态

}

// 电池S48100Sdo读取固定参数
void batteryS48100ReadParameter(void)
{
		UNS32 size;

		// 固定参数(S48100电池通讯协议的分段SDO通讯有问题)
//		size = sizeof(battery_data.battHarewareVersion);		// 电池硬件版本号
//		ReadSDO(&CANopen_Master_M200_Data, S48100_NODE_ID, S48100_INDEX, S48100_SUB_INDEX_hardwareVersion, domain, battery_data.battHarewareVersion, &size, 0);
//		size = sizeof(battery_data.battSoftwareVersion);		// 电池软件版本号
//		ReadSDO(&CANopen_Master_M200_Data, S48100_NODE_ID, S48100_INDEX, S48100_SUB_INDEX_softwareVersion, domain, battery_data.battSoftwareVersion, &size, 0);
//		size = sizeof(battery_data.battSN);									// 电池SN码
//		ReadSDO(&CANopen_Master_M200_Data, S48100_NODE_ID, S48100_INDEX, S48100_SUB_INDEX_batterySN, domain, battery_data.battSN, &size, 0);


		// 电池温度
		size = sizeof(battery_data.battTempPoint);			// 温度测试点数量读取
		ReadSDO(&CANopen_Master_M200_Data, S48100_NODE_ID, S48100_INDEX, S48100_SUB_INDEX_battTempPoint, uint8, &battery_data.battTempPoint, &size, 0);

		// 容量和健康度参数
		size = sizeof(battery_data.voitageCurrent);  // 给2字节的size	// 电池设计容量；IOT无单位mAH，double，取值为正；
		ReadSDO(&CANopen_Master_M200_Data, S48100_NODE_ID, S48100_INDEX, S48100_SUB_INDEX_capDesign, uint16, &battery_data.capDesign, &size, 0);

}

// 电池S48100Sdo读取数据
void batteryS48100ReadData(void)
{

		UNS32 size;

        uint16_t battery_data_temp;
		// 电池温度
		//battery_data.batt_temp[8]; 						//电池温度：老电池只用前4位：0-1是电芯0-1温度，2是MOS温度，3是MOS温度；新电池用8位：0-7是电芯0-7的温度
		size = sizeof(battery_data.batt_temp[0]);
		if(!ReadSDO(&CANopen_Master_M200_Data, S48100_NODE_ID, S48100_INDEX, S48100_SUB_INDEX_battTemp1, int16, &battery_data_temp, &size, 0))
        battery_data.batt_temp[0] = battery_data_temp / 10;

		if(!ReadSDO(&CANopen_Master_M200_Data, S48100_NODE_ID, S48100_INDEX, S48100_SUB_INDEX_battTemp2, int16, &battery_data_temp, &size, 0))
        battery_data.batt_temp[1] = battery_data_temp / 10;

		if(!ReadSDO(&CANopen_Master_M200_Data, S48100_NODE_ID, S48100_INDEX, S48100_SUB_INDEX_battTemp3, int16, &battery_data_temp, &size, 0))
        battery_data.batt_temp[2] = battery_data_temp / 10;

		if(!ReadSDO(&CANopen_Master_M200_Data, S48100_NODE_ID, S48100_INDEX, S48100_SUB_INDEX_battTemp4, int16, &battery_data_temp, &size, 0))
        battery_data.batt_temp[3] = battery_data_temp / 10;

		// 充电相关参数
		//battery_data.charge_overflag 			= 0;        // 电池过充电标志位
		//battery_data.spentChargeTime 			= 0;    	// 已经充电时间（分钟）
        battery_data.circle 					= 0;      	// 循环次数
		//battery_data.timestamp 				= 0;        // 已充电时间时间戳形式
		//battery_data.chargeTimeRemain 		= 0;   		// 充电剩余时间（分钟）
		//battery_data.dischargeTimeRemain 	    = 0;   		// 放电剩余时间（分钟）
		//battery_data.lastChargeTime[16];      			// 最后一次充电时间（时间格式）

		// 电压电流参数
		size = sizeof(battery_data.cellVoitageMax);				//电池最高电芯电压
		ReadSDO(&CANopen_Master_M200_Data, S48100_NODE_ID, S48100_INDEX, S48100_SUB_INDEX_cellVoltageMax, uint16, &battery_data.cellVoitageMax, &size, 0);
		size = sizeof(battery_data.cellVoitageMin);				// 电池最低电芯电压
		ReadSDO(&CANopen_Master_M200_Data, S48100_NODE_ID, S48100_INDEX, S48100_SUB_INDEX_cellVoltageMin, uint16, &battery_data.cellVoitageMin, &size, 0);


		// 电池状态
		size = sizeof(battery_data.cellEquilibriumState);		//均衡状态
		ReadSDO(&CANopen_Master_M200_Data, S48100_NODE_ID, S48100_INDEX, S48100_SUB_INDEX_cellEquilibriumState, uint16, &battery_data.cellEquilibriumState, &size, 0);
}

// 辅助S48100进行充电
void S48100B_TEMP_CHECK(void)
{
    Message message;
    message.cob_id = 0x604;
    message.len = 8;
    message.data[0] = 0x40;
    message.data[1] = 0x00;
    message.data[2] = 0x63;
    message.data[3] = 0x0D;
    message.data[4] = 0x00;
    message.data[5] = 0x00;
    message.data[6] = 0x00;
    message.data[7] = 0x00;

    canSend(CANopen_Master_M200_Data.canHandle, &message);
}

// 电池QC105基础读取命令
void send_battery_cmd(uint8_t* msg,uint8_t BATSTDID)
{
	strcpy((char *)can_handle.TxData, (const char *)msg);
    can_handle.TxHeader.StdId = BATSTDID;
	can_handle.TxHeader.DLC   = MOTO_DLC;
	can_handle.TxHeader.IDE		= CAN_ID_STD;
	can_handle.TxHeader.RTR		= CAN_RTR_DATA;
	can_handle.TxHeader.TransmitGlobalTime = DISABLE;
	HAL_CAN_AddTxMessage(&hcan1, &can_handle.TxHeader, can_handle.TxData,&can_handle.TxMailbox);
}

// 电池QC105读取电量包
void read_energy_data(void)
{
	send_battery_cmd(CAN_TX_Data1,BAT_STAR_StdId);
	send_battery_cmd(CAN_TX_Data2_energy,BAT_DATA_StdId);
	send_battery_cmd(CAN_TX_Data3,BAT_OVER_StdId);
}

// 电池QC105读取电流包
void read_current_data(void)
{
	send_battery_cmd(CAN_TX_Data1,BAT_STAR_StdId);
	send_battery_cmd(CAN_TX_Data2_current,BAT_DATA_StdId);
	send_battery_cmd(CAN_TX_Data3,BAT_OVER_StdId);
}

// 电池QC105返回数据处理
void battery_ack_handle(void)
{
	switch(control_flag.bat_read_flag)
	{
        case BAT_ENERGY:
            if(can_handle.RxData[0] == 0xEA  && can_handle.RxData[5] == 0x04)
            {
                battery_data.energy_rx_flag = TRUE;
            }
            if(battery_data.energy_rx_flag == TRUE)
            {
                if(battery_data.energy_rx_count <= 8)
                {
                    battery_data.energy_rx_count++;
                    switch(battery_data.energy_rx_count)
                    {
                        case 1:
                            battery_data.soc =  can_handle.RxData[7];																						// 电量SOC
                            break;

                        case 2:
                            battery_data.circle = (can_handle.RxData[1]<<8)|can_handle.RxData[2];										                    // 电池循环次数
                            battery_data.capDesignH = (can_handle.RxData[4]<<24)|(can_handle.RxData[5]<<16)|(can_handle.RxData[7]<<8)|0x00;
                            //battery_data.batteryCapDesign = (can_handle.RxData[4]<<8)|can_handle.RxData[5];
                            break;

                        case 3:
                            battery_data.capDesign = battery_data.capDesignH | can_handle.RxData[0];								                        // 电池设计容量
                            battery_data.capOverall = (can_handle.RxData[2]<<24)|(can_handle.RxData[3]<<16)|(can_handle.RxData[5]<<8)|can_handle.RxData[6];	// 读电池满容量
                            break;

                        case 4:
                            battery_data.capRemain = (can_handle.RxData[0]<<24)|(can_handle.RxData[1]<<16)|(can_handle.RxData[3]<<8)|can_handle.RxData[4];	// 读电池剩余容量
                            battery_data.dischargeTimeRemain = (can_handle.RxData[6]<<8)|can_handle.RxData[7];			                                    // 读电池放电时间
                            break;

                        case 5:
                            battery_data.chargeTimeRemain = (can_handle.RxData[1]<<8)|can_handle.RxData[2];					                                // 电池充电剩余时间
                            break;

                        case 6:
                            battery_data.voitageCurrentH = can_handle.RxData[7];																		    // 读电池当前电压高字节
                            break;

                        case 7:
                            battery_data.voitageCurrent = (battery_data.voitageCurrentH<<8)|can_handle.RxData[0];		        // 读电池当前电压
                            battery_data.cellVoitageMax = (can_handle.RxData[1]<<8)|can_handle.RxData[2];						// 读电池最高电芯电压
                            battery_data.cellVoitageMin = (can_handle.RxData[3]<<8)|can_handle.RxData[4];						// 读电池最低电芯电压
                            sprintf(battery_data.battHarewareVersion, "%d", can_handle.RxData[6]);								// 电池硬件版本号
                            break;

                        case 8:
                            battery_data.energy_rx_count = 0;
                            battery_data.energy_rx_flag = FALSE;
                            break;
                    }
                }
            }
            break;

        case BAT_CURRENT:
            if(can_handle.RxData[0] == 0xEA  && can_handle.RxData[5] == 0x03)
            {
                battery_data.energy_rx_flag = TRUE;
            }
            if(battery_data.energy_rx_flag == TRUE)
            {
                if(battery_data.energy_rx_count <= 4)
                {
                    battery_data.energy_rx_count++;
                    switch(battery_data.energy_rx_count)
                    {
                        case 1:
                            battery_data.charge_state = can_handle.RxData[6];			                                // 0x02 0 0 0 0 0 0 1 0充电标志位
                            if(battery_data.charge_state & 0x02)									                    // 电池充放电状态
                            {
                                battery_data.charge_flag = TRUE;
                            }
                            else
                            {
                                battery_data.charge_flag = FALSE;
                            }
                            battery_data.currentCurrentH = can_handle.RxData[7];										// 读取当前电流高字节
                            break;

                        case 2:
                            battery_data.currentCurrent = (battery_data.currentCurrentH<<8)|can_handle.RxData[0];		// 读取当前电流
                            battery_data.protect_state = (can_handle.RxData[3]<<8)|can_handle.RxData[4];				// 读电池保护状态
                            battery_data.battTempPoint = can_handle.RxData[5];			                                // 读取测温点数量
                            battery_data.batt_temp[0] = can_handle.RxData[6] - 40;		                                // 读取温度0
                            battery_data.batt_temp[1] = can_handle.RxData[7] - 40;		                                // 读取温度1

                            break;

                        case 3:
                            battery_data.batt_temp[2] = can_handle.RxData[0] - 40;		                                // 读取温度2
                            battery_data.batt_temp[3] = can_handle.RxData[1] - 40;		                                // 读取温度3
                            battery_data.cellEquilibriumState = (can_handle.RxData[5]<<8)|can_handle.RxData[6];			// 电池均衡状态
                            sprintf(battery_data.battHarewareVersion, "%d", can_handle.RxData[7]);						// 电池软件版本号
                            break;

                        case 4:
                            battery_data.faultState		= can_handle.RxData[1];
                            battery_data.warningState = (can_handle.RxData[2]<<8)|can_handle.RxData[3];					// 电池告警状态
                            battery_data.energy_rx_count = 0;
                            battery_data.energy_rx_flag = FALSE;
                            break;
                    }
                }
            }
            break;
        case BAT_VOLTAGE:
            break;
	}
}

// 电机驱动器监控参数读取
void read_moto_data(void)
{
	send_moto_cmd(MOTO_STATE, MOTO_REGIS_NUM1, 0, MOTO_ReadREQ_ID);           //读取电机驱动器状态
	send_moto_cmd(MOTO_TEMP1, MOTO_REGIS_NUM1, 0, MOTO_ReadREQ_ID);           //读取驱动单元1温度
	send_moto_cmd(MOTO_TEMP2, MOTO_REGIS_NUM1, 0, MOTO_ReadREQ_ID);           //读取驱动单元2温度
	send_moto_cmd(MOTO_TEMP3, MOTO_REGIS_NUM1, 0, MOTO_ReadREQ_ID);           //读取驱动单元3温度
	send_moto_cmd(MOTO_TEMP4, MOTO_REGIS_NUM1, 0, MOTO_ReadREQ_ID);           //读取驱动单元4温度

}

// 电机返回数据处理，主动上报
void moto_ack_handle(void)
{
	can_read_data.Cmd_addr = can_handle.RxData[0];
	switch(can_read_data.Cmd_addr)
	{
		case MOTO1_HALL:
			//电机霍尔寄存器处理
            can_read_data.Hall_new = (can_handle.RxData[5]<<24)|(can_handle.RxData[4]<<16)|(can_handle.RxData[3]<<8)|can_handle.RxData[2];
            can_read_data.Hall_rate = can_read_data.Hall_new - can_read_data.Hall_old;
            if(can_read_data.Hall_rate<0)
            {
                can_read_data.Hall_rate = ~can_read_data.Hall_rate;
            }
            can_read_data.Speed = can_read_data.Hall_rate*750/650;
            if(control_flag.Car_State == RUN) {
                sing_work_event.mileage = sing_work_event.mileage + can_read_data.Speed;//里程累计1秒1次
            }
            //if(can_read_data.Speed != 0){//依据霍尔判断
            //	SETBIT(mqtt_pub_inform.runvehicleStatus,14);
            //}
            //else {
            //	CLRBIT(mqtt_pub_inform.runvehicleStatus,14);
            //}
            can_read_data.Hall_old = can_read_data.Hall_new;
            break;

		case MOTO_STATE:
			//电机状态寄存器处理
            can_read_data.moto_state = (can_handle.RxData[3]<<8)|can_handle.RxData[2];
			break;
		case MOTO_TEMP1:
			//驱动单元1温度寄存器处理
            can_read_data.moto_temp1 = ((can_handle.RxData[3]<<8)|can_handle.RxData[2]) / 10;
			break;
		case MOTO_TEMP2:
			//驱动单元2温度寄存器处理
            can_read_data.moto_temp2 = ((can_handle.RxData[3]<<8)|can_handle.RxData[2]) / 10;
			break;
		case MOTO_TEMP3:
			//驱动单元3温度寄存器处理
            can_read_data.moto_temp3 = ((can_handle.RxData[3]<<8)|can_handle.RxData[2]) / 10;
			break;
		case MOTO_TEMP4:
			//驱动单元4温度寄存器处理
            can_read_data.moto_temp4 = ((can_handle.RxData[3]<<8)|can_handle.RxData[2]) / 10;
			break;
	}

}

// 该函数因CANopen上线已作废，CAN线传感器读取函数
void read_driver_sensor(void)
{
	check_driver_sensor(TEMP, DRIVERID);
	check_driver_sensor(ANGLE, DRIVERID);
	check_driver_sensor(HUMITURE1, HUMITURE1ID);
	check_driver_sensor(HUMITURE2, HUMITURE2ID);
}

// 该函数因CANopen上线已作废，CAN线传感器读取函数
//void driver_sensor_ack_handle(void)
//{
//	switch(can_handle.RxHeader.StdId)
//	{
//		case DRIVERID:
//			//电驱板急停状态、箱体温度、、处理
//			driver_sensor.device_id = can_handle.RxData[2];
//			switch(driver_sensor.device_id)
//			{
//				case SCRAM:
//					//control_flag.scram_stop_flag = can_handle.RxData[4];
//					break;
//
//				case TEMP:
//					//
//					driver_sensor.boxPT100_1       = can_handle.RxData[4];
//					driver_sensor.boxPT100_2       = can_handle.RxData[5];
//					driver_sensor.temp_mcu 	       = can_handle.RxData[6];
//					driver_sensor.temp_6050        = can_handle.RxData[7];
//					break;
//
//				case KAIHE_DRIVER:
//					//control_flag.driver_kaihe_flag 		 = can_handle.RxData[5];
//					break;
//
//				case ANGLE:
//					//
//						driver_sensor.pitch					 = can_handle.RxData[4];
//						driver_sensor.roll 					 = can_handle.RxData[5];
//						driver_sensor.yaw  					 = can_handle.RxData[6];
//					break;
//			}
//			break;
//		case SOFTSWID:
//			//软开关状态处理
//			//control_flag.Soft_sw_flag 				 = can_handle.RxData[5];
//			break;
//		case HUMITURE1ID:
//			//温湿度传感器1处理
//			driver_sensor.humiture_wendu1  		 = can_handle.RxData[4];
//			driver_sensor.humiture_shidu1  		 = can_handle.RxData[5];
//			break;
//		case HUMITURE2ID:
//			//温湿度传感器2处理
//			driver_sensor.humiture_wendu2  		 = can_handle.RxData[4];
//			driver_sensor.humiture_shidu2  		 = can_handle.RxData[5];
//			break;
//	}
//}


