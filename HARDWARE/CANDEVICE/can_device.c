#include "can_device.h"
#include "cmsis_os.h"
#include "CANopen_Master_M200.h"
#include "canfestival.h"
BATTERY_DATA			 battery_data;
//DRIVER_SENSOR 		 driver_sensor;

extern CO_Data CANopen_Master_M200_Data;

// CAN�������汾�Ŷ�ȡ
void canDeviceVersionRead(void)
{
    UNS32 size;
    // ��������汾��
    size = sizeof(driverBoard_softwareVersion);
    ReadSDO(&CANopen_Master_M200_Data, CANDevice_NodeID_driver, CANDevice_INDEX_softwareVersion, CANDevice_SUB_INDEX, uint8, &driverBoard_softwareVersion, &size, 0);

    // �����ذ汾��
    size = sizeof(softSwitch_softwareVersion);
    ReadSDO(&CANopen_Master_M200_Data, CANDevice_NodeID_softWare, CANDevice_INDEX_softwareVersion, CANDevice_SUB_INDEX, uint8, &softSwitch_softwareVersion, &size, 0);

    // �������������汾��
    size = sizeof(speaker_softwareVersion);
    ReadSDO(&CANopen_Master_M200_Data, CANDevice_NodeID_speak, CANDevice_INDEX_softwareVersion, CANDevice_SUB_INDEX, uint8, &speaker_softwareVersion, &size, 0);

    // ��ָʾ����汾��
    size = sizeof(LED12VL_softwareVersion);
    ReadSDO(&CANopen_Master_M200_Data, CANDevice_NodeID_indecatorLightL, CANDevice_INDEX_softwareVersion, CANDevice_SUB_INDEX, uint8, &LED12VL_softwareVersion, &size, 0);

    // ��ָʾ���Ұ汾��
    size = sizeof(LED12VR_softwareVersion);
    ReadSDO(&CANopen_Master_M200_Data, CANDevice_NodeID_indecatorLightR, CANDevice_INDEX_softwareVersion, CANDevice_SUB_INDEX, uint8, &LED12VR_softwareVersion, &size, 0);

    // ����ʪ�ȳ���汾��
    size = sizeof(SHT30CarBox_softwareVersion);
    ReadSDO(&CANopen_Master_M200_Data, CANDevice_NodeID_sht30CarBox, CANDevice_INDEX_softwareVersion, CANDevice_SUB_INDEX, uint8, &SHT30CarBox_softwareVersion, &size, 0);

    // ����ʪ�Ȼ����汾��
    size = sizeof(SHT30Environment_softwareVersion);
    ReadSDO(&CANopen_Master_M200_Data, CANDevice_NodeID_sht30Environment, CANDevice_INDEX_softwareVersion, CANDevice_SUB_INDEX, uint8, &SHT30Environment_softwareVersion, &size, 0);
}

// ָʾ�ư�Ԥ����Ŀ��˸
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

// ����ģ�鲥��Ԥ����Ŀ
void speakItem(uint8_t item)
{
    WriteSDO(&CANopen_Master_M200_Data, CANDevice_NodeID_speak, SPEAK_INDEX_ITEM, SPEAK_SUB_INDEX_ITEM, 1, uint8, &item, 0);
}

// ������ݳ�ʼ��
void battery_init(void)
{

		// �̶�����
		//battery_data.battHarewareVersion			= 0;		// ���Ӳ���汾��
		//battery_data.battSoftwareVersion			= 0;		// ���Ӳ���汾��

		// SPB21-SW20-008-A01ͨѶ����
		battery_data.cur_count 					    = 1;		// 1��ȡ������ 2��ȡ������
		battery_data.energy_rx_count				= 0;		// ����֡����
		battery_data.energy_rx_flag					= FALSE;	// ����֡����

		// ����¶�
		battery_data.battTempPoint					= 4;		// �¶Ȳɼ������
		for(int i=0; i<8; i++)
        {
				battery_data.batt_temp[i] = 0;				    // ����¶ȣ��ϵ��ֻ��ǰ4λ��0-1�ǵ�о0-1�¶ȣ�2��MOS�¶ȣ�3��MOS�¶ȣ��µ����8λ��0-7�ǵ�о0-7���¶�
		}
		// �����ز���
		battery_data.charge_state					= 0;       	// ��������ŵ�״̬
		battery_data.charge_flag					= FALSE;    // ��س���־λ
		battery_data.charge_overflag				= 0;        // ��ع�����־λ
		battery_data.spentChargeTime				= 0;    	// �Ѿ����ʱ�䣨���ӣ�
		battery_data.circle							= 0;        // ѭ������
		battery_data.timestamp						= 0;        // �ѳ��ʱ��ʱ�����ʽ
		battery_data.chargeTimeRemain				= 0;   		// ���ʣ��ʱ�䣨���ӣ�
		battery_data.dischargeTimeRemain		    = 0;   		// �ŵ�ʣ��ʱ�䣨���ӣ�
		//battery_data.lastChargeTime[16];      				// ���һ�γ��ʱ�䣨ʱ���ʽ��

		// ��ѹ��������
		battery_data.voitageCurrentH				= 0;		// ��ص�ǰ��ѹ���ֽ�
		battery_data.voitageCurrent					= 0;		// ��ص�ǰ��ѹ
		battery_data.cellVoitageMax					= 0;		// �����ߵ�о��ѹ
		battery_data.cellVoitageMin					= 0;		// �����͵�о��ѹ
		battery_data.currentCurrentH				= 0;		// ��ص�ǰ�������ֽ�
		battery_data.currentCurrent					= 0;		// ��ص�ǰ����

		// �����ͽ����Ȳ���
		battery_data.soc							= 0;		// ��ص���ʣ��ٷֱ�
		battery_data.capDesignH						= 0;		// ������������
		battery_data.capDesign						= 0;		// ����������
		battery_data.capOverall						= 0;		// ���������
		battery_data.capRemain						= 0;		// ��ص�ǰ��ʣ������
		battery_data.soh							= 100;		// ��ؽ����ٷֱ�

		// ���״̬
		battery_data.protect_state					= 0;		// ��ر���״̬
		battery_data.warningState					= 0;		// ��ظ澯״̬
		battery_data.faultState						= 0;		// ��ع���״̬
		battery_data.bmsState						= 0;		// BMS״̬
		battery_data.socState						= 0;		// SOC״̬



		// ȷ���������
		batteryTypeRead();
}

// ȷ���������
void batteryTypeRead(void)
{
    // ȷ���������
    UNS32 size;
    size = sizeof(s48100Battery_batterySOC);
    for(uint8_t i=0; i<=3; i++ )
    {
        if(ReadSDO(&CANopen_Master_M200_Data, 0x04, 0x6300, 0x05, uint16, &s48100Battery_batterySOC, &size, 0) == 0)
        {
            battery_data.battType			= S48100;								//�������
            battery_data.soc = s48100Battery_batterySOC / 10;
            strcpy(battery_data.battTypeName, "S48100");
        }
        else
        {
            battery_data.battType			= QC105;								//�������,Ĭ��
            strcpy(battery_data.battTypeName, "QC105");
        }
    }

    // �ϵ��ȡ���SOC����
    if (battery_data.battType == QC105)
    {
        control_flag.bat_read_flag = BAT_ENERGY;
        read_energy_data();		//��ȡ��ʼ����
    }
    else if(battery_data.battType == s48100Battery_batterySOC)
    {
        batteryS48100ReadParameter();
    }
}


// ��ض�ȡ���ݣ����ݣ�
void batteryReadData(void)
{
    if(battery_data.battType == QC105)
    {
        switch(battery_data.cur_count)
        {
            case 1://��ȡ�������ݰ�
                battery_data.cur_count=2;
                control_flag.bat_read_flag = BAT_ENERGY;
                read_energy_data();
                break;

            case 2://��ȡ�������ݰ�
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

// ���S48100������ݿ���
void batteryS48100PdoDataCopy(void)
{
    // �����ز���
    battery_data.charge_state 				= s48100Battery_chargeDischargeControl;     //��������ŵ�״̬

    if (battery_data.charge_state  == 0x31 | battery_data.charge_state  == 0x01)	    //��س���־λ
        battery_data.charge_flag 			= TRUE;
    else
        battery_data.charge_flag 			= FALSE;

    // �����ͽ����Ȳ���
    battery_data.soc 						= s48100Battery_batterySOC / 10;			// ��ص���ʣ��ٷֱ�SOC
    battery_data.capRemain 					= s48100Battery_capRemain * 1000;			// ��ص�ǰ��ʣ������
    battery_data.soh 						= s48100Battery_batterySOH;					// ��ؽ����ٷֱ�SOH

    battery_data.chargeTimeRemain 		    = (1000-s48100Battery_batterySOC)*120/1000;                 // ���ʣ��ʱ�䣨���ӣ�

    // ��ѹ��������
    battery_data.voitageCurrent 			= s48100Battery_currentVoltage * 10;		// ��ص�ǰ��ѹ
    battery_data.currentCurrent 			= (int16_t)s48100Battery_currentCurrent;	// ��ص�ǰ����

    // ���״̬
    battery_data.protect_state 				= s48100Battery_protectState;				// ����״̬
    battery_data.warningState				= s48100Battery_warningState;				// �澯״̬
    battery_data.faultState					= s48100Battery_batteryFaultState;			// ����״̬
    battery_data.bmsState					= s48100Battery_BMSStatus;					// BMS״̬

}

// ���S48100Sdo��ȡ�̶�����
void batteryS48100ReadParameter(void)
{
		UNS32 size;

		// �̶�����(S48100���ͨѶЭ��ķֶ�SDOͨѶ������)
//		size = sizeof(battery_data.battHarewareVersion);		// ���Ӳ���汾��
//		ReadSDO(&CANopen_Master_M200_Data, S48100_NODE_ID, S48100_INDEX, S48100_SUB_INDEX_hardwareVersion, domain, battery_data.battHarewareVersion, &size, 0);
//		size = sizeof(battery_data.battSoftwareVersion);		// �������汾��
//		ReadSDO(&CANopen_Master_M200_Data, S48100_NODE_ID, S48100_INDEX, S48100_SUB_INDEX_softwareVersion, domain, battery_data.battSoftwareVersion, &size, 0);
//		size = sizeof(battery_data.battSN);									// ���SN��
//		ReadSDO(&CANopen_Master_M200_Data, S48100_NODE_ID, S48100_INDEX, S48100_SUB_INDEX_batterySN, domain, battery_data.battSN, &size, 0);


		// ����¶�
		size = sizeof(battery_data.battTempPoint);			// �¶Ȳ��Ե�������ȡ
		ReadSDO(&CANopen_Master_M200_Data, S48100_NODE_ID, S48100_INDEX, S48100_SUB_INDEX_battTempPoint, uint8, &battery_data.battTempPoint, &size, 0);

		// �����ͽ����Ȳ���
		size = sizeof(battery_data.voitageCurrent);  // ��2�ֽڵ�size	// ������������IOT�޵�λmAH��double��ȡֵΪ����
		ReadSDO(&CANopen_Master_M200_Data, S48100_NODE_ID, S48100_INDEX, S48100_SUB_INDEX_capDesign, uint16, &battery_data.capDesign, &size, 0);

}

// ���S48100Sdo��ȡ����
void batteryS48100ReadData(void)
{

		UNS32 size;

        uint16_t battery_data_temp;
		// ����¶�
		//battery_data.batt_temp[8]; 						//����¶ȣ��ϵ��ֻ��ǰ4λ��0-1�ǵ�о0-1�¶ȣ�2��MOS�¶ȣ�3��MOS�¶ȣ��µ����8λ��0-7�ǵ�о0-7���¶�
		size = sizeof(battery_data.batt_temp[0]);
		if(!ReadSDO(&CANopen_Master_M200_Data, S48100_NODE_ID, S48100_INDEX, S48100_SUB_INDEX_battTemp1, int16, &battery_data_temp, &size, 0))
        battery_data.batt_temp[0] = battery_data_temp / 10;

		if(!ReadSDO(&CANopen_Master_M200_Data, S48100_NODE_ID, S48100_INDEX, S48100_SUB_INDEX_battTemp2, int16, &battery_data_temp, &size, 0))
        battery_data.batt_temp[1] = battery_data_temp / 10;

		if(!ReadSDO(&CANopen_Master_M200_Data, S48100_NODE_ID, S48100_INDEX, S48100_SUB_INDEX_battTemp3, int16, &battery_data_temp, &size, 0))
        battery_data.batt_temp[2] = battery_data_temp / 10;

		if(!ReadSDO(&CANopen_Master_M200_Data, S48100_NODE_ID, S48100_INDEX, S48100_SUB_INDEX_battTemp4, int16, &battery_data_temp, &size, 0))
        battery_data.batt_temp[3] = battery_data_temp / 10;

		// �����ز���
		//battery_data.charge_overflag 			= 0;        // ��ع�����־λ
		//battery_data.spentChargeTime 			= 0;    	// �Ѿ����ʱ�䣨���ӣ�
        battery_data.circle 					= 0;      	// ѭ������
		//battery_data.timestamp 				= 0;        // �ѳ��ʱ��ʱ�����ʽ
		//battery_data.chargeTimeRemain 		= 0;   		// ���ʣ��ʱ�䣨���ӣ�
		//battery_data.dischargeTimeRemain 	    = 0;   		// �ŵ�ʣ��ʱ�䣨���ӣ�
		//battery_data.lastChargeTime[16];      			// ���һ�γ��ʱ�䣨ʱ���ʽ��

		// ��ѹ��������
		size = sizeof(battery_data.cellVoitageMax);				//�����ߵ�о��ѹ
		ReadSDO(&CANopen_Master_M200_Data, S48100_NODE_ID, S48100_INDEX, S48100_SUB_INDEX_cellVoltageMax, uint16, &battery_data.cellVoitageMax, &size, 0);
		size = sizeof(battery_data.cellVoitageMin);				// �����͵�о��ѹ
		ReadSDO(&CANopen_Master_M200_Data, S48100_NODE_ID, S48100_INDEX, S48100_SUB_INDEX_cellVoltageMin, uint16, &battery_data.cellVoitageMin, &size, 0);


		// ���״̬
		size = sizeof(battery_data.cellEquilibriumState);		//����״̬
		ReadSDO(&CANopen_Master_M200_Data, S48100_NODE_ID, S48100_INDEX, S48100_SUB_INDEX_cellEquilibriumState, uint16, &battery_data.cellEquilibriumState, &size, 0);
}

// ����S48100���г��
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

// ���QC105������ȡ����
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

// ���QC105��ȡ������
void read_energy_data(void)
{
	send_battery_cmd(CAN_TX_Data1,BAT_STAR_StdId);
	send_battery_cmd(CAN_TX_Data2_energy,BAT_DATA_StdId);
	send_battery_cmd(CAN_TX_Data3,BAT_OVER_StdId);
}

// ���QC105��ȡ������
void read_current_data(void)
{
	send_battery_cmd(CAN_TX_Data1,BAT_STAR_StdId);
	send_battery_cmd(CAN_TX_Data2_current,BAT_DATA_StdId);
	send_battery_cmd(CAN_TX_Data3,BAT_OVER_StdId);
}

// ���QC105�������ݴ���
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
                            battery_data.soc =  can_handle.RxData[7];																						// ����SOC
                            break;

                        case 2:
                            battery_data.circle = (can_handle.RxData[1]<<8)|can_handle.RxData[2];										                    // ���ѭ������
                            battery_data.capDesignH = (can_handle.RxData[4]<<24)|(can_handle.RxData[5]<<16)|(can_handle.RxData[7]<<8)|0x00;
                            //battery_data.batteryCapDesign = (can_handle.RxData[4]<<8)|can_handle.RxData[5];
                            break;

                        case 3:
                            battery_data.capDesign = battery_data.capDesignH | can_handle.RxData[0];								                        // ����������
                            battery_data.capOverall = (can_handle.RxData[2]<<24)|(can_handle.RxData[3]<<16)|(can_handle.RxData[5]<<8)|can_handle.RxData[6];	// �����������
                            break;

                        case 4:
                            battery_data.capRemain = (can_handle.RxData[0]<<24)|(can_handle.RxData[1]<<16)|(can_handle.RxData[3]<<8)|can_handle.RxData[4];	// �����ʣ������
                            battery_data.dischargeTimeRemain = (can_handle.RxData[6]<<8)|can_handle.RxData[7];			                                    // ����طŵ�ʱ��
                            break;

                        case 5:
                            battery_data.chargeTimeRemain = (can_handle.RxData[1]<<8)|can_handle.RxData[2];					                                // ��س��ʣ��ʱ��
                            break;

                        case 6:
                            battery_data.voitageCurrentH = can_handle.RxData[7];																		    // ����ص�ǰ��ѹ���ֽ�
                            break;

                        case 7:
                            battery_data.voitageCurrent = (battery_data.voitageCurrentH<<8)|can_handle.RxData[0];		        // ����ص�ǰ��ѹ
                            battery_data.cellVoitageMax = (can_handle.RxData[1]<<8)|can_handle.RxData[2];						// �������ߵ�о��ѹ
                            battery_data.cellVoitageMin = (can_handle.RxData[3]<<8)|can_handle.RxData[4];						// �������͵�о��ѹ
                            sprintf(battery_data.battHarewareVersion, "%d", can_handle.RxData[6]);								// ���Ӳ���汾��
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
                            battery_data.charge_state = can_handle.RxData[6];			                                // 0x02 0 0 0 0 0 0 1 0����־λ
                            if(battery_data.charge_state & 0x02)									                    // ��س�ŵ�״̬
                            {
                                battery_data.charge_flag = TRUE;
                            }
                            else
                            {
                                battery_data.charge_flag = FALSE;
                            }
                            battery_data.currentCurrentH = can_handle.RxData[7];										// ��ȡ��ǰ�������ֽ�
                            break;

                        case 2:
                            battery_data.currentCurrent = (battery_data.currentCurrentH<<8)|can_handle.RxData[0];		// ��ȡ��ǰ����
                            battery_data.protect_state = (can_handle.RxData[3]<<8)|can_handle.RxData[4];				// ����ر���״̬
                            battery_data.battTempPoint = can_handle.RxData[5];			                                // ��ȡ���µ�����
                            battery_data.batt_temp[0] = can_handle.RxData[6] - 40;		                                // ��ȡ�¶�0
                            battery_data.batt_temp[1] = can_handle.RxData[7] - 40;		                                // ��ȡ�¶�1

                            break;

                        case 3:
                            battery_data.batt_temp[2] = can_handle.RxData[0] - 40;		                                // ��ȡ�¶�2
                            battery_data.batt_temp[3] = can_handle.RxData[1] - 40;		                                // ��ȡ�¶�3
                            battery_data.cellEquilibriumState = (can_handle.RxData[5]<<8)|can_handle.RxData[6];			// ��ؾ���״̬
                            sprintf(battery_data.battHarewareVersion, "%d", can_handle.RxData[7]);						// �������汾��
                            break;

                        case 4:
                            battery_data.faultState		= can_handle.RxData[1];
                            battery_data.warningState = (can_handle.RxData[2]<<8)|can_handle.RxData[3];					// ��ظ澯״̬
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

// �����������ز�����ȡ
void read_moto_data(void)
{
	send_moto_cmd(MOTO_STATE, MOTO_REGIS_NUM1, 0, MOTO_ReadREQ_ID);           //��ȡ���������״̬
	send_moto_cmd(MOTO_TEMP1, MOTO_REGIS_NUM1, 0, MOTO_ReadREQ_ID);           //��ȡ������Ԫ1�¶�
	send_moto_cmd(MOTO_TEMP2, MOTO_REGIS_NUM1, 0, MOTO_ReadREQ_ID);           //��ȡ������Ԫ2�¶�
	send_moto_cmd(MOTO_TEMP3, MOTO_REGIS_NUM1, 0, MOTO_ReadREQ_ID);           //��ȡ������Ԫ3�¶�
	send_moto_cmd(MOTO_TEMP4, MOTO_REGIS_NUM1, 0, MOTO_ReadREQ_ID);           //��ȡ������Ԫ4�¶�

}

// ����������ݴ��������ϱ�
void moto_ack_handle(void)
{
	can_read_data.Cmd_addr = can_handle.RxData[0];
	switch(can_read_data.Cmd_addr)
	{
		case MOTO1_HALL:
			//��������Ĵ�������
            can_read_data.Hall_new = (can_handle.RxData[5]<<24)|(can_handle.RxData[4]<<16)|(can_handle.RxData[3]<<8)|can_handle.RxData[2];
            can_read_data.Hall_rate = can_read_data.Hall_new - can_read_data.Hall_old;
            if(can_read_data.Hall_rate<0)
            {
                can_read_data.Hall_rate = ~can_read_data.Hall_rate;
            }
            can_read_data.Speed = can_read_data.Hall_rate*750/650;
            if(control_flag.Car_State == RUN) {
                sing_work_event.mileage = sing_work_event.mileage + can_read_data.Speed;//����ۼ�1��1��
            }
            //if(can_read_data.Speed != 0){//���ݻ����ж�
            //	SETBIT(mqtt_pub_inform.runvehicleStatus,14);
            //}
            //else {
            //	CLRBIT(mqtt_pub_inform.runvehicleStatus,14);
            //}
            can_read_data.Hall_old = can_read_data.Hall_new;
            break;

		case MOTO_STATE:
			//���״̬�Ĵ�������
            can_read_data.moto_state = (can_handle.RxData[3]<<8)|can_handle.RxData[2];
			break;
		case MOTO_TEMP1:
			//������Ԫ1�¶ȼĴ�������
            can_read_data.moto_temp1 = ((can_handle.RxData[3]<<8)|can_handle.RxData[2]) / 10;
			break;
		case MOTO_TEMP2:
			//������Ԫ2�¶ȼĴ�������
            can_read_data.moto_temp2 = ((can_handle.RxData[3]<<8)|can_handle.RxData[2]) / 10;
			break;
		case MOTO_TEMP3:
			//������Ԫ3�¶ȼĴ�������
            can_read_data.moto_temp3 = ((can_handle.RxData[3]<<8)|can_handle.RxData[2]) / 10;
			break;
		case MOTO_TEMP4:
			//������Ԫ4�¶ȼĴ�������
            can_read_data.moto_temp4 = ((can_handle.RxData[3]<<8)|can_handle.RxData[2]) / 10;
			break;
	}

}

// �ú�����CANopen���������ϣ�CAN�ߴ�������ȡ����
void read_driver_sensor(void)
{
	check_driver_sensor(TEMP, DRIVERID);
	check_driver_sensor(ANGLE, DRIVERID);
	check_driver_sensor(HUMITURE1, HUMITURE1ID);
	check_driver_sensor(HUMITURE2, HUMITURE2ID);
}

// �ú�����CANopen���������ϣ�CAN�ߴ�������ȡ����
//void driver_sensor_ack_handle(void)
//{
//	switch(can_handle.RxHeader.StdId)
//	{
//		case DRIVERID:
//			//�����弱ͣ״̬�������¶ȡ�������
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
//			//����״̬����
//			//control_flag.Soft_sw_flag 				 = can_handle.RxData[5];
//			break;
//		case HUMITURE1ID:
//			//��ʪ�ȴ�����1����
//			driver_sensor.humiture_wendu1  		 = can_handle.RxData[4];
//			driver_sensor.humiture_shidu1  		 = can_handle.RxData[5];
//			break;
//		case HUMITURE2ID:
//			//��ʪ�ȴ�����2����
//			driver_sensor.humiture_wendu2  		 = can_handle.RxData[4];
//			driver_sensor.humiture_shidu2  		 = can_handle.RxData[5];
//			break;
//	}
//}


