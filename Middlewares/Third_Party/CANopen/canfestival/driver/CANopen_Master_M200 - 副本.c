
/* File generated by gen_cfile.py. Should not be modified. */

#include "CANopen_Master_M200.h"

/**************************************************************************/
/* Declaration of mapped variables                                        */
/**************************************************************************/
UNS8 array[] =		/* Mapped at index 0x200A, subindex 0x01 - 0x32 */
  {
    0x0,	/* 0 */
    0x0,	/* 0 */
    0x0,	/* 0 */
    0x0,	/* 0 */
    0x0,	/* 0 */
    0x0,	/* 0 */
    0x0,	/* 0 */
    0x0,	/* 0 */
    0x0,	/* 0 */
    0x0,	/* 0 */
    0x0,	/* 0 */
    0x0,	/* 0 */
    0x0,	/* 0 */
    0x0,	/* 0 */
    0x0,	/* 0 */
    0x0,	/* 0 */
    0x0,	/* 0 */
    0x0,	/* 0 */
    0x0,	/* 0 */
    0x0,	/* 0 */
    0x0,	/* 0 */
    0x0,	/* 0 */
    0x0,	/* 0 */
    0x0,	/* 0 */
    0x0,	/* 0 */
    0x0,	/* 0 */
    0x0,	/* 0 */
    0x0,	/* 0 */
    0x0,	/* 0 */
    0x0,	/* 0 */
    0x0,	/* 0 */
    0x0,	/* 0 */
    0x0,	/* 0 */
    0x0,	/* 0 */
    0x0,	/* 0 */
    0x0,	/* 0 */
    0x0,	/* 0 */
    0x0,	/* 0 */
    0x0,	/* 0 */
    0x0,	/* 0 */
    0x0,	/* 0 */
    0x0,	/* 0 */
    0x0,	/* 0 */
    0x0,	/* 0 */
    0x0,	/* 0 */
    0x0,	/* 0 */
    0x0,	/* 0 */
    0x0,	/* 0 */
    0x0,	/* 0 */
    0x0	/* 0 */
  };
UNS8 rec_Undefined = 0x0;		/* Mapped at index 0x200B, subindex 0x01 */
UNS8 rec_Undefined = 0x0;		/* Mapped at index 0x200B, subindex 0x02 */
UNS8 rec_Undefined = 0x0;		/* Mapped at index 0x200B, subindex 0x03 */
UNS8 rec_Undefined = 0x0;		/* Mapped at index 0x200B, subindex 0x04 */
UNS8 rec_Undefined = 0x0;		/* Mapped at index 0x200B, subindex 0x05 */

/**************************************************************************/
/* Declaration of value range types                                       */
/**************************************************************************/

#define valueRange_EMC 0x9F /* Type for index 0x1003 subindex 0x00 (only set of value 0 is possible) */
UNS32 CANopen_Master_M200_valueRangeTest (UNS8 typeValue, void * value)
{
  switch (typeValue) {
    case valueRange_EMC:
      if (*(UNS8*)value != (UNS8)0) return OD_VALUE_RANGE_EXCEEDED;
      break;
  }
  return 0;
}

/**************************************************************************/
/* The node id                                                            */
/**************************************************************************/
/* node_id default value.*/
UNS8 CANopen_Master_M200_bDeviceNodeId = 0x00;

/**************************************************************************/
/* Array of message processing information */

const UNS8 CANopen_Master_M200_iam_a_slave = 0;

TIMER_HANDLE CANopen_Master_M200_heartBeatTimers[1] = {TIMER_NONE};

/*
$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$

                               OBJECT DICTIONARY

$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
*/

/* index 0x1000 :   Device Type. */
                    UNS32 CANopen_Master_M200_obj1000 = 0x0;	/* 0 */
                    subindex CANopen_Master_M200_Index1000[] = 
                     {
                       { RO, uint32, sizeof (UNS32), (void*)&CANopen_Master_M200_obj1000, NULL }
                     };

/* index 0x1001 :   Error Register. */
                    UNS8 CANopen_Master_M200_obj1001 = 0x0;	/* 0 */
                    subindex CANopen_Master_M200_Index1001[] = 
                     {
                       { RO, uint8, sizeof (UNS8), (void*)&CANopen_Master_M200_obj1001, NULL }
                     };

/* index 0x1002 :   Manufacturer Status Register. */
                    UNS32 CANopen_Master_M200_obj1002 = 0x0;	/* 0 */
                    subindex CANopen_Master_M200_Index1002[] = 
                     {
                       { RO, uint32, sizeof (UNS32), (void*)&CANopen_Master_M200_obj1002, NULL }
                     };

/* index 0x1003 :   Pre-defined Error Field */
                    UNS8 CANopen_Master_M200_highestSubIndex_obj1003 = 0; /* number of subindex - 1*/
                    UNS32 CANopen_Master_M200_obj1003[] = 
                    {
                      0x0	/* 0 */
                    };
                    subindex CANopen_Master_M200_Index1003[] = 
                     {
                       { RW, valueRange_EMC, sizeof (UNS8), (void*)&CANopen_Master_M200_highestSubIndex_obj1003, NULL },
                       { RO, uint32, sizeof (UNS32), (void*)&CANopen_Master_M200_obj1003[0], NULL }
                     };

/* index 0x1005 :   SYNC COB ID */
                    UNS32 CANopen_Master_M200_obj1005 = 0x0;   /* 0 */

/* index 0x1006 :   Communication / Cycle Period */
                    UNS32 CANopen_Master_M200_obj1006 = 0x0;   /* 0 */

/* index 0x1008 :   Manufacturer Device Name. */
                    UNS8 CANopen_Master_M200_obj1008[10] = "M200";
                    subindex CANopen_Master_M200_Index1008[] = 
                     {
                       { RO, visible_string, 10, (void*)&CANopen_Master_M200_obj1008, NULL }
                     };

/* index 0x1009 :   Manufacturer Hardware Version. */
                    UNS8 CANopen_Master_M200_obj1009[10] = "V2.0.0";
                    subindex CANopen_Master_M200_Index1009[] = 
                     {
                       { RO, visible_string, 10, (void*)&CANopen_Master_M200_obj1009, NULL }
                     };

/* index 0x100A :   Manufacturer Software Version. */
                    UNS8 CANopen_Master_M200_obj100A[10] = "V1.0.0";
                    subindex CANopen_Master_M200_Index100A[] = 
                     {
                       { RO, visible_string, 10, (void*)&CANopen_Master_M200_obj100A, NULL }
                     };

/* index 0x100C :   Guard Time */ 
                    UNS16 CANopen_Master_M200_obj100C = 0x0;   /* 0 */

/* index 0x100D :   Life Time Factor */ 
                    UNS8 CANopen_Master_M200_obj100D = 0x0;   /* 0 */

/* index 0x1014 :   Emergency COB ID */
                    UNS32 CANopen_Master_M200_obj1014 = 0x80 + 0x00;   /* 128 + NodeID */

/* index 0x1016 :   Consumer Heartbeat Time. */
                    UNS8 CANopen_Master_M200_highestSubIndex_obj1016 = 1; /* number of subindex - 1*/
                    UNS32 CANopen_Master_M200_obj1016[] = 
                    {
                      0x11388	/* 70536 */
                    };
                    subindex CANopen_Master_M200_Index1016[] = 
                     {
                       { RO, uint8, sizeof (UNS8), (void*)&CANopen_Master_M200_highestSubIndex_obj1016, NULL },
                       { RW, uint32, sizeof (UNS32), (void*)&CANopen_Master_M200_obj1016[0], NULL }
                     };

/* index 0x1017 :   Producer Heartbeat Time */ 
                    UNS16 CANopen_Master_M200_obj1017 = 0x0;   /* 0 */

/* index 0x1018 :   Identity. */
                    UNS8 CANopen_Master_M200_highestSubIndex_obj1018 = 4; /* number of subindex - 1*/
                    UNS32 CANopen_Master_M200_obj1018_Vendor_ID = 0x0;	/* 0 */
                    UNS32 CANopen_Master_M200_obj1018_Product_Code = 0x0;	/* 0 */
                    UNS32 CANopen_Master_M200_obj1018_Revision_Number = 0x0;	/* 0 */
                    UNS32 CANopen_Master_M200_obj1018_Serial_Number = 0x0;	/* 0 */
                    subindex CANopen_Master_M200_Index1018[] = 
                     {
                       { RO, uint8, sizeof (UNS8), (void*)&CANopen_Master_M200_highestSubIndex_obj1018, NULL },
                       { RO, uint32, sizeof (UNS32), (void*)&CANopen_Master_M200_obj1018_Vendor_ID, NULL },
                       { RO, uint32, sizeof (UNS32), (void*)&CANopen_Master_M200_obj1018_Product_Code, NULL },
                       { RO, uint32, sizeof (UNS32), (void*)&CANopen_Master_M200_obj1018_Revision_Number, NULL },
                       { RO, uint32, sizeof (UNS32), (void*)&CANopen_Master_M200_obj1018_Serial_Number, NULL }
                     };

/* index 0x1280 :   Client SDO 1 Parameter. */
                    UNS8 CANopen_Master_M200_highestSubIndex_obj1280 = 3; /* number of subindex - 1*/
                    UNS32 CANopen_Master_M200_obj1280_COB_ID_Client_to_Server_Transmit_SDO = 0x601;	/* 1537 */
                    UNS32 CANopen_Master_M200_obj1280_COB_ID_Server_to_Client_Receive_SDO = 0x581;	/* 1409 */
                    UNS8 CANopen_Master_M200_obj1280_Node_ID_of_the_SDO_Server = 0x1;	/* 1 */
                    subindex CANopen_Master_M200_Index1280[] = 
                     {
                       { RO, uint8, sizeof (UNS8), (void*)&CANopen_Master_M200_highestSubIndex_obj1280, NULL },
                       { RW, uint32, sizeof (UNS32), (void*)&CANopen_Master_M200_obj1280_COB_ID_Client_to_Server_Transmit_SDO, NULL },
                       { RW, uint32, sizeof (UNS32), (void*)&CANopen_Master_M200_obj1280_COB_ID_Server_to_Client_Receive_SDO, NULL },
                       { RW, uint8, sizeof (UNS8), (void*)&CANopen_Master_M200_obj1280_Node_ID_of_the_SDO_Server, NULL }
                     };

/* index 0x1281 :   Client SDO 2 Parameter. */
                    UNS8 CANopen_Master_M200_highestSubIndex_obj1281 = 3; /* number of subindex - 1*/
                    UNS32 CANopen_Master_M200_obj1281_COB_ID_Client_to_Server_Transmit_SDO = 0x0;	/* 0 */
                    UNS32 CANopen_Master_M200_obj1281_COB_ID_Server_to_Client_Receive_SDO = 0x0;	/* 0 */
                    UNS8 CANopen_Master_M200_obj1281_Node_ID_of_the_SDO_Server = 0x0;	/* 0 */
                    subindex CANopen_Master_M200_Index1281[] = 
                     {
                       { RO, uint8, sizeof (UNS8), (void*)&CANopen_Master_M200_highestSubIndex_obj1281, NULL },
                       { RW, uint32, sizeof (UNS32), (void*)&CANopen_Master_M200_obj1281_COB_ID_Client_to_Server_Transmit_SDO, NULL },
                       { RW, uint32, sizeof (UNS32), (void*)&CANopen_Master_M200_obj1281_COB_ID_Server_to_Client_Receive_SDO, NULL },
                       { RW, uint8, sizeof (UNS8), (void*)&CANopen_Master_M200_obj1281_Node_ID_of_the_SDO_Server, NULL }
                     };

/* index 0x1800 :   Transmit PDO 1 Parameter. */
                    UNS8 CANopen_Master_M200_highestSubIndex_obj1800 = 6; /* number of subindex - 1*/
                    UNS32 CANopen_Master_M200_obj1800_COB_ID_used_by_PDO = 0x185;	/* 389 */
                    UNS8 CANopen_Master_M200_obj1800_Transmission_Type = 0x0;	/* 0 */
                    UNS16 CANopen_Master_M200_obj1800_Inhibit_Time = 0x0;	/* 0 */
                    UNS8 CANopen_Master_M200_obj1800_Compatibility_Entry = 0x0;	/* 0 */
                    UNS16 CANopen_Master_M200_obj1800_Event_Timer = 0x0;	/* 0 */
                    UNS8 CANopen_Master_M200_obj1800_SYNC_start_value = 0x0;	/* 0 */
                    subindex CANopen_Master_M200_Index1800[] = 
                     {
                       { RO, uint8, sizeof (UNS8), (void*)&CANopen_Master_M200_highestSubIndex_obj1800, NULL },
                       { RW, uint32, sizeof (UNS32), (void*)&CANopen_Master_M200_obj1800_COB_ID_used_by_PDO, NULL },
                       { RW, uint8, sizeof (UNS8), (void*)&CANopen_Master_M200_obj1800_Transmission_Type, NULL },
                       { RW, uint16, sizeof (UNS16), (void*)&CANopen_Master_M200_obj1800_Inhibit_Time, NULL },
                       { RW, uint8, sizeof (UNS8), (void*)&CANopen_Master_M200_obj1800_Compatibility_Entry, NULL },
                       { RW, uint16, sizeof (UNS16), (void*)&CANopen_Master_M200_obj1800_Event_Timer, NULL },
                       { RW, uint8, sizeof (UNS8), (void*)&CANopen_Master_M200_obj1800_SYNC_start_value, NULL }
                     };

/* index 0x1A00 :   Transmit PDO 1 Mapping. */
                    UNS8 CANopen_Master_M200_highestSubIndex_obj1A00 = 1; /* number of subindex - 1*/
                    UNS32 CANopen_Master_M200_obj1A00[] = 
                    {
                      0x0	/* 0 */
                    };
                    subindex CANopen_Master_M200_Index1A00[] = 
                     {
                       { RW, uint8, sizeof (UNS8), (void*)&CANopen_Master_M200_highestSubIndex_obj1A00, NULL },
                       { RW, uint32, sizeof (UNS32), (void*)&CANopen_Master_M200_obj1A00[0], NULL }
                     };

/* index 0x200A :   Mapped variable array */
                    UNS8 CANopen_Master_M200_highestSubIndex_obj200A = 50; /* number of subindex - 1*/
                    subindex CANopen_Master_M200_Index200A[] = 
                     {
                       { RO, uint8, sizeof (UNS8), (void*)&CANopen_Master_M200_highestSubIndex_obj200A, NULL },
                       { RW, uint8, sizeof (UNS8), (void*)&array[0], NULL },
                       { RW, uint8, sizeof (UNS8), (void*)&array[1], NULL },
                       { RW, uint8, sizeof (UNS8), (void*)&array[2], NULL },
                       { RW, uint8, sizeof (UNS8), (void*)&array[3], NULL },
                       { RW, uint8, sizeof (UNS8), (void*)&array[4], NULL },
                       { RW, uint8, sizeof (UNS8), (void*)&array[5], NULL },
                       { RW, uint8, sizeof (UNS8), (void*)&array[6], NULL },
                       { RW, uint8, sizeof (UNS8), (void*)&array[7], NULL },
                       { RW, uint8, sizeof (UNS8), (void*)&array[8], NULL },
                       { RW, uint8, sizeof (UNS8), (void*)&array[9], NULL },
                       { RW, uint8, sizeof (UNS8), (void*)&array[10], NULL },
                       { RW, uint8, sizeof (UNS8), (void*)&array[11], NULL },
                       { RW, uint8, sizeof (UNS8), (void*)&array[12], NULL },
                       { RW, uint8, sizeof (UNS8), (void*)&array[13], NULL },
                       { RW, uint8, sizeof (UNS8), (void*)&array[14], NULL },
                       { RW, uint8, sizeof (UNS8), (void*)&array[15], NULL },
                       { RW, uint8, sizeof (UNS8), (void*)&array[16], NULL },
                       { RW, uint8, sizeof (UNS8), (void*)&array[17], NULL },
                       { RW, uint8, sizeof (UNS8), (void*)&array[18], NULL },
                       { RW, uint8, sizeof (UNS8), (void*)&array[19], NULL },
                       { RW, uint8, sizeof (UNS8), (void*)&array[20], NULL },
                       { RW, uint8, sizeof (UNS8), (void*)&array[21], NULL },
                       { RW, uint8, sizeof (UNS8), (void*)&array[22], NULL },
                       { RW, uint8, sizeof (UNS8), (void*)&array[23], NULL },
                       { RW, uint8, sizeof (UNS8), (void*)&array[24], NULL },
                       { RW, uint8, sizeof (UNS8), (void*)&array[25], NULL },
                       { RW, uint8, sizeof (UNS8), (void*)&array[26], NULL },
                       { RW, uint8, sizeof (UNS8), (void*)&array[27], NULL },
                       { RW, uint8, sizeof (UNS8), (void*)&array[28], NULL },
                       { RW, uint8, sizeof (UNS8), (void*)&array[29], NULL },
                       { RW, uint8, sizeof (UNS8), (void*)&array[30], NULL },
                       { RW, uint8, sizeof (UNS8), (void*)&array[31], NULL },
                       { RW, uint8, sizeof (UNS8), (void*)&array[32], NULL },
                       { RW, uint8, sizeof (UNS8), (void*)&array[33], NULL },
                       { RW, uint8, sizeof (UNS8), (void*)&array[34], NULL },
                       { RW, uint8, sizeof (UNS8), (void*)&array[35], NULL },
                       { RW, uint8, sizeof (UNS8), (void*)&array[36], NULL },
                       { RW, uint8, sizeof (UNS8), (void*)&array[37], NULL },
                       { RW, uint8, sizeof (UNS8), (void*)&array[38], NULL },
                       { RW, uint8, sizeof (UNS8), (void*)&array[39], NULL },
                       { RW, uint8, sizeof (UNS8), (void*)&array[40], NULL },
                       { RW, uint8, sizeof (UNS8), (void*)&array[41], NULL },
                       { RW, uint8, sizeof (UNS8), (void*)&array[42], NULL },
                       { RW, uint8, sizeof (UNS8), (void*)&array[43], NULL },
                       { RW, uint8, sizeof (UNS8), (void*)&array[44], NULL },
                       { RW, uint8, sizeof (UNS8), (void*)&array[45], NULL },
                       { RW, uint8, sizeof (UNS8), (void*)&array[46], NULL },
                       { RW, uint8, sizeof (UNS8), (void*)&array[47], NULL },
                       { RW, uint8, sizeof (UNS8), (void*)&array[48], NULL },
                       { RW, uint8, sizeof (UNS8), (void*)&array[49], NULL }
                     };

/* index 0x200B :   Mapped variable rec */
                    UNS8 CANopen_Master_M200_highestSubIndex_obj200B = 5; /* number of subindex - 1*/
                    subindex CANopen_Master_M200_Index200B[] = 
                     {
                       { RO, uint8, sizeof (UNS8), (void*)&CANopen_Master_M200_highestSubIndex_obj200B, NULL },
                       { RW, uint8, sizeof (UNS8), (void*)&rec_Undefined, NULL },
                       { RW, uint8, sizeof (UNS8), (void*)&rec_Undefined, NULL },
                       { RW, uint8, sizeof (UNS8), (void*)&rec_Undefined, NULL },
                       { RW, uint8, sizeof (UNS8), (void*)&rec_Undefined, NULL },
                       { RW, uint8, sizeof (UNS8), (void*)&rec_Undefined, NULL }
                     };

/**************************************************************************/
/* Declaration of pointed variables                                       */
/**************************************************************************/

const indextable CANopen_Master_M200_objdict[] = 
{
  { (subindex*)CANopen_Master_M200_Index1000,sizeof(CANopen_Master_M200_Index1000)/sizeof(CANopen_Master_M200_Index1000[0]), 0x1000},
  { (subindex*)CANopen_Master_M200_Index1001,sizeof(CANopen_Master_M200_Index1001)/sizeof(CANopen_Master_M200_Index1001[0]), 0x1001},
  { (subindex*)CANopen_Master_M200_Index1002,sizeof(CANopen_Master_M200_Index1002)/sizeof(CANopen_Master_M200_Index1002[0]), 0x1002},
  { (subindex*)CANopen_Master_M200_Index1008,sizeof(CANopen_Master_M200_Index1008)/sizeof(CANopen_Master_M200_Index1008[0]), 0x1008},
  { (subindex*)CANopen_Master_M200_Index1009,sizeof(CANopen_Master_M200_Index1009)/sizeof(CANopen_Master_M200_Index1009[0]), 0x1009},
  { (subindex*)CANopen_Master_M200_Index100A,sizeof(CANopen_Master_M200_Index100A)/sizeof(CANopen_Master_M200_Index100A[0]), 0x100A},
  { (subindex*)CANopen_Master_M200_Index1016,sizeof(CANopen_Master_M200_Index1016)/sizeof(CANopen_Master_M200_Index1016[0]), 0x1016},
  { (subindex*)CANopen_Master_M200_Index1018,sizeof(CANopen_Master_M200_Index1018)/sizeof(CANopen_Master_M200_Index1018[0]), 0x1018},
  { (subindex*)CANopen_Master_M200_Index1280,sizeof(CANopen_Master_M200_Index1280)/sizeof(CANopen_Master_M200_Index1280[0]), 0x1280},
  { (subindex*)CANopen_Master_M200_Index1281,sizeof(CANopen_Master_M200_Index1281)/sizeof(CANopen_Master_M200_Index1281[0]), 0x1281},
  { (subindex*)CANopen_Master_M200_Index1800,sizeof(CANopen_Master_M200_Index1800)/sizeof(CANopen_Master_M200_Index1800[0]), 0x1800},
  { (subindex*)CANopen_Master_M200_Index1A00,sizeof(CANopen_Master_M200_Index1A00)/sizeof(CANopen_Master_M200_Index1A00[0]), 0x1A00},
  { (subindex*)CANopen_Master_M200_Index200A,sizeof(CANopen_Master_M200_Index200A)/sizeof(CANopen_Master_M200_Index200A[0]), 0x200A},
  { (subindex*)CANopen_Master_M200_Index200B,sizeof(CANopen_Master_M200_Index200B)/sizeof(CANopen_Master_M200_Index200B[0]), 0x200B},
};

const indextable * CANopen_Master_M200_scanIndexOD (CO_Data *d, UNS16 wIndex, UNS32 * errorCode)
{
	(void)d;
	int i;
	switch(wIndex){
		case 0x1000: i = 0;break;
		case 0x1001: i = 1;break;
		case 0x1002: i = 2;break;
		case 0x1008: i = 3;break;
		case 0x1009: i = 4;break;
		case 0x100A: i = 5;break;
		case 0x1016: i = 6;break;
		case 0x1018: i = 7;break;
		case 0x1280: i = 8;break;
		case 0x1281: i = 9;break;
		case 0x1800: i = 10;break;
		case 0x1A00: i = 11;break;
		case 0x200A: i = 12;break;
		case 0x200B: i = 13;break;
		default:
			*errorCode = OD_NO_SUCH_OBJECT;
			return NULL;
	}
	*errorCode = OD_SUCCESSFUL;
	return &CANopen_Master_M200_objdict[i];
}

/* 
 * To count at which received SYNC a PDO must be sent.
 * Even if no pdoTransmit are defined, at least one entry is computed
 * for compilations issues.
 */
s_PDO_status CANopen_Master_M200_PDO_status[1] = {s_PDO_status_Initializer};

const quick_index CANopen_Master_M200_firstIndex = {
  0, /* SDO_SVR */
  8, /* SDO_CLT */
  0, /* PDO_RCV */
  0, /* PDO_RCV_MAP */
  10, /* PDO_TRS */
  11 /* PDO_TRS_MAP */
};

const quick_index CANopen_Master_M200_lastIndex = {
  0, /* SDO_SVR */
  9, /* SDO_CLT */
  0, /* PDO_RCV */
  0, /* PDO_RCV_MAP */
  10, /* PDO_TRS */
  11 /* PDO_TRS_MAP */
};

const UNS16 CANopen_Master_M200_ObjdictSize = sizeof(CANopen_Master_M200_objdict)/sizeof(CANopen_Master_M200_objdict[0]); 

CO_Data CANopen_Master_M200_Data = CANOPEN_NODE_DATA_INITIALIZER(CANopen_Master_M200);

