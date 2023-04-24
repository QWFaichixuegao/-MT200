/*
This file is part of CanFestival, a library implementing CanOpen Stack.

Copyright (C): Edouard TISSERANT and Francis DUPIN
AVR Port: Andreas GLAUSER and Peter CHRISTEN

See COPYING file for copyrights details.

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*/


#ifndef __CAN_CANFESTIVAL__
#define __CAN_CANFESTIVAL__

#include "applicfg.h"
#include "data.h"
#include "can.h"

// ---------  to be called by user app ---------
void initTimer(void);
UNS8 canSend(CAN_PORT notused, Message *m);
UNS8 canChangeBaudRate(CAN_PORT port, char* baud);
void setTimer(TIMEVAL value);
TIMEVAL getElapsedTime(void);
void canReceive_Callback(CAN_RxHeaderTypeDef *pHeader,uint8_t* msg);
void timerForCan(void);

void heartbeat_error(CO_Data* d, UNS8 heartbeatID);
void post_SlaveBootup(CO_Data* d, UNS8 heartbeatID);
//UNS8 ReadSDO(CO_Data* d, UNS8 nodeId, UNS16 index, UNS8 subIndex, UNS8 dataType, void* data, UNS32* size);
UNS8 ReadSDO(CO_Data* d, UNS8 nodeId, UNS16 index, UNS8 subIndex, UNS8 dataType, void* data, UNS32* size, UNS8 useBlockMode);
UNS8 WriteSDO(CO_Data* d, UNS8 nodeId, UNS16 index, UNS8 subIndex, UNS32 count, UNS8 dataType, void* data, UNS8 useBlockMode);
void carDeviceStart(void);
UNS8 carDeviceCheck(void);

#endif
