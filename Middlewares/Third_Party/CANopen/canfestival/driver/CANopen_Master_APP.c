


//SDO读canopen从站数据
UNS8 ReadSDO(UNS8 nodeId, UNS16 index, UNS8 subIndex, UNS8 dataType, void* data, UNS32* size)
{
	UNS32 abortCode = 0;
	UNS8 res = SDO_UPLOAD_IN_PROGRESS;
	// Read SDO
	UNS8 err = readNetworkDict(&TestMaster_Data, nodeId, index, subIndex, dataType, 0);
	if (err)
		return 0xFF;
	for (;;)
	{
		res = getReadResultNetworkDict(&TestMaster_Data, nodeId, data, size, &abortCode);
        //printf("sendsdo res = %x\n", res);
		if (res != SDO_UPLOAD_IN_PROGRESS)
			break;
		sleep_proc(1);
		continue;
	}
	closeSDOtransfer(&TestMaster_Data, nodeId, SDO_CLIENT);
	if (res == SDO_FINISHED)
		return 0;
	return 0xFF;
}

// SDO写canopen从站数据
UNS8 WriteSDO(UNS8 nodeId, UNS16 index, UNS8 subIndex, UNS32 count, UNS8 dataType, void* data, UNS8 useBlockMode)
{
    UNS32 abortCode = 0;
    UNS8 res = SDO_DOWNLOAD_IN_PROGRESS;
    // Write SDO
    UNS8 err = writeNetworkDict(&TestMaster_Data, nodeId, index, subIndex, count, dataType, data, useBlockMode);
    if (err)
        return 0xFF;
    for (;;)
    {
        res = getWriteResultNetworkDict(&TestMaster_Data, nodeId, &abortCode);
        //printf("write res = %x\n", res);
        if (res != SDO_DOWNLOAD_IN_PROGRESS)
            break;
        sleep_proc(1);
        continue;
    }
    closeSDOtransfer(&TestMaster_Data, nodeId, SDO_CLIENT);
    
    if (res == SDO_FINISHED)
        return 0;
    return 0xFF;
}
