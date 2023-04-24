#include "at24cxx.h"
/** 描述  ：在EEPROM的一个写循环中可以写多个字节，但一次写入的字节数
	*         不能超过EEPROM页的大小。
	* 输入  ：
	*         -MemAddress 接收数据的EEPROM的地址（是WORD ADDRESS，当占用Device Address时需将占用的位与出）
	*					-pBuffer 		要写入EEPROM数据的指针缓冲区
	*         -BufferSize 要写入EEPROM的字节数
	*/
HAL_StatusTypeDef AT24CXX_PageWrite(uint16_t MemAddress, uint8_t* pBuffer, uint32_t BufferSize)
{
	uint8_t dev_addr;//EEPROM的设备地址    
	HAL_StatusTypeDef status = HAL_OK;
																			//16位字节地址，3字节发送。
//FIRST WORD ADDRESS中存放的是  	1     0     1    0     A2   A1   A0   0。
//SECOND WORD ADDRESS中存放的是	a15   a14   a13  a12    a11  a10   a9  a8。
//Device Address中存放的是 			 a7    a6    a5   a4     a3   a2   a1  a0。
	dev_addr = W_IIC_ADDR_AT24CXX | ((A2A1A0 << 1) & 0x0E);     
	status = HAL_I2C_Mem_Write_DMA(&hi2c1, dev_addr, (uint16_t)MemAddress, I2C_MEMADD_SIZE_16BIT, pBuffer, BufferSize);
	while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY)
	{}
	/* Check if the EEPROM is ready for a new operation */
	while (HAL_I2C_IsDeviceReady(&hi2c1, dev_addr, 100, 100) == HAL_TIMEOUT);
	return status;
}

HAL_StatusTypeDef AT24CXX_WriteData(uint16_t MemAddress, uint8_t* pBuffer, uint32_t BufferSize)
{
	uint8_t NumOfPage = 0, NumOfSingle = 0, Addr = 0, count = 0;
	HAL_StatusTypeDef err = HAL_OK;

	Addr = MemAddress % AT24CXX_PAGE_SIZE;//看写入地址是不是128的整数倍等于0的话起始地址刚好是页开始地址，不为0则起始地址偏离页开始地址
	count = AT24CXX_PAGE_SIZE - Addr;//若起始地址偏离页开始地址，则计算出从偏离地址开始到该页末尾可写的剩余字节数
	NumOfPage =  BufferSize / AT24CXX_PAGE_SIZE;//看写入的数据是不是128的整数倍等于0的话<=128
	NumOfSingle = BufferSize % AT24CXX_PAGE_SIZE;//剩余小于一页的字节数，(起始地址刚好是页开始地址写在最末尾的一页，起始地址偏离页开始地址写在开头)
 
	//对于起始地址，有两种情况，分别判断
	if(Addr == 0)//如果写入地址与页地址 对齐 则起始地址刚好是页开始地址
		{
			//对于写入的字节数，有两种情况，分别判断 
			if(NumOfPage == 0)//如果写入大小 小于等于 页大小 
			{
				//////  写入页  //////
				return (AT24CXX_PageWrite(MemAddress,pBuffer,BufferSize));
			}
			else/*如果写入大小 大于 页大小*/
			{
				/*-------写整页--------*/
				while(NumOfPage--)
				{
					err = AT24CXX_PageWrite(MemAddress,pBuffer,AT24CXX_PAGE_SIZE);
					if(err != HAL_OK)
						return err; 
					MemAddress +=  AT24CXX_PAGE_SIZE;
					pBuffer += AT24CXX_PAGE_SIZE;
				}
				/*------写最后一页------*/
				if(NumOfSingle!=0)
				{
					return (AT24CXX_PageWrite(MemAddress,pBuffer,NumOfSingle)); 
				}
			}
		}
	else//如果写入地址与页地址 不对齐 则起始地址偏离页开始地址
		{
			//对于写入的字节数，有两种情况，分别判断
			if(BufferSize <= count)//如果写入大小 小于等于 页剩余大小
			{
				//////  写入页  //////
				return (AT24CXX_PageWrite(MemAddress,pBuffer,NumOfSingle));
			}
			else/*如果写入大小 大于 页剩余大小*/
			{
				BufferSize -= count;
				NumOfPage =  BufferSize / AT24CXX_PAGE_SIZE;
				NumOfSingle = BufferSize % AT24CXX_PAGE_SIZE;	
				/*-------先写当前页--------*/
				err = AT24CXX_PageWrite(MemAddress,pBuffer,count);
				if(err != HAL_OK)
					return err; 
				MemAddress += count;
				pBuffer += count;
				/*-------写中间整页--------*/
				while(NumOfPage--)
				{
					err = AT24CXX_PageWrite(MemAddress,pBuffer,AT24CXX_PAGE_SIZE);
					if(err != HAL_OK)
						return err;
					MemAddress +=  AT24CXX_PAGE_SIZE;
					pBuffer += AT24CXX_PAGE_SIZE;  
				}
				/*-------写最后一页--------*/
				if(NumOfSingle != 0)
				{
					return (AT24CXX_PageWrite(MemAddress,pBuffer,NumOfSingle)); 
				}
			}
		}
	return err;
}

