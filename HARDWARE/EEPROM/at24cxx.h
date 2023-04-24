#ifndef __AT24CXX_H_
#define __AT24CXX_H_

#include "i2c.h"

#define W_IIC_ADDR_AT24CXX  0xA0	//1 0 1 0 0  0  0  0 = 0xA0
#define R_IIC_ADDR_AT24CXX  0xA1	//1 0 1 0 0  0  0  1 = 0xA1 
#define A2A1A0            0   		//硬件上三个引脚接了地

#define I2C_TIMEOUT  			200				//100 /*<! Value of Timeout when I2C communication fails */

#define AT24CXX_MAX_SIZE  0x10000   // AT24C512含有512K/8即64K个字节

#define AT24CXX_PAGE_SIZE 128   			// AT24C512页写缓冲器大小为128Byte

#define WR_DATA_LEN 50

#define DEV_VERSION_ADD 0x0180     //版本号开始地址 5Byte(1.0.1)


typedef struct
{
	uint8_t Mileage[4];
	uint8_t RunTime[4];
	uint8_t WaterPumpTime[4];
	uint8_t FanMachineryTime[4];
} EEPROM_PACK;
extern EEPROM_PACK eeprom_packet;
HAL_StatusTypeDef AT24CXX_PageWrite(uint16_t MemAddress, uint8_t* pBuffer, uint32_t BufferSize);
HAL_StatusTypeDef AT24CXX_WriteData(uint16_t MemAddress, uint8_t* pBuffer, uint32_t BufferSize);
uint8_t AT24CXX_Check(void);

#endif
/*20
1
15
1
4
*/
