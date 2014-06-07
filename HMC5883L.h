//HMC5883L I2C library for ARM STM32F103xx Microcontrollers - Main header file 
//Has bit, byte and buffer I2C R/W functions
// 24/05/2012 by Harinadha Reddy Chintalapalli <harinath.ec@gmail.com>
// Changelog:
//     2012-05-24 - initial release. Thanks to Jeff Rowberg <jeff@rowberg.net> for his AVR/Arduino
//                  based development which inspired me & taken as reference to develop this.
/* ============================================================================================
 HMC5883L device I2C library code for ARM STM32F103xx is placed under the MIT license
 Copyright (c) 2012 Harinadha Reddy Chintalapalli

 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:

 The above copyright notice and this permission notice shall be included in
 all copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 THE SOFTWARE.
 ================================================================================================
 */

#ifndef _HMC5883L_H_
#define _HMC5883L_H_

#ifdef __cplusplus
extern "C"
{
#endif 

/* Includes */
#include "HAL_HMC5883L.h" 

#define HMC5883L_ADDRESS            0x1E // this device only has one address
#define HMC5883L_DEFAULT_ADDRESS    (HMC5883L_ADDRESS<<1)

#define HMC5883L_RA_CONFIG_A        0x00
#define HMC5883L_RA_CONFIG_B        0x01
#define HMC5883L_RA_MODE            0x02
#define HMC5883L_RA_DATAX_H         0x03
#define HMC5883L_RA_DATAX_L         0x04
#define HMC5883L_RA_DATAY_H         0x05
#define HMC5883L_RA_DATAY_L         0x06
#define HMC5883L_RA_DATAZ_H         0x07
#define HMC5883L_RA_DATAZ_L         0x08
#define HMC5883L_RA_STATUS          0x09
#define HMC5883L_RA_ID_A            0x0A
#define HMC5883L_RA_ID_B            0x0B
#define HMC5883L_RA_ID_C            0x0C

#define HMC5883L_CRA_AVERAGE_BIT    6
#define HMC5883L_CRA_AVERAGE_LENGTH 2
#define HMC5883L_CRA_RATE_BIT       4
#define HMC5883L_CRA_RATE_LENGTH    3
#define HMC5883L_CRA_BIAS_BIT       1
#define HMC5883L_CRA_BIAS_LENGTH    2

#define HMC5883L_AVERAGING_1        0x00
#define HMC5883L_AVERAGING_2        0x01
#define HMC5883L_AVERAGING_4        0x02
#define HMC5883L_AVERAGING_8        0x03

#define HMC5883L_RATE_0P75          0x00
#define HMC5883L_RATE_1P5           0x01
#define HMC5883L_RATE_3             0x02
#define HMC5883L_RATE_7P5           0x03
#define HMC5883L_RATE_15            0x04
#define HMC5883L_RATE_30            0x05
#define HMC5883L_RATE_75            0x06

#define HMC5883L_BIAS_NORMAL        0x00
#define HMC5883L_BIAS_POSITIVE      0x01
#define HMC5883L_BIAS_NEGATIVE      0x02

#define HMC5883L_CRB_GAIN_BIT       7
#define HMC5883L_CRB_GAIN_LENGTH    3

#define HMC5883L_GAIN_1370          0x00
#define HMC5883L_GAIN_1090          0x01
#define HMC5883L_GAIN_820           0x02
#define HMC5883L_GAIN_660           0x03
#define HMC5883L_GAIN_440           0x04
#define HMC5883L_GAIN_390           0x05
#define HMC5883L_GAIN_330           0x06
#define HMC5883L_GAIN_220           0x07

#define HMC5883L_MODEREG_BIT        1
#define HMC5883L_MODEREG_LENGTH     2

#define HMC5883L_MODE_CONTINUOUS    0x00
#define HMC5883L_MODE_SINGLE        0x01
#define HMC5883L_MODE_IDLE          0x02

#define HMC5883L_STATUS_LOCK_BIT    1
#define HMC5883L_STATUS_READY_BIT   0

void HMC5883L_Initialize();
bool HMC5883L_TestConnection();

// CONFIG_A register
uint8_t HMC5883L_GetSampleAveraging();
void HMC5883L_SetSampleAveraging(uint8_t averaging);
uint8_t HMC5883L_GetDataRate();
void HMC5883L_SetDataRate(uint8_t rate);
uint8_t HMC5883L_GetMeasurementBias();
void HMC5883L_SetMeasurementBias(uint8_t bias);

// CONFIG_B register
uint8_t HMC5883L_GetGain();
void HMC5883L_SetGain(uint8_t gain);

// MODE register
uint8_t HMC5883L_GetMode();
void HMC5883L_SetMode(uint8_t mode);

// DATA* registers
void HMC5883L_GetHeading(s16* Mag);
// STATUS register
bool HMC5883L_GetLockStatus();
bool HMC5883L_GetReadyStatus();

void HMC5883L_WriteBits(uint8_t slaveAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data);
void HMC5883L_WriteBit(uint8_t slaveAddr, uint8_t regAddr, uint8_t bitNum, uint8_t data);
void HMC5883L_ReadBits(uint8_t slaveAddr, uint8_t regAddr , uint8_t bitStart, uint8_t length, uint8_t *data);
void HMC5883L_ReadBit(uint8_t slaveAddr, uint8_t regAddr, uint8_t bitNum, uint8_t *data);

void HMC5883L_I2C_Init();
void HMC5883L_I2C_ByteWrite(u8 slaveAddr, u8* pBuffer, u8 WriteAddr);
void HMC5883L_I2C_BufferRead(u8 slaveAddr,u8* pBuffer, u8 ReadAddr, u16 NumByteToRead);

#ifdef __cplusplus
}
#endif

#endif /* _HMC5883L_H_ */

/******************* (C) COPYRIGHT 2012 Harinadha Reddy Chintalapalli *****END OF FILE****/
