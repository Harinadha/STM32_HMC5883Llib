//HMC5883L I2C library for ARM STM32F103xx Microcontrollers - Header file has defines
// to choose I2C peripheral, speed & pins. 
// 24/05/2012 by Harinadha Reddy Chintalapalli <harinath.ec@gmail.com>
// Changelog:
//     2012-05-24 - initial release.
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

/* Define to prevent recursive inclusion*/
#ifndef __HAL_HMC5883L_H
#define __HAL_HMC5883L_H

#ifdef __cplusplus
extern "C" {
#endif 

/* Includes */
#include "stm32f10x.h"

/**
 * @addtogroup  HMC5883L_I2C_Define
 * @{
 */

#define HMC5883L_I2C                  I2C2
#define HMC5883L_I2C_RCC_Periph       RCC_APB1Periph_I2C2
#define HMC5883L_I2C_Port             GPIOB
#define HMC5883L_I2C_SCL_Pin          GPIO_Pin_10
#define HMC5883L_I2C_SDA_Pin          GPIO_Pin_11
#define HMC5883L_I2C_RCC_Port         RCC_APB2Periph_GPIOB
#define HMC5883L_I2C_Speed            100000

/**
 *@}
 *//* end of group HMC5883L_I2C_Define */

#ifdef __cplusplus
}
#endif

#endif /* __HAL___HMC5883L_H */

/******************* (C) COPYRIGHT 2012 Harinadha Reddy Chintalapalli *****END OF FILE****/
