/*******************************************************************************
* Copyright (c) 2020, STMicroelectronics - All Rights Reserved
*
* This file is part of the VL53L7CX Ultra Lite Driver and is dual licensed,
* either 'STMicroelectronics Proprietary license'
* or 'BSD 3-clause "New" or "Revised" License' , at your option.
*
********************************************************************************
*
* 'STMicroelectronics Proprietary license'
*
********************************************************************************
*
* License terms: STMicroelectronics Proprietary in accordance with licensing
* terms at www.st.com/sla0081
*
* STMicroelectronics confidential
* Reproduction and Communication of this document is strictly prohibited unless
* specifically authorized in writing by STMicroelectronics.
*
*
********************************************************************************
*
* Alternatively, the VL53L7CX Ultra Lite Driver may be distributed under the
* terms of 'BSD 3-clause "New" or "Revised" License', in which case the
* following provisions apply instead of the ones mentioned above :
*
********************************************************************************
*
* License terms: BSD 3-clause "New" or "Revised" License.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice, this
* list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright notice,
* this list of conditions and the following disclaimer in the documentation
* and/or other materials provided with the distribution.
*
* 3. Neither the name of the copyright holder nor the names of its contributors
* may be used to endorse or promote products derived from this software
* without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*
*******************************************************************************/

#include "vl53l7cx_platform.h"


uint8_t vl53l7cx_readByte(VL53L7CX_Platform *p_platform, uint16_t RegisterAdress, uint8_t *p_value)
{
	uint8_t status = 0;
	uint8_t data_write[2];
	uint8_t data_read[1];

	data_write[0] = (RegisterAdress >> 8) & 0xFF;
	data_write[1] = RegisterAdress & 0xFF;
	status = HAL_I2C_Master_Transmit(&p_platform->i2c, p_platform->address, data_write, 2, 100);
	status = HAL_I2C_Master_Receive(&p_platform->i2c, p_platform->address, data_read, 1, 100);
	*p_value = data_read[0];
  
	return status;
}

uint8_t vl53l7cx_writeByte(VL53L7CX_Platform *p_platform, uint16_t RegisterAdress, uint8_t value)
{
	uint8_t data_write[3];
	uint8_t status = 0;

	data_write[0] = (RegisterAdress >> 8) & 0xFF;
	data_write[1] = RegisterAdress & 0xFF;
	data_write[2] = value & 0xFF;
	status = HAL_I2C_Master_Transmit(&p_platform->i2c,p_platform->address, data_write, 3, 100);

	return status;
}


uint8_t vl53l7cx_writeMulti(VL53L7CX_Platform *p_platform, uint16_t RegisterAdress, uint8_t *p_values, uint32_t size)
{
	uint8_t status = HAL_I2C_Mem_Write(&p_platform->i2c, p_platform->address, RegisterAdress, I2C_MEMADD_SIZE_16BIT, p_values, size, 65535);

	return status;
}

uint8_t vl53l7cx_readMulti(VL53L7CX_Platform *p_platform, uint16_t RegisterAdress, uint8_t *p_values, uint32_t size)
{
	uint8_t status;
	uint8_t data_write[2];
	data_write[0] = (RegisterAdress>>8) & 0xFF;
	data_write[1] = RegisterAdress & 0xFF;
	status = HAL_I2C_Master_Transmit(&p_platform->i2c, p_platform->address, data_write, 2, 100);
	status += HAL_I2C_Master_Receive(&p_platform->i2c, p_platform->address, p_values, size, 100);

	return status;
}

uint8_t vl53l7cx_resetSensor(VL53L7CX_Platform *p_platform)
{
    HAL_GPIO_WritePin(p_platform->rsc_io.port_name, p_platform->rsc_io.pin_number, GPIO_PIN_RESET);
    vl53l7cx_waitMs(p_platform, 100);
    HAL_GPIO_WritePin(p_platform->rsc_io.port_name, p_platform->rsc_io.pin_number, GPIO_PIN_SET);
	vl53l7cx_waitMs(p_platform, 100);
	HAL_GPIO_WritePin(p_platform->rsc_io.port_name, p_platform->rsc_io.pin_number, GPIO_PIN_RESET);
	vl53l7cx_waitMs(p_platform, 100);

	return 0;
}

uint8_t vl53l7cx_enableLP(VL53L7CX_Platform *p_platform)
{
    HAL_GPIO_WritePin(p_platform->lpn_io.port_name, p_platform->lpn_io.pin_number, GPIO_PIN_SET);

	return 0;
}

uint8_t vl53l7cx_disableLP(VL53L7CX_Platform *p_platform)
{
	HAL_GPIO_WritePin(p_platform->lpn_io.port_name, p_platform->lpn_io.pin_number, GPIO_PIN_RESET);

	return 0;
}

uint8_t vl53l7cx_getINTstatus(VL53L7CX_Platform *p_platform)
{
	return HAL_GPIO_ReadPin(p_platform->int_io.port_name, p_platform->int_io.pin_number);
}


void vl53l7cx_swapBuffer(uint8_t *buffer, uint16_t size)
{
	uint32_t i, tmp;

	/* Example of possible implementation using <string.h> */
	for(i = 0; i < size; i = i + 4)
	{
		tmp = (
		  buffer[i]<<24)
		|(buffer[i+1]<<16)
		|(buffer[i+2]<<8)
		|(buffer[i+3]);

		memcpy(&(buffer[i]), &tmp, 4);
	}
}

uint8_t vl53l7cx_waitMs(VL53L7CX_Platform *p_platform, uint32_t TimeMs)
{
	HAL_Delay(TimeMs);
	return 0;
}
