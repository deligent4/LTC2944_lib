/*
 * ltc2944.c
 *
 *  Created on: Dec 5, 2023
 *      Author: rahul
 */

#include "ltc2944.h"
#include "i2c.h"

ltc2944_data_t ltc2944_data;
float Perscaler_Table[] = {1.0, 4.0, 16.0, 64.0, 256.0, 1024.0, 4096.0};
//
void LTC2944_Init(ltc2944_configuration_t ltc2944){
	uint8_t ctrl_reg = 0;
	float prescalar_value;
	uint8_t status;

	ctrl_reg |= ltc2944.adc_mode << 6;
	ctrl_reg |= ltc2944.prescalar_mode << 3;
	ctrl_reg |= ltc2944.alcc_mode << 1;

	status = HAL_I2C_Mem_Write(&(ltc2944.i2c_handle), LTC2944_ADDRESS,
			CONTROL_REGISTER, 1, &ctrl_reg, 1, HAL_MAX_DELAY);

	if(status == HAL_OK){
		prescalar_value = Perscaler_Table[ltc2944.prescalar_mode];
		ltc2944_data.qLSB = FACTOR_CHARGE_QLSB * (0.05 / ltc2944.sense_resistor)
							* (prescalar_value / 4096.0);
	}
}

uint8_t LTC2944_Get_Battery_Data(ltc2944_configuration_t *ltc2944){
	uint16_t temp;
	uint8_t status;
	uint8_t data_buffer[NUMBER_OF_REGISTERS];

	status = HAL_I2C_Mem_Read(&(ltc2944->i2c_handle), LTC2944_ADDRESS, STATUS_REGISTER,
			1 , data_buffer, NUMBER_OF_REGISTERS, HAL_MAX_DELAY);

	if(status == HAL_OK){

		temp = (data_buffer[ACCUMULATED_CHARGE_MSB]) << 8 | (data_buffer[ACCUMULATED_CHARGE_LSB]);
		ltc2944_data.acc_charge = ltc2944_data.qLSB * temp;

		temp = (data_buffer[VOLTAGE_MSB] << 8) | (data_buffer[VOLTAGE_LSB]);
		ltc2944_data.voltage = 70.8 * (float)(temp/65535.0);

		temp = (data_buffer[CURRENT_MSB] << 8) | (data_buffer)[CURRENT_LSB];
		ltc2944_data.current = (0.064 / ltc2944->sense_resistor) * ((temp - 32767.0) / 32767.0);

		temp = (data_buffer[TEMPERATURE_MSB] << 8) | (data_buffer[TEMPERATURE_LSB]);
		ltc2944_data.temperature = (501 * (float)(temp / 65535)) - 273;
	}

	return status;
}


float LTC2944_Get_Voltage(ltc2944_configuration_t *ltc2944){
	uint8_t status = LTC2944_Get_Battery_Data(ltc2944);
	if(status == HAL_OK){
		float data = ltc2944_data.voltage;
		return data;
	}else{
		return 0;
	}
}

float LTC2944_Get_Current(ltc2944_configuration_t *ltc2944){
	uint8_t status = LTC2944_Get_Battery_Data(ltc2944);
	if(status == HAL_OK){
		float data = ltc2944_data.current;
		return data;
	}else{
		return 0;
	}
}


float LTC2944_Get_Temperature(ltc2944_configuration_t *ltc2944){
	uint8_t status = LTC2944_Get_Battery_Data(ltc2944);
	if(status == HAL_OK){
		float data = ltc2944_data.temperature;
		return data;
	}else{
		return 0;
	}
}

float LTC2944_Get_Charge(ltc2944_configuration_t *ltc2944){
	uint8_t status = LTC2944_Get_Battery_Data(ltc2944);
	if(status == HAL_OK){
		float data = ltc2944_data.acc_charge;
		return data;
	}else{
		return 0;
	}
}




