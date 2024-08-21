/*
 * ltc2944.c
 *
 *  Created on: Dec 5, 2023
 *      Author: rahul
 */

#include "ltc2944.h"
#include "i2c.h"
#include "stdio.h"

ltc2944_data_t ltc2944_data;
uint16_t Prescaler_Table[] = {1, 4, 16, 64, 256, 1024, 4096};
//uint16_t prescaler_value;

uint8_t LTC2944_Init(ltc2944_configuration_t ltc2944){
	uint8_t ctrl_reg = 0;
	uint8_t prescaler_value;
	float psc_temp;
	uint8_t status;

	// calculate prescalar value based on the battery parameters
	psc_temp = (4096.0f * ltc2944.batt_capacity * ltc2944.sense_resistor)
					/ (65535.0f * FACTOR_CHARGE_QLSB * 50.0f);
	// loop through the Prescaler_Table[] to find the best match of
	// prescalar based on battery and sense resistor values.
	for(uint8_t i = 0; i <= sizeof(Prescaler_Table); i++){
		if((uint16_t)psc_temp <= Prescaler_Table[i]){
			if(i != 0){
				if((uint16_t)psc_temp >= Prescaler_Table[i-1]);
				prescaler_value = Prescaler_Table[i];
				break;			// break from the for loop when a
								// match is found, otherwise it will
								// loop through the whole table
			}
			else{
				prescaler_value = 1;
			}
		}
	}

	// calculate the amount of charge represented by the
	// least significant bit (qLSB) of the accumulated charge registers
	ltc2944_data.qLSB = FACTOR_CHARGE_QLSB * (50.0f / ltc2944.sense_resistor)
								* (prescaler_value / 4096.0f);

	// calculate the value to put in the control register
	ctrl_reg |= ltc2944.adc_mode << 6;
	ctrl_reg |= (uint8_t)prescaler_value << 3;
	ctrl_reg |= ltc2944.alcc_mode << 1;

	status = HAL_I2C_Mem_Write(&(ltc2944.i2c_handle), LTC2944_ADDRESS,
			CONTROL_REGISTER, 1, &ctrl_reg, 1, 10);

	return status;
}


HAL_StatusTypeDef LTC2944_Get_Battery_Data(ltc2944_configuration_t *ltc2944){
	uint16_t temp;
	uint8_t status;
	uint8_t data_buffer[NUMBER_OF_REGISTERS];

	status = HAL_I2C_Mem_Read(&(ltc2944->i2c_handle), LTC2944_ADDRESS, STATUS_REGISTER,
			1 , data_buffer, NUMBER_OF_REGISTERS, 1);

	if(status == HAL_OK){

		temp = (data_buffer[ACCUMULATED_CHARGE_MSB]) << 8 | (data_buffer[ACCUMULATED_CHARGE_LSB]);
		ltc2944_data.acc_charge = ltc2944_data.qLSB * temp;

		temp = (data_buffer[VOLTAGE_MSB] << 8) | (data_buffer[VOLTAGE_LSB]);
		ltc2944_data.voltage = 70.8f * (temp/65535.0f);

		temp = (data_buffer[CURRENT_MSB] << 8) | (data_buffer)[CURRENT_LSB];
		ltc2944_data.current = ((((float)temp / 32767.0f) - 1) * (64.0f / ltc2944->sense_resistor));

		temp = (data_buffer[TEMPERATURE_MSB] << 8) | (data_buffer[TEMPERATURE_LSB]);
		ltc2944_data.temperature = (501.0f * (temp / 65535.0f)) - 273.0f;
	}
	else{
		ltc2944_data.acc_charge =	0;
		ltc2944_data.current = 		0;
		ltc2944_data.qLSB = 		0;
		ltc2944_data.temperature = 	0;
		ltc2944_data.voltage = 		0;
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






