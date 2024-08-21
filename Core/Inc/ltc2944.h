/*
 * ltc2944.h
 *
 *  Created on: Dec 5, 2023
 *      Author: rahul
 */

#ifndef INC_LTC2944_H_
#define INC_LTC2944_H_

#include "i2c.h"

#define LTC2944_ADDRESS					(uint8_t)(0b1100100 << 1)	//0x64 in HEX

#define STATUS_REGISTER 				(uint8_t)0x00
#define CONTROL_REGISTER 				(uint8_t)0x01
#define ACCUMULATED_CHARGE_MSB 			(uint8_t)0x02
#define ACCUMULATED_CHARGE_LSB 			(uint8_t)0x03
#define CHARGE_THRESHOLD_HIGH_MSB 		(uint8_t)0x04
#define CHARGE_THRESHOLD_HIGH_LSB 		(uint8_t)0x05
#define CHARGE_THRESHOLD_LOW_MSB 		(uint8_t)0x06
#define CHARGE_THRESHOLD_LOW_LSB 		(uint8_t)0x07
#define VOLTAGE_MSB 					(uint8_t)0x08
#define VOLTAGE_LSB 					(uint8_t)0x09
#define VOLTAGE_THRESHOLD_HIGH_MSB 		(uint8_t)0x0A
#define VOLTAGE_THRESHOLD_HIGH_LSB 		(uint8_t)0x0B
#define VOLTAGE_THRESHOLD_LOW_MSB 		(uint8_t)0x0C
#define VOLTAGE_THRESHOLD_LOW_LSB 		(uint8_t)0x0D
#define CURRENT_MSB 					(uint8_t)0x0E
#define CURRENT_LSB 					(uint8_t)0x0F
#define CURRENT_THRESHOLD_HIGH_MSB 		(uint8_t)0x10
#define CURRENT_THRESHOLD_HIGH_LSB 		(uint8_t)0x11
#define CURRENT_THRESHOLD_LOW_MSB 		(uint8_t)0x12
#define CURRENT_THRESHOLD_LOW_LSB 		(uint8_t)0x13
#define TEMPERATURE_MSB 				(uint8_t)0x14
#define TEMPERATURE_LSB 				(uint8_t)0x15
#define TEMPERATURE_THRESHOLD_HIGH 		(uint8_t)0x16
#define TEMPERATURE_THRESHOLD_LOW 		(uint8_t)0x17

#define NUMBER_OF_REGISTERS		23


/* Status Register (A) */
#define STATUS_RESERVED_BIT                  	7
#define STATUS_CURRENT_ALERT_BIT              	6
#define STATUS_CHARGE_OVERFLOW_UNDERFLOW_BIT  	5
#define STATUS_TEMPERATURE_ALERT_BIT           	4
#define STATUS_CHARGE_ALERT_HIGH_BIT           	3
#define STATUS_CHARGE_ALERT_LOW_BIT            	2
#define STATUS_VOLTAGE_ALERT_BIT               	1
#define STATUS_UNDERVOLTAGE_LOCKOUT_ALERT_BIT 	0


#define FACTOR_CHARGE_QLSB		(float)0.340			// 0.340mAh when, Prescalar = 4096 and Rsns = 50mOhms

/* Control Register (B) */
/*
 *  Coulomb Counter ADC Mode
 */


typedef enum
{
	Sleep_Mode		= 0,	// Put the device on sleep mode
	Manual_Mode		= 1,	// Manual Mode: performing single conversions of voltage, current and temperature then sleep
	Scan_Mode		= 2,	// Scan Mode: performing voltage, current and temperature conversion every 10s
	Automatic_Mode  = 3		// Automatic Mode: continuously performing voltage, current and temperature	conversions
}adc_mode_t;

/*
 * Coulomb Counter prescaling factor M between 1 and 4096.
 */
typedef enum
{
	Factor_1		= 0,	// Prescalar value 1
	Factor_4		= 1,	// Prescalar value 4
	Factor_16		= 2,	// Prescalar value 16
	Factor_64		= 3,	// Prescalar value 64
	Factor_256		= 4,	// Prescalar value 256
	Factor_1024		= 5,	// Prescalar value 1024
	Factor_4096		= 6		// Prescalar value 4096
}prescaler_factor_t;

/*
 * Coulomb Counter ALCC pin configuration
 */
typedef enum
{
	ALCC_Disable 			= 0,	// ALCC Pin Disabled
	ALCC_Charge_Complete 	= 1,	// Pin becomes logic input and accepts charge complete inverted signal
	ALCC_Alert_Mode			= 2,	// Alert functionality enabled. Pin becomes logic output
	ALCC_Reserved			= 3		// Not Allowed
}alcc_mode_t;


/*
 * LTC2944 Configuration Struct
 */
typedef struct
{
	adc_mode_t 			adc_mode;			// This can be any value of enum adc_mode_t
	alcc_mode_t			alcc_mode;			// This can be any value of enum alcc_mode_t
	float				sense_resistor;		// Sense resistor value in milli-ohms
	float 				batt_capacity;		// Known battery capacity in mAhs
	I2C_HandleTypeDef 	i2c_handle;			// Handle to I2C bus used

}ltc2944_configuration_t;


/*
 * LTC2944 internal data
 */
typedef struct
{
	float	acc_charge;				// Acculumated Charge
	float	voltage;				// Input Voltage
	float	current;				// Input Current
	float	temperature;			// LTC2944 ICC Temperature
	float	qLSB;					// The amount of charge represented by the least
									// significant bit (qLSB) of the accumulated charge register
}ltc2944_data_t;


uint8_t LTC2944_Init(ltc2944_configuration_t ltc2944);


HAL_StatusTypeDef LTC2944_Get_Battery_Data(ltc2944_configuration_t *ltc2944);
float LTC2944_Get_Voltage(ltc2944_configuration_t *ltc2944);

float LTC2944_Get_Voltage(ltc2944_configuration_t *ltc2944);

float LTC2944_Get_Current(ltc2944_configuration_t *ltc2944);

float LTC2944_Get_Temperature(ltc2944_configuration_t *ltc2944);

float LTC2944_Get_Charge(ltc2944_configuration_t *ltc2944);




#endif /* INC_LTC2944_H_ */
