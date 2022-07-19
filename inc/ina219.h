/*============================================================================
 * Autor: Luciano Perren <lgperren@gmail.com>
 * TP Final Protocolos de Comunicación en Sistemas Embebidos
 * Date: 2019/08/07
 * Docentes: Pablo Gomez y Eric Pernia
 * ina219.h
 *===========================================================================*/

#ifndef MIS_PROGRAMAS_INA219_INC_INA219_H_
#define MIS_PROGRAMAS_INA219_INC_INA219_H_

/*==================[inclusions]=============================================*/

#include "sapi.h"

/*==================[macros]=================================================*/

// INA219 registers
#define INA219_CONFIGURATION						0x00	// Configuration Register (R/W)
#define INA219_SHUNT_VOLTAGE_OUT					0x01	// Shunt Voltage Register (R)
#define INA219_BUS_VOLTAGE_OUT						0x02	// Bus Voltage Register (R)
#define INA219_POWER_OUT							0x03	// Power Register (R)
#define INA219_CURRENT_OUT							0x04	// Current Register (R)
#define INA219_CALIBRATION							0x05	// Calibration Register (R/W)

// I2C baudrate
#define INA219_I2C_RATE				             	400000  // 400 kHz

// I2C address
typedef enum {
INA219_ADDRESS_0  = 0x40,						 	// 1000000 (A1 = A0 = GND)
INA219_ADDRESS_1  = 0x41,							// 1000001 (A1 = GND, A0 = Vs)
INA219_ADDRESS_2  = 0x42,							// 1000010 (A1 = GND, A0 = SDA)
INA219_ADDRESS_3  = 0x43,							// 1000011 (A1 = GND, A0 = SCL)
INA219_ADDRESS_4  = 0x44,						 	// 1000100 (A1 = Vs, A0 = GND)
INA219_ADDRESS_5  = 0x45,							// 1000101 (A1 = A0 = Vs)
INA219_ADDRESS_6  = 0x46,							// 1000110 (A1 = Vs, A0 = SDA)
INA219_ADDRESS_7  = 0x47,						 	// 1000111 (A1 = Vs, A0 = SCL)
INA219_ADDRESS_8  = 0x48,							// 1001000 (A1 = SDA, A0 = GND)
INA219_ADDRESS_9  = 0x49,							// 1001001 (A1 = SDA, A0 = Vs)
INA219_ADDRESS_10 = 0x4A,							// 1001010 (A1 = A0 = SDA)
INA219_ADDRESS_11 =	0x4B,							// 1001011 (A1 = SDA, A0 = SCL)
INA219_ADDRESS_12 = 0x4C,							// 1001100 (A1 = SCL, A0 = GND)
INA219_ADDRESS_13 =	0x4D,						    // 1001101 (A1 = SCL, A0 = Vs)
INA219_ADDRESS_14 = 0x4E,							// 1001110 (A1 = SCL, A0 = SDA)
INA219_ADDRESS_15 = 0x4F							// 1001111 (A1 = A0 = SCL)
} INA219_address_t;

// Bus voltage range values -> bit 13 (BRNG)
#define	INA219_CONFIG_BRNG_16V						0x0		// 0-16V Range
#define	INA219_CONFIG_BRNG_32V						0x1		// 0-32V Range

// Values for gain bits -> bit 12-11 (PG1-PG0)
#define	INA219_CONFIG_GAIN_40MV						0x0		// Gain 1, 40mV Range
#define	INA219_CONFIG_GAIN_80MV						0x1		// Gain /2, 80mV Range
#define	INA219_CONFIG_GAIN_160MV					0x2		// Gain /4, 160mV Range
#define	INA219_CONFIG_GAIN_320MV					0x3		// Gain /8, 320mV Range

// Values for bus ADC resolution -> bit 10-9-8-7 (BADC4-BADC3-BADC2-BADC1)
#define	INA219_CONFIG_BADC_9BIT_1S 					0x0		// 1 sample x 9-bits bus resolution = 0..511
#define	INA219_CONFIG_BADC_10BIT_1S					0x1    	// 1 sample x 10-bits bus resolution = 0..1023
#define	INA219_CONFIG_BADC_11BIT_1S					0x2    	// 1 sample x 11-bits bus resolution = 0..2047
#define	INA219_CONFIG_BADC_12BIT_1S   				0x3    	// 1 sample x 12-bits bus resolution = 0..4095 (default)
#define	INA219_CONFIG_BADC_12BIT_2S     			0x9 	// 2 samples x 12-bits bus resolution
#define	INA219_CONFIG_BADC_12BIT_4S   				0xA		// 4 samples x 12-bits bus resolution
#define	INA219_CONFIG_BADC_12BIT_8S   				0xB		// 8 samples x 12-bits bus resolution
#define	INA219_CONFIG_BADC_12BIT_16S  				0xC		// 16 samples x 12-bits bus resolution
#define	INA219_CONFIG_BADC_12BIT_32S  				0xD		// 32 samples x 12-bits bus resolution
#define	INA219_CONFIG_BADC_12BIT_64S  				0xE		// 64 samples x 12-bits bus resolution
#define	INA219_CONFIG_BADC_12BIT_128S 				0xF 	// 128 samples x 12-bits bus resolution

// Values for shunt ADC resolution -> bit 6-5-4-3 (SADC4-SADC3-SADC2-SADC1)
#define	INA219_CONFIG_SADC_9BIT_1S    				0x0		// 1 sample x 9-bits shunt resolution
#define	INA219_CONFIG_SADC_10BIT_1S   				0x1		// 1 sample x 10-bits shunt resolution
#define	INA219_CONFIG_SADC_11BIT_1S   				0x2		// 1 sample x 11-bits shunt resolution
#define	INA219_CONFIG_SADC_12BIT_1S   				0x3		// 1 sample x 12-bits shunt resolution (default)
#define	INA219_CONFIG_SADC_12BIT_2S   				0x9		// 2 samples x 12-bits shunt resolution
#define	INA219_CONFIG_SADC_12BIT_4S   				0xA		// 4 samples x 12-bits shunt resolution
#define	INA219_CONFIG_SADC_12BIT_8S   				0xB		// 8 samples x 12-bits shunt resolution
#define	INA219_CONFIG_SADC_12BIT_16S  				0xC		// 16 samples x 12-bits shunt resolution
#define	INA219_CONFIG_SADC_12BIT_32S  				0xD		// 32 samples x 12-bits shunt resolution
#define	INA219_CONFIG_SADC_12BIT_64S  				0xE		// 64 samples x 12-bits shunt resolution
#define	INA219_CONFIG_SADC_12BIT_128S 				0xF		// Power downIG_SADC_12BIT_128S	// 128 samples x 12-bits shunt resolution

// Operating Mode -> bit 2-1-0 (MODE3-MODE2-MODE1)
#define	INA219_CONFIG_MODE_POWERDOWN				0x0		// Power down
#define	INA219_CONFIG_MODE_SVOLT_TRIGGERED			0x1		// Shunt voltage triggered
#define	INA219_CONFIG_MODE_BVOLT_TRIGGERED			0x2		// Bus voltage triggered
#define	INA219_CONFIG_MODE_SANDBVOLT_TRIGGERED		0x3		// Shunt and bus voltage triggered
#define	INA219_CONFIG_MODE_ADCOFF					0x4		// ADC off (disables)
#define	INA219_CONFIG_MODE_SVOLT_CONTINUOUS			0x5		// Shunt voltage continuous
#define	INA219_CONFIG_MODE_BVOLT_CONTINUOUS			0x6		// Bus voltage continuous
#define	INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS		0x7		// Shunt and bus voltage continuous

/*==================[typedef]================================================*/

//Control structure for INA219 operation
typedef struct {
	INA219_address_t address;

	//Configuration Register
	//uint8_t RST_bit;			// Reset bit (bit 15)
	uint8_t BRNG_bit;			// Bus voltage range (bit 13)
	uint8_t PG_bits;			// Gain and range (bit 12 y 11)
	uint8_t BADC_bits;			// ADC resolution/averaging (bit 10, 9, 8 y 7)
	uint8_t SADC_bits;			// Shunt ADC resolution/averaging (bit 6, 5, 4 y 3)
	uint8_t MODE_bits;			// Operating mode (bit 2, 1 y 0)

	float shunt_volt;			// Variable donde se almacena el valor a mostrar de "shunt voltage"
	float bus_volt;				// Variable donde se almacena el valor a mostrar de "bus voltage"
	float power;				// Variable donde se almacena el valor a mostrar de "power"
	float current;				// Variable donde se almacena el valor a mostrar de "current"

} INA219_Control_t;

/*==================[external functions declaration]=========================*/

// Initialize INA219
bool_t ina219Init( INA219_address_t address );

// Función que realiza el casteo a float de los registros leídos
bool_t ina219Read();

// Returns the shunt voltage measurement
float ina219ShowShuntVoltage( void );

// Returns the bus voltage measurement
float ina219ShowBusVoltage( void );

// Returns the power calculated
float ina219ShowPower( void );

// Returns the current calculated
float ina219ShowCurrent( void );

/*==================[end of file]============================================*/
#endif /* MIS_PROGRAMAS_INA219_INC_INA219_H_ */
