/*============================================================================
 * Autor: Luciano Perren <lgperren@gmail.com>
 * TP Final Protocolos de Comunicación en Sistemas Embebidos
 * Date: 2019/08/07
 * Docentes: Pablo Gomez y Eric Pernia
 * ina219.c
 *===========================================================================*/

/*==================[inlcusiones]============================================*/

#include "../inc/ina219.h"
#include "sapi.h"
#include "sapi_i2c.h"

/*================[declaración de funciones internas]========================*/

static bool_t ina219SetConfigAndCalibRegister( void );
static bool_t ina219GetRegisters( int16_t *shunt_volt, int16_t *bus_volt, int16_t *power, int16_t *current);

/*====================[definición de datos internos]=========================*/

//INA219 control structure
static INA219_Control_t control;

static int16_t ina219_shunt_volt = 0;
static int16_t ina219_bus_volt = 0;
static int16_t ina219_power = 0;
static int16_t ina219_current = 0;
static bool_t readOK;

/*==================[internal functions definition]==========================*/

// Función que carga los valores en el "Configuration Register" y "Calibration Register" y
// verifica que hallan sido correctamente escritos
static bool_t ina219SetConfigAndCalibRegister( void )
{
	uint8_t transmitDataBuffer[3];									// Datos a transmitir
	uint8_t receiveDataBuffer[2];									// Datos recibidos
	uint16_t configRegister = 0x0;									// Configuration Register
	uint16_t calibRegister = 0x2000;								// Calibration Register (este valor debe ser calculado)
	uint8_t readDataBuffer;											// Register pointer address to read
	bool_t configRegisterOK;										// Bit para verificar escritura
	bool_t calibRegisterOK;											// Bit para verificar escritura

	control.BRNG_bit = INA219_CONFIG_BRNG_16V;						// Setting bus voltage range
	control.PG_bits = INA219_CONFIG_GAIN_40MV;						// Setting gain and range
	control.BADC_bits = INA219_CONFIG_BADC_12BIT_1S;				// Setting bus ADC resolution/averaging
	control.SADC_bits = INA219_CONFIG_SADC_12BIT_1S;				// Setting shunt ADC resolution/averaging
	control.MODE_bits = INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS;	// Setting operating mode

	// Build Configuration Register (16 bits)
	configRegister =  ( control.BRNG_bit & 0x01 ) << 13;			// Set bit BRNG (13)
	configRegister |= ( control.PG_bits & 0x03 ) << 11;				// Set bit PG1 and PG0 (12-11)
	configRegister |= ( control.BADC_bits & 0x0F ) << 7;			// Set bit BADC4/3/2/1 (10-9-8-7)
	configRegister |= ( control.SADC_bits & 0x0F ) << 3;			// Set bit SADC4/3/2/1 (6-5-4-3)
	configRegister |= ( control.MODE_bits & 0x07 );					// Set bit MODE3/2/1 (2-1-0)

	transmitDataBuffer[0] = INA219_CONFIGURATION;					// Configuration Register pointer address
	transmitDataBuffer[1] = (configRegister >> 8) & 0xFF;			// MSB 1st
	transmitDataBuffer[2] = configRegister & 0xFF;					// LSB 2nd
	i2cWrite(I2C0, control.address, transmitDataBuffer, 3, TRUE);	// Write Configuration Register

	delay(10);														// Wait

	// Luego de escribir el registro lo leo, y comparo los valores
	readDataBuffer = INA219_CONFIGURATION;							// Register pointer address to read
    i2cWriteRead( I2C0, control.address, &readDataBuffer, 1, TRUE, receiveDataBuffer, 2, TRUE );
		if( ( transmitDataBuffer[1] == receiveDataBuffer[0] ) &&
			( transmitDataBuffer[2] == receiveDataBuffer[1]) ) {
			configRegisterOK = TRUE;
		}
		else {
			configRegisterOK = FALSE;
		}

	// Write Calibration Register
	transmitDataBuffer[0] = INA219_CALIBRATION;						// Calibration Register pointer address
	transmitDataBuffer[1] = (calibRegister >> 8) & 0xFF;			// MSB 1st
	transmitDataBuffer[2] = calibRegister & 0xFF;					// LSB 2nd
	i2cWrite(I2C0, control.address, transmitDataBuffer, 3, TRUE);	// Write Calibration Register

	delay(10);														// Wait

	// Luego de escribir el registro lo leo, y comparo los valores
	readDataBuffer = INA219_CALIBRATION;							// Register pointer address to read
	i2cWriteRead( I2C0, control.address, &readDataBuffer, 1, TRUE, receiveDataBuffer, 2, TRUE );
		if( ( transmitDataBuffer[1] == receiveDataBuffer[0] ) &&
			( transmitDataBuffer[2] == receiveDataBuffer[1]) ) {
				calibRegisterOK = TRUE;
			}
			else {
				calibRegisterOK = FALSE;
			}

		if( ( configRegisterOK == TRUE ) && ( calibRegisterOK == TRUE) ) {
			return TRUE;
		}
		else {
			return FALSE;
		}
}

// Función que realiza la lectura de los registros "Shunt Voltage", "Bus Voltage", "Power Register" y
// "Current Register"
static bool_t ina219GetRegisters( int16_t *shunt_volt, int16_t *bus_volt, int16_t *power, int16_t *current )
{
	uint8_t receiveDataBuffer[2];									// Array donde almaceno lo leído
	uint8_t dataToReadBuffer;										// Register pointer address to read
	uint8_t ready;													// Conversion Ready

	dataToReadBuffer = INA219_BUS_VOLTAGE_OUT;			 			// Bus voltage measurement data
	i2cWriteRead( I2C0, control.address, &dataToReadBuffer, 1, TRUE, receiveDataBuffer, 2, TRUE);
	ready = receiveDataBuffer[1] & 0x2;								// Consulto el bit CNVR
	if ( ready == 0x2 ) {											// Si CNVR = 1 comienzo lectura

		dataToReadBuffer = INA219_SHUNT_VOLTAGE_OUT;		 		// Shunt voltage measurement data
		i2cWriteRead( I2C0, control.address, &dataToReadBuffer, 1, TRUE, receiveDataBuffer, 2, TRUE );
		*shunt_volt = receiveDataBuffer[0];                         // MSB 1st
		*shunt_volt = ( *shunt_volt << 8 ) | receiveDataBuffer[1];  // LSB 2nd

		dataToReadBuffer = INA219_BUS_VOLTAGE_OUT;			 		 // Bus voltage measurement data
		i2cWriteRead( I2C0, control.address, &dataToReadBuffer, 1, TRUE, receiveDataBuffer, 2, TRUE);
		*bus_volt = receiveDataBuffer[0];				 			 // MSB 1st
		*bus_volt = ( *bus_volt << 8 ) | receiveDataBuffer[1];	     // LSB 2nd
		*bus_volt = ( *bus_volt >> 3 );								 // No se utilizan los 3 bits menos significativos

		dataToReadBuffer = INA219_POWER_OUT;				 		 // Power measurement data
		i2cWriteRead( I2C0, control.address, &dataToReadBuffer, 1, TRUE, receiveDataBuffer, 2, TRUE);
		*power = receiveDataBuffer[0];					 			 // MSB 1st
		*power = ( *power << 8 ) | receiveDataBuffer[1];             // LSB 2nd

		dataToReadBuffer = INA219_CURRENT_OUT;				 		 // Value of the current flowing through the shunt resistor
		i2cWriteRead( I2C0, control.address, &dataToReadBuffer, 1, TRUE, receiveDataBuffer, 2, TRUE);
		*current = receiveDataBuffer[0];					 		 // MSB 1st
		*current = ( *current << 8 ) | receiveDataBuffer[1];		 // LSB 2nd

		return( TRUE );			// Si se realizaron todas las lecturas devuelve TRUE
	}
	else {
		return( FALSE );		// Si CNVR = 0, devuelvo FALSE
	}
}

/*=======================[external functions definition]=========================*/

// Función que inicializa al INA219, escribiendo el Registro de Configuración y el
// Registro de Calibración
bool_t ina219Init( INA219_address_t address )
{
	bool_t writeOK;

	i2cInit(I2C0, INA219_I2C_RATE);                      // Starting I2C bus
	control.address = address;                           // Set slave address

	writeOK = ina219SetConfigAndCalibRegister();		 // Set configuration and calibration registers for INA219

	return writeOK;										 // Successful init, return TRUE
}

// Función que realiza el casteo a float de los registros leídos
bool_t ina219Read()
{
	readOK = ina219GetRegisters( &ina219_shunt_volt, &ina219_bus_volt, &ina219_power, &ina219_current );
			 if (readOK == TRUE) {
				control.shunt_volt = (float)ina219_shunt_volt * 0.01;
				control.bus_volt = (float)ina219_bus_volt * 0.004;
				control.power = (float)ina219_power;
				control.current = (float)ina219_current * 0.05;
				return TRUE;
			 }
			 else {
				return FALSE;
			 }
}

float ina219ShowShuntVoltage( void )
{
	return control.shunt_volt;
}

float ina219ShowBusVoltage( void )
{
	return control.bus_volt;
}

float ina219ShowPower( void )
{
	return control.power;
}

float ina219ObtenerCorriente( void )
{
	uint32_t corriente = (uint32_t)control.current;
	printf( "Corriente: %d mA\r\n", corriente );

	return corriente;
}

/*==================[Calibration Register and Scaling]=======================*/
// EXAMPLES:

/* VBUS_MAX = 32V - VSHUNT_MAX = 0.32V - RSHUNT = 0.1 OHM

   1. Determine max possible current
   MaxPossible_I = VSHUNT_MAX / RSHUNT = 3.2A

   2. Determine max expected current
   MaxExpected_I = 2.0A

   3. Calculate possible range of LSBs (Min = 15-bit, Max = 12-bit)
   MinimumLSB = MaxExpected_I/32767
   MinimumLSB = 0.000061              (61uA per bit)
   MaximumLSB = MaxExpected_I/4096
   MaximumLSB = 0.000488              (488uA per bit)

   4. Choose an LSB between the min and max values
   (Preferrably a roundish number close to MinLSB)
   CurrentLSB = 0.0001 (100uA per bit)

   5. Compute the calibration register
   Cal = trunc [0.04096 / (Current_LSB * RSHUNT)] = [0.04096 / (0.0001 * 0.1)] =
   Cal = 4096 = 0x1000													      */

   //ina219_calValue = 0x1000;

/* VBUS_MAX = 32V - VSHUNT_MAX = 0.32V - RSHUNT = 0.1 OHM

   1. Determine max possible current
   MaxPossible_I = VSHUNT_MAX / RSHUNT = 3.2A

   2. Determine max expected current
   MaxExpected_I = 1.0A

   3. Calculate possible range of LSBs (Min = 15-bit, Max = 12-bit)
   MinimumLSB = MaxExpected_I/32767
   MinimumLSB = 0.0000305              (30.5uA per bit)
   MaximumLSB = MaxExpected_I/4096
   MaximumLSB = 0.000244               (244uA per bit)

   4. Choose an LSB between the min and max values
   (Preferrably a roundish number close to MinLSB)
   CurrentLSB = 0.000040 (40uA per bit)

   5. Compute the calibration register
   Cal = trunc [0.04096 / (Current_LSB * RSHUNT)] = [0.04096 / (0.000040 * 0.1)] =
   Cal = 10240 = 0x2800	                                                        */

   //ina219_calValue = 0x2800; <---

/* VBUS_MAX = 16V - VSHUNT_MAX = 0.04V - RSHUNT = 0.1 OHM

	1. Determine max possible current
	MaxPossible_I = VSHUNT_MAX / RSHUNT = 0.4A

	2. Determine max expected current
	MaxExpected_I = 0.4A

	3. Calculate possible range of LSBs (Min = 15-bit, Max = 12-bit)
	MinimumLSB = MaxExpected_I/32767
	MinimumLSB = 0.0000122              (12uA per bit)
	MaximumLSB = MaxExpected_I/4096
	MaximumLSB = 0.0000976              (98uA per bit)

	4. Choose an LSB between the min and max values
	(Preferrably a roundish number close to MinLSB)
	CurrentLSB = 0.00005 (50uA per bit)

	5. Compute the calibration register
	Cal = trunc [0.04096 / (Current_LSB * RSHUNT)] = [0.04096 / (0.00005 * 0.1)] =
	Cal = 8192 = 0x2000													      */

	//ina219_calValue = 0x2000; <---




