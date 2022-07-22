/*
 * DS2782.h
 *
 *  Created on: 11 февр. 2021 г.
 *      Author: belyaev
 */

#ifndef DS2782_H_
#define DS2782_H_
//*******************************************************************************************
//*******************************************************************************************

#include "main.h"

//*******************************************************************************************
//*******************************************************************************************
#define DS2782_ADDR (0x34 << 1)
#define DS2782_I2C	I2C2

//************************************************************
//Регистры DS2782.
//typedef enum {
//   Parameter_CONTROL         = 0x60, // Control Register
//   Parameter_AB              = 0x61, // Accumulation Bias
//   Parameter_AC              = 0x62, // Aging Capacity (2 bytes)
//   Parameter_VCHG            = 0x64, // Charge Voltage
//   Parameter_IMIN            = 0x65, // Minimum Charge Current
//   Parameter_VAE             = 0x66, // Active Empty Voltage
//   Parameter_IAE             = 0x67, // Active Empty Current
//   Parameter_AE_40           = 0x68, // Active Empty 40
//   Parameter_RSNSP           = 0x69, // Sense Resistor Prime
//   Parameter_Full_40         = 0x6a, // Full 40 (2 bytes)
//   Parameter_Full_Slope      = 0x6c, // Full Slope (4 bytes)
//   Parameter_AE_Slope        = 0x70, // Active Empty Slope (4 bytes)
//   Parameter_SE_Slope        = 0x74, // Standby Empty Slope (4 bytes)
//   Parameter_RSGAIN          = 0x78, // Sense Resistor Gain (2 bytes)
//   Parameter_RSTC            = 0x7a, // Sense Resistor Temperature Coefficient
//   Parameter_FRSGAIN         = 0x7b, // Factory Gain (2 bytes)
//   // Reserved, 0x7d
//   Parameter_Address         = 0x7e, // 2-Wire Slave Address
//   // Reserved, 0x7f
// }DS2782_ParameterRegisters_t;
//************************************************************
typedef enum{
	// Name                              Acc.   Size  Description
	// Reserved, 0x00
	Register_STATUS           = 0x01, // R/W    1     Status
	Register_RAAC             = 0x02, // R      2     Remaining Active Absolute Capacity
	Register_RSAC             = 0x04, // R      2     Remaining Standby Absolute Capacity
	Register_RARC             = 0x06, // R      1     Remaining Active Relative Capacity
	Register_RSRC             = 0x07, // R      1     Remaining Standby Relative Capacity
	Register_IAVG             = 0x08, // R      2     Average Current
	Register_TEMP             = 0x0a, // R      2     Temperature
	Register_VOLT             = 0x0c, // R      2     Voltage
	Register_CURRENT          = 0x0e, // R      2     Current
	Register_ACR              = 0x10, // R/W    2     Accumulated Current (auto-saved)
	Register_ACRL             = 0x12, // R      2     Low Accumulated Current
	Register_AS               = 0x14, // R/W    1     Age Scalar (auto-saved)
	Register_SFR              = 0x15, // R/W    1     Special Feature Register
	Register_FULL             = 0x16, // R      2     Full Capacity
	Register_AE               = 0x18, // R      2     Active Empty Capacity
	Register_SE               = 0x1a, // R      2     Standby Empty Capacity
	// Reserved, 0x1c - 0x1e
	Register_EEPROM           = 0x1f, // R/W    1     EEPROM, Block 0
	Register_User_EEPROM_1    = 0x20, // R/W    16    User EEPROM, Lockable, Block 0
	Register_User_EEPROM_2    = 0x30, // R/W    8     Additional User EEPROM, Lockable, Block 0
	// Reserved, 0x38 - 0x5f
	//---------------------------------------------------
	//Parameter EEPROM Memory Block 1
	// R/W    32    Parameter EEPROM, Lockable, Block 1
	Parameter_CONTROL         = 0x60, // Control Register
	Parameter_AB              = 0x61, // Accumulation Bias
	Parameter_AC              = 0x62, // Aging Capacity (2 bytes)
	Parameter_VCHG            = 0x64, // Charge Voltage
	Parameter_IMIN            = 0x65, // Minimum Charge Current
	Parameter_VAE             = 0x66, // Active Empty Voltage
	Parameter_IAE             = 0x67, // Active Empty Current
	Parameter_AE_40           = 0x68, // Active Empty 40
	Parameter_RSNSP           = 0x69, // Sense Resistor Prime
	Parameter_Full_40         = 0x6a, // Full 40 (2 bytes)
	Parameter_Full_Slope      = 0x6c, // Full Slope (4 bytes)
	Parameter_AE_Slope        = 0x70, // Active Empty Slope (4 bytes)
	Parameter_SE_Slope        = 0x74, // Standby Empty Slope (4 bytes)
	Parameter_RSGAIN          = 0x78, // Sense Resistor Gain (2 bytes)
	Parameter_RSTC            = 0x7a, // Sense Resistor Temperature Coefficient
	Parameter_FRSGAIN         = 0x7b, // Factory Gain (2 bytes)
	// Reserved, 0x7d
	Parameter_Address         = 0x7e, // 2-Wire Slave Address
	// Reserved, 0x7f
	//---------------------------------------------------
	// Reserved, 0x80 - 0xef
	Register_Unique_ID        = 0xf0, // R      8     Unique ID (factory option)
	// Reserved, 0xf8 - 0xfd
	Register_Function_Command = 0xfe, // W      1     Function Command
	// Reserved, 0xff
 }DS2782_Registers_t;
 //************************************************************
typedef struct{
	uint32_t I2C_Address;
	uint32_t ID;
	uint32_t Temperature;
	uint32_t Voltage;
	int16_t  Current;
}DS2782_t;
//*******************************************************************************************
//*******************************************************************************************
void     DS2782_Init(I2C_TypeDef *i2c);
uint16_t DS2782_ReadADC(DS2782_Registers_t addrReg, uint8_t len);

void     DS2782_GetI2cAddress(DS2782_t *ds);
void 	 DS2782_GetID(DS2782_t *ds);
void 	 DS2782_GetTemperature(DS2782_t *ds);
void 	 DS2782_GetCurrent(DS2782_t *ds);
void 	 DS2782_GetVoltage(DS2782_t *ds);
void Coulomb_Calc(uint16_t current);

//*******************************************************************************************
//*******************************************************************************************
#endif /* DS2782_H_ */
