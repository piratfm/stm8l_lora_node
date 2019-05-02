// Distributed with a free-will license.
// Use it any way you want, profit or free, provided it fits in the licenses of its associated works.
// BMP180
// This code is designed to work with the BMP180_I2CS I2C Mini Module available from ControlEverything.com.
// https://www.controleverything.com/content/Pressure?sku=BMP180_I2CS#tabs-0-product_tabset-2


#include "stm8l15x.h"
#include "stm8l15x_it.h"    /* SDCC patch: required by SDCC for interrupts */
#include "stdio.h"
#include "int64.h"
#include "main.h"

#include "BME280.h"

#define BMxx80_DELAY_TIME 5000
#define BMP180_DELAY { volatile uint16_t v;  for (v=0; v < BMxx80_DELAY_TIME; v++) nop(); }


// Carries fine temperature as global value for pressure and humidity calculation
static int32_t t_fine;
static uint8_t i2c_addr = BME280_ADDR;
bool sensor_is_bme280 = false;
bool sensor_is_bmp280 = false;
bool sensor_is_bmp180 = false;

static BME280_Compensation_TypeDef cal_param;
static BMP180_Calibration_TypeDef BMP180_Calibration;



#define BMXX80_ReadReg(reg) i2c_read_byte_data(i2c_addr, reg)
#define BMXX80_WriteReg(reg, data) i2c_write_byte_data(i2c_addr, reg, data)
#define BMXX80_ReadShortLSBFirst(reg) i2c_read_short_data(i2c_addr, reg, true)
#define BMXX80_ReadShort(reg) i2c_read_short_data(i2c_addr, reg, false)
#define BMXX80_Read3(reg) i2c_read_3bytes_data(i2c_addr, reg)

uint16_t BMP180_Read_UT(void);
uint32_t BMP180_Read_PT(uint8_t oss);
int16_t BMP180_Calc_RT(uint16_t UT);
int32_t BMP180_Calc_RP(uint32_t UP, uint8_t oss);

void BMxx80_read_data(int16_t *t, uint16_t *p, uint8_t *h)
{
  int i;
  *t=0;
  *p=0;
  *h=0;
  
  hal_delayMs(10);
  BMXX80_Reset();
  hal_delayMs(5);
  
  if(sensor_is_bme280) {
    // Set normal mode inactive duration (standby time)
  BME280_SetStandby(BME280_STBY_1s);

  // Set IIR filter constant
  BME280_SetFilter(BME280_FILTER_4);

  // Set oversampling for temperature
  BME280_SetOSRST(BME280_OSRS_T_x1);

  // Set oversampling for pressure
  BME280_SetOSRSP(BME280_OSRS_P_x1);

  // Set oversampling for humidity
  BME280_SetOSRSH(BME280_OSRS_H_x1);

  
  // Get status of measurements
  i  = BMXX80_ReadReg(BME280_REG_CTRL_MEAS);
  i |= BMXX80_ReadReg(BME280_REG_CTRL_HUM) << 8;
  printf("Measurements status: %04X\r\n\tTemperature: %s\r\n\tPressure   : %s\r\n\tHumidity   : %s\r\n",
                  i,
                  (i & BME280_OSRS_T_MSK) ? "ON" : "OFF",
                  (i & BME280_OSRS_P_MSK) ? "ON" : "OFF",
                  ((i >> 8) & BME280_OSRS_H_MSK) ? "ON" : "OFF"
          );

  
 ////////////////////////////////////////////////////////////////////////////////////
  int32_t UT,UP,UH;
  // Pressure in Q24.8 format
  uint32_t press_q24_8;
  // Humidity in Q22.10 format
  uint32_t hum_q22_10;
  // Human-readable temperature, pressure and humidity value
  int32_t temperature;
  uint32_t pressure;
  uint32_t humidity;
  
  // Main loop
  BME280_SetMode(BME280_MODE_FORCED);
    do {
      hal_delayMs(10);

      // Check current status of chip
      i = BME280_GetStatus();
      printf("Status: [%02X] %s %s\r\n",
                    i,
                    (i & BME280_STATUS_MEASURING) ? "MEASURING" : "READY",
                    (i & BME280_STATUS_IM_UPDATE) ? "NVM_UPDATE" : "NVM_READY"
            );
    } while((i & BME280_STATUS_MEASURING) || (i & BME280_STATUS_IM_UPDATE));
    

    // Get all raw readings from the chip
    i = BME280_Read_UTPH(&UT,&UP,&UH);
    //printf("Raw TPH: %08X %08X %08X\r\n",UT,UP,UH);
    printf("Raw UT: %ld\r\n",UT);
    printf("Raw UP: %ld\r\n",UP);
    printf("Raw UH: %ld\r\n",UH);

    // Calculate compensated values
    if (UT == 0x80000) {
            // Either temperature measurement is configured as 'skip' or first conversion is not completed yet
            printf("Temperature: no data\r\n");
    } else {
            // Temperature (must be calculated first)
		//	UT = 0x84d3c; // test raw value: 25.90C
            temperature = BME280_CalcT(UT);
            //printf("Temperature: %i.%02uC\r\n",
            //                temperature / 100,
            //                temperature % 100
            printf("Temperature x100: %ld C\r\n", temperature);
    }

    if (UH == 0x8000) {
            // Either humidity measurement is configured as 'skip' or first conversion is not completed yet
            printf("Humidity: no data\r\n");
    } else {
            // Humidity
//			UH = 0x7e47; // test raw value: 61.313%RH
            hum_q22_10 = BME280_CalcH(UH);
//			hum_q22_10 = 47445; // test Q22.10 value, output must be 46.333
            // Convert Q22.10 value to integer
            // e.g. Q22.10 value '47445' will be converted to '46333' what represents 46.333
            // Fractional part computed as (frac / 2^10)
            humidity = ((hum_q22_10 >> 10) * 1000) + (((hum_q22_10 & 0x3ff) * 976562) / 1000000);
            printf("Humidity: %lu.%lu%%RH\r\n",
                            humidity / 1000,
                            humidity % 1000
                    );
    }

    if (UT == 0x80000) {
            // Either pressure measurement is configured as 'skip' or first conversion is not completed yet
            printf("Pressure: no data\r\n");
    } else {
            // Pressure
//			UP = 0x554d8; // test raw value: 99488.136Pa = 994.881hPa = 746.224mmHg
            press_q24_8 = BME280_CalcP(UP);
//			press_q24_8 = 24674867; // test Q24.8 value, output must be 96386.199
            // Convert Q24.8 value to integer
            // e.g. Q24.8 value '24674867' will be converted to '96386199' what represents 96386.199
            // Fractional part computed as (frac / 2^8)
            pressure = ((press_q24_8 >> 8) * 1000) + (((press_q24_8 & 0xff) * 390625) / 100000);
            printf("Pressure: %lu.%luPa = %lu.%luhPa\r\n",
                            pressure / 1000,
                            pressure % 1000,
                            pressure / 100000,
                            (pressure % 100000) / 100
                    );
    }
    
    
    *t = temperature/10;
    *p = pressure/1000 / 10;
    *h = humidity / (1000/2);
    
  }
    
  if(sensor_is_bmp180) {
    *h = 0;
    
    uint32_t u_temp,u_pres;
    int32_t rt,rp;

    u_temp = BMP180_Read_UT();
    //u_temp = 27898;
    rt = BMP180_Calc_RT(u_temp);

    u_pres = BMP180_Read_PT(0);
    //u_pres = 23843;
    rp = BMP180_Calc_RP(u_pres,0);

    printf("UT =  %ld C\r\n", u_temp);
    printf("Temperature x100: %ld C\r\n", rt);

    printf("UP = %ld\r\n", u_pres);
    printf("pressure: %ld C\r\n", rp);
    
    *t = rt;
    *p = rp/10;
    
  }
    printf("------------------------\r\n");
  
  
  
  
}


int16_t BMP180_Calc_RT(uint16_t UT) {
	BMP180_Calibration.B5  = (((int32_t)UT - (int32_t)BMP180_Calibration.AC6) * (int32_t)BMP180_Calibration.AC5) >> 15;
	BMP180_Calibration.B5 += ((int32_t)BMP180_Calibration.MC << 11) / (BMP180_Calibration.B5 + BMP180_Calibration.MD);

	return (BMP180_Calibration.B5 + 8) >> 4;
}

int32_t BMP180_Calc_RP(uint32_t UP, uint8_t oss) {
	int32_t B3,B6,X3,p;
	uint32_t B4,B7;

	B6 = BMP180_Calibration.B5 - 4000;
	X3 = ((BMP180_Calibration.B2 * ((B6 * B6) >> 12)) >> 11) + ((BMP180_Calibration.AC2 * B6) >> 11);
	B3 = (((((int32_t)BMP180_Calibration.AC1) * 4 + X3) << oss) + 2) >> 2;
	X3 = (((BMP180_Calibration.AC3 * B6) >> 13) + ((BMP180_Calibration.B1 * ((B6 * B6) >> 12)) >> 16) + 2) >> 2;
	B4 = (BMP180_Calibration.AC4 * (uint32_t)(X3 + 32768)) >> 15;
	B7 = ((uint32_t)UP - B3) * (50000 >> oss);
	if (B7 < 0x80000000) p = (B7 << 1) / B4; else p = (B7 / B4) << 1;
	p += ((((p >> 8) * (p >> 8) * BMP180_PARAM_MG) >> 16) + ((BMP180_PARAM_MH * p) >> 16) + BMP180_PARAM_MI) >> 4;

	return p;
}


uint16_t BMP180_Read_UT(void) {
	uint16_t UT;

	BMXX80_WriteReg(BME280_REG_CTRL_MEAS,BMP180_T_MEASURE);
	hal_delayMs(10); // Wait for 4.5ms by datasheet

        UT = (uint16_t)BMXX80_ReadShort(BMP180_ADC_OUT_MSB_REG);
	return UT;
}

uint32_t BMP180_Read_PT(uint8_t oss) {
	uint32_t PT;
	uint8_t cmd,delay;

	switch(oss) {
	case 0:
		cmd = BMP180_P0_MEASURE;
		delay   = 6+3;
		break;
	case 1:
		cmd = BMP180_P1_MEASURE;
		delay   = 9+3;
		break;
	case 2:
		cmd = BMP180_P2_MEASURE;
		delay   = 15+3;
		break;
	case 3:
		cmd = BMP180_P3_MEASURE;
		delay   = 27+3;
		break;
	}

	BMXX80_WriteReg(BME280_REG_CTRL_MEAS,cmd);
	hal_delayMs(delay);
//	BMP180_WriteReg(0xf4,0x34);
//	Delay_ms(27);

        PT = BMXX80_Read3(BMP180_ADC_OUT_MSB_REG);
	return PT >> (8 - oss);
}



BME280_RESULT BME280_Check(void) {
	return (BMXX80_ReadReg(BME280_REG_ID) == 0x60) ? BME280_SUCCESS : BME280_ERROR;
}

BME280_RESULT BMP180_Check(void) {
	return (BMXX80_ReadReg(BME280_REG_ID) == 0x55) ? BME280_SUCCESS : BME280_ERROR;
}

// Order BME280 to do a software reset
// note: after reset the chip will be unaccessible during 3ms
inline void BMXX80_Reset(void) {
	BMXX80_WriteReg(BME280_REG_RESET,BME280_SOFT_RESET_KEY);
}

// Get version of the BME280 chip
// return:
//   BME280 chip version or zero if no BME280 present on the I2C bus or it was an I2C timeout
inline uint8_t BMXX80_GetVersion(void) {
	return BMXX80_ReadReg(BME280_REG_ID);
}

// Get current status of the BME280 chip
// return:
//   Status of the BME280 chip or zero if no BME280 present on the I2C bus or it was an I2C timeout
inline uint8_t BME280_GetStatus(void) {
	return BMXX80_ReadReg(BME280_REG_STATUS) & BME280_STATUS_MSK;
}

// Get current sensor mode of the BME280 chip
// return:
//   Sensor mode of the BME280 chip or zero if no BME280 present on the I2C bus or it was an I2C timeout
inline uint8_t BME280_GetMode(void) {
	return BMXX80_ReadReg(BME280_REG_CTRL_MEAS) & BME280_MODE_MSK;
}

// Set sensor mode of the BME280 chip
// input:
//   mode - new mode (BME280_MODE_SLEEP, BME280_MODE_FORCED or BME280_MODE_NORMAL)
void BME280_SetMode(uint8_t mode) {
	uint8_t reg;

	// Read the 'ctrl_meas' (0xF4) register and clear 'mode' bits
	reg = BMXX80_ReadReg(BME280_REG_CTRL_MEAS) & ~BME280_MODE_MSK;

	// Configure new mode
	reg |= mode & BME280_MODE_MSK;

	// Write value back to the register
	BMXX80_WriteReg(BME280_REG_CTRL_MEAS,reg);
}

// Set coefficient of the IIR filter
// input:
//   filter - new coefficient value (one of BME280_FILTER_x values)
void BME280_SetFilter(uint8_t filter) {
	uint8_t reg;

	// Read the 'config' (0xF5) register and clear 'filter' bits
	reg = BMXX80_ReadReg(BME280_REG_CONFIG) & ~BME280_FILTER_MSK;

	// Configure new filter value
	reg |= filter & BME280_FILTER_MSK;

	// Write value back to the register
	BMXX80_WriteReg(BME280_REG_CONFIG,reg);
}

// Set inactive duration in normal mode (Tstandby)
// input:
//   tsb - new inactive duration (one of BME280_STBY_x values)
void BME280_SetStandby(uint8_t tsb) {
	uint8_t reg;

	// Read the 'config' (0xF5) register and clear 'filter' bits
	reg = BMXX80_ReadReg(BME280_REG_CONFIG) & ~BME280_STBY_MSK;

	// Configure new standby value
	reg |= tsb & BME280_STBY_MSK;

	// Write value back to the register
	BMXX80_WriteReg(BME280_REG_CONFIG,reg);
}

// Set oversampling of temperature data
// input:
//   osrs - new oversampling value (one of BME280_OSRS_T_Xx values)
void BME280_SetOSRST(uint8_t osrs) {
	uint8_t reg;

	// Read the 'ctrl_meas' (0xF4) register and clear 'osrs_t' bits
	reg = BMXX80_ReadReg(BME280_REG_CTRL_MEAS) & ~BME280_OSRS_T_MSK;

	// Configure new oversampling value
	reg |= osrs & BME280_OSRS_T_MSK;

	// Write value back to the register
	BMXX80_WriteReg(BME280_REG_CTRL_MEAS,reg);
}

// Set oversampling of pressure data
// input:
//   osrs - new oversampling value (one of BME280_OSRS_P_Xx values)
void BME280_SetOSRSP(uint8_t osrs) {
	uint8_t reg;

	// Read the 'ctrl_meas' (0xF4) register and clear 'osrs_p' bits
	reg = BMXX80_ReadReg(BME280_REG_CTRL_MEAS) & ~BME280_OSRS_P_MSK;

	// Configure new oversampling value
	reg |= osrs & BME280_OSRS_P_MSK;

	// Write value back to the register
	BMXX80_WriteReg(BME280_REG_CTRL_MEAS,reg);
}

// Set oversampling of humidity data
// input:
//   osrs - new oversampling value (one of BME280_OSRS_H_Xx values)
void BME280_SetOSRSH(uint8_t osrs) {
	uint8_t reg;

	// Read the 'ctrl_hum' (0xF2) register and clear 'osrs_h' bits
	reg = BMXX80_ReadReg(BME280_REG_CTRL_HUM) & ~BME280_OSRS_H_MSK;

	// Configure new oversampling value
	reg |= osrs & BME280_OSRS_H_MSK;

	// Write value back to the register
	BMXX80_WriteReg(BME280_REG_CTRL_HUM,reg);

	// Changes to 'ctrl_hum' register only become effective after a write to 'ctrl_meas' register
	// Thus read a value of the 'ctrl_meas' register and write it back after write to the 'ctrl_hum'

	// Read the 'ctrl_meas' (0xF4) register
	reg = BMXX80_ReadReg(BME280_REG_CTRL_MEAS);

	// Write back value of 'ctrl_meas' register to activate changes in 'ctrl_hum' register
	BMXX80_WriteReg(BME280_REG_CTRL_MEAS,reg);
}

// Read calibration data
BME280_RESULT BME280_Read_Calibration(void) {
	uint8_t buf[3];

	// Read pressure and temperature calibration data (calib00..calib23)
        cal_param.dig_T1 = BMXX80_ReadShortLSBFirst(BME280_REG_CALIB00);
	cal_param.dig_T2 = BMXX80_ReadShortLSBFirst(BME280_REG_CALIB00+2);
	cal_param.dig_T3 = BMXX80_ReadShortLSBFirst(BME280_REG_CALIB00+4);
	cal_param.dig_P1 = BMXX80_ReadShortLSBFirst(BME280_REG_CALIB00+6);
	cal_param.dig_P2 = BMXX80_ReadShortLSBFirst(BME280_REG_CALIB00+8);
	cal_param.dig_P3 = BMXX80_ReadShortLSBFirst(BME280_REG_CALIB00+10);
	cal_param.dig_P4 = BMXX80_ReadShortLSBFirst(BME280_REG_CALIB00+12);
	cal_param.dig_P5 = BMXX80_ReadShortLSBFirst(BME280_REG_CALIB00+14);
	cal_param.dig_P6 = BMXX80_ReadShortLSBFirst(BME280_REG_CALIB00+16);
	cal_param.dig_P7 = BMXX80_ReadShortLSBFirst(BME280_REG_CALIB00+18);
	cal_param.dig_P8 = BMXX80_ReadShortLSBFirst(BME280_REG_CALIB00+20);
	cal_param.dig_P9 = BMXX80_ReadShortLSBFirst(BME280_REG_CALIB00+22);
	// Skip one byte (calib24) and read H1 (calib25)
	cal_param.dig_H1 = BMXX80_ReadReg(BME280_REG_CALIB25);
	cal_param.dig_H2 = BMXX80_ReadShortLSBFirst(BME280_REG_CALIB26);
	cal_param.dig_H3 = BMXX80_ReadReg(BME280_REG_CALIB26+2);
	buf[0] = BMXX80_ReadReg(BME280_REG_CALIB26+3);
        buf[1] = BMXX80_ReadReg(BME280_REG_CALIB26+4);
	buf[2] = BMXX80_ReadReg(BME280_REG_CALIB26+5);
        cal_param.dig_H4 = (int16_t)((((int8_t)buf[0]) << 4) | (buf[1] & 0x0f));
	cal_param.dig_H5 = (int16_t)((((int8_t)buf[2]) << 4) | (buf[1]  >>  4));
	cal_param.dig_H6 = BMXX80_ReadReg(BME280_REG_CALIB26+6);

	return BME280_SUCCESS;
}


// Read all raw values
// input:
//   UT = pointer to store temperature value
//   UP = pointer to store pressure value
//   UH = pointer to store humidity value
// return:
//   BME280_ERROR in case of I2C timeout, BME280_SUCCESS otherwise
// note: 0x80000 value for UT and UP and 0x8000 for UH means no data
BME280_RESULT BME280_Read_UTPH(int32_t *UT, int32_t *UP, int32_t *UH) {
#if 0
  uint8_t buf[8];
  // Clear result values
  //*UT = 0x80000;
  //*UP = 0x80000;
  //*UH = 0x8000;
  // Send 'press_msb' register address
  i2c_write_byte(BME280_ADDR<<1, true, false);
  i2c_write_byte(BME280_REG_PRESS_MSB, false, false);
  i2c_write_byte(BME280_ADDR<<1|0x01, true, false);
  uint8_t i;
  printf("UTPH: ");
  for(i=0;i<8;i++) {
    buf[i]=i2c_read_byte((i!=7), (i==7));
    printf("%02X ", buf[i]);
  }
  printf("\r\n");
  *UP = (uint32_t)(((uint32_t)buf[0] << 12) | ((uint32_t)buf[1] << 4) | ((uint32_t)buf[2] >> 4));
  *UT = (uint32_t)(((uint32_t)buf[3] << 12) | ((uint32_t)buf[4] << 4) | ((uint32_t)buf[5] >> 4));
  *UH = (uint32_t)(((uint32_t)buf[6] <<  8) |  (uint32_t)buf[7]);
#else
  // Read the 'press', 'temp' and 'hum' registers
  *UP = BMXX80_Read3(BME280_REG_PRESS_MSB) >> 4;
  *UT = BMXX80_Read3(BME280_REG_TEMP_MSB) >> 4;
  *UH = BMXX80_ReadShort(BME280_REG_HUM_MSB);
#endif
  return BME280_SUCCESS;
}

// Calculate temperature from raw value, resolution is 0.01 degree
// input:
//   UT - raw temperature value
// return: temperature in Celsius degrees (value of '5123' equals '51.23C')
// note: code from the BME280 datasheet (rev 1.1)



/*
        dig_T1:22897    dig_T2:-5783    dig_T3:12800


UT=547008

((547008>>3) - (22897<<1) ) · (-5783) >> 11   = shift((shift(547008; -3) - shift(22897; 1)) · (-5783); -11) = -63766
+24315
*/

int32_t BME280_CalcT(int32_t UT) {
#if 0
  int32_t var2;
    t_fine = (int32_t)((UT >> 3) - ((int32_t)cal_param.dig_T1 << 1));
    t_fine = mulsh32s(t_fine, cal_param.dig_T2, 11);
    var2 = (int32_t)((UT >> 4) - ((int32_t)cal_param.dig_T1));
    var2 = mulsh32s(var2, var2, 12);
    t_fine += mulsh32s(var2, cal_param.dig_T3, 14);
    return (t_fine * 5 + 128) >> 8;  
#endif
  t_fine  = ((((UT >> 3) - ((int32_t)cal_param.dig_T1 << 1))) * ((int32_t)cal_param.dig_T2)) >> 11;
  t_fine += (((((UT >> 4) - ((int32_t)cal_param.dig_T1)) * ((UT >> 4) - ((int32_t)cal_param.dig_T1))) >> 12) * ((int32_t)cal_param.dig_T3)) >> 14;

  return ((t_fine * 5) + 128) >> 8;
}

// Calculate pressure from raw value, resolution is 0.001 Pa
// input:
//   UP - raw pressure value
// return: pressure in Pa as unsigned 32-bit integer in Q24.8 format (24 integer and 8 fractional bits)
// note: output value of '24674867' represents 24674867/256 = 96386.2 Pa = 963.862 hPa
// note: BME280_CalcT must be called before calling this function
// note: using 64-bit calculations
// note: code from the BME280 datasheet (rev 1.1)

/*
        dig_P1:60811    dig_P2:16343    dig_P3:-12277
        dig_P4:12057    dig_P5:-14081   dig_P6:-1537
        dig_P7:3120     dig_P8:8401     dig_P9:-30701
*/
uint32_t BME280_CalcP(int32_t UP) {
	int32_t v1,v2;
	uint32_t p;

	v1 = (((int32_t)t_fine) >> 1) - (int32_t)64000;
	v2 = (((v1 >> 2) * (v1 >> 2)) >> 11 ) * ((int32_t)cal_param.dig_P6);
	v2 = v2 + ((v1 * ((int32_t)cal_param.dig_P5)) << 1);
	v2 = (v2 >> 2) + (((int32_t)cal_param.dig_P4) << 16);
	v1 = (((cal_param.dig_P3 * (((v1 >> 2) * (v1 >> 2)) >> 13 )) >> 3) + ((((int32_t)cal_param.dig_P2) * v1) >> 1)) >> 18;
	v1 = (((32768 + v1)) * ((int32_t)cal_param.dig_P1)) >> 15;
	if (v1 == 0) return 0; // avoid exception caused by division by zero
	p = (((uint32_t)(((int32_t)1048576) - UP) - (v2 >> 12))) * 3125;
	if (p < 0x80000000) {
		p = (p << 1) / ((uint32_t)v1);
	} else {
		p = (p / (uint32_t)v1) << 1;
	}
	v1 = (((int32_t)cal_param.dig_P9) * ((int32_t)(((p >> 3) * (p >> 3)) >> 13))) >> 12;
	v2 = (((int32_t)(p >> 2)) * ((int32_t)cal_param.dig_P8)) >> 13;
	p = (uint32_t)((int32_t)p + ((v1 + v2 + cal_param.dig_P7) >> 4));

	// Convert pressure to Q24.8 format (fractional part always be .000)
	p <<= 8;
	return (uint32_t)p;
}



// Calculate humidity from raw value, resolution is 0.001 %RH
// input:
//   UH - raw humidity value
// return: humidity in %RH as unsigned 32-bit integer in Q22.10 format (22 integer and 10 fractional bits)
// note: output value of '47445' represents 47445/1024 = 46.333 %RH
// note: BME280_CalcT must be called before calling this function
// note: code from the BME280 datasheet (rev 1.1)

uint32_t BME280_CalcH(int32_t UH) {
	int32_t vx1;

	vx1  = t_fine - (int32_t)76800;
	vx1  = ((((UH << 14) - ((int32_t)cal_param.dig_H4 << 20) - ((int32_t)cal_param.dig_H5 * vx1)) +	(int32_t)16384) >> 15) *
			(((((((vx1 * (int32_t)cal_param.dig_H6) >> 10) * (((vx1 * (int32_t)cal_param.dig_H3) >> 11) +
			(int32_t)32768)) >> 10) + (int32_t)2097152) * ((int32_t)cal_param.dig_H2) + 8192) >> 14);
	vx1 -= ((((vx1 >> 15) * (vx1 >> 15)) >> 7) * (int32_t)cal_param.dig_H1) >> 4;
	vx1  = (vx1 < 0) ? 0 : vx1;
	vx1  = (vx1 > 419430400) ? 419430400 : vx1;

	return (uint32_t)(vx1 >> 12);
}



void BMxx80_init()
{
  //i2c_scan();
  int i;
  
  i2c_addr = BME280_ADDR_G;
  if(!i2c_write_byte( i2c_addr << 1, true, true)) {
    if(BME280_Check() == BME280_SUCCESS) {
      sensor_is_bme280 = true;
    } else if(BMP180_Check() == BME280_SUCCESS) {
      sensor_is_bmp180 = true;
    }
  } else {
    i2c_addr = BME280_ADDR_V;
    if(!i2c_write_byte( i2c_addr << 1, true, true)) {
      if(BME280_Check() == BME280_SUCCESS) {
        sensor_is_bme280 = true;
      } else if(BMP180_Check() == BME280_SUCCESS) {
        sensor_is_bmp180 = true;
      }
    }
  }

  BMXX80_Reset();
  hal_delayMs(5);

  // Get version of the chip (BME280 = 0x60)
  i = BMXX80_GetVersion();
  printf("ChipID: %02X\r\n",i);

  
  
  if(sensor_is_bme280) {

    // Reset the BME280 chip
  #if 0
    // Chip start-up time is 2ms, must wait until it becomes accessible
    // Instead of using delay function, wait while chip loads the NVM data to image register
    // It loads the NVM data during start-up or before each measurement starts, so
    // wait until chip becomes accessible through the I2C interface and loads the NVM data
    int i = 0xfff;
    while (i-- && !(BME280_GetStatus() & BME280_STATUS_IM_UPDATE));
    if (i == 0) {
            // Some banana happens (no respond from the chip after reset or NVM bit stuck forever)
            printf("BME280 timeout\r\n");
            while(1);
    }
  #endif
    // Get current status of the chip
    i = BME280_GetStatus();
    printf("Status: [%02X] %s %s\r\n",
                    i,
                    (i & BME280_STATUS_MEASURING) ? "MEASURING" : "READY",
                    (i & BME280_STATUS_IM_UPDATE) ? "NVM_UPDATE" : "NVM_READY"
            );

    // Get current working mode (must be SLEEP after reset)
    i = BME280_GetMode();
    printf("Mode: [%02X] ",i);
    switch (i) {
            case BME280_MODE_SLEEP:
                    printf("SLEEP\r\n");
                    break;
            case BME280_MODE_FORCED:
                    printf("FORCED\r\n");
                    break;
            default:
                    printf("NORMAL\r\n");
                    break;
    }

    // Read calibration values
    i = BME280_Read_Calibration();
    printf("Read calibration: [%02X]\r\n",i);
    printf("\tdig_T1:%u\tdig_T2:%i\tdig_T3:%i\r\n",cal_param.dig_T1,cal_param.dig_T2,cal_param.dig_T3);
    printf("\tdig_P1:%u\tdig_P2:%i\tdig_P3:%i\r\n",cal_param.dig_P1,cal_param.dig_P2,cal_param.dig_P3);
    printf("\tdig_P4:%i\tdig_P5:%i\tdig_P6:%i\r\n",cal_param.dig_P4,cal_param.dig_P5,cal_param.dig_P6);
    printf("\tdig_P7:%i\tdig_P8:%i\tdig_P9:%i\r\n",cal_param.dig_P7,cal_param.dig_P8,cal_param.dig_P9);
    printf("\tdig_H1:%u\tdig_H2:%i\tdig_H3:%u\r\n",cal_param.dig_H1,cal_param.dig_H2,cal_param.dig_H3);
    printf("\tdig_H4:%i\tdig_H5:%i\tdig_H6:%i\r\n",cal_param.dig_H4,cal_param.dig_H5,cal_param.dig_H6);
  }
  
   if(sensor_is_bmp180) {
       // Calibration Cofficients stored in EEPROM of the device
        BMP180_Calibration.AC1 = BMXX80_ReadShort(0xAA);
        BMP180_Calibration.AC2 = BMXX80_ReadShort(0xAC);
        BMP180_Calibration.AC3 = BMXX80_ReadShort(0xAE);
        BMP180_Calibration.AC4 = BMXX80_ReadShort(0xB0);
        BMP180_Calibration.AC5 = BMXX80_ReadShort(0xB2);
        BMP180_Calibration.AC6 = BMXX80_ReadShort(0xB4);
        BMP180_Calibration.B1  = BMXX80_ReadShort(0xB6);
        BMP180_Calibration.B2  = BMXX80_ReadShort(0xB8);
        BMP180_Calibration.MB  = BMXX80_ReadShort(0xBA);
        BMP180_Calibration.MC  = BMXX80_ReadShort(0xBC);
        BMP180_Calibration.MD  = BMXX80_ReadShort(0xBE);
#if 1
        printf("AC1=%d\r\n", BMP180_Calibration.AC1);
        printf("AC2=%d\r\n", BMP180_Calibration.AC2);
        printf("AC3=%d\r\n", BMP180_Calibration.AC3);
        printf("AC4=%u\r\n", BMP180_Calibration.AC4);
        printf("AC5=%u\r\n", BMP180_Calibration.AC5);
        printf("AC6=%u\r\n", BMP180_Calibration.AC6);
        
        printf("B1=%d\r\n", BMP180_Calibration.B1);
        printf("B2=%d\r\n", BMP180_Calibration.B2);
        
        printf("MB=%d\r\n", BMP180_Calibration.MB);
        printf("MC=%d\r\n", BMP180_Calibration.MC);
        printf("MD=%d\r\n", BMP180_Calibration.MD);
#endif
   }
  
  
  
#if 0
  // Set normal mode inactive duration (standby time)
  BME280_SetStandby(BME280_STBY_1s);

  // Set IIR filter constant
  BME280_SetFilter(BME280_FILTER_4);

  // Set oversampling for temperature
  BME280_SetOSRST(BME280_OSRS_T_x1);

  // Set oversampling for pressure
  BME280_SetOSRSP(BME280_OSRS_P_x1);

  // Set oversampling for humidity
  BME280_SetOSRSH(BME280_OSRS_H_x1);

  
  // Get status of measurements
  i  = BME280_ReadReg(BME280_REG_CTRL_MEAS);
  i |= BME280_ReadReg(BME280_REG_CTRL_HUM) << 8;
  printf("Measurements status: %04X\r\n\tTemperature: %s\r\n\tPressure   : %s\r\n\tHumidity   : %s\r\n",
                  i,
                  (i & BME280_OSRS_T_MSK) ? "ON" : "OFF",
                  (i & BME280_OSRS_P_MSK) ? "ON" : "OFF",
                  ((i >> 8) & BME280_OSRS_H_MSK) ? "ON" : "OFF"
          );

  
 ////////////////////////////////////////////////////////////////////////////////////
  int32_t UT,UP,UH;
  // Pressure in Q24.8 format
  uint32_t press_q24_8;
  // Humidity in Q22.10 format
  uint32_t hum_q22_10;
  // Human-readable temperature, pressure and humidity value
  int32_t temperature;
  uint32_t pressure;
  uint32_t humidity;
  
  
  // Main loop
  while(1) {
    BME280_SetMode(BME280_MODE_FORCED);
    do {
      hal_delayMs(10);

      // Check current status of chip
      i = BME280_GetStatus();
      printf("Status: [%02X] %s %s\r\n",
                    i,
                    (i & BME280_STATUS_MEASURING) ? "MEASURING" : "READY",
                    (i & BME280_STATUS_IM_UPDATE) ? "NVM_UPDATE" : "NVM_READY"
            );
    } while((i & BME280_STATUS_MEASURING) || (i & BME280_STATUS_IM_UPDATE));
    

    // Get all raw readings from the chip
    i = BME280_Read_UTPH(&UT,&UP,&UH);
    //printf("Raw TPH: %08X %08X %08X\r\n",UT,UP,UH);
    printf("Raw UT: %ld\r\n",UT);
    printf("Raw UP: %ld\r\n",UP);
    printf("Raw UH: %ld\r\n",UH);

    // Calculate compensated values
    if (UT == 0x80000) {
            // Either temperature measurement is configured as 'skip' or first conversion is not completed yet
            printf("Temperature: no data\r\n");
    } else {
            // Temperature (must be calculated first)
			UT = 0x84d3c; // test raw value: 25.90C
            temperature = BME280_CalcT(UT);
            //printf("Temperature: %i.%02uC\r\n",
            //                temperature / 100,
            //                temperature % 100
            printf("Temperature x100: %ld C\r\n", temperature);
    }

    if (UH == 0x8000) {
            // Either humidity measurement is configured as 'skip' or first conversion is not completed yet
            printf("Humidity: no data\r\n");
    } else {
            // Humidity
//			UH = 0x7e47; // test raw value: 61.313%RH
            hum_q22_10 = BME280_CalcH(UH);
//			hum_q22_10 = 47445; // test Q22.10 value, output must be 46.333
            // Convert Q22.10 value to integer
            // e.g. Q22.10 value '47445' will be converted to '46333' what represents 46.333
            // Fractional part computed as (frac / 2^10)
            humidity = ((hum_q22_10 >> 10) * 1000) + (((hum_q22_10 & 0x3ff) * 976562) / 1000000);
            printf("Humidity: %lu.%lu%%RH\r\n",
                            humidity / 1000,
                            humidity % 1000
                    );
    }

    if (UT == 0x80000) {
            // Either pressure measurement is configured as 'skip' or first conversion is not completed yet
            printf("Pressure: no data\r\n");
    } else {
            // Pressure
//			UP = 0x554d8; // test raw value: 99488.136Pa = 994.881hPa = 746.224mmHg
            press_q24_8 = BME280_CalcP(UP);
//			press_q24_8 = 24674867; // test Q24.8 value, output must be 96386.199
            // Convert Q24.8 value to integer
            // e.g. Q24.8 value '24674867' will be converted to '96386199' what represents 96386.199
            // Fractional part computed as (frac / 2^8)
            pressure = ((press_q24_8 >> 8) * 1000) + (((press_q24_8 & 0xff) * 390625) / 100000);
            printf("Pressure: %lu.%luPa = %lu.%luhPa\r\n",
                            pressure / 1000,
                            pressure % 1000,
                            pressure / 100000,
                            (pressure % 100000) / 100
                    );
    }
    // Start measurement (if FORCED mode enabled)
//		BME280_SetMode(BME280_MODE_FORCED);
    printf("------------------------\r\n");
    hal_delayMs(10000);
  }
#endif
}

#if 0
void BMP180_read_data(int16_t *t, uint16_t *p)
{
  // Select measurement control register(0xF4)
  // Enable temperature measurement(0x2E)
  i2c_write_byte_data(0x77, 0xF4, 0x2E);
  BMP180_DELAY;

  // Read 2 bytes of data from register(0xF6)
  // temp msb, temp lsb
  // Convert the data
  int temp = i2c_read_short_data(BMP180_ADDR, 0xF6, false);
  printf("Temperature Raw : %d | %04x \r\n", temp, temp);

  // Select measurement control register(0xf4)
  // Enable pressure measurement, OSS = 1(0x74)
  i2c_write_byte_data(0x77, 0xF4, 0x74);

  // Callibration for Temperature
  //long long X1 = (temp - AC6) * AC5 / 32768.0; //6811,9622
  //int32_t X1 = muldiv32s((int32_t)temp - AC6, AC5, 32768);
  int32_t X1 = mulsh32s((int32_t)temp - AC6, AC5, 15);
  //printf("X1: %ld \r\n", X1);
  //long long X2 = (MC * 2048) / (X1 + MD); //-2516,9784
  int32_t X2 = muldiv32s(MC, 2048, X1 + MD);
  //printf("X2: %ld \r\n", X2);
  int32_t B5 = X1 + X2; //4294,9838
  //printf("B5: %ld \r\n", B5);
  //long cTemp = B5 * 5/8 + 5;
  
  long cTemp = (B5/8 + 1)*5;
  printf("Temperature in Celsius : %ld x100 C\r\n", cTemp);
  *t = cTemp/10;

  //this is not required  
  BMP180_DELAY;

  // Read 3 bytes of data from register(0xF6)
  // pres msb1, pres msb, pres lsb
  // Convert the data
  int32_t pres = (i2c_read_3bytes_data(BMP180_ADDR, 0xF6))/128;
  printf("Pressure Raw : %ld | %08x \r\n", pres, (uint32_t)pres);
  
  // Calibration for Pressure
  int32_t B6 = B5 - 4000; // 4282 - 4000 = 274
  //X1 = (B2 * (B6 * B6 / 4096)) / 2048;
  //X1 = muldiv32s(B6,B6,4096);
  X1 =  mulsh32s(B6,B6, 12);
  //printf("X1p: %ld \r\n", X1); // 18
  //X1 = (B2 *X1)/2048;
  //X1 = muldiv32s(X1,B2,2048);
  X1 =  mulsh32s(X1,B2, 11);
  //printf("X1: %ld \r\n", X1);  // 0
  //X2 = AC2 * B6 / 2048;
  //X2 = muldiv32s(AC2,B6,2048); //-1279 * 274 / 2048 = -171
  X2 = mulsh32s(AC2,B6,11); //-1279 * 274 / 2048 = -171
  //printf("X2: %ld \r\n", X2);
  int32_t X3 = X1 + X2; // 0 -171 = -171
  //printf("X3: %ld \r\n", X3);  // - 171
  int32_t B3 = (((((int32_t)AC1) * 4 + X3) * 2) + 2) / 4; //
  //printf("B3: %ld \r\n", B3); //18107
  //X1 = AC3 * B6 / 8192.0;
  X1 = mulsh32s(AC3,B6,13);
  //printf("X1: %ld \r\n", X1); // -485
  //X2 = (B1 * (B6 * B6 / 2048.0)) / 65536.0;
  X2 = mulsh32s(B6,B6,11);
  //printf("X2p: %ld \r\n", X2);
  X2 = mulsh32s(B1,X2,16);
  //printf("X2: %ld \r\n", X2); //3
  X3 = ((X1 + X2) + 2) / 4;
  //printf("X3: %ld \r\n", X3); //-120
  //long long B4 = AC4 * (X3 + 32768) / 32768.0;
  int32_t B4 = mulsh32s(AC4, (X3 + 32768), 15);
  //printf("B4: %ld \r\n", B4); //-30560
  //long long B7 = ((pres - B3) * (25000.0));
  //b7= 1719325000 30bit
  int32_t B71 = ((pres - B3));
  //printf("B7p: %ld \r\n", B71); //68773

  int32_t pressure = 0;
  //if(B7 < 2147483648UL)
  //  pressure = (B7 * 2) / B4;
  //else
  //  pressure = (B7 / B4) * 2;

//  if(B71 < 85900 /*2147483648/25000*/)
//    //pressure = (B71*25000 * 2) / B4;
//    pressure = muldiv32s(B71, 50000, B4);
//  else
//    //pressure = (B71*25000 / B4) * 2;
//    pressure = muldiv32s(B71, 50000, B4);
  //pressure = muldiv32s(B71, 50000, B4);
  pressure = muldiv32s(B71, 25000, B4)*2;
  //printf("pressure: %ld \r\n", pressure); //-112528
  

  X1 = (pressure / 256) * (pressure / 256);
  //X1 = muldiv32s(pressure, pressure, 65536);
  //printf("X1: %ld \r\n", X1); //193228

  //X1 = (X1 * 3038.0) / 65536.0;
  X1 = mulsh32s(X1, 3038, 16);
  //printf("X1: %ld \r\n", X1); //8957
  //X2 = ((-7357) * pressure) / 65536.0;
  X2 = mulsh32s(pressure, -7357, 16);
  //printf("X2: %ld \r\n", X2); // 12632
  int32_t pressure1 = pressure + (X1 + X2 + 3791) / 16;
  //printf("pressure1: %ld \r\n", pressure1);

  // Calculate Altitude
  //long altitude = 44330 * (1 - pow(pressure1/1013.25, 0.1903));

  // Output data to screen
  printf("Pressure : %ld Pa \r\n", pressure1);
  *p = pressure1/10;
}

#endif
