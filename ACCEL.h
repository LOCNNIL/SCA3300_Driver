/*
 * ACCEL.h
 *
 *  Created on: Mar 10, 2021
 *      Author: linco
 */

#ifndef INC_ACCEL_H_
#define INC_ACCEL_H_

#include "main.h"

#define SCA3300_MAX_SPI_FREQ_HZ  8000000
#define SCA3300_CHIP_ID           0x0051

#define TEMP_SIGNAL_SENSITIVITY    18.9
#define TEMP_ABSOLUTE_ZERO       -273.15

/* SCA3300 Sensivity definitions */
#define SENSITIVITY_MODE_1          2700 // +/-6g   1350 LSB/g
#define SENSITIVITY_MODE_2          1350 // +/-2g   2700 LSB/g
#define SENSITIVITY_MODE_3_4        5400 // +/-1.5g 5400 LSB/g

/* SCA3300 SPI frame field masks */
#define OPCODE_FIELD_MASK     0xFC000000
#define RS_FIELD_MASK         0x03000000
#define DATA_FIELD_MASK       0x00FFFF00
#define CRC_FIELD_MASK        0x000000FF

/* SCA3300 return status */
#define ST_START_UP                 0x00
#define ST_NORMAL_OP                0x01
#define ST_RESERVED					0x10
#define ST_ERROR                    0x11

/* Status Explanation */
#define SCA3300_ERR_DIGI1_BIT          9
#define SCA3300_ERR_DIGI2_BIT          8
#define SCA3300_ERR_CLOCK_BIT          7
#define SCA3300_ERR_STAT_BIT           6
#define SCA3300_ERR_TEMP_BIT           5
#define SCA3300_ERR_PWR_BIT            4
#define SCA3300_ERR_MEM_BIT            3
#define SCA3300_ERR_DIGI3_BIT          2
#define SCA3300_ERR_MODE_CHANGE_BIT    1
#define SCA3300_ERR_PIN_CONTINUITY_BIT 0

/* SCA3300 SPI requests */
#define REQ_READ_ACC_X        0x040000F7
#define REQ_READ_ACC_Y        0x080000FD
#define REQ_READ_ACC_Z        0x0C0000FB
#define REQ_READ_STO          0x100000E9
#define REQ_READ_TEMP         0x140000EF
#define REQ_READ_STATUS       0x180000E5
#define REQ_WRITE_SW_RESET    0xB4002098
#define REQ_WRITE_MODE1       0xB400001F
#define REQ_WRITE_MODE2       0xB4000102
#define REQ_WRITE_MODE3       0xB4000225
#define REQ_WRITE_MODE4       0xB4000338
#define REQ_READ_WHOAMI       0x40000091

typedef enum {
	X = 0, Y = 1, Z = 2
} AXIS;

typedef enum {
	ERR = 0, OPMODE1 = 2700, /*!<   3g full-scale. 88 Hz 1st order low pass filter (default) */
	OPMODE2 = 1350, /*!<   6g full-scale. 88 Hz 1st order low pass filter */
	OPMODE3 = 5400, /*!< 1.5g full-scale. 88 Hz 1st order low pass filter */
	OPMODE4 = 5400
} operationMode;

typedef enum {
	XAXIS = 0, YAXIS, ZAXIS, STO, TEMPE, REDIR
} SLOT;

typedef enum {
	INITIALIZE, READING, CHANGING_MODE, TESTING, RESETING,
} OPERATION;

struct MESURES {
	uint8_t ACCX[4];
	uint8_t ACCY[4];
	uint8_t ACCZ[4];
	uint8_t STOO[4];
	uint8_t TEMP[4];
};

uint8_t Get_Return_Status(uint8_t *buff);

uint8_t RSX(void);
uint8_t RSY(void);
uint8_t RSZ(void);

void ACCEL_INIT(SPI_HandleTypeDef *hspi1, operationMode operation,
		struct MESURES *dados);

void ACCEL_vars(double *accx, double *accy, double *accz, double *temp);

void ACCEL_Vars_Statistic(double *accx, double *accy, double *accz);

uint8_t Calculate_CRC_frame(uint32_t Data);

void Switch_CS_Pin(void);

void delay_us(uint16_t us);

static uint8_t CRC8(uint8_t BitValue, uint8_t CRCC);

void Set_Moving_Average_Samples(uint16_t new);

uint8_t Get_Moving_Average_Samples(void);

double movingAvg(double *ptrArrNumbers, double *ptrSum, uint8_t pos,
		uint8_t len, double nextNum);

double Process_Accel(uint8_t *aAccel, int aSensivity);

void set_slot(SLOT new_slot);

SLOT get_slot(void);

void Set_Mode(operationMode MODE);

int Get_Mode(void);

void Set_Operation(OPERATION OP);

OPERATION Get_Operation();

int16_t Process_STO(void);

double Process_Temp(void);

double Get_AccelX(void);

double Get_AccelY(void);

double Get_AccelZ(void);

#endif /* INC_ACCEL_H_ */
