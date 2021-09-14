/*
 * ACCEL.c
 *
 *  Created on: Mar 10, 2021
 *      Author: linco
 */

#include "ACCEL.h"

#define MAX_MOV_AVG_SIZE 60
#define BUFF_MOV_AVG 20

static uint8_t Status_Summary_BUFF[4] = { 0, 0, 0, 0 };

static uint8_t BUFFER_MODE[4] = { 0xB4, 0x00, 0x00, 0x1F };

static double *accx_spi = NULL;
static double *accy_spi = NULL;
static double *accz_spi = NULL;
static double *temp_spi = NULL;

static double *accx_spi_mov = NULL;
static double *accy_spi_mov = NULL;
static double *accz_spi_mov = NULL;

static uint16_t index_mov_avrg = 10;

const int8_t Read_Status_Summary[4] = { 0x18, 0x00, 0x00, 0xE5 };
const int8_t Read_WHOAMI[4] = { 0x40, 0x00, 0x00, 0x91 };

const int8_t SW_reset[4] = { 0xB4, 0x00, 0x20, 0x98 };
const int8_t Chag_mode_1[4] = { 0xB4, 0x00, 0x00, 0x1F };
const int8_t Chag_mode_2[4] = { 0xB4, 0x00, 0x01, 0x02 };
const int8_t Chag_mode_3[4] = { 0xB4, 0x00, 0x02, 0x25 };
const int8_t Chag_mode_4[4] = { 0xB4, 0x00, 0x03, 0x38 };

const int8_t Read_ACCX[4] = { 0x04, 0x00, 0x00, 0xF7 };	//acceleration in x
const int8_t Read_ACCY[4] = { 0x08, 0x00, 0x00, 0xFD };	//acceleration in y
const int8_t Read_ACCZ[4] = { 0x0C, 0x00, 0x00, 0xFB };	//acceleration in z
const int8_t Read_STO[4] = { 0x10, 0x00, 0x00, 0xE9 };	//STO = self-test output
const int8_t Read_TEMP[4] = { 0x14, 0x00, 0x00, 0xEF };	// Temperature

static uint32_t count_error_spi_callback = 0;

struct MESURES *ptr_dados = NULL;

static SLOT slot = XAXIS;

static SPI_HandleTypeDef *SPI;

static OPERATION operation_accel = INITIALIZE;

static int aSensivity = SENSITIVITY_MODE_1;

static int16_t RESULTADO_STO = 0;

uint32_t flags;

extern osEventFlagsId_t Event_GroupHandle;
extern osThreadId_t vTaskBluetoothHandle;
extern osThreadId_t vTask_ACCELHandle;
extern osThreadId_t vTask_BAROMETERHandle;

/*BEGIN LIBRARY PRIVATE PROTOTYPE FUNCTIONS*/
uint8_t Calculate_CRC_frame(uint32_t Data);
void Switch_CS_Pin(void);
void delay_us(uint16_t us);
static uint8_t CRC8(uint8_t BitValue, uint8_t CRCC);
void Set_Moving_Average_Samples(uint16_t new);
uint8_t Get_Moving_Average_Samples(void);
double movingAvg(double *ptrArrNumbers, double *ptrSum, uint8_t pos,
		uint8_t len, double nextNum);
double Process_Accel(uint8_t *aAccel, int aSensivity);
int16_t Process_STO(void);
double Process_Temp(void);

/*END LIBRARY PRIVATE PROTOTYPE FUNCTIONS*/

/********BEGIN DMA TREATMENT*****************************************************************/

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi) {
	count_error_spi_callback++;
	if (count_error_spi_callback >= 10) {
		Error_Handler();
	}
	//Error_Handler();
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi) {
	flags = osThreadFlagsSet(vTask_ACCELHandle, TASK_ACCEL_FLAG);
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi) {

	Switch_CS_Pin();

	switch (Get_Operation()) {

	case READING:

		switch (get_slot()) {
		case XAXIS:
			HAL_SPI_Receive_DMA(SPI, ptr_dados->ACCX, 4);
			//HAL_SPI_Receive_DMA(SPI, DATA.ACCX, 4);
			//slot = YAXIS;
			set_slot(YAXIS);
			break;

		case YAXIS:
			HAL_SPI_Receive_DMA(SPI, ptr_dados->ACCY, 4);
			//HAL_SPI_Receive_DMA(SPI, DATA.ACCY, 4);
			//slot = ZAXIS;
			set_slot(ZAXIS);
			break;

		case ZAXIS:
			HAL_SPI_Receive_DMA(SPI, ptr_dados->ACCZ, 4);
			//HAL_SPI_Receive_DMA(SPI, DATA.ACCZ, 4);
			//slot = STO;
			set_slot(STO);
			break;

		case STO:
			HAL_SPI_Receive_DMA(SPI, ptr_dados->STOO, 4);
			//HAL_SPI_Receive_DMA(SPI, DATA.STOO, 4);
			//slot = TEMPE;
			set_slot(TEMPE);
			break;

		case TEMPE:
			HAL_SPI_Receive_DMA(SPI, ptr_dados->TEMP, 4);
			//HAL_SPI_Receive_DMA(SPI, DATA.TEMP, 4);
			//slot = XAXIS;
			set_slot(XAXIS);
			break;

		default:
			set_slot(XAXIS);
			//slot = XAXIS;
			break;
		}
		break;

	case CHANGING_MODE:
		HAL_SPI_Transmit_DMA(SPI, BUFFER_MODE, 4);//Before use this is necessary set the mode with "Set_Mode"
		Set_Operation(READING);
		break;

	case TESTING:
		//to be implemented
		HAL_SPI_Receive_DMA(SPI, Status_Summary_BUFF, 4);
		Set_Operation(READING);		//Return to read task

		//slot = REDIR;
		set_slot(REDIR);
		break;

	case RESETING:
		HAL_SPI_Transmit_DMA(SPI, SW_reset, 4);
		Set_Operation(READING);
		HAL_SPI_Transmit_DMA(SPI, Read_ACCX, 4);
		break;

	default:
		break;

	}
	osThreadFlagsSet(vTask_BAROMETERHandle, TASK_BAROMETER_FLAG);

}

/********END DMA TREATMENT*****************************************************************/

/*FUNCTION TO READ STATUS REGISTER*/
uint8_t Get_Return_Status(uint8_t *buff) {
	return (uint8_t) (0x03 & buff[0]);
}

uint8_t RSX(void) {
	return Get_Return_Status(ptr_dados->ACCX);
}
uint8_t RSY(void) {
	return Get_Return_Status(ptr_dados->ACCY);
}
uint8_t RSZ(void) {
	return Get_Return_Status(ptr_dados->ACCZ);
}

void Switch_CS_Pin(void) {

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);	//End of transmission
	//put flag here?

	delay_us(13);

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);//Begin of transmission
}


//Function to initialize ACCEL
void ACCEL_INIT(SPI_HandleTypeDef *hspi1,
		operationMode operation, struct MESURES *dados) {

	SPI = hspi1;
	ptr_dados = dados;

	HAL_SPI_Transmit_DMA(SPI, SW_reset, 4);
	HAL_Delay(50);

	Set_Mode(operation);
	HAL_Delay(50);

	HAL_SPI_Transmit_DMA(SPI, Read_Status_Summary, 4);
	HAL_Delay(50);

	HAL_SPI_Transmit_DMA(SPI, Read_Status_Summary, 4);
	HAL_Delay(50);

	HAL_SPI_Transmit_DMA(SPI, Read_Status_Summary, 4);
	HAL_Delay(50);

	//HAL_SPI_Transmit_DMA(SPI, Read_Status_Summary, 4);
	//HAL_Delay(50);

	Set_Operation(READING);

	slot = XAXIS;
	HAL_SPI_Transmit_DMA(SPI, Read_ACCX, 4);
	HAL_Delay(2);
}

void set_slot(SLOT new_slot) {
	slot = new_slot;
}

SLOT get_slot(void) {
	return slot;
}

void ACCEL_vars(double *accx, double *accy, double *accz, double *temp) {
	accx_spi = accx;
	accy_spi = accy;
	accz_spi = accz;
	temp_spi = temp;
}

void ACCEL_Vars_Statistic(double *accx, double *accy, double *accz) {
	accx_spi_mov = accx;
	accy_spi_mov = accy;
	accz_spi_mov = accz;
}

void Set_Operation(OPERATION OP) {
	operation_accel = OP;
}

OPERATION Get_Operation() {
	return operation_accel;
}

void Set_Mode(operationMode MODE) {
	/* Update sensivity to get accel */
	switch (MODE) {
	case OPMODE1:
		aSensivity = SENSITIVITY_MODE_1;
		memcpy(BUFFER_MODE, Chag_mode_1, 4 * sizeof(uint8_t));
		HAL_SPI_Transmit_DMA(SPI, Chag_mode_1, 4);
		HAL_Delay(50);
		break;

	case OPMODE2:
		aSensivity = SENSITIVITY_MODE_2;
		memcpy(BUFFER_MODE, Chag_mode_2, 4 * sizeof(uint8_t));
		HAL_SPI_Transmit_DMA(SPI, Chag_mode_2, 4);
		HAL_Delay(50);
		break;

	case OPMODE3:
		aSensivity = SENSITIVITY_MODE_3_4;
		memcpy(BUFFER_MODE, Chag_mode_3, 4 * sizeof(uint8_t));
		HAL_SPI_Transmit_DMA(SPI, Chag_mode_3, 4);
		HAL_Delay(50);
		break;

	default:
		aSensivity = SENSITIVITY_MODE_3_4;
		memcpy(BUFFER_MODE, Chag_mode_4, 4 * sizeof(uint8_t));
		HAL_SPI_Transmit_DMA(SPI, Chag_mode_4, 4);
		HAL_Delay(50);
		break;
	}
}

int Get_Mode(void) {
	return aSensivity;
}

//HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size, uint32_t Timeout)
// Steps to receive data from SPI:
// 1. Put CS Low - Activate comunication
// 2. Transmit register address
// 3. Read from MISO

void Set_Moving_Average_Samples(uint16_t new) {
	index_mov_avrg = new;
}

uint8_t Get_Moving_Average_Samples(void) {
	return index_mov_avrg;
}

//Function to calculate aceleration moving average
double movingAvg(double *ptrArrNumbers, double *ptrSum, uint8_t pos,
		uint8_t len, double nextNum) {
	//Subtract the oldest number from the prev sum, add the new number
	*ptrSum = *ptrSum - ptrArrNumbers[pos] + nextNum;
	//Assign the nextNum to the position in the array
	ptrArrNumbers[pos] = nextNum;
	//return the average
	return *ptrSum / len;
}

//Function to process acceleration value
double Process_Accel(uint8_t *aAccel, const int aSensivity) {
	int16_t accel = 0;
	accel |= (int16_t) aAccel[1] << 8;
	accel |= (int16_t) aAccel[2];
	double value = (double) accel / aSensivity;
	return value;
}

int16_t Process_STO(void) {
	RESULTADO_STO = 0;
	RESULTADO_STO |= (int16_t) ptr_dados->STOO[1] << 8;
	RESULTADO_STO |= (int16_t) ptr_dados->STOO[2];
	return RESULTADO_STO;
}

//Function to process temperature value
double Process_Temp(void) {
	int16_t temp = 0;
	temp |= (int16_t) ptr_dados->TEMP[1] << 8;
	temp |= (int16_t) ptr_dados->TEMP[2];
	double temperatura = TEMP_ABSOLUTE_ZERO + (temp / TEMP_SIGNAL_SENSITIVITY);
	return temperatura;
}

// Calculate CRC for 24 MSB's of the 32 bit dword
// (8 LSB's are the CRC field and are not included in CRC calculation)
uint8_t Calculate_CRC_frame(uint32_t Data) {
	uint8_t BitIndex;
	uint8_t BitValue;
	uint8_t CRCC;
	CRCC = 0xFF;
	for (BitIndex = 31; BitIndex > 7; BitIndex--) {
		BitValue = (uint8_t) ((Data >> BitIndex) & 0x01);
		CRCC = CRC8(BitValue, CRCC);
	}
	CRCC = (uint8_t) ~CRCC;
	return CRCC;
}

static uint8_t CRC8(uint8_t BitValue, uint8_t CRCC) {
	uint8_t Temp;
	Temp = (uint8_t) (CRCC & 0x80);
	if (BitValue == 0x01) {
		Temp ^= 0x80;
	}
	CRCC <<= 1;
	if (Temp > 0) {
		CRCC ^= 0x1D;
	}
	return CRCC;
}

