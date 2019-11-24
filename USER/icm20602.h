#ifndef _ICM20602_H_
#define _ICM20602_H_
#include "includes.h"
#include "usart.h"
#include "physparams.h"
namespace MotorControl{

typedef struct
{
    int16_t  ACCEL_X;
    int16_t  ACCEL_Y;
    int16_t  ACCEL_Z;
    int16_t  TEMP;
    int16_t  GYRO_X;
    int16_t  GYRO_Y;
    int16_t  GYRO_Z;
}IMU_DataDef;

typedef struct __ImuValues
{
    float acclX, acclY, acclZ;
    float temp;
    float angvX, angvY, angvZ;
} ImuValues;
//extern IMU_DataDef IMU_Data;

#define IMUDigitalInterface SPI
//#if IMUDigitalInterface == SPI


#define IMU_XG_OFFS_TC_H 0x04
#define IMU_XG_OFFS_TC_L 0x05
#define IMU_YG_OFFS_TC_H 0x07
#define IMU_YG_OFFS_TC_L 0x08
#define IMU_ZG_OFFS_TC_H 0x0A
#define IMU_ZG_OFFS_TC_L 0x0B

#define IMU_SMOLRT_DIV 0x19
#define IMU_CONFIG 0x1A
#define IMU_GYRO_CONFIG 0x1B
#define IMU_ACCEL_CONFIG 0x1C
#define IMU_ACCEL_CONFIG2 0x1D
#define IMU_FIFO_EN 0x23
#define IMU_INT_ENABLE 0x38
#define IMU_INT_STATUS 0x3A
#define IMU_ACCEL_XOUT 0x3B
#define IMU_ACCEL_YOUT 0x3D
#define IMU_GYRO_ZOUT 0x47
#define IMU_USER_CTRL 0x6A
#define IMU_PWR_MGMT_1 0x6B
#define IMU_PWR_MGMT_2 0x6C
#define IMU_I2CIF_CFG 0x70

#define	IMU_WHO_AM_I	0x75

#define SPIRXBUFFERSIZE 10



const float AcclFc = 99.f;
const float AcclPacc = 9.8 *1e-4;
const float Accla0 = 0.5 * 1e-3 * 9.8;

extern SPI_HandleTypeDef  spi_Handler;
extern u8 spiRxBuffer[SPIRXBUFFERSIZE];//HAL¿âÊ¹ÓÃµÄSPI Buffer size


class ICM20602{
private:
	void ImuSpiInit();
protected:

public:
    ICM20602();
//    ImuValues ImuData;
    IMU_DataDef IMU_Data;

		void IMU_WriteSingleRegister(uint8_t reg, uint8_t data);
		uint8_t IMU_ReadSingleRegister(uint8_t reg);
		void IMU_WriteRegister(uint8_t reg, uint8_t * data, int len);
		void IMU_WriteRegister(uint8_t * data, int len);
		void IMU_ReadRegister(uint8_t reg, uint8_t * data, int len);
    uint8_t ImuReadData();
		void ImuReset();
		void IMU_Init();
    void ImuSelfTest();

};

}
#endif
