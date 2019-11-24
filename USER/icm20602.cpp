#include "icm20602.h"


extern OS_ERR err;
void HAL_SPI_MspInit(SPI_HandleTypeDef *hspi)
{
		GPIO_InitTypeDef GPIO_Initure;
		__HAL_RCC_SPI1_CLK_ENABLE();        //使能SPI1时钟
		__HAL_RCC_GPIOA_CLK_ENABLE();       //使能GPIOA时钟
		__HAL_RCC_GPIOE_CLK_ENABLE();       //使能GPIOA时钟
    GPIO_Initure.Pin=GPIO_PIN_15;            //PE15
    GPIO_Initure.Mode=GPIO_MODE_INPUT;      //输入
    GPIO_Initure.Pull=GPIO_PULLUP;        //下拉
    GPIO_Initure.Speed=GPIO_SPEED_FREQ_VERY_HIGH;     //高速
    HAL_GPIO_Init(GPIOE,&GPIO_Initure);
	
		GPIO_Initure.Pin=GPIO_PIN_4;
		GPIO_Initure.Mode=GPIO_MODE_OUTPUT_PP;          //复用
		GPIO_Initure.Pull=GPIO_PULLUP;              
		GPIO_Initure.Speed=GPIO_SPEED_FREQ_VERY_HIGH;  //高速

		HAL_GPIO_Init(GPIOA,&GPIO_Initure);
	
		GPIO_Initure.Pin=GPIO_PIN_5|GPIO_PIN_7;
		GPIO_Initure.Pull=GPIO_PULLUP; 
		GPIO_Initure.Mode=GPIO_MODE_AF_PP;	
		GPIO_Initure.Speed=GPIO_SPEED_FREQ_VERY_HIGH;   //高速
		GPIO_Initure.Alternate=GPIO_AF5_SPI1;   //复用为QSPI
		HAL_GPIO_Init(GPIOA,&GPIO_Initure);
		GPIO_Initure.Pin=GPIO_PIN_6;
		GPIO_Initure.Pull=GPIO_NOPULL; 
		GPIO_Initure.Mode=GPIO_MODE_AF_PP;
		HAL_GPIO_Init(GPIOA,&GPIO_Initure);
		HAL_NVIC_EnableIRQ(SPI1_IRQn);				//使能USART1中断通道
		HAL_NVIC_SetPriority(SPI1_IRQn,3,3);			//抢占优先级3，子优先级3
}

namespace MotorControl{
	OS_SEM ImuTransmitFinish;
	SPI_HandleTypeDef spi_Handler;
	u8 spiRxBuffer[SPIRXBUFFERSIZE];
	
	#define IMU_SPI_CS_Low  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET)
	#define IMU_SPI_CS_High HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET)
//union _imuData
//{
//    unsigned char buf[14];
//    struct
//    {
//        unsigned char axh, axl, ayh, ayl, azh, azl;
//        unsigned char th, tl;
//        unsigned char gxh, gxl, gyh, gyl, gzh, gzl;
//    };
//} imuData;

//unsigned char imuReadSeq[] = {
//    IMU_ACCEL_XOUT | 0x80   // 0x3B: start of data
//};
//unsigned char imuReadGYRO_ZSeq[] = {
//    (IMU_ACCEL_XOUT + 12) | 0x80,   // 0x3B: start of data
//    0x00,
//    0x00
//};
//unsigned char imuReadACCL_XSeq[] = {
//    (IMU_ACCEL_XOUT) | 0x80,   // 0x3B: start of data
//    0x00,
//    0x00
//};

void ICM20602::ImuSpiInit()
{
		spi_Handler.Instance = SPI1;
		spi_Handler.Init.Mode = SPI_MODE_MASTER;
		spi_Handler.Init.Direction = SPI_DIRECTION_2LINES;
		spi_Handler.Init.DataSize = SPI_DATASIZE_8BIT;
		spi_Handler.Init.CLKPolarity = SPI_POLARITY_LOW;
		spi_Handler.Init.CLKPhase = SPI_PHASE_1EDGE;
		spi_Handler.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
		spi_Handler.Init.FirstBit = SPI_FIRSTBIT_MSB;
		spi_Handler.Init.NSS = SPI_NSS_SOFT;
		spi_Handler.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
		spi_Handler.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;//active untill for EOT
		spi_Handler.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
		spi_Handler.Init.CRCPolynomial = 7;
		HAL_SPI_Init(&spi_Handler);
		IMU_SPI_CS_High;
}
ICM20602::ICM20602()
{
		ImuSpiInit();
		IMU_Init();
}

void ICM20602::IMU_WriteSingleRegister(uint8_t reg, uint8_t data)
{
    IMU_SPI_CS_Low;
		uint8_t dataSend[2] = {reg,data};
		HAL_SPI_TransmitReceive(&spi_Handler, dataSend, spiRxBuffer,2,100);
    IMU_SPI_CS_High;
}

uint8_t ICM20602::IMU_ReadSingleRegister(uint8_t reg)
{
    uint8_t dataSend[2] = {0x80u | reg, 0x00};
    IMU_SPI_CS_Low;
		HAL_SPI_TransmitReceive(&spi_Handler, dataSend, spiRxBuffer,2,100);
    IMU_SPI_CS_High;
    return spiRxBuffer[1];
}

void ICM20602::IMU_WriteRegister(uint8_t reg, uint8_t * data, int len)
{
    IMU_SPI_CS_Low;
		uint8_t reg_temp = reg;
		HAL_SPI_TransmitReceive(&spi_Handler, &reg_temp, spiRxBuffer,1,100);
		HAL_SPI_TransmitReceive(&spi_Handler, data, spiRxBuffer,len,100);
    IMU_SPI_CS_High;
}

void ICM20602::IMU_WriteRegister(uint8_t * data, int len)
{
	IMU_SPI_CS_Low;
	HAL_SPI_TransmitReceive(&spi_Handler, data, spiRxBuffer,len,100);
	IMU_SPI_CS_High;
}

void ICM20602::IMU_ReadRegister(uint8_t reg, uint8_t * data, int len)
{
    IMU_SPI_CS_Low;
    uint8_t dataSend[2] = {0x80u | reg, 0x00};
		HAL_SPI_Transmit(&spi_Handler, &dataSend[0], 1, 100);
		HAL_SPI_TransmitReceive(&spi_Handler, &dataSend[1], data,len,100);
    IMU_SPI_CS_High;
}

void ICM20602::IMU_Init()
{
    OS_ERR err;
		uint8_t id;
//		IMU_WriteSingleRegister(IMU_USER_CTRL, 0x00);//Enable FIFO and reset FiFO, signal path
		IMU_WriteSingleRegister(IMU_PWR_MGMT_1, 0x80);//Reset Imu
		OSTimeDlyHMSM(0, 0, 0, 100, OS_OPT_TIME_HMSM_STRICT, &err);
		IMU_WriteSingleRegister(IMU_PWR_MGMT_1, 0x00);//Auto set clk, disable temperature sensor;
    OSTimeDlyHMSM(0, 0, 0, 50, OS_OPT_TIME_HMSM_STRICT, &err);
		IMU_WriteSingleRegister(IMU_I2CIF_CFG, 0x40);//disable IIC
		id = IMU_ReadSingleRegister(IMU_WHO_AM_I);//read id
		printf("icm_20602 id=0x%x\r\n",id);
		if(id != 0x12)
		{
			printf("icm_20602 id error !!!\r\n");
		}
		IMU_WriteSingleRegister(IMU_PWR_MGMT_2, 0x00);
		uint8_t imuIntiSeq[] = {
			IMU_SMOLRT_DIV,
			0x00, // 0x19: sr = gyro rate / 1
			0x02, // 0x1A:FIFO mode disable, ext sync disable, dlpf GroBW = 8173
			0x18, // 0x1B: gyro fs =  +-2000dps
			0x08,  // 0x1C: accl fs = +-4g
			0x04   //0x1D: Avg 4, accel dlpf BW= 99
		};
		IMU_WriteRegister(imuIntiSeq, 6);
}
static uint8_t crashDet = 0;
uint8_t ICM20602::ImuReadData()
{
    uint8_t dataTemp[2];
		uint8_t i;

	
    IMU_ReadRegister(IMU_GYRO_ZOUT, dataTemp, 2);
    IMU_Data.GYRO_Z = ((uint16_t)dataTemp[0] << 8) | dataTemp[1];
	
		if((IMU_Data.GYRO_Z < -32000)||(IMU_Data.GYRO_Z > 32000))crashDet++;
		else crashDet /= 2;
	
//		for(i=0;i<8;i++);
	
		IMU_ReadRegister(IMU_ACCEL_YOUT, dataTemp, 2);
    IMU_Data.ACCEL_Y = ((uint16_t)dataTemp[0] << 8) | dataTemp[1];
	
		if(IMU_Data.ACCEL_Y < -32000)crashDet += 30;
		return crashDet;	
}

void ICM20602::ImuSelfTest()
{
		uint8_t id;
    ImuReadData();
		printf("Imu Gyro_Z: %5d\r\n", IMU_Data.GYRO_Z);
		printf("Imu Accl_Y: %5d\r\n", IMU_Data.ACCEL_Y);
		id = IMU_ReadSingleRegister(IMU_WHO_AM_I);//read id
		printf("icm_20602 id=0x%x\r\n",id);
		if(id != 0x12)
		{
			printf("icm_20602 id error !!!\r\n");
		}
}
void ICM20602::ImuReset()
{
	IMU_Init();
}
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
	if(hspi->Instance == SPI1)
		OSSemPost(&MotorControl::ImuTransmitFinish,OS_OPT_POST_ALL,&err);
}
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
	if(hspi->Instance == SPI1)
		OSSemPost(&MotorControl::ImuTransmitFinish,OS_OPT_POST_ALL,&err);
}
void SPI1_IRQHandler(void)
{
#if SYSTEM_SUPPORT_OS	 	//使用OS
	OSIntEnter();    
#endif
	HAL_SPI_IRQHandler(&MotorControl::spi_Handler);
		
#if SYSTEM_SUPPORT_OS	 	//使用OS
	OSIntExit();  											 
#endif
}

