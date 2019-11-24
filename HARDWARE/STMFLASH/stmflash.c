#include "stmflash.h"
#include "delay.h"
#include "includes.h"
//#include "Infrared.h"
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32H7������
//STM32�ڲ�FLASH��д ��������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//��������:2017/8/15
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	

/*************************************************************************/
/*************************���������ʵ����BANK1****************************/
/*************************************************************************/


//��ȡָ����ַ����(32λ����) 
//faddr:����ַ 
//����ֵ:��Ӧ����.
u32 STMFLASH_ReadWord(u32 faddr)
{
	return *(__IO uint32_t *)faddr; 
}

//��ȡĳ����ַ���ڵ�flash����,������BANK1����
//addr:flash��ַ
//����ֵ:0~11,��addr���ڵ�����
uint16_t STMFLASH_GetFlashSector(u32 addr)
{
	if(addr<ADDR_FLASH_SECTOR_1_BANK1)return FLASH_SECTOR_0;
	else if(addr<ADDR_FLASH_SECTOR_2_BANK1)return FLASH_SECTOR_1;
	else if(addr<ADDR_FLASH_SECTOR_3_BANK1)return FLASH_SECTOR_2;
	else if(addr<ADDR_FLASH_SECTOR_4_BANK1)return FLASH_SECTOR_3;
	else if(addr<ADDR_FLASH_SECTOR_5_BANK1)return FLASH_SECTOR_4;
	else if(addr<ADDR_FLASH_SECTOR_6_BANK1)return FLASH_SECTOR_5;
	else if(addr<ADDR_FLASH_SECTOR_7_BANK1)return FLASH_SECTOR_6;
	return FLASH_SECTOR_7;	
}

//��ָ����ַ��ʼд��ָ�����ȵ�����
//�ر�ע��:��ΪSTM32H7������ʵ��̫��,û�취���ر�����������,���Ա�����
//         д��ַ�����0XFF,��ô���Ȳ������������Ҳ�������������.����
//         д��0XFF�ĵ�ַ,�����������������ݶ�ʧ.����д֮ǰȷ��������
//         û����Ҫ����,��������������Ȳ�����,Ȼ����������д. 
//�ú�����OTP����Ҳ��Ч!��������дOTP��!
//OTP�����ַ��Χ:0X1FF0F000~0X1FF0F41F
//WriteAddr:��ʼ��ַ(�˵�ַ����Ϊ4�ı���!!)
//pBuffer:����ָ��
//NumToWrite:��(32λ)��(����Ҫд���32λ���ݵĸ���.) 
void STMFLASH_Write(u32 WriteAddr,u32 *pBuffer,u32 NumToWrite)	
{ 
    FLASH_EraseInitTypeDef FlashEraseInit;
    HAL_StatusTypeDef FlashStatus=HAL_OK;
    u32 SectorError=0;
		u32 addrx=0;
		u32 endaddr=0;	
    if(WriteAddr<STM32_FLASH_BASE||WriteAddr%4)return;	//�Ƿ���ַ
    
 	HAL_FLASH_Unlock();             //����	
	addrx=WriteAddr;				//д�����ʼ��ַ
	endaddr=WriteAddr+NumToWrite*4;	//д��Ľ�����ַ
    
    if(addrx<0X1FF00000)
    {
        while(addrx<endaddr)		//ɨ��һ���ϰ�.(�Է�FFFFFFFF�ĵط�,�Ȳ���)
		{
			if(STMFLASH_ReadWord(addrx)!=0XFFFFFFFF)//�з�0XFFFFFFFF�ĵط�,Ҫ�����������
			{   
				FlashEraseInit.Banks=FLASH_BANK_1;						//����BANK1
                FlashEraseInit.TypeErase=FLASH_TYPEERASE_SECTORS;       //�������ͣ��������� 
                FlashEraseInit.Sector=STMFLASH_GetFlashSector(addrx);   //Ҫ����������
                FlashEraseInit.NbSectors=1;                             //һ��ֻ����һ������
                FlashEraseInit.VoltageRange=FLASH_VOLTAGE_RANGE_3;      //��ѹ��Χ��VCC=2.7~3.6V֮��!!
                if(HAL_FLASHEx_Erase(&FlashEraseInit,&SectorError)!=HAL_OK) 
                {
                    break;//����������	
                }
                SCB_CleanInvalidateDCache();                            //�����Ч��D-Cache
			}else addrx+=4;
            FLASH_WaitForLastOperation(FLASH_WAITETIME,FLASH_BANK_1);    //�ȴ��ϴβ������
        }
    }
    FlashStatus=FLASH_WaitForLastOperation(FLASH_WAITETIME,FLASH_BANK_1);       //�ȴ��ϴβ������
	if(FlashStatus==HAL_OK)
	{
		while(WriteAddr<endaddr)//д����
		{
            if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD,WriteAddr,(uint64_t)pBuffer)!=HAL_OK)//д������
			{ 
				break;	//д���쳣
			}
			WriteAddr+=32;
			pBuffer+=8;
		} 
	}
	HAL_FLASH_Lock();           //����
} 

//��ָ����ַ��ʼ����ָ�����ȵ�����
//ReadAddr:��ʼ��ַ
//pBuffer:����ָ��
//NumToRead:��(32λ)��
void STMFLASH_Read(u32 ReadAddr,u32 *pBuffer,u32 NumToRead)   	
{
	u32 i;
	for(i=0;i<NumToRead;i++)
	{
		pBuffer[i]=STMFLASH_ReadWord(ReadAddr);//��ȡ4���ֽ�.
		ReadAddr+=4;//ƫ��4���ֽ�.	
	}
}

void UploadMazeInfo(uint32_t WriteAddr,
	uint32_t * grids, uint32_t gridsSize,
	uint32_t * searchedTimes, uint32_t searchedTimesSize,
	uint32_t * height, uint32_t heightSize)
{
    FLASH_EraseInitTypeDef FlashEraseInit;
    HAL_StatusTypeDef FlashStatus=HAL_OK;
    u32 SectorError=0;
		u32 addrx=0;
		u32 endaddr=0;	
    if(WriteAddr<STM32_FLASH_BASE||WriteAddr%4)return;	//�Ƿ���ַ
		
//		CPU_SR_ALLOC();
//		CPU_CRITICAL_ENTER();
//		IR_SL(0);
//		IR_FR(0);
//		IR_SR(0);
//		IR_FL(0);
		HAL_FLASH_Unlock();             //����
//		FLASH_DataCacheCmd(DISABLE);//FLASH�����ڼ�,�����ֹ���ݻ���
			
		addrx=WriteAddr;				//д�����ʼ��ַ
		endaddr=WriteAddr + gridsSize + searchedTimesSize + heightSize;	//д��Ľ�����ַ
	
    if(addrx<0X1FF00000)
    {
        while(addrx<endaddr)		//ɨ��һ���ϰ�.(�Է�FFFFFFFF�ĵط�,�Ȳ���)
		{
			if(STMFLASH_ReadWord(addrx)!=0XFFFFFFFF)//�з�0XFFFFFFFF�ĵط�,Ҫ�����������
			{   
				FlashEraseInit.Banks=FLASH_BANK_1;						//����BANK1
                FlashEraseInit.TypeErase=FLASH_TYPEERASE_SECTORS;       //�������ͣ��������� 
                FlashEraseInit.Sector=STMFLASH_GetFlashSector(addrx);   //Ҫ����������
                FlashEraseInit.NbSectors=1;                             //һ��ֻ����һ������
                FlashEraseInit.VoltageRange=FLASH_VOLTAGE_RANGE_3;      //��ѹ��Χ��VCC=2.7~3.6V֮��!!
                if(HAL_FLASHEx_Erase(&FlashEraseInit,&SectorError)!=HAL_OK) 
                {
                    break;//����������	
                }
                SCB_CleanInvalidateDCache();                            //�����Ч��D-Cache
			}else addrx+=4;
            FLASH_WaitForLastOperation(FLASH_WAITETIME,FLASH_BANK_1);    //�ȴ��ϴβ������
        }
    }
    FlashStatus=FLASH_WaitForLastOperation(FLASH_WAITETIME,FLASH_BANK_1);       //�ȴ��ϴβ������
		
		
		if(FlashStatus==HAL_OK)
		{
			endaddr = WriteAddr + gridsSize;
			while(WriteAddr<endaddr)//д����
			{
				
				if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD,WriteAddr,(uint64_t)grids)!=HAL_OK)//д������
				{ 
					break;	//д���쳣
				}
				WriteAddr+=32;
				grids+=2;
			} 
			endaddr += searchedTimesSize;
			while(WriteAddr<endaddr)//д����
			{
				if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD,WriteAddr,(uint64_t)grids)!=HAL_OK)//д������
				{ 
					break;	//д���쳣
				}
				WriteAddr+=32;
				grids+=2;
			}
			endaddr += heightSize;
			while(WriteAddr<endaddr)//д����
			{
				if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD,WriteAddr,(uint64_t)grids)!=HAL_OK)//д������
				{ 
					break;	//д���쳣
				}
				WriteAddr+=32;
				grids+=2;
			}
		}
		//FLASH_DataCacheCmd(ENABLE);	//FLASH��������,�������ݻ���
		HAL_FLASH_Lock();           //����
//		CPU_CRITICAL_EXIT();
}

void UploadMazeInfo1(uint32_t WriteAddr,
	uint32_t * grids, uint32_t gridsSize,
	uint32_t * searchedTimes, uint32_t searchedTimesSize,
	uint32_t * height, uint32_t heightSize){
		STMFLASH_Write(WriteAddr,grids,gridsSize/4);
		STMFLASH_Write(WriteAddr+0X00010000,searchedTimes,searchedTimesSize/4);
		STMFLASH_Write(WriteAddr+0X00020000,height,heightSize/4);
	}
