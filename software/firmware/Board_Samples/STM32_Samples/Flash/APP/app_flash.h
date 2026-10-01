#ifndef __APP_FLASH_H__
#define __APP_FLASH_H__
#include "stdint.h"   

//FLASH起始地址
#define STM32_FLASH_BASE 0x08000000 	//STM32 FLASH的起始地址
#define STM32_FLASH_END  0x081FFFFF 	//STM32 FLASH的结束地址


//FLASH 扇区的起始地址,分2个bank,每个bank 1MB
#define BANK1_FLASH_SECTOR_0     ((uint32_t)0x08000000) 	//Bank1扇区0起始地址, 128 Kbytes  
#define BANK1_FLASH_SECTOR_1     ((uint32_t)0x08020000) 	//Bank1扇区1起始地址, 128 Kbytes  
#define BANK1_FLASH_SECTOR_2     ((uint32_t)0x08040000) 	//Bank1扇区2起始地址, 128 Kbytes  
#define BANK1_FLASH_SECTOR_3     ((uint32_t)0x08060000) 	//Bank1扇区3起始地址, 128 Kbytes  
#define BANK1_FLASH_SECTOR_4     ((uint32_t)0x08080000) 	//Bank1扇区4起始地址, 128 Kbytes  
#define BANK1_FLASH_SECTOR_5     ((uint32_t)0x080A0000) 	//Bank1扇区5起始地址, 128 Kbytes  
#define BANK1_FLASH_SECTOR_6     ((uint32_t)0x080C0000) 	//Bank1扇区6起始地址, 128 Kbytes  
#define BANK1_FLASH_SECTOR_7     ((uint32_t)0x080E0000) 	//Bank1扇区7起始地址, 128 Kbytes 
#define BANK2_FLASH_SECTOR_0     ((uint32_t)0x08100000) 	//Bank2扇区0起始地址, 128 Kbytes  
#define BANK2_FLASH_SECTOR_1     ((uint32_t)0x08120000) 	//Bank2扇区1起始地址, 128 Kbytes  
#define BANK2_FLASH_SECTOR_2     ((uint32_t)0x08140000) 	//Bank2扇区2起始地址, 128 Kbytes  
#define BANK2_FLASH_SECTOR_3     ((uint32_t)0x08160000) 	//Bank2扇区3起始地址, 128 Kbytes  
#define BANK2_FLASH_SECTOR_4     ((uint32_t)0x08180000) 	//Bank2扇区4起始地址, 128 Kbytes  
#define BANK2_FLASH_SECTOR_5     ((uint32_t)0x081A0000) 	//Bank2扇区5起始地址, 128 Kbytes  
#define BANK2_FLASH_SECTOR_6     ((uint32_t)0x081C0000) 	//Bank2扇区6起始地址, 128 Kbytes  
#define BANK2_FLASH_SECTOR_7     ((uint32_t)0x081E0000) 	//Bank2扇区7起始地址, 128 Kbytes    


uint32_t Flash_Read_Word(uint32_t addr);
void Flash_Read(uint32_t addr, uint32_t *output, uint32_t len);
int Flash_Write(uint32_t addr, uint32_t *data, uint32_t len);


#endif

