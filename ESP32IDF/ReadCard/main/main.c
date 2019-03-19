
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"

#include "driver/spi_master.h"
#include "driver/gpio.h"


#include "MFRC522.h"




void app_main()
{
	printf("Hello world!\n");

	/* Print chip information */
	esp_chip_info_t chip_info;
	esp_chip_info(&chip_info);
	printf("This is ESP32 chip with %d CPU cores, WiFi%s%s, ",
			chip_info.cores,
			(chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
					(chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");

	printf("silicon revision %d, ", chip_info.revision);

	printf("%dMB %s flash\n", spi_flash_get_chip_size() / (1024 * 1024),
			(chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");


	PCD_SPI();
	PCD_Init();
	uint32_t u32CardNo;
	uint32_t u32Cnt = 0;
	while(1)
	{
		if(PICC_IsNewCardPresent())
		{
			printf("Card No = %u\r\n", u32Cnt++);
			printf("Antenna gain = [%x]\n", PCD_GetAntennaGain());

			PCD_SetAntennaGain(RxGain_48dB);

			printf("Antenna gain = [%x]\n", PCD_GetAntennaGain());


			printf("Card present \r\n");
			if(PICC_ReadCardSerial())
			{

				u32CardNo = 0x00 ;
				u32CardNo = (u32CardNo) | ((uint32_t)uid.uidByte[0] << 24);
				u32CardNo = (u32CardNo) | ((uint32_t)uid.uidByte[1] << 16);
				u32CardNo = (u32CardNo) | ((uint32_t)uid.uidByte[2] <<  8);
				u32CardNo = (u32CardNo) | ((uint32_t)uid.uidByte[3] <<  0);
				printf("CardNo = [%x]\r\n", u32CardNo );
//				for(int i = 0; i < 10; i++)
//					printf("%x ", uid.uidByte[i]);
				printf("\r\n");
				uid.size = 0;
				uid.sak = 0;
				for(int i = 0 ;i < 10; i++)
					uid.uidByte[i] = 0;
			}
		}
		vTaskDelay(pdMS_TO_TICKS(100));
	}



}