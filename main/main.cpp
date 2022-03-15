
#include "freertos/FreeRTOS.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "esp_wifi.h"
#include "esp_sntp.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "ESP_log.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include <string.h>
#include <cjson.h>
#include "lib/rtos/task.h"
#include "lib/rtos/stream.h"
#include "lib/rtos/timer.h"
#include "lib/tcpip/tcpsocket.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"


#define SSID			"vanBassum"
#define PSWD			"pedaalemmerzak"

#define SD_CS  			GPIO_NUM_25
#define SD_SCLK			GPIO_NUM_14 
#define SD_MOSI			GPIO_NUM_13 
#define SD_MISO			GPIO_NUM_4


#define SD_MOUNT_POINT	"/sdcard"


extern "C" {
	void app_main();
}

esp_err_t event_handler(void* ctx, system_event_t* event)
{
	return ESP_OK;
}

void StartWIFI()
{
	tcpip_adapter_init();
	ESP_ERROR_CHECK(esp_event_loop_init(event_handler, NULL));
	wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
	ESP_ERROR_CHECK(esp_wifi_init(&cfg));
	ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
	ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));

	wifi_config_t sta_config = { };
	memcpy(sta_config.sta.ssid, SSID, strlen(SSID));
	memcpy(sta_config.sta.password, PSWD, strlen(PSWD));
	sta_config.sta.bssid_set = false;
	ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sta_config));
	ESP_ERROR_CHECK(esp_wifi_start());
	ESP_ERROR_CHECK(esp_wifi_connect());

	setenv("TZ", "CET-1CEST,M3.5.0,M10.5.0/3", 1);
	tzset();
	sntp_setoperatingmode(SNTP_OPMODE_POLL);
	sntp_setservername(0, "pool.ntp.org");
	sntp_init();
}





void SDInit()
{
	esp_err_t ret;
	esp_vfs_fat_sdmmc_mount_config_t mount_config = {
		.format_if_mount_failed = false,
		.max_files = 5,
		.allocation_unit_size = 16 * 1024
	};
	
	sdmmc_card_t *card;
	const char mount_point[] = SD_MOUNT_POINT;
	ESP_LOGI("SDInit", "Initializing SD card");
	ESP_LOGI("SDInit", "Using SPI peripheral");
	
	
	gpio_set_pull_mode(SD_MOSI, GPIO_PULLUP_ONLY);
	gpio_set_pull_mode(SD_MISO, GPIO_PULLUP_ONLY); 
	gpio_set_pull_mode(SD_SCLK, GPIO_PULLUP_ONLY); 

	sdmmc_host_t host = SDSPI_HOST_DEFAULT();
	//host.slot = VSPI_HOST;
	//host.flags = SDMMC_HOST_FLAG_1BIT;
	//host.max_freq_khz = SDMMC_FREQ_PROBING;
	
	//host.flags = SDMMC_HOST_FLAG_SPI | SDMMC_HOST_FLAG_DEINIT_ARG | SDMMC_HOST_FLAG_1BIT;
	host.max_freq_khz = 1000;
	
	spi_bus_config_t bus_cfg = {
		.mosi_io_num = SD_MOSI,
		.miso_io_num = SD_MISO,
		.sclk_io_num = SD_SCLK,
		.quadwp_io_num = -1,
		.quadhd_io_num = -1,
		.max_transfer_sz = 4000,
		
	};
	ret = spi_bus_initialize((spi_host_device_t) host.slot, &bus_cfg, SPI_DMA_CH_AUTO);
	if (ret != ESP_OK) {
		ESP_LOGE("SDInit", "Failed to initialize bus.");
		return;
	}
	
	sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
	slot_config.gpio_cs = SD_CS;
	slot_config.host_id = (spi_host_device_t)host.slot;
	
	ESP_LOGI("SDInit", "Mounting filesystem");
	ret = esp_vfs_fat_sdspi_mount(mount_point, &host, &slot_config, &mount_config, &card);

	if (ret != ESP_OK) {
		if (ret == ESP_FAIL) {
			ESP_LOGE("SDInit",
				"Failed to mount filesystem. "
			         "If you want the card to be formatted, set the CONFIG_EXAMPLE_FORMAT_IF_MOUNT_FAILED menuconfig option.");
		}
		else {
			ESP_LOGE("SDInit",
				"Failed to initialize the card (%s). "
			         "Make sure SD card lines have pull-up resistors in place.",
				esp_err_to_name(ret));
		}
		return;
	}
	ESP_LOGI("SDInit", "Filesystem mounted");
	
	// Card has been initialized, print its properties
	sdmmc_card_print_info(stdout, card);

	
}















void app_main(void)
{
	nvs_flash_init();
	StartWIFI();
	SDInit();
}
