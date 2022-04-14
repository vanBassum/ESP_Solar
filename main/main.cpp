
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
#include "lib/rtos/task.h"
#include "ina226.h"
#include <driver/spi_master.h>
#include "st7735.h"
#include "fonts.h"
#include "esp_pm.h"


#define SSID			"vanBassum"
#define PSWD			"pedaalemmerzak"




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
	setenv("TZ", "UTC-1UTC,M3.5.0,M10.5.0/3", 1);
	tzset();
	sntp_setoperatingmode(SNTP_OPMODE_POLL);
	sntp_setservername(0, "pool.ntp.org");
	sntp_init();
}


spi_device_handle_t handle;

void spi_master_init()
{
	esp_err_t ret;

	gpio_pad_select_gpio(ST7735_CS_Pin);
	gpio_set_direction(ST7735_CS_Pin, GPIO_MODE_OUTPUT);
	gpio_set_level(ST7735_CS_Pin, 0);


	gpio_pad_select_gpio(ST7735_DC_Pin);
	gpio_set_direction(ST7735_DC_Pin, GPIO_MODE_OUTPUT);
	gpio_set_level(ST7735_DC_Pin, 0);

	
	if (ST7735_RES_Pin >= 0) {
		gpio_pad_select_gpio(ST7735_RES_Pin);
		gpio_set_direction(ST7735_RES_Pin, GPIO_MODE_OUTPUT);
		gpio_set_level(ST7735_RES_Pin, 0);
		vTaskDelay(pdMS_TO_TICKS(100));
		gpio_set_level(ST7735_RES_Pin, 1);
	}

	
	spi_bus_config_t buscfg = {
		.mosi_io_num = ST7735_SDA_Pin,
		.miso_io_num = -1,
		.sclk_io_num = ST7735_SCL_Pin,
		.quadwp_io_num = -1,
		.quadhd_io_num = -1
	};

	ret = spi_bus_initialize(HSPI_HOST, &buscfg, 1);
	assert(ret == ESP_OK);

	spi_device_interface_config_t devcfg =  {
		.clock_speed_hz = 20000000,
		.spics_io_num = ST7735_CS_Pin,
		.flags = SPI_DEVICE_NO_DUMMY,
		.queue_size = 7,
	};

	ret = spi_bus_add_device(HSPI_HOST, &devcfg, &handle);
	assert(ret == ESP_OK);
	//dev->_dc = GPIO_DC;
	//dev->_bl = GPIO_BL;
	//dev->_SPIHandle = handle;
}


void WrtPin(gpio_num_t pin, bool state)
{
	gpio_set_level(pin, state);
}

void spi_master_write_byte(uint8_t* Data, size_t DataLength, int timeout)
{
	spi_transaction_t SPITransaction;
	esp_err_t ret;

	if (DataLength > 0) {
		memset(&SPITransaction, 0, sizeof(spi_transaction_t));
		SPITransaction.length = DataLength * 8;
		SPITransaction.tx_buffer = Data;
		ret = spi_device_transmit(handle, &SPITransaction);
		assert(ret == ESP_OK); 
	}

	//return true;
}



void app_main(void)
{	
	nvs_flash_init();
	//StartWIFI();
	
	spi_master_init();
	SetWritePinCallback(WrtPin);
	SetTransmitCallback(spi_master_write_byte);
	
	

	ST7735_Init();
	ST7735_FillScreen(0x00);
		
	
	
	INA226 inaBATT;
	inaBATT.begin(0x40);
	inaBATT.configure(INA226_AVERAGES_16, INA226_BUS_CONV_TIME_8244US, INA226_SHUNT_CONV_TIME_8244US, INA226_MODE_SHUNT_BUS_TRIG);
	inaBATT.calibrate(0.005, 10);
	INA226 inaPV;
	inaPV.begin(0x41);
	inaPV.configure(INA226_AVERAGES_16, INA226_BUS_CONV_TIME_8244US, INA226_SHUNT_CONV_TIME_8244US, INA226_MODE_SHUNT_BUS_TRIG);
	inaPV.calibrate(0.005, 10);
	
	char buf[128];
	
	TickType_t prev = xTaskGetTickCount();
	while (true)
	{
		float uBat, iBat, pBat;
		uBat = inaBATT.readBusVoltage();
		iBat = inaBATT.readShuntVoltage() / 0.005;
		pBat = iBat * uBat;
		
		float uPV, iPV, pPV;
		uPV = inaPV.readBusVoltage();
		iPV = inaPV.readShuntVoltage() / 0.005;
		pPV = uPV * iPV;
		
		
		
		snprintf(buf, sizeof(buf), "U = %0.3fV", uBat);
		ST7735_WriteString(0, 0, buf, Font_11x18, 0xFFFF, 0x0000);
		
		snprintf(buf, sizeof(buf), "I = %0.3fA", iBat);
		ST7735_WriteString(0, 32, buf, Font_11x18, 0xFFFF, 0x0000);
		
		snprintf(buf, sizeof(buf), "P = %0.3fW", pBat);
		ST7735_WriteString(0, 64, buf, Font_11x18, 0xFFFF, 0x0000);

		TickType_t now = xTaskGetTickCount();
		int fps =  1000 / (portTICK_PERIOD_MS * (now - prev));
		prev = now;
		
		snprintf(buf, sizeof(buf), "%d fps", fps);
		ST7735_WriteString(0, 64+32, buf, Font_11x18, 0xFFFF, 0x0000);
	}	
}
