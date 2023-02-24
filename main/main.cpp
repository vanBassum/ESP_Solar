
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
#include "ina3221.h"


#define SSID			"vanBassum"
#define PSWD			"pedaalemmerzak"

#define SCL 			GPIO_NUM_22
#define SDA				GPIO_NUM_21


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

static void i2c_master_init(int sda, int scl)
{
	static bool initialized = false;
	if (!initialized)
	{	
		int i2c_master_port = 0;
		i2c_config_t conf;
		conf.mode = I2C_MODE_MASTER;
		conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
		conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
		conf.master.clk_speed = 400000;
		conf.sda_io_num = sda;
		conf.scl_io_num = scl;
		conf.clk_flags = 0;
		i2c_param_config(i2c_master_port, &conf);
		if (i2c_driver_install(i2c_master_port, conf.mode, 0, 0, 0) == ESP_OK)
		{
			initialized = true;
		}
	}
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

	ST7735_WriteString(0, 0, "Hello", Font_11x18, 0xFFFF, 0x0000);

	i2c_master_init(SDA, SCL);



	INA3221 ina3221(INA3221_ADDR40_GND);
	ina3221.begin();
	ina3221.reset();
	ina3221.setShuntRes(5, 5, 5);

	char buf[128];
	
	TickType_t prev = xTaskGetTickCount();
	while (true)
	{
		float current[3], voltage[3];
		current[0] = ina3221.getCurrent(INA3221_CH1);
		voltage[0] = ina3221.getVoltage(INA3221_CH1);
		current[1] = ina3221.getCurrent(INA3221_CH2);
		voltage[1] = ina3221.getVoltage(INA3221_CH2);
		current[2] = ina3221.getCurrent(INA3221_CH3);
		voltage[2] = ina3221.getVoltage(INA3221_CH3);

		ESP_LOGI("MAIN", "%.2fV %.2fA   %.2fV %.2fA   %.2fV %.2fA", voltage[0], current[0], voltage[1], current[1], voltage[2], current[2]);
		vTaskDelay(pdMS_TO_TICKS(250));
	}	
}
