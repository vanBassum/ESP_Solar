#include "esp_system.h"
#include "esp_base.h"
#include "esp_lvgl.h"
#include "esp_com.h"
#include "esp_drivers.h"
#include "esp_log.h"
#include "rom/gpio.h"


using namespace ESP_Base;
using namespace ESP_LVGL;
using namespace ESP_Com;
using namespace ESP_Drivers;

#define SSID				"vanBassum"
#define PSWD				"pedaalemmerzak"

#define I2C_SCL 		  	GPIO_NUM_22
#define I2C_SDA			  	GPIO_NUM_21

#define SPI_CLK_Pin       	GPIO_NUM_27
#define SPI_MOSI_Pin      	GPIO_NUM_14
#define SPI_MISO_Pin      	GPIO_NUM_NC

#define DISPLAY_RES_Pin     GPIO_NUM_25
#define DISPLAY_DC_Pin      GPIO_NUM_13
#define DISPLAY_CS_Pin      GPIO_NUM_26



const char* TAG = "Main";

extern "C" {
	void app_main();
}


spi_device_handle_t handle;

void spi_master_init()
{
	esp_err_t ret;

	gpio_pad_select_gpio(DISPLAY_CS_Pin);
	gpio_set_direction(DISPLAY_CS_Pin, GPIO_MODE_OUTPUT);
	gpio_set_level(DISPLAY_CS_Pin, 0);


	gpio_pad_select_gpio(DISPLAY_DC_Pin);
	gpio_set_direction(DISPLAY_DC_Pin, GPIO_MODE_OUTPUT);
	gpio_set_level(DISPLAY_DC_Pin, 0);

	
	if (DISPLAY_RES_Pin >= 0) {
		//gpio_pad_select_gpio(DISPLAY_RES_Pin);
		gpio_set_direction(DISPLAY_RES_Pin, GPIO_MODE_OUTPUT);
		gpio_set_level(DISPLAY_RES_Pin, 0);
		vTaskDelay(pdMS_TO_TICKS(100));
		gpio_set_level(DISPLAY_RES_Pin, 1);
	}

	
	spi_bus_config_t buscfg = {
		.mosi_io_num = SPI_MOSI_Pin,
		.miso_io_num = SPI_MISO_Pin,
		.sclk_io_num = SPI_CLK_Pin,
		.quadwp_io_num = -1,
		.quadhd_io_num = -1
	};

	ret = spi_bus_initialize(HSPI_HOST, &buscfg, 1);
	assert(ret == ESP_OK);

	spi_device_interface_config_t devcfg =  {
		.clock_speed_hz = 20000000,
		.spics_io_num = DISPLAY_CS_Pin,
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
	SPIBus spiBus;
	ST7735 st7735;

	INIT_AND_CONTINUE(TAG, "Initializing NVS", ESP32::InitNVS());
	//INIT_AND_CONTINUE(TAG, "Initializing WIFI", ESP32::InitWIFI(SSID, PSWD));
	//INIT_AND_CONTINUE(TAG, "Initializing NTP", ESP32::InitNTP());
	INIT_AND_CONTINUE(TAG, "Initializing LVGL", LVGL::Init());


	//spi_master_init();
	//SetWritePinCallback(WrtPin);
	//SetTransmitCallback(spi_master_write_byte);
	//ST7735_Init();
	//ST7735_FillScreen(0x00);
	//ST7735_WriteString(0, 0, "Hello", Font_11x18, 0xFFFF, 0x0000);
	
	INIT_AND_CONTINUE(TAG, "Initializing SPI", spiBus.Init(HSPI_HOST, SPI_CLK_Pin, SPI_MOSI_Pin, SPI_MISO_Pin));
	
	st7735.settings.cs = DISPLAY_CS_Pin;
	st7735.settings.dc = DISPLAY_DC_Pin;
	st7735.settings.rst = DISPLAY_RES_Pin;
	INIT_AND_CONTINUE(TAG, "Initializing st7735", st7735.Init(spiBus));
	
	DisplayST7735 display;
	display.Init(&st7735);
	
	Screen screen;
	screen.Init();
	
	Label label;
	label.Init(screen);
	
	label.SetText("Test");
	
	
	
	//for(int i=0; i<128; i++)
	//{
	//	st7735.ST7735_DrawPixel(i, i, 0x0000);
	//	st7735.ST7735_DrawPixel(128-i, i, 0xFFFF);
	//}

	while(1)
	{
		vTaskDelay(pdMS_TO_TICKS(100));
	}

}



