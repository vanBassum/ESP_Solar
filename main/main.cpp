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


void app_main(void)
{	
	I2CBus i2cBus;
	SPIBus spiBus;
	ST7735 st7735;
	INA3221 ina3221;

	st7735.settings.cs = DISPLAY_CS_Pin;
	st7735.settings.dc = DISPLAY_DC_Pin;
	st7735.settings.rst = DISPLAY_RES_Pin;

	INIT_AND_CONTINUE(TAG, "Initializing NVS", ESP32::InitNVS());
	//INIT_AND_CONTINUE(TAG, "Initializing WIFI", ESP32::InitWIFI(SSID, PSWD));
	//INIT_AND_CONTINUE(TAG, "Initializing NTP", ESP32::InitNTP());


	INIT_AND_CONTINUE(TAG, "Initializing I2C", i2cBus.Init(I2C_NUM_0, I2C_SCL, I2C_SDA));
	INIT_AND_CONTINUE(TAG, "Initializing ina3221", ina3221.Init(&i2cBus, INA3221_ADDR40_GND));
	INIT_AND_CONTINUE(TAG, "Initializing LVGL", LVGL::Init());
	INIT_AND_CONTINUE(TAG, "Initializing SPI", spiBus.Init(HSPI_HOST, SPI_CLK_Pin, SPI_MOSI_Pin, SPI_MISO_Pin, GPIO_NUM_NC, GPIO_NUM_NC, SPI_DMA_CH_AUTO));
	INIT_AND_CONTINUE(TAG, "Initializing st7735", st7735.Init(spiBus));
	ina3221.setShuntRes(5, 5, 5);


	DisplayST7735 display;
	display.Init(&st7735);


	Screen screen;
	screen.Init();

	Label labels[6];

	for(int i=0; i<6; i++)
	{
		labels[i].Init(screen);
		labels[i].SetPosition((i%2) * 64, (i/2) * 12);
	}

//

	while (true)
	{
		float current[3], voltage[3];
		current[0] = 0;//ina3221.getCurrent(INA3221_CH1);
		voltage[0] = 0;//ina3221.getVoltage(INA3221_CH1);
		current[1] = 0;//ina3221.getCurrent(INA3221_CH2);
		voltage[1] = 0;//ina3221.getVoltage(INA3221_CH2);
		current[2] = 0;//ina3221.getCurrent(INA3221_CH3);
		voltage[2] = 0;//ina3221.getVoltage(INA3221_CH3);

		//ESP_LOGI("MAIN", "%.2fV %.2fA   %.2fV %.2fA   %.2fV %.2fA", voltage[0], current[0], voltage[1], current[1], voltage[2], current[2]);
		vTaskDelay(pdMS_TO_TICKS(250));
	}	


	while(1)
	{
		float current[3], voltage[3];
		current[0] = ina3221.getCurrent(INA3221_CH1);
		voltage[0] = ina3221.getVoltage(INA3221_CH1);
		current[1] = ina3221.getCurrent(INA3221_CH2);
		voltage[1] = ina3221.getVoltage(INA3221_CH2);
		current[2] = ina3221.getCurrent(INA3221_CH3);
		voltage[2] = ina3221.getVoltage(INA3221_CH3);



		current[0] = 0.0f;
        voltage[0] = 0.0f;
        current[1] = 0.0f;
        voltage[1] = 0.0f;
        current[2] = 0.0f;
        voltage[2] = 0.0f;


		ESP_LOGI("MAIN", "%.2fV %.2fA   %.2fV %.2fA   %.2fV %.2fA", voltage[0], current[0], voltage[1], current[1], voltage[2], current[2]);

		labels[0].SetText("%dmV", current[0]);
		labels[1].SetText("%dmA", voltage[0]);
		labels[2].SetText("%dmV", current[1]);
		labels[3].SetText("%dmA", voltage[1]);
		labels[4].SetText("%dmV", current[2]);
		labels[5].SetText("%dmA", voltage[2]);



		vTaskDelay(pdMS_TO_TICKS(20));
	}

}



