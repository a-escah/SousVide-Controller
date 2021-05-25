#include <driver/gpio.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <string.h>
#include <limits.h>
#include <driver/spi_common.h>
#include <driver/spi_master.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <math.h>

int DAC_sdi = 13; //SDI
int DAC_ldac = 12; //ldac
int DAC_sck = 14;  //SCLK
int DAC_cs = 15;   //CS

spi_device_handle_t device_Handle;
spi_host_device_t DACspi = SPI2_HOST;

esp_err_t DACsetup() {
//CS configuration
	  gpio_config_t gpioConfig = {};
	  gpioConfig.intr_type = GPIO_INTR_DISABLE;
	  gpioConfig.mode = GPIO_MODE_OUTPUT;
	  gpioConfig.pull_up_en = GPIO_PULLUP_ENABLE;
	  gpioConfig.pin_bit_mask = 1ULL << DAC_cs;
	  gpio_config(&gpioConfig);
	  
	  spi_bus_config_t busConfig = {};
	  busConfig.miso_io_num = DAC_sdi;
	  busConfig.sclk_io_num = DAC_sck;
	  busConfig.quadhd_io_num = -1;
	  busConfig.quadwp_io_num = -1;
	  esp_err_t err = spi_bus_initialize(DACspi, &busConfig, 0);
	  // INVALID_STATE means the host is already in use - that's OK
	  if (err == ESP_ERR_INVALID_STATE) {
	    ESP_LOGD(TAG, "SPI bus already initialized");
	  } else if (err != ESP_OK) {
	    ESP_LOGE(TAG, "Error initializing SPI bus: %s", esp_err_to_name(err));
	    return err;
	  }

	  spi_device_interface_config_t deviceConfig = {};
	  deviceConfig.spics_io_num = -1;  // ESP32's hardware CS is too quick
	  deviceConfig.clock_speed_hz = 3000000;
	  deviceConfig.mode = 1;
	  deviceConfig.address_bits = 0;
	  deviceConfig.command_bits = 0;
	  deviceConfig.flags = SPI_DEVICE_HALFDUPLEX;
	  deviceConfig.queue_size = 1;
	  err = spi_bus_add_device(DACspi, &deviceConfig, &device_Handle);
	  if (err != ESP_OK) {
	    ESP_LOGE(TAG, "Error adding SPI device: %s", esp_err_to_name(err));
	    return err;
	  }
	  return err;
	  }

int gain = 0;

void set_gain(int a){
	if (a == 1) {gain = 1;}
	else {gain = 0;}
}

void write_to_dac(int spi_message){
	printf("DAC message: %d  \n",spi_message);
	spi_transaction_t t1;
	memset(&t1, 0, sizeof(t1)); //Zero out the transaction
	t1.length = 16;
	t1.flags = SPI_TRANS_USE_TXDATA;
	t1.tx_data[0] = ((spi_message >> 8) & 0xff);
	t1.tx_data[1] = (spi_message & 0xff);
	gpio_set_level(DAC_cs, 0);
	spi_device_transmit(device_Handle, &t1);
	//spi_device_queue_trans(device_Handle, &t1, portMAX_DELAY);
	gpio_set_level(DAC_cs, 1);
}

void disable_DAC(int a){
	int DACmessage = 0;
	if (a == 1){ DACmessage = 1<<15;} //Channel B
	else {DACmessage = 0;} //Channel A
	write_to_dac(DACmessage);
}

void set_Dac_out(double voltage){
	if (voltage == 0) {
		disable_DAC(1);
	}
	else{
		if (voltage > 4.096) { voltage = 4.096; } // set max voltage amount;
		int n = 0;
		int DACmessage = 0;
		n = voltage*1000;
		DACmessage = 1<<15| gain<<13| 1<<12| n;
		write_to_dac(DACmessage);
	}
}

//void set_Dac_out(int voltage){
	//if (voltage > 4095) { voltage = 4095; } // set max voltage amount;
	//int DACmessage = 0;
	//DACmessage = 1<<15| gain<<13| 1<<12| voltage;
	//printf("DAC message: %d  \n",DACmessage);
	//write_to_dac(DACmessage);
//}


