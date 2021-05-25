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


#define MAX31865_CONFIG_REG 0x00
#define MAX31865_RTD_REG 0x01
#define MAX31865_HIGH_FAULT_REG 0x03
#define MAX31865_LOW_FAULT_REG 0x05
#define MAX31865_FAULT_STATUS_REG 0x07

#define MAX31865_REG_WRITE_OFFSET 0x80

#define MAX31865_CONFIG_VBIAS_BIT 7
#define MAX31865_CONFIG_CONVERSIONMODE_BIT 6
#define MAX31865_CONFIG_1SHOT_BIT 5
#define MAX31865_CONFIG_NWIRES_BIT 4
#define MAX31865_CONFIG_FAULTDETECTION_BIT 2
#define MAX31865_CONFIG_FAULTSTATUS_BIT 1
#define MAX31865_CONFIG_MAINSFILTER_BIT 0

#define POSITIVE_BCD  0x12
#define NEGATIVE_BCD  0x13

//Wires Enum
enum Max31865NWires { Three = 1, Two = 0, Four = 0 };

//Fault Detection enumeration
enum Max31865FaultDetection {
  NoAction = 0b00,
  AutoDelay = 0b01,
  ManualDelayCycle1 = 0b10,
  ManualDelayCycle2 = 0b11
};
//enumeration for Filter (default = 60)
enum Max31865Filter { Hz50 = 1, Hz60 = 0 };

//Error enum
enum Max31865Error {
  NoError = 0,
  Voltage = 2,
  RTDInLow,
  RefLow,
  RefHigh,
  RTDLow,
  RTDHigh
};

//MAX31865 Config
struct max31865_config_t {
  bool vbias;
  bool autoConversion;
  uint8_t nWires;
  uint8_t faultDetection;
  uint8_t filter;
};

//RTD Configuration
struct max31865_rtd_config_t {
  float ref;
  float nominal;
};

float RTD_A = 3.9083e-3;
float RTD_B = -5.775e-7; //constants

//Pin Configuration
int miso = 19; //SDO
int mosi = 23; //SDI
int sck = 18;  //SCLK
int cs = 5;   //CS
int drdy = 16; //DRDY //Set to -1 if unused

spi_host_device_t hostDevice = SPI3_HOST;

struct max31865_config_t chipConfig;
spi_device_handle_t deviceHandle;
SemaphoreHandle_t drdySemaphore;



//Function declarations
 float RTDtoTemperature(uint16_t rtd, struct max31865_rtd_config_t rtdConfig) {
	  float Rrtd = (rtd * rtdConfig.ref) / (1U << 15U);
	  float temperature;
	  temperature = (RTD_A * RTD_A - (4 * RTD_B)) + (((4 * RTD_B) / rtdConfig.nominal) * Rrtd);
	  temperature = (sqrt(temperature) - RTD_A) / (2 * RTD_B);

	  if (temperature > 0.0) {
	    return temperature;
	  } // otherwise must use different function

	  Rrtd /= rtdConfig.nominal;
	  Rrtd *= 100.0;
	  return -242.02 + 2.2228 * Rrtd + 2.5859e-3 * pow(Rrtd, 2) - 4.8260e-6 * pow(Rrtd, 3) -
			  2.8183e-8 * pow(Rrtd, 4) + 1.5243e-10 * pow(Rrtd, 5);
	}

 /*
static uint16_t temperatureToRTD(float temperature,
                                struct max31865_rtd_config_t rtdConfig){
	  float Rrtd = rtdConfig.nominal *
	               (1.0 + RTD_A * temperature + RTD_B * pow(temperature, 2));
	  if (temperature < 0.0) {
	    Rrtd +=
	        rtdConfig.nominal * RTD_C * (temperature - 100.0) * pow(temperature, 3);
	  }
	  return lroundf(Rrtd * (1U << 15U) / rtdConfig.ref);
	} */

const char *errorToString(uint8_t error) {
  switch (error) {
    case NoError: { return "No error"; }
    case Voltage: { return "Over/under voltage fault"; }
    case RTDInLow: { return "RTDIN- < 0.85*VBIAS (FORCE- open)"; }
    case RefLow: { return "REFIN- < 0.85*VBIAS (FORCE- open)"; }
    case RefHigh: { return "REFIN- > 0.85*VBIAS"; }
    case RTDLow: { return "RTD below low threshold"; }
    case RTDHigh: { return "RTD above high threshold"; }
  }
  return "";
}

static void drdyInterruptHandler(void *arg) {
	  static BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	  xSemaphoreGiveFromISR((SemaphoreHandle_t) arg, &xHigherPriorityTaskWoken);
	  if (xHigherPriorityTaskWoken) {
	    portYIELD_FROM_ISR()
	  }
	}

static const char *TAG = "Max31865";

esp_err_t writeSPI(uint8_t addr, uint8_t *data, size_t size) {
	  assert(size <= 4);  // we're using the transaction buffers
	  spi_transaction_t transaction = {};
	  transaction.length = CHAR_BIT * size;
	  transaction.rxlength = 0;
	  transaction.addr = addr | MAX31865_REG_WRITE_OFFSET;
	  transaction.flags = SPI_TRANS_USE_TXDATA;
	  memcpy(transaction.tx_data, data, size);
	  gpio_set_level(cs, 0);
	  esp_err_t err = spi_device_polling_transmit(deviceHandle, &transaction);
	  gpio_set_level(cs, 1);
	  return err;
	}

esp_err_t readSPI(uint8_t addr, uint8_t *result, size_t size){
	  assert(size <= 4);  // we're using the transaction buffers
	  spi_transaction_t transaction = {};
	  transaction.length = 0;
	  transaction.rxlength = CHAR_BIT * size;
	  transaction.addr = addr & (MAX31865_REG_WRITE_OFFSET - 1);
	  transaction.flags = SPI_TRANS_USE_RXDATA;
	  gpio_set_level((gpio_num_t)cs, 0);
	  esp_err_t err = spi_device_polling_transmit(deviceHandle, &transaction);
	  gpio_set_level((gpio_num_t)cs, 1);
	  if (err != ESP_OK) {
	    ESP_LOGE(TAG, "Error sending SPI transaction: %s", esp_err_to_name(err));
	    return err;
	  }
	  memcpy(result, transaction.rx_data, size);
	  return ESP_OK;
	}

esp_err_t setConfig(struct max31865_config_t config){
	  chipConfig = config;
	  uint8_t configByte = 0;
	  if (config.vbias) { configByte |= 1UL << MAX31865_CONFIG_VBIAS_BIT; }
	  if (config.autoConversion) { configByte |= 1UL << MAX31865_CONFIG_CONVERSIONMODE_BIT; }
	  if (config.nWires == Three) { configByte |= 1UL << MAX31865_CONFIG_NWIRES_BIT; }
	  if (config.faultDetection != NoAction) { configByte |= (uint8_t)config.faultDetection << MAX31865_CONFIG_FAULTDETECTION_BIT; }
	  if (config.filter != Hz60) { configByte |= 1UL << MAX31865_CONFIG_MAINSFILTER_BIT; }
	  return writeSPI(MAX31865_CONFIG_REG, &configByte, 1);
	}

esp_err_t getConfig(struct max31865_config_t *config){
	  uint8_t configByte = 0;
	  esp_err_t err = readSPI(MAX31865_CONFIG_REG, &configByte, 1);
	  if (err != ESP_OK) {
	    ESP_LOGE(TAG, "Error reading config: %s", esp_err_to_name(err));
	    return err;
	  }
	  config->vbias = ((configByte >> MAX31865_CONFIG_VBIAS_BIT) & 1U) != 0;
	  config->autoConversion =
	      ((configByte >> MAX31865_CONFIG_CONVERSIONMODE_BIT) & 1U) != 0;
	  config->nWires = ((configByte >> MAX31865_CONFIG_NWIRES_BIT) & 1U);
	  config->faultDetection = ((configByte >> MAX31865_CONFIG_FAULTDETECTION_BIT) & 1U);
	  config->filter = ((configByte >> MAX31865_CONFIG_MAINSFILTER_BIT) & 1U);
	  return ESP_OK;
	}


esp_err_t begin(struct max31865_config_t config)  {
	//CS configuration
	  gpio_config_t gpioConfig = {};
	  gpioConfig.intr_type = GPIO_INTR_DISABLE;
	  gpioConfig.mode = GPIO_MODE_OUTPUT;
	  gpioConfig.pull_up_en = GPIO_PULLUP_ENABLE;
	  gpioConfig.pin_bit_mask = 1ULL << cs;
	  gpio_config(&gpioConfig);

	  if (drdy > -1) {
		  //drdy configuration
	    gpio_config_t gpioConfig = {};
	    gpioConfig.intr_type = GPIO_INTR_NEGEDGE;
	    gpioConfig.mode = GPIO_MODE_INPUT;
	    gpioConfig.pull_up_en = GPIO_PULLUP_ENABLE;
	    gpioConfig.pin_bit_mask = 1ULL << drdy;
	    gpio_config(&gpioConfig);

	    drdySemaphore = xSemaphoreCreateBinary();
	    // There won't be a negative edge interrupt if it's already low
	    if ((gpio_num_t*)drdy == 0) {
	      xSemaphoreGive(drdySemaphore);
	    }

	    gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
	    gpio_isr_handler_add(drdy, &drdyInterruptHandler,
	                         drdySemaphore);
	  }

	  spi_bus_config_t busConfig = {};
	  busConfig.miso_io_num = miso;
	  busConfig.mosi_io_num = mosi;
	  busConfig.sclk_io_num = sck;
	  busConfig.quadhd_io_num = -1;
	  busConfig.quadwp_io_num = -1;
	  esp_err_t err = spi_bus_initialize(hostDevice, &busConfig, 0);
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
	  deviceConfig.address_bits = CHAR_BIT;
	  deviceConfig.command_bits = 0;
	  deviceConfig.flags = SPI_DEVICE_HALFDUPLEX;
	  deviceConfig.queue_size = 1;
	  err = spi_bus_add_device(hostDevice, &deviceConfig, &deviceHandle);
	  if (err != ESP_OK) {
	    ESP_LOGE(TAG, "Error adding SPI device: %s", esp_err_to_name(err));
	    return err;
	  }
	  return setConfig(config);
	}


esp_err_t setRTDThresholds(uint16_t min, uint16_t max) {
	  assert((min < (1 << 15)) && (max < (1 << 15)));
	  uint8_t thresholds[4];
	  thresholds[0] = (uint8_t)((max << 1) >> CHAR_BIT);
	  thresholds[1] = (uint8_t)(max << 1);
	  thresholds[2] = (uint8_t)((min << 1) >> CHAR_BIT);
	  thresholds[3] = (uint8_t)(min << 1);
	  return writeSPI(MAX31865_HIGH_FAULT_REG, thresholds, sizeof(thresholds));
	}

esp_err_t clearFault(){
  uint8_t configByte = 0;
  esp_err_t err = readSPI(MAX31865_CONFIG_REG, &configByte, 1);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Error reading config: %s", esp_err_to_name(err));
    return err;
  }
  configByte &= ~(1U << MAX31865_CONFIG_FAULTSTATUS_BIT);
  return writeSPI(MAX31865_CONFIG_REG, &configByte, 1);
}

esp_err_t readFaultStatus(uint8_t *fault){
	  *fault = NoError;
	  uint8_t faultByte = 0;
	  esp_err_t err = readSPI(MAX31865_FAULT_STATUS_REG, &faultByte, 1);
	  if (err != ESP_OK) {
	    ESP_LOGE(TAG, "Error reading fault status: %s", esp_err_to_name(err));
	    return err;
	  }
	  if (faultByte != 0) {
	    *fault = (CHAR_BIT * sizeof(unsigned int) - 1 -
	                                        __builtin_clz(faultByte));
	  }
	  return clearFault();
	}

esp_err_t getRTD(uint16_t *rtd, uint8_t *fault){
	  struct max31865_config_t oldConfig = chipConfig;
	  bool restoreConfig = false;
	  if (!chipConfig.vbias) {
	    restoreConfig = true;
	    chipConfig.vbias = true;
	    esp_err_t err = setConfig(chipConfig);
	    if (err != ESP_OK) {
	      ESP_LOGE(TAG, "Error setting config: %s", esp_err_to_name(err));
	      return err;
	    }
	    vTaskDelay(pdMS_TO_TICKS(10));
	  }
	  if (!chipConfig.autoConversion) {
	    restoreConfig = true;
	    uint8_t configByte = 0;
	    esp_err_t err = readSPI(MAX31865_CONFIG_REG, &configByte, 1);
	    if (err != ESP_OK) {
	      ESP_LOGE(TAG, "Error reading config: %s", esp_err_to_name(err));
	      return err;
	    }
	    configByte |= 1U << MAX31865_CONFIG_1SHOT_BIT;
	    err = writeSPI(MAX31865_CONFIG_REG, &configByte, 1);
	    if (err != ESP_OK) {
	      ESP_LOGE(TAG, "Error writing config: %s", esp_err_to_name(err));
	      return err;
	    }
	    vTaskDelay(pdMS_TO_TICKS(65));
	  } else if (drdy > -1) {
	    xSemaphoreTake(drdySemaphore, portMAX_DELAY);
	  }

	  uint8_t rtdBytes[2];
	  esp_err_t err = readSPI(MAX31865_RTD_REG, rtdBytes, 2);
	  if (err != ESP_OK) {
	    ESP_LOGE(TAG, "Error reading RTD: %s", esp_err_to_name(err));
	    return err;
	  }

	  if ((bool)(rtdBytes[1] & 1U)) {
	    *rtd = 0;
	    if (fault == NULL) {
	      auto uint8_t tmp = NoError;
	      fault = &tmp;
	    }
	    readFaultStatus(fault);
	    ESP_LOGW(TAG, "Sensor fault detected: %s", errorToString(*fault));
	    return ESP_ERR_INVALID_RESPONSE;
	  }

	  *rtd = rtdBytes[0] << CHAR_BIT;
	  *rtd |= rtdBytes[1];
	  *rtd >>= 1U;

	  return restoreConfig ? setConfig(oldConfig) : ESP_OK;
	}

uint32_t decimal_to_bcd(uint32_t num)
	{
		return ((num / 10000 * 9256) + ((num % 10000) / 1000 * 516) +
            ((num % 1000) / 100 * 26) + (num % 100) / 10) * 6 + num;
	}

/* Convert float value to binary coded decimal format.
 * It takes eight bits to store fractional part of float and sixteen bits for integral part.
 * Last eight bits store sign of value */
uint32_t float_to_bcd(float num)
	{
    	uint32_t bcd = 0;
    	float integral;
    	float fractional = modff(fabs(num), &integral);

    	bcd |= decimal_to_bcd(integral) << 8;
    	bcd |= decimal_to_bcd(fractional * 100); // get first two digits from fractional part
    	bcd |= num >= 0 ? POSITIVE_BCD << 24 : NEGATIVE_BCD << 24;

    	return bcd;
	}

/*
void app_main(void)
{
	struct max31865_config_t tempConfig = {};
	  tempConfig.autoConversion = true;
	  tempConfig.vbias = false;
	  tempConfig.filter = Hz60;
	  tempConfig.nWires = Four;
	 struct max31865_rtd_config_t rtdConfig = {};
	  rtdConfig.nominal = 100.0f;
	  rtdConfig.ref = 432.0f;
	  ESP_ERROR_CHECK(begin(tempConfig));
	  ESP_ERROR_CHECK(setRTDThresholds(0x1000, 0x2500));

	  while (true) {
	      uint16_t rtd;
	      uint8_t fault = NoError;
	      ESP_ERROR_CHECK(getRTD(&rtd, &fault));
	      float temp = RTDtoTemperature(rtd, rtdConfig);
	      printf("Temperature: %.2f C \n",temp);
	      vTaskDelay(pdMS_TO_TICKS(1000));
	    }
}
*/
