#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "soil_sensor/soil_sensor.h"
#include "nvs_flash.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "mqtt_client.h"

#include "esp_system.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_wifi.h"

#include "lwip/err.h"
#include "lwip/sys.h"


#include "buffer/ring_buffer.h"

#define RED_PIN GPIO_NUM_4
#define BLUE_PIN GPIO_NUM_5
#define GREEN_PIN GPIO_NUM_6
#define LIGHT_SENSOR GPIO_NUM_0
#define SDA_PIN 2
#define SCL_PIN 3
#define EXAMPLE_PIN_NUM_RST -1

#define I2C_TIMEOUT_MS 2000
#define I2C_SLV_BUFSIZE 0 //Only operating in master mode, buffer size 0

#define AM_ADDR 0x5C
#define AM_WRITE_CMD 0x10
#define AM_READ_CMD 0x03
#define AM_HI_HUM 0x00
#define AM_LO_HUM 0x01
#define AM_HI_TEMP 0x02
#define AM_LO_TEMP 0x03

#define AM_DELAY_1 (2)
#define AM_DELAY_2 (3)

#define ESP_WIFI_SSID "Oliver - iPhone"
#define ESP_WIFI_PW "Oliven13"
#define ESP_MAX_RETRY 5

#define MQTT_BROKER "mqtt://broker.emqx.io"
#define MQTT_TOPIC "Fiedler/Test"

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static const char *TAG = "wifi station";
static int s_retry_num = 0;
static esp_mqtt_client_handle_t client;

static EventGroupHandle_t s_wifi_event_group;

void init_mqtt() {
esp_mqtt_client_config_t mqtt_cfg = {
    .broker.address.uri = MQTT_BROKER
  };

client = esp_mqtt_client_init(&mqtt_cfg);
}

static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < ESP_MAX_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG,"connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = ESP_WIFI_SSID,
            .password = ESP_WIFI_PW,
            /* Setting a password implies station will connect to all security modes including WEP/WPA.
             * However these modes are deprecated and not advisable to be used. Incase your Access point
             * doesn't support WPA2, these mode can be enabled by commenting below line */
	     .threshold.authmode = WIFI_AUTH_WPA2_PSK,

            .pmf_cfg = {
                .capable = true,
                .required = false
            },
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(TAG, "wifi_init_sta finished.");

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "connected to ap SSID:%s password:%s",
                 ESP_WIFI_SSID, ESP_WIFI_PW);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
                 ESP_WIFI_SSID, ESP_WIFI_PW);
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }

    /* The event will not be processed after unregister */
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, instance_got_ip));
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, instance_any_id));
    vEventGroupDelete(s_wifi_event_group);
}


void send_mqtt(char* s, int size) {

  esp_err_t err = esp_mqtt_client_start(client);

  if (err != ESP_OK) {
    printf("Error setting up MQTT\n");
  }
  else {
    printf("MQTT Succesful\n");
  }

  int x = esp_mqtt_client_publish(client, MQTT_TOPIC, s, size, 1, 0);

  esp_mqtt_client_stop(client);

  printf("Result of publish: %d\n", x);
}

void setupRGB();

void setup_light_sensor(adc_oneshot_unit_handle_t *handle, adc_oneshot_unit_init_cfg_t *adc_config);

void turnLightBlue();

void turnLightGreen(); 

void turnLightRed();

void turnLightOff();

void ChangeLight(short unsigned int* prev);

void printBuffer(struct Ring_buffer *buffer);

esp_err_t wake_am(i2c_port_t port, uint8_t am_addr);

esp_err_t read_am_temp(i2c_port_t port, uint8_t am_addr, uint16_t* temp_reading);

void fillSample(struct Sample *s, i2c_port_t i2c_port, adc_oneshot_unit_handle_t *handle, adc_oneshot_unit_init_cfg_t *config);

void prepare_mqtt_packet(struct Ring_buffer *buffer, char *s);

void app_main() {
  esp_err_t ret = nvs_flash_init();
  wifi_init_sta();
  init_mqtt();
  adc_oneshot_unit_handle_t handle;
  adc_oneshot_unit_init_cfg_t adc_config = {.unit_id = ADC_UNIT_1};
  i2c_port_t i2c_port;
  esp_err_t err;
  i2c_port = setup_i2c(SDA_PIN, SCL_PIN);
  setup_light_sensor(&handle, &adc_config);
  setupRGB();


  char packet[50];
  int ctr = 0;
  struct Sample newSample;
  struct Ring_buffer *buffer = init_buffer(10);

  while(true) {
    fillSample(&newSample, i2c_port, &handle, &adc_config);
    input_sample(buffer, &newSample);
    if (ctr == buffer->size) {
      print_buffer(buffer);
      ctr = 0;
      prepare_mqtt_packet(buffer, packet);
      send_mqtt(packet, strlen(packet));
    }
    ctr++;
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void fillSample(struct Sample *s, i2c_port_t i2c_port, adc_oneshot_unit_handle_t *handle, adc_oneshot_unit_init_cfg_t *config) {
  esp_err_t err;
  uint16_t temp_reading;
  s->moisture = read_soil_sensor(i2c_port);
  err = read_am_temp(i2c_port, AM_ADDR, &temp_reading);
  if (err == ESP_OK) {
    s->temperature = (float) temp_reading / 10.0;
  } else {
    printf("ERR unable to read TEMP: %x (%s)\n", err, esp_err_to_name(err));
  }
  err = adc_oneshot_read(handle, ADC_CHANNEL_0, &s->light_level);
  if (err != ESP_OK) {
    printf("ADC failed\n");
  }
  ChangeLight(&s->moisture);
}

void prepare_mqtt_packet(struct Ring_buffer *buffer, char *s) {
  struct Buffer_element *element = buffer->head;
  struct Sample *sample = element->values;
  float* temp = malloc(sizeof(float)); 
  *temp = 0;
  int* light_level = malloc(sizeof(int));
  *light_level = 0;
  unsigned short* moisture = malloc(sizeof(unsigned short)); 
  *moisture = 0;
  for (int i = 0; i < buffer->size; i++)
  {
    *temp += sample->temperature;
    *light_level += sample->light_level;
    *moisture += sample->moisture;
    element = element->next; 
    sample = element->values;
  }

  *temp /= buffer->size;
  *light_level /= buffer->size;
  *moisture /= buffer->size;

  sprintf(s, "Sample average is: Temp: %f, Light: %d, Moisture: %u\n", *temp, *light_level, *moisture);
  free(temp);
  free(light_level);
  free(moisture);
}

esp_err_t wake_am(i2c_port_t port, uint8_t am_addr) {
  esp_err_t res; 
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd); 
  i2c_master_write_byte(cmd, am_addr << 1, true); 
  i2c_master_stop(cmd);

  res = i2c_master_cmd_begin(port, cmd, I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
  i2c_cmd_link_delete(cmd);

  return res;
}

esp_err_t read_am_temp(i2c_port_t port, uint8_t am_addr, uint16_t* temp_reading) {
  esp_err_t err;
  uint8_t read_cmd[] = {AM_READ_CMD, AM_HI_TEMP, 2}; //2 because we're reading two items
  uint8_t rbuf[6]; 

  for(int i = 0; i < 5; i++) {
    err = wake_am(port, am_addr);
    if (err == ESP_OK) {
      break;
    }
    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
  if (err != ESP_OK) {
    ESP_LOGI("I2C", "AM did not respond: %s", esp_err_to_name(err));
  }
  vTaskDelay(AM_DELAY_1 / portTICK_PERIOD_MS);

  if (err != ESP_OK) {
    ESP_LOGI("I2C", "AM did not respond to wake command %s", esp_err_to_name(err)); 
    return err;
  }

  vTaskDelay(AM_DELAY_1 / portTICK_PERIOD_MS);

  err = i2c_master_write_to_device(port, am_addr, read_cmd, 3, I2C_TIMEOUT_MS);

  if (err != ESP_OK) {
    ESP_LOGI("I2C", "am2320 did not respond to read command: %s", esp_err_to_name(err));
    return err;
  }
  vTaskDelay(AM_DELAY_2 / portTICK_PERIOD_MS);

  err = i2c_master_read_from_device(port, am_addr, rbuf, 6, I2C_TIMEOUT_MS);
  if (err != ESP_OK) {
    ESP_LOGI("I2C", "am2320 did not return read data: %s", esp_err_to_name(err));
    return err;
  }

  if (rbuf[1] != 2) {
    ESP_LOGI("I2C", "am2320 did not return expected number of bytes (got %d)", rbuf[1]);
  }
  *temp_reading = (((uint16_t) rbuf[2]) << 8) | ((uint16_t) rbuf[3]);
  return ESP_OK; 
}

void turnLightGreen() {
  gpio_set_level(GREEN_PIN, false);
  gpio_set_level(RED_PIN, true);
  gpio_set_level(BLUE_PIN, true);
}

void turnLightRed() {
  gpio_set_level(GREEN_PIN, true);
  gpio_set_level(RED_PIN, false);
  gpio_set_level(BLUE_PIN, true);
}

void turnLightBlue() {
  gpio_set_level(GREEN_PIN, true);
  gpio_set_level(RED_PIN, true);
  gpio_set_level(BLUE_PIN, false);
}

void turnLightOff() {
  gpio_set_level(GREEN_PIN, true);
  gpio_set_level(RED_PIN, true);
  gpio_set_level(BLUE_PIN, true);
}

void ChangeLight(short unsigned int* prev) {
  if (*prev <= 400) {
    turnLightRed();
  } else if (*prev <= 700) {
    turnLightGreen(); 
  } else {
    turnLightBlue();
  }
}


void setupRGB() {
  printf("LED set up\n");
  gpio_reset_pin(RED_PIN);
  gpio_reset_pin(BLUE_PIN);
  gpio_reset_pin(GREEN_PIN);
  //Set the direction of the pin, configuring it as an output pin
  gpio_set_direction(RED_PIN, GPIO_MODE_OUTPUT);
  gpio_set_direction(BLUE_PIN, GPIO_MODE_OUTPUT);
  gpio_set_direction(GREEN_PIN, GPIO_MODE_OUTPUT);
}

void setup_light_sensor(adc_oneshot_unit_handle_t *handle, adc_oneshot_unit_init_cfg_t *adc_config) {
  esp_err_t err;

  //Set up and get ADC oneshot handle
  
  err = adc_oneshot_new_unit(adc_config, handle);
  if (err != ESP_OK) {
    printf("ERR: ADC Oneshot not set up: %x\n", err);
  } else {
    printf("ADC Oneshot was set up\n");
  }

  //Configure the unit
  adc_oneshot_chan_cfg_t chan_cfg = {
    .atten = ADC_ATTEN_DB_0,
    .bitwidth = ADC_BITWIDTH_DEFAULT
  };

  err = adc_oneshot_config_channel(handle, ADC_CHANNEL_0, &chan_cfg); //Channel 0 = GPIO 0
  if (err != ESP_OK) {
    printf("ERR: ADC channel not configured: %x\n", err);
  } else {
    printf("ADC channel was configured\n");
  }
}