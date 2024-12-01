
#include <esp_wifi.h>
#include <esp_event.h>
#include <esp_log.h>
#include <esp_system.h>
#include <nvs_flash.h>
#include <sys/param.h>
#include "esp_netif.h"
#include "esp_eth.h"
#include "protocol_examples_common.h"
#include <esp_https_server.h>
#include "esp_tls.h"
#include <string.h>
#include "driver/gpio.h"
#include <stdio.h>
#include "driver/adc.h"
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc_cal.h"
#include "sdkconfig.h"
#include "HX711.h"
#include "esp_timer.h"

#define DEFAULT_VREF 1100
#define NO_OF_SAMPLES 64

// khai báo các giá trị dự báo tình trạng cây trồng
static char my_string_1[100] = "Binh Thuong";
static char my_string_2[100] = "Kho Heo";
static char my_string_3[100] = "Ung Nuoc";
static char my_string_4[100] = "Thieu Anh Sang";
static char my_string_5[100] = "May bom vao dang bat";
static char my_string_6[100] = "May bom vao dang tat";
static char my_string_7[100] = "May bom ra dang bat";
static char my_string_8[100] = "May bom ra dang tat";

int bomnuoc_1 =0;       // tình trạng máy bơm  vào
int bomnuoc_2 =0;    // tình trạng máy bơm ra

// Khai báo chân cảm biến độ ẩm đất
static esp_adc_cal_characteristics_t *adc_chars; 
static const adc_channel_t channel = ADC_CHANNEL_0;
static const adc_atten_t atten = ADC_ATTEN_DB_11; 
static const adc_unit_t unit = ADC_UNIT_1;

// Khai báo chân của loadcell HX711
#define AVG_SAMPLES   10
#define GPIO_DATA   GPIO_NUM_15
#define GPIO_SCLK   GPIO_NUM_16

//khai báo chân

#define DHT_PIN GPIO_NUM_4 

#define I2C_MASTER_SCL_IO 22    // Chân SCL của ESP32
#define I2C_MASTER_SDA_IO 21    // Chân SDA của ESP32
#define I2C_MASTER_FREQ_HZ 100000 // Tần số I2C 100 kHz

volatile   int16_t temperature = 0; 
volatile int16_t humidity = 0;
volatile  uint16_t lux = 0;


static SemaphoreHandle_t data_moisure_sensor_semaphore; // Semaphore bảo vệ data_moisure_sensor

static SemaphoreHandle_t lux_semaphore; // Semaphore bảo vệ cảm biến ánh sáng
static SemaphoreHandle_t humid_semaphore; // Semaphore bảo vệ sensor
static SemaphoreHandle_t temp_semaphore; // Semaphore bảo vệ sensor


volatile int data_moisure_sensor = 0;


static const char *TAG_LOADCELL = "HX711_TEST";

static void weight_reading_task(void *arg);
static void initialise_weight_sensor(void);


float  zero = 838752.765;
float load      ;
float weight = 0;





void delay_us(uint32_t us){ 
    uint32_t start = esp_timer_get_time(); 
    while ((esp_timer_get_time() - start) < us) { 
        // Busy wait 
    }
}

void i2c_start() {
    gpio_set_level(I2C_MASTER_SDA_IO, 1);
    gpio_set_level(I2C_MASTER_SCL_IO, 1);
    delay_us(5);
    gpio_set_level(I2C_MASTER_SDA_IO, 0);
    delay_us(5);
    gpio_set_level(I2C_MASTER_SCL_IO, 0);
}

void i2c_stop() {
    gpio_set_level(I2C_MASTER_SDA_IO, 0);
    gpio_set_level(I2C_MASTER_SCL_IO, 1);
    delay_us(5);
    gpio_set_level(I2C_MASTER_SDA_IO, 1);
    delay_us(5);
}

void i2c_write_bit(int bit) {
    gpio_set_level(I2C_MASTER_SDA_IO, bit);
    delay_us(5);
    gpio_set_level(I2C_MASTER_SCL_IO, 1);
    delay_us(5);
    gpio_set_level(I2C_MASTER_SCL_IO, 0);
    delay_us(5);
}

int i2c_read_bit() {
    gpio_set_direction(I2C_MASTER_SDA_IO, GPIO_MODE_INPUT);
    delay_us(5);
    gpio_set_level(I2C_MASTER_SCL_IO, 1);
    int bit = gpio_get_level(I2C_MASTER_SDA_IO);
    delay_us(5);
    gpio_set_level(I2C_MASTER_SCL_IO, 0);
    gpio_set_direction(I2C_MASTER_SDA_IO, GPIO_MODE_OUTPUT);
    return bit;
}

void i2c_write_byte(uint8_t byte) {
    for (int i = 0; i < 8; i++) {
        i2c_write_bit((byte >> (7 - i)) & 1);
    }
    i2c_read_bit(); // Đọc ACK/NACK
}

uint8_t i2c_read_byte(int ack) {
    uint8_t byte = 0;
    for (int i = 0; i < 8; i++) {
        byte = (byte << 1) | i2c_read_bit();
    }
    i2c_write_bit(ack);
    return byte;
}

void read_dht22(int16_t *temperature, int16_t *humidity) { 
    uint8_t data[5] = {0};
    // Send start signal 
    gpio_set_direction(DHT_PIN, GPIO_MODE_OUTPUT); 
    gpio_set_level(DHT_PIN, 0); 
    delay_us(20000); // 20ms 
    gpio_set_level(DHT_PIN, 1); 
    delay_us(40); // 40us 
    gpio_set_direction(DHT_PIN, GPIO_MODE_INPUT);
    // Wait for response 
    while (gpio_get_level(DHT_PIN) == 1); 
    while (gpio_get_level(DHT_PIN) == 0); 
    while (gpio_get_level(DHT_PIN) == 1);
    // Read data 
    for (int i = 0; i < 40; i++) { 
        while (gpio_get_level(DHT_PIN) == 0); 
        delay_us(28); 
        if (gpio_get_level(DHT_PIN) == 1) { 
            data[i / 8] |= (1 << (7 - (i % 8))); 
        } 
        while (gpio_get_level(DHT_PIN) == 1); 
    }
    // Checksum 
    if (data[4] == ((data[0] + data[1] + data[2] + data[3]) & 0xFF)) { 
        *humidity = data[0] << 8 | data[1]; 
        *temperature = data[2] << 8 | data[3]; 
    } else { 
        printf("Checksum failed\n");
    } 
}

void init_adc(){
    if (unit == ADC_UNIT_1){
        adc1_config_width(ADC_WIDTH_BIT_12); 
        adc1_config_channel_atten(channel,atten);
    } else {
        adc2_config_channel_atten((adc2_channel_t)channel,atten);
    }
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
}

int read_adc(){
    int adc_reading = 0;
    for (int i = 0; i < NO_OF_SAMPLES; i++) {
        if (unit == ADC_UNIT_1){
            adc_reading += adc1_get_raw((adc1_channel_t)channel);
        } else {
            int raw;
            adc2_get_raw((adc2_channel_t)channel, ADC_WIDTH_BIT_12, &raw); 
            adc_reading += raw;
        }
    }
    adc_reading /= NO_OF_SAMPLES;
    return adc_reading;
}






static void initialise_weight_sensor(void)
{
    ESP_LOGI(TAG_LOADCELL, "****************** Initializing weight sensor **********");
    xTaskCreatePinnedToCore(weight_reading_task, "weight_reading_task", 4096, NULL, 1, NULL, 0);
}

static const char *TAG = "main";

/* HTTP GET handler */
static esp_err_t root_get_handler(httpd_req_t *req)
{
    extern unsigned char view_start[] asm("_binary_view_html_start");
    extern unsigned char view_end[] asm("_binary_view_html_end");
    size_t view_len = view_end - view_start;
    char viewHtml[view_len];
    memcpy(viewHtml, view_start, view_len);

    ESP_LOGI(TAG, "URI: %s", req->uri);

     const char *plant_status = NULL;
        const char *pump_status_1 = NULL;
const char *pump_status_2 = NULL;
    // Lấy giá trị cảm biến một cách an toàn
    int moisure_sensor_value;
    

    // CHO CẢM BIẾN ĐỘ ẨM ĐẤT
    if (xSemaphoreTake(data_moisure_sensor_semaphore, pdMS_TO_TICKS(100)) == pdTRUE)
    {
        moisure_sensor_value = data_moisure_sensor;
        xSemaphoreGive(data_moisure_sensor_semaphore);
    }
    else
    {
        ESP_LOGE(TAG, "Failed to acquire semaphore (moisture)");
        moisure_sensor_value = -1; // Lỗi nếu không lấy được giá trị
    }

     // Giả lập logic phân loại tình trạng cây trồng (dựa trên giá trị cảm biến độ ẩm đất)
    if (data_moisure_sensor >= 60 && data_moisure_sensor <= 80)
    {
        plant_status = my_string_1; // Bình thường
    }
    else if (data_moisure_sensor < 60)
    {
        plant_status = my_string_2; // Khô héo
    }
    else if (data_moisure_sensor > 80)
    {
        plant_status = my_string_3; // Úng nước
    }
    else
    {
        plant_status = my_string_4; // Thiếu ánh sáng (hoặc giá trị mặc định khác)
    }


    if (bomnuoc_1 == 1) 
    { 
        pump_status_1 = my_string_5; //may bom vao dang bat
    }
    else 
    {
        pump_status_1= my_string_6; // may bom vao dang tat
    }


    if (bomnuoc_2 ==1) 
    { 
        pump_status_2= my_string_7; //may bom vao dang bat
    }
    else 
    {
        pump_status_2= my_string_8; // may bom vao dang tat
    }





    char *viewHtmlUpdated;
    int formattedStrResult_1 = asprintf(&viewHtmlUpdated, viewHtml, moisure_sensor_value,  load, lux, temperature, humidity );

    httpd_resp_set_type(req, "text/html");

     int formattedStrResult_2 = asprintf(&viewHtmlUpdated, viewHtml, plant_status);

     httpd_resp_set_type(req, "text/html");

    
     int formattedStrResult_3 = asprintf(&viewHtmlUpdated, viewHtml, pump_status_1);

     httpd_resp_set_type(req, "text/html");
     
     int formattedStrResult_4 = asprintf(&viewHtmlUpdated, viewHtml, pump_status_2);

     httpd_resp_set_type(req, "text/html");


    if (formattedStrResult_1> 0)
    {
        httpd_resp_send(req, viewHtmlUpdated, view_len);
        free(viewHtmlUpdated);
    }
    else
    {
        ESP_LOGE(TAG, "Error updating variables");
        httpd_resp_send(req, viewHtml, view_len);
    }


     if (formattedStrResult_2> 0)
    {
        httpd_resp_send(req, viewHtmlUpdated, strlen(viewHtmlUpdated));
        free(viewHtmlUpdated);
    }
    else
    {
        ESP_LOGE(TAG, "Error updating variables");
        httpd_resp_send(req, viewHtml, view_len);
    }





if (formattedStrResult_3> 0)
    {
        httpd_resp_send(req, viewHtmlUpdated, strlen(viewHtmlUpdated));
        free(viewHtmlUpdated);
    }
    else
    {
        ESP_LOGE(TAG, "Error updating variables");
        httpd_resp_send(req, viewHtml, view_len);
    }


if (formattedStrResult_4> 0)
    {
        httpd_resp_send(req, viewHtmlUpdated, strlen(viewHtmlUpdated));
        free(viewHtmlUpdated);
    }
    else
    {
        ESP_LOGE(TAG, "Error updating variables");
        httpd_resp_send(req, viewHtml, view_len);
    }




    return ESP_OK;
}

static const httpd_uri_t root = {
    .uri = "/",
    .method = HTTP_GET,
    .handler = root_get_handler
};

static httpd_handle_t start_webserver(void)
{
    httpd_handle_t server = NULL;

    // Start the httpd server
    ESP_LOGI(TAG, "Starting server");

    httpd_ssl_config_t conf = HTTPD_SSL_CONFIG_DEFAULT();
    conf.transport_mode = HTTPD_SSL_TRANSPORT_INSECURE;
    esp_err_t ret = httpd_ssl_start(&server, &conf);
    if (ESP_OK != ret)
    {
        ESP_LOGI(TAG, "Error starting server!");
        return NULL;
    }

    // Set URI handlers
    ESP_LOGI(TAG, "Registering URI handlers");
    httpd_register_uri_handler(server, &root);
    return server;
}

static void stop_webserver(httpd_handle_t server)
{
    // Stop the httpd server
    httpd_ssl_stop(server);
}

static void disconnect_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    httpd_handle_t *server = (httpd_handle_t *)arg;
    if (*server)
    {
        stop_webserver(*server);
        *server = NULL;
    }
}

static void connect_handler(void *arg, esp_event_base_t event_base,
                            int32_t event_id, void *event_data)
{
    httpd_handle_t *server = (httpd_handle_t *)arg;
    if (*server == NULL)
    {
        *server = start_webserver();
    }
}













void app_main(void)
{
    static httpd_handle_t server = NULL;
    
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &connect_handler, &server));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, &disconnect_handler, &server));
    ESP_ERROR_CHECK(example_connect());
    nvs_flash_init();
  //  initialise_weight_sensor();


      // setup Loadcell
    HX711_init(GPIO_DATA,GPIO_SCLK,eGAIN_128); 
    nvs_flash_init();
    
    
//setup bom
    gpio_config_t GPIO_Config = {};
    GPIO_Config.pin_bit_mask = (1<<23);
    GPIO_Config.mode = GPIO_MODE_OUTPUT;
    GPIO_Config.pull_up_en = GPIO_PULLUP_DISABLE;
    GPIO_Config.pull_down_en = GPIO_PULLDOWN_DISABLE;
    GPIO_Config.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&GPIO_Config);
    

    if (unit == ADC_UNIT_1)
    {
        adc1_config_width(ADC_WIDTH_BIT_12);
        adc1_config_channel_atten(channel, atten);
    }
    else
    {
        adc2_config_channel_atten((adc2_channel_t)channel, atten);
    }
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_characterize(unit, atten, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);




    // Tạo semaphore cho cảm biến độ ẩm đất
    data_moisure_sensor_semaphore = xSemaphoreCreateMutex();
    if (data_moisure_sensor_semaphore == NULL)
    {
        ESP_LOGE(TAG, "Failed to create semaphore");
        return;
    }


    //BH1750 VS DHT22

     init_adc();

    gpio_set_direction(I2C_MASTER_SCL_IO, GPIO_MODE_OUTPUT);
    gpio_set_direction(I2C_MASTER_SDA_IO, GPIO_MODE_OUTPUT);

    // while (1) { 
    //      vTaskDelay(pdMS_TO_TICKS(10)); // Nhường CPU 10ms
    //     read_dht22(&temperature, &humidity); 
    //     printf("ĐỘ ẨM MÔI TRƯỜNG: %d %% \nNHIỆT ĐỘ MÔI TRƯỜNG: %dºC\n", humidity / 10, temperature / 10);
        
    //     int adc_reading_1 = read_adc();
    //     int voltage_1 = esp_adc_cal_raw_to_voltage(adc_reading_1, adc_chars);  
    //     int value = (100 - ((float)(adc_reading_1 - 1800) / (4095 - 1800)) * 100);
    //     printf("ĐỘ ẨM ĐẤT:        %d %%\n", value);

    //     i2c_start();
    //     i2c_write_byte(0x46); // Địa chỉ I2C của BH1750 với lệnh ghi
    //     i2c_write_byte(0x10); // Lệnh khởi động đo ánh sáng
    //     i2c_stop();

    //     vTaskDelay(180 / portTICK_PERIOD_MS); // Đợi cảm biến đo xong

    //     i2c_start();
    //     i2c_write_byte(0x47); // Địa chỉ I2C của BH1750 với lệnh đọc
    //     uint8_t msb = i2c_read_byte(1); // Đọc byte cao
    //     uint8_t lsb = i2c_read_byte(0); // Đọc byte thấp
    //     i2c_stop();

    //     lux = (msb << 8) | lsb;
    //     printf("ÁNH SÁNG:         %d lx\n\n", lux);
        
    //     vTaskDelay(pdMS_TO_TICKS(1000));
 
    //      vTaskDelay(pdMS_TO_TICKS(10)); // Nhường CPU 10ms
    //     //CAMBIEN
    //     int adc_reading_2 = 0;
    //     for (int i = 0; i < NO_OF_SAMPLES; i++)
    //     {
    //         if (unit == ADC_UNIT_1)
    //         {
    //             adc_reading_2 += adc1_get_raw((adc1_channel_t)channel);
    //         }
    //         else
    //         {
    //             int raw;
    //             adc2_get_raw((adc2_channel_t)channel, ADC_WIDTH_BIT_12, &raw);
    //             adc_reading_2 += raw;
    //         }
    //     }
    //     adc_reading_2 /= NO_OF_SAMPLES;
    //     int voltage = esp_adc_cal_raw_to_voltage(adc_reading_2, adc_chars);
    //     printf("Raw: %d\tVoltage: %d mV\n", adc_reading_2, voltage);

    //      vTaskDelay(pdMS_TO_TICKS(10)); // Nhường CPU 10ms
    //     // Tính toán độ ẩm
    //     int value_1 = (100 - ((float)(adc_reading_2 - 1400) / (4095 - 1400)) * 100);
    //     value = (value < 0) ? 0 : ((value > 100) ? 100 : value);

    //     printf("Độ ẩm: %d %%\n", value_1);
         
         
    //     vTaskDelay(pdMS_TO_TICKS(10)); // Nhường CPU 10ms
    //     // Cập nhật data_moisure_sensor một cách an toàn
    //     if (xSemaphoreTake(data_moisure_sensor_semaphore, pdMS_TO_TICKS(100)) == pdTRUE)
    //     {
    //         data_moisure_sensor = value;
    //         xSemaphoreGive(data_moisure_sensor_semaphore);
    //     }

    //     vTaskDelay(pdMS_TO_TICKS(1000));
    //      vTaskDelay(pdMS_TO_TICKS(10)); // Nhường CPU 10ms
    //     //Loadcell
    //     weight =0;
    //     for (char i = 0; i < 50; i++) 
	//     {
	// 	weight += HX711_read();
	//     }
    //     load = ((weight/500) - zero)/ 37.55 ;
    //     ESP_LOGI(TAG, "******* Loadcell = %f *********\n ", load);
    //     vTaskDelay(pdMS_TO_TICKS(9));
    //      vTaskDelay(pdMS_TO_TICKS(10)); // Nhường CPU 10ms
    //     //Bom 
    //     gpio_set_level(23,0);
    //     vTaskDelay(100/ portTICK_PERIOD_MS);
    //     bomnuoc_1 =1;
    //     gpio_set_level(23,1);
    //     vTaskDelay(100/ portTICK_PERIOD_MS); 
    //      vTaskDelay(pdMS_TO_TICKS(10)); // Nhường CPU 10ms
    //      vTaskDelay(pdMS_TO_TICKS(5000)); // Nhường CPU
    // }


   
}