/*
 *
 * Written by Olof Astrand <olof.astrand@gmail.com>
 *
 * This example code is in the Public Domain (or CC0 licensed, at your option.)
 * Unless required by applicable law or agreed to in writing, this
 * software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
 * CONDITIONS OF ANY KIND, either express or implied.
 *
*/
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_event_loop.h"
#include "nvs_flash.h"

#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event_loop.h"
#include "esp_log.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"
#include "lwip/dns.h"
#include "esp_sleep.h"

#include "driver/gpio.h"
#include "sdkconfig.h"
#include "driver/i2c.h"
#include "Si7021.h"
#include "BME280.h" 

// Use internal sensor
uint8_t temprature_sens_read();	

bool start_wifi=false;

// Use same inputs as sparkfun

#define I2C_MASTER_SCL_IO    22    /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO    21    /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM   I2C_NUM_0   /*!< I2C port number for master dev */
#define I2C_MASTER_TX_BUF_DISABLE   0   /*!< I2C master do not need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0   /*!< I2C master do not need buffer */
#define I2C_MASTER_FREQ_HZ    100000     /*!< I2C master clock frequency */

#define WRITE_BIT  I2C_MASTER_WRITE /*!< I2C master write */
#define READ_BIT   I2C_MASTER_READ  /*!< I2C master read */
#define ACK_CHECK_EN   0x1     /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS  0x0     /*!< I2C master will not check ack from slave */
#define ACK_VAL    0x0         /*!< I2C ack value */
#define NACK_VAL   0x1         /*!< I2C nack value */

RTC_DATA_ATTR static int boot_count = 0;

// Socket info
#define WEB_SERVER "thingspeak.com"
#define WEB_PORT 80
#define WEB_URL "http://thingspeak.com/"

/* FreeRTOS event group to signal when we are connected & ready to make a request */
static EventGroupHandle_t wifi_event_group;

/* The event group allows multiple bits for each event,
   but we only care about one event - are we connected
   to the AP with an IP? */
const int CONNECTED_BIT = BIT0;

const int POSTED_BIT = BIT1;

static const char *TAG = "weather";




/**
 * @brief test function to show buffer
 */
void show_buf(uint8_t* buf, int len)
{
    int i;
    for (i = 0; i < len; i++) {
        printf("%02x ", buf[i]);
        if (( i + 1 ) % 16 == 0) {
            printf("\n");
        }
    }
    printf("\n");
}


/**
 *
 * _______________________________________________________________________________________
 * | start | slave_addr  +ack | 
 * --------|--------------------------|----------------------|--------------------|------|
 *
 */
esp_err_t i2c_master_check_slave(i2c_port_t i2c_num,uint8_t addr)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( addr << 1 ) , ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}


/**
 * @brief i2c master initialization
 */
void i2c_init()
{
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    i2c_param_config(i2c_master_port, &conf);
    i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}



/**
 * @brief Scan all i2c adresses 
 */
void i2c_scan() {
	int address;
    int ret;
	int foundCount = 0;
	for (address=1; address<127; address++) {
        ret=i2c_master_check_slave(I2C_MASTER_NUM,address);
        if (ret == ESP_OK) {
            printf("Found device addres: %02x\n", address);
            foundCount++;
        }
    }
    printf("Done scanning.. found %d devices\n",foundCount);
}


#define BLINK_GPIO 5

void blink_task(void *pvParameters)
{
    gpio_pad_select_gpio(BLINK_GPIO);
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);


	while (1) {
          gpio_set_level(BLINK_GPIO, 0);
          vTaskDelay(1000 / portTICK_RATE_MS);
          gpio_set_level(BLINK_GPIO, 1);
          vTaskDelay(1000 / portTICK_RATE_MS);
          gpio_set_level(BLINK_GPIO, 0);
          vTaskDelay(1000 / portTICK_RATE_MS);
          gpio_set_level(BLINK_GPIO, 1);
          vTaskDelay(1000 / portTICK_RATE_MS);


        if (start_wifi) {
            xEventGroupWaitBits(wifi_event_group, POSTED_BIT,
                                false, true, portMAX_DELAY);
        }
	      //esp_deep_sleep_pd_config(ESP_PD_DOMAIN_RTC_SLOW_MEM, ESP_PD_OPTION_ON);


		/* wakeup from deep sleep after 6 seconds */
		esp_deep_sleep(1000*1000*6);
	}
}

//#define THINGSPEAK_CHANNEL_KEY "8FKJRMLXT2CPYCVO"
#define THINGSPEAK_CHANNEL_KEY "FETKGOXXT8BVCMVC"

char temp_buff[1024];

static void http_get_task(void *pvParameters)
{
    //const struct addrinfo hints = {
    //    .ai_family = AF_INET,
    //    .ai_socktype = SOCK_STREAM,
    //};
    //struct addrinfo *res;
    //struct in_addr *addr;
    struct sockaddr_in dest; 
    int times_tried=0;

    int s, r;
    char recv_buf[64];

    while(times_tried++<3) {
        /* Wait for the callback to set the CONNECTED_BIT in the
           event group.
        */
        xEventGroupWaitBits(wifi_event_group, CONNECTED_BIT,
                            false, true, portMAX_DELAY);
        ESP_LOGI(TAG, "Connected to AP");

/* DNS failed
        int err = getaddrinfo(WEB_SERVER, "80", &hints, &res);

        if(err != 0 || res == NULL) {
            ESP_LOGE(TAG, "DNS lookup failed err=%d res=%p", err, res);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            continue;
        }
*/
        memset(&dest, 0, sizeof(dest));                          /* zero the struct */
        dest.sin_family = AF_INET;
        inet_pton(AF_INET, "184.106.153.149", &dest.sin_addr.s_addr);  /* set destination IP number */ 
        dest.sin_port = htons(80);                              /* set destination port number */

        //res=&dest;
        /* Code to print the resolved IP.

           Note: inet_ntoa is non-reentrant, look at ipaddr_ntoa_r for "real" code */
        //addr = &((struct sockaddr_in *)res->ai_addr)->sin_addr;
        //ESP_LOGI(TAG, "DNS lookup succeeded. IP=%s", inet_ntoa(*addr));

        s = socket(AF_INET, SOCK_STREAM,  IPPROTO_TCP);
        if(s < 0) {
            ESP_LOGE(TAG, "... Failed to allocate socket.");
            //freeaddrinfo(res);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            continue;
        }
        ESP_LOGI(TAG, "... allocated socket\r\n");

        if(connect(s, (struct sockaddr *)&dest, sizeof(struct sockaddr_in)) != 0) {
            ESP_LOGE(TAG, "... socket connect failed errno=%d", errno);
            close(s);
            //freeaddrinfo(res);
            vTaskDelay(4000 / portTICK_PERIOD_MS);
            continue;
        }

        ESP_LOGI(TAG, "... connected");
        //freeaddrinfo(res);

        if (write(s, temp_buff, strlen(temp_buff)) < 0) {
            ESP_LOGE(TAG, "... socket send failed");
            close(s);
            vTaskDelay(4000 / portTICK_PERIOD_MS);
            continue;
        }
        ESP_LOGI(TAG, "... socket send success");
        printf("%s\n",temp_buff);

        /* Read HTTP response */
        do {
            bzero(recv_buf, sizeof(recv_buf));
            r = read(s, recv_buf, sizeof(recv_buf)-1);
            for(int i = 0; i < r; i++) {
                putchar(recv_buf[i]);
            }
        } while(r > 0);

        ESP_LOGI(TAG, "... done reading from socket. Last read return=%d errno=%d\r\n", r, errno);
        close(s);
        for(int countdown = 10; countdown >= 0; countdown--) {
            ESP_LOGI(TAG, "%d... ", countdown);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
        ESP_LOGI(TAG, "Starting again!");
        xEventGroupSetBits(wifi_event_group, POSTED_BIT);
    }
    ESP_LOGI(TAG, "Giving up!");
    xEventGroupSetBits(wifi_event_group, POSTED_BIT);

}

typedef struct tempHumidityParameters {
    int temp;
    int humidity;
} tempHumidityParameters;

void post_data(tempHumidityParameters *first,int internal_temp,tempHumidityParameters *second,int pressure)
{
    int itemp=first->temp;
    int humid=first->humidity;

    sprintf(temp_buff,"GET /update?api_key=%s&field1=%d&field2=%d&field3=%d&field4=%d&field5=%d&field6=%d HTTP/1.0\n\n",THINGSPEAK_CHANNEL_KEY,itemp,humid,internal_temp,second->temp,second->humidity,pressure);

    xTaskCreate(&http_get_task, "http_get_task", 2*4096, NULL, 5, NULL);

}

static esp_err_t event_handler(void *ctx, system_event_t *event)
{
    switch(event->event_id) {
    case SYSTEM_EVENT_STA_START:
        esp_wifi_connect();
        break;
    case SYSTEM_EVENT_STA_GOT_IP:
        xEventGroupSetBits(wifi_event_group, CONNECTED_BIT);
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
        /* This is a workaround as ESP32 WiFi libs don't currently
           auto-reassociate. */
        esp_wifi_connect();
        xEventGroupClearBits(wifi_event_group, CONNECTED_BIT);
        break;
    default:
        break;
    }
    return ESP_OK;
}



static void initialise_wifi(void)
{
    tcpip_adapter_init();
    wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK( esp_event_loop_init(event_handler, NULL) );
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = "pappas",
            .password = "hashtag1bestammer",
        },
    };
    ESP_LOGI(TAG, "Setting WiFi configuration SSID %s...", wifi_config.sta.ssid);
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK( esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
    ESP_ERROR_CHECK( esp_wifi_start() );
}

#if 0
static void bme280_task(void *pvParameters) {
    float rh;
    float temp;
    float pressure;
    int times=0;
    i2c_bme280_begin();

    while (times++<10) {
        printf("chip id %x\n",i2c_bme280_read_register(BME280_CHIP_ID_REG));

        temp=i2c_bme280_read_temp();
        pressure=i2c_bme280_read_pressure();
        rh=i2c_bme280_read_rh();

        printf("RH %f Temp %f , pressure %f\n",rh,temp,pressure);

    }
    i2c_bme280_end();

}
#endif

void app_main()
{
	printf("Starting ... \r\n");
    //boot_count;
    ESP_LOGI(TAG, "Boot count: %d", boot_count);

    i2c_init();
    //i2c_scan();
    

#if 1
    printf("Displaying all regs\n");
	uint8_t memCounter = 0x80;
	uint8_t tempReadData;
	for(int rowi = 8; rowi < 16; rowi++ )
	{
		printf("0x%02x ",rowi);
		for(int coli = 0; coli < 16; coli++ )
		{
			tempReadData = i2c_bme280_read_register(memCounter);
			printf("%02x",(tempReadData >> 4) & 0x0F);//Print first hex nibble
			printf("%02x",tempReadData & 0x0F);//Print second hex nibble
			printf(" ");
			memCounter++;
		}
		printf("\n");
    }
#endif


    
    // Every minute start wifi
    if (boot_count%10==0) {
        nvs_flash_init();
        initialise_wifi();
        start_wifi=true;
        boot_count++;

    } else {
        // We dont start wifi, our job is done.. go to sleep asap.
        boot_count++;
        start_wifi=false;
        
        xTaskCreatePinnedToCore(&blink_task, "blink_task", 4096*2, NULL, 5,
            NULL, 0);

        vTaskDelay(10000 / portTICK_PERIOD_MS);

    }

    // read sensor early
    uint8_t internal=temprature_sens_read();	




    int times=0;
    tempHumidityParameters i7021;
    tempHumidityParameters bme280;

    //float rh,rh280;
    //float temp,temp280;
    float pressure;

    i7021.humidity=i2c_7021_read_rh();

    i7021.temp=(int)i2c_7021_read_temp();

    printf("RH %d Temp %d , internal %d\n",i7021.humidity,i7021.temp,internal);

    printf("chip id %x\n",i2c_bme280_read_register(BME280_CHIP_ID_REG));


    uint8_t status;
    int statustimes=0;

    printf("status %x\n",i2c_bme280_read_register(0xF3));
    i2c_bme280_begin();
    printf("status after begin %x\n",i2c_bme280_read_register(0xF3));

    while(times++ < 2 ) 
    {
        printf("power mode %x\n",i2c_bme280_get_power_mode());
        printf("status %x\n",i2c_bme280_read_register(0xF3));
        
        i2c_bme280_force_readings();
        printf("status %x\n",i2c_bme280_read_register(0xF3));

        status=i2c_bme280_read_register(0xF3);
        while((status & (3<<1)) == (3<<1) && (statustimes++ <10))
        {
            status=i2c_bme280_read_register(0xF3);
            printf(".");
        }
        bme280.temp=(int)i2c_bme280_read_temp();
        pressure=i2c_bme280_read_pressure();
        bme280.humidity=(int)i2c_bme280_read_rh();
        printf("RH %d Temp %d , pressure %f\n",bme280.humidity,bme280.temp,pressure/100.0);
        printf("power mode %x\n",i2c_bme280_get_power_mode());
    }
    // Not necessary
    //i2c_bme280_end();

    if (start_wifi) {
        post_data(&i7021,internal,&bme280,(int)pressure/100.0);
        //xEventGroupSetBits(wifi_event_group, POSTED_BIT);
    } 

    xTaskCreatePinnedToCore(&blink_task, "blink_task", 4096*2, NULL, 5,
				NULL, 0);

}
