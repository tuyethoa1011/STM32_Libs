#include <stdio.h>
#include "stdint.h"
#include "string.h"

#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "driver/gpio.h"

//------I2C LCD lib -----
#include "esp_log.h"
#include "driver/i2c.h"
#include "ssd1306.h"

//Chân được cấu hình dựa trên KIT ESP được sử dụng (Current: ESP32-S2 AI Thinker)
#define TXD_PIN (GPIO_NUM_17)
#define RXD_PIN (GPIO_NUM_16)

//----- I2C LCD define -----
#define CONFIG_SSD1306_OPMODE 0x3C

#define I2C_MASTER_SCL_IO 4            /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO 5              /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM 1     /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ 100000        /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */

#define DELAY_TIME_BETWEEN_ITEMS_MS 1000 /*!< delay time between different test items */

#define SSD1306_OLED_ADDR   0x3C  /*!< slave address for OLED SSD1306 */
#define SSD1306_CMD_START CONFIG_SSD1306_OPMODE   /*!< Operation mode */
#define WRITE_BIT I2C_MASTER_WRITE              /*!< I2C master write */
#define READ_BIT I2C_MASTER_READ                /*!< I2C master read */
//----- I2C LCD define -----


static const int RX_BUF_SIZE = 1024;

char Rx_Buffer1[200]; //Buffer for original value
char Rx_Buffer2[100]; //Buffer for CAN value


//----- I2C LCD essential function -----
/**
 * @brief i2c master initialization
 */
static esp_err_t i2c_master_init(void)
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
    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}
//----- I2C LCD essential function -----

void uart_debug_init(void) {
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    // We won't use a buffer for sending data.
    uart_driver_install(UART_NUM_2, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_2, &uart_config);
    uart_set_pin(UART_NUM_2, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}


void uint8StringToHex(const uint8_t* input, size_t length, char* hexString) {
    for (size_t i = 0; i < length; ++i) {
        sprintf(hexString + i * 2, "%02x", input[i]);
    }
}

void concatenate_string(char* s, char* s1)
{
    int i;
 
    int j = strlen(s);
 
    for (i = 0; s1[i] != '\0'; i++) {
        s[i + j] = s1[i];
    }
 
    s[i + j] = '\0';
 
    return;
}

static void rx_task(void *arg)
{
    static const char *RX_TASK_TAG = "RX_TASK";
    esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);
    uint8_t* data = (uint8_t*) malloc(RX_BUF_SIZE+1);
    while (1) {   
        const int rxBytes = uart_read_bytes(UART_NUM_2, data, RX_BUF_SIZE, 500/ portTICK_RATE_MS);
        if (rxBytes > 0) {
            data[rxBytes] = 0;
            ESP_LOGI(RX_TASK_TAG, "Read %d bytes: '%s'", rxBytes, data);  

             // Allocate space for the hex string (twice the length of the original string + 1 for the null terminator)
            char hexRepresentation[8 * 2 + 1];

            memset(Rx_Buffer1,0,sizeof(Rx_Buffer1)); //clear buffer1 before write
            memset(hexRepresentation,0,sizeof(hexRepresentation)); //clear buffer before write
            memset(Rx_Buffer2,0,sizeof(Rx_Buffer2)); //clear buffer before write

            sprintf((char*)Rx_Buffer1,"UART: \n%s\n",data);

            // Convert the uint8_t string to hex
            uint8StringToHex(data, 8, hexRepresentation);

            sprintf((char*)Rx_Buffer2,"CAN: \n%s\n",hexRepresentation);

            concatenate_string(Rx_Buffer1,Rx_Buffer2);
            
            task_ssd1306_display_clear(I2C_MASTER_NUM);      
           
            task_ssd1306_display_text(Rx_Buffer1,I2C_MASTER_NUM);
        }
    }
    free(data);
}


void app_main(void)
{   
    uart_debug_init(); //uart init 
    i2c_master_init();
    ssd1306_init(I2C_MASTER_NUM);
    xTaskCreate(rx_task, "uart_rx_task", 1024*2, NULL, configMAX_PRIORITIES-1, NULL);
}
