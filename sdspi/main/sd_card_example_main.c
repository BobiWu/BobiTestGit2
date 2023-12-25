/* SD card and FAT filesystem example.
   This example uses SPI peripheral to communicate with SD card.

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
//-------adc-----------------
#include "driver/adc.h"
#define ADC1_TEST_CHANNEL (3)
//-------adc-----------------

//-------random--------------
#include "esp_random.h"
//-------random--------------

//-------wdt-----------------
#include "esp_task_wdt.h"
#define TWDT_TIMEOUT_S          3
#define TASK_RESET_PERIOD_S     2
/*
* 检查任务看门狗输出的宏，并在返回错误代码的时候触发中止
*/
#define CHECK_ERROR_CODE(returned, expected) ({                        \
             if(returned != expected){                                  \
                 printf("TWDT ERROR\n");                                \
                 abort();                                               \
             }                                                          \
 })
//static TaskHandle_t task_handles[portNUM_PROCESSORS];
//-------wdt-----------------

//-------Timer---------------
#include "soc/soc.h"
#include "driver/timer.h"
#include "esp_clk_tree.h"

#define TIMER_RESOLUTION_HZ   1000000 // 1MHz resolution
#define TIMER_ALARM_PERIOD_S  0.0001//1//0.5     // Alarm period 0.5s   

//static int time_count = 0;
//-------Timer---------------

//-------test ir detection--
//static	int irBitSendingCounters = 0;	//32 //29 //28
  
#define	Imm_4LedsDelayTime	20500
static int  delay1000msAndTurnOffNorthRx4Led = 0;
static int  delay1000msAndTurnOffEastRx4Led = 0;
static int  delay1000msAndTurnOffSouthRx4Led = 0;
static int  delay1000msAndTurnOffWestRx4Led = 0;

static uint16_t  NorthRxPreviousData = 0;
static uint16_t  EastRxPreviousData = 0;
static uint16_t  SouthRxPreviousData = 0;
static uint16_t  WestRxPreviousData = 0;

static	unsigned int 	newNorthRxData=0;
static	unsigned int 	newEastRxData=0;	    
static	unsigned int 	newSouthRxData=0;	    
static	unsigned int 	newWestRxData=0;
//-------test ir detection--

//-------SD card-------------
#include <string.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
//-------SD card-------------

//#include "sd_card_example_main.h"
#include "IrRxTx_mc3416_aw9523b_i2c.h"

//-------WS2812b-------------
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/rmt_tx.h"
#include "led_strip_encoder.h"
//-------WS2812b-------------

//-------i2s speaker---------
#include <stdio.h>
#include "driver/i2s.h"
#include "esp_system.h"
//#include "esp_spiffs.h"

void SetAndWaitPlaySoundFile(const char * SoundName);

//static const char *playSoundName;  
static char *playSoundName;  
static bool needToPlayNewSound = false;
//static float fvolume = 0.5;
#define MAX_VOLUME    4
#define MIN_VOLUME    0
static uint8_t volumeSet = MAX_VOLUME;//0,1,2,3,4   "0 is mute"   "5 is max"
//-------i2s speaker---------

//-------SD card-------------
#define EXAMPLE_MAX_CHAR_SIZE    64
static const char *TAG = "example";
#define MOUNT_POINT "/sdcard"

// Pin assignments can be set in menuconfig, see "SD SPI Example Configuration" menu.
// You can also change the pin assignments here by changing the following 4 lines.
#define PIN_NUM_MISO  19
#define PIN_NUM_MOSI  23
#define PIN_NUM_CLK   18
#define PIN_NUM_CS    4

//struct stat st;
//-------SD card-------------


//-------i2c-----------------//ing //定义 ram 和 关键字
static unsigned int 	cubeCurrentPosition = 0;
static unsigned int 	cubeChangedPositionHp = 0;

static unsigned int		cubeShakeHp = 0;

static unsigned int 	port1VolUpSwOnHp = 0;
static unsigned int 	port1VolDwSwOnHp = 0;
static unsigned int 	port1btn2SwOnHp = 0;
static unsigned int 	port1btn3SwOnHp = 0;
static unsigned int 	port1btn4SwOnHp = 0;
static unsigned int 	port1VusbSwOnHp = 0;
static unsigned int 	port1VusbSwOffHp = 0;
//-------i2c-----------------

//-------generic_gpio--------
#include <stdlib.h>
#include <inttypes.h>
#include "freertos/queue.h"

#define GPIO_OUTPUT_IO_0    12  //CONFIG_GPIO_OUTPUT_0
#define GPIO_OUTPUT_PIN_SEL  ((1ULL<<GPIO_OUTPUT_IO_0))//((1ULL<<GPIO_OUTPUT_IO_0) | (1ULL<<GPIO_OUTPUT_IO_1))
/*
 * Let's say, GPIO_OUTPUT_IO_0=18, GPIO_OUTPUT_IO_1=19
 * In binary representation,
 * 1ULL<<GPIO_OUTPUT_IO_0 is equal to 0000000000000000000001000000000000000000 and
 * 1ULL<<GPIO_OUTPUT_IO_1 is equal to 0000000000000000000010000000000000000000
 * GPIO_OUTPUT_PIN_SEL                0000000000000000000011000000000000000000
 * */
#define GPIO_INPUT_IO_0     34   //BTN

#define GPIO_INPUT_IO_1     32   //IR_RX_NORTH 
#define GPIO_INPUT_IO_2     33   //IR_RX_EAST 
#define GPIO_INPUT_IO_3     25   //IR_RX_SOUTH
#define GPIO_INPUT_IO_4     13   //IR_RX_WEST 

#define GPIO_INPUT_IO_5     2    //BTN5
#define GPIO_INPUT_PIN_SEL  ((1ULL<<GPIO_INPUT_IO_0) | (1ULL<<GPIO_INPUT_IO_1) | (1ULL<<GPIO_INPUT_IO_2) | (1ULL<<GPIO_INPUT_IO_3) | (1ULL<<GPIO_INPUT_IO_4) | (1ULL<<GPIO_INPUT_IO_5))

#define btnInput            GPIO_INPUT_IO_0
#define btn5Input           GPIO_INPUT_IO_5

/*
 * Let's say, GPIO_INPUT_IO_0=4, GPIO_INPUT_IO_1=5
 * In binary representation,
 * 1ULL<<GPIO_INPUT_IO_0 is equal to 0000000000000000000000000000000000010000 and
 * 1ULL<<GPIO_INPUT_IO_1 is equal to 0000000000000000000000000000000000100000
 * GPIO_INPUT_PIN_SEL                0000000000000000000000000000000000110000
 * */
#define ESP_INTR_FLAG_DEFAULT 0

static QueueHandle_t gpio_evt_queue = NULL;

static int  btnOnHappen = 0;
static int  btn5OnHappen = 0;
//-------generic_gpio--------


//-------WS2812b-------------
#define RMT_LED_STRIP_RESOLUTION_HZ 10000000 // 10MHz resolution, 1 tick = 0.1us (led strip needs a high resolution)
#define RMT_LED_STRIP_GPIO_NUM      15//0

#define EXAMPLE_LED_NUMBERS         50//49//24
#define EXAMPLE_CHASE_SPEED_MS      10//20

//static const char *TAG = "example";

static uint8_t led_strip_pixels[EXAMPLE_LED_NUMBERS * 3] = {
0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,
0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,
0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,
0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,
0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,
0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,
0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,
0x00,0x00,0x00
    };

static uint8_t led_strip_pixels_ShowRxData[EXAMPLE_LED_NUMBERS * 3] = {
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00
    };

char wavBuffer[1024];   // 接收缓冲区
static esp_err_t s_example_read_ws2812b_bin_file(const char *path);

rmt_channel_handle_t led_chan = NULL;
rmt_encoder_handle_t led_encoder = NULL;
rmt_transmit_config_t tx_config = {
        .loop_count = 0, // no transfer loop
    };
//-------WS2812b-------------

//-------uart----------------
#include "driver/uart.h"
#include "string.h"
#include "driver/gpio.h"

static const int RX_BUF_SIZE = 1024;

#define TXD_PIN (GPIO_NUM_16)   //GPIO_NUM_4
#define RXD_PIN (GPIO_NUM_17)   //GPIO_NUM_5

static int cardOn_debounce = 0;
static int cardOff_Hold = 0;    
static int cardOnHappen = 0;

static int cardOff_debounce = 0;
static int cardOn_Hold = 0;        
static int cardOffHappen = 0;   

static int uid_card_number = 0;

//const uart_config_t uart_config = {
uart_config_t uart_config = {  
    .baud_rate = 9600, //115200
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    .source_clk = UART_SCLK_DEFAULT,
};
void init(void) {
    // We won't use a buffer for sending data.
    uart_driver_install(UART_NUM_1, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

int sendData(const char* logName, uint8_t arry[], int len) //char  uint8_t arry[]
{
    //const int len = sizeof(arry);
    const int txBytes = uart_write_bytes(UART_NUM_1, arry, len);
    //concel_debig ESP_LOGI(logName, "Wrote %d bytes", txBytes);
    return txBytes;
}

static void tx_task(void *arg)
{
    int len = 0;
    static const char *TX_TASK_TAG = "TX_TASK";
    esp_log_level_set(TX_TASK_TAG, ESP_LOG_INFO);


    uint8_t arry0[] = {
                        0x1F,0X7A  //0XEB=9600  0X7A=115200
                      };
    len = sizeof(arry0)/sizeof(arry0[0]);
    sendData(TX_TASK_TAG, arry0, len);
    vTaskDelay(10 / portTICK_PERIOD_MS);     //100
    uart_config.baud_rate = 115200;
    uart_param_config(UART_NUM_1, &uart_config);
    
    
    uint8_t arry1[] = {
                        0x88,0x08,0x00,0x12,0x00,0x92,0x13,0x00,0x24,0x26,
                        0x26,0x68,0x2A,0x80,0x2B,0xA9,0x2C,0x03,0x2D,0xE8,
                        0x15,0x40,0x11,0x3D,0x01,0x00,0x14,0x83
                      };
    len = sizeof(arry1)/sizeof(arry1[0]);
    sendData(TX_TASK_TAG, arry1, len);
    vTaskDelay(50 / portTICK_PERIOD_MS);    //100
    
    
    uint8_t arry2[] = {
                        0x26,0x68,0x88,0x08,0x00,0x0D,0x07,0x94,0x14,0x83,
                        0x84,0x04,0x14,0x01,0x00,0x8A,0x0A,0x80,0x09,0x26,
                        0x01,0x0C,0x8D,0x0D,0x87,0x84,0x8D,0x0D,0x07,0x86,
                        0x86,0x8A,0x8C,0x89,0x8C,0x0C,0x90,0x01,0x00
                      };
    len = sizeof(arry2)/sizeof(arry2[0]);
    sendData(TX_TASK_TAG, arry2, len);
    vTaskDelay(50 / portTICK_PERIOD_MS);    //100
    

    while (1) { 
                /*               
                uint8_t arry3[] = {
									0x26,0x68,0x88,0x08,0x00,0x0D,0x07,0x94,0x14,0x83,
									0x84,0x04,0x14,0x01,0x00,0x8A,0x0A,0x80,0x09,0x26,
									0x01,0x0C,0x8D,0x0D,0x87,0x84,0x8D,0x0D,0x07,0x86,
									0x86,0x8A,0x8C,0x89,0x8C,0x0C,0x90,0x01,0x00,0x88,
									0x08,0x00,0x0D,0x00,0x8E,0x0E,0x20,0x84,0x04,0x64,
									0x01,0x00,0x8A,0x0A,0x80,0x09,0x93,0x09,0x20,0x01,
									0x0C,0x8D,0x0D,0x80,0x84,0x8D,0x0D,0x00,0x86,0x86,
									0x8A,0x8C,0x89,0x89,0x89,0x89
                                  };
                len = sizeof(arry3)/sizeof(arry3[0]);
                sendData(TX_TASK_TAG, arry3, len);
                vTaskDelay(1000 / portTICK_PERIOD_MS);  //100 //500
                */

                
                uint8_t arry3[] = {
                                    0x26,0x68,0x88,0x08,0x00,0x0D,0x07,0x94,0x14,0x83,
                                    0x84,0x04,0x14,0x01,0x00,0x8A,0x0A,0x80,0x09,0x26,
                                    0x01,0x0C,0x8D,0x0D,0x87,0x84,0x8D,0x0D,0x07,0x86,
                                    0x86,0x8A,0x8C,0x89,0x8C,0x0C,0x90,0x01,0x00,0x26,
                                    0x48,0x88,0x08,0x00,0x0D,0x07,0x94,0x14,0x83,0x84,
                                    0x04,0x45,0x01,0x00,0x8A,0x0A,0x80,0x09,0x26,0x01,
                                    0x0C,0x8D,0x0D,0x87,0x84,0x84,0x84,0x84,0x84,0x84,
                                    0x84,0x84,0x84,0x84,0x84,0x84,0x84,0x84,0x8D,0x0D,
                                    0x07,0x86,0x86,0x8A,0x8C,0x89,0x8C,0x0C,0x90,0x01,
                                    0x00,0x26,0x58,0x88,0x08,0x00,0x0D,0x07,0x94,0x14,
                                    0x83,0x84,0x04,0x45,0x01,0x00,0x8A,0x0A,0x80,0x09,
                                    0x26,0x01,0x0C,0x8D,0x0D,0x87,0x84,0x84,0x84,0x84,
                                    0x84,0x84,0x84,0x84,0x84,0x84,0x84,0x84,0x84,0x84,
                                    0x8D,0x0D,0x07,0x86,0x86,0x8A,0x8C,0x89,0x8C,0x0C,
                                    0x90,0x01,0x00,0x88,0x08,0x00,0x0D,0x00,0x8E,0x0E,
                                    0x20,0x84,0x04,0x64,0x01,0x00,0x8A,0x0A,0x80,0x09,
                                    0x93,0x09,0x20,0x01,0x0C,0x8D,0x0D,0x80,0x84,0x8D,
                                    0x0D,0x00,0x86,0x86,0x8A,0x8C,0x89,0x89,0x89,0x89,
                                    0x89,0x8C,0x0C,0x90,0x01,0x00,0x8E,0x0E,0xA0,0x85,
                                    0x05,0x00,0x01,0x00,0x8A,0x0A,0x80,0x09,0x93,0x09,
                                    0x70,0x09,0x59,0x09,0x88,0x09,0x04,0x09,0x99,0x09,
                                    0x4C,0x01,0x03,0x85,0xA2,0xA1,0x88,0x08,0x00,0x84,
                                    0x04,0x64,0x01,0x00,0x8A,0x0A,0x80,0x09,0x93,0x09,
                                    0x70,0x09,0x59,0x09,0x88,0x09,0x04,0x09,0x99,0x09,
                                    0x4C,0x09,0x60,0x09,0x10,0x01,0x0C,0x8D,0x0D,0x80,
                                    0x84,0x84,0x8D,0x0D,0x00,0x86,0x86,0x8A,0x8C,0x89,
                                    0x89,0x89,0x8C,0x0C,0x90,0x01,0x00
                                  };
                len = sizeof(arry3)/sizeof(arry3[0]);
                sendData(TX_TASK_TAG, arry3, len);
                vTaskDelay(200 / portTICK_PERIOD_MS); //200 //500
                
    }
}

static void rx_task(void *arg)
{
    //const char *file_RgbPicture;	
	//int ret; 
    //struct stat st;
    int  UidCardNumber = -2;

    static const char *RX_TASK_TAG = "RX_TASK";
    esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);
    uint8_t* data = (uint8_t*) malloc(RX_BUF_SIZE+1);
    while (1) {
        const int rxBytes = uart_read_bytes(UART_NUM_1, data, RX_BUF_SIZE, 30 / portTICK_PERIOD_MS);  //1000
        if (rxBytes > 0) {
            data[rxBytes] = 0;
            //concel_debig ESP_LOGI(RX_TASK_TAG, "Read %d bytes: '%s'", rxBytes, data);
            //concel_debig ESP_LOG_BUFFER_HEXDUMP(RX_TASK_TAG, data, rxBytes, ESP_LOG_INFO);

            if (rxBytes == 185)//185 is RFID uid feedback
            {
                if(data[126] == 0xA0)
                {
                    if(data[127] == 0xA1)
                    {                   
                        if(data[128] == 0xA2)
                        {
                            if(
                            		(data[129] == 0x00)||	//uid: 0xA0,   0xA1,   0xA2,   0x00
                            		(data[129] == 0x01)||	//uid: 0xA0,   0xA1,   0xA2,   0x01
                            		(data[129] == 0x02)||	//uid: 0xA0,   0xA1,   0xA2,   0x02
                            		(data[129] == 0x03)||	//uid: 0xA0,   0xA1,   0xA2,   0x03
                            		(data[129] == 0x04)		//uid: 0xA0,   0xA1,   0xA2,   0x04
                            	)
                            {
        						cardOff_debounce = 0;
            					cardOn_debounce++;
            					if(cardOn_debounce >= 1) 
            					{//50
                					if(!cardOn_Hold) 
                					{										
                						cardOff_Hold = 0;
                    					cardOn_Hold = 1;
                    					uid_card_number = data[129];
                    					cardOnHappen = 1;
                					}
            					}                                
                            }
                            else
                            {
                                UidCardNumber = -1;
                            }
                        }
                        else
                        {
                            UidCardNumber = -1;
                        } 
                    }
                    else
                    {
                        UidCardNumber = -1;
                    }
                }
                else
                {
                    UidCardNumber = -1;

        			cardOn_debounce = 0;
            		cardOff_debounce++;
            	    if(cardOff_debounce >= 3) 
            		{//50
                	    if(!cardOff_Hold) 
                		{
                			if(cardOn_Hold == 1)
                			{
                				cardOffHappen = 1;
                			}	                				                    		                    		
                    		cardOff_Hold = 1;
                    		cardOn_Hold = 0;
                		}
            		}
                }
                if(UidCardNumber == -1)
                {
                    UidCardNumber = -2;   
                }
            }
            
        }
    }
    free(data);
}
//-------uart----------------

//-------generic_gpio--------
static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

void button_task(void *arg)
{
    int button_state = 0;
    int button_debounce = 0;
    int button_pressed = 0;
    while (1) {
        button_state = gpio_get_level(btnInput);//gpio_get_level(BUTTON_PIN);
        if (button_state == 1) {
            button_debounce++;
            if (button_debounce >= 3) { //10
                if (!button_pressed) {
					ESP_LOGI(TAG,"Button Pressed!\n");
                    button_pressed = 1;
                    btnOnHappen = 1;
                }
            }
        } else {
            button_debounce = 0;
            button_pressed = 0;
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void button5_task(void *arg)
{
    int button5_state = 0;
    int button5_debounce = 0;
    int button5_pressed = 0;
    while (1) {
        button5_state = gpio_get_level(btn5Input);//gpio_get_level(button5_PIN);
        if (button5_state == 1) {
            button5_debounce++;
            if (button5_debounce >= 3) { //10
                if (!button5_pressed) {
					ESP_LOGI(TAG,"button5 Pressed!\n");
                    button5_pressed = 1;
                    btn5OnHappen = 1;
                }
            }
        } else {
            button5_debounce = 0;
            button5_pressed = 0;
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}
//-------generic_gpio--------


//-------test ir detection--
//-------
void clear_otherRxLed_if_same_tx(uint16_t rx_data, uint16_t north_show, uint16_t east_show, uint16_t south_show, uint16_t west_show)	 
{
			if(NorthRxPreviousData == (rx_data | north_show))
			{
				led_strip_pixels_ShowRxData[north_show + 0] = 0x00;
				led_strip_pixels_ShowRxData[north_show + 1] = 0x00;    
				led_strip_pixels_ShowRxData[north_show + 2] = 0x00; 				
			}	
			if(EastRxPreviousData == (rx_data | east_show))
			{
				led_strip_pixels_ShowRxData[east_show + 0] = 0x00;
				led_strip_pixels_ShowRxData[east_show + 1] = 0x00;    
				led_strip_pixels_ShowRxData[east_show + 2] = 0x00; 								
			}
			if(SouthRxPreviousData == (rx_data | south_show))
			{
				led_strip_pixels_ShowRxData[south_show + 0] = 0x00;
				led_strip_pixels_ShowRxData[south_show + 1] = 0x00;    
				led_strip_pixels_ShowRxData[south_show + 2] = 0x00; 								
			}
			if(WestRxPreviousData == (rx_data | west_show))
			{
				led_strip_pixels_ShowRxData[west_show + 0] = 0x00;
				led_strip_pixels_ShowRxData[west_show + 1] = 0x00;    
				led_strip_pixels_ShowRxData[west_show + 2] = 0x00; 								
			}			
}				        
//-------
void clear_north_4_led(void)
{
	led_strip_pixels_ShowRxData[1 * 3 + 0] = 0x00;
	led_strip_pixels_ShowRxData[1 * 3 + 1] = 0x00;    
	led_strip_pixels_ShowRxData[1 * 3 + 2] = 0x00; 
	
	led_strip_pixels_ShowRxData[2 * 3 + 0] = 0x00;
	led_strip_pixels_ShowRxData[2 * 3 + 1] = 0x00;    
	led_strip_pixels_ShowRxData[2 * 3 + 2] = 0x00; 
	
	led_strip_pixels_ShowRxData[3 * 3 + 0] = 0x00;
	led_strip_pixels_ShowRxData[3 * 3 + 1] = 0x00;    
	led_strip_pixels_ShowRxData[3 * 3 + 2] = 0x00; 			
	
	led_strip_pixels_ShowRxData[4 * 3 + 0] = 0x00;
	led_strip_pixels_ShowRxData[4 * 3 + 1] = 0x00;    
	led_strip_pixels_ShowRxData[4 * 3 + 2] = 0x00; 			
}
void turn_off_north_4_led_after_delay(void)
{
	if(delay1000msAndTurnOffNorthRx4Led == 1)
	{
		delay1000msAndTurnOffNorthRx4Led = 0;
		clear_north_4_led();
        ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels_ShowRxData, sizeof(led_strip_pixels_ShowRxData), &tx_config));
        ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));		
		NorthRxPreviousData = 0;
	}	
}
void show_north_rec_data(void)
{
	int refreshLeds = 0;
	switch (newNorthRxData){
	case 0xC8://(R)		//0xC8	NorthRx Main_Cube's NorthTx
			clear_north_4_led();	
			clear_otherRxLed_if_same_tx(0xC800, 1 * 3, 13 * 3, 47 * 3, 35 * 3);						
			led_strip_pixels_ShowRxData[1 * 3 + 0] = 0x08;
			led_strip_pixels_ShowRxData[1 * 3 + 1] = 0x00;    
			led_strip_pixels_ShowRxData[1 * 3 + 2] = 0x00; 
			delay1000msAndTurnOffNorthRx4Led = Imm_4LedsDelayTime;      
			refreshLeds = 1;	
			NorthRxPreviousData = 0xC800 | 1 * 3;					
	        break;
	case 0xA8://(G)		//0xA8	NorthRx Sub_Cube1's NorthTx
			clear_north_4_led();		
			clear_otherRxLed_if_same_tx(0xA800, 1 * 3, 13 * 3, 47 * 3, 35 * 3);									
			led_strip_pixels_ShowRxData[1 * 3 + 0] = 0x00;
			led_strip_pixels_ShowRxData[1 * 3 + 1] = 0x08;    
			led_strip_pixels_ShowRxData[1 * 3 + 2] = 0x00;
			delay1000msAndTurnOffNorthRx4Led = Imm_4LedsDelayTime;      
			refreshLeds = 1;    						     
			NorthRxPreviousData = 0xA800 | 1 * 3;			        	
	        break;
	case 0x98://(B)		//0x98	NorthRx Sub_Cube2's NorthTx
			clear_north_4_led();		
			clear_otherRxLed_if_same_tx(0x9800, 1 * 3, 13 * 3, 47 * 3, 35 * 3);									
			led_strip_pixels_ShowRxData[1 * 3 + 0] = 0x00;
			led_strip_pixels_ShowRxData[1 * 3 + 1] = 0x00;    
			led_strip_pixels_ShowRxData[1 * 3 + 2] = 0x08;
			delay1000msAndTurnOffNorthRx4Led = Imm_4LedsDelayTime;      
			refreshLeds = 1;    						     
			NorthRxPreviousData = 0x9800 | 1 * 3;			        	
	        break;
	case 0x68://(RG)	//0x68	NorthRx Sub_Cube3's NorthTx
			clear_north_4_led();		
			clear_otherRxLed_if_same_tx(0x6800, 1 * 3, 13 * 3, 47 * 3, 35 * 3);									
			led_strip_pixels_ShowRxData[1 * 3 + 0] = 0x08;
			led_strip_pixels_ShowRxData[1 * 3 + 1] = 0x08;    
			led_strip_pixels_ShowRxData[1 * 3 + 2] = 0x00; 
			delay1000msAndTurnOffNorthRx4Led = Imm_4LedsDelayTime;      
			refreshLeds = 1;    						     
			NorthRxPreviousData = 0x6800 | 1 * 3;			       	
	        break;
	case 0x58://(RGB)	//0x58	NorthRx Sub_Cube4's NorthTx
			clear_north_4_led();		
			clear_otherRxLed_if_same_tx(0x5800, 1 * 3, 13 * 3, 47 * 3, 35 * 3);									
			led_strip_pixels_ShowRxData[1 * 3 + 0] = 0x08;
			led_strip_pixels_ShowRxData[1 * 3 + 1] = 0x08;    
			led_strip_pixels_ShowRxData[1 * 3 + 2] = 0x08;
			delay1000msAndTurnOffNorthRx4Led = Imm_4LedsDelayTime;      
			refreshLeds = 1;    						     
			NorthRxPreviousData = 0x5800 | 1 * 3;			        	
	        break;
	
	case 0xC1://(R)		//0xC1	NorthRx Main_Cube's EasyTx  
			clear_north_4_led();		
			clear_otherRxLed_if_same_tx(0xC100, 2 * 3, 20 * 3, 46 * 3, 28 * 3);			
			led_strip_pixels_ShowRxData[2 * 3 + 0] = 0x08;
			led_strip_pixels_ShowRxData[2 * 3 + 1] = 0x00;
			led_strip_pixels_ShowRxData[2 * 3 + 2] = 0x00;   
			delay1000msAndTurnOffNorthRx4Led = Imm_4LedsDelayTime;      
			refreshLeds = 1;    						     
			NorthRxPreviousData = 0xC100 | 2 * 3;			    	
	        break;                                
	case 0xA1://(G)		//0xA1	NorthRx Sub_Cube1's EasyTx         
			clear_north_4_led();		                  
			clear_otherRxLed_if_same_tx(0xA100, 2 * 3, 20 * 3, 46 * 3, 28 * 3);						
			led_strip_pixels_ShowRxData[2 * 3 + 0] = 0x00;
			led_strip_pixels_ShowRxData[2 * 3 + 1] = 0x08;
			led_strip_pixels_ShowRxData[2 * 3 + 2] = 0x00;
			delay1000msAndTurnOffNorthRx4Led = Imm_4LedsDelayTime;      
			refreshLeds = 1;    						     
			NorthRxPreviousData = 0xA100 | 2 * 3;			       	
	        break;                                
	case 0x91://(B)		//0x91	NorthRx Sub_Cube2's EasyTx         
			clear_north_4_led();		                  
			clear_otherRxLed_if_same_tx(0x9100, 2 * 3, 20 * 3, 46 * 3, 28 * 3);						
			led_strip_pixels_ShowRxData[2 * 3 + 0] = 0x00;
			led_strip_pixels_ShowRxData[2 * 3 + 1] = 0x00;
			led_strip_pixels_ShowRxData[2 * 3 + 2] = 0x08;   
			delay1000msAndTurnOffNorthRx4Led = Imm_4LedsDelayTime;      
			refreshLeds = 1;    						     
			NorthRxPreviousData = 0x9100 | 2 * 3;			    	
	        break;                                
	case 0x61://(RG) 	//0x61	NorthRx Sub_Cube3's EasyTx       
			clear_north_4_led();		                  
			clear_otherRxLed_if_same_tx(0x6100, 2 * 3, 20 * 3, 46 * 3, 28 * 3);						
			led_strip_pixels_ShowRxData[2 * 3 + 0] = 0x08;
			led_strip_pixels_ShowRxData[2 * 3 + 1] = 0x08;
			led_strip_pixels_ShowRxData[2 * 3 + 2] = 0x00;
			delay1000msAndTurnOffNorthRx4Led = Imm_4LedsDelayTime;      
			refreshLeds = 1;    						     
			NorthRxPreviousData = 0x6100 | 2 * 3;			       	
	        break;                                
	case 0x51://(RGB)	//0x51	NorthRx Sub_Cube4's EasyTx		       
			clear_north_4_led();		                  
			clear_otherRxLed_if_same_tx(0x5100, 2 * 3, 20 * 3, 46 * 3, 28 * 3);						
			led_strip_pixels_ShowRxData[2 * 3 + 0] = 0x08;
			led_strip_pixels_ShowRxData[2 * 3 + 1] = 0x08;
			led_strip_pixels_ShowRxData[2 * 3 + 2] = 0x08;  
			delay1000msAndTurnOffNorthRx4Led = Imm_4LedsDelayTime;      
			refreshLeds = 1;    						     
			NorthRxPreviousData = 0x5100 | 2 * 3;			     	
	        break;                  
	
	case 0xC4://(R) 	//0xC4	NorthRx Main_Cube's SouthTx 
			clear_north_4_led();		
			clear_otherRxLed_if_same_tx(0xC400, 3 * 3, 27 * 3, 45 * 3, 21 * 3);			
			led_strip_pixels_ShowRxData[3 * 3 + 0] = 0x08;
			led_strip_pixels_ShowRxData[3 * 3 + 1] = 0x00;
			led_strip_pixels_ShowRxData[3 * 3 + 2] = 0x00;
			delay1000msAndTurnOffNorthRx4Led = Imm_4LedsDelayTime;      
			refreshLeds = 1;    									
			NorthRxPreviousData = 0xC400 | 3 * 3;				            	
	        break;                                
	case 0xA4://(G) 	//0xA4	NorthRx Sub_Cube1's SouthTx        
			clear_north_4_led();		                  
			clear_otherRxLed_if_same_tx(0xA400, 3 * 3, 27 * 3, 45 * 3, 21 * 3);						
			led_strip_pixels_ShowRxData[3 * 3 + 0] = 0x00;
			led_strip_pixels_ShowRxData[3 * 3 + 1] = 0x08;
			led_strip_pixels_ShowRxData[3 * 3 + 2] = 0x00;	
			delay1000msAndTurnOffNorthRx4Led = Imm_4LedsDelayTime;      
			refreshLeds = 1;    									
			NorthRxPreviousData = 0xA400 | 3 * 3;			            	
	        break;                                
	case 0x94://(B)  	//0x94	NorthRx Sub_Cube2's SouthTx       
			clear_north_4_led();		                  
			clear_otherRxLed_if_same_tx(0x9400, 3 * 3, 27 * 3, 45 * 3, 21 * 3);						
			led_strip_pixels_ShowRxData[3 * 3 + 0] = 0x00;
			led_strip_pixels_ShowRxData[3 * 3 + 1] = 0x00;
			led_strip_pixels_ShowRxData[3 * 3 + 2] = 0x08;
			delay1000msAndTurnOffNorthRx4Led = Imm_4LedsDelayTime;      
			refreshLeds = 1;    									
			NorthRxPreviousData = 0x9400 | 3 * 3;				            	
	        break;                                
	case 0x64://(RG)	//0x64	NorthRx Sub_Cube3's SouthTx        
			clear_north_4_led();		                  
			clear_otherRxLed_if_same_tx(0x6400, 3 * 3, 27 * 3, 45 * 3, 21 * 3);						
			led_strip_pixels_ShowRxData[3 * 3 + 0] = 0x08;
			led_strip_pixels_ShowRxData[3 * 3 + 1] = 0x08;
			led_strip_pixels_ShowRxData[3 * 3 + 2] = 0x00;
			delay1000msAndTurnOffNorthRx4Led = Imm_4LedsDelayTime;      
			refreshLeds = 1;    									
			NorthRxPreviousData = 0x6400 | 3 * 3;				            	
	        break;                                
	case 0x54://(RGB)	//0x54	NorthRx Sub_Cube4's SouthTx       
			clear_north_4_led();		                  
			clear_otherRxLed_if_same_tx(0x5400, 3 * 3, 27 * 3, 45 * 3, 21 * 3);						
			led_strip_pixels_ShowRxData[3 * 3 + 0] = 0x08;
			led_strip_pixels_ShowRxData[3 * 3 + 1] = 0x08;
			led_strip_pixels_ShowRxData[3 * 3 + 2] = 0x08;
			delay1000msAndTurnOffNorthRx4Led = Imm_4LedsDelayTime;      
			refreshLeds = 1;    									
			NorthRxPreviousData = 0x5400 | 3 * 3;				            	
	        break;                                  

	case 0xC2://(R)		//0xC2	NorthRx Main_Cube's WestTx  	
			clear_north_4_led();		
			clear_otherRxLed_if_same_tx(0xC200, 4 * 3, 34 * 3, 44 * 3, 14 * 3);			
			led_strip_pixels_ShowRxData[4 * 3 + 0] = 0x08;
			led_strip_pixels_ShowRxData[4 * 3 + 1] = 0x00;
			led_strip_pixels_ShowRxData[4 * 3 + 2] = 0x00;
			delay1000msAndTurnOffNorthRx4Led = Imm_4LedsDelayTime;      
			refreshLeds = 1;    									
			NorthRxPreviousData = 0xC200 | 4 * 3;			            	
	        break;                                
	case 0xA2://(G)		//0xA2	NorthRx Sub_Cube1's WestTx         
			clear_north_4_led();		                  
			clear_otherRxLed_if_same_tx(0xA200, 4 * 3, 34 * 3, 44 * 3, 14 * 3);			
			led_strip_pixels_ShowRxData[4 * 3 + 0] = 0x00;
			led_strip_pixels_ShowRxData[4 * 3 + 1] = 0x08;
			led_strip_pixels_ShowRxData[4 * 3 + 2] = 0x00;
			delay1000msAndTurnOffNorthRx4Led = Imm_4LedsDelayTime;      
			refreshLeds = 1;    									
			NorthRxPreviousData = 0xA200 | 4 * 3;			            	
	        break;                                
	case 0x92://(B)		//0x92	NorthRx Sub_Cube2's WestTx         
			clear_north_4_led();		                  
			clear_otherRxLed_if_same_tx(0x9200, 4 * 3, 34 * 3, 44 * 3, 14 * 3);			
			led_strip_pixels_ShowRxData[4 * 3 + 0] = 0x00;
			led_strip_pixels_ShowRxData[4 * 3 + 1] = 0x00;
			led_strip_pixels_ShowRxData[4 * 3 + 2] = 0x08;
			delay1000msAndTurnOffNorthRx4Led = Imm_4LedsDelayTime;      
			refreshLeds = 1;    									
			NorthRxPreviousData = 0x9200 | 4 * 3;			            	
	        break;                                
	case 0x62://(RG)	//0x62	NorthRx Sub_Cube3's WestTx		        
			clear_north_4_led();		                  
			clear_otherRxLed_if_same_tx(0x6200, 4 * 3, 34 * 3, 44 * 3, 14 * 3);			
			led_strip_pixels_ShowRxData[4 * 3 + 0] = 0x08;
			led_strip_pixels_ShowRxData[4 * 3 + 1] = 0x08;
			led_strip_pixels_ShowRxData[4 * 3 + 2] = 0x00;
			delay1000msAndTurnOffNorthRx4Led = Imm_4LedsDelayTime;      
			refreshLeds = 1;    									
			NorthRxPreviousData = 0x6200 | 4 * 3;			            	
	        break;                                
	case 0x52://(RGB)	//0x52	NorthRx Sub_Cube4's WestTx	           
			clear_north_4_led();		                  
			clear_otherRxLed_if_same_tx(0x5200, 4 * 3, 34 * 3, 44 * 3, 14 * 3);			
			led_strip_pixels_ShowRxData[4 * 3 + 0] = 0x08;
			led_strip_pixels_ShowRxData[4 * 3 + 1] = 0x08;
			led_strip_pixels_ShowRxData[4 * 3 + 2] = 0x08;
			delay1000msAndTurnOffNorthRx4Led = Imm_4LedsDelayTime;      
			refreshLeds = 1;    	
			NorthRxPreviousData = 0x5200 | 4 * 3;								
	        break;                                  
	                    
	default:
	    break;
	}	
	if(refreshLeds > 0)
	{//refresh North leds
		refreshLeds = 0;
        ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels_ShowRxData, sizeof(led_strip_pixels_ShowRxData), &tx_config));
        ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));		
	}	
}
//-------
void clear_east_4_led(void)
{
	led_strip_pixels_ShowRxData[13 * 3 + 0] = 0x00;
	led_strip_pixels_ShowRxData[13 * 3 + 1] = 0x00;    
	led_strip_pixels_ShowRxData[13 * 3 + 2] = 0x00; 
	
	led_strip_pixels_ShowRxData[20 * 3 + 0] = 0x00;
	led_strip_pixels_ShowRxData[20 * 3 + 1] = 0x00;    
	led_strip_pixels_ShowRxData[20 * 3 + 2] = 0x00; 
	
	led_strip_pixels_ShowRxData[27 * 3 + 0] = 0x00;
	led_strip_pixels_ShowRxData[27 * 3 + 1] = 0x00;    
	led_strip_pixels_ShowRxData[27 * 3 + 2] = 0x00; 			
	
	led_strip_pixels_ShowRxData[34 * 3 + 0] = 0x00;
	led_strip_pixels_ShowRxData[34 * 3 + 1] = 0x00;    
	led_strip_pixels_ShowRxData[34 * 3 + 2] = 0x00; 			
}
void turn_off_east_4_led_after_delay(void)
{
	if(delay1000msAndTurnOffEastRx4Led == 1)
	{
		delay1000msAndTurnOffEastRx4Led = 0;
		clear_east_4_led();
        ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels_ShowRxData, sizeof(led_strip_pixels_ShowRxData), &tx_config));
        ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));		
		EastRxPreviousData = 0;
	}	
}
void show_east_rec_data(void)
{
	int refreshLeds = 0;    
	switch (newEastRxData){		
	case 0xC8://(R)		//0xC8	EastRx  Main_Cube's NorthTx
			clear_east_4_led();
			clear_otherRxLed_if_same_tx(0xC800, 1 * 3, 13 * 3, 47 * 3, 35 * 3);			
			led_strip_pixels_ShowRxData[13 * 3 + 0] = 0x08;
			led_strip_pixels_ShowRxData[13 * 3 + 1] = 0x00;
			led_strip_pixels_ShowRxData[13 * 3 + 2] = 0x00; 
			delay1000msAndTurnOffEastRx4Led = Imm_4LedsDelayTime;      
			refreshLeds = 1;        				      
			EastRxPreviousData = 0xC800 | 13 * 3;			             	
	        break;                                 
	case 0xA8://(G)		//0xA8	EastRx  Sub_Cube1's NorthTx        
			clear_east_4_led();		                    
			clear_otherRxLed_if_same_tx(0xA800, 1 * 3, 13 * 3, 47 * 3, 35 * 3);			
			led_strip_pixels_ShowRxData[13 * 3 + 0] = 0x00;
			led_strip_pixels_ShowRxData[13 * 3 + 1] = 0x08;
			led_strip_pixels_ShowRxData[13 * 3 + 2] = 0x00;                    	
			delay1000msAndTurnOffEastRx4Led = Imm_4LedsDelayTime;      
			refreshLeds = 1;        				    
			EastRxPreviousData = 0xA800 | 13 * 3;			               	        				
	        break;                                 
	case 0x98://(B)		//0x98	EastRx  Sub_Cube2's NorthTx        
			clear_east_4_led();		                    
			clear_otherRxLed_if_same_tx(0x9800, 1 * 3, 13 * 3, 47 * 3, 35 * 3);			
			led_strip_pixels_ShowRxData[13 * 3 + 0] = 0x00;
			led_strip_pixels_ShowRxData[13 * 3 + 1] = 0x00;
			led_strip_pixels_ShowRxData[13 * 3 + 2] = 0x08;                    	
			delay1000msAndTurnOffEastRx4Led = Imm_4LedsDelayTime;      
			refreshLeds = 1;
			EastRxPreviousData = 0x9800 | 13 * 3;			        				                   	        				
	        break;                                 
	case 0x68://(RG)	//0x68	EastRx  Sub_Cube3's NorthTx       
			clear_east_4_led();		                    
			clear_otherRxLed_if_same_tx(0x6800, 1 * 3, 13 * 3, 47 * 3, 35 * 3);			
			led_strip_pixels_ShowRxData[13 * 3 + 0] = 0x08;
			led_strip_pixels_ShowRxData[13 * 3 + 1] = 0x08;
		 	led_strip_pixels_ShowRxData[13 * 3 + 2] = 0x00;        
			delay1000msAndTurnOffEastRx4Led = Imm_4LedsDelayTime;      
			refreshLeds = 1;        				                   	        			 	            	
			EastRxPreviousData = 0x6800 | 13 * 3;			
	        break;                                 
	case 0x58://(RGB)	//0x58	EastRx  Sub_Cube4's NorthTx      
			clear_east_4_led();		                    
			clear_otherRxLed_if_same_tx(0x5800, 1 * 3, 13 * 3, 47 * 3, 35 * 3);			
		    led_strip_pixels_ShowRxData[13 * 3 + 0] = 0x08;
		    led_strip_pixels_ShowRxData[13 * 3 + 1] = 0x08;
		    led_strip_pixels_ShowRxData[13 * 3 + 2] = 0x08;                    	
			delay1000msAndTurnOffEastRx4Led = Imm_4LedsDelayTime;      
			refreshLeds = 1;        				                   	        			  
			EastRxPreviousData = 0x5800 | 13 * 3;			
	        break;                                 
	                                           
	case 0xC1://(R)		//0xC1	EastRx  Main_Cube's EasyTx        
			clear_east_4_led();		                    
			clear_otherRxLed_if_same_tx(0xC100, 2 * 3, 20 * 3, 46 * 3, 28 * 3);			
		  led_strip_pixels_ShowRxData[20 * 3 + 0] = 0x08;
		  led_strip_pixels_ShowRxData[20 * 3 + 1] = 0x00;
		  led_strip_pixels_ShowRxData[20 * 3 + 2] = 0x00;     
			delay1000msAndTurnOffEastRx4Led = Imm_4LedsDelayTime;      
			refreshLeds = 1;        				                   			    			         	
			EastRxPreviousData = 0xC100 | 20 * 3;			
	    break;                                 
	case 0xA1://(G)   //0xA1	EastRx  Sub_Cube1's EasyTx     
			clear_east_4_led();		                    
			clear_otherRxLed_if_same_tx(0xA100, 2 * 3, 20 * 3, 46 * 3, 28 * 3);			
		    led_strip_pixels_ShowRxData[20 * 3 + 0] = 0x00;
		    led_strip_pixels_ShowRxData[20 * 3 + 1] = 0x08;
		    led_strip_pixels_ShowRxData[20 * 3 + 2] = 0x00;            	
			delay1000msAndTurnOffEastRx4Led = Imm_4LedsDelayTime;      
			refreshLeds = 1;        				                   			    			  
			EastRxPreviousData = 0xA100 | 20 * 3;			
	        break;                                 
	case 0x91://(B)   //0x91	EastRx  Sub_Cube2's EasyTx     
			clear_east_4_led();		                    
			clear_otherRxLed_if_same_tx(0x9100, 2 * 3, 20 * 3, 46 * 3, 28 * 3);			
		    led_strip_pixels_ShowRxData[20 * 3 + 0] = 0x00;
		    led_strip_pixels_ShowRxData[20 * 3 + 1] = 0x00;
		    led_strip_pixels_ShowRxData[20 * 3 + 2] = 0x08;            	
			delay1000msAndTurnOffEastRx4Led = Imm_4LedsDelayTime;      
			refreshLeds = 1;        				                   			    			  
			EastRxPreviousData = 0x9100 | 20 * 3;			
	        break;                                 
	case 0x61://(RG) 	//0x61	EastRx  Sub_Cube3's EasyTx      
			clear_east_4_led();		                    
			clear_otherRxLed_if_same_tx(0x6100, 2 * 3, 20 * 3, 46 * 3, 28 * 3);			
		    led_strip_pixels_ShowRxData[20 * 3 + 0] = 0x08;
		    led_strip_pixels_ShowRxData[20 * 3 + 1] = 0x08;
		    led_strip_pixels_ShowRxData[20 * 3 + 2] = 0x00;            	
			delay1000msAndTurnOffEastRx4Led = Imm_4LedsDelayTime;      
			refreshLeds = 1;        				                   			    			  
			EastRxPreviousData = 0x6100 | 20 * 3;			
	        break;                                 
	case 0x51://(RGB) //0x51	EastRx  Sub_Cube4's EasyTx     
			clear_east_4_led();		                    
			clear_otherRxLed_if_same_tx(0x5100, 2 * 3, 20 * 3, 46 * 3, 28 * 3);			
		    led_strip_pixels_ShowRxData[20 * 3 + 0] = 0x08;
		    led_strip_pixels_ShowRxData[20 * 3 + 1] = 0x08;
		    led_strip_pixels_ShowRxData[20 * 3 + 2] = 0x08;            	
			delay1000msAndTurnOffEastRx4Led = Imm_4LedsDelayTime;      
			refreshLeds = 1;        				                   			    			  
			EastRxPreviousData = 0x5100 | 20 * 3;			
	        break;                                 
                                         
	case 0xC4://(R)		//0xC4	EastRx  Main_Cube's SouthTx        
			clear_east_4_led();		                    
			clear_otherRxLed_if_same_tx(0xC400, 3 * 3, 27 * 3, 45 * 3, 21 * 3);			
		    led_strip_pixels_ShowRxData[27 * 3 + 0] = 0x08;
		    led_strip_pixels_ShowRxData[27 * 3 + 1] = 0x00;
		    led_strip_pixels_ShowRxData[27 * 3 + 2] = 0x00;		
			delay1000msAndTurnOffEastRx4Led = Imm_4LedsDelayTime;      
			refreshLeds = 1;        				                   			    			  		            	
			EastRxPreviousData = 0xC400 | 27 * 3;			
	        break;                                 
	case 0xA4://(G)		//0xA4	EastRx  Sub_Cube1's SouthTx        
			clear_east_4_led();		                    
			clear_otherRxLed_if_same_tx(0xA400, 3 * 3, 27 * 3, 45 * 3, 21 * 3);			
		    led_strip_pixels_ShowRxData[27 * 3 + 0] = 0x00;
		    led_strip_pixels_ShowRxData[27 * 3 + 1] = 0x08;
		    led_strip_pixels_ShowRxData[27 * 3 + 2] = 0x00;				            	
			delay1000msAndTurnOffEastRx4Led = Imm_4LedsDelayTime;      
			refreshLeds = 1;        				                   			    			  
			EastRxPreviousData = 0xA400 | 27 * 3;			
	        break;                                 
	case 0x94://(B)		//0x94	EastRx  Sub_Cube2's SouthTx        
			clear_east_4_led();		                    
			clear_otherRxLed_if_same_tx(0x9400, 3 * 3, 27 * 3, 45 * 3, 21 * 3);			
		    led_strip_pixels_ShowRxData[27 * 3 + 0] = 0x00;
		    led_strip_pixels_ShowRxData[27 * 3 + 1] = 0x00;
		    led_strip_pixels_ShowRxData[27 * 3 + 2] = 0x08;				            	
			delay1000msAndTurnOffEastRx4Led = Imm_4LedsDelayTime;      
			refreshLeds = 1;        				                   			    			  
			EastRxPreviousData = 0x9400 | 27 * 3;			
	        break;                                 
	case 0x64://(RG)  //0x64	EastRx  Sub_Cube3's SouthTx     
			clear_east_4_led();		                    
			clear_otherRxLed_if_same_tx(0x6400, 3 * 3, 27 * 3, 45 * 3, 21 * 3);			
		    led_strip_pixels_ShowRxData[27 * 3 + 0] = 0x08;
		    led_strip_pixels_ShowRxData[27 * 3 + 1] = 0x08;
		    led_strip_pixels_ShowRxData[27 * 3 + 2] = 0x00;				            	
			delay1000msAndTurnOffEastRx4Led = Imm_4LedsDelayTime;      
			refreshLeds = 1;        				                   			    			  
			EastRxPreviousData = 0x6400 | 27 * 3;			
	        break;                                 
	case 0x54://(RGB) //0x54	EastRx  Sub_Cube4's SouthTx	       
			clear_east_4_led();		                    
			clear_otherRxLed_if_same_tx(0x5400, 3 * 3, 27 * 3, 45 * 3, 21 * 3);			
		    led_strip_pixels_ShowRxData[27 * 3 + 0] = 0x08;
		    led_strip_pixels_ShowRxData[27 * 3 + 1] = 0x08;
		    led_strip_pixels_ShowRxData[27 * 3 + 2] = 0x08;				            	
			delay1000msAndTurnOffEastRx4Led = Imm_4LedsDelayTime;      
			refreshLeds = 1;        				                   			    			  
			EastRxPreviousData = 0x5400 | 27 * 3;			
	        break;                                  
	                                           
	case 0xC2://(R)		//0xC2	EastRx  Main_Cube's WestTx        
			clear_east_4_led();		                    
			clear_otherRxLed_if_same_tx(0xC200, 4 * 3, 34 * 3, 44 * 3, 14 * 3);			
		    led_strip_pixels_ShowRxData[34 * 3 + 0] = 0x08;
		    led_strip_pixels_ShowRxData[34 * 3 + 1] = 0x00;
		    led_strip_pixels_ShowRxData[34 * 3 + 2] = 0x00;			            	
			delay1000msAndTurnOffEastRx4Led = Imm_4LedsDelayTime;      
			refreshLeds = 1;        				                   			    			  
			EastRxPreviousData = 0xC200 | 34 * 3;			
	        break;                                 
	case 0xA2://(G) 	//0xA2	EastRx  Sub_Cube1's WestTx       
			clear_east_4_led();		                    
			clear_otherRxLed_if_same_tx(0xA200, 4 * 3, 34 * 3, 44 * 3, 14 * 3);			
		    led_strip_pixels_ShowRxData[34 * 3 + 0] = 0x00;
		    led_strip_pixels_ShowRxData[34 * 3 + 1] = 0x08;
		    led_strip_pixels_ShowRxData[34 * 3 + 2] = 0x00;			            	
			delay1000msAndTurnOffEastRx4Led = Imm_4LedsDelayTime;      
			refreshLeds = 1;        				                   			    			  
			EastRxPreviousData = 0xA200 | 34 * 3;			
	        break;                                 
	case 0x92://(B)   //0x92	EastRx  Sub_Cube2's WestTx     
			clear_east_4_led();		                    
			clear_otherRxLed_if_same_tx(0x9200, 4 * 3, 34 * 3, 44 * 3, 14 * 3);			
		    led_strip_pixels_ShowRxData[34 * 3 + 0] = 0x00;
		    led_strip_pixels_ShowRxData[34 * 3 + 1] = 0x00;
		    led_strip_pixels_ShowRxData[34 * 3 + 2] = 0x08;			            	
			delay1000msAndTurnOffEastRx4Led = Imm_4LedsDelayTime;      
			refreshLeds = 1;        				                   			    			  
			EastRxPreviousData = 0x9200 | 34 * 3;			
	        break;                                 
	case 0x62://(RG)  //0x62	EastRx  Sub_Cube3's WestTx     
			clear_east_4_led();		                    
			clear_otherRxLed_if_same_tx(0x6200, 4 * 3, 34 * 3, 44 * 3, 14 * 3);			
		    led_strip_pixels_ShowRxData[34 * 3 + 0] = 0x08;
		    led_strip_pixels_ShowRxData[34 * 3 + 1] = 0x08;
		    led_strip_pixels_ShowRxData[34 * 3 + 2] = 0x00;			            	
			delay1000msAndTurnOffEastRx4Led = Imm_4LedsDelayTime;      
			refreshLeds = 1;        				                   			    			  
			EastRxPreviousData = 0x6200 | 34 * 3;			
	        break;                                 
	case 0x52://(RGB) //0x52	EastRx  Sub_Cube4's WestTx    
			clear_east_4_led();		                    
			clear_otherRxLed_if_same_tx(0x5200, 4 * 3, 34 * 3, 44 * 3, 14 * 3);			
		    led_strip_pixels_ShowRxData[34 * 3 + 0] = 0x08;
		    led_strip_pixels_ShowRxData[34 * 3 + 1] = 0x08;
		    led_strip_pixels_ShowRxData[34 * 3 + 2] = 0x08;			            	
			delay1000msAndTurnOffEastRx4Led = Imm_4LedsDelayTime;      
			refreshLeds = 1;
			EastRxPreviousData = 0x5200 | 34 * 3;			
	        break;                                  
	                    
	default:
	    break;
	}	
	if(refreshLeds > 0)
	{//refresh East leds
		refreshLeds = 0;
        ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels_ShowRxData, sizeof(led_strip_pixels_ShowRxData), &tx_config));
        ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));		
	}	
}
//-------
void clear_south_4_led(void)
{
	led_strip_pixels_ShowRxData[47 * 3 + 0] = 0x00;
	led_strip_pixels_ShowRxData[47 * 3 + 1] = 0x00;    
	led_strip_pixels_ShowRxData[47 * 3 + 2] = 0x00; 
	
	led_strip_pixels_ShowRxData[46 * 3 + 0] = 0x00;
	led_strip_pixels_ShowRxData[46 * 3 + 1] = 0x00;    
	led_strip_pixels_ShowRxData[46 * 3 + 2] = 0x00; 
	
	led_strip_pixels_ShowRxData[45 * 3 + 0] = 0x00;
	led_strip_pixels_ShowRxData[45 * 3 + 1] = 0x00;    
	led_strip_pixels_ShowRxData[45 * 3 + 2] = 0x00; 			
	
	led_strip_pixels_ShowRxData[44 * 3 + 0] = 0x00;
	led_strip_pixels_ShowRxData[44 * 3 + 1] = 0x00;    
	led_strip_pixels_ShowRxData[44 * 3 + 2] = 0x00; 			
}
void turn_off_south_4_led_after_delay(void)
{
	if(delay1000msAndTurnOffSouthRx4Led == 1)
	{
		delay1000msAndTurnOffSouthRx4Led = 0;
		clear_south_4_led();
        ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels_ShowRxData, sizeof(led_strip_pixels_ShowRxData), &tx_config));
        ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));		
		SouthRxPreviousData = 0;
	}	
}
void show_south_rec_data(void)
{
	int refreshLeds = 0;    
	switch (newSouthRxData){
	case 0xC8://(R)		//0xC8	SouthRx Main_Cube's NorthTx
			clear_south_4_led();
			clear_otherRxLed_if_same_tx(0xC800, 1 * 3, 13 * 3, 47 * 3, 35 * 3);			
		  led_strip_pixels_ShowRxData[47 * 3 + 0] = 0x08;
		  led_strip_pixels_ShowRxData[47 * 3 + 1] = 0x00;
		  led_strip_pixels_ShowRxData[47 * 3 + 2] = 0x00;                   	
		  delay1000msAndTurnOffSouthRx4Led = Imm_4LedsDelayTime;      
		  refreshLeds = 1;
			SouthRxPreviousData = 0xC800 | 47 * 3;		  
	    break;                                 
	case 0xA8://(G)   //0xA8	SouthRx Sub_Cube1's NorthTx      
			clear_south_4_led();		                   
			clear_otherRxLed_if_same_tx(0xA800, 1 * 3, 13 * 3, 47 * 3, 35 * 3);			
		  led_strip_pixels_ShowRxData[47 * 3 + 0] = 0x00;
		  led_strip_pixels_ShowRxData[47 * 3 + 1] = 0x08;
		  led_strip_pixels_ShowRxData[47 * 3 + 2] = 0x00;
		  delay1000msAndTurnOffSouthRx4Led = Imm_4LedsDelayTime;      
		  refreshLeds = 1;        			                     	
			SouthRxPreviousData = 0xA800 | 47 * 3;		  
	    break;                                 
	case 0x98://(B)   //0x98	SouthRx Sub_Cube2's NorthTx      
			clear_south_4_led();		                   
			clear_otherRxLed_if_same_tx(0x9800, 1 * 3, 13 * 3, 47 * 3, 35 * 3);			
		  led_strip_pixels_ShowRxData[47 * 3 + 0] = 0x00;
		  led_strip_pixels_ShowRxData[47 * 3 + 1] = 0x00;
		  led_strip_pixels_ShowRxData[47 * 3 + 2] = 0x08;                   	
		  delay1000msAndTurnOffSouthRx4Led = Imm_4LedsDelayTime;      
		  refreshLeds = 1;        			  
			SouthRxPreviousData = 0x9800 | 47 * 3;		  
	    break;                                 
	case 0x68://(RG)  //0x68	SouthRx Sub_Cube3's NorthTx      
			clear_south_4_led();		                   
			clear_otherRxLed_if_same_tx(0x6800, 1 * 3, 13 * 3, 47 * 3, 35 * 3);			
		  led_strip_pixels_ShowRxData[47 * 3 + 0] = 0x08;
		  led_strip_pixels_ShowRxData[47 * 3 + 1] = 0x08;
		  led_strip_pixels_ShowRxData[47 * 3 + 2] = 0x00;                   	
		  delay1000msAndTurnOffSouthRx4Led = Imm_4LedsDelayTime;      
		  refreshLeds = 1;        			  
			SouthRxPreviousData = 0x6800 | 47 * 3;		  
	    break;                                 
	case 0x58://(RGB) //0x58	SouthRx Sub_Cube4's NorthTx		      
			clear_south_4_led();		                   
			clear_otherRxLed_if_same_tx(0x5800, 1 * 3, 13 * 3, 47 * 3, 35 * 3);			
		  led_strip_pixels_ShowRxData[47 * 3 + 0] = 0x08;
		  led_strip_pixels_ShowRxData[47 * 3 + 1] = 0x08;
		  led_strip_pixels_ShowRxData[47 * 3 + 2] = 0x08;                   	
		  delay1000msAndTurnOffSouthRx4Led = Imm_4LedsDelayTime;      
		  refreshLeds = 1;        			  
			SouthRxPreviousData = 0x5800 | 47 * 3;		  
	    break;                                 

	case 0xC1://(R)   //0xC1	SouthRx Main_Cube's EasyTx      
			clear_south_4_led();		                   
			clear_otherRxLed_if_same_tx(0xC100, 2 * 3, 20 * 3, 46 * 3, 28 * 3);			
		  led_strip_pixels_ShowRxData[46 * 3 + 0] = 0x08;
		  led_strip_pixels_ShowRxData[46 * 3 + 1] = 0x00;
		  led_strip_pixels_ShowRxData[46 * 3 + 2] = 0x00;				            	
		  delay1000msAndTurnOffSouthRx4Led = Imm_4LedsDelayTime;      
		  refreshLeds = 1;		    			  
			SouthRxPreviousData = 0xC100 | 46 * 3;		  
	    break;                                 
	case 0xA1://(G)   //0xA1	SouthRx Sub_Cube1's EasyTx      
			clear_south_4_led();		                   
			clear_otherRxLed_if_same_tx(0xA100, 2 * 3, 20 * 3, 46 * 3, 28 * 3);			
		  led_strip_pixels_ShowRxData[46 * 3 + 0] = 0x00;
		  led_strip_pixels_ShowRxData[46 * 3 + 1] = 0x08;
		  led_strip_pixels_ShowRxData[46 * 3 + 2] = 0x00;				            	
		  delay1000msAndTurnOffSouthRx4Led = Imm_4LedsDelayTime;      
		  refreshLeds = 1;		    			  
			SouthRxPreviousData = 0xA100 | 46 * 3;		  
	    break;                                 
	case 0x91://(B)   //0x91	SouthRx Sub_Cube2's EasyTx     
			clear_south_4_led();		                   
			clear_otherRxLed_if_same_tx(0x9100, 2 * 3, 20 * 3, 46 * 3, 28 * 3);			
		  led_strip_pixels_ShowRxData[46 * 3 + 0] = 0x00;
		  led_strip_pixels_ShowRxData[46 * 3 + 1] = 0x00;
		  led_strip_pixels_ShowRxData[46 * 3 + 2] = 0x08;				            	
		  delay1000msAndTurnOffSouthRx4Led = Imm_4LedsDelayTime;      
		  refreshLeds = 1;		    			  
			SouthRxPreviousData = 0x9100 | 46 * 3;		  
	    break;                                 
	case 0x61://(RG)  //0x61	SouthRx Sub_Cube3's EasyTx      
			clear_south_4_led();		                   
			clear_otherRxLed_if_same_tx(0x6100, 2 * 3, 20 * 3, 46 * 3, 28 * 3);			
		  led_strip_pixels_ShowRxData[46 * 3 + 0] = 0x08;
		  led_strip_pixels_ShowRxData[46 * 3 + 1] = 0x08;
		  led_strip_pixels_ShowRxData[46 * 3 + 2] = 0x00;				            	
		  delay1000msAndTurnOffSouthRx4Led = Imm_4LedsDelayTime;      
		  refreshLeds = 1;		    			  
			SouthRxPreviousData = 0x6100 | 46 * 3;		  
	    break;                                 
	case 0x51://(RGB) //0x51	SouthRx Sub_Cube4's EasyTx	                                                 
			clear_south_4_led();		                   
			clear_otherRxLed_if_same_tx(0x5100, 2 * 3, 20 * 3, 46 * 3, 28 * 3);			
		  led_strip_pixels_ShowRxData[46 * 3 + 0] = 0x08;
		  led_strip_pixels_ShowRxData[46 * 3 + 1] = 0x08;
		  led_strip_pixels_ShowRxData[46 * 3 + 2] = 0x08;				            	
		  delay1000msAndTurnOffSouthRx4Led = Imm_4LedsDelayTime;      
		  refreshLeds = 1;		    			  
			SouthRxPreviousData = 0x5100 | 46 * 3;		  
	    break;                                 
                                        
	case 0xC4://(R)   //0xC4	SouthRx Main_Cube's SouthTx      
			clear_south_4_led();		                   
			clear_otherRxLed_if_same_tx(0xC400, 3 * 3, 27 * 3, 45 * 3, 21 * 3);			
		  led_strip_pixels_ShowRxData[45 * 3 + 0] = 0x08;
		  led_strip_pixels_ShowRxData[45 * 3 + 1] = 0x00;
		  led_strip_pixels_ShowRxData[45 * 3 + 2] = 0x00;				            	
		  delay1000msAndTurnOffSouthRx4Led = Imm_4LedsDelayTime;      
		  refreshLeds = 1;		    			  
			SouthRxPreviousData = 0xC400 | 45 * 3;		  
	    break;                                 
	case 0xA4://(G)   //0xA4	SouthRx Sub_Cube1's SouthTx      
			clear_south_4_led();		                   
			clear_otherRxLed_if_same_tx(0xA400, 3 * 3, 27 * 3, 45 * 3, 21 * 3);			
		  led_strip_pixels_ShowRxData[45 * 3 + 0] = 0x00;
		  led_strip_pixels_ShowRxData[45 * 3 + 1] = 0x08;
		  led_strip_pixels_ShowRxData[45 * 3 + 2] = 0x00;				            	
		  delay1000msAndTurnOffSouthRx4Led = Imm_4LedsDelayTime;      
		  refreshLeds = 1;		    			  
			SouthRxPreviousData = 0xA400 | 45 * 3;		  
	    break;                                 
	case 0x94://(B)   //0x94	SouthRx Sub_Cube2's SouthTx      
			clear_south_4_led();		                   
			clear_otherRxLed_if_same_tx(0x9400, 3 * 3, 27 * 3, 45 * 3, 21 * 3);			
		  led_strip_pixels_ShowRxData[45 * 3 + 0] = 0x00;
		  led_strip_pixels_ShowRxData[45 * 3 + 1] = 0x00;
		  led_strip_pixels_ShowRxData[45 * 3 + 2] = 0x08;				            	
		  delay1000msAndTurnOffSouthRx4Led = Imm_4LedsDelayTime;      
		  refreshLeds = 1;		    			  
			SouthRxPreviousData = 0x9400 | 45 * 3;		  
	    break;                                 
	case 0x64://(RG)  //0x64	SouthRx Sub_Cube3's SouthTx      
			clear_south_4_led();		                   
			clear_otherRxLed_if_same_tx(0x6400, 3 * 3, 27 * 3, 45 * 3, 21 * 3);			
		  led_strip_pixels_ShowRxData[45 * 3 + 0] = 0x08;
		  led_strip_pixels_ShowRxData[45 * 3 + 1] = 0x08;
		  led_strip_pixels_ShowRxData[45 * 3 + 2] = 0x00;				            	
		  delay1000msAndTurnOffSouthRx4Led = Imm_4LedsDelayTime;      
		  refreshLeds = 1;		    			  
			SouthRxPreviousData = 0x6400 | 45 * 3;		  
	    break;                                 
	case 0x54://(RGB) //0x54	SouthRx Sub_Cube4's SouthTx	         
			clear_south_4_led();		                   
			clear_otherRxLed_if_same_tx(0x5400, 3 * 3, 27 * 3, 45 * 3, 21 * 3);			
		  led_strip_pixels_ShowRxData[45 * 3 + 0] = 0x08;
		  led_strip_pixels_ShowRxData[45 * 3 + 1] = 0x08;
		  led_strip_pixels_ShowRxData[45 * 3 + 2] = 0x08;				            	
		  delay1000msAndTurnOffSouthRx4Led = Imm_4LedsDelayTime;      
		  refreshLeds = 1;		    			  
			SouthRxPreviousData = 0x5400 | 45 * 3;		  
	    break;                                  
	                                           
	case 0xC2://(R)   //0xC2	SouthRx Main_Cube's WestTx      
			clear_south_4_led();		          
			clear_otherRxLed_if_same_tx(0xC200, 4 * 3, 34 * 3, 44 * 3, 14 * 3);			         
		  led_strip_pixels_ShowRxData[44 * 3 + 0] = 0x08;
		  led_strip_pixels_ShowRxData[44 * 3 + 1] = 0x00;
		  led_strip_pixels_ShowRxData[44 * 3 + 2] = 0x00;				            	
		  delay1000msAndTurnOffSouthRx4Led = Imm_4LedsDelayTime;      
		  refreshLeds = 1;		    			  
			SouthRxPreviousData = 0xC200 | 44 * 3;		  
	    break;                                 
	case 0xA2://(G)   //0xA2	SouthRx Sub_Cube1's WestTx      
			clear_south_4_led();		                   
			clear_otherRxLed_if_same_tx(0xA200, 4 * 3, 34 * 3, 44 * 3, 14 * 3);			
		  led_strip_pixels_ShowRxData[44 * 3 + 0] = 0x00;
		  led_strip_pixels_ShowRxData[44 * 3 + 1] = 0x08;
		  led_strip_pixels_ShowRxData[44 * 3 + 2] = 0x00;				            	
		  delay1000msAndTurnOffSouthRx4Led = Imm_4LedsDelayTime;      
		  refreshLeds = 1;		    			  
			SouthRxPreviousData = 0xA200 | 44 * 3;		  
	    break;                                 
	case 0x92://(B)   //0x92	SouthRx Sub_Cube2's WestTx      
			clear_south_4_led();		                   
			clear_otherRxLed_if_same_tx(0x9200, 4 * 3, 34 * 3, 44 * 3, 14 * 3);			
		  led_strip_pixels_ShowRxData[44 * 3 + 0] = 0x00;
		  led_strip_pixels_ShowRxData[44 * 3 + 1] = 0x00;
		  led_strip_pixels_ShowRxData[44 * 3 + 2] = 0x08;				            	
		  delay1000msAndTurnOffSouthRx4Led = Imm_4LedsDelayTime;      
		  refreshLeds = 1;		    			  
			SouthRxPreviousData = 0x9200 | 44 * 3;		  
	    break;                                 
	case 0x62://(RG)  //0x62	SouthRx Sub_Cube3's WestTx      
			clear_south_4_led();		                   
			clear_otherRxLed_if_same_tx(0x6200, 4 * 3, 34 * 3, 44 * 3, 14 * 3);			
		  led_strip_pixels_ShowRxData[44 * 3 + 0] = 0x08;
		  led_strip_pixels_ShowRxData[44 * 3 + 1] = 0x08;
		  led_strip_pixels_ShowRxData[44 * 3 + 2] = 0x00;				            	
		  delay1000msAndTurnOffSouthRx4Led = Imm_4LedsDelayTime;      
		  refreshLeds = 1;		    			  
			SouthRxPreviousData = 0x6200 | 44 * 3;		  
	    break;                                 
	case 0x52://(RGB) //0x52	SouthRx Sub_Cube4's WestTx      
			clear_south_4_led();		                   
			clear_otherRxLed_if_same_tx(0x5200, 4 * 3, 34 * 3, 44 * 3, 14 * 3);			
		  led_strip_pixels_ShowRxData[44 * 3 + 0] = 0x08;
		  led_strip_pixels_ShowRxData[44 * 3 + 1] = 0x08;
		  led_strip_pixels_ShowRxData[44 * 3 + 2] = 0x08;				            	
		  delay1000msAndTurnOffSouthRx4Led = Imm_4LedsDelayTime;      
		  refreshLeds = 1;		    			  
			SouthRxPreviousData = 0x5200 | 44 * 3;		  
	    break;                                  
	                    
	default:
	    break;
	}	
	if(refreshLeds > 0)
	{//refresh South leds
		refreshLeds = 0;
    ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels_ShowRxData, sizeof(led_strip_pixels_ShowRxData), &tx_config));
    ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));		
	}	
}
//-------
void clear_west_4_led(void)
{
	led_strip_pixels_ShowRxData[35 * 3 + 0] = 0x00;
	led_strip_pixels_ShowRxData[35 * 3 + 1] = 0x00;    
	led_strip_pixels_ShowRxData[35 * 3 + 2] = 0x00; 
	
	led_strip_pixels_ShowRxData[28 * 3 + 0] = 0x00;
	led_strip_pixels_ShowRxData[28 * 3 + 1] = 0x00;    
	led_strip_pixels_ShowRxData[28 * 3 + 2] = 0x00; 
	
	led_strip_pixels_ShowRxData[21 * 3 + 0] = 0x00;
	led_strip_pixels_ShowRxData[21 * 3 + 1] = 0x00;    
	led_strip_pixels_ShowRxData[21 * 3 + 2] = 0x00; 			
	
	led_strip_pixels_ShowRxData[14 * 3 + 0] = 0x00;
	led_strip_pixels_ShowRxData[14 * 3 + 1] = 0x00;    
	led_strip_pixels_ShowRxData[14 * 3 + 2] = 0x00; 			
}
void turn_off_west_4_led_after_delay(void)
{
	if(delay1000msAndTurnOffWestRx4Led == 1)
	{
		delay1000msAndTurnOffWestRx4Led = 0;
		clear_west_4_led();
    ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels_ShowRxData, sizeof(led_strip_pixels_ShowRxData), &tx_config));
    ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));		
		WestRxPreviousData = 0;    
	}	
}
void show_west_rec_data(void)
{
	int refreshLeds = 0;    
	switch (newWestRxData){	
	case 0xC8://(R)	 //0xC8	WestRx  Main_Cube's NorthTx
			clear_west_4_led();
			clear_otherRxLed_if_same_tx(0xC800, 1 * 3, 13 * 3, 47 * 3, 35 * 3);			
		  led_strip_pixels_ShowRxData[35 * 3 + 0] = 0x08;
		  led_strip_pixels_ShowRxData[35 * 3 + 1] = 0x00;
		  led_strip_pixels_ShowRxData[35 * 3 + 2] = 0x00;                   	
		  delay1000msAndTurnOffWestRx4Led = Imm_4LedsDelayTime;      
		  refreshLeds = 1;
			WestRxPreviousData = 0xC800 | 35 * 3;		  
	    break;                                 
	case 0xA8://(G)  //0xA8	WestRx  Sub_Cube1's NorthTx      
			clear_west_4_led();		                    
			clear_otherRxLed_if_same_tx(0xA800, 1 * 3, 13 * 3, 47 * 3, 35 * 3);			
		  led_strip_pixels_ShowRxData[35 * 3 + 0] = 0x00;
		  led_strip_pixels_ShowRxData[35 * 3 + 1] = 0x08;
		  led_strip_pixels_ShowRxData[35 * 3 + 2] = 0x00;                   	
		  delay1000msAndTurnOffWestRx4Led = Imm_4LedsDelayTime;      
		  refreshLeds = 1;        			  
			WestRxPreviousData = 0xA800 | 35 * 3;		  
	    break;                                 
	case 0x98://(B)  //0x98	WestRx  Sub_Cube2's NorthTx      
			clear_west_4_led();		                    
			clear_otherRxLed_if_same_tx(0x9800, 1 * 3, 13 * 3, 47 * 3, 35 * 3);			
		  led_strip_pixels_ShowRxData[35 * 3 + 0] = 0x00;
		  led_strip_pixels_ShowRxData[35 * 3 + 1] = 0x00;
		  led_strip_pixels_ShowRxData[35 * 3 + 2] = 0x08;                
		  delay1000msAndTurnOffWestRx4Led = Imm_4LedsDelayTime;      
		  refreshLeds = 1;        			     	
			WestRxPreviousData = 0x9800 | 35 * 3;		  
	    break;                                 
	case 0x68://(RG) //0x68	WestRx  Sub_Cube3's NorthTx      
			clear_west_4_led();		                    
			clear_otherRxLed_if_same_tx(0x6800, 1 * 3, 13 * 3, 47 * 3, 35 * 3);			
		  led_strip_pixels_ShowRxData[35 * 3 + 0] = 0x08;
		  led_strip_pixels_ShowRxData[35 * 3 + 1] = 0x08;
		  led_strip_pixels_ShowRxData[35 * 3 + 2] = 0x00;                   	
		  delay1000msAndTurnOffWestRx4Led = Imm_4LedsDelayTime;      
		  refreshLeds = 1;        			  
			WestRxPreviousData = 0x6800 | 35 * 3;		  
	    break;                                 
	case 0x58://(RGB)//0x58	WestRx  Sub_Cube4's NorthTx	      
			clear_west_4_led();		                    
			clear_otherRxLed_if_same_tx(0x5800, 1 * 3, 13 * 3, 47 * 3, 35 * 3);			
		  led_strip_pixels_ShowRxData[35 * 3 + 0] = 0x08;
		  led_strip_pixels_ShowRxData[35 * 3 + 1] = 0x08;
		  led_strip_pixels_ShowRxData[35 * 3 + 2] = 0x08;                   	
		  delay1000msAndTurnOffWestRx4Led = Imm_4LedsDelayTime;      
		  refreshLeds = 1;        			  
			WestRxPreviousData = 0x5800 | 35 * 3;		  
	    break;                                 
                                          
	case 0xC1://(R)  //0xC1	WestRx  Main_Cube's EasyTx      
			clear_west_4_led();		             
			clear_otherRxLed_if_same_tx(0xC100, 2 * 3, 20 * 3, 46 * 3, 28 * 3);			       
		  led_strip_pixels_ShowRxData[28 * 3 + 0] = 0x08;
		  led_strip_pixels_ShowRxData[28 * 3 + 1] = 0x00;
		  led_strip_pixels_ShowRxData[28 * 3 + 2] = 0x00;            	
		  delay1000msAndTurnOffWestRx4Led = Imm_4LedsDelayTime;      
		  refreshLeds = 1;		    			  
			WestRxPreviousData = 0xC100 | 28 * 3;		  
	    break;                                 
	case 0xA1://(G)  //0xA1	WestRx  Sub_Cube1's EasyTx         
			clear_west_4_led();		                    
			clear_otherRxLed_if_same_tx(0xA100, 2 * 3, 20 * 3, 46 * 3, 28 * 3);			
		  led_strip_pixels_ShowRxData[28 * 3 + 0] = 0x00;
		  led_strip_pixels_ShowRxData[28 * 3 + 1] = 0x08;
		  led_strip_pixels_ShowRxData[28 * 3 + 2] = 0x00;            	
		  delay1000msAndTurnOffWestRx4Led = Imm_4LedsDelayTime;      
		  refreshLeds = 1;		    			  
			WestRxPreviousData = 0xA100 | 28 * 3;		  
	    break;                                 
	case 0x91://(B)  //0x91	WestRx  Sub_Cube2's EasyTx           
			clear_west_4_led();		                    
			clear_otherRxLed_if_same_tx(0x9100, 2 * 3, 20 * 3, 46 * 3, 28 * 3);			
		  led_strip_pixels_ShowRxData[28 * 3 + 0] = 0x00;
		  led_strip_pixels_ShowRxData[28 * 3 + 1] = 0x00;
		  led_strip_pixels_ShowRxData[28 * 3 + 2] = 0x08;            	
		  delay1000msAndTurnOffWestRx4Led = Imm_4LedsDelayTime;      
		  refreshLeds = 1;		    			  
			WestRxPreviousData = 0x9100 | 28 * 3;		  
	    break;                                 
	case 0x61://(RG) //0x61	WestRx  Sub_Cube3's EasyTx           
			clear_west_4_led();		                    
			clear_otherRxLed_if_same_tx(0x6100, 2 * 3, 20 * 3, 46 * 3, 28 * 3);			
		  led_strip_pixels_ShowRxData[28 * 3 + 0] = 0x08;
		  led_strip_pixels_ShowRxData[28 * 3 + 1] = 0x08;
		  led_strip_pixels_ShowRxData[28 * 3 + 2] = 0x00;            	
		  delay1000msAndTurnOffWestRx4Led = Imm_4LedsDelayTime;      
		  refreshLeds = 1;		    			  
			WestRxPreviousData = 0x6100 | 28 * 3;		  
	    break;                                 
	case 0x51://(RGB)//0x51	WestRx  Sub_Cube4's EasyTx	        
			clear_west_4_led();		                    
			clear_otherRxLed_if_same_tx(0x5100, 2 * 3, 20 * 3, 46 * 3, 28 * 3);			
		  led_strip_pixels_ShowRxData[28 * 3 + 0] = 0x08;
		  led_strip_pixels_ShowRxData[28 * 3 + 1] = 0x08;
		  led_strip_pixels_ShowRxData[28 * 3 + 2] = 0x08;            	
		  delay1000msAndTurnOffWestRx4Led = Imm_4LedsDelayTime;      
		  refreshLeds = 1;		    			  
			WestRxPreviousData = 0x5100 | 28 * 3;		  
	    break;                                 
                                           
	case 0xC4://(R)  //0xC4	WestRx  Main_Cube's SouthTx      
			clear_west_4_led();		                    
			clear_otherRxLed_if_same_tx(0xC400, 3 * 3, 27 * 3, 45 * 3, 21 * 3);			
		  led_strip_pixels_ShowRxData[21 * 3 + 0] = 0x08;
		  led_strip_pixels_ShowRxData[21 * 3 + 1] = 0x00;
		  led_strip_pixels_ShowRxData[21 * 3 + 2] = 0x00;			            	
		  delay1000msAndTurnOffWestRx4Led = Imm_4LedsDelayTime;      
		  refreshLeds = 1;		    			  
			WestRxPreviousData = 0xC400 | 21 * 3;		  
	    break;                                 
	case 0xA4://(G)  //0xA4	WestRx  Sub_Cube1's SouthTx      
			clear_west_4_led();		                    
			clear_otherRxLed_if_same_tx(0xA400, 3 * 3, 27 * 3, 45 * 3, 21 * 3);			
		  led_strip_pixels_ShowRxData[21 * 3 + 0] = 0x00;
		  led_strip_pixels_ShowRxData[21 * 3 + 1] = 0x08;
		  led_strip_pixels_ShowRxData[21 * 3 + 2] = 0x00;			            	
		  delay1000msAndTurnOffWestRx4Led = Imm_4LedsDelayTime;      
		  refreshLeds = 1;		    			  
			WestRxPreviousData = 0xA400 | 21 * 3;		  
	    break;                                 
	case 0x94://(B)  //0x94	WestRx  Sub_Cube2's SouthTx      
			clear_west_4_led();		                    
			clear_otherRxLed_if_same_tx(0x9400, 3 * 3, 27 * 3, 45 * 3, 21 * 3);			
		  led_strip_pixels_ShowRxData[21 * 3 + 0] = 0x00;
		  led_strip_pixels_ShowRxData[21 * 3 + 1] = 0x00;
		  led_strip_pixels_ShowRxData[21 * 3 + 2] = 0x08;			            	
		  delay1000msAndTurnOffWestRx4Led = Imm_4LedsDelayTime;      
		  refreshLeds = 1;		    			  
			WestRxPreviousData = 0x9400 | 21 * 3;		  
	    break;                                 
	case 0x64://(RG) //0x64	WestRx  Sub_Cube3's SouthTx      
			clear_west_4_led();		                    
			clear_otherRxLed_if_same_tx(0x6400, 3 * 3, 27 * 3, 45 * 3, 21 * 3);			
		  led_strip_pixels_ShowRxData[21 * 3 + 0] = 0x08;
		  led_strip_pixels_ShowRxData[21 * 3 + 1] = 0x08;
		  led_strip_pixels_ShowRxData[21 * 3 + 2] = 0x00;			            	
		  delay1000msAndTurnOffWestRx4Led = Imm_4LedsDelayTime;      
		  refreshLeds = 1;		    			  
			WestRxPreviousData = 0x6400 | 21 * 3;		  
	    break;                                 
	case 0x54://(RGB)//0x54	WestRx  Sub_Cube4's SouthTx	      
			clear_west_4_led();		                    
			clear_otherRxLed_if_same_tx(0x5400, 3 * 3, 27 * 3, 45 * 3, 21 * 3);			
		  led_strip_pixels_ShowRxData[21 * 3 + 0] = 0x08;
		  led_strip_pixels_ShowRxData[21 * 3 + 1] = 0x08;
		  led_strip_pixels_ShowRxData[21 * 3 + 2] = 0x08;			            	
		  delay1000msAndTurnOffWestRx4Led = Imm_4LedsDelayTime;      
		  refreshLeds = 1;		    			  
			WestRxPreviousData = 0x5400 | 21 * 3;		  
	    break;                                  
                                        
	case 0xC2://(R)  //0xC2	WestRx  Main_Cube's WestTx          
			clear_west_4_led();		                    
			clear_otherRxLed_if_same_tx(0xC200, 4 * 3, 34 * 3, 44 * 3, 14 * 3);			
		  led_strip_pixels_ShowRxData[14 * 3 + 0] = 0x08;
		  led_strip_pixels_ShowRxData[14 * 3 + 1] = 0x00;		
		  led_strip_pixels_ShowRxData[14 * 3 + 2] = 0x00;				            	
		  delay1000msAndTurnOffWestRx4Led = Imm_4LedsDelayTime;      
		  refreshLeds = 1;		    			  
			WestRxPreviousData = 0xC200 | 14 * 3;		  
	    break;                                 
	case 0xA2://(G)  //0xA2	WestRx  Sub_Cube1's WestTx          
			clear_west_4_led();		                    
			clear_otherRxLed_if_same_tx(0xA200, 4 * 3, 34 * 3, 44 * 3, 14 * 3);			
		  led_strip_pixels_ShowRxData[14 * 3 + 0] = 0x00;
		  led_strip_pixels_ShowRxData[14 * 3 + 1] = 0x08;		
		  led_strip_pixels_ShowRxData[14 * 3 + 2] = 0x00;				            	
		  delay1000msAndTurnOffWestRx4Led = Imm_4LedsDelayTime;      
		  refreshLeds = 1;		    			  
			WestRxPreviousData = 0xA200 | 14 * 3;		  
	    break;                                 
	case 0x92://(B)  //0x92	WestRx  Sub_Cube2's WestTx        
			clear_west_4_led();		                    
			clear_otherRxLed_if_same_tx(0x9200, 4 * 3, 34 * 3, 44 * 3, 14 * 3);			
		  led_strip_pixels_ShowRxData[14 * 3 + 0] = 0x00;
		  led_strip_pixels_ShowRxData[14 * 3 + 1] = 0x00;		
		  led_strip_pixels_ShowRxData[14 * 3 + 2] = 0x08;				            	
		  delay1000msAndTurnOffWestRx4Led = Imm_4LedsDelayTime;      
		  refreshLeds = 1;		    			  
			WestRxPreviousData = 0x9200 | 14 * 3;		  
	    break;                                 
	case 0x62://(RG) //0x62	WestRx  Sub_Cube3's WestTx           
			clear_west_4_led();		                    
			clear_otherRxLed_if_same_tx(0x6200, 4 * 3, 34 * 3, 44 * 3, 14 * 3);			
		  led_strip_pixels_ShowRxData[14 * 3 + 0] = 0x08;
		  led_strip_pixels_ShowRxData[14 * 3 + 1] = 0x08;		
		  led_strip_pixels_ShowRxData[14 * 3 + 2] = 0x00;				            	
		  delay1000msAndTurnOffWestRx4Led = Imm_4LedsDelayTime;      
		  refreshLeds = 1;		    			  
			WestRxPreviousData = 0x6200 | 14 * 3;		  
	    break;                                 
	case 0x52://(RGB)//0x52	WestRx  Sub_Cube4's WestTx	            
			clear_west_4_led();		                    
			clear_otherRxLed_if_same_tx(0x5200, 4 * 3, 34 * 3, 44 * 3, 14 * 3);			
		  led_strip_pixels_ShowRxData[14 * 3 + 0] = 0x08;
		  led_strip_pixels_ShowRxData[14 * 3 + 1] = 0x08;		
		  led_strip_pixels_ShowRxData[14 * 3 + 2] = 0x08;				            	
		  delay1000msAndTurnOffWestRx4Led = Imm_4LedsDelayTime;      
		  refreshLeds = 1;		    			  
			WestRxPreviousData = 0x5200 | 14 * 3;		  
	    break;                                  
	                    
	default:
	    break;
	}	
	if(refreshLeds > 0)
	{//refresh West leds
		refreshLeds = 0;
    ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels_ShowRxData, sizeof(led_strip_pixels_ShowRxData), &tx_config));
    ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));		
	}		
}
//-------
//-------test ir detection--

//-------Timer---------------
/**
 * @brief A sample structure to pass events from the timer ISR to task
 */
typedef struct {
    uint64_t timer_count_value;
} example_timer_event_t;

/**
 * @brief Timer user data, will be pass to timer alarm callback
 */
typedef struct {
    QueueHandle_t user_queue;
    int timer_group;
    int timer_idx;
    int alarm_value;
    bool auto_reload;
} example_timer_user_data_t;

static bool IRAM_ATTR timer_group_isr_callback(void *args)
{
    BaseType_t high_task_awoken = pdFALSE;

    /*
    example_timer_user_data_t *user_data = (example_timer_user_data_t *) args;
    // fetch current count value
    uint64_t timer_count_value = timer_group_get_counter_value_in_isr(user_data->timer_group, user_data->timer_idx);
    example_timer_event_t evt = {
        .timer_count_value = timer_count_value,
    };
    */
    

		unsigned int 	timer_100us_temp=0;
		
		//-------Ir NorthRx----------
		timer_100us_temp = north_rx_per_per100us();
    if(timer_100us_temp != 0)
    {
    	newNorthRxData = timer_100us_temp;
    }	
		//-------Ir NorthRx----------
		    
		//-------Ir EastRx----------
		timer_100us_temp = east_rx_per_per100us();
    if(timer_100us_temp != 0)
    {
    	newEastRxData = timer_100us_temp;
    }	
		//-------Ir EastRx---------- 

		//-------Ir SouthRx----------
		timer_100us_temp = south_rx_per_per100us();
    if(timer_100us_temp != 0)
    {
    	newSouthRxData = timer_100us_temp;    	
    }	
		//-------Ir SouthRx----------

		//-------Ir WestRx----------
		timer_100us_temp = west_rx_per_per100us();
    if(timer_100us_temp != 0)
    {
    	newWestRxData = timer_100us_temp;
    }	
		//-------Ir WestRx----------

		timer_100us_temp = read_cube_position_per100us();
    if(timer_100us_temp != 0)
    {
    	cubeCurrentPosition = timer_100us_temp;    
    }
    
		timer_100us_temp = read_cube_changed_position_flag_per100us();
    if(timer_100us_temp != 0)
    {
    	cubeChangedPositionHp = timer_100us_temp;
    }    
    
		timer_100us_temp = read_shake_happen_flag_per100us();
    if(timer_100us_temp != 0)
    {
    	cubeShakeHp = timer_100us_temp;    
    }	
		
		timer_100us_temp = read_VolUpSwOn_flag_per100us();
    if(timer_100us_temp != 0)
    {
    	port1VolUpSwOnHp = timer_100us_temp;    
    }	
    
		timer_100us_temp = read_VolDwSwOn_flag_per100us();
    if(timer_100us_temp != 0)
    {
    	port1VolDwSwOnHp = timer_100us_temp;    
    }	
    
		timer_100us_temp = read_btn2SwOn_flag_per100us();
    if(timer_100us_temp != 0)
    {
    	port1btn2SwOnHp = timer_100us_temp;    
    }	
    
		timer_100us_temp = read_btn3SwOn_flag_per100us();
    if(timer_100us_temp != 0)
    {
    	port1btn3SwOnHp = timer_100us_temp;    
    }	
    
		timer_100us_temp = read_btn4SwOn_flag_per100us();
    if(timer_100us_temp != 0)
    {
    	port1btn4SwOnHp = timer_100us_temp;    
    }	
    
		timer_100us_temp = read_VusbSwOn_flag_per100us();
    if(timer_100us_temp != 0)
    {
    	port1VusbSwOnHp = timer_100us_temp;    
    }
    
		timer_100us_temp = read_VusbSwOff_flag_per100us();
    if(timer_100us_temp != 0)
    {
    	port1VusbSwOffHp = timer_100us_temp;    
    }	            
    
		

	//-------test ir detection--
	if(delay1000msAndTurnOffNorthRx4Led > 1)
	{
		delay1000msAndTurnOffNorthRx4Led--;
	}	
	if(delay1000msAndTurnOffEastRx4Led > 1)
	{
		delay1000msAndTurnOffEastRx4Led--;
	}	
	if(delay1000msAndTurnOffSouthRx4Led > 1)
	{
		delay1000msAndTurnOffSouthRx4Led--;
	}	
	if(delay1000msAndTurnOffWestRx4Led > 1)
	{
		delay1000msAndTurnOffWestRx4Led--;
	}							
	//-------test ir detection--

    //time_count++;
    //gpio_set_level(GPIO_OUTPUT_IO_0, time_count % 2);

    /*
    // set new alarm value if necessary
    if (!user_data->auto_reload) {
        user_data->alarm_value += TIMER_ALARM_PERIOD_S * TIMER_RESOLUTION_HZ;
        timer_group_set_alarm_value_in_isr(user_data->timer_group, user_data->timer_idx, user_data->alarm_value);
    }

    // Send the event data back to the main program task
    xQueueSendFromISR(user_data->user_queue, &evt, &high_task_awoken);
    */
    
    return high_task_awoken == pdTRUE; // return whether a task switch is needed
}

static void example_tg_timer_init(example_timer_user_data_t *user_data)
{
    int group = user_data->timer_group;
    int timer = user_data->timer_idx;

    uint32_t clk_src_hz = 0;
    ESP_ERROR_CHECK(esp_clk_tree_src_get_freq_hz((soc_module_clk_t)TIMER_SRC_CLK_DEFAULT, ESP_CLK_TREE_SRC_FREQ_PRECISION_CACHED, &clk_src_hz));
    timer_config_t config = {
        .clk_src = TIMER_SRC_CLK_DEFAULT,
        .divider = clk_src_hz / TIMER_RESOLUTION_HZ,
        .counter_dir = TIMER_COUNT_UP,
        .counter_en = TIMER_PAUSE,
        .alarm_en = TIMER_ALARM_EN,
        .auto_reload = user_data->auto_reload,
    };
    ESP_ERROR_CHECK(timer_init(group, timer, &config));

    // For the timer counter to a initial value
    ESP_ERROR_CHECK(timer_set_counter_value(group, timer, 0));
    // Set alarm value and enable alarm interrupt
    ESP_ERROR_CHECK(timer_set_alarm_value(group, timer, user_data->alarm_value));
    ESP_ERROR_CHECK(timer_enable_intr(group, timer));
    // Hook interrupt callback
    ESP_ERROR_CHECK(timer_isr_callback_add(group, timer, timer_group_isr_callback, user_data, 0));
    // Start timer
    ESP_ERROR_CHECK(timer_start(group, timer));
}
//-------Timer---------------

//-------wdt-----------------
//用户任务的回调函数
void reset_task(void *arg)
{
    //将任务添加到任务看门狗，并检查是否已添加
    CHECK_ERROR_CODE(esp_task_wdt_add(NULL), ESP_OK);
    CHECK_ERROR_CODE(esp_task_wdt_status(NULL), ESP_OK);
    while(1){
        //每2秒重置一次看门狗
        CHECK_ERROR_CODE(esp_task_wdt_reset(), ESP_OK);  //注释这一行可以触发看门狗超时
        vTaskDelay(3000 / portTICK_PERIOD_MS);
    }
 }
//-------wdt-----------------

//-------i2s speaker---------
typedef struct 
{
    //   RIFF Section    
    char RIFFSectionID[4];      // Letters "RIFF"
    uint32_t Size;              // Size of entire file less 8
    char RiffFormat[4];         // Letters "WAVE"
    
    //   Format Section    
    char FormatSectionID[4];    // letters "fmt"
    uint32_t FormatSize;        // Size of format section less 8
    uint16_t FormatID;          // 1=uncompressed PCM
    uint16_t NumChannels;       // 1=mono,2=stereo
    uint32_t SampleRate;        // 44100, 16000, 8000 etc.
    uint32_t ByteRate;          // =SampleRate * Channels * (BitsPerSample/8)
    uint16_t BlockAlign;        // =Channels * (BitsPerSample/8)
    uint16_t BitsPerSample;     // 8,16,24 or 32
    
    // Data Section
    char DataSectionID[4];      // The letters "data"
    uint32_t DataSize;          // Size of the data that follows
}WavHeader_Struct;
WavHeader_Struct WavHeader;
bool ValidWavData(WavHeader_Struct* Wav);
void sound_terminate(void);
//-------i2s speaker---------

//-------SD card-------------
static esp_err_t s_example_write_file(const char *path, char *data)
{
    ESP_LOGI(TAG, "Opening file %s", path);
    FILE *f = fopen(path, "w");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file for writing");
        return ESP_FAIL;
    }
    fprintf(f, data);
    fclose(f);
    ESP_LOGI(TAG, "File written");

    return ESP_OK;
}
//---------
static esp_err_t s_example_read_file(const char *path)
{
    ESP_LOGI(TAG, "Reading file %s", path);
    FILE *f = fopen(path, "r");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file for reading");
        return ESP_FAIL;
    }
    char line[EXAMPLE_MAX_CHAR_SIZE];
    fgets(line, sizeof(line), f);
    fclose(f);

    // strip newline
    char *pos = strchr(line, '\n');
    if (pos) {
        *pos = '\0';
    }
    ESP_LOGI(TAG, "Read from file: '%s'", line);

    return ESP_OK;
}
//---------
static esp_err_t s_example_read_ws2812b_bin_file(const char *path)
{
    ESP_LOGI(TAG, "Reading file %s", path);
    FILE *f = fopen(path, "r");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file for reading");
        return ESP_FAIL;
    }
    fread(led_strip_pixels, sizeof(uint8_t), sizeof(led_strip_pixels), f);        
    fclose(f);

    //debug ESP_LOGI(TAG, "Showing file %s", path);
    
    //int i;
    //for(i =0; i < EXAMPLE_LED_NUMBERS * 3; i++){
    //    printf("%x ", led_strip_pixels[i]);
    //}
    //printf("\n");    

    return ESP_OK;
}
//---------
//void SetAndWaitPlaySoundFile(void)
void SetAndWaitPlaySoundFile(const char * SoundName)
{
    FILE* fname;    
    fname = fopen(SoundName, "rb");
    //fname = fopen(MOUNT_POINT"/Sound1.wav", "rb");
    if (fname == NULL) 
    {
        //printf("Can not open sound file\r\n");    
        ESP_LOGI(TAG, "Can not open sound file");        
    } 
    else 
    {
        //printf("Can open and play sound file\r\n");    
        //ESP_LOGI(TAG, "Can open and play sound file");
        ESP_LOGI(TAG, "Opening file %s", SoundName);

        // 读取wav文件信息
        fseek(fname, 0, SEEK_SET);  // 重新将指针指向文件首部
        fread(wavBuffer, sizeof(char), 44, fname);
        memcpy(&WavHeader, wavBuffer, 44);
        if(ValidWavData(&WavHeader))
        {
            i2s_set_sample_rates(I2S_NUM_0, WavHeader.SampleRate);
        }
        else
        {
            ESP_LOGI(TAG, "Failed to read wav file");
            //printf("Failed to read Sound2.wav file\r\n");    
        }
        uint32_t wavData_size = WavHeader.DataSize; // 保存文件字符数
        uint16_t readTimes = 0;                     // 需要读的次数
        size_t BytesWritten;

        for (readTimes = 0; readTimes < (wavData_size / 1024); readTimes++)                                 //1024 
        {
            fread(wavBuffer, sizeof(char), 1024, fname); // 读文件                                          //1024         
            i2s_write(I2S_NUM_0, wavBuffer, 1024, &BytesWritten, portMAX_DELAY);                           //1024   
            //vTaskDelay(5 / portTICK_PERIOD_MS);//vTaskDelay(pdMS_TO_TICKS(200)); //延时200ms
        }

        memset(wavBuffer, 0, 1024);                               // 清空，准备读少于1024的字节             //1024 
        fread(wavBuffer, sizeof(char), (wavData_size % 1024), fname); // 读文件                             //1024 

        i2s_write(I2S_NUM_0, wavBuffer, (wavData_size % 1024) + 20, &BytesWritten, portMAX_DELAY);          //1024  //20

        //bobi   printf("readtimes=%d,last read len = %d\r\n", readTimes, (wavData_size % 1024));           //1024 

        //printf("sound play over\r\n");
        //ESP_LOGI(TAG, "sound play over 1");
        sound_terminate(); //清空缓存      
        ESP_LOGI(TAG, "sound play over 2");        
    }    
    // 关闭文件
    fclose(fname);    
}
//-------SD card-------------

//-------WS2812b-------------
/**
 * @brief Simple helper function, converting HSV color space to RGB color space
 *
 * Wiki: https://en.wikipedia.org/wiki/HSL_and_HSV
 *
 */
void led_strip_hsv2rgb(uint32_t h, uint32_t s, uint32_t v, uint32_t *r, uint32_t *g, uint32_t *b)
{
    h %= 360; // h -> [0,360]
    uint32_t rgb_max = v * 2.55f;
    uint32_t rgb_min = rgb_max * (100 - s) / 100.0f;

    uint32_t i = h / 60;
    uint32_t diff = h % 60;

    // RGB adjustment amount by hue
    uint32_t rgb_adj = (rgb_max - rgb_min) * diff / 60;

    switch (i) {
    case 0:
        *r = rgb_max;
        *g = rgb_min + rgb_adj;
        *b = rgb_min;
        break;
    case 1:
        *r = rgb_max - rgb_adj;
        *g = rgb_max;
        *b = rgb_min;
        break;
    case 2:
        *r = rgb_min;
        *g = rgb_max;
        *b = rgb_min + rgb_adj;
        break;
    case 3:
        *r = rgb_min;
        *g = rgb_max - rgb_adj;
        *b = rgb_max;
        break;
    case 4:
        *r = rgb_min + rgb_adj;
        *g = rgb_min;
        *b = rgb_max;
        break;
    default:
        *r = rgb_max;
        *g = rgb_min;
        *b = rgb_max - rgb_adj;
        break;
    }
}
//-------WS2812b-------------

//-------WS2812b-------------
bool ValidWavData(WavHeader_Struct* Wav)
{
    if(memcmp(Wav->RIFFSectionID,"RIFF",4)!=0) 
    {    
        printf("Invlaid data - Not RIFF format");
        return false;        
    }
    if(memcmp(Wav->RiffFormat,"WAVE",4)!=0)
    {
        printf("Invlaid data - Not Wave file");
        return false;           
    }
    if(memcmp(Wav->FormatSectionID,"fmt",3)!=0) 
    {
        printf("Invlaid data - No format section found");
        return false;       
    }
    if(memcmp(Wav->DataSectionID,"data",4)!=0) 
    {
        printf("Invlaid data - data section not found");
        return false;      
    }
    if(Wav->FormatID!=1) 
    {
        printf("Invlaid data - format Id must be 1");
        return false;                          
    }
    if(Wav->FormatSize!=16) 
    {
        printf("Invlaid data - format section size must be 16.");
        return false;                          
    }
    if((Wav->NumChannels!=1)&(Wav->NumChannels!=2))
    {
        printf("Invlaid data - only mono or stereo permitted.");
        return false;   
    }
    if(Wav->SampleRate>48000) 
    {
        printf("Invlaid data - Sample rate cannot be greater than 48000");
        return false;                       
    }
    if((Wav->BitsPerSample!=8)& (Wav->BitsPerSample!=16)) 
    {
        printf("Invlaid data - Only 8 or 16 bits per sample permitted.");
        return false;                        
    }
    return true;
}
//-------
void sound_terminate(void)
{
    vTaskDelay(200 / portTICK_PERIOD_MS);//vTaskDelay(pdMS_TO_TICKS(200)); //延时200ms  //200
    i2s_zero_dma_buffer(I2S_NUM_0);   // Clean the DMA buffer
    //i2s_stop(I2S_NUM_0);
    //i2s_start(I2S_NUM_0);//增加测试
}
//-------WS2812b-------------


//-------adc-----------------
void adc1task(void* arg)
{
     // initialize ADC
     adc1_config_width(ADC_WIDTH_BIT_12);
     adc1_config_channel_atten(ADC1_TEST_CHANNEL,ADC_ATTEN_DB_11);
     while(1){
         //printf("The adc1 value:%d\n",adc1_get_voltage(ADC1_TEST_CHANNEL));
         printf("The adc1 value:%d\n",adc1_get_raw(ADC1_TEST_CHANNEL));
         vTaskDelay(1000/portTICK_PERIOD_MS); 
     }
}
//-------adc-----------------


//-------testPlaySound-------
void testPlaySound(void* arg)
{    	
	while(1)
	{
   	    if(needToPlayNewSound)
   	    {
            needToPlayNewSound = false;

    	    FILE* fname;    

    		//i2s_zero_dma_buffer(I2S_NUM_0);   // Clean the DMA buffer
            //i2s_start(I2S_NUM_0);

    	    //playSoundName = MOUNT_POINT"/Sound3.wav";
    	    fname = fopen(playSoundName, "rb");
    	    if (fname == NULL) 
    	    {
    	        //printf("Can not open sound file\r\n");    
    	        ESP_LOGI(TAG, "Can not open sound file");        
    	    } 
    	    else 
    	    {
    	        //printf("Can open and play sound file\r\n");    
    	        //ESP_LOGI(TAG, "Can open and play sound file");
    	        //ESP_LOGI(TAG, "Opening file %s", playSoundName);
                //i2s_set_volume(I2S_NUM_0, 0.5);
    	
    	        // 读取wav文件信息
    	        fseek(fname, 0, SEEK_SET);  // 重新将指针指向文件首部
    	        fread(wavBuffer, sizeof(char), 44, fname);
    	        memcpy(&WavHeader, wavBuffer, 44);
    	        if(ValidWavData(&WavHeader))
    	        {
    	            i2s_set_sample_rates(I2S_NUM_0, WavHeader.SampleRate);
    	        }
    	        else
    	        {
    	            ESP_LOGI(TAG, "Failed to read wav file");
    	            //printf("Failed to read Sound2.wav file\r\n");    
    	        }
    	        uint32_t wavData_size = WavHeader.DataSize; // 保存文件字符数
    	        uint16_t readTimes = 0;                     // 需要读的次数
    	        size_t BytesWritten;
    	
    	        for (readTimes = 0; readTimes < (wavData_size / 1024); readTimes++)                                 //1024 
    	        {
    	            fread(wavBuffer, sizeof(char), 1024, fname); // 读文件   
    	            
    	            
  					//-----volume set-----
					/*Array used for converting two bytes in an unsigned int*/
					unsigned char uintBytes[2];
  						
					/*The unsigned int obtained*/
					uint16_t * conv;
  						
					/*The new value calculated*/
					uint16_t nuovoValore;
  					int16_t int16_nuovoValore;

                    //float fvolume = 0.2;
  						
  					int k;								
					for(k=0; k<1024;  k += 2)
  					{									
					    /*I read 2 bytes form the array and "convert" it in an unsigned int*/
						uintBytes[0]=wavBuffer[k];
						uintBytes[1]=wavBuffer[k+1];
							
						conv=(uint16_t *) &uintBytes[0];
							
						/*Calculate the new value (-30%) to write in the new file*/								
						//nuovoValore= *conv - ((float)*conv*50/100);
						//          if(nuovoValore<0) nuovoValore=0;
  						nuovoValore= *conv;  						
  						int16_nuovoValore = (int16_t)nuovoValore;
                        //int16_nuovoValore = int16_nuovoValore * fvolume;//0.2;
                        if(volumeSet == 0)
                            int16_nuovoValore = int16_nuovoValore * 0.0;
                        else if(volumeSet == 1)
                            int16_nuovoValore = int16_nuovoValore * 0.25;
                        else if(volumeSet == 2)
                            int16_nuovoValore = int16_nuovoValore * 0.5;
                        else if(volumeSet == 3)
                            int16_nuovoValore = int16_nuovoValore * 0.75;
                        else if(volumeSet == 4)
                            int16_nuovoValore = int16_nuovoValore * 1.0;

  						nuovoValore = int16_nuovoValore;
  						//nuovoValore=nuovoValore*1;
											
						wavBuffer[k]=nuovoValore & 0x00FF;
											
						nuovoValore=nuovoValore >> 8;
						wavBuffer[k+1]=nuovoValore & 0x00FF;																																							
					}									
					//-----volume set-----

    	            i2s_write(I2S_NUM_0, wavBuffer, 1024, &BytesWritten, portMAX_DELAY);                           //1024   
    	            //vTaskDelay(5 / portTICK_PERIOD_MS);//vTaskDelay(pdMS_TO_TICKS(200)); //延时200ms    	        
    	            if(needToPlayNewSound)
    	            {
    	        	    readTimes = wavData_size / 1024;
    	            }	
    	        }
    	
    	        memset(wavBuffer, 0, 1024);                               // 清空，准备读少于1024的字节             //1024 
    	        fread(wavBuffer, sizeof(char), (wavData_size % 1024), fname); // 读文件                             //1024 
    	
    	        i2s_write(I2S_NUM_0, wavBuffer, (wavData_size % 1024) + 20, &BytesWritten, portMAX_DELAY);          //1024  //20
    	
    	        //bobi   printf("readtimes=%d,last read len = %d\r\n", readTimes, (wavData_size % 1024));           //1024 
    	
    	        //printf("sound play over\r\n");
    	        //ESP_LOGI(TAG, "sound play over 1");
                int i;                
                for (i = 0; i < 50; i++)
                {
                    vTaskDelay(1 / portTICK_PERIOD_MS);//vTaskDelay(pdMS_TO_TICKS(200)); //延时200ms  //200            
    	            if(needToPlayNewSound)
    	            {
    	        	    i = 2000;
    	            }                    
                }
    			
    			i2s_zero_dma_buffer(I2S_NUM_0);   // Clean the DMA buffer
    			i2s_stop(I2S_NUM_0);
    			//i2s_start(I2S_NUM_0);//增加测试
    	    
    	        ESP_LOGI(TAG, "sound play over 2");        
    	    }    
    	    // 关闭文件
    	    fclose(fname);   		   		   		
   	    }    			
	}
}
//-------testPlaySound-------


void app_main(void)
{
    //-------WS2812b-------------
    //uint32_t red = 0;
    //uint32_t green = 0;
    //uint32_t blue = 0;
    //uint16_t hue = 0;
    //uint16_t start_rgb = 0;
    //-------WS2812b-------------

    //-------generic_gpio--------
    //zero-initialize the config structure.
    gpio_config_t io_conf = {};
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);

    gpio_set_level(GPIO_OUTPUT_IO_0, 1);
    
    //interrupt of rising edge
    io_conf.intr_type = GPIO_INTR_DISABLE;//GPIO_INTR_POSEDGE
    //bit mask of the pins, use GPIO4/5 here
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    //set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
//    //enable pull-up mode
//    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);
    //-------generic_gpio--------


    //-------adc-----------------
    /*
    //---ADC1 Init---
    adc_oneshot_unit_handle_t adc1_handle;
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };    
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    //---ADC1 Config---
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = EXAMPLE_ADC_ATTEN,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, EXAMPLE_ADC1_CHAN0, &config));

    //---ADC1 Calibration Init---
    adc_cali_handle_t adc1_cali_chan0_handle = NULL;

    bool do_calibration1_chan0 = example_adc_calibration_init(ADC_UNIT_1, EXAMPLE_ADC1_CHAN0, EXAMPLE_ADC_ATTEN, &adc1_cali_chan0_handle);    

    while (1) {
        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, EXAMPLE_ADC1_CHAN0, &adc_raw[0][0]));
        ESP_LOGI(TAG, "ADC%d Channel[%d] Raw Data: %d", ADC_UNIT_1 + 1, EXAMPLE_ADC1_CHAN0, adc_raw[0][0]);
        if (do_calibration1_chan0) {
            ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_chan0_handle, adc_raw[0][0], &voltage[0][0]));
            ESP_LOGI(TAG, "ADC%d Channel[%d] Cali Voltage: %d mV", ADC_UNIT_1 + 1, EXAMPLE_ADC1_CHAN0, voltage[0][0]);
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    */
    //-------adc-----------------

    //-------i2s speaker---------
    // 初始化I2S引脚
    i2s_config_t i2s_config = {
        .mode = I2S_MODE_MASTER | I2S_MODE_TX,              // 使用主模式并设置为发送数据
        .sample_rate = 44100,                               // 设置采样率为44100Hz
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,       // 设置每个采样点的位数为16位
        .channel_format = I2S_CHANNEL_FMT_ONLY_RIGHT,       // 只使用右声道                       //I2S_CHANNEL_FMT_ONLY_RIGHT
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,  // I2S通信格式
        .dma_buf_count = 10,                                 // 设置DMA缓冲区数量为8                //10            //8
        .dma_buf_len = 256,//1024,                                // 每个DMA缓冲区的长度为1024字节  //1024          //1024
        .intr_alloc_flags = ESP_INTR_FLAG_EDGE,             // 分配中断标志
    };
    i2s_pin_config_t pin_config = {
        .bck_io_num = GPIO_NUM_27,   // BCLK引脚号
        .ws_io_num = GPIO_NUM_26,    // LRCK引脚号
        .data_out_num = GPIO_NUM_14, // DATA引脚号
        .data_in_num = -1,           // DATA_IN引脚号
    };
    i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
    i2s_set_pin(I2S_NUM_0, &pin_config);
    //-------i2s speaker---------
    
    //-------SD card-------------
    esp_err_t ret;
    // Options for mounting the filesystem.
    // If format_if_mount_failed is set to true, SD card will be partitioned and
    // formatted in case when mounting fails.
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
#ifdef CONFIG_EXAMPLE_FORMAT_IF_MOUNT_FAILED
        .format_if_mount_failed = true,
#else
        .format_if_mount_failed = false,
#endif // EXAMPLE_FORMAT_IF_MOUNT_FAILED
        .max_files = 32, //32 //30 //5
        .allocation_unit_size = 2 * 1024 //16 * 1024
    };
    sdmmc_card_t *card;
    const char mount_point[] = MOUNT_POINT;
    ESP_LOGI(TAG, "Initializing SD card");

    // Use settings defined above to initialize SD card and mount FAT filesystem.
    // Note: esp_vfs_fat_sdmmc/sdspi_mount is all-in-one convenience functions.
    // Please check its source code and implement error recovery when developing
    // production applications.
    ESP_LOGI(TAG, "Using SPI peripheral");

    // By default, SD card frequency is initialized to SDMMC_FREQ_DEFAULT (20MHz)
    // For setting a specific frequency, use host.max_freq_khz (range 400kHz - 20MHz for SDSPI)
    // Example: for fixed frequency of 10MHz, use host.max_freq_khz = 10000;
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();

    spi_bus_config_t bus_cfg = {
        .mosi_io_num = PIN_NUM_MOSI,
        .miso_io_num = PIN_NUM_MISO,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000,
    };
    ret = spi_bus_initialize(host.slot, &bus_cfg, SDSPI_DEFAULT_DMA);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize bus.");
        return;
    }

    // This initializes the slot without card detect (CD) and write protect (WP) signals.
    // Modify slot_config.gpio_cd and slot_config.gpio_wp if your board has these signals.
    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = PIN_NUM_CS;
    slot_config.host_id = host.slot;

    ESP_LOGI(TAG, "Mounting filesystem");
    ret = esp_vfs_fat_sdspi_mount(mount_point, &host, &slot_config, &mount_config, &card);

    printf("App step1\r\n");   

    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount filesystem. "
                     "If you want the card to be formatted, set the CONFIG_EXAMPLE_FORMAT_IF_MOUNT_FAILED menuconfig option.");
        } else {
            ESP_LOGE(TAG, "Failed to initialize the card (%s). "
                     "Make sure SD card lines have pull-up resistors in place.", esp_err_to_name(ret));
        }
        return;
    }
    ESP_LOGI(TAG, "Filesystem mounted");

    // Card has been initialized, print its properties
    sdmmc_card_print_info(stdout, card);

    // Use POSIX and C standard library functions to work with files.

    // First create a file.
    const char *file_hello2 = MOUNT_POINT"/hello2.txt";
    char data[EXAMPLE_MAX_CHAR_SIZE];
    snprintf(data, EXAMPLE_MAX_CHAR_SIZE, "%s %s!\n", "Hello2", card->cid.name);
    ret = s_example_write_file(file_hello2, data);
    if (ret != ESP_OK) {
        return;
    }

    /*
    // First create a file.
    const char *file_hello3 = MOUNT_POINT"/hello3.txt";
    //char data[EXAMPLE_MAX_CHAR_SIZE];
    snprintf(data, EXAMPLE_MAX_CHAR_SIZE, "%s %s!\n", "Hello3", card->cid.name);
    ret = s_example_write_file(file_hello3, data);
    if (ret != ESP_OK) {
        return;
    }
    */

    /*
    // First create a file.
    const char *file_hello4 = MOUNT_POINT"/hello4.txt";
    //char data[EXAMPLE_MAX_CHAR_SIZE];
    snprintf(data, EXAMPLE_MAX_CHAR_SIZE, "%s %s!\n", "Hello4", card->cid.name);
    ret = s_example_write_file(file_hello4, data);
    if (ret != ESP_OK) {
        return;
    }
    */
    
    const char *file_Sound;

    file_Sound = MOUNT_POINT"/Sound1.wav";
    //char data[EXAMPLE_MAX_CHAR_SIZE];
    struct stat st;
    if (stat(file_Sound, &st) == 0) {
        // Delete it if it exists
        printf("Had Sound1.wav\r\n");   
    }
    else
    {
        printf("Noo Sound1.wav\r\n");    
    }    

    file_Sound = MOUNT_POINT"/Sound2.wav";
    //char data[EXAMPLE_MAX_CHAR_SIZE];    
    if (stat(file_Sound, &st) == 0) {
        // Delete it if it exists
        printf("Had Sound2.wav\r\n");   
    }
    else
    {
        printf("Noo Sound2.wav\r\n");    
    }    

    file_Sound = MOUNT_POINT"/Sound3.wav";
    //char data[EXAMPLE_MAX_CHAR_SIZE];    
    if (stat(file_Sound, &st) == 0) {
        // Delete it if it exists
        printf("Had Sound3.wav\r\n");   
    }
    else
    {
        printf("Noo Sound3.wav\r\n");    
    }

    file_Sound = MOUNT_POINT"/Sound4.wav";
    //char data[EXAMPLE_MAX_CHAR_SIZE];    
    if (stat(file_Sound, &st) == 0) {
        // Delete it if it exists
        printf("Had Sound4.wav\r\n");   
    }
    else
    {
        printf("Noo Sound4.wav\r\n");    
    }

    const char *file_Sound3 = MOUNT_POINT"/GTestOK.wav";
    //char data[EXAMPLE_MAX_CHAR_SIZE];    
    if (stat(file_Sound3, &st) == 0) {
        // Delete it if it exists
        printf("Had GTestOK.wav\r\n");   
    }
    else
    {
        printf("Noo GTestOK.wav\r\n");    
    }

    file_Sound = MOUNT_POINT"/A_blue.bin";
    //char data[EXAMPLE_MAX_CHAR_SIZE];    
    if (stat(file_Sound, &st) == 0) {
        // Delete it if it exists
        printf("Had A_blue.bin\r\n");   
    }
    else
    {
        printf("Noo A_blue.bin\r\n");    
    }

    file_Sound = MOUNT_POINT"/B_blue.bin";
    //char data[EXAMPLE_MAX_CHAR_SIZE];    
    if (stat(file_Sound, &st) == 0) {
        // Delete it if it exists
        printf("Had B_blue.bin\r\n");   
    }
    else
    {
        printf("No B_blue.bin\r\n");    
    }

    file_Sound = MOUNT_POINT"/C_blue.bin";
    //char data[EXAMPLE_MAX_CHAR_SIZE];    
    if (stat(file_Sound, &st) == 0) {
        // Delete it if it exists
        printf("Had C_blue.bin\r\n");   
    }
    else
    {
        printf("Noo C_blue.bin\r\n");    
    }

    file_Sound = MOUNT_POINT"/D_blue.bin";
    //char data[EXAMPLE_MAX_CHAR_SIZE];    
    if (stat(file_Sound, &st) == 0) {
        // Delete it if it exists
        printf("Had D_blue.bin\r\n");   
    }
    else
    {
        printf("No D_blue.bin\r\n");    
    }

    file_Sound = MOUNT_POINT"/E_blue.bin";
    //char data[EXAMPLE_MAX_CHAR_SIZE];    
    if (stat(file_Sound, &st) == 0) {
        // Delete it if it exists
        printf("Had E_blue.bin\r\n");   
    }
    else
    {
        printf("Noo E_blue.bin\r\n");    
    }

    file_Sound = MOUNT_POINT"/F_blue.bin";
    //char data[EXAMPLE_MAX_CHAR_SIZE];    
    if (stat(file_Sound, &st) == 0) {
        // Delete it if it exists
        printf("Had F_blue.bin\r\n");   
    }
    else
    {
        printf("No F_blue.bin\r\n");    
    }

    file_Sound = MOUNT_POINT"/G_blue.bin";
    //char data[EXAMPLE_MAX_CHAR_SIZE];    
    if (stat(file_Sound, &st) == 0) {
        // Delete it if it exists
        printf("Had G_blue.bin\r\n");   
    }
    else
    {
        printf("Noo G_blue.bin\r\n");    
    }

    file_Sound = MOUNT_POINT"/H_blue.bin";
    //char data[EXAMPLE_MAX_CHAR_SIZE];    
    if (stat(file_Sound, &st) == 0) {
        // Delete it if it exists
        printf("Had H_blue.bin\r\n");   
    }
    else
    {
        printf("No H_blue.bin\r\n");    
    }
    
    file_Sound = MOUNT_POINT"/J_blue.bin";
    //char data[EXAMPLE_MAX_CHAR_SIZE];    
    if (stat(file_Sound, &st) == 0) {
        // Delete it if it exists
        printf("Had J_blue.bin\r\n");   
    }
    else
    {
        printf("Noo J_blue.bin\r\n");    
    }

    file_Sound = MOUNT_POINT"/A_green.bin";
    //char data[EXAMPLE_MAX_CHAR_SIZE];    
    if (stat(file_Sound, &st) == 0) {
        // Delete it if it exists
        printf("Had A_green.bin\r\n");   
    }
    else
    {
        printf("No A_green.bin\r\n");    
    }    

    file_Sound = MOUNT_POINT"/0.bin";
    //char data[EXAMPLE_MAX_CHAR_SIZE];    
    if (stat(file_Sound, &st) == 0) {
        // Delete it if it exists
        printf("Had 0.bin\r\n");   
    }
    else
    {
        printf("Noo 0.bin\r\n");    
    }     

    file_Sound = MOUNT_POINT"/1.bin";
    //char data[EXAMPLE_MAX_CHAR_SIZE];    
    if (stat(file_Sound, &st) == 0) {
        // Delete it if it exists
        printf("Had 1.bin\r\n");   
    }
    else
    {
        printf("No 1.bin\r\n");    
    }     

    file_Sound = MOUNT_POINT"/2.bin";
    //char data[EXAMPLE_MAX_CHAR_SIZE];    
    if (stat(file_Sound, &st) == 0) {
        // Delete it if it exists
        printf("Had 2.bin\r\n");   
    }
    else
    {
        printf("Noo 2.bin\r\n");    
    }     

    file_Sound = MOUNT_POINT"/3.bin";
    //char data[EXAMPLE_MAX_CHAR_SIZE];    
    if (stat(file_Sound, &st) == 0) {
        // Delete it if it exists
        printf("Had 3.bin\r\n");   
    }
    else
    {
        printf("No 3.bin\r\n");    
    }   

    file_Sound = MOUNT_POINT"/4.bin";
    //char data[EXAMPLE_MAX_CHAR_SIZE];    
    if (stat(file_Sound, &st) == 0) {
        // Delete it if it exists
        printf("Had 4.bin\r\n");   
    }
    else
    {
        printf("Noo 4.bin\r\n");    
    }     

    file_Sound = MOUNT_POINT"/5.bin";
    //char data[EXAMPLE_MAX_CHAR_SIZE];    
    if (stat(file_Sound, &st) == 0) {
        // Delete it if it exists
        printf("Had 5.bin\r\n");   
    }
    else
    {
        printf("No 5.bin\r\n");    
    }     

    file_Sound = MOUNT_POINT"/6.bin";
    //char data[EXAMPLE_MAX_CHAR_SIZE];    
    if (stat(file_Sound, &st) == 0) {
        // Delete it if it exists
        printf("Had 6.bin\r\n");   
    }
    else
    {
        printf("Noo 6.bin\r\n");    
    }     

     file_Sound = MOUNT_POINT"/7.bin";
    //char data[EXAMPLE_MAX_CHAR_SIZE];    
    if (stat(file_Sound, &st) == 0) {
        // Delete it if it exists
        printf("Had 7.bin\r\n");   
    }
    else
    {
        printf("No 7.bin\r\n");    
    }     

     file_Sound = MOUNT_POINT"/8.bin";
    //char data[EXAMPLE_MAX_CHAR_SIZE];    
    if (stat(file_Sound, &st) == 0) {
        // Delete it if it exists
        printf("Had 8.bin\r\n");   
    }
    else
    {
        printf("Noo 8.bin\r\n");    
    }     

    file_Sound = MOUNT_POINT"/9.bin";
    //char data[EXAMPLE_MAX_CHAR_SIZE];    
    if (stat(file_Sound, &st) == 0) {
        // Delete it if it exists
        printf("Had 9.bin\r\n");   
    }
    else
    {
        printf("No 9.bin\r\n");    
    }     

     file_Sound = MOUNT_POINT"/10.bin";
    //char data[EXAMPLE_MAX_CHAR_SIZE];    
    if (stat(file_Sound, &st) == 0) {
        // Delete it if it exists
        printf("Had 10.bin\r\n");   
    }
    else
    {
        printf("Noo 10.bin\r\n");    
    }                          

     file_Sound = MOUNT_POINT"/0blue.bin";
    //char data[EXAMPLE_MAX_CHAR_SIZE];    
    if (stat(file_Sound, &st) == 0) {
        // Delete it if it exists
        printf("Had 0blue.bin\r\n");   
    }
    else
    {
        printf("Noo 0blue.bin\r\n");    
    }

     file_Sound = MOUNT_POINT"/0yellow.bin";
    //char data[EXAMPLE_MAX_CHAR_SIZE];    
    if (stat(file_Sound, &st) == 0) {
        // Delete it if it exists
        printf("Had 0yellow.bin\r\n");   
    }
    else
    {
        printf("Noo 0yellow.bin\r\n");    
    }

     file_Sound = MOUNT_POINT"/9blue.bin";
    //char data[EXAMPLE_MAX_CHAR_SIZE];    
    if (stat(file_Sound, &st) == 0) {
        // Delete it if it exists
        printf("Had 9blue.bin\r\n");   
    }
    else
    {
        printf("Noo 9blue.bin\r\n");    
    }

     file_Sound = MOUNT_POINT"/9yellow.bin";
    //char data[EXAMPLE_MAX_CHAR_SIZE];    
    if (stat(file_Sound, &st) == 0) {
        // Delete it if it exists
        printf("Had 9yellow.bin\r\n");   
    }
    else
    {
        printf("Noo 9yellow.bin\r\n");    
    }

     file_Sound = MOUNT_POINT"/20green.bin";
    //char data[EXAMPLE_MAX_CHAR_SIZE];    
    if (stat(file_Sound, &st) == 0) {
        // Delete it if it exists
        printf("Had 20green.bin\r\n");   
    }
    else
    {
        printf("Noo 20green.bin\r\n");    
    }

     file_Sound = MOUNT_POINT"/20yellow.bin";
    //char data[EXAMPLE_MAX_CHAR_SIZE];    
    if (stat(file_Sound, &st) == 0) {
        // Delete it if it exists
        printf("Had 20yellow.bin\r\n");   
    }
    else
    {
        printf("Noo 20yellow.bin\r\n");    
    }

    FILE* f;    

    // 打开文件进行读取
    const char *Sound_Name = MOUNT_POINT"/Sound1.wav";
    f = fopen(Sound_Name, "rb");
    //f = fopen(MOUNT_POINT"/Sound1.wav", "rb");
    if (f == NULL) 
    {
        printf("Can not open Sound1.wav\r\n");    
    } 
    else 
    {
        printf("Can open Sound1.wav\r\n");    
    }    
    // 关闭文件
    fclose(f);

    //Sound_Name = MOUNT_POINT"/Sound1.wav";
    //SetAndWaitPlaySoundFile(Sound_Name);    

/* //debug
    Sound_Name = MOUNT_POINT"/Sound1.wav";
    SetAndWaitPlaySoundFile(Sound_Name);

    Sound_Name = MOUNT_POINT"/Sound2.wav";
    SetAndWaitPlaySoundFile(Sound_Name);

    Sound_Name = MOUNT_POINT"/Sound3.wav";
    SetAndWaitPlaySoundFile(Sound_Name);

    Sound_Name = MOUNT_POINT"/Sound4.wav";
    SetAndWaitPlaySoundFile(Sound_Name);
*/
    //Sound_Name = MOUNT_POINT"/Sound5.wav";
    //SetAndWaitPlaySoundFile(Sound_Name);

    //Sound_Name = MOUNT_POINT"/Sound6.wav";
    //SetAndWaitPlaySoundFile(Sound_Name);

    //Sound_Name = MOUNT_POINT"/Sound7.wav";
    //SetAndWaitPlaySoundFile(Sound_Name);

    //Sound_Name = MOUNT_POINT"/Sound8.wav";
    //SetAndWaitPlaySoundFile(Sound_Name);

    //Sound_Name = MOUNT_POINT"/Sound9.wav";
    //SetAndWaitPlaySoundFile(Sound_Name);

    //Sound_Name = MOUNT_POINT"/Sound10.wav";
    //SetAndWaitPlaySoundFile(Sound_Name);

    //Sound_Name = MOUNT_POINT"/Sound11.wav";
    //SetAndWaitPlaySoundFile(Sound_Name);

    //Sound_Name = MOUNT_POINT"/Sound12.wav";
    //SetAndWaitPlaySoundFile(Sound_Name);

    //Sound_Name = MOUNT_POINT"/Sound13.wav";
    //SetAndWaitPlaySoundFile(Sound_Name);
    /*
    //char SoundName[EXAMPLE_MAX_CHAR_SIZE];
    //SoundName[]="/Sound2.wav";
    // 打开文件进行读取
    Sound_Name = MOUNT_POINT"/Sound3.wav";
    f = fopen(Sound_Name, "rb");
    //f = fopen(MOUNT_POINT"/Sound3.wav", "rb");
    if (f == NULL) 
    {
        printf("Can not open Sound3.wav\r\n");    
    } 
    else 
    {
        printf("Can open Sound3.wav\r\n");    

        //char wavBuffer[1024];   // 接收缓冲区

        // 读取wav文件信息
        fseek(f, 0, SEEK_SET);  // 重新将指针指向文件首部
        fread(wavBuffer, sizeof(char), 44, f);
        memcpy(&WavHeader, wavBuffer, 44);
        if(ValidWavData(&WavHeader))
        {
            i2s_set_sample_rates(I2S_NUM_0, WavHeader.SampleRate);
        }
        else
        {
            //ESP_LOGI("AUDIO", "Failed to read wav file");
            printf("Failed to read Sound2.wav file\r\n");    
        }
        uint32_t wavData_size = WavHeader.DataSize; // 保存文件字符数
        uint16_t readTimes = 0;                     // 需要读的次数
        size_t BytesWritten;

        for (readTimes = 0; readTimes < (wavData_size / 1024); readTimes++)
        {
            fread(wavBuffer, sizeof(char), 1024, f); // 读文件
            i2s_write(I2S_NUM_0, wavBuffer, 1024, &BytesWritten, portMAX_DELAY);
        }

        memset(wavBuffer, 0, 1024);                               // 清空，准备读少于1024的字节
        fread(wavBuffer, sizeof(char), (wavData_size % 1024), f); // 读文件

        i2s_write(I2S_NUM_0, wavBuffer, (wavData_size % 1024) + 20, &BytesWritten, portMAX_DELAY);

        //bobi   printf("readtimes=%d,last read len = %d\r\n", readTimes, (wavData_size % 1024));

        printf("sound play over\r\n");
        sound_terminate(); //清空缓存      
    }    
    // 关闭文件
    fclose(f);
    */

    // All done, unmount partition and disable SPI peripheral
    //esp_vfs_fat_sdcard_unmount(mount_point, card);
    //ESP_LOGI(TAG, "Card unmounted");

    //deinitialize the bus after all devices are removed
    //spi_bus_free(host.slot);
    //-------SD card-------------
    
    //-------wdt-----------------
    /*
    printf("Initialize TWDT\n");
    //初始化和重新初始化任务看门狗
    //CHECK_ERROR_CODE(esp_task_wdt_init(TWDT_TIMEOUT_S, false), ESP_OK);
    CHECK_ERROR_CODE(esp_task_wdt_init(TWDT_TIMEOUT_S), ESP_OK);

    //启动时未订阅空闲任务，则订阅到任务看门狗
    #ifndef CONFIG_ESP_TASK_WDT_CHECK_IDLE_TASK_CPU0
        esp_task_wdt_add(xTaskGetIdleTaskHandleForCPU(0));
    #endif
    #ifndef CONFIG_ESP_TASK_WDT_CHECK_IDLE_TASK_CPU1
        esp_task_wdt_add(xTaskGetIdleTaskHandleForCPU(1));
    #endif

    //创建用户任务并添加到任务看门狗
    for(int i = 0; i < portNUM_PROCESSORS; i++){
        xTaskCreatePinnedToCore(reset_task, "reset task", 1024, NULL, 10, &task_handles[i], i);
    }    
    */
    //-------wdt-----------------

    //-------WS2812b-------------
    ESP_LOGI(TAG, "Create RMT TX channel");
    //rmt_channel_handle_t led_chan = NULL;
    rmt_tx_channel_config_t tx_chan_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT, // select source clock
        .gpio_num = RMT_LED_STRIP_GPIO_NUM,
        .mem_block_symbols = 128, //64 // increase the block size can make the LED less flickering,default 64 is not enough
        .resolution_hz = RMT_LED_STRIP_RESOLUTION_HZ,
        .trans_queue_depth = 8, //4 // set the number of transactions that can be pending in the background
        //.flags.with_dma = true, ///set, the driver will allocate an RMT channel with DMA capability */ //The rmt has no DMA on ESP32.
    };
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config, &led_chan));

    ESP_LOGI(TAG, "Install led strip encoder");
    //rmt_encoder_handle_t led_encoder = NULL;
    led_strip_encoder_config_t encoder_config = {
        .resolution = RMT_LED_STRIP_RESOLUTION_HZ,
    };
    ESP_ERROR_CHECK(rmt_new_led_strip_encoder(&encoder_config, &led_encoder));

    ESP_LOGI(TAG, "Enable RMT TX channel");
    ESP_ERROR_CHECK(rmt_enable(led_chan));

    ESP_LOGI(TAG, "Start LED rainbow chase");
    //rmt_transmit_config_t tx_config = {
    //    .loop_count = 0, // no transfer loop
    //};

    /*
    ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));
    ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));    
    vTaskDelay(300 / portTICK_PERIOD_MS);

    ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels_ShowRxData, sizeof(led_strip_pixels_ShowRxData), &tx_config));
    ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));
    vTaskDelay(300 / portTICK_PERIOD_MS);        

    ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));
    ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));    
    vTaskDelay(300 / portTICK_PERIOD_MS);
    */        

    ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));
    ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));    
    vTaskDelay(300 / portTICK_PERIOD_MS);

 	#ifdef        MAIN_CUBE
    file_Sound = MOUNT_POINT"/0green.bin";
	#elif defined SUB_CUBE1
    file_Sound = MOUNT_POINT"/1green.bin";
	#elif defined SUB_CUBE2
    file_Sound = MOUNT_POINT"/2green.bin";
	#elif defined SUB_CUBE3
    file_Sound = MOUNT_POINT"/3green.bin";
	#else
    file_Sound = MOUNT_POINT"/4green.bin";
	#endif

    ret = s_example_read_ws2812b_bin_file(file_Sound);
    ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));
    ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));              	
    vTaskDelay(500 / portTICK_PERIOD_MS);       

    ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels_ShowRxData, sizeof(led_strip_pixels_ShowRxData), &tx_config));
    ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));
    vTaskDelay(300 / portTICK_PERIOD_MS);        

    //ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));
    //ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));    
    //vTaskDelay(300 / portTICK_PERIOD_MS);  
    //-------WS2812b-------------
	
    //-------i2c-----------------//ing    
	    mc3416_aw9523b_init_withI2c();
    //-------i2c-----------------

    ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels_ShowRxData, sizeof(led_strip_pixels_ShowRxData), &tx_config));
    ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));
        
    Sound_Name = MOUNT_POINT"/Sound1.wav";
    SetAndWaitPlaySoundFile(Sound_Name);
            
    //Sound_Name = MOUNT_POINT"/Sound1.wav";
    //SetAndWaitPlaySoundFile(Sound_Name);
    //Sound_Name = MOUNT_POINT"/Sound1.wav";
    //SetAndWaitPlaySoundFile(Sound_Name);
    //Sound_Name = MOUNT_POINT"/Sound1.wav";
    //SetAndWaitPlaySoundFile(Sound_Name);

    Sound_Name = MOUNT_POINT"/Sound2.wav";
    SetAndWaitPlaySoundFile(Sound_Name);
    //Sound_Name = MOUNT_POINT"/Sound2.wav";
    //SetAndWaitPlaySoundFile(Sound_Name);
    //Sound_Name = MOUNT_POINT"/Sound2.wav";
    //SetAndWaitPlaySoundFile(Sound_Name);
    //Sound_Name = MOUNT_POINT"/Sound2.wav";
    //SetAndWaitPlaySoundFile(Sound_Name);

    file_Sound = MOUNT_POINT"/Sound3.wav";
    SetAndWaitPlaySoundFile(file_Sound);
    //file_Sound = MOUNT_POINT"/Sound3.wav";
    //SetAndWaitPlaySoundFile(file_Sound);
    //file_Sound = MOUNT_POINT"/Sound3.wav";
    //SetAndWaitPlaySoundFile(file_Sound);
    //file_Sound = MOUNT_POINT"/Sound3.wav";
    //SetAndWaitPlaySoundFile(file_Sound);            

    //Sound_Name = MOUNT_POINT"/Sound4.wav";
    //SetAndWaitPlaySoundFile(Sound_Name);
    //Sound_Name = MOUNT_POINT"/Sound4.wav";
    //SetAndWaitPlaySoundFile(Sound_Name);
    //Sound_Name = MOUNT_POINT"/Sound4.wav";
    //SetAndWaitPlaySoundFile(Sound_Name);
    //Sound_Name = MOUNT_POINT"/Sound4.wav";
    //SetAndWaitPlaySoundFile(Sound_Name);

    //Sound_Name = MOUNT_POINT"/Sound5.wav";
    //SetAndWaitPlaySoundFile(Sound_Name);
    //Sound_Name = MOUNT_POINT"/Sound5.wav";
    //SetAndWaitPlaySoundFile(Sound_Name);
    //Sound_Name = MOUNT_POINT"/Sound5.wav";
    //SetAndWaitPlaySoundFile(Sound_Name);
    //Sound_Name = MOUNT_POINT"/Sound5.wav";
    //SetAndWaitPlaySoundFile(Sound_Name);    

    //Sound_Name = MOUNT_POINT"/Sound6.wav";
    //SetAndWaitPlaySoundFile(Sound_Name);
    //Sound_Name = MOUNT_POINT"/Sound6.wav";
    //SetAndWaitPlaySoundFile(Sound_Name);
    //Sound_Name = MOUNT_POINT"/Sound6.wav";
    //SetAndWaitPlaySoundFile(Sound_Name);
    //Sound_Name = MOUNT_POINT"/Sound6.wav";
    //SetAndWaitPlaySoundFile(Sound_Name);
    //-------WS2812b-------------

    /*
 //debug
    file_Sound = MOUNT_POINT"/0red.bin";
    //char data[EXAMPLE_MAX_CHAR_SIZE];    
    if (stat(file_Sound, &st) == 0) {
        // Delete it if it exists
        printf("Had 0red.bin\r\n");   

        ret = s_example_read_ws2812b_bin_file(file_Sound);
    }
    else
    {
        printf("Noo 0red.bin\r\n");    
    }
    ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));
    ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));    
    vTaskDelay(500 / portTICK_PERIOD_MS);

    file_Sound = MOUNT_POINT"/1red.bin";
    //char data[EXAMPLE_MAX_CHAR_SIZE];    
    if (stat(file_Sound, &st) == 0) {
        // Delete it if it exists
        printf("Had 1red.bin\r\n");   

        ret = s_example_read_ws2812b_bin_file(file_Sound);
    }
    else
    {
        printf("Noo 1red.bin\r\n");    
    }
    ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));
    ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));    
    vTaskDelay(500 / portTICK_PERIOD_MS);

    file_Sound = MOUNT_POINT"/2red.bin";
    //char data[EXAMPLE_MAX_CHAR_SIZE];    
    if (stat(file_Sound, &st) == 0) {
        // Delete it if it exists
        printf("Had 2red.bin\r\n");   

        ret = s_example_read_ws2812b_bin_file(file_Sound);
    }
    else
    {
        printf("Noo 2red.bin\r\n");    
    }
    ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));
    ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));    
    vTaskDelay(500 / portTICK_PERIOD_MS);

    file_Sound = MOUNT_POINT"/3red.bin";
    //char data[EXAMPLE_MAX_CHAR_SIZE];    
    if (stat(file_Sound, &st) == 0) {
        // Delete it if it exists
        printf("Had 3red.bin\r\n");   

        ret = s_example_read_ws2812b_bin_file(file_Sound);
    }
    else
    {
        printf("Noo 3red.bin\r\n");    
    }
    ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));
    ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));    
    vTaskDelay(500 / portTICK_PERIOD_MS);

    file_Sound = MOUNT_POINT"/4red.bin";
    //char data[EXAMPLE_MAX_CHAR_SIZE];    
    if (stat(file_Sound, &st) == 0) {
        // Delete it if it exists
        printf("Had 4red.bin\r\n");   

        ret = s_example_read_ws2812b_bin_file(file_Sound);
    }
    else
    {
        printf("Noo 4red.bin\r\n");    
    }
    ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));
    ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));    
    vTaskDelay(500 / portTICK_PERIOD_MS);

    file_Sound = MOUNT_POINT"/5red.bin";
    //char data[EXAMPLE_MAX_CHAR_SIZE];    
    if (stat(file_Sound, &st) == 0) {
        // Delete it if it exists
        printf("Had 5red.bin\r\n");   

        ret = s_example_read_ws2812b_bin_file(file_Sound);
    }
    else
    {
        printf("Noo 5red.bin\r\n");    
    }
    ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));
    ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));    
    vTaskDelay(500 / portTICK_PERIOD_MS);

    file_Sound = MOUNT_POINT"/6red.bin";
    //char data[EXAMPLE_MAX_CHAR_SIZE];    
    if (stat(file_Sound, &st) == 0) {
        // Delete it if it exists
        printf("Had 6red.bin\r\n");   

        ret = s_example_read_ws2812b_bin_file(file_Sound);
    }
    else
    {
        printf("Noo 6red.bin\r\n");    
    }
    ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));
    ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));    
    vTaskDelay(500 / portTICK_PERIOD_MS);

    file_Sound = MOUNT_POINT"/7red.bin";
    //char data[EXAMPLE_MAX_CHAR_SIZE];    
    if (stat(file_Sound, &st) == 0) {
        // Delete it if it exists
        printf("Had 7red.bin\r\n");   

        ret = s_example_read_ws2812b_bin_file(file_Sound);
    }
    else
    {
        printf("Noo 7red.bin\r\n");    
    }
    ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));
    ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));    
    vTaskDelay(500 / portTICK_PERIOD_MS);

    file_Sound = MOUNT_POINT"/8red.bin";
    //char data[EXAMPLE_MAX_CHAR_SIZE];    
    if (stat(file_Sound, &st) == 0) {
        // Delete it if it exists
        printf("Had 8red.bin\r\n");   

        ret = s_example_read_ws2812b_bin_file(file_Sound);
    }
    else
    {
        printf("Noo 8red.bin\r\n");    
    }
    ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));
    ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));    
    vTaskDelay(500 / portTICK_PERIOD_MS);    

    file_Sound = MOUNT_POINT"/9red.bin";
    //char data[EXAMPLE_MAX_CHAR_SIZE];    
    if (stat(file_Sound, &st) == 0) {
        // Delete it if it exists
        printf("Had 9red.bin\r\n");   

        ret = s_example_read_ws2812b_bin_file(file_Sound);
    }
    else
    {
        printf("Noo 9red.bin\r\n");    
    }
    ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));
    ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));    
    vTaskDelay(500 / portTICK_PERIOD_MS);

    file_Sound = MOUNT_POINT"/10red.bin";
    //char data[EXAMPLE_MAX_CHAR_SIZE];    
    if (stat(file_Sound, &st) == 0) {
        // Delete it if it exists
        printf("Had 10red.bin\r\n");   

        ret = s_example_read_ws2812b_bin_file(file_Sound);
    }
    else
    {
        printf("Noo 10red.bin\r\n");    
    }
    ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));
    ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));    
    vTaskDelay(500 / portTICK_PERIOD_MS);

    file_Sound = MOUNT_POINT"/11red.bin";
    //char data[EXAMPLE_MAX_CHAR_SIZE];    
    if (stat(file_Sound, &st) == 0) {
        // Delete it if it exists
        printf("Had 11red.bin\r\n");   

        ret = s_example_read_ws2812b_bin_file(file_Sound);
    }
    else
    {
        printf("Noo 11red.bin\r\n");    
    }
    ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));
    ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));    
    vTaskDelay(500 / portTICK_PERIOD_MS);

    file_Sound = MOUNT_POINT"/12red.bin";
    //char data[EXAMPLE_MAX_CHAR_SIZE];    
    if (stat(file_Sound, &st) == 0) {
        // Delete it if it exists
        printf("Had 12red.bin\r\n");   

        ret = s_example_read_ws2812b_bin_file(file_Sound);
    }
    else
    {
        printf("Noo 12red.bin\r\n");    
    }
    ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));
    ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));    
    vTaskDelay(500 / portTICK_PERIOD_MS);

    file_Sound = MOUNT_POINT"/13red.bin";
    //char data[EXAMPLE_MAX_CHAR_SIZE];    
    if (stat(file_Sound, &st) == 0) {
        // Delete it if it exists
        printf("Had 13red.bin\r\n");   

        ret = s_example_read_ws2812b_bin_file(file_Sound);
    }
    else
    {
        printf("Noo 13red.bin\r\n");    
    }
    ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));
    ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));    
    vTaskDelay(500 / portTICK_PERIOD_MS);
    
    file_Sound = MOUNT_POINT"/14red.bin";
    //char data[EXAMPLE_MAX_CHAR_SIZE];    
    if (stat(file_Sound, &st) == 0) {
        // Delete it if it exists
        printf("Had 14red.bin\r\n");   

        ret = s_example_read_ws2812b_bin_file(file_Sound);
    }
    else
    {
        printf("Noo 14red.bin\r\n");    
    }
    ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));
    ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));    
    vTaskDelay(500 / portTICK_PERIOD_MS);

    file_Sound = MOUNT_POINT"/15red.bin";
    //char data[EXAMPLE_MAX_CHAR_SIZE];    
    if (stat(file_Sound, &st) == 0) {
        // Delete it if it exists
        printf("Had 15red.bin\r\n");   

        ret = s_example_read_ws2812b_bin_file(file_Sound);
    }
    else
    {
        printf("Noo 15red.bin\r\n");    
    }
    ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));
    ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));    
    vTaskDelay(500 / portTICK_PERIOD_MS);
    
    file_Sound = MOUNT_POINT"/16red.bin";
    //char data[EXAMPLE_MAX_CHAR_SIZE];    
    if (stat(file_Sound, &st) == 0) {
        // Delete it if it exists
        printf("Had 16red.bin\r\n");   

        ret = s_example_read_ws2812b_bin_file(file_Sound);
    }
    else
    {
        printf("Noo 16red.bin\r\n");    
    }
    ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));
    ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));    
    vTaskDelay(500 / portTICK_PERIOD_MS);
    
    file_Sound = MOUNT_POINT"/17red.bin";
    //char data[EXAMPLE_MAX_CHAR_SIZE];    
    if (stat(file_Sound, &st) == 0) {
        // Delete it if it exists
        printf("Had 17red.bin\r\n");   

        ret = s_example_read_ws2812b_bin_file(file_Sound);
    }
    else
    {
        printf("Noo 17red.bin\r\n");    
    }
    ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));
    ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));    
    vTaskDelay(500 / portTICK_PERIOD_MS);
    
    file_Sound = MOUNT_POINT"/18red.bin";
    //char data[EXAMPLE_MAX_CHAR_SIZE];    
    if (stat(file_Sound, &st) == 0) {
        // Delete it if it exists
        printf("Had 18red.bin\r\n");   

        ret = s_example_read_ws2812b_bin_file(file_Sound);
    }
    else
    {
        printf("Noo 18red.bin\r\n");    
    }
    ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));
    ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));    
    vTaskDelay(500 / portTICK_PERIOD_MS);
    
    file_Sound = MOUNT_POINT"/19red.bin";
    //char data[EXAMPLE_MAX_CHAR_SIZE];    
    if (stat(file_Sound, &st) == 0) {
        // Delete it if it exists
        printf("Had 19red.bin\r\n");   

        ret = s_example_read_ws2812b_bin_file(file_Sound);
    }
    else
    {
        printf("Noo 19red.bin\r\n");    
    }
    ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));
    ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));    
    vTaskDelay(500 / portTICK_PERIOD_MS);
    
    file_Sound = MOUNT_POINT"/20red.bin";
    //char data[EXAMPLE_MAX_CHAR_SIZE];    
    if (stat(file_Sound, &st) == 0) {
        // Delete it if it exists
        printf("Had 20red.bin\r\n");   

        ret = s_example_read_ws2812b_bin_file(file_Sound);
    }
    else
    {
        printf("Noo 20red.bin\r\n");    
    }
    ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));
    ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));    
    vTaskDelay(500 / portTICK_PERIOD_MS);

    
    file_Sound = MOUNT_POINT"/11.bin";
    //char data[EXAMPLE_MAX_CHAR_SIZE];    
    if (stat(file_Sound, &st) == 0) {
        // Delete it if it exists
        printf("Had 11.bin\r\n");   

        ret = s_example_read_ws2812b_bin_file(file_Sound);
    }
    else
    {
        printf("Noo 11.bin\r\n");    
    }
    ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));
    ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));    
    vTaskDelay(300 / portTICK_PERIOD_MS);

    file_Sound = MOUNT_POINT"/12.bin";
    //char data[EXAMPLE_MAX_CHAR_SIZE];    
    if (stat(file_Sound, &st) == 0) {
        // Delete it if it exists
        printf("Had 12.bin\r\n");   

        ret = s_example_read_ws2812b_bin_file(file_Sound);
    }
    else
    {
        printf("Noo 12.bin\r\n");    
    }
    ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));
    ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));    
    vTaskDelay(300 / portTICK_PERIOD_MS);

    file_Sound = MOUNT_POINT"/13.bin";
    //char data[EXAMPLE_MAX_CHAR_SIZE];    
    if (stat(file_Sound, &st) == 0) {
        // Delete it if it exists
        printf("Had 13.bin\r\n");   

        ret = s_example_read_ws2812b_bin_file(file_Sound);
    }
    else
    {
        printf("Noo 13.bin\r\n");    
    }
    ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));
    ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));    
    vTaskDelay(300 / portTICK_PERIOD_MS);
    
    file_Sound = MOUNT_POINT"/14.bin";
    //char data[EXAMPLE_MAX_CHAR_SIZE];    
    if (stat(file_Sound, &st) == 0) {
        // Delete it if it exists
        printf("Had 14.bin\r\n");   

        ret = s_example_read_ws2812b_bin_file(file_Sound);
    }
    else
    {
        printf("Noo 14.bin\r\n");    
    }
    ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));
    ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));    
    vTaskDelay(300 / portTICK_PERIOD_MS);

    file_Sound = MOUNT_POINT"/15.bin";
    //char data[EXAMPLE_MAX_CHAR_SIZE];    
    if (stat(file_Sound, &st) == 0) {
        // Delete it if it exists
        printf("Had 15.bin\r\n");   

        ret = s_example_read_ws2812b_bin_file(file_Sound);
    }
    else
    {
        printf("Noo 15.bin\r\n");    
    }
    ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));
    ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));    
    vTaskDelay(300 / portTICK_PERIOD_MS);
    
    file_Sound = MOUNT_POINT"/16.bin";
    //char data[EXAMPLE_MAX_CHAR_SIZE];    
    if (stat(file_Sound, &st) == 0) {
        // Delete it if it exists
        printf("Had 16.bin\r\n");   

        ret = s_example_read_ws2812b_bin_file(file_Sound);
    }
    else
    {
        printf("Noo 16.bin\r\n");    
    }
    ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));
    ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));    
    vTaskDelay(300 / portTICK_PERIOD_MS);
    
    file_Sound = MOUNT_POINT"/17.bin";
    //char data[EXAMPLE_MAX_CHAR_SIZE];    
    if (stat(file_Sound, &st) == 0) {
        // Delete it if it exists
        printf("Had 17.bin\r\n");   

        ret = s_example_read_ws2812b_bin_file(file_Sound);
    }
    else
    {
        printf("Noo 17.bin\r\n");    
    }
    ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));
    ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));    
    vTaskDelay(300 / portTICK_PERIOD_MS);
    
    file_Sound = MOUNT_POINT"/18.bin";
    //char data[EXAMPLE_MAX_CHAR_SIZE];    
    if (stat(file_Sound, &st) == 0) {
        // Delete it if it exists
        printf("Had 18.bin\r\n");   

        ret = s_example_read_ws2812b_bin_file(file_Sound);
    }
    else
    {
        printf("Noo 18.bin\r\n");    
    }
    ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));
    ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));    
    vTaskDelay(300 / portTICK_PERIOD_MS);

    file_Sound = MOUNT_POINT"/19.bin";
    //char data[EXAMPLE_MAX_CHAR_SIZE];    
    if (stat(file_Sound, &st) == 0) {
        // Delete it if it exists
        printf("Had 19.bin\r\n");   

        ret = s_example_read_ws2812b_bin_file(file_Sound);
    }
    else
    {
        printf("Noo 19.bin\r\n");    
    }
    ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));
    ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));    
    vTaskDelay(300 / portTICK_PERIOD_MS);         
    */

    file_Sound = MOUNT_POINT"/0yellow.bin";
    //char data[EXAMPLE_MAX_CHAR_SIZE];    
    if (stat(file_Sound, &st) == 0) {
        // Delete it if it exists
        printf("Had 0yellow.bin\r\n");   

        ret = s_example_read_ws2812b_bin_file(file_Sound);
    }
    else
    {
        printf("Noo 0yellow.bin\r\n");    
    }
    ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));
    ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));    
    vTaskDelay(500 / portTICK_PERIOD_MS);    //300   

    /*
    file_Sound = MOUNT_POINT"/20number0blue.bin";
    //char data[EXAMPLE_MAX_CHAR_SIZE];    
    if (stat(file_Sound, &st) == 0) {
        // Delete it if it exists
        printf("Had 20number0blue.bin\r\n");   

        ret = s_example_read_ws2812b_bin_file(file_Sound);
    }
    else
    {
        printf("Noo 20number0blue.bin\r\n");    
    }
    ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));
    ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));    
    vTaskDelay(2000 / portTICK_PERIOD_MS);    //300
    */
    
    /*
    while (1) {
        for (int i = 0; i < 3; i++) {
            for (int j = i; j < EXAMPLE_LED_NUMBERS; j += 3) {
                // Build RGB pixels
                hue = j * 360 / EXAMPLE_LED_NUMBERS + start_rgb;
                led_strip_hsv2rgb(hue, 100, 100, &red, &green, &blue);
                led_strip_pixels[j * 3 + 0] = green;
                led_strip_pixels[j * 3 + 1] = blue;
                led_strip_pixels[j * 3 + 2] = red;
            }
            // Flush RGB values to LEDs
            ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));
            ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));
            vTaskDelay(pdMS_TO_TICKS(EXAMPLE_CHASE_SPEED_MS));
            memset(led_strip_pixels, 0, sizeof(led_strip_pixels));
            ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));
            ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));
            vTaskDelay(pdMS_TO_TICKS(EXAMPLE_CHASE_SPEED_MS));
        }
        start_rgb += 60;
    }
    */
    //-------WS2812b-------------

    //-------SD card-------------    
    /*
    // All done, unmount partition and disable SPI peripheral
    esp_vfs_fat_sdcard_unmount(mount_point, card);
    ESP_LOGI(TAG, "Card unmounted");

    //deinitialize the bus after all devices are removed
    spi_bus_free(host.slot);    
    */
    //-------SD card-------------

    //-------Timer---------------
    example_timer_user_data_t *user_data = calloc(1, sizeof(example_timer_user_data_t));
    assert(user_data);
    user_data->user_queue = xQueueCreate(10, sizeof(example_timer_event_t));
    assert(user_data->user_queue);
    user_data->timer_group = 0;
    user_data->timer_idx = 0;
    user_data->alarm_value = TIMER_ALARM_PERIOD_S * TIMER_RESOLUTION_HZ;

    ESP_LOGI(TAG, "Init timer with auto-reload");
    user_data->auto_reload = true;
    example_tg_timer_init(user_data);    
    //-------Timer---------------
    /*
    while(1)
    {
                file_Sound = MOUNT_POINT"/1red.bin"; //G_blue
                ret = s_example_read_ws2812b_bin_file(file_Sound);
                ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));
                ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));

                vTaskDelay(300 / portTICK_PERIOD_MS);    //300   

                file_Sound = MOUNT_POINT"/3green.bin";  //F_blue
                ret = s_example_read_ws2812b_bin_file(file_Sound);
                ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));
                ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));                

                vTaskDelay(300 / portTICK_PERIOD_MS);    //300   
    }
    */
    //-------generic_gpio--------
    /*
//    //change gpio interrupt type for one pin
//    gpio_set_intr_type(GPIO_INPUT_IO_0, GPIO_INTR_ANYEDGE);

    //create a queue to handle gpio event from isr
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    //start gpio task
    xTaskCreate(gpio_task_example, "gpio_task_example", 1024*4, NULL, 10, NULL);

    //install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(GPIO_INPUT_IO_0, gpio_isr_handler, (void*) GPIO_INPUT_IO_0);
//    //hook isr handler for specific gpio pin
//    gpio_isr_handler_add(GPIO_INPUT_IO_1, gpio_isr_handler, (void*) GPIO_INPUT_IO_1);    
    */
    xTaskCreate(button_task, "button_task", 2048, NULL, 10, NULL);   
    xTaskCreate(button5_task, "button5_task", 2048, NULL, 10, NULL);       
    //-------generic_gpio--------

    //-------i2c-----------------//ing
    xTaskCreate(SendIr_ReadMc3416_ReadAw9523bSw_task, "SendIr_ReadMc3416_ReadAw9523bSw_task", 1024*3, NULL, 10, NULL);
    //-------i2c-----------------

    //-------uart----------------
    init();    
    xTaskCreate(rx_task, "uart_rx_task", 1024*3, NULL, configMAX_PRIORITIES, NULL);   //1024*3 //1024*2
    xTaskCreate(tx_task, "uart_tx_task", 1024*3, NULL, configMAX_PRIORITIES-1, NULL); //1024*3 //1024*2        
    //-------uart----------------

    //-------adc-----------------
    xTaskCreate(adc1task, "adc1task", 1024*3, NULL, 10, NULL);
    //-------adc-----------------

    //-------testPlaySound-----------------
    xTaskCreate(testPlaySound, "testPlaySound", 1024*3, NULL, 2, NULL); //1024*3
    //-------testPlaySound-----------------				    
				    
				    
    //int testIoInput_state = 0;
    int cnt = 0;
    while (1)
    {   
        //vTaskDelay(500 / portTICK_PERIOD_MS);        
        //esp_task_wdt_reset();   
        
        //testIoInput_state = gpio_get_level(GPIO_INPUT_IO_1);
        //if (testIoInput_state == 0)
        //    gpio_set_level(GPIO_OUTPUT_IO_0, 0);
        //else
        //    gpio_set_level(GPIO_OUTPUT_IO_0, 1);

        turn_off_north_4_led_after_delay();
        turn_off_east_4_led_after_delay();
        turn_off_south_4_led_after_delay();
        turn_off_west_4_led_after_delay();                        

		//-------Ir NorthRx----------
		if(newNorthRxData != 0)          
		{
            printf("North-->%x\r\n", newNorthRxData);
            show_north_rec_data();
						newNorthRxData=0;
		}	       
		//-------Ir NorthRx----------
		
		//-------Ir EastRx----------
		if(newEastRxData != 0)          
		{
            printf("East-->%x\r\n", newEastRxData);
            show_east_rec_data();
						newEastRxData=0;
		}	       
		//-------Ir EastRx----------

		//-------Ir SouthRx----------
		if(newSouthRxData != 0)          
		{
            printf("South-->%x\r\n", newSouthRxData);
            show_south_rec_data();
						newSouthRxData=0;
		}	       
		//-------Ir SouthRx----------
		
		//-------Ir WestRx----------
		if(newWestRxData != 0)          
		{
            printf("West-->%x\r\n", newWestRxData);
            show_west_rec_data();
						newWestRxData=0;
		}	       
		//-------Ir WestRx----------
		
        //gpio_set_level(GPIO_OUTPUT_IO_0, cnt % 2);   
        if(cubeChangedPositionHp == 1)
        {
            cubeChangedPositionHp = 0;
            switch (cubeCurrentPosition) {
            case 0:
                file_Sound = MOUNT_POINT"/0red.bin";
                ret = s_example_read_ws2812b_bin_file(file_Sound);
                ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));
                ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));
                break;
            case 1:
                file_Sound = MOUNT_POINT"/1red.bin";
                ret = s_example_read_ws2812b_bin_file(file_Sound);
                ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));
                ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));            
                break;
            case 2:
                file_Sound = MOUNT_POINT"/2red.bin";
                ret = s_example_read_ws2812b_bin_file(file_Sound);
                ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));
                ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));            
                break;
            case 3:
                file_Sound = MOUNT_POINT"/3red.bin";
                ret = s_example_read_ws2812b_bin_file(file_Sound);
                ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));
                ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));            
                break;
            case 4:
                file_Sound = MOUNT_POINT"/4red.bin";
                ret = s_example_read_ws2812b_bin_file(file_Sound);
                ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));
                ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));            
                break;
            case 5:
                file_Sound = MOUNT_POINT"/5red.bin";
                ret = s_example_read_ws2812b_bin_file(file_Sound);
                ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));
                ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));            
                break;        
            default:
                break;
            }
        }

        if(cubeShakeHp == 1)
        {
            cubeShakeHp = 0;

            playSoundName = MOUNT_POINT"/Sound6.wav";
            needToPlayNewSound = true;								
          				
			file_Sound = MOUNT_POINT"/1.bin";					 
            ret = s_example_read_ws2812b_bin_file(file_Sound);
            ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));
            ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));               																							            
        }
								
        if(btnOnHappen == 1)
        {            
            btnOnHappen = 0;                        

            //#define M 100
            //#define N 1000
            //random = rand()%(N-M+1)+M;
            //printf("random = %d\n",random);

            //unsigned int randomValue = esp_random();               
            //randomValue = randomValue%(1000 - 100 + 1) + 100;
            //printf("random value is %x   %d\r\n", randomValue, randomValue);

            cnt++;
            if(cnt % 2)
            {
                file_Sound = MOUNT_POINT"/1.bin"; //G_blue
                ret = s_example_read_ws2812b_bin_file(file_Sound);
                ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));
                ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));
            }
            else
            {
                file_Sound = MOUNT_POINT"/1blue.bin";  //F_blue
                ret = s_example_read_ws2812b_bin_file(file_Sound);
                ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));
                ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));                
            }
            playSoundName = MOUNT_POINT"/Sound1.wav";                                                           
            needToPlayNewSound = true;            
        }  

        if(port1btn2SwOnHp == 1)
        {
            port1btn2SwOnHp = 0;
            cnt++;
            if(cnt % 2)
            {
                file_Sound = MOUNT_POINT"/2.bin"; //G_blue
                ret = s_example_read_ws2812b_bin_file(file_Sound);
                ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));
                ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));
            }
            else
            {
                file_Sound = MOUNT_POINT"/2blue.bin";  //F_blue
                ret = s_example_read_ws2812b_bin_file(file_Sound);
                ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));
                ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));                
            }
            playSoundName = MOUNT_POINT"/Sound5.wav";                                                           
            needToPlayNewSound = true;                        
        }

        if(port1btn3SwOnHp == 1)
        {
            port1btn3SwOnHp = 0;
            cnt++;
            if(cnt % 2)
            {
                file_Sound = MOUNT_POINT"/3.bin"; //G_blue
                ret = s_example_read_ws2812b_bin_file(file_Sound);
                ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));
                ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));
            }
            else
            {
                file_Sound = MOUNT_POINT"/3blue.bin";  //F_blue
                ret = s_example_read_ws2812b_bin_file(file_Sound);
                ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));
                ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));                
            }
            playSoundName = MOUNT_POINT"/Sound7.wav";                                                           
            needToPlayNewSound = true;                        
        }
        
        if(port1btn4SwOnHp == 1)
        {
            port1btn4SwOnHp = 0;
            cnt++;
            if(cnt % 2)
            {
                file_Sound = MOUNT_POINT"/4.bin"; //G_blue
                ret = s_example_read_ws2812b_bin_file(file_Sound);
                ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));
                ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));
            }
            else
            {
                file_Sound = MOUNT_POINT"/4blue.bin";  //F_blue
                ret = s_example_read_ws2812b_bin_file(file_Sound);
                ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));
                ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));      
            }
            //file_Sound = MOUNT_POINT"/Sound3.wav";
            //SetAndWaitPlaySoundFile(file_Sound);    
            playSoundName = MOUNT_POINT"/Sound3.wav";                                                           
            needToPlayNewSound = true;
        }

        if(btn5OnHappen == 1)
        {            
            btn5OnHappen = 0;                        

            cnt++;
            if(cnt % 2)
            {
                file_Sound = MOUNT_POINT"/5.bin"; //G_blue
                ret = s_example_read_ws2812b_bin_file(file_Sound);
                ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));
                ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));
            }
            else
            {
                file_Sound = MOUNT_POINT"/5blue.bin";  //F_blue
                ret = s_example_read_ws2812b_bin_file(file_Sound);
                ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));
                ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));                
            }
            playSoundName = MOUNT_POINT"/Sound8.wav";                                                           
            needToPlayNewSound = true;            
        }  

        if(port1VolUpSwOnHp == 1)
        {            
            port1VolUpSwOnHp = 0;
            cnt++;
            if(cnt % 2)
            {
                file_Sound = MOUNT_POINT"/6.bin"; //G_blue
                ret = s_example_read_ws2812b_bin_file(file_Sound);
                ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));
                ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));
            }
            else
            {
                file_Sound = MOUNT_POINT"/6blue.bin";  //F_blue
                ret = s_example_read_ws2812b_bin_file(file_Sound);
                ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));
                ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));                
            }
            if(volumeSet < MAX_VOLUME)
                volumeSet++;    
            playSoundName = MOUNT_POINT"/Sound2.wav";                                                           
            needToPlayNewSound = true;            
        }

        if(port1VolDwSwOnHp == 1)
        {            
            port1VolDwSwOnHp = 0;
            cnt++;
            if(cnt % 2)
            {
                file_Sound = MOUNT_POINT"/7.bin"; //G_blue
                ret = s_example_read_ws2812b_bin_file(file_Sound);
                ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));
                ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));
            }
            else
            {
                file_Sound = MOUNT_POINT"/7blue.bin";  //F_blue
                ret = s_example_read_ws2812b_bin_file(file_Sound);
                ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));
                ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));                
            }
            if(volumeSet > MIN_VOLUME)
                volumeSet--;                
            playSoundName = MOUNT_POINT"/Sound4.wav";                                                           
            needToPlayNewSound = true;            
        }                    

        
        if(port1VusbSwOnHp == 1)
        {
            port1VusbSwOnHp = 0;
            
            file_Sound = MOUNT_POINT"/13.bin"; //G_blue
            ret = s_example_read_ws2812b_bin_file(file_Sound);
            ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));
            ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));
        }        
        if(port1VusbSwOffHp == 1)
        {
            port1VusbSwOffHp = 0;
            
            file_Sound = MOUNT_POINT"/14.bin";  //F_blue
            ret = s_example_read_ws2812b_bin_file(file_Sound);
            ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));
            ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));      
        }                             
				
				if(cardOnHappen == 1)
				{
					cardOnHappen = 0;
					if(uid_card_number == 0x00)
					{
    		        	playSoundName = MOUNT_POINT"/Sound1.wav";
    		        	needToPlayNewSound = true;
						file_Sound = MOUNT_POINT"/A_blue.bin";                                	
					}							
					else if(uid_card_number == 0x01)
					{
    		        	playSoundName = MOUNT_POINT"/Sound2.wav";
    		        	needToPlayNewSound = true;							
						//file_Sound = MOUNT_POINT"/B_blue.bin";                                
    		            file_Sound = MOUNT_POINT"/GO1.bin";                               
    		            ret = s_example_read_ws2812b_bin_file(file_Sound);
    		            ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));
    		            ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));
    		            vTaskDelay(100 / portTICK_PERIOD_MS); 
    		
    		            file_Sound = MOUNT_POINT"/GO2.bin";                               
    		            ret = s_example_read_ws2812b_bin_file(file_Sound);
    		            ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));
    		            ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));
    		            vTaskDelay(100 / portTICK_PERIOD_MS); 
    		
    		            file_Sound = MOUNT_POINT"/GO3.bin";                               
    		            ret = s_example_read_ws2812b_bin_file(file_Sound);
    		            ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));
    		            ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));
    		            vTaskDelay(100 / portTICK_PERIOD_MS); 
    		
    		            file_Sound = MOUNT_POINT"/GO4.bin";                               
    		            ret = s_example_read_ws2812b_bin_file(file_Sound);
    		            ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));
    		            ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));
    		            vTaskDelay(100 / portTICK_PERIOD_MS); 
    		
    		            file_Sound = MOUNT_POINT"/GO5.bin";                               
    		            ret = s_example_read_ws2812b_bin_file(file_Sound);
    		            ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));
    		            ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));
    		            vTaskDelay(100 / portTICK_PERIOD_MS);
    		
    		            file_Sound = MOUNT_POINT"/GO6.bin";                               
    		            ret = s_example_read_ws2812b_bin_file(file_Sound);
    		            ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));
    		            ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));
    		            vTaskDelay(100 / portTICK_PERIOD_MS);
    		
    		            file_Sound = MOUNT_POINT"/GO7.bin";                               
    		            ret = s_example_read_ws2812b_bin_file(file_Sound);
    		            ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));
    		            ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));
    		            vTaskDelay(100 / portTICK_PERIOD_MS);
    		
    		            file_Sound = MOUNT_POINT"/GO8.bin";                               
    		            ret = s_example_read_ws2812b_bin_file(file_Sound);
    		            ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));
    		            ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));
    		            vTaskDelay(100 / portTICK_PERIOD_MS);
    		
    		            file_Sound = MOUNT_POINT"/GO9.bin";                               
    		            ret = s_example_read_ws2812b_bin_file(file_Sound);
    		            ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));
    		            ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));
    		            vTaskDelay(100 / portTICK_PERIOD_MS);
    		
    		            file_Sound = MOUNT_POINT"/GO10.bin";                               
    		            ret = s_example_read_ws2812b_bin_file(file_Sound);
    		            ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));
    		            ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));
    		            vTaskDelay(300 / portTICK_PERIOD_MS);
    		
    		            file_Sound = MOUNT_POINT"/GO11.bin";                               
    		            ret = s_example_read_ws2812b_bin_file(file_Sound);
    		            ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));
    		            ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));
    		            vTaskDelay(300 / portTICK_PERIOD_MS);
    		
    		            file_Sound = MOUNT_POINT"/GO12.bin";                               
    		            ret = s_example_read_ws2812b_bin_file(file_Sound);
    		            ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));
    		            ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));
    		            vTaskDelay(300 / portTICK_PERIOD_MS);
					}
					else if(uid_card_number == 0x02)		
					{
    		        	playSoundName = MOUNT_POINT"/Sound3.wav";
    		        	needToPlayNewSound = true;							
						file_Sound = MOUNT_POINT"/C_blue.bin";                                					
					}								
					else if(uid_card_number == 0x03)							
					{
    		        	playSoundName = MOUNT_POINT"/Sound4.wav";
    		        	needToPlayNewSound = true;							
						file_Sound = MOUNT_POINT"/D_blue.bin";                                
					}								
					else //if(uid_card_number == 0x04)						
					{
    		        	playSoundName = MOUNT_POINT"/Sound5.wav";
    		        	needToPlayNewSound = true;							
						file_Sound = MOUNT_POINT"/E_blue.bin";
					}								
    		        ret = s_example_read_ws2812b_bin_file(file_Sound);
    		        ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));
    		        ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));               																				
				}	
						
				if(cardOffHappen == 1)
				{
				    cardOffHappen = 0;
							
    		        playSoundName = MOUNT_POINT"/Sound6.wav";
    		        needToPlayNewSound = true;								
    		      				
					file_Sound = MOUNT_POINT"/0.bin";					 
    		        ret = s_example_read_ws2812b_bin_file(file_Sound);
    		        ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));
    		        ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));               																							
				}	

    }
       
}