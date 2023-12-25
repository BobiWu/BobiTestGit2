#include "driver/gpio.h"
#include "IrRxTx_mc3416_aw9523b_i2c.h"
#include "esp_log.h"
#include <string.h>
#include "driver/i2c.h"
#include "read_mc3416_posi_and_shake.h"
#include "esp_random.h"

static const char *TAG = "3416And9523";

static	int irBitSendingCounters = 0;	//32 //29 //28

//-------
/**
 * @brief Read a sequence of bytes from a MC3416 sensor registers
 */
static esp_err_t mc3416_register_read(uint8_t reg_addr, uint8_t *data, size_t len)
{                                                       //0x4C        
    return i2c_master_write_read_device(I2C_MASTER_NUM, MC3416_SENSOR_ADDR, &reg_addr, 1, data, len, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}
//-------
/**
 * @brief Write a byte to a MC3416 sensor register
 */
static esp_err_t mc3416_register_write_byte(uint8_t reg_addr, uint8_t data)
{
    int ret;
    uint8_t write_buf[2] = {reg_addr, data};
                                                     //0x4C
    ret = i2c_master_write_to_device(I2C_MASTER_NUM, MC3416_SENSOR_ADDR, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);

    return ret;
}
//-------
//-------
/**
 * @brief Read a sequence of bytes from a AW9523B sensor registers
 */
static esp_err_t aw9523b_register_read(uint8_t reg_addr, uint8_t *data, size_t len)
{                                                       //0x5B        
    return i2c_master_write_read_device(I2C_MASTER_NUM, AW9523B_SENSOR_ADDR, &reg_addr, 1, data, len, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}
//-------
/**
 * @brief Write a byte to a AW9523B sensor register
 */
static esp_err_t aw9523b_register_write_byte(uint8_t reg_addr, uint8_t data)
{
    int ret;
    uint8_t write_buf[2] = {reg_addr, data};
                                                     //0x5B
    ret = i2c_master_write_to_device(I2C_MASTER_NUM, AW9523B_SENSOR_ADDR, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);

    return ret;
}
//-------
/**
 * @brief i2c master initialization
 */
static esp_err_t newi2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    i2c_param_config(i2c_master_port, &conf);

    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

//111
void mc3416_aw9523b_init_withI2c(void)
{
    ESP_ERROR_CHECK(newi2c_master_init());
    ESP_LOGI(TAG, "I2C initialized successfully");	

    //-------MC34XX------
    //Read the MC3416 WHO_AM_I register, on power up the register should have the value 0x71     
    mc3416_register_read(MC3416_WHO_AM_I_REG_ADDR, data_i2c, 1);    
    if(data_i2c[0] == 0xA0)
    {
    	ESP_LOGI(TAG, "MC3416 WHO_AM_I = %X", data_i2c[0]);
        //---MC3416 Init---
        mc3416_register_write_byte(0x07, 0x40);//power off
        mc3416_register_write_byte(0x08, 0x01);//256 hz		3.90625ms
        mc3416_register_write_byte(0x20, 0x29);//0x29:x010 1001=+-8g
        mc3416_register_write_byte(0x07, 0x41);//power on    
        //---MC3416 Init---
    }
    else
    {		
        mc3416_register_read(MC3423_WHO_AM_I_REG_ADDR, data_i2c, 1);        
        if(data_i2c[0] == 0x10)
        {
        	ESP_LOGI(TAG, "MC3423 WHO_AM_I = %X", data_i2c[0]);
            //---MC3423 Init---
            mc3416_register_write_byte(0x07, 0x40);//power off
            mc3416_register_write_byte(0x08, 0x0A);//256 hz		3.90625ms
            mc3416_register_write_byte(0x20, 0x25);//0x25:x010 x101=+-8g  14bit
            mc3416_register_write_byte(0x07, 0x41);//power on    
            //---MC3423 Init---            
        }
        else
        {
        	ESP_LOGI(TAG, "MC3423 WHO_AM_I = ERROR");
        }
    }
    //-------MC34XX------	
    
    //-------AW9523B------
    aw9523b_register_read(AW9523B_WHO_AM_I_REG_ADDR, data_i2c, 1);      
    if(data_i2c[0] == 0x23)
    {
    	ESP_LOGI(TAG, "AW9523B WHO_AM_I = %X", data_i2c[0]);
        
        aw9523b_register_write_byte(CONFIG_PORT0, 0xF0);//0b11110000
        aw9523b_register_read(CONFIG_PORT0, data_i2c, 1);
        ESP_LOGI(TAG, "AW9523B CONFIG_PORT0 = %X", data_i2c[0]);

        aw9523b_register_write_byte(GCR, 0x10);//0b000 1 0000 //GPOMD=1  P0--->Push-Pull
        aw9523b_register_read(GCR, data_i2c, 1);
        ESP_LOGI(TAG, "AW9523B GCR = %X", data_i2c[0]);

        aw9523b_register_write_byte(OUTPUT_PORT0, 0xF0);//0b11110000
        aw9523b_register_read(OUTPUT_PORT0, data_i2c, 1);
        ESP_LOGI(TAG, "AW9523B OUTPUT_PORT0 = %X", data_i2c[0]);

        aw9523b_register_write_byte(CONFIG_PORT1, 0x3F);//0b00111111
        aw9523b_register_read(CONFIG_PORT1, data_i2c, 1);
        ESP_LOGI(TAG, "AW9523B CONFIG_PORT1 = %X", data_i2c[0]);

        aw9523b_register_write_byte(OUTPUT_PORT1, 0x40);//0b01000000    AMP_SD=1	//999
        aw9523b_register_read(OUTPUT_PORT1, data_i2c, 1);
        ESP_LOGI(TAG, "AW9523B OUTPUT_PORT1 = %X", data_i2c[0]);

        aw9523b_register_write_byte(INT_PORT0, 0xFF);//0b11101111		0:EN  1:DIS     0b11101111(MC3416 int)
        aw9523b_register_read(INT_PORT0, data_i2c, 1);
        ESP_LOGI(TAG, "AW9523B INT_PORT0 = %X", data_i2c[0]);

        aw9523b_register_write_byte(INT_PORT1, 0xFF);//0b11 111111		0:EN  1:DIS
        aw9523b_register_read(INT_PORT1, data_i2c, 1);
        ESP_LOGI(TAG, "AW9523B INT_PORT1 = %X", data_i2c[0]);

        //aw9523b_register_write_byte(OUTPUT_PORT0, 0xF0);//0b11110000
        aw9523b_register_read(INPUT_PORT0, data_i2c, 1);
        aw9523b_register_read(INPUT_PORT1, data_i2c, 1);    
    
		aw9523b_register_write_byte(OUTPUT_PORT1, 0xC0);//0b11000000  ESP-NOW=1  AMP_SD=1	//999
		aw9523b_register_read(OUTPUT_PORT1, data_i2c, 1);        
    }
    else
    {
    	ESP_LOGI(TAG, "AW9523B WHO_AM_I = ERROR");    		
    }
    //-------AW9523B------    
}

void SendIr_ReadMc3416_ReadAw9523bSw_task(void *arg)        
{
    int port1VolUpSw_debounce = 0;
    int port1VolUpSw_pressed = 0;        

    int port1VolDwSw_debounce = 0;
    int port1VolDwSw_pressed = 0;        

    int port1btn2Sw_debounce = 0;
    int port1btn2Sw_pressed = 0;       

    int port1btn3Sw_debounce = 0;
    int port1btn3Sw_pressed = 0;

    int port1btn4Sw_debounce = 0;
    int port1btn4Sw_pressed = 0;

    int port1VusbSwOn_debounce = 0;
    int port1VusbSwOff_debounce = 0;
    int port1VusbSwOn_Hold = 0;        
    int port1VusbSwOff_Hold = 0;            

    int readMC3416PeriodCounter = 0;  


	uint8_t portBit3ToBit0Value = 0;	
    unsigned int sendIrCodePeriod = 1;	

    irBitSendingCounters = 29;	//32 //29 //28

    //unsigned int temp = 0;

	unsigned int irTxEast;
	unsigned int irTxWest;
	unsigned int irTxSouth;
	unsigned int irTxNorth;

#ifdef        MAIN_CUBE
	irTxEast = 0xFC455510;
	irTxWest = 0xFC455450;
	irTxSouth = 0xFC455150;
	irTxNorth = 0xFC454550;   
	ESP_LOGI(TAG, "MAIN_CUBE");	   
#elif defined SUB_CUBE1
	irTxEast = 0xFC515510;
	irTxWest = 0xFC515450;
	irTxSouth = 0xFC515150;
	irTxNorth = 0xFC514550;   
	ESP_LOGI(TAG, "SUB_CUBE1");	   	
#elif defined SUB_CUBE2
	irTxEast = 0xFC545510;
	irTxWest = 0xFC545450;
	irTxSouth = 0xFC545150;
	irTxNorth = 0xFC544550;   
	ESP_LOGI(TAG, "SUB_CUBE2");	   	
#elif defined SUB_CUBE3
	irTxEast = 0xFD115510;
	irTxWest = 0xFD115450;
	irTxSouth = 0xFD115150;
	irTxNorth = 0xFD114550;   
	ESP_LOGI(TAG, "SUB_CUBE3");	   	
#else
	irTxEast = 0xFD145510;
	irTxWest = 0xFD145450;
	irTxSouth = 0xFD145150;
	irTxNorth = 0xFD144550;   
	ESP_LOGI(TAG, "SUB_CUBE4");	   	
#endif


/*
//--------------
TX          Main_Cube   Sub_Cube1   Sub_Cube2   Sub_Cube3   Sub_Cube4
NORTH    :  0xFC454550  0xFC514550  0xFC544550  0xFD114550  0xFD144550
EAST     :  0xFC455510  0xFC515510  0xFC545510  0xFD115510  0xFD145510
SOUTH    :  0xFC455150  0xFC515150  0xFC545150  0xFD115150  0xFD145150
WEST     :  0xFC455450  0xFC515450  0xFC545450  0xFD115450  0xFD145450
//--------------
TX          Main_Cube   Sub_Cube1   Sub_Cube2   Sub_Cube3   Sub_Cube4
NORTH    :  0xC8        0xA8        0x98        0x68        0x58
EAST     :  0xC1        0xA1        0x91        0x61        0x51
SOUTH    :  0xC4        0xA4        0x94        0x64        0x54
WEST     :  0xC2        0xA2        0x92        0x62        0x52
//--------------
Main_Cube:                      0001 0001 01   01
Sub_Cube1:                      0001 01   0001 01
Sub_Cube2:                      0001 01   01   0001
Sub_Cube3:                      01   0001 0001 01
Sub_Cube4:                      01   0001 01   0001
//--------------
IR_TX_NORTH :   Header-6ms	    Bit1 Bit1 Bit0 Bit0     Bit0 Bit0 Bit0 Bit1     off        	0b11001000      
  				111111    	    0001 0001 01   01       0001 01   01   01       0000		(0xFC454550)
IR_TX_EAST  :   Header-6ms	    Bit1 Bit1 Bit0 Bit0     Bit0 Bit0 Bit0 Bit1              	0b11000001      
  				111111    	    0001 0001 01   01       01   01   01   0001     0000		(0xFC455510)
IR_TX_SOUTH :	Header-6ms	    Bit1 Bit1 Bit0 Bit0     Bit0 Bit0 Bit0 Bit1                 0b11000100      
  				111111    	    0001 0001 01   01       01   0001 01   01       0000		(0xFC455150)
IR_TX_WEST  :   Header-6ms	    Bit1 Bit1 Bit0 Bit0     Bit0 Bit0 Bit0 Bit1                 0b11000010      
  				111111    	    0001 0001 01   01       01   01   0001 01       0000		(0xFC455450)    
//--------------
IR_TX_NORTH :   Header-6ms	    Bit1 Bit1 Bit0 Bit0     Bit0 Bit0 Bit0 Bit1     off         0b10101000      
  				111111    	    0001 01   0001 01       0001 01   01   01       0000		(0xFC514550)
IR_TX_EAST  :   Header-6ms	    Bit1 Bit1 Bit0 Bit0     Bit0 Bit0 Bit0 Bit1              	0b10100001      
  				111111    	    0001 01   0001 01       01   01   01   0001     0000		(0xFC515510)
IR_TX_SOUTH :	Header-6ms	    Bit1 Bit1 Bit0 Bit0     Bit0 Bit0 Bit0 Bit1                 0b10100100      
  				111111    	    0001 01   0001 01       01   0001 01   01       0000		(0xFC515150) 
IR_TX_WEST  :   Header-6ms	    Bit1 Bit1 Bit0 Bit0     Bit0 Bit0 Bit0 Bit1                 0b10100010      
  				111111    	    0001 01   0001 01       01   01   0001 01       0000		(0xFC515450)
//--------------
IR_TX_NORTH :   Header-6ms	    Bit1 Bit1 Bit0 Bit0     Bit0 Bit0 Bit0 Bit1     off         0b10011000      
  				111111    	    0001 01   01   0001     0001 01   01   01       0000		(0xFC544550)
IR_TX_EAST  :   Header-6ms	    Bit1 Bit1 Bit0 Bit0     Bit0 Bit0 Bit0 Bit1              	0b10010001      
  				111111    	    0001 01   01   0001     01   01   01   0001     0000		(0xFC545510)
IR_TX_SOUTH :	Header-6ms	    Bit1 Bit1 Bit0 Bit0     Bit0 Bit0 Bit0 Bit1                 0b10010100      
  				111111    	    0001 01   01   0001     01   0001 01   01       0000		(0xFC545150) 
IR_TX_WEST  :   Header-6ms	    Bit1 Bit1 Bit0 Bit0     Bit0 Bit0 Bit0 Bit1                 0b10010010      
  				111111    	    0001 01   01   0001     01   01   0001 01       0000		(0xFC545450)
//--------------
IR_TX_NORTH :   Header-6ms	    Bit1 Bit1 Bit0 Bit0     Bit0 Bit0 Bit0 Bit1     off         0b01101000      
  				111111    	    01   0001 0001 01       0001 01   01   01       0000		(0xFD114550)
IR_TX_EAST  :   Header-6ms	    Bit1 Bit1 Bit0 Bit0     Bit0 Bit0 Bit0 Bit1              	0b01100001      
  				111111    	    01   0001 0001 01       01   01   01   0001     0000		(0xFD115510)
IR_TX_SOUTH :	Header-6ms	    Bit1 Bit1 Bit0 Bit0     Bit0 Bit0 Bit0 Bit1                 0b01100100      
  				111111    	    01   0001 0001 01       01   0001 01   01       0000		(0xFD115150) 
IR_TX_WEST  :   Header-6ms	    Bit1 Bit1 Bit0 Bit0     Bit0 Bit0 Bit0 Bit1                 0b01100010      
  				111111    	    01   0001 0001 01       01   01   0001 01       0000		(0xFD115450)
//--------------
IR_TX_NORTH :   Header-6ms	    Bit1 Bit1 Bit0 Bit0     Bit0 Bit0 Bit0 Bit1     off         0b01011000      
  				111111    	    01   0001 01   0001     0001 01   01   01       0000		(0xFD144550)
IR_TX_EAST  :   Header-6ms	    Bit1 Bit1 Bit0 Bit0     Bit0 Bit0 Bit0 Bit1              	0b01010001      
  				111111    	    01   0001 01   0001     01   01   01   0001     0000		(0xFD145510)
IR_TX_SOUTH :	Header-6ms	    Bit1 Bit1 Bit0 Bit0     Bit0 Bit0 Bit0 Bit1                 0b01010100      
  				111111    	    01   0001 01   0001     01   0001 01   01       0000		(0xFD145150) 
IR_TX_WEST  :   Header-6ms	    Bit1 Bit1 Bit0 Bit0     Bit0 Bit0 Bit0 Bit1                 0b01010010      
  				111111    	    01   0001 01   0001     01   01   0001 01       0000		(0xFD145450)
//--------------
//--------------
*/	

    while (1) {  
        //int cnt++;
        //gpio_set_level(GPIO_OUTPUT_IO_0, cnt % 2);    
        vTaskDelay(1 / portTICK_PERIOD_MS); //1

        //---Send IR LED---
        //------
	    if(sendIrCodePeriod == 1){
		    //--- 	
            portBit3ToBit0Value = 0;

            if(irTxEast & 0x80000000)
			    portBit3ToBit0Value = portBit3ToBit0Value | 0x01;
		    else
                portBit3ToBit0Value = portBit3ToBit0Value & 0xFE;
		    irTxEast <<= 1;
		    //--- 		            
		    if(irTxWest & 0x80000000)
			    portBit3ToBit0Value = portBit3ToBit0Value | 0x02;
		    else
                portBit3ToBit0Value = portBit3ToBit0Value & 0xFD;		
		    irTxWest <<= 1;	
		    //--- 		
		    if(irTxSouth & 0x80000000)
			    portBit3ToBit0Value = portBit3ToBit0Value | 0x04;
		    else
                portBit3ToBit0Value = portBit3ToBit0Value & 0xFB;
		    irTxSouth <<= 1;		
		    //--- 		
		    if(irTxNorth & 0x80000000)
			    portBit3ToBit0Value = portBit3ToBit0Value | 0x08;
		    else
                portBit3ToBit0Value = portBit3ToBit0Value & 0xF7;	
		    irTxNorth <<= 1;	
		    //--- 		
  	        aw9523b_register_write_byte(OUTPUT_PORT0, portBit3ToBit0Value);
		    //--- 		
		    irBitSendingCounters--;
		    if(irBitSendingCounters == 0){
                //Sending one byte finish
			    //irBitSendingCounters = 29;	//32 //29 //28

                #ifdef        MAIN_CUBE
	            irTxEast = 0xFC455510;
	            irTxWest = 0xFC455450;
	            irTxSouth = 0xFC455150;
	            irTxNorth = 0xFC454550;   
                #elif defined SUB_CUBE1
	            irTxEast = 0xFC515510;
	            irTxWest = 0xFC515450;
	            irTxSouth = 0xFC515150;
	            irTxNorth = 0xFC514550;   
                #elif defined SUB_CUBE2
	            irTxEast = 0xFC545510;
	            irTxWest = 0xFC545450;
	            irTxSouth = 0xFC545150;
	            irTxNorth = 0xFC544550;   
                #elif defined SUB_CUBE3
	            irTxEast = 0xFD115510;
	            irTxWest = 0xFD115450;
	            irTxSouth = 0xFD115150;
	            irTxNorth = 0xFD114550;   
                #else
	            irTxEast = 0xFD145510;
	            irTxWest = 0xFD145450;
	            irTxSouth = 0xFD145150;
	            irTxNorth = 0xFD144550;   
                #endif    

                //sendIrCodePeriod = 500;		

                //#define M 100
                //#define N 1000
                //random = rand()%(N-M+1)+M;
                //printf("random = %d\n",random);

                unsigned int randomValue = esp_random();               
                randomValue = randomValue%(500 - 100 + 1) + 100; //randomValue = (rand() % (max - min + 1)) + min;     //[min,max]  //1000
                //printf("random value is %x   %d\r\n", randomValue, randomValue);            
			    sendIrCodePeriod = randomValue;		
		    }	
	    }	
		//------ 		        
	    else if(sendIrCodePeriod > 1){
		    sendIrCodePeriod--;
            if(sendIrCodePeriod == 1)
            {
                irBitSendingCounters = 29;	//32 //29 //28
            }
	    }
        //------
        //---Send IR LED---

        //---Read MC3416 input---
        readMC3416PeriodCounter++;
        if(readMC3416PeriodCounter == 5)//5
        {
            readMC3416PeriodCounter = 0;    
            mc3416_register_read(MC3416_XOUT_EX_L_REG_ADDR, data_i2c, 6);
            //ESP_LOGI(TAG, "MC3416 x = %X%X      y = %X%X      z = %X%X", data_i2c[1], data_i2c[0], data_i2c[3], data_i2c[2], data_i2c[5], data_i2c[4]);  
            
            XOUT_Data = 0;
            XOUT_Data = data_i2c[1];
            XOUT_Data <<= 8;
            XOUT_Data = XOUT_Data + data_i2c[0];

            YOUT_Data = 0;
            YOUT_Data = data_i2c[3];
            YOUT_Data <<= 8;
            YOUT_Data = YOUT_Data + data_i2c[2];

            ZOUT_Data = 0;
            ZOUT_Data = data_i2c[5];
            ZOUT_Data <<= 8;
            ZOUT_Data = ZOUT_Data + data_i2c[4];
            
            //ESP_LOGI(TAG, "MC3416 x = %d      y = %d      z = %d", XOUT_Data, YOUT_Data, ZOUT_Data);   
            uint8_t cube_new_position = 0;
            cube_new_position = CheckGrabItPosition(XOUT_Data, YOUT_Data, ZOUT_Data);
            if(cube_previous_position != cube_new_position)
            {
                cube_previous_position = cube_new_position;
                cubePositionChangedHappen = 1;
                ESP_LOGI(TAG, "MC3416 Position = %d", cube_previous_position);    
            }
            
            if(CheckNewShake(XOUT_Data, YOUT_Data, ZOUT_Data))
            {
                cubeShakeHappen = 1;            
                ESP_LOGI(TAG, "cubeShakeHappen");	   
            }

            //ESP_LOGI(TAG, "MC3416 x = %d      y = %d      z = %d", XOUT_Data, YOUT_Data, ZOUT_Data);   
        }
        //---Read MC3416 input---

        //---Read AW9523B input---
        aw9523b_register_read(INPUT_PORT0, data_i2c, 1);    
        port0InputValue = data_i2c[0];        

        aw9523b_register_read(INPUT_PORT1, data_i2c, 1);    
        port1InputValue = data_i2c[0];

        if (port1InputValue & port1VolUpSwBit) {
            port1VolUpSw_debounce++;
            if (port1VolUpSw_debounce >= 30) {
                if (!port1VolUpSw_pressed) {										
                    port1VolUpSw_pressed = 1;
                    port1VolUpSwOnHappen = 1;
                    ESP_LOGI(TAG, "port1VolUpSwOnHappen");	
                }
            }
        } 
        else {
            port1VolUpSw_debounce = 0;
            port1VolUpSw_pressed = 0;
        }    

        if (port1InputValue & port1VolDwSwBit) {
            port1VolDwSw_debounce++;
            if (port1VolDwSw_debounce >= 30) {
                if (!port1VolDwSw_pressed) {										
                    port1VolDwSw_pressed = 1;
                    port1VolDwSwOnHappen = 1;
                    ESP_LOGI(TAG, "port1VolDwSwOnHappen");	
                }
            }
        } 
        else {
            port1VolDwSw_debounce = 0;
            port1VolDwSw_pressed = 0;
        }

        if (port1InputValue & port1btn2SwBit) {
            port1btn2Sw_debounce++;
            if (port1btn2Sw_debounce >= 30) {
                if (!port1btn2Sw_pressed) {										
                    port1btn2Sw_pressed = 1;
                    port1btn2SwOnHappen = 1;
                    ESP_LOGI(TAG, "port1btn2SwOnHappen");	
                }
            }
        } 
        else {
            port1btn2Sw_debounce = 0;
            port1btn2Sw_pressed = 0;
        }

        if (port1InputValue & port1btn3SwBit) {
            port1btn3Sw_debounce++;
            if (port1btn3Sw_debounce >= 30) {
                if (!port1btn3Sw_pressed) {										
                    port1btn3Sw_pressed = 1;
                    port1btn3SwOnHappen = 1;
                    ESP_LOGI(TAG, "port1btn3SwOnHappen");	
                }
            }
        } 
        else {
            port1btn3Sw_debounce = 0;
            port1btn3Sw_pressed = 0;
        }

        if (port1InputValue & port1btn4SwBit) {
            port1btn4Sw_debounce++;
            if (port1btn4Sw_debounce >= 30) {
                if (!port1btn4Sw_pressed) {										
                    port1btn4Sw_pressed = 1;
                    port1btn4SwOnHappen = 1;
                    ESP_LOGI(TAG, "port1btn4SwOnHappen");	
                }
            }
        } 
        else {
            port1btn4Sw_debounce = 0;
            port1btn4Sw_pressed = 0;
        }

        if (port1InputValue & port1VusbSwBit) {
        	port1VusbSwOn_debounce = 0;
            port1VusbSwOff_debounce++;
            if (port1VusbSwOff_debounce >= 100) {
                if (!port1VusbSwOff_Hold) {
                	port1VusbSwOn_Hold = 0;
                    port1VusbSwOff_Hold = 1;
                    port1VusbSwOffHappen = 1;             
                    ESP_LOGI(TAG, "port1VusbSwOffHappen");	       
                }
            }
        } 
        else {            
        	port1VusbSwOff_debounce = 0;
            port1VusbSwOn_debounce++;
            if (port1VusbSwOn_debounce >= 100) {
                if (!port1VusbSwOn_Hold) {										
                	port1VusbSwOff_Hold = 0;
                    port1VusbSwOn_Hold = 1;
                    port1VusbSwOnHappen = 1;
                    ESP_LOGI(TAG, "port1VusbSwOnHappen");	
                }
            }            
        }       
    }	
}

//ing2
//-------Ir NorthRx----------
unsigned int 	north_rx_per_per100us(void)
{
    int irNorthRx_state = 0;	    

    if(irBitSendingCounters > 0) 
    {//if sending ir, do not rec
      	gettingNorthRx0or1 = 0;
      	NorthRxHighTimeCounter = 0;
      	NorthRxLowTimeCounter = 0;        
    }

    irNorthRx_state = gpio_get_level(IrNorthRx);
    if(irNorthRx_state == 1)
    {//High come     	
        if(NorthRxLowTimeCounter < (1500/Imm_IrNorthRxUnitTime))//1000us
        {//;;checking data 0_1
            if(gettingNorthRx0or1 == 0)
            {//It is nosie,clear
                NorthRxLowTimeCounter = 0;
            }
            else
            {//;;
      	        NorthRxHighTimeCounter++;
      	        if(NorthRxHighTimeCounter >= (5500/Imm_IrNorthRxUnitTime))	
      	        {//more than 5.5ms if 3.4ms,error
      	            gettingNorthRx0or1 = 0;
      	            NorthRxHighTimeCounter = 0;
      	            NorthRxLowTimeCounter = 0;
                    NorthRxBitCounter = 0;
      	        }
      	        else//;;
      	        {
	  			    if(NorthRxLowTimeCounter != 0)
	  			    {
	  			        if((NorthRxLowTimeCounter >= (200/Imm_IrNorthRxUnitTime)) && (NorthRxLowTimeCounter < (1500/Imm_IrNorthRxUnitTime)))//1000us
	  			        {
    			            NorthRxLowTimeCounter = 0;
    			            gettingNorthRxData <<= 1;
	  			            if(NorthRxHighTimeCounter < (1800/Imm_IrNorthRxUnitTime))
	  			            {//;;Got 0
	  					        //gettingNorthRxData&=0x3FFFE;
	  						    gettingNorthRxData &= 0xFFFFFFFE;
	  			            }
	  			            else
	  			            {//;;Got 1
	  						    gettingNorthRxData |= 0x00000001;
	  			            }
	  			            NorthRxHighTimeCounter = 0;
	  			            NorthRxBitCounter--;
	  			            if(NorthRxBitCounter == 0)	
	  			            {//Finished
	  						    gettingNorthRx0or1 = 0;
	  						    NorthRxHighTimeCounter = 0;
	  						    NorthRxLowTimeCounter = 0;
	  						    gotNorthRx8BitDataHappen = 1;
	  						    NorthRxGotData = gettingNorthRxData;  
                                if(NorthRxGotData == 0x00)//debug1
                                {
                                    NorthRxGotData = 0x54;
                                }
	  			            }
	  			        }
	  			    }
      	        }
            }
        }
        else
        {
            if(//666b
      	        ((5000/Imm_IrNorthRxUnitTime) <= NorthRxLowTimeCounter) &&
      	        (NorthRxLowTimeCounter < (8000/Imm_IrNorthRxUnitTime))
              )
            {//;;check header
                NorthRxBitCounter = 8;//32;//8;	
                gettingNorthRx0or1 = 1;
                gettingNorthRxData = 0;
                NorthRxHighTimeCounter = 0;
                NorthRxLowTimeCounter = 0;                      
            }
            else
            {
                NorthRxLowTimeCounter = 0;  	//;;notheader£¬not 0 or 1£¬error
            }
        }
    }
    else
    {//Low come
        NorthRxLowTimeCounter++;                  
    }
//ing2    
    if(gotNorthRx8BitDataHappen==1)
    {
    	gotNorthRx8BitDataHappen=0;
    	return (NorthRxGotData);
    }	
    else
    {
    	return (0);
    }	
}
//-------Ir NorthRx----------

//-------Ir EastRx----------
unsigned int 	east_rx_per_per100us(void)
{
    int irEastRx_state = 0;	    

    if(irBitSendingCounters > 0)
    {//if sending ir, do not rec
      	gettingEastRx0or1 = 0;
      	EastRxHighTimeCounter = 0;
      	EastRxLowTimeCounter = 0;        
    }    

    irEastRx_state = gpio_get_level(IrEastRx);	        	
    if(irEastRx_state == 1)
    {//High come
        if(EastRxLowTimeCounter < (1500/Imm_IrEastRxUnitTime))//1000us
        {//;;checking data 0_1
            if(gettingEastRx0or1 == 0)
            {//It is nosie,clear
                EastRxLowTimeCounter = 0;
            }
            else
            {//;;
      	        EastRxHighTimeCounter++;
      	        if(EastRxHighTimeCounter >= (5500/Imm_IrEastRxUnitTime))	
      	        {//more than 5.5ms if 3.4ms,error
      	            gettingEastRx0or1 = 0;
      	            EastRxHighTimeCounter = 0;
      	            EastRxLowTimeCounter = 0;
                    EastRxBitCounter = 0;
      	        }
      	        else//;;
      	        {
	  			    if(EastRxLowTimeCounter != 0)
	  			    {
	  			        if((EastRxLowTimeCounter >= (200/Imm_IrEastRxUnitTime)) && (EastRxLowTimeCounter < (1500/Imm_IrEastRxUnitTime)))//1000us
	  			        {
    			            EastRxLowTimeCounter = 0;
    			            gettingEastRxData <<= 1;
	  			            if(EastRxHighTimeCounter < (1800/Imm_IrEastRxUnitTime))
	  			            {//;;Got 0
	  					        //gettingEastRxData&=0x3FFFE;
	  						    gettingEastRxData &= 0xFFFFFFFE;
	  			            }
	  			            else
	  			            {//;;Got 1
	  						    gettingEastRxData |= 0x00000001;
	  			            }
	  			            EastRxHighTimeCounter = 0;
	  			            EastRxBitCounter--;
	  			            if(EastRxBitCounter == 0)	
	  			            {//Finished
	  						    gettingEastRx0or1 = 0;
	  						    EastRxHighTimeCounter = 0;
	  						    EastRxLowTimeCounter = 0;
	  						    gotEastRx8BitDataHappen = 1;
	  						    EastRxGotData = gettingEastRxData;  
                                if(EastRxGotData == 0x00)//debug1
                                {
                                    EastRxGotData = 0x54;
                                }                        
	  			            }
	  			        }
	  			    }
      	        }
            }
        }
        else
        {
            if(//666b
      	        ((5000/Imm_IrEastRxUnitTime) <= EastRxLowTimeCounter) &&
      	        (EastRxLowTimeCounter < (8000/Imm_IrEastRxUnitTime))
              )
            {//;;check header
                EastRxBitCounter = 8;//32;//8;	
                gettingEastRx0or1 = 1;
                gettingEastRxData = 0;
                EastRxHighTimeCounter = 0;
                EastRxLowTimeCounter = 0;
            }
            else
            {
                EastRxLowTimeCounter = 0;  	//;;notheader£¬not 0 or 1£¬error
            }
        }
    }
    else
    {//Low come
        EastRxLowTimeCounter++;
    }
//ing2    
    if(gotEastRx8BitDataHappen==1)
    {
    	gotEastRx8BitDataHappen=0;
    	return (EastRxGotData);
    }	
    else
    {
    	return (0);
    }    
}
//-------Ir EastRx----------

//-------Ir SouthRx----------
unsigned int 	south_rx_per_per100us(void)
{
    int irSouthRx_state = 0;	    

    if(irBitSendingCounters > 0)
    {//if sending ir, do not rec
      	gettingSouthRx0or1 = 0;
      	SouthRxHighTimeCounter = 0;
      	SouthRxLowTimeCounter = 0;        
    }    

    irSouthRx_state = gpio_get_level(IrSouthRx);	        	
    if(irSouthRx_state == 1)
    {//High come
        if(SouthRxLowTimeCounter < (1500/Imm_IrSouthRxUnitTime))//1000us
        {//;;checking data 0_1
            if(gettingSouthRx0or1 == 0)
            {//It is nosie,clear
                SouthRxLowTimeCounter = 0;
            }
            else
            {//;;
      	        SouthRxHighTimeCounter++;
      	        if(SouthRxHighTimeCounter >= (5500/Imm_IrSouthRxUnitTime))	
      	        {//more than 5.5ms if 3.4ms,error
      	            gettingSouthRx0or1 = 0;
      	            SouthRxHighTimeCounter = 0;
      	            SouthRxLowTimeCounter = 0;
                    SouthRxBitCounter = 0;
      	        }
      	        else//;;
      	        {
	  			    if(SouthRxLowTimeCounter != 0)
	  			    {
	  			        if((SouthRxLowTimeCounter >= (200/Imm_IrSouthRxUnitTime)) && (SouthRxLowTimeCounter < (1500/Imm_IrSouthRxUnitTime)))//1000us
	  			        {
    			            SouthRxLowTimeCounter = 0;
    			            gettingSouthRxData <<= 1;
	  			            if(SouthRxHighTimeCounter < (1800/Imm_IrSouthRxUnitTime))
	  			            {//;;Got 0
	  					        //gettingSouthRxData&=0x3FFFE;
	  						    gettingSouthRxData &= 0xFFFFFFFE;
	  			            }
	  			            else
	  			            {//;;Got 1
	  						    gettingSouthRxData |= 0x00000001;
	  			            }
	  			            SouthRxHighTimeCounter = 0;
	  			            SouthRxBitCounter--;
	  			            if(SouthRxBitCounter == 0)	
	  			            {//Finished
	  						    gettingSouthRx0or1 = 0;
	  						    SouthRxHighTimeCounter = 0;
	  						    SouthRxLowTimeCounter = 0;
	  						    gotSouthRx8BitDataHappen = 1;
	  						    SouthRxGotData = gettingSouthRxData;  
                                if(SouthRxGotData == 0x00)//debug1
                                {
                                    SouthRxGotData = 0x54;
                                }                         
	  			            }                            
	  			        }
	  			    }
      	        }
            }
        }
        else
        {
            if(//666b
      	        ((5000/Imm_IrSouthRxUnitTime) <= SouthRxLowTimeCounter) &&
      	        (SouthRxLowTimeCounter < (8000/Imm_IrSouthRxUnitTime))
              )
            {//;;check header
                SouthRxBitCounter = 8;//32;//8;	
                gettingSouthRx0or1 = 1;
                gettingSouthRxData = 0;
                SouthRxHighTimeCounter = 0;
                SouthRxLowTimeCounter = 0;                       
            }
            else
            {
                SouthRxLowTimeCounter = 0;  	//;;notheader£¬not 0 or 1£¬error
            }
        }
    }
    else
    {//Low come
        SouthRxLowTimeCounter++;
    }
//ing2    
    if(gotSouthRx8BitDataHappen==1)
    {
    	gotSouthRx8BitDataHappen=0;
    	return (SouthRxGotData);
    }	
    else
    {
    	return (0);
    }    
}
//-------Ir SouthRx----------

//-------Ir WestRx----------
unsigned int 	west_rx_per_per100us(void)
{
    int irWestRx_state = 0;	    

    if(irBitSendingCounters > 0)
    {//if sending ir, do not rec
      	gettingWestRx0or1 = 0;
      	WestRxHighTimeCounter = 0;
      	WestRxLowTimeCounter = 0;        
    }

    irWestRx_state = gpio_get_level(IrWestRx);	        	
    if(irWestRx_state == 1)
    {//High come
        if(WestRxLowTimeCounter < (1500/Imm_IrWestRxUnitTime))//1000us
        {//;;checking data 0_1
            if(gettingWestRx0or1 == 0)
            {//It is nosie,clear
                WestRxLowTimeCounter = 0;
            }
            else
            {//;;
      	        WestRxHighTimeCounter++;
      	        if(WestRxHighTimeCounter >= (5500/Imm_IrWestRxUnitTime))	
      	        {//more than 5.5ms if 3.4ms,error
      	            gettingWestRx0or1 = 0;
      	            WestRxHighTimeCounter = 0;
      	            WestRxLowTimeCounter = 0;
                    WestRxBitCounter = 0;
      	        }
      	        else//;;
      	        {
	  			    if(WestRxLowTimeCounter != 0)
	  			    {
	  			        if((WestRxLowTimeCounter >= (200/Imm_IrWestRxUnitTime)) && (WestRxLowTimeCounter < (1500/Imm_IrWestRxUnitTime)))//1000us
	  			        {
    			            WestRxLowTimeCounter = 0;
    			            gettingWestRxData <<= 1;
	  			            if(WestRxHighTimeCounter < (1800/Imm_IrWestRxUnitTime))
	  			            {//;;Got 0
	  					        //gettingWestRxData&=0x3FFFE;
	  						    gettingWestRxData &= 0xFFFFFFFE;
	  			            }
	  			            else
	  			            {//;;Got 1
	  						    gettingWestRxData |= 0x00000001;
	  			            }
	  			            WestRxHighTimeCounter = 0;
	  			            WestRxBitCounter--;
	  			            if(WestRxBitCounter == 0)	
	  			            {//Finished
	  						    gettingWestRx0or1 = 0;
	  						    WestRxHighTimeCounter = 0;
	  						    WestRxLowTimeCounter = 0;
	  						    gotWestRx8BitDataHappen = 1;
	  						    WestRxGotData = gettingWestRxData;  
                                if(WestRxGotData == 0x00)//debug1
                                {
                                    WestRxGotData = 0x54;
                                }
	  			            }
	  			        }
	  			    }
      	        }
            }
        }
        else
        {
            if(//666b
      	        ((5000/Imm_IrWestRxUnitTime) <= WestRxLowTimeCounter) &&
      	        (WestRxLowTimeCounter < (8000/Imm_IrWestRxUnitTime))
              )
            {//;;check header
                WestRxBitCounter = 8;//32;//8;	
                gettingWestRx0or1 = 1;
                gettingWestRxData = 0;
                WestRxHighTimeCounter = 0;
                WestRxLowTimeCounter = 0;
            }
            else
            {
                WestRxLowTimeCounter = 0;  	//;;notheader£¬not 0 or 1£¬error
            }
        }
    }
    else
    {//Low come
        WestRxLowTimeCounter++;
    }
//ing2    
    if(gotWestRx8BitDataHappen==1)
    {
    	gotWestRx8BitDataHappen=0;
    	return (WestRxGotData);
    }	
    else
    {
    	return (0);
    }    
}
//-------Ir WestRx----------

unsigned int 	read_cube_position_per100us(void)
{
	return cube_previous_position;	
}

unsigned int 	read_cube_changed_position_flag_per100us(void)
{
	if(cubePositionChangedHappen == 1)
	{
		cubePositionChangedHappen=0;
		return 1;
	}			
	else
	{
		return 0;	
	}			
}

unsigned int 	read_shake_happen_flag_per100us(void)
{
	if(cubeShakeHappen == 1)
	{
		cubeShakeHappen=0;
		return 1;
	}			
	else
	{
		return 0;	
	}			
}

unsigned int 	read_VolUpSwOn_flag_per100us(void)
{
	if(port1VolUpSwOnHappen == 1)
	{
		port1VolUpSwOnHappen=0;
		return 1;
	}			
	else
	{
		return 0;	
	}			
}

unsigned int 	read_VolDwSwOn_flag_per100us(void)
{
	if(port1VolDwSwOnHappen == 1)
	{
		port1VolDwSwOnHappen=0;
		return 1;
	}			
	else
	{
		return 0;	
	}			
}

unsigned int 	read_btn2SwOn_flag_per100us(void)
{
	if(port1btn2SwOnHappen == 1)
	{
		port1btn2SwOnHappen=0;
		return 1;
	}			
	else
	{
		return 0;	
	}			
}

unsigned int 	read_btn3SwOn_flag_per100us(void)
{
	if(port1btn3SwOnHappen == 1)
	{
		port1btn3SwOnHappen=0;
		return 1;
	}			
	else
	{
		return 0;	
	}			
}

unsigned int 	read_btn4SwOn_flag_per100us(void)
{
	if(port1btn4SwOnHappen == 1)
	{
		port1btn4SwOnHappen=0;
		return 1;
	}			
	else
	{
		return 0;	
	}			
}

unsigned int 	read_VusbSwOn_flag_per100us(void)
{
	if(port1VusbSwOnHappen == 1)
	{
		port1VusbSwOnHappen=0;
		return 1;
	}			
	else
	{
		return 0;	
	}			
}

unsigned int 	read_VusbSwOff_flag_per100us(void)
{
	if(port1VusbSwOffHappen == 1)
	{
		port1VusbSwOffHappen=0;
		return 1;
	}			
	else
	{
		return 0;	
	}			
}