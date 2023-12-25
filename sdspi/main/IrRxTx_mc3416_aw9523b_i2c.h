#include "esp_err.h"
#include <stdint.h>

//static const char *TAG = "IrTx3416And9523";

#include "soc/soc.h"
#include "driver/timer.h"
#include "esp_clk_tree.h"

#include <string.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"


#define SUB_CUBE4//MAIN_CUBE,SUB_CUBE1,SUB_CUBE2,SUB_CUBE3,SUB_CUBE4


//-------i2c-----------------
#define MC3416_XOUT_EX_L_REG_ADDR          0x0D                 /*!< Register addresses of the "who am I" register */
#define MC3416_XOUT_EX_H_REG_ADDR          0x0E                 /*!< Register addresses of the "who am I" register */

#define MC3416_YOUT_EX_L_REG_ADDR          0x0F                 /*!< Register addresses of the "who am I" register */
#define MC3416_YOUT_EX_H_REG_ADDR          0x10                 /*!< Register addresses of the "who am I" register */

#define MC3416_ZOUT_EX_L_REG_ADDR          0x11                 /*!< Register addresses of the "who am I" register */
#define MC3416_ZOUT_EX_H_REG_ADDR          0x12                 /*!< Register addresses of the "who am I" register */


#define I2C_MASTER_NUM              0                          /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define MC3416_SENSOR_ADDR                 0x4C//0x68          /*!< Slave address of the MC3416 sensor */
#define I2C_MASTER_TIMEOUT_MS       1000

#define AW9523B_SENSOR_ADDR             (0x5B)	        		/*!< Slave address of the AW9523B sensor */

#define I2C_MASTER_SCL_IO           22                         /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           21                         /*!< GPIO number used for I2C master data  */

#define I2C_MASTER_FREQ_HZ          400000                     /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */

#define MC3416_WHO_AM_I_REG_ADDR           0X18//0x75          /*!< Register addresses of the "who am I" register */
#define MC3423_WHO_AM_I_REG_ADDR           0X3B                /*!< Register addresses of the "who am I" register */

static uint8_t data_i2c[6] = {0}; //2

#define AW9523B_WHO_AM_I_REG_ADDR       (0x10)        			/*!< Register addresses of the "who am I" register */

#define INPUT_PORT0                     (0x00)//R   Equal to P0 Input_Port0 P0 port input state
#define INPUT_PORT1                     (0x01)//R   Equal to P1 Input_Port1 P1 port input state
#define OUTPUT_PORT0                    (0x02)//W/R Refer to table 1 Output_Port0 P0 port output state
#define OUTPUT_PORT1                    (0x03)//W/R Refer to table 1 Output_Port1 P1 port output state
#define CONFIG_PORT0                    (0x04)// W/R 00H Config_Port0 P0 port direction configure
#define CONFIG_PORT1                    (0x05)//W/R 00H Config_Port1 P1 port direction configure
#define INT_PORT0                       (0x06)//W/R 00H Int_Port0 P0 port interrupt enable
#define INT_PORT1                       (0x07)// W/R 00H Int_Port1 P1 port interrupt enable
#define ID                              (0x10)// R 23H ID ID register (read only)
#define GCR                             (0x11)//W/R 00H CTL Global control register
//-------i2c-----------------

static int16_t XOUT_Data = 0;
static int16_t YOUT_Data = 0;
static int16_t ZOUT_Data = 0;

static unsigned int cube_previous_position = 0;
static unsigned int cubePositionChangedHappen = 0;

static unsigned int cubeShakeHappen = 0;

static uint8_t port0InputValue = 0;
static uint8_t port1InputValue = 0;

#define port1VolUpSwBit    0x01
static unsigned int  port1VolUpSwOnHappen = 0;

#define port1VolDwSwBit    0x02
static unsigned int  port1VolDwSwOnHappen = 0;

#define port1btn2SwBit    0x04
static unsigned int  port1btn2SwOnHappen = 0;

#define port1btn3SwBit    0x08
static unsigned int  port1btn3SwOnHappen = 0;

#define port1btn4SwBit    0x10
static unsigned int  port1btn4SwOnHappen = 0;

#define port1VusbSwBit    0x20
static unsigned int  port1VusbSwOnHappen = 0;
static unsigned int  port1VusbSwOffHappen = 0;

#define port0StdbySwBit    0x20

#define port0ChrgSwBit    0x40

static esp_err_t MC3416_register_read(uint8_t reg_addr, uint8_t *data, size_t len);
static esp_err_t MC3416_register_write_byte(uint8_t reg_addr, uint8_t data);

//-------Ir NorthRx----------	
#define IrNorthRx           32
#define	Imm_IrNorthRxUnitTime	100
static  unsigned int	        NorthRxHighTimeCounter=0;	
static	unsigned int		    NorthRxLowTimeCounter=0;
static	unsigned int			NorthRxBitCounter=0;		
static	unsigned int			gettingNorthRx0or1=0;	
static	unsigned int 	gettingNorthRxData=0;	
static	unsigned int 	NorthRxGotData=0;	
static	unsigned int			gotNorthRx8BitDataHappen=0;
//-------Ir NorthRx----------

//-------Ir EastRx----------	
#define IrEastRx            33
#define	Imm_IrEastRxUnitTime	100
static  unsigned int	        EastRxHighTimeCounter=0;	
static	unsigned int		    EastRxLowTimeCounter=0;
static	unsigned int			EastRxBitCounter=0;		
static	unsigned int			gettingEastRx0or1=0;	
static	unsigned int 	gettingEastRxData=0;	
static	unsigned int 	EastRxGotData=0;	
static	unsigned int			gotEastRx8BitDataHappen=0;
//-------Ir EastRx----------

//-------Ir SouthRx----------	
#define IrSouthRx           25
#define	Imm_IrSouthRxUnitTime	100
static  unsigned int	        SouthRxHighTimeCounter=0;	
static	unsigned int		    SouthRxLowTimeCounter=0;
static	unsigned int			SouthRxBitCounter=0;		
static	unsigned int			gettingSouthRx0or1=0;	
static	unsigned int 	gettingSouthRxData=0;	
static	unsigned int 	SouthRxGotData=0;	
static	unsigned int			gotSouthRx8BitDataHappen=0;
//-------Ir SouthRx----------

//-------Ir WestRx----------	
#define IrWestRx            13
#define	Imm_IrWestRxUnitTime	100
static  unsigned int	        WestRxHighTimeCounter=0;	
static	unsigned int		    WestRxLowTimeCounter=0;
static	unsigned int			WestRxBitCounter=0;		
static	unsigned int			gettingWestRx0or1=0;	
static	unsigned int 	gettingWestRxData=0;	
static	unsigned int 	WestRxGotData=0;	
static	unsigned int			gotWestRx8BitDataHappen=0;
//-------Ir WestRx----------
	
unsigned int 	north_rx_per_per100us(void);
unsigned int 	east_rx_per_per100us(void);
unsigned int 	south_rx_per_per100us(void);
unsigned int 	west_rx_per_per100us(void);

unsigned int 	read_cube_position_per100us(void);
unsigned int 	read_cube_changed_position_flag_per100us(void);
unsigned int 	read_shake_happen_flag_per100us(void);

unsigned int 	read_VolUpSwOn_flag_per100us(void);
unsigned int 	read_VolDwSwOn_flag_per100us(void);
unsigned int 	read_btn2SwOn_flag_per100us(void);
unsigned int 	read_btn3SwOn_flag_per100us(void);
unsigned int 	read_btn4SwOn_flag_per100us(void);
unsigned int 	read_VusbSwOn_flag_per100us(void);
unsigned int 	read_VusbSwOff_flag_per100us(void);

void mc3416_aw9523b_init_withI2c(void);
void SendIr_ReadMc3416_ReadAw9523bSw_task(void *arg);