#include <stdint.h>

#define	Imm_GrabItPosition_Xz														0
#define	Imm_GrabItPosition_Xf														1
#define	Imm_GrabItPosition_Yz														2
#define	Imm_GrabItPosition_Yf														3
#define	Imm_GrabItPosition_Zz														4
#define	Imm_GrabItPosition_Zf														5

#define	Imm_GrabItPosition_DebounceTime_First		1
#define	Imm_GrabItPosition_DebounceTime_AfterFirst	160//100

static 	uint16_t U16_t_GrabItPosition_DebounceTime=Imm_GrabItPosition_DebounceTime_First;

static  uint8_t	U8_GrabItPosition=0;

#define	Imm_Array_Number																(5+1)
static 	int16_t	U16_t_GrabItPosition_OnDebounce[Imm_Array_Number]={0};


//-------------------------New shake----------------------------
#define	Imm_Acce_Range	4//2	//2g=1	4g=2  8g=4  16g=8	//6663
						//4g		//8g		//4g 		//2g
#define Imm_NewShakeGSensorPulseValue	(20000)	//30000		//(20000)	//20000		//Before1207 //20000
#define Imm_NewShakeGSensorPulseTime	(30/5)	//50		//(30)		//30
static		uint16_t		U16_HoldAndDelayCheckNewShake=0; 

static		uint16_t		U16_NewShakeCounter=0;
static		uint16_t		U16_NewShakeTimeDelay=0;
static		uint16_t		U16_CountDownAndClearShakeRam=0;

static		uint16_t		U16_NewShakeZoutZhengOnDebounce=0;
static		uint16_t		U16_NewShakeZoutZhengOffDebounce=0;
static		uint16_t		U16_NewShakeZoutFanOnDebounce=0;
static		uint16_t		U16_NewShakeZoutFanOffDebounce=0;

static		uint16_t		U16_NewShakeYoutZhengOnDebounce=0;
static		uint16_t		U16_NewShakeYoutZhengOffDebounce=0;
static		uint16_t		U16_NewShakeYoutFanOnDebounce=0;
static		uint16_t		U16_NewShakeYoutFanOffDebounce=0;

static		uint16_t		U16_NewShakeXoutZhengOnDebounce=0;
static		uint16_t		U16_NewShakeXoutZhengOffDebounce=0;
static		uint16_t		U16_NewShakeXoutFanOnDebounce=0;
static		uint16_t		U16_NewShakeXoutFanOffDebounce=0;

static 		uint16_t 		U16_DelayAndThenCkNewShake=0;

static  	uint8_t			U8_PlayShakeHappen=0;

#define		Imm_ShakeUpDownTimes	6
//-------------------------New shake----------------------------

uint8_t	 CheckGrabItPosition(int16_t XOUT_Data, int16_t YOUT_Data, int16_t ZOUT_Data);
bool CheckNewShake(int16_t xAxisData, int16_t yAxisData, int16_t zAxisData);