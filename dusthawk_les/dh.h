#ifndef __DH_H__
#define __DH_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>

typedef struct{
	float sp;
	float er;
	float fb;
	int 	adjst;
	int 	duty;
}plant_t;

typedef struct{
	int fr;
	int ft;
	int fp;
	int at;
	int bp;
}acc_t;



typedef struct{
	char dd;
	char mm;
	char yy;
	char h;
	char m;
	char s;
}calender_t;

typedef struct{
	float sum_of_squares;
	float sum;
	int   block_size;
}cv_t;


typedef struct{
	float mul;
	float off;	
}calib_t;

typedef struct{
	calib_t fr;
	calib_t ft;
	calib_t fp;
	calib_t at;
	calib_t bp;
	float shdn_fr;
	float shdn_fp;
	int leak_chk_limit;
	int buffer_size;
}calibration_t;

typedef struct{
	float tv;
	float afr;
	float ifr;
	float afp;
	float ifp;
	int    et;
	float cv;
	cv_t  cv_parm;
    int   err_code;
    int   new_file;
	calender_t start_cal;
    calender_t stop_cal;
}summary_t;

typedef struct{
	float set_fr;
	int rec_int;
	int comp;
    int   new_file;
	char f_id[16];
}user_settings_t;


typedef struct{
	int auto_mode;
	int sec_to_run;
	calender_t start_cal;
	calender_t stop_cal;
}auto_mode_t;


typedef struct{
	unsigned int fsm_state;
}fsm_t;


typedef struct
{
	int32_t i32Size;
	void *pvParameters;
	void (*pfnIndicatorOn)(void);
	void (*pfnIndicatorOff)(void);
	void (*pfnIndicatorToggle)(void);
	void (*pfnPumpOn)(void);
	void (*pfnPumpOff)(void);
	void (*pfnValveOpen)(int32_t duty);
	void (*pfnValveClose)(void);
	void (*pfnDelay)(uint32_t usec);
	void (*pfnLcdPrintStr)(uint8_t r, uint8_t c, const char *str);
	void (*pfnLcdPrintChr)(uint8_t r, uint8_t c, const char chr);
	void (*pfnLcdCursor)(uint8_t r, uint8_t c, uint8_t csr);
	void (*pfnLcdClear)(void);
	void (*pfnDebugPrint)(const char *pcString, ...);
	void (*pfnTickCallback)(void);
	uint32_t (*pfnKeyScan)(void);
	void (*pfnMemRead)(uint32_t *pui32Data, uint32_t ui32Address, uint32_t ui32Count);
	void (*pfnMemWrite)(uint32_t *pui32Data, uint32_t ui32Address, uint32_t ui32Count);
	void (*pfnGetClock)(calender_t *cal);
	void (*pfnSetClock)(calender_t *cal);
	void (*pfnGetAdcMv)(float *adc);
	void (*pfnUSBCallback)(void);
	void (*pfnIntEnable01Hz)(void);
	void (*pfnIntDisable01Hz)(void);
	void (*pfnIntEnable32Hz)(void);
	void (*pfnIntDisable32Hz)(void);
	int32_t (*pfnMountDisk)(int32_t dsk);
	int32_t (*pfnUnMountDisk)(int32_t dsk);
	int32_t (*pfnCheckDisk)(int32_t dsk);
	void (*pfnCreateLogFile)(void);
	int32_t (*pfnDeleteLogFile)(void);
	void (*pfnPrintLogFile)(const char *format, ...);
	void (*pfnCpoyLogFile)(void);
}dusthawk_t;


typedef struct 
{
  uint32_t (*Task)(void);   // output function
  unsigned long Next[10];
}state_t;


void dusthawk_isr_1_hz(void);
void dusthawk_isr_32_hz(void);
void dusthawk_init(dusthawk_t *);
void dusthawk_start(void);


#ifdef __cplusplus
}
#endif

#endif

