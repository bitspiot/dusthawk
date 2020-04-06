#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "inc/hw_gpio.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_nvic.h"
#include "inc/hw_ints.h"
#include "driverlib/gpio.h"
#include "driverlib/ssi.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "utils/uartstdio.h"
#include "utils/ustdlib.h"
#include "driverlib/eeprom.h"
#include "driverlib/adc.h"
#include "driverlib/pwm.h"
#include "driverlib/timer.h"

#include "driverlib/udma.h"
#include "usblib/usblib.h"
#include "usblib/usbmsc.h"
#include "usblib/host/usbhost.h"
#include "usblib/host/usbhmsc.h"
#include "utils/uartstdio.h"
#include "utils/ustdlib.h"
#include "fatfs/src/ff.h"
#include "fatfs/src/diskio.h"

#include "bsp.h"
#include "lcd.h"
#include "rtc.h"
#include "buttons.h"
#include "dusthawk_les/dh.h"

#define TICKS_PER_SECOND 				100
#define MS_PER_SYSTICK 					(1000 / TICKS_PER_SECOND)
#define USBMSC_DRIVE_RETRY      4
#define HCD_MEMORY_SIZE         128

uint8_t g_pHCDPool[HCD_MEMORY_SIZE];
tUSBHMSCInstance *g_psMSCInstance = 0;

DECLARE_EVENT_DRIVER(g_sUSBEventDriver, 0, 0, USBHCDEvents);

static tUSBHostClassDriver const * const g_ppHostClassDrivers[] =
{
    &g_sUSBHostMSCClassDriver,
    &g_sUSBEventDriver
};

typedef enum
{
	STATE_NO_DEVICE,
	STATE_DEVICE_ENUM,
	STATE_DEVICE_READY,
	STATE_UNKNOWN_DEVICE,
	STATE_TIMEOUT_DEVICE,
	STATE_POWER_FAULT
}tState;

static const uint32_t g_ui32NumHostClassDrivers = sizeof(g_ppHostClassDrivers) / sizeof(tUSBHostClassDriver *);

#if defined(ewarm)
#pragma data_alignment=1024
tDMAControlTable g_sDMAControlTable[6];
#elif defined(ccs)
#pragma DATA_ALIGN(g_sDMAControlTable, 1024)
tDMAControlTable g_sDMAControlTable[6];
#else
tDMAControlTable g_sDMAControlTable[6] __attribute__ ((aligned(1024)));
#endif


#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line){
}
#endif


static FATFS g_sFatFs[2];

volatile tState g_eState;
volatile tState g_eUIState;

tUSBMode g_eCurrentUSBMode;
uint32_t g_ui32UnknownClass;

uint32_t g_ui32SysTickCount;
uint32_t g_ui32LastTick;

void MSCCallback(tUSBHMSCInstance *ps32Instance, uint32_t ui32Event, void *pvData)
{
    switch(ui32Event)
    {
        case MSC_EVENT_OPEN:
        {
						UARTprintf("MSC_EVENT_OPEN\n");
            g_eState = STATE_DEVICE_ENUM;
            break;
        }
				
        case MSC_EVENT_CLOSE:
        {
					UARTprintf("MSC_EVENT_CLOSE\n");
					g_eState = STATE_NO_DEVICE;
					break;
        }

        default:
        {
					UARTprintf("default\n");
            break;
        }
    }
}

void USBHCDEvents(void *pvData)
{
    tEventInfo *pEventInfo;
    pEventInfo = (tEventInfo *)pvData;
	
    switch(pEventInfo->ui32Event)
    {
        case USB_EVENT_UNKNOWN_CONNECTED:
        {
            UARTprintf("USB_EVENT_UNKNOWN_CONNECTED\n");
            g_ui32UnknownClass = pEventInfo->ui32Instance;
            g_eState = STATE_UNKNOWN_DEVICE;
            break;
        }
        case USB_EVENT_DISCONNECTED:
        {
            UARTprintf("USB_EVENT_DISCONNECTED\n");
            g_eState = STATE_NO_DEVICE;
            break;
        }
        case USB_EVENT_POWER_FAULT:
        {
            UARTprintf("USB_EVENT_POWER_FAULT\n");
            g_eState = STATE_POWER_FAULT;
            break;
        }
        default:
        {
            UARTprintf("default\n");
            break;
        }
    }
}

uint32_t GetTickms(void)
{
    uint32_t ui32RetVal;
    uint32_t ui32Saved;

    ui32RetVal = g_ui32SysTickCount;
    ui32Saved = ui32RetVal;

    if(ui32Saved > g_ui32LastTick)
    {
        ui32RetVal = ui32Saved - g_ui32LastTick;
    }
    else
    {
        ui32RetVal = g_ui32LastTick - ui32Saved;
    }
		
    g_ui32LastTick = ui32Saved;
		
    return(ui32RetVal * 10);
}


void usb_status(void){
	
	char buff[128];
	uint32_t ui32DriveTimeout;
	
	if(g_eState == STATE_DEVICE_ENUM)
		{
			if(USBHMSCDriveReady(g_psMSCInstance) != 0)
			{
                SysCtlDelay(SysCtlClockGet()/(3*2));
                ui32DriveTimeout--;
				
				if(ui32DriveTimeout == 0)
				{
						g_eState = STATE_TIMEOUT_DEVICE;
				}
            }
			
			g_eState = STATE_DEVICE_READY;
		}
		USBOTGMain(GetTickms());
}

void SysTickIntHandler(void)
{
	disk_timerproc();
	g_ui32SysTickCount++;
}

void GPIODIntHandler(void)
{
	GPIOIntClear(GPIO_PORTD_BASE,GPIO_PIN_6);
	dusthawk_isr_1_hz();
}

void bsp_init(void)
{
	
	//USB
	uint32_t ui32DriveTimeout = USBMSC_DRIVE_RETRY;
	g_eState = STATE_NO_DEVICE;
	g_eUIState = STATE_NO_DEVICE;
	ROM_FPULazyStackingEnable();
	
	ROM_SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |SYSCTL_XTAL_16MHZ);
	// GPIO
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF))
	{
	
	}
	
	ROM_GPIOPinConfigure(GPIO_PC6_USB0EPEN);
	
	GPIOPinTypeUSBAnalog(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);
	// DP+DN
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);            
	GPIOPinTypeUSBAnalog(GPIO_PORTD_BASE, GPIO_PIN_4 | GPIO_PIN_5);

	ROM_SysTickPeriodSet(ROM_SysCtlClockGet()/ TICKS_PER_SECOND);
	ROM_SysTickEnable();
	ROM_SysTickIntEnable();
	
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UDMA);
	ROM_uDMAEnable();
	ROM_uDMAControlBaseSet(g_sDMAControlTable);
	
	USBStackModeSet(0, eUSBModeHost, 0);
	
	USBHCDRegisterDrivers(0, g_ppHostClassDrivers, g_ui32NumHostClassDrivers);
	
	g_psMSCInstance = USBHMSCDriveOpen(0, MSCCallback);
	
	USBHCDPowerConfigInit(0, USBHCD_VBUS_AUTO_LOW | USBHCD_VBUS_FILTER);
	
	USBHCDInit(0, g_pHCDPool, HCD_MEMORY_SIZE);
		
	//LCD Pin Configuration
	ROM_GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE,GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7);
	ROM_GPIOPadConfigSet(GPIO_PORTA_BASE,GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7,GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
	//LCD EN2 Configuration
	ROM_GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE,GPIO_PIN_4);
	ROM_GPIOPadConfigSet(GPIO_PORTB_BASE,GPIO_PIN_4,GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

	//UART Configuration 
	ROM_GPIOPinConfigure(GPIO_PA0_U0RX);
  ROM_GPIOPinConfigure(GPIO_PA1_U0TX);
	ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
	UARTStdioConfig(0, 115200, ROM_SysCtlClockGet());
	
	//This pin will be used for RTC interrupt 
	ROM_GPIOPinTypeGPIOInput(GPIO_PORTD_BASE,GPIO_PIN_6);
	ROM_GPIOPadConfigSet(GPIO_PORTD_BASE,GPIO_PIN_6,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU);
	ROM_GPIOIntTypeSet(GPIO_PORTD_BASE,GPIO_PIN_6,GPIO_FALLING_EDGE);
	GPIOIntEnable(GPIO_PORTD_BASE,GPIO_PIN_6);
	// Enable the GPIO port that is used for the on-board LED.
	
	
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI3);
	GPIOPinConfigure(GPIO_PD0_SSI3CLK);
	GPIOPinConfigure(GPIO_PD1_SSI3FSS);
	GPIOPinConfigure(GPIO_PD2_SSI3RX);
	GPIOPinConfigure(GPIO_PD3_SSI3TX);
	GPIOPinTypeSSI(GPIO_PORTD_BASE,GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);
	
	// Pump Init PF2
	ROM_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE,GPIO_PIN_2);
	ROM_GPIOPadConfigSet(GPIO_PORTF_BASE,GPIO_PIN_2,GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPD);
	// FAN Init PB5
	ROM_GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE,GPIO_PIN_5);
	ROM_GPIOPadConfigSet(GPIO_PORTB_BASE,GPIO_PIN_5,GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPD);
	// PWM init PF3
	
	SysCtlPWMClockSet(SYSCTL_PWMDIV_1);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	GPIOPinConfigure(GPIO_PF3_M1PWM7);
	GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_3);
	PWMGenConfigure(PWM1_BASE, PWM_GEN_3, PWM_GEN_MODE_UP_DOWN |PWM_GEN_MODE_NO_SYNC);
  PWMGenPeriodSet(PWM1_BASE, PWM_GEN_3,60000);
	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7,0);
	PWMDeadBandDisable(PWM1_BASE, PWM_GEN_3);
  PWMGenEnable(PWM1_BASE, PWM_GEN_3);
	PWMOutputState(PWM1_BASE, PWM_OUT_7_BIT, false);
	
	// Check if the peripheral access is enabled.
	
	SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_0);
	GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_1);
	GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_2);
	GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3);
	GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_4);
	GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_5);

	ADCSequenceConfigure(ADC0_BASE, 0, ADC_TRIGGER_PROCESSOR, 0); // Sequencer 
	ADCSequenceStepConfigure(ADC0_BASE, 0, 0, ADC_CTL_CH0);
	ADCSequenceStepConfigure(ADC0_BASE, 0, 1, ADC_CTL_CH1);
	ADCSequenceStepConfigure(ADC0_BASE, 0, 2, ADC_CTL_CH2);
	ADCSequenceStepConfigure(ADC0_BASE, 0, 3, ADC_CTL_CH3);
	ADCSequenceStepConfigure(ADC0_BASE, 0, 4, ADC_CTL_CH8);
	ADCSequenceStepConfigure(ADC0_BASE, 0, 5, ADC_CTL_CH9);
	ADCSequenceStepConfigure(ADC0_BASE, 0, 6, ADC_CTL_CH9);
	ADCSequenceStepConfigure(ADC0_BASE, 0, 7, ADC_CTL_CH9 | ADC_CTL_IE | ADC_CTL_END);
	ADCHardwareOversampleConfigure(ADC0_BASE,64);
	ADCSequenceEnable(ADC0_BASE, 0);
	ADCIntClear(ADC0_BASE, 0);

	//
	// Enable the GPIO pin for the LED (PF3).  Set the direction as output, and
	// enable the GPIO pin for digital function.
	//
	GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1);
	bsp_timer_capture_init();
	lcd_init();
	lcd_clr();
	rtc_init();
	ds3231_enable_sqw();
	ds3231_enable_32k();
//	ds3231_disable_32k();
	buttons_init();
	bsp_eeprom_init();
	IntMasterEnable();
}


void bsp_led_on(void)
{
	 GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1,GPIO_PIN_1);
}

void bsp_led_off(void)
{
	 GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1,~GPIO_PIN_1);
}

void bsp_delay(uint32_t usec)
{
	SysCtlDelay(usec);
}

void bsp_lcd_print_str(uint8_t r, uint8_t c, const char *str)
{
	lcd_locate(r,c);
	lcd_puts(str);
}

void bsp_lcd_print_char(uint8_t r, uint8_t c, const char chr)
{
	lcd_locate(r,c);
	lcd_putc(chr);
}

void bsp_lcd_cursor(uint8_t r, uint8_t c, uint8_t csr)
{
	lcd_locate(r,c);
	
	if(csr==CSR_BLOCK)
		lcd_cursor(CSR_BLOCK);
	else if(csr==CSR_UNDER)
		lcd_cursor(CSR_UNDER);
	else
		lcd_cursor(CSR_BLOCK);	
}

void bsp_lcd_clear(void)
{
	lcd_clr();
}


uint32_t bsp_key_scanner(void)
{
	USBOTGMain(GetTickms());
	return buttons_poll(0,0);
}


int32_t bsp_eeprom_init(void){
	
	uint32_t ui32EEPROMInit;
	
	SysCtlPeripheralEnable(SYSCTL_PERIPH_EEPROM0);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_EEPROM0);
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_EEPROM0))
	{
		//wait 
	}
	ui32EEPROMInit = EEPROMInit();
	
	if(ui32EEPROMInit == EEPROM_INIT_OK)
	{
		return 0;
	}
	return -1;
}

void bsp_eeprom_read(uint32_t *pui32Data, uint32_t ui32Address, uint32_t ui32Count)
{
	EEPROMRead(pui32Data,ui32Address,ui32Count);
}

void bsp_eeprom_write(uint32_t *pui32Data, uint32_t ui32Address, uint32_t ui32Count)
{
	EEPROMProgram(pui32Data,ui32Address,ui32Count);
}


void bsp_set_clock(calender_t *cal){
	
	RtcSet(RTC_MDAY,cal->dd);
	RtcSet(RTC_MONTH,cal->mm);
	RtcSet(RTC_YEAR,cal->yy);
	RtcSet(RTC_HOUR,cal->h);
	RtcSet(RTC_MIN,cal->m);
	RtcSet(RTC_SEC,cal->s);

}

void bsp_get_clock(calender_t *cal){

	cal->dd=(uint32_t)RtcGet(RTC_MDAY);
	cal->mm=(uint32_t)RtcGet(RTC_MONTH);
	cal->yy=(uint32_t)RtcGet(RTC_YEAR);
	cal->h=(uint32_t)RtcGet(RTC_HOUR);
	cal->m=(uint32_t)RtcGet(RTC_MIN);
	cal->s=(uint32_t)RtcGet(RTC_SEC);	
}


/*

*/

#define ADC_MV_FACTOR	(0.805860f)
#define SCALE_FACTOR	(2.466666f)
#define CONV_FACTOR		(ADC_MV_FACTOR * SCALE_FACTOR)


void bsp_adc_get_mv(float *adc){
	
	uint32_t pui32ADC0Value[8];
	
	// trigger the processor to get sample
	ADCProcessorTrigger(ADC0_BASE, 0);
	while(!ADCIntStatus(ADC0_BASE, 0, false)){
		// Wait while coversion is in progress
	}
	// Clear the interuppt
	ADCIntClear(ADC0_BASE, 0);
	// Get the Data
	ADCSequenceDataGet(ADC0_BASE, 0, pui32ADC0Value);
	//Convert count to milivolt
	adc[0]=pui32ADC0Value[0]*CONV_FACTOR;
	adc[1]=pui32ADC0Value[1]*CONV_FACTOR;
	adc[2]=pui32ADC0Value[2]*CONV_FACTOR;
	adc[3]=pui32ADC0Value[3]*CONV_FACTOR;
	adc[4]=pui32ADC0Value[4]*CONV_FACTOR;
	adc[5]=pui32ADC0Value[5]*CONV_FACTOR;
	adc[6]=pui32ADC0Value[6]*CONV_FACTOR;
	adc[7]=pui32ADC0Value[7]*CONV_FACTOR;
	
}

void bsp_pump_on(void){
	GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_5,GPIO_PIN_5);
}

void bsp_pump_off(void){
	GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_5,~GPIO_PIN_5);
}

void WTimer5AIntHandler(void){
	ROM_TimerIntClear(WTIMER5_BASE,TIMER_CAPA_MATCH);
	UARTprintf("WTimer5AIntHandler Event \n");
	ROM_TimerEnable(WTIMER5_BASE,TIMER_A);
	
}

void WTimer5BIntHandler(void){
	ROM_TimerIntClear(WTIMER5_BASE,TIMER_CAPB_MATCH);
	ROM_TimerEnable(WTIMER5_BASE,TIMER_B);
	dusthawk_isr_32_hz();
}



void bsp_timer_capture_init(void){
	
    SysCtlPeripheralEnable(SYSCTL_PERIPH_WTIMER5);

    //	GPIOPinTypeTimer(GPIO_PORTD_BASE,GPIO_PIN_6);
    //	GPIOPinConfigure(GPIO_PD6_WT5CCP0);
    //	GPIOPadConfigSet(GPIO_PORTD_BASE, GPIO_PIN_6,GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
    //	
    //	TimerConfigure(WTIMER5_BASE,(TIMER_CFG_SPLIT_PAIR|TIMER_CFG_A_CAP_COUNT));
    //	TimerControlEvent(WTIMER5_BASE, TIMER_A, TIMER_EVENT_NEG_EDGE);
    //	TimerLoadSet(WTIMER5_BASE, TIMER_A, 1);
    //  TimerMatchSet(WTIMER5_BASE, TIMER_A, 0);
    //	//TimerIntRegister(WTIMER5_BASE,TIMER_A,WTimer5AIntHandler);
    //  TimerIntEnable(WTIMER5_BASE, TIMER_CAPA_MATCH);
    //	TimerEnable(WTIMER5_BASE, TIMER_A);
    //	ROM_TimerIntClear(WTIMER5_BASE,TIMER_CAPA_MATCH);
    //	IntEnable(INT_WTIMER5A);


    HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(GPIO_PORTD_BASE + GPIO_O_CR) |= 0x80;
    HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = 0;

    GPIOPinTypeTimer(GPIO_PORTD_BASE,GPIO_PIN_7);
    GPIOPinConfigure(GPIO_PD7_WT5CCP1);
    GPIOPadConfigSet(GPIO_PORTD_BASE, GPIO_PIN_7,GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

    TimerConfigure(WTIMER5_BASE,(TIMER_CFG_SPLIT_PAIR|TIMER_CFG_B_CAP_COUNT));
    TimerControlEvent(WTIMER5_BASE, TIMER_B, TIMER_EVENT_POS_EDGE);
    // 64 HZ= 32768/512
    TimerLoadSet(WTIMER5_BASE, TIMER_B,512);
    TimerMatchSet(WTIMER5_BASE, TIMER_B, 0);
    TimerIntRegister(WTIMER5_BASE,TIMER_B,WTimer5BIntHandler);
    bsp_disable_int_32_hz();
	
}

void bsp_valve_open(int32_t duty){
	PWMOutputState(PWM1_BASE, PWM_OUT_7_BIT, true);
	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7,duty);
}

void bsp_valve_close(){
	PWMOutputState(PWM1_BASE, PWM_OUT_7_BIT, false);
}


void bsp_enable_int_01_hz(void){
	IntEnable(INT_GPIOD);
	GPIOIntClear(GPIO_PORTD_BASE,GPIO_PIN_6);
	
}	
	
void bsp_disable_int_01_hz(void){
	GPIOIntClear(GPIO_PORTD_BASE,GPIO_PIN_6);
	IntDisable(INT_GPIOD);
}

void bsp_enable_int_32_hz(void){
	TimerEnable(WTIMER5_BASE, TIMER_B);
	TimerIntEnable(WTIMER5_BASE, TIMER_CAPB_MATCH);
	IntEnable(INT_WTIMER5B);
	ROM_TimerIntClear(WTIMER5_BASE,TIMER_CAPB_MATCH);
}

void bsp_disable_int_32_hz(void){
	TimerDisable(WTIMER5_BASE, TIMER_B);
	TimerIntDisable(WTIMER5_BASE, TIMER_CAPB_MATCH);
	IntDisable(INT_WTIMER5B);
}


////////////////DISK IO Wrappers///////////////////////////////////////////

FATFS fs[2];         /* Work area (file system object) for logical drives */



int bsp_mount_disk(int32_t drv){
	return f_mount(drv, &fs[drv]);
}

int bsp_unmount_disk(int32_t drv){
	return f_mount(drv,NULL);
}

int bsp_chk_disk(int32_t drv){
	disk_initialize(drv);
	return disk_status(drv);
}

void bsp_create_log(void){
	FIL fil;      				/* file object */
	FRESULT res;         /* FatFs function common result code */
	UINT br, bw;         /* File read/write count */
	
	if(f_open(&fil, "0:LOG.CSV", FA_OPEN_EXISTING | FA_WRITE)!=FR_OK)
	{
		if(f_open(&fil, "0:LOG.CSV", FA_CREATE_ALWAYS | FA_WRITE)==FR_OK){
			f_write(&fil,"DD-MM-YYYY,hh:mm:ss,AT(deg_c),FT(deg_c),FP(mmhg),BP(mbar),FR(lpm),CV(%),TV(m3),ET(hh:mm),\n", 90, &bw);
			f_close(&fil);
		}
	}
}

int32_t bsp_delete_log(void){
    FRESULT res;
    res=f_unlink("0:LOG.CSV");
    if(res==FR_OK)
    {
        UARTprintf("FILE DELETED SUCCESS..\n");
    }
    
    else
    {
        UARTprintf("FILE DELETED FAILURE:%d\n",(int)res);
    }
    
    return 0;
}


void bsp_print_log(const char *format, ...){
	va_list  args;
	char  buf[128];
	FIL fil;      				/* file object */
	FRESULT res;         /* FatFs function common result code */
	UINT br, bw;         /* File read/write count */
	
	va_start(args, format);
	
	vsprintf(buf, format, args);
	
	br=strlen(buf);
	
	UARTprintf("%s",buf);
	
	res = f_open(&fil, "0:LOG.CSV", FA_OPEN_EXISTING | FA_WRITE);
	
	if(res==FR_OK){
		
        UARTprintf("FILE OPENED OK..\n");
        
		f_lseek(&fil,f_size(&fil));
		
		res = f_write(&fil, buf, br, &bw);
		
		if(br==bw){
			//handle error here
			
		}
		
		f_close(&fil);
	}
	
	va_end(args);
}

void bsp_copy_log(void){
	FIL fsrc, fdst;      /* file objects */
	BYTE buffer[512];    /* file copy buffer */
	FRESULT res;         /* FatFs function common result code */
	UINT br, bw;         /* File read/write count */
    
	//f_mount(1, &fs[1]);
	
	/* Open source file on the drive 1 */
    res = f_open(&fsrc, "0:LOG.CSV", FA_OPEN_EXISTING | FA_READ);
	
	if(res==FR_OK){
		UARTprintf("SD CARD FILE OPENED\n");
	}
    
	res = f_open(&fdst, "1:MFC.CSV", FA_CREATE_ALWAYS | FA_WRITE);

	if(res==FR_OK){
		UARTprintf("USB FILE OPENED\n");
	}
	
	for (;;) 
	{
		UARTprintf("COPYING DATA..\n");
        res = f_read(&fsrc, buffer, sizeof(buffer), &br);    /* Read a chunk of src file */
        if (res || br == 0) break; /* error or eof */
        res = f_write(&fdst, buffer, br, &bw);               /* Write it to the dst file */
        if (res || bw < br) break; /* error or disk full */
		USBOTGMain(GetTickms());
    }
	
	f_close(&fsrc);
    f_close(&fdst);
    
	f_mount(1, NULL);
}