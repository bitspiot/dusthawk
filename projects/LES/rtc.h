#ifndef __DS3231_H__
#define __DS3231_H__

#include <stdbool.h>
#include <stdint.h>

//*****************************************************************************
//
// The I2C slave address of the AT24C08A EEPROM device.  This address is based
// on the A2 pin of the AT24C08A being pulled high on the board.
//
//*****************************************************************************
#define RTC_ADDR              0x68

//*****************************************************************************
//
// The states in the interrupt handler state machine.
//
//*****************************************************************************
#define RTC_SEC        0x00  
#define RTC_MIN        0x01  
#define RTC_HOUR       0x02
#define RTC_WDAY       0x03
#define RTC_MDAY       0x04
#define RTC_MONTH      0x05
#define RTC_YEAR       0x06

#define AL1SEC_REG     0x07
#define AL1MIN_REG     0x08
#define AL1HOUR_REG    0x09
#define AL1WDAY_REG    0x0A

#define AL2MIN_REG     0x0B
#define AL2HOUR_REG    0x0C
#define AL2WDAY_REG    0x0D

#define CONTROL_REG          0x0E
#define STATUS_REG           0x0F
#define AGING_OFFSET_REG     0x0F
#define TMP_UP_REG           0x11
#define TMP_LOW_REG          0x12


void rtc_init(void);
void RtcByteWrite(uint8_t addr, uint8_t data);
uint8_t RtcGet(uint8_t param);
void RtcSet(uint8_t param,uint8_t value);
void RtcAlarmEnable(void);
void RtcStatusClear(void);

void ds3231_enable_sqw(void);
void ds3231_disable_sqw(void);
void ds3231_enable_32k(void);
void ds3231_disable_32k(void);
		
#endif

