#ifndef __BSP_H__
#define __BSP_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include "dusthawk_les/dh.h"
	
void bsp_init(void);
void bsp_led_on(void);
void bsp_led_off(void);
void bsp_delay(uint32_t);
void bsp_lcd_print_str(uint8_t r, uint8_t c, const char *str);
void bsp_lcd_print_char(uint8_t r, uint8_t c, const char chr);
void bsp_lcd_cursor(uint8_t r, uint8_t c, uint8_t csr);
void bsp_lcd_clear(void);
//void bsp_debug_print(const char* str);
uint32_t bsp_key_scanner(void);
	
int32_t bsp_eeprom_init(void);
void bsp_eeprom_read(uint32_t *pui32Data, uint32_t ui32Address, uint32_t ui32Count);
void bsp_eeprom_write(uint32_t *pui32Data, uint32_t ui32Address, uint32_t ui32Count);
	
void bsp_set_clock(calender_t *cal);
void bsp_get_clock(calender_t *cal);
void bsp_adc_get_mv(float *adc);

void bsp_valve_open(int32_t duty);
void bsp_valve_close(void);


void bsp_pump_on(void);
void bsp_pump_off(void);

void usb_status(void);
void bsp_enable_int_01_hz(void);
void bsp_disable_int_01_hz(void);
void bsp_enable_int_32_hz(void);
void bsp_disable_int_32_hz(void);


int bsp_mount_disk(int drv);
int bsp_unmount_disk(int drv);
int bsp_chk_disk(int drv);
void bsp_create_log(void);
int32_t bsp_delete_log(void);
void bsp_print_log(const char *format, ...);
void bsp_copy_log(void);
			
void bsp_test_sdc_and_pendrv(void);

void bsp_timer_capture_init(void);
	
#ifdef __cplusplus
}
#endif

#endif
