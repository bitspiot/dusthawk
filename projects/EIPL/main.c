#include "bsp.h"
#include "dusthawk/dh.h"
#include "utils/uartstdio.h"


dusthawk_t dusthawk=
{
	sizeof(dusthawk_t),
	0,
	bsp_led_on,
	bsp_led_off,
	0,
	bsp_pump_on,
	bsp_pump_off,
	bsp_valve_open,
	bsp_valve_close,
	bsp_delay,
	bsp_lcd_print_str,
	bsp_lcd_print_char,
	bsp_lcd_cursor,
	bsp_lcd_clear,
	UARTprintf,
	0,
	bsp_key_scanner,
	bsp_eeprom_read,
	bsp_eeprom_write,
	bsp_get_clock,
	bsp_set_clock,
	bsp_adc_get_mv,
	usb_status,
	bsp_enable_int_01_hz,
	bsp_disable_int_01_hz,
	bsp_enable_int_32_hz,
	bsp_disable_int_32_hz,
	bsp_mount_disk,
	bsp_unmount_disk,
	bsp_chk_disk,
	bsp_create_log,
	bsp_delete_log,
	bsp_print_log,
	bsp_copy_log
};

int main(void)
{	
		bsp_init();
		dusthawk_init(&dusthawk);
		dusthawk_start();		
    while(1)
    {
			usb_status();
			bsp_delay(80000000);
    }
}
