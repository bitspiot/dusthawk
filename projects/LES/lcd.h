/*-----------------------------------------------------------------------*/
/* EZ-LCD - Generic control module include/configuration file            */
/*-----------------------------------------------------------------------*/

#ifndef _EZLCD_DEFINED
#define _EZLCD_DEFINED

/*--------------------------------------------------*/
/* Configuration Options                            */
/*--------------------------------------------------*/

#define _LCD_ROWS	4		/* Number of Rows (1,2 or 4) */
#define _LCD_COLS	20		/* Number of Columns (8..40) */

#define _USE_CURSOR	1		/* 1:Enable lcd_cursor function */
#define _USE_CGRAM	1		/* 1:Enable lcd_setcg function */

#define	_USE_FUEL	1		/* 1:Enable lcd_put_fuel function (_USE_CGRAM must be 1) */

#define	_USE_BAR	1		/* 1:Enable lcd_put_bar function (_USE_CGRAM must be 1) */
#define	_MAX_BAR	255		/* Maximum value for lcd_put_bar function */

#define	_USE_POINT	1		/* 1:Enable lcd_put_point function (_USE_CGRAM must be 1) */
#define	_MAX_POINT	255		/* Maximum value for lcd_put_point function */

#define	_BASE_GRAPH	1		/* Common user character used by lcd_put_bar/lcd_put_point function (2 chars from this) */



/*--------------------------------------------------*/
/* API declareations                                */
/*--------------------------------------------------*/

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif
void lcd_init (void);
void lcd_locate (uint8_t, uint8_t);
void lcd_putc (uint8_t);
void lcd_puts (const char *);
void lcd_clr(void);
void lcd_cursor (uint8_t);
void lcd_setcg (uint8_t, uint8_t, const uint8_t*);
void lcd_put_fuel (int8_t, uint8_t);
void lcd_put_bar (uint16_t, uint8_t, uint8_t);
void lcd_put_point (uint16_t, uint8_t, uint8_t);
#ifdef __cplusplus
}
#endif

#define CSR_OFF		0
#define CSR_BLOCK	1
#define CSR_UNDER	2


#endif	/* #ifndef _EZLCD */
