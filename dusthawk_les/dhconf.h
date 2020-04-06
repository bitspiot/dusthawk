#ifndef __DHCONF_H__
#define __DHCONF_H__

#ifdef __cplusplus
extern "C"
{
#endif

#define ENTER_BUTTON						0x04   
#define UP_BUTTON								0x01	 
#define DN_BUTTON								0x08	 
#define MENU_BUTTON							0x10	 
#define NO_BUTTON								0x09	
	
#define ALL_BUTTONS           (UP_BUTTON | DN_BUTTON | MENU_BUTTON | ENTER_BUTTON )

	
// EEPROM LOCATION
	
#define CALIBRATION             0		// Calibration type size 
#define USERSETTINGS            64
#define AUTOSETTINGS            128
#define SUMMARY                 192
#define FSM_STATE				256

#ifdef __cplusplus
}
#endif

#endif
