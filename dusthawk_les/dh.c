
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include "dhtypes.h"
#include "dhconf.h"
#include "dh.h"
#include "arm_math.h"


#define REMEMBER_THIS_STATE() set_struct(&S,sizeof(int),FSM_STATE)
#define RECALL_STATE()        get_struct(&S,sizeof(int),FSM_STATE)
#define FORGET_THIS_STATE()

#define SAVE_CALIBRATION()      set_struct(&calib_data,sizeof(calibration_t),CALIBRATION)
#define LOAD_CALIBRATION()      get_struct(&calib_data,sizeof(calibration_t),CALIBRATION)

#define SAVE_USERSETTING()      set_struct(&user_data,sizeof(user_settings_t),USERSETTINGS)
#define LOAD_USERSETTING()      get_struct(&user_data,sizeof(user_settings_t),USERSETTINGS)

#define SAVE_SUMMARY()          set_struct(&summary_data,sizeof(summary_t),SUMMARY)
#define LOAD_SUMMARY()          get_struct(&summary_data,sizeof(summary_t),SUMMARY)

#define SAVE_AUTOMODE()         set_struct(&auto_mode_data,sizeof(auto_mode_t),AUTOSETTINGS)
#define LOAD_AUTOMODE()         get_struct(&auto_mode_data,sizeof(auto_mode_t),AUTOSETTINGS)



#define WAIT_FOR_UP_KEY()       while(key_scanner()!=1)
#define WAIT_FOR_DOWN_KEY()     while(key_scanner()!=2)
#define WAIT_FOR_MENU_KEY()     while(key_scanner()!=3)
#define WAIT_FOR_ENTER_KEY()    while(key_scanner()!=4)  
#define WAIT_FOR_ANY_KEY()      while(key_scanner()==9)    
#define WAIT_FOREVER()          while(1)


static uint32_t key_scanner(void);
static uint32_t main_menu(void);
static uint32_t sample_start(void);
static uint32_t machine_setting(void);
static uint32_t maual_mode(void);
static uint32_t auto_mode(void);
static uint32_t user_settings(void);
static uint32_t data_transfer(void);
static uint32_t edit_rec_interval(void);
static uint32_t edit_flow_rate(void);
static uint32_t comp_on_off(void);
static uint32_t time_settings(void);
static uint32_t diagnostics(void);
static uint32_t run_module(void);

static uint32_t factory_settings(void);
static uint32_t calibration_setup(void);
static uint32_t at_calibration(void);
static uint32_t ft_calibration(void);
static uint32_t fr_calibration(void);
static uint32_t fp_calibration(void);
static uint32_t bp_calibration(void);
static uint32_t leak_check_limit(void);
static uint32_t shdn_flow_rate(void);
static uint32_t shdn_pressure(void);

static uint32_t edit_filter_id(void);
static uint32_t auto_setting_state(void);
static uint32_t auto_wait_state(void);
static uint32_t auto_exit_state(void);

static uint32_t memory_status(void);
static uint32_t memory_erase(void);

static uint32_t sample_adjst(void);
static uint32_t flow_adjst(void);
static uint32_t sampling_summary(void);
static uint32_t leak_check(void);

static void get_sensor_values(volatile float *physical,calibration_t *calib);
static void reset_summary(void);
static int32_t Password(void);

static dusthawk_t *dh;    //Dusk hawk pointer
static uint32_t S;        // index into current state
static uint32_t Input;    // index into next state

static calibration_t calib_data;
static user_settings_t user_data;
static summary_t summary_data;
static auto_mode_t auto_mode_data;
static cv_t cv;

static volatile uint32_t rtc_event;
static volatile uint32_t sample_event;

arm_pid_instance_f32 PID;
int32_t duty_adjst;
int32_t duty;

extern volatile int32_t usb_detect;

#define CTRL_SAMPLE_SIZE 64

typedef struct{
    float fr[CTRL_SAMPLE_SIZE];
    float fp[CTRL_SAMPLE_SIZE];
    float at[CTRL_SAMPLE_SIZE];
    float ft[CTRL_SAMPLE_SIZE];
    float bp[CTRL_SAMPLE_SIZE];
}ctrl_avg_t;

static volatile ctrl_avg_t ctrl_avg;
static volatile float raw_sensor_values[5];

typedef struct{
    float fr;
    float fp;
    float at;
    float ft;
    float bp;
}avg_t;

static volatile avg_t avg;
static volatile avg_t sample_avg;


// LCD Internal
enum{
    CURSOR_OFF=0,
    CURSOR_BLOCK,
    CURSOR_UNDER
};

//Machine modes
enum {
    MANUAL_MODE=0,
    AUTO_MODE
};

// state enumeration
enum
{
    M0=0,ST,MM,AM,MA,US,FS,CS,AT,FT,FR,FP,BP,EF,RI,FE,CE,
    TS,DG,RM,AS,AW,AE,MS,ME,SS,LC,LL,SF,SP,DT,
    SIZE
};


// Finite Sate machine

const state_t FSM[SIZE]={
    {&main_menu          ,{M0,ST,SS,DT,MA,EF,LC,DT,SS,M0}},
		{&sample_start			 ,{M0,MM,AM,ST,ST,ST,ST,ST,ST,ST}},
    {&maual_mode         ,{EF,RM,RM,RM,MM,MM,MM,MM,MM,MM}},
    {&auto_mode          ,{EF,RM,RM,RM,AM,AM,AM,AM,AM,AM}},
		{&machine_setting    ,{M0,US,TS,FS,MA,MA,MA,MA,MA,MA}},
    {&user_settings      ,{MA,RI,FE,ME,DG,US,US,US,US,US}},
    {&factory_settings   ,{MA,CS,LL,SF,SP,FS,FS,FS,FS,FS}},
    {&calibration_setup  ,{FS,AT,FT,FR,FP,BP,CS,CS,CS,CS}},
    {&at_calibration     ,{CS,AT,AT,AT,AT,AT,AT,AT,AT,AT}},
    {&ft_calibration     ,{CS,FT,FT,FT,FT,FT,FT,FT,FT,FT}},
    {&fr_calibration     ,{CS,FR,FR,FR,FR,FR,FR,FR,FR,FR}},
    {&fp_calibration     ,{CS,FP,FP,FP,FP,FP,FP,FP,FP,FP}},
    {&bp_calibration     ,{CS,BP,BP,BP,BP,BP,BP,BP,BP,BP}},
    {&edit_filter_id     ,{RM,AS,M0,M0,M0,M0,M0,M0,M0,M0}},
    {&edit_rec_interval  ,{US,RI,RI,RI,RI,RI,RI,RI,RI,RI}},
    {&edit_flow_rate     ,{US,FE,FE,FE,FE,FE,FE,FE,FE,FE}},
    {&comp_on_off        ,{US,CE,CE,CE,CE,CE,CE,CE,CE,CE}},
    {&time_settings      ,{US,TS,TS,TS,TS,TS,TS,TS,TS,TS}},
    {&diagnostics        ,{US,US,DG,DG,DG,DG,DG,DG,DG,DG}},
    {&run_module         ,{M0,M0,M0,SS,M0,SS,M0,M0,M0,M0}},
    {&auto_setting_state ,{M0,AW,AS,AS,AS,AS,AS,AS,AS,AS}},
    {&auto_wait_state    ,{AW,AW,AW,AE,RM,AW,AW,AW,AW,AW}},
    {&auto_exit_state    ,{M0,AW,AW,AW,AW,AW,AW,AW,AW,AW}},
    {&memory_status      ,{US,MS,MS,MS,MS,MS,MS,MS,MS,MS}},
    {&memory_erase       ,{US,ME,ME,ME,ME,ME,ME,ME,ME,ME}},
    {&sampling_summary   ,{M0,M0,M0,M0,RM,M0,SS,SS,SS,M0}},
    {&leak_check         ,{M0,LC,LC,LC,LC,LC,LC,LC,LC,LC}},
    {&leak_check_limit   ,{FS,FS,FS,FS,FS,FS,FS,FS,FS,FS}},
    {&shdn_flow_rate     ,{FS,FS,FS,FS,FS,FS,FS,FS,FS,FS}},
    {&shdn_pressure      ,{FS,FS,FS,FS,FS,FS,FS,FS,FS,FS}},
    {&data_transfer      ,{M0,M0,M0,M0,M0,M0,M0,M0,M0,M0}},
};

static void profiler(void){
    dh->pfnIndicatorOn();
    dh->pfnDelay(1000);
    dh->pfnIndicatorOff();
    dh->pfnDelay(1000);
}


static void get_struct(void *ptr , int size , int loc){

    dh->pfnMemRead((uint32_t *)ptr,loc,size);
}

static void set_struct(void *ptr , int size , int loc){

    dh->pfnMemWrite((uint32_t *)ptr,loc,size);
}




static void edit_integer_value_min_max(int32_t *val,const char* fmt, char r, char c,int min, int max, int resolution){
    char str[16];
    int done;
    int i;
    uint32_t retNum;
    uint32_t keyVal;
    int offset;

    retNum=*val;
    offset=atoi(fmt+1);
    sprintf(str,fmt,retNum);
    dh->pfnLcdPrintStr(r,c,str);
    dh->pfnLcdCursor(r,c+offset-1,CURSOR_UNDER);

    done=1;
    i=0;

    while(done){

    keyVal=key_scanner();

        switch(keyVal)
            {
                case 1:
                    {
                        dh->pfnLcdCursor(r,c,CURSOR_UNDER);
                        retNum+=resolution;
                        if(retNum>=max)
                        {
                            retNum=min;
                        }
                        else if(retNum<=min)
                        {
                            retNum=max;
                        }
                        sprintf(str,fmt,retNum);
                        dh->pfnLcdPrintStr(r,c,str);
                    }break;

                case 2:
                    {
                        dh->pfnLcdCursor(r,c,CURSOR_UNDER);
                        retNum-=resolution;
                        if(retNum>=max)
                        {
                            retNum=min;
                        }
                        else if(retNum<=min)
                        {
                            retNum=max;
                        }
                        sprintf(str,fmt,retNum);
                        dh->pfnLcdPrintStr(r,c,str);
                    }break;

                case 3:
                    {
                        dh->pfnLcdCursor(r,c,CURSOR_BLOCK);
                        i-=1;
                        if(i<0)
                        {
                            done=0;
                        }
                    }break;

                case 4:
                    {
                        dh->pfnLcdCursor(r,c,CURSOR_BLOCK);
                        i+=1;
                        if(i>=1)
                        {
                            done=0;
                            i=0;
                            sprintf(str,fmt,retNum);
                            dh->pfnLcdPrintStr(r,c,str);
                        }
                    }break;

                case 9:
                    {
                         dh->pfnLcdCursor(r,c+offset-1,CURSOR_UNDER);
                    }break;
            }
    }

    *val=retNum;
}

static void edit_float_value(float *fnum,const char* fmt, char r, char c){

    const char value[]="0123456789.-";
    
    int done;
    int i=0,j=0;
    float retNum;
    char str[20];
    uint32_t keyVal;
    retNum=*fnum;
    memset(str,'\0',sizeof(str));
    sprintf(str,fmt,*fnum);
    dh->pfnLcdPrintStr(r,c,str);
    dh->pfnLcdCursor(r,c,CURSOR_BLOCK);

    done=0;
    i=0;
    
    j=str[i]-'0';

    if(str[i]=='.'){
        j=10;
    }
    if(str[i]=='-'){
        j=11;
    }
    
   while(!done){

        keyVal=key_scanner();

        switch(keyVal){
            case 1: 
                {
                    dh->pfnDebugPrint("i:%d j:%d\n",i,j);
                    dh->pfnLcdCursor(r,c+i,CURSOR_UNDER);
                    j++;
                    if(j>(strlen(value)-1)){
                        j=0;
                    }
                    str[i]=value[j];
                    
                    dh->pfnLcdPrintChr(r,c+i,str[i]);
                    break;
                }

            case 2:
                {
                    dh->pfnDebugPrint("i:%d j:%d\n",i,j);
                    dh->pfnLcdCursor(r,c+i,CURSOR_UNDER);
                    j--;
                    if(j<0){
                        j=(strlen(value)-1);
                    }
                    str[i]=value[j];
                    
                    dh->pfnLcdPrintChr(r,c+i,str[i]);
                    break;
                }

            case 3:
                {
                    dh->pfnDebugPrint("i:%d j:%d\n",i,j);
                    dh->pfnLcdCursor(r,c+i,CURSOR_BLOCK);
                    i-=1;
                    
                    if(i<0){
                        i=0;
                        done=1;
                    }
                    j=str[i]-'0';
                    
                    if(str[i]=='.'){
                        j=10;
                    }
                    if(str[i]=='-'){
                        j=11;
                    }

                    break;
                }

            case 4:
                {
                    dh->pfnDebugPrint("i:%d j:%d\n",i,j);
                    dh->pfnLcdCursor(r,c+i,CURSOR_BLOCK);
                    
                    i+=1;
                    
                   if(i>strlen(str)-1){
                        done=1;
                        i=0;
                        dh->pfnLcdPrintStr(r,c+i,str);
                        sscanf(str,"%f",&retNum);
                    }
                   
                    j=str[i]-'0';
                    
                    if(str[i]=='.'){
                        j=10;
                    }
                    if(str[i]=='-'){
                        j=11;
                    }
                    
                    break;
                }

            case 9:
                {
                    
                    dh->pfnLcdCursor(r,c+i,CURSOR_BLOCK);
                    break;
                }

        }
    }

    *fnum = retNum;
}

static void edit_string(char* str,int r, int c){

//    const char value[]="0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ";
    const char value[]="0123456789";
    char temp[20];
    uint32_t keyVal;
    int done=0;
    int i=0,j=0;

    dh->pfnDebugPrint(str);

    memset(temp,'\0',sizeof(temp));

    snprintf(temp,9,"%8s",str);

    dh->pfnLcdPrintStr(r,c,temp);

    dh->pfnLcdCursor(r,c,CURSOR_BLOCK);

    dh->pfnDebugPrint("strlen(value):%d\n strln(temp):%d\n",strlen(value),strlen(temp));
    dh->pfnDebugPrint(temp);
    
    j=temp[i]-'0'; 
    
    while(!done){

        keyVal=key_scanner();

        switch(keyVal){
            case 1: 
                {
                    dh->pfnDebugPrint("i:%d j:%d\n",i,j);
                    dh->pfnLcdCursor(r,c+i,CURSOR_UNDER);
                    j++;
                    if(j>(strlen(value)-1)){
                        j=0;
                    }
                    temp[i]=value[j];
                    
                    dh->pfnLcdPrintChr(r,c+i,temp[i]);
                    break;
                }

            case 2:
                {
                    dh->pfnDebugPrint("i:%d j:%d\n",i,j);
                    dh->pfnLcdCursor(r,c+i,CURSOR_UNDER);
                    j--;
                    if(j<0){
                        j=(strlen(value)-1);
                    }
                    temp[i]=value[j];
                    
                    dh->pfnLcdPrintChr(r,c+i,temp[i]);
                    break;
                }

            case 3:
                {
                    dh->pfnDebugPrint("i:%d j:%d\n",i,j);
                    dh->pfnLcdCursor(r,c+i,CURSOR_BLOCK);
                    i-=1;
                    if(i<0){
                        done=1;
                    }
                    j=temp[i]-'0';   
                    break;
                }

            case 4:
                {
                    dh->pfnDebugPrint("i:%d j:%d\n",i,j);
                    dh->pfnLcdCursor(r,c+i,CURSOR_BLOCK);
                    i+=1;
                    if(i>strlen(str)-1){
                        done=1;
                        i=0;
                        dh->pfnLcdPrintStr(r,c+i,temp);
                        sscanf(temp,"%8s",str);
                    }
                    j=temp[i]-'0';
                    break;
                }

            case 9:
                {
                    
                    dh->pfnLcdCursor(r,c+i,CURSOR_BLOCK);
                    break;
                }

        }
    }
    strcpy(str,temp);

}


static void edit_string_1(char* str,int r, int c){

//    const char value[]="0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ";
    const char value[]="0123456789";
    char temp[20];
    uint32_t keyVal;
    int done=0;
    int i=0,j=0;

    dh->pfnDebugPrint(str);

    memset(temp,'\0',sizeof(temp));

    snprintf(temp,5,"%4s",str);

    dh->pfnLcdPrintStr(r,c,temp);

    dh->pfnLcdCursor(r,c,CURSOR_BLOCK);

    dh->pfnDebugPrint("strlen(value):%d\n strln(temp):%d\n",strlen(value),strlen(temp));
    dh->pfnDebugPrint(temp);
    
    j=temp[i]-'0';
    
    while(!done){

        keyVal=key_scanner();

        switch(keyVal){
            case 1: 
                {
                    dh->pfnDebugPrint("i:%d j:%d\n",i,j);
                    dh->pfnLcdCursor(r,c+i,CURSOR_UNDER);
                    j++;
                    if(j>(strlen(value)-1)){
                        j=0;
                    }
                    temp[i]=value[j];
                    
                    dh->pfnLcdPrintChr(r,c+i,temp[i]);
                    break;
                }

            case 2:
                {
                    dh->pfnDebugPrint("i:%d j:%d\n",i,j);
                    dh->pfnLcdCursor(r,c+i,CURSOR_UNDER);
                    j--;
                    if(j<0){
                        j=(strlen(value)-1);
                    }
                    temp[i]=value[j];
                    
                    dh->pfnLcdPrintChr(r,c+i,temp[i]);
                    break;
                }

            case 3:
                {
                    dh->pfnDebugPrint("i:%d j:%d\n",i,j);
                    dh->pfnLcdCursor(r,c+i,CURSOR_BLOCK);
                    i-=1;
                    if(i<0){
                      done=1;
                    }
                    j=temp[i]-'0';
                    break;
                }

            case 4:
                {
                    dh->pfnDebugPrint("i:%d j:%d\n",i,j);
                    dh->pfnLcdCursor(r,c+i,CURSOR_BLOCK);
                    i+=1;
                    if(i>strlen(str)-1){
                        done=1;
                        i=0;
                        dh->pfnLcdPrintStr(r,c+i,temp);
                        sscanf(temp,"%4s",str);
                    }
                    j=temp[i]-'0';
                    break;
                }

            case 9:
                {
                    
                    dh->pfnLcdCursor(r,c+i,CURSOR_BLOCK);
                    break;
                }

        }
    }
    strcpy(str,temp);

}


static uint32_t key_scanner(void){

    uint32_t keyVal;
    static uint8_t ui8CurButtonState, ui8PrevButtonState;

    keyVal=9;
    ui8CurButtonState = dh->pfnKeyScan();
    // Check if previous debounced state is equal to the current state.
    if(ui8CurButtonState != ui8PrevButtonState)
    {
            ui8PrevButtonState = ui8CurButtonState;
            switch(ui8CurButtonState & ALL_BUTTONS)
            {
                    case UP_BUTTON:
                        keyVal=1;break;
                    case DN_BUTTON:
                        keyVal=2;break;
                    case MENU_BUTTON:
                        keyVal=3;break;
                    case ENTER_BUTTON:
                        keyVal=4;break;
                    default:
                        keyVal=9;break;
            }
    }
    return keyVal;
}

uint32_t  menu_scanner(uint8_t menuItemCount, void (*pfnTask)(void)){

    uint32_t return_value;
    uint32_t key_value;

    static int i=0;
    const int8_t r[]={0x00,0x01,0x02,0x03,0x00,0x01,0x02,0x03};
    const int8_t c[]={0x00,0x00,0x00,0x00,0x14,0x14,0x14,0x14};

    return_value=NO_BUTTON;
    dh->pfnLcdCursor(0,0,CURSOR_BLOCK);

     do{
            (*pfnTask)();
            dh->pfnUSBCallback();
            key_value=key_scanner();
            dh->pfnLcdCursor(r[i],c[i],CURSOR_BLOCK);
        }while(key_value==NO_BUTTON);

    switch(key_value){
            case 1    :i--;
                                if(i>menuItemCount)i=0;
                                else if(i<0)i=menuItemCount;
                                dh->pfnLcdCursor(r[i],c[i],CURSOR_UNDER);break;

            case 2    :i++;
                                if(i>menuItemCount)i=0;
                                else if(i<0)i=menuItemCount;
                                dh->pfnLcdCursor(r[i],c[i],CURSOR_UNDER);break;

            case 3    :return_value=0;break;
          case 4    :return_value=i+1;i=0;break;
            default   :dh->pfnLcdCursor(r[i],c[i],CURSOR_UNDER);break;
    }

    dh->pfnLcdCursor(r[i],c[i],CURSOR_BLOCK);

    return return_value;
}
static uint32_t main_menu(void){

    uint32_t ret_val;
    
    REMEMBER_THIS_STATE();
    //state is neither auto nor manual 
    auto_mode_data.auto_mode=2;
    
    dh->pfnDebugPrint("[INFO]:main_menu..\n");
    dh->pfnLcdPrintStr(0,0, " SAMPLING START     ");
    dh->pfnLcdPrintStr(1,0, " SAMPLING SUMMARY   ");
    dh->pfnLcdPrintStr(2,0, " DATA TRANSFER      ");
    dh->pfnLcdPrintStr(3,0, " MACHINE SETTING    ");
       
    return menu_scanner(3,profiler);
}

static uint32_t sample_start(void){

    uint32_t ret_val;
    
    REMEMBER_THIS_STATE();
    //state is neither auto nor manual 
    auto_mode_data.auto_mode=2;
    
    dh->pfnDebugPrint("[INFO]:sample start..\n");
    dh->pfnLcdPrintStr(0,0," MANUAL MODE         ");
    dh->pfnLcdPrintStr(1,0," AUTO MODE           ");
		dh->pfnLcdPrintStr(2,0,"                     ");
		dh->pfnLcdPrintStr(3,0,"                     ");
           
    return menu_scanner(1,profiler);
}
static uint32_t machine_setting(void){

    uint32_t ret_val;
    
    REMEMBER_THIS_STATE();
    //state is neither auto nor manual 
    auto_mode_data.auto_mode=2;
    
    dh->pfnDebugPrint("[INFO]:machine setting..\n");
    dh->pfnLcdPrintStr(0,0," USER SETTING        ");
		dh->pfnLcdPrintStr(1,0," CLOCK SETTING       ");
		dh->pfnLcdPrintStr(2,0," FACTORY SETTING     ");
		dh->pfnLcdPrintStr(3,0,"                     ");
           
    ret_val=menu_scanner(2,profiler);
    if(ret_val==3){
        if(Password()==0){
            ret_val=0;
        }
    }
    
    return ret_val;
}

static uint32_t maual_mode(void){
    FORGET_THIS_STATE();
    auto_mode_data.auto_mode=0;
    set_struct(&auto_mode_data,sizeof(auto_mode_t),AUTOSETTINGS);
    return 0;
}

static uint32_t auto_mode(void){
    FORGET_THIS_STATE();
    auto_mode_data.auto_mode=1;
    set_struct(&auto_mode_data,sizeof(auto_mode_t),AUTOSETTINGS);
    return 0;
}

static uint32_t user_settings(void){
    
    FORGET_THIS_STATE();
    dh->pfnDebugPrint("[INFO]:user_settings..\n");

    dh->pfnLcdPrintStr(0,0, " SAMPLING INTERVAL   ");
    dh->pfnLcdPrintStr(1,0, " FLOW RATE           ");
		dh->pfnLcdPrintStr(2,0, " MEMORY ERASE        ");
    dh->pfnLcdPrintStr(3,0, " DIAGNOSTICS         ");
    

    return menu_scanner(6,profiler);
}

static uint32_t edit_rec_interval(void){
    
    FORGET_THIS_STATE();
    dh->pfnDebugPrint("[INFO]:edit_rec_interval..\n");

    dh->pfnLcdClear();
    dh->pfnLcdPrintStr(0,0, "SAMPLING INTERVAL");
    edit_integer_value_min_max(&user_data.rec_int,"%02d Min",1,6,1,60,1);
    dh->pfnLcdPrintStr(2,6, "SAVING..");
    set_struct(&user_data,sizeof(user_settings_t),USERSETTINGS);
    dh->pfnDelay(80000000/6);
    dh->pfnLcdPrintStr(2,6, "SAVED.. ");
    dh->pfnDelay(80000000/6);

    return 0;
}

static uint32_t data_transfer(void){
    
    FORGET_THIS_STATE();
    dh->pfnDebugPrint("[INFO]:edit_rec_interval..\n");
    dh->pfnLcdClear();
    dh->pfnLcdPrintStr(0,0, "  DATA TRANSFER     ");
    dh->pfnLcdPrintStr(1,0, "CHECKING INTERNAL DRIVE");
    dh->pfnDelay(80000000/6);
  
    
    if(dh->pfnCheckDisk(0) !=0 ){
        dh->pfnLcdPrintStr(2,0,"   ERROR: DH002  ");
        dh->pfnLcdPrintStr(3,0,"INTERNAL DISK ERROR");
        dh->pfnDelay(80000000/6);
        WAIT_FOR_ANY_KEY();
        return 0;
    }
    else{
        dh->pfnLcdPrintStr(2,0,"INTERNAL DRIVE OK");
    }
    
    dh->pfnLcdPrintStr(1,0, "CHECKING EXTERNAL DRIVE");
    dh->pfnDelay(80000000/6);
    
    if(dh->pfnMountDisk(1) != 0 ){
        dh->pfnLcdPrintStr(2,0,"  ERROR: DH003 ");
        dh->pfnLcdPrintStr(3,0,"EXTERNAL DISK ERROR ");
        dh->pfnDelay(80000000/6);
        WAIT_FOR_ANY_KEY();
        return 0;
    }

    if(dh->pfnCheckDisk(1) !=0 ){
       dh->pfnLcdPrintStr(2,0,"  ERROR: DH004      ");
        dh->pfnLcdPrintStr(3,0,"EXTERNAL DISK ERROR");
        dh->pfnDelay(80000000/6);
        WAIT_FOR_ANY_KEY();
        return 0;
    }
    
    else{
        dh->pfnLcdPrintStr(2,0,"EXTERNAL PEN DRIVE FOUND ");
    }
    
    dh->pfnLcdPrintStr(3,0,    "TRANSFERRING DATA....");
    dh->pfnCpoyLogFile();
    dh->pfnDelay(80000000/3);
    dh->pfnLcdPrintStr(3,0,    "TRANSFERRING DATA....DONE");
    dh->pfnDelay(80000000/3);
    dh->pfnLcdPrintStr(3,0,    "Please Remove Pen Drive");
    dh->pfnDelay(80000000/3);
    WAIT_FOR_ANY_KEY();
    return 0;
}

static uint32_t edit_flow_rate(void){
    
    FORGET_THIS_STATE();
    
    dh->pfnDebugPrint("[INFO]:edit_flow_rate..\n");

    dh->pfnLcdClear();
    dh->pfnLcdPrintStr(0,4, " FLOW RATE ");
    dh->pfnLcdPrintStr(1,10, " Lpm ");
    edit_float_value(&user_data.set_fr,"%4.1f",1,6);
    dh->pfnLcdPrintStr(2,6, "SAVING..");
    set_struct(&user_data,sizeof(user_settings_t),USERSETTINGS);
    dh->pfnDelay(80000000/6);
    dh->pfnLcdPrintStr(2,6, "SAVED.. ");
    dh->pfnDelay(80000000/6);

    return 0;
}


static uint32_t comp_on_off(void){

    int done=0;
    unsigned int key_val;
    
    FORGET_THIS_STATE();
    dh->pfnDebugPrint("[INFO]:comp_on_off..\n");

    dh->pfnLcdClear();
    dh->pfnLcdPrintStr(1,0, "COMPENSATION");

    if(user_data.comp == 1)
    {
        dh->pfnLcdPrintStr(1,13, ":ON   ");
    }
    else
    {
        dh->pfnLcdPrintStr(1,13, ":OFF ");
    }

    while(!done)
    {
        key_val=key_scanner();

        switch(key_val)
        {
            case 1:
            {
                user_data.comp++;
                user_data.comp&=0x01;

                if(user_data.comp == 1)
                {
                    dh->pfnLcdPrintStr(1,13, ":ON   ");
                }
                else
                {
                    dh->pfnLcdPrintStr(1,13, ":OFF ");
                }


            }break;

            case 2:
            {
                user_data.comp--;
                user_data.comp&=0x01;

                if(user_data.comp == 1)
                {
                    dh->pfnLcdPrintStr(1,13, ":ON   ");
                }
                else
                {
                    dh->pfnLcdPrintStr(1,13, ":OFF ");
                }

            }break;

            case 3:
            case 4:
            {
                dh->pfnLcdPrintStr(2,6, "SAVING..");
                set_struct(&user_data,sizeof(user_settings_t),USERSETTINGS);
                dh->pfnDelay(80000000/12);
                dh->pfnLcdPrintStr(2,6, "SAVED.. ");
                dh->pfnDelay(80000000/12);
                done=1;
            }
        }
    }
    return 0;
}


static uint32_t time_settings(void){
    char str[20];
    int16_t i,done;
    calender_t cal;
    uint32_t key_value;
    /*
    Row and col coordinate for editing the date
    */
    const uint8_t row[6]={2,2,2,3,3,3};  // row coordinate
    const uint8_t col[6]={8,11,14,8,11,14};    // col coordinate
    const uint8_t min[6]={1,1,17,0,0,0};
    const uint8_t max[6]={31,12,35,23,59,59};
    /*
    Array to hold the date and time
    the data is held in format:ddmmyyhhmmss
    */
    int32_t dt[6];//ddmmyyhhMMss

    /*
    Get the current snapshot of the time and date from RTC and store in the datetime array 'dt[12]'
    10's digit is sotred in even index of the array and 1's digit in the odd indexes.
    */

    dh->pfnGetClock(&cal);

    dt[0]=cal.dd;
    dt[1]=cal.mm;
    dt[2]=cal.yy;
    dt[3]=cal.h;
    dt[4]=cal.m;
    dt[5]=cal.s;

    /*
    clear the index and set the done=1
    */
    i=0;
    done=0;
    dh->pfnLcdClear();
    dh->pfnLcdPrintStr(0,0,"Enter New Date Time");
    sprintf(str,"Date:%02d/%02d/%02d",dt[0],dt[1],dt[2]);

    dh->pfnLcdPrintStr(2,3,str);

    sprintf(str,"Time:%02d:%02d:%02d",dt[3],dt[4],dt[5]);
    dh->pfnLcdPrintStr(3,3,str);

    dh->pfnLcdCursor(row[0],col[0]+1,CURSOR_UNDER);

    /*
    stay here until done=1;
    */
    while(!done){

        key_value=key_scanner();
        switch(key_value){

            case 1:{
                dh->pfnLcdCursor(row[i],col[i],CURSOR_UNDER);
                ++dt[i];
                if(dt[i]>max[i])dt[i]=min[i];
                else if(dt[i]<min[i])dt[i]=max[i];
                sprintf(str,"%02d",dt[i]);
                dh->pfnLcdPrintStr(row[i],col[i],str);

                }break;

            case 2:{
                dh->pfnLcdCursor(row[i],col[i],CURSOR_UNDER);
                --dt[i];
                if(dt[i]>max[i])dt[i]=min[i];
                else if(dt[i]<min[i])dt[i]=max[i];
                sprintf(str,"%02d",dt[i]);
                dh->pfnLcdPrintStr(row[i],col[i],str);
                 }break;

            case 3:{
                    dh->pfnLcdCursor(row[i],col[i],CURSOR_UNDER);
                    i-=1;
                    if(i<0)
                    {
                        i=0;
                        done=1;
                        dh->pfnLcdClear();
                    }
                    }break;

            case 4:{
                dh->pfnLcdCursor(row[i],col[i],CURSOR_UNDER);
                i+=1;
                if(i>=6){
                    done=1;i=0;
                    dh->pfnLcdClear();
                    dh->pfnLcdPrintStr(1,4,"UPDATING...");
                    /*
                    update the RTC
                    */
                    cal.dd=dt[0];
                    cal.mm=dt[1];
                    cal.yy=dt[2];
                    cal.h=dt[3];
                    cal.m=dt[4];
                    cal.s=dt[5];
                    dh->pfnSetClock(&cal);
                    /*
                    wait for at least a second
                    */
                    dh->pfnDelay(50000000/6);
                    dh->pfnLcdClear();
                }
                }break;

            case 9:{
                    dh->pfnLcdCursor(row[i],col[i]+1,CURSOR_UNDER);
                    }break;
        }
    }

    return 0;
}

static uint32_t diagnostics(void){
    char buff[128];
    float adc_values[8];
    uint32_t key_val=9;
    int pump_toggle;
    uint16_t duty;

    dh->pfnDebugPrint("[INFO]:diagnostics..\n");
    FORGET_THIS_STATE();
    
    dh->pfnLcdClear();
    dh->pfnLcdPrintStr(2,4,"PRESS ENTER -> PUMP ON/OFF");
    dh->pfnLcdPrintStr(3,4,"PRESS UP/DN -> FLOW ADJUST");
    dh->pfnIntEnable01Hz();
    
    dh->pfnLcdCursor(0,0,CURSOR_OFF);
    pump_toggle=0;
    duty=40000;
    do{
        key_val=key_scanner();

        if(key_val==1){
            duty+=500;
            if(duty>60000){
             duty=60000;   
            }
            dh->pfnValveOpen(duty);
        }
           
        if(key_val==2){
            duty-=500;
            if(duty <1000){
                duty=1000;
            }
            dh->pfnValveOpen(duty);
        }
        
        
        
        if(key_val==4){
            pump_toggle++;
            pump_toggle&=0x01;
            dh->pfnDebugPrint("key:%d, pump_toggle:%d\n",(int)key_val,pump_toggle);
            
            if(pump_toggle==1){
                dh->pfnPumpOn();
                dh->pfnValveOpen(duty);
                
            }
            else{
                dh->pfnPumpOff();
                dh->pfnValveClose();
            }  
        }
        
        if(rtc_event)
        {
            rtc_event=0;
            
           
            
            dh->pfnGetAdcMv(adc_values);
            sprintf(buff,"FR:%04d  mV\n",(int)adc_values[1]);
            dh->pfnDebugPrint(buff);
            dh->pfnLcdPrintStr(0,0,buff);
            
            sprintf(buff,"FP:%04d  mV\n",(int)adc_values[2]);
            dh->pfnDebugPrint(buff);
            dh->pfnLcdPrintStr(1,0,buff);
            
            sprintf(buff,"BP:%04d  mV\n",(int)adc_values[3]);
            dh->pfnDebugPrint(buff);
            dh->pfnLcdPrintStr(2,0,buff);
            
            sprintf(buff,"AT:%04d  mV\n",(int)adc_values[4]);
            dh->pfnDebugPrint(buff);
            dh->pfnLcdPrintStr(3,0,buff);
            
            sprintf(buff,"FT:%04d  mV\n",(int)adc_values[5]);
            dh->pfnDebugPrint(buff);
            dh->pfnLcdPrintStr(0,0,buff);
            
        }

    }while(key_val!=3);

    dh->pfnPumpOff();
    dh->pfnValveClose();
    dh->pfnIntDisable01Hz();
    dh->pfnDebugPrint("\ndiagnostics exit\n");

    return 0;
}

static int32_t Password(void){
	
	char password[16];
	char temp[16];
	int32_t ret=-1;
	
    dh->pfnLcdClear();

	dh->pfnLcdPrintStr(0,0,"   Enter Password   ");
	strcpy(temp,"0000");
	edit_string_1(temp,1,8);
	
    temp[4]='\n';
    
	password[0]='0';
	password[1]='9';
	password[2]='1';
	password[3]='1';
	password[4]='\n';
	
	if(strncmp(temp,password,4)==0)
	{
		dh->pfnLcdPrintStr(2,0,"   Access Granted   ");
		ret=1;
	}
	
	else{
		dh->pfnLcdPrintStr(2,0,"   Access Denied    ");
		ret=0;
	}
	
	dh->pfnDelay(16000000/3);
	
	return ret;
}



static uint32_t factory_settings(void){
  
    FORGET_THIS_STATE();
    
    dh->pfnDebugPrint("[INFO]:factory_settings..\n");
    
    dh->pfnLcdPrintStr(0,0, " CALIBRATION SETUP  ");
    dh->pfnLcdPrintStr(1,0, " LEAK CHECK LIMIT   ");
    dh->pfnLcdPrintStr(2,0, " SHUTDOWN FLOWRATE  ");
    dh->pfnLcdPrintStr(3,0, " SHUTDOWN PRESSURE  ");
    
    return menu_scanner(3,profiler);
}


static uint32_t calibration_setup(void){

    dh->pfnDebugPrint("[INFO]:calibration_setup..\n");

    // Save the FSM State
    FORGET_THIS_STATE();
    
    dh->pfnLcdPrintStr(0,0, " AT CALIBRATION     ");
    dh->pfnLcdPrintStr(1,0, " FT CALIBRATION     ");
    dh->pfnLcdPrintStr(2,0, " FR CALIBRATION     ");
    dh->pfnLcdPrintStr(3,0, " FP CALIBRATION     ");
    dh->pfnLcdPrintStr(0,20," BP CALIBRATION     ");
    
    return menu_scanner(4,profiler);
}


static uint32_t at_calibration(void)
{
    char buff[32];

    dh->pfnDebugPrint("[INFO]:at_calibration..\n");
    dh->pfnLcdClear();

    sprintf(buff," AT MUL:%07.3f",calib_data.at.mul);
    dh->pfnLcdPrintStr(0,0,buff);

    sprintf(buff," AT OFF:%07.3f",calib_data.at.off);
    dh->pfnLcdPrintStr(1,0,buff);

    edit_float_value(&calib_data.at.mul,"%07.3f",0,8);
    edit_float_value(&calib_data.at.off,"%07.3f",1,8);

    dh->pfnDelay(1000000);

    set_struct(&calib_data,sizeof(calibration_t),CALIBRATION);

    return 0;
}


static uint32_t ft_calibration(void)
{
    char buff[32];

    dh->pfnDebugPrint("[INFO]:ft_calibration..\n");
    dh->pfnLcdClear();

    sprintf(buff," FT MUL:%07.3f",calib_data.ft.mul);
    dh->pfnLcdPrintStr(0,0,buff);

    sprintf(buff," FT OFF:%07.3f",calib_data.ft.off);
    dh->pfnLcdPrintStr(1,0,buff);

    edit_float_value(&calib_data.ft.mul,"%07.3f",0,8);
    edit_float_value(&calib_data.ft.off,"%07.3f",1,8);

    dh->pfnDelay(1000000);

    set_struct(&calib_data,sizeof(calibration_t),CALIBRATION);

    return 0;

}
static uint32_t fr_calibration(void)
{
    char buff[32];

    dh->pfnDebugPrint("[INFO]:fr_calibration..\n");
    dh->pfnLcdClear();

    sprintf(buff," FR MUL:%07.3f",calib_data.fr.mul);
    dh->pfnLcdPrintStr(0,0,buff);

    sprintf(buff," FR OFF:%07.3f",calib_data.fr.off);
    dh->pfnLcdPrintStr(1,0,buff);

    edit_float_value(&calib_data.fr.mul,"%07.3f",0,8);
    edit_float_value(&calib_data.fr.off,"%07.3f",1,8);

    dh->pfnDelay(1000000);

    set_struct(&calib_data,sizeof(calibration_t),CALIBRATION);

    return 0;
}

static uint32_t fp_calibration(void)
{
    char buff[32];

    dh->pfnDebugPrint("[INFO]:fp_calibration..\n");
    dh->pfnLcdClear();

    sprintf(buff," FP MUL:%07.3f",calib_data.fp.mul);
    dh->pfnLcdPrintStr(0,0,buff);

    sprintf(buff," FP OFF:%07.3f",calib_data.fp.off);
    dh->pfnLcdPrintStr(1,0,buff);

    edit_float_value(&calib_data.fp.mul,"%07.3f",0,8);
    edit_float_value(&calib_data.fp.off,"%07.3f",1,8);

    dh->pfnDelay(1000000);

    set_struct(&calib_data,sizeof(calibration_t),CALIBRATION);

    return 0;
}

static uint32_t bp_calibration(void)
{
    char buff[32];

    dh->pfnDebugPrint("[INFO]:bp_calibration..\n");
    dh->pfnLcdClear();

    sprintf(buff," BP MUL:%07.3f",calib_data.bp.mul);
    dh->pfnLcdPrintStr(0,0,buff);

    sprintf(buff," BP OFF:%07.3f",calib_data.bp.off);
    dh->pfnLcdPrintStr(1,0,buff);

    edit_float_value(&calib_data.bp.mul,"%07.3f",0,8);
    edit_float_value(&calib_data.bp.off,"%07.3f",1,8);

    dh->pfnDelay(1000000);

    set_struct(&calib_data,sizeof(calibration_t),CALIBRATION);

    return 0;
}


static uint32_t edit_filter_id(void){
    
    char str[16];

    dh->pfnDebugPrint("[INFO]:edit_filter_id..\n");
    dh->pfnLcdClear();

    dh->pfnLcdPrintStr(0,2,"EDIT FILTER ID:");

    memset(str,'\0',sizeof(str));

    user_data.f_id[8]='\0';

    sprintf(str,"%8s",user_data.f_id);

    edit_string(str,1,5);

    // update filter id field 
    LOAD_USERSETTING();
    
    if(strcmp(str,user_data.f_id)!=0){
        strncpy(user_data.f_id,str,8);
        dh->pfnDebugPrint("fid changed\n");
        SAVE_USERSETTING();
        reset_summary();
        
    }
    
    if(summary_data.new_file==1){
         //if new file found then write filter id
        summary_data.new_file=0;
        SAVE_SUMMARY();
        dh->pfnDebugPrint("writing filter id in the file\n");
        dh->pfnPrintLogFile("FILTER ID: %8s,\n",user_data.f_id);   
    }
    
    if(auto_mode_data.auto_mode==0){
        dh->pfnDebugPrint("auto_mode_data:manual\n");
        return 0;
    }

    if(auto_mode_data.auto_mode==1){
        dh->pfnDebugPrint("auto_mode_data:auto\n");
        return 1;
    }

    return 2;
}


static int ValidateSchedule(calender_t *t1, calender_t *t2){
	
	struct tm T1,T2;
	time_t timer1,timer2;
	
	T1.tm_year =(2000+t1->yy)-1900;
	T1.tm_mon = (t1->mm)-1;
	T1.tm_mday = (t1->dd);
	T1.tm_hour = (t1->h);
	T1.tm_min = (t1->m);
	T1.tm_sec = (t1->s);
	
	T2.tm_year =(2000+t2->yy)-1900;
	T2.tm_mon = (t2->mm)-1;
	T2.tm_mday = (t2->dd);
	T2.tm_hour = (t2->h);
	T2.tm_min = (t2->m);
	T2.tm_sec = (t2->s);
	
	timer1=mktime(&T1);
	timer2=mktime(&T2);
    
	dh->pfnDebugPrint("T1:%d\tT2:%d\n",timer1,timer2);
	dh->pfnDebugPrint("Diff:%d\n",(timer2-timer1));
	
	return (timer2-timer1);
}

static uint32_t auto_setting_state(void){
		const uint8_t row[12]={0,0,0,1,1,1,2,2,2,3,3,3};                // row coordinate
		const uint8_t col[12]={11,14,17,11,14,17,11,14,17,11,14,17};	// col coordinate
		//dd/mm/yy hh:mm:ss
		const uint8_t min[12]={1,1,16,0,0,0,1,1,16,0,0,0};  		    // min value
		const uint8_t max[12]={31,12,99,23,59,59,31,12,99,23,59,59};	// max value
			
		char str[20];
		static int32_t i=0,done=1;
		int32_t validityCnt;
		uint32_t nxtState=0;
		uint32_t machineState;
		uint32_t keyVal;
		calender_t cal_now;

		int32_t dt[12];      /*ddmmyyHMShhmm*/

		FORGET_THIS_STATE();
		auto_mode_data.auto_mode=1;
		dh->pfnGetClock(&auto_mode_data.start_cal);
		dh->pfnGetClock(&auto_mode_data.stop_cal);

		dt[0]=auto_mode_data.start_cal.dd;
		dt[1]=auto_mode_data.start_cal.mm;
		dt[2]=auto_mode_data.start_cal.yy;
		dt[3]=auto_mode_data.start_cal.h;
		dt[4]=auto_mode_data.start_cal.m;
		dt[5]=auto_mode_data.start_cal.s;
		
		dt[6]=auto_mode_data.stop_cal.dd;
		dt[7]=auto_mode_data.stop_cal.mm;
		dt[8]=auto_mode_data.stop_cal.yy;
		dt[9]=auto_mode_data.stop_cal.h;
		dt[10]=auto_mode_data.stop_cal.m;
		dt[11]=auto_mode_data.stop_cal.s; 
			
		dh->pfnLcdClear();
			//dh->pfnLcdPrintStr(0,0,"   START SCHEDULE ");
		sprintf(str,"START DATE:%02d/%02d/%02d",auto_mode_data.start_cal.dd,auto_mode_data.start_cal.mm,auto_mode_data.start_cal.yy);
		dh->pfnLcdPrintStr(0,0,str);
		sprintf(str,"START TIME:%02d:%02d:%02d",auto_mode_data.start_cal.h,auto_mode_data.start_cal.m,auto_mode_data.start_cal.s);
		dh->pfnLcdPrintStr(1,0,str);

		//dh->pfnLcdPrintStr(0,20,"    STOP SCHEDULE ");
		sprintf(str," STOP DATE:%02d/%02d/%02d",auto_mode_data.stop_cal.dd,auto_mode_data.stop_cal.mm,auto_mode_data.stop_cal.yy);
		dh->pfnLcdPrintStr(2,0,str);
		sprintf(str," STOP TIME:%02d:%02d:%02d",auto_mode_data.stop_cal.h,auto_mode_data.stop_cal.m,auto_mode_data.stop_cal.s);
		dh->pfnLcdPrintStr(3,0,str);
		dh->pfnDebugPrint("[INFO]:auto_setting_state..\n");
			
     
    dh->pfnLcdCursor(row[0],col[0],CURSOR_BLOCK);
    while(done)
        {
            keyVal=key_scanner();
            
            switch(keyVal)
                {
                case 1:
                    {
                        dh->pfnLcdCursor(row[i],col[i],CURSOR_UNDER);
                        ++dt[i];
                        if(dt[i]>max[i])
                        {
                            dt[i]=min[i];
                        }
                        else if(dt[i]<min[i])
                        {
                            dt[i]=max[i];
                        }
                        sprintf(str,"%02d",dt[i]);
                        dh->pfnLcdPrintStr(row[i],col[i],str);
                    }break;
                
                case 2:
                    {
                        dh->pfnLcdCursor(row[i],col[i],CURSOR_UNDER);
                        --dt[i];
                        if(dt[i]>=max[i])
                        {
                            dt[i]=min[i];
                        }
                        else if(dt[i]<min[i])
                        {
                            dt[i]=max[i];
                        }
                        sprintf(str,"%02d",dt[i]);
                        dh->pfnLcdPrintStr(row[i],col[i],str);
                    }break;
                
                case 3:
                    {
                        dh->pfnLcdCursor(row[i],col[i],CURSOR_UNDER);
                        i-=1;
                        if(i<0)
                        {
                            done=0;
                            i=0;
                            dh->pfnLcdClear();
                        }
                    }break;
                
                case 4:
                    {
                        dh->pfnLcdCursor(row[i],col[i],CURSOR_UNDER);
                        i+=1;
                        if(i>=12)
                        {
                            done=0;
                            i=0;
                            // save the array in the schedule calender
                            
                            auto_mode_data.start_cal.dd=dt[0];
                            auto_mode_data.start_cal.mm=dt[1];
                            auto_mode_data.start_cal.yy=dt[2];
                            auto_mode_data.start_cal.h=dt[3];
                            auto_mode_data.start_cal.m=dt[4];
                            auto_mode_data.start_cal.s=dt[5];

                            auto_mode_data.stop_cal.dd=dt[6];
                            auto_mode_data.stop_cal.mm=dt[7];
                            auto_mode_data.stop_cal.yy=dt[8];
                            auto_mode_data.stop_cal.h=dt[9];
                            auto_mode_data.stop_cal.m=dt[10];
                            auto_mode_data.stop_cal.s=dt[11];
                            dh->pfnLcdClear();
                            dh->pfnGetClock(&cal_now);
                            validityCnt=ValidateSchedule(&cal_now,&auto_mode_data.start_cal);
                            
                            if(validityCnt>=10)
                            {
                                sprintf(str,"Valid:%d  ",validityCnt);
                                dh->pfnLcdPrintStr(1,0,str);
                                auto_mode_data.auto_mode=1;
                                auto_mode_data.sec_to_run=ValidateSchedule(&auto_mode_data.start_cal,&auto_mode_data.stop_cal);
                                dh->pfnDebugPrint("Seconds to run:%d",(int)auto_mode_data.sec_to_run);
                                set_struct(&auto_mode_data,sizeof(auto_mode_t),AUTOSETTINGS);
                                nxtState=1;
                                dh->pfnDelay(16000000/3);
                            }
                            else
                            {
                                sprintf(str,"Invalid:%d",validityCnt);
                                dh->pfnLcdPrintStr(1,0,str);
                                auto_mode_data.auto_mode=0;
                                nxtState=0;
                                dh->pfnDelay(16000000/3);
                            }
                        }
                        
                    }break;
                
                case 9:
                    {
                        dh->pfnLcdCursor(row[i],col[i]+1,CURSOR_UNDER);
                    }break;
                }
        }
    return nxtState;
}


static uint32_t auto_wait_state(void){
    
    char str[20];	
	int prevSec;
	int32_t tmDiff;
	uint32_t absTicks;
    calender_t cal_now;
	uint32_t keyVal;
    
    dh->pfnDebugPrint("[INFO]:auto_wait_state..\n");
    
    REMEMBER_THIS_STATE();
	
    get_struct(&auto_mode_data,sizeof(auto_mode_t),AUTOSETTINGS);
    dh->pfnGetClock(&cal_now);
    
    dh->pfnLcdClear();
    
	//dh->pfnLcdPrintStr(0,0,"   START TIME ");
	sprintf(str,"START DATE:%02d/%02d/%02d",auto_mode_data.start_cal.dd,auto_mode_data.start_cal.mm,auto_mode_data.start_cal.yy);
	dh->pfnLcdPrintStr(0,0,str);
	sprintf(str,"START TIME:%02d:%02d:%02d",auto_mode_data.start_cal.h,auto_mode_data.start_cal.m,auto_mode_data.start_cal.s);
	dh->pfnLcdPrintStr(1,0,str);
	
	//dh->pfnLcdPrintStr(0,20,"    CURRENT TIME ");
	sprintf(str," CURR DATE:%02d/%02d/%02d",cal_now.dd,cal_now.mm,cal_now.yy);
	dh->pfnLcdPrintStr(2,0,str);
	sprintf(str," CURR TIME:%02d:%02d:%02d",cal_now.h,cal_now.m,cal_now.s);
	dh->pfnLcdPrintStr(3,0,str);
    
    dh->pfnIntEnable01Hz();
    
     do{
		 keyVal=key_scanner();
        
         if(rtc_event)
         {
            rtc_event=0;
            dh->pfnGetClock(&cal_now);
            sprintf(str," CURR DATE:%02d/%02d/%02d",cal_now.dd,cal_now.mm,cal_now.yy);
            dh->pfnLcdPrintStr(2,0,str);
            sprintf(str," CURR TIME:%02d:%02d:%02d",cal_now.h,cal_now.m,cal_now.s);
            dh->pfnLcdPrintStr(3,0,str);
            //Logic for start in auto state
            tmDiff=ValidateSchedule(&cal_now,&auto_mode_data.start_cal);
            if(tmDiff<=0)
            {
                absTicks=auto_mode_data.sec_to_run;
                //check if stop time is over
                //If time is not over then run the remaining ticks
                if((absTicks-tmDiff)>0)
                {
                    keyVal=4; 
                }
                break;
            }
             
         }
         
     }while(keyVal==9);
     
     dh->pfnIntDisable01Hz();
    return keyVal;
}

static uint32_t auto_exit_state(void){
    
    uint32_t timeOut=0;
    uint32_t keyVal,retVal;
	dh->pfnLcdClear();
	dh->pfnLcdPrintStr(0, 0,"  TO EXIT AUTO MODE     ");
	dh->pfnLcdPrintStr(1, 0,"                        ");
	dh->pfnLcdPrintStr(2, 0,"     PRESS ENTER        ");
	dh->pfnLcdPrintStr(3, 0,"        ..              ");
    
    dh->pfnDelay(100000);
    dh->pfnIntEnable01Hz();
    retVal=1;
	do{
        keyVal=key_scanner();
        if(rtc_event)
        {
            rtc_event=0;
            timeOut++;
            if(timeOut>=5)
            {
               retVal=1;
               keyVal=3;
            }
            
        }
    }while(keyVal==9);
	
    if(keyVal==4)
    {
        retVal=0;
        auto_mode_data.auto_mode=0;
        auto_mode_data.sec_to_run=0;
        set_struct(&auto_mode_data,sizeof(auto_mode_t),AUTOSETTINGS);
    }
	return retVal;
}

static uint32_t memory_status(void){
    uint32_t key_val;
    dh->pfnDebugPrint("[INFO]:memory_status..\n");
    dh->pfnLcdClear();
    dh->pfnLcdPrintStr(0,0,"    MEMEORY STATUS    ");
    dh->pfnLcdPrintStr(1,0,"    <10 % USED        ");
    dh->pfnLcdPrintStr(3,0," RESS ANY KEY TO EXIT ");
    
    //Set cursor over button
    dh->pfnLcdCursor(0,0,CURSOR_OFF);
    
    dh->pfnIntEnable01Hz();
    
    do{
        key_val=key_scanner();
        dh->pfnLcdPrintStr(3,0,"                      ");
        
        if(rtc_event){
            rtc_event=0;
            dh->pfnLcdPrintStr(3,0,"PRESS ANY KEY TO EXIT");
            dh->pfnDelay(4000000);
        }
     }while(key_val==9);
    
     dh->pfnIntDisable01Hz();
    
    return 0;
}
static uint32_t memory_erase(void){
    uint32_t key_val;
    dh->pfnDebugPrint("[INFO]:memory_erase..\n");
    dh->pfnLcdClear();
    dh->pfnLcdPrintStr(0,0,"ALL DATA AND SUMMARY");
    dh->pfnLcdPrintStr(1,0,"   WILL BE ERASED   ");//                ");
		dh->pfnLcdPrintStr(2,0,"   ENTER TO DELETE  ");
    dh->pfnLcdPrintStr(3,0,"   MENU  TO SKIP    ");
   
    do{
        key_val=key_scanner();    
     }while(key_val==9);
     
     if(key_val==4){
        dh->pfnLcdClear();
        dh->pfnLcdPrintStr(1,0,"   DELETING FILE... ");
        dh->pfnDelay(8000000);
        dh->pfnLcdClear();
        dh->pfnLcdPrintStr(1,0,"   FILE DELETED...  ");
        dh->pfnDeleteLogFile();
        dh->pfnDelay(8000000);
        // Clear the summary
        reset_summary(); 
        dh->pfnCreateLogFile();
     }
     
    return 0;
}

static uint32_t sampling_summary(void){
	uint32_t key_val;
    char str[32];
	char fId[16];
	calender_t lCal;
	dh->pfnLcdClear();
	//Get the latest summary
    
    LOAD_SUMMARY();
    
	//print formatted summary on screen
	sprintf(str,"TV:%0.3f m3\n",summary_data.tv);    
    dh->pfnLcdPrintStr(0,0,str);
    
    sprintf(str,"ET:%02d:%02d hr\n",summary_data.et/3600,((summary_data.et%3600)/60));
    dh->pfnLcdPrintStr(0,12,str);
	
	sprintf(str,"FR:%0.1f\n",summary_data.afr);
    dh->pfnLcdPrintStr(1,0,str);
    
	sprintf(str,"FP:%0.0f\n",summary_data.afp);
	dh->pfnLcdPrintStr(1,12,str);
	
	sprintf(str,"CV:%0.2f \n",summary_data.cv);
	dh->pfnLcdPrintStr(2,0,str);
	
    get_struct(&user_data,sizeof(user_settings_t),USERSETTINGS);
	sprintf(str,"FID:%8s\n",user_data.f_id);
    dh->pfnLcdPrintStr(2,8,str);
    
//    sprintf(str,"START:%02d/%02d/%02d %02d:%02d:%02d\n",summary_data.start_cal.dd,summary_data.start_cal.mm,\
//                            summary_data.start_cal.yy,summary_data.start_cal.h,summary_data.start_cal.m,summary_data.start_cal.s);
//    dh->pfnLcdPrintStr(2,0,str);
    
//    if(summary_data.err_code==0){
//         dh->pfnLcdPrintStr(2,26,"NORMAL");
//    }
//    
//    else if(summary_data.err_code==1){
//         dh->pfnLcdPrintStr(2,26,"UNDER FLOW");
//    }
//    
//    else if(summary_data.err_code==2){
//         dh->pfnLcdPrintStr(2,26,"OVER PRESSURE");
//    }
//    
//    else if(summary_data.err_code==3){
//         dh->pfnLcdPrintStr(2,26,"FILTER CHOKED");
//    }
    
    
    
    sprintf(str,"ST:%02d/%02d/%02d %02d:%02d:%02d\n",summary_data.stop_cal.dd,summary_data.stop_cal.mm,\
                            summary_data.stop_cal.yy,summary_data.stop_cal.h,summary_data.stop_cal.m,summary_data.stop_cal.s);
    dh->pfnLcdPrintStr(3,0,str);
    
    dh->pfnIntEnable01Hz();
    
    do{
        key_val= key_scanner();
        if(rtc_event)
        {
            rtc_event=0;
            dh->pfnGetClock(&lCal);
            //sprintf(str,"CT:%02d:%02d:%02d",lCal.h,lCal.m,lCal.s);
            //dh->pfnLcdPrintStr(3,26,str);	       
        }
	}while(key_val==9);
    
    dh->pfnIntDisable01Hz();
    
    //check if user entry is from main menu
    if(auto_mode_data.auto_mode==2){
        //go back to main menu
        key_val=5;
    }
    
    return key_val;
}


static uint32_t leak_check(void){
    uint32_t key_val;
    uint32_t duty;
    uint32_t tout;
    float fr,fp;
    char str[32];
    dh->pfnLcdClear();
    dh->pfnLcdPrintStr(0,8,"MOUNT LEACK CHECK ADAPTER");
    dh->pfnLcdPrintStr(1,14,"PRESS ENTER");
    
    do{
		key_val=key_scanner();
	}while(key_val!=4);
    
    dh->pfnLcdClear();
    dh->pfnLcdPrintStr(0,8,"AUTO LEACK CHECK STARTED");
    
    dh->pfnPumpOn();
    duty=50000;
    dh->pfnValveOpen(duty);
    dh->pfnIntEnable01Hz();
    do
    {
        key_val=key_scanner();
        
        if(key_val==1){
            duty+=100;
        }
        
        if(key_val==2){
            duty-=100;
        }
        
        duty&=65535;
        
        dh->pfnValveOpen(duty);
         
        if(rtc_event){
            rtc_event=0;
            get_sensor_values(raw_sensor_values,&calib_data);
            sprintf(str,"FR:%0.1f LPM FP:%03d mmHg ",raw_sensor_values[3],(int)raw_sensor_values[2]);
            dh->pfnLcdPrintStr(1,8,str);
            fp+=10;
        }
	}while(fp<=300);
    dh->pfnValveClose();
    dh->pfnPumpOff();
    dh->pfnLcdClear();
    dh->pfnLcdPrintStr(0,2,"AUTO LEAK CHECK INITIATED FOR 180 SEC");
    tout=180;
    
    do{
        if(rtc_event){
            rtc_event=0;
            get_sensor_values(raw_sensor_values,&calib_data);
            sprintf(str,"FP:%03d Time:%03d",(int)fp,180-tout);
            dh->pfnLcdPrintStr(1,14,str);
            tout--;
        }
    }while(tout);
    
    get_struct(&calib_data,sizeof(calibration_t),CALIBRATION);
    fp=270;
    
    if((300-fp)>calib_data.leak_chk_limit){
        dh->pfnLcdPrintStr(2,14,"LEACK CHECK FAILED!!");
	}
	else{
		dh->pfnLcdPrintStr(2,14,"LEACK CHECK PASSED..");
	}
    
    dh->pfnLcdPrintStr(3,14,"PRESS ENTER TO EXIT");	
	do{
		 key_val=key_scanner();
	}while(key_val!=4);
    
    return 0;
}


static uint32_t leak_check_limit(void){

    dh->pfnDebugPrint("[INFO]:leak_check_limit..\n");
    dh->pfnLcdClear();
    dh->pfnLcdPrintStr(0,0,"     LEAK LIMIT     ");
    edit_integer_value_min_max(&calib_data.leak_chk_limit,"%03d mmHg",1,6,1,100,1);
    dh->pfnLcdPrintStr(3,6, "SAVING..");
    set_struct(&calib_data,sizeof(calibration_t),CALIBRATION);
    dh->pfnDelay(50000000/6);
    dh->pfnLcdPrintStr(3,6, "SAVED.. ");
    dh->pfnDelay(50000000/6);
    
    return 0;
}


static uint32_t shdn_flow_rate(void){

    dh->pfnDebugPrint("[INFO]:shdn_flow_rate..\n");
    dh->pfnLcdClear();
    dh->pfnLcdPrintStr(0,0, "   SHUTDOWN FLOW    ");
		dh->pfnLcdPrintStr(1,0, "          LPM       ");
		edit_float_value(&calib_data.shdn_fr,"%04.1f ",1,5);
    dh->pfnLcdPrintStr(2,6, "SAVING..");
    set_struct(&calib_data,sizeof(calibration_t),CALIBRATION);
    dh->pfnDelay(50000000/6);
    dh->pfnLcdPrintStr(2,6, "SAVED.. ");
    dh->pfnDelay(50000000/6);
    
    return 0;
}

static uint32_t shdn_pressure(void){

    dh->pfnDebugPrint("[INFO]:shdn_pressure..\n");
    dh->pfnLcdClear();
    dh->pfnLcdPrintStr(0,0,"  SHUTDOWN PRESSURE ");
		dh->pfnLcdPrintStr(1,0,"            mmHg    ");
    edit_float_value(&calib_data.shdn_fp,"%05.1f",1,5);
    dh->pfnLcdPrintStr(2,6, "SAVING..");
    set_struct(&calib_data,sizeof(calibration_t),CALIBRATION);
    dh->pfnDelay(50000000/6);
    dh->pfnLcdPrintStr(2,6, "SAVED.. ");
    dh->pfnDelay(50000000/6);
    
    return 0;
}


static void get_sensor_values(volatile float *physical,calibration_t *calib){

    float temp_adc[8];
    dh->pfnGetAdcMv(temp_adc);

    physical[0] = (temp_adc[4]*calib->at.mul) + (calib->at.off);
    physical[1] = (temp_adc[5]*calib->ft.mul) + (calib->ft.off);
    physical[2] = (temp_adc[2]*calib->fp.mul) + (calib->fp.off);
    physical[3] = (temp_adc[1]*calib->fr.mul) + (calib->fr.off);
    physical[4] = (temp_adc[3]*calib->bp.mul) + (calib->bp.off);

}

static void apply_flow_compensation(volatile float *physical, float *flow){

    //If compensation is allowed by user
    if(user_data.comp==1){
        //comensation logic
    }

}


void dusthawk_isr_32_hz(void){

    static volatile uint32_t i=0;
    static volatile uint32_t first_iteration=1;
    volatile uint32_t j;

    dh->pfnIndicatorOn();

    get_sensor_values(raw_sensor_values,&calib_data);

    ctrl_avg.at[i] = raw_sensor_values[0];
    ctrl_avg.ft[i] = raw_sensor_values[1];
    ctrl_avg.fp[i] = raw_sensor_values[2];
    ctrl_avg.fr[i] = raw_sensor_values[3];
    ctrl_avg.bp[i] = raw_sensor_values[4];
    i++;
    i &= 63;

    first_iteration++;

    if(first_iteration >= 63)
        first_iteration = 63;

    for(j=0; j <= first_iteration; j++){
        avg.at += ctrl_avg.at[j];
        avg.ft += ctrl_avg.ft[j];
        avg.fr += ctrl_avg.fr[j];
        avg.fp += ctrl_avg.fp[j];
        avg.bp += ctrl_avg.bp[j];
    }

    avg.at /= first_iteration;
    avg.ft /= first_iteration;
    avg.fr /= first_iteration;
    avg.fp /= first_iteration;
    avg.bp /= first_iteration;

// apply_flow_compensation()

    duty_adjst=(int32_t)arm_pid_f32(&PID,(user_data.set_fr - avg.fr));
    duty += duty_adjst;
    if(duty>59000)
        duty=59000;
    else if(duty <2000)
        duty=2000;

    dh->pfnValveOpen(duty);
    dh->pfnIndicatorOff();

}

void dusthawk_isr_1_hz(void){
    rtc_event = 1;
}


static uint32_t verify_fr_and_fp(float fr, float fp){
    
    static int32_t  over_pressure_cnt=0;
    static int32_t  under_flow_cnt=0;
    const  int32_t  threshold=10;
    
    uint32_t ret_val=0;
    
    if((fr<calib_data.shdn_fr)){
        under_flow_cnt++;
        if(under_flow_cnt>=threshold){
            under_flow_cnt=threshold*2;
        }
    }
    
    else if(under_flow_cnt){
         under_flow_cnt--;
    }
    
    if(fp>calib_data.shdn_fp){
        over_pressure_cnt++;
        if(over_pressure_cnt>threshold){
            over_pressure_cnt=threshold*2;
        }
    }
    
    else if(over_pressure_cnt){
        over_pressure_cnt--;
    }
    
    if(over_pressure_cnt==0 && under_flow_cnt==0){
        ret_val=0; 
    }
    else if((under_flow_cnt > threshold) && (over_pressure_cnt < threshold)){
       ret_val=1; 
    }
    else if((over_pressure_cnt > threshold) && (under_flow_cnt < threshold)){
        ret_val=2; 
    }
    else if((over_pressure_cnt > threshold) && (under_flow_cnt > threshold)){
        ret_val=3; 
    }
    
    dh->pfnDebugPrint("under_flow_cnt:%d \t over_pressure_cnt:%d \t alarm:%d\n",under_flow_cnt,over_pressure_cnt,ret_val);
    
    return ret_val;
}

uint32_t run_module(void){
    char str[64];
    uint32_t key_val;
    calender_t cal;
    int32_t rtc_ticks;
    int32_t rem_ticks;
		int32_t abs_ticks;
		int32_t dif_ticks;
    int32_t auto_ticks;
    int32_t alarm;
    int32_t alarm_count;
		int32_t screen_count=0;
    
    REMEMBER_THIS_STATE();
    dh->pfnDebugPrint("run_module\n");
    
    LOAD_SUMMARY();
    LOAD_USERSETTING();
    LOAD_AUTOMODE();
    
    dh->pfnLcdClear();
		dh->pfnLcdCursor(0,0,CURSOR_OFF);
    
    // If Fresh Sampling 
    if(summary_data.et==0){
         // Get the current time stamp
         dh->pfnGetClock(&summary_data.start_cal);
    }
    //Clear the error flag
    summary_data.err_code=0;
    SAVE_SUMMARY();
    
    //sprintf(str,"SD:%02d/%02d/%02d",summary_data.start_cal.dd,summary_data.start_cal.mm,summary_data.start_cal.yy);
    //dh->pfnLcdPrintStr(0,27,str);

    //sprintf(str,"ST:%02d:%02d:%02d",summary_data.start_cal.h,summary_data.start_cal.m,summary_data.start_cal.s);
    //dh->pfnLcdPrintStr(1,27,str);
    
    // If machine is in auto mode 
    if(auto_mode_data.auto_mode==1)
    {
			// Print the Mode onthe screen
			dh->pfnGetClock(&cal);
			dh->pfnLcdPrintStr(0,16,"AUTO");
			//Check if the last auto mode has something to run
			dif_ticks = ValidateSchedule(&cal,&auto_mode_data.start_cal);
			abs_ticks = auto_mode_data.sec_to_run;
			//Calculate the remianing ticks 
			rem_ticks = abs_ticks+dif_ticks;

			dh->pfnDebugPrint("dif_ticks:%d\n",dif_ticks);
			dh->pfnDebugPrint("abs_ticks:%d\n",abs_ticks);
			dh->pfnDebugPrint("rem_ticks:%d\n",rem_ticks);
        
        //If ticks are remaining 
			if(rem_ticks>=0)
			{
					auto_ticks=rem_ticks;
			}

			else
			{
				auto_ticks=0;
				auto_mode_data.auto_mode=0;
						SAVE_AUTOMODE();
				dh->pfnDebugPrint("EXITING AUTO MODE\n");
				return 3;
			}
		}
		else
		{
			dh->pfnLcdPrintStr(0,17,"MAN");
		}
    rtc_ticks=0;
    sample_avg.at=0.0f;
    sample_avg.bp=0.0f;
    sample_avg.fp=0.0f;
    sample_avg.fr=0.0f;
    sample_avg.ft=0.0f;
    alarm=0;
    alarm_count=0;
    
    dh->pfnDelay(50);
    memset((void *)ctrl_avg.fr,16,CTRL_SAMPLE_SIZE);
    dh->pfnIntEnable32Hz();
    dh->pfnPumpOn();
    dh->pfnDelay(50);
    dh->pfnIntEnable01Hz();
    dh->pfnLcdCursor(0,0,CURSOR_OFF);
    
    do{

        key_val=key_scanner();
			
        if(rtc_event){
            rtc_event=0;
            rtc_ticks++;
            
            dh->pfnGetClock(&cal);
            sprintf(str,"%02d/%02d/%02d",cal.dd,cal.mm,cal.yy);
            dh->pfnLcdPrintStr(3,0,str);
            sprintf(str,"%02d:%02d:%02d",cal.h,cal.m,cal.s);
            dh->pfnLcdPrintStr(3,12,str);

						//************************** screen 1 ******************
            if(screen_count==0){	
							sprintf(str,"AT:%04.1f Deg C",avg.at);
							dh->pfnLcdPrintStr(0,0,str);

							sprintf(str,"FT:%04.1f Deg C",avg.ft);
							dh->pfnLcdPrintStr(1,0,str);
						
							sprintf(str,"FR:%04.1f LPM",avg.fr);
							dh->pfnLcdPrintStr(2,0,str);
							summary_data.tv += (float)avg.fr*0.0000166666f;
						
							summary_data.et++;
							sprintf(str,"ET:%02d:%02d",summary_data.et/3600,((summary_data.et%3600)/60));
							dh->pfnLcdPrintStr(2,12,str);
						}
						//*********************** screen 2 **********************
						else{
								sprintf(str,"FP:%03d mmHg ",(int)avg.fp);
								dh->pfnLcdPrintStr(0,0,str);

								sprintf(str,"BP:%04d mbar ",(int)avg.bp);
								dh->pfnLcdPrintStr(1,0,str);					
						
								sprintf(str,"TV:%06.3f m3",summary_data.tv);
								dh->pfnLcdPrintStr(2,0,str);

								sprintf(str," CV:%04.1f",summary_data.cv);
								dh->pfnLcdPrintStr(2,12,str);
						}
												        
            alarm=verify_fr_and_fp(avg.fr,avg.fp);
            
            if(alarm==0){
                alarm_count=0;
            }
            if(alarm==1){
                alarm_count++;
            }
            
            if(alarm==2){
                alarm_count++;
            }
            
            if(alarm==3){
                alarm_count++;
            }
            
            if(alarm_count>=180){
                SAVE_SUMMARY();
                key_val=3;
            }
            
            summary_data.err_code=alarm;
            
            sample_avg.at+=avg.at;
            sample_avg.bp+=avg.bp;
            sample_avg.fp+=avg.fp;
            sample_avg.fr+=avg.fr;
            sample_avg.ft+=avg.ft;
                  
            set_struct(&summary_data,sizeof(summary_t),SUMMARY);
            
            if(auto_mode_data.auto_mode==1)
            {
                auto_ticks--;
                if(auto_ticks <= 0)
                {
                    dh->pfnDebugPrint("AUTO OVER\n");
                    key_val = 5;
								}
            }
            
            if(rtc_ticks>=(user_data.rec_int*60))
            {
                sample_avg.at/=rtc_ticks;
                sample_avg.bp/=rtc_ticks;
                sample_avg.fp/=rtc_ticks;
                sample_avg.fr/=rtc_ticks;
                sample_avg.ft/=rtc_ticks;
                
                summary_data.afp=sample_avg.fp;
                summary_data.afr=sample_avg.fr;
                summary_data.cv_parm.block_size++;
                summary_data.cv_parm.sum += sample_avg.fr;
                summary_data.cv_parm.sum_of_squares+=(sample_avg.fr*sample_avg.fr);
                
                sprintf(str,"BS:%d , FR:%f \n",(int)summary_data.cv_parm.block_size,sample_avg.fr);
                dh->pfnDebugPrint(str);
                
                sprintf(str,"sum:%f , sumofsqw:%f \n",summary_data.cv_parm.sum,summary_data.cv_parm.sum_of_squares);
                dh->pfnDebugPrint(str);
                
                sprintf(str,"ticks:%d\n",rtc_ticks);
                dh->pfnDebugPrint(str);
                
                if(summary_data.cv_parm.block_size>1){
                    float temp_cv = ((summary_data.cv_parm.sum_of_squares) - (pow(summary_data.cv_parm.sum,2)/summary_data.cv_parm.block_size))/(summary_data.cv_parm.block_size - 1);
                    float std_dev = sqrtf(temp_cv);
                    float mean = summary_data.cv_parm.sum/summary_data.cv_parm.block_size;
                    summary_data.cv=(std_dev/mean)*100.0f;
                }
              
                SAVE_SUMMARY();
                
                dh->pfnPrintLogFile("%02d-%02d-%02d,%02d:%02d:%02d,%04.1f,%04.1f,%03d,%04d,%04.1f,%05.2f,%06.3f,%02d:%02d,\n",\
                                    cal.dd,cal.mm,cal.yy,cal.h,cal.m,cal.s,sample_avg.at,sample_avg.ft,(int)sample_avg.fp,(int)sample_avg.bp,sample_avg.fr,\
                                    summary_data.cv,summary_data.tv,(int)summary_data.et/3600,(int)((summary_data.et%3600)/60));
                                    
                rtc_ticks=0;
                sample_avg.at=0.0f;
                sample_avg.bp=0.0f;
                sample_avg.fp=0.0f;
                sample_avg.fr=0.0f;
                sample_avg.ft=0.0f;
            }            
        }
				if(key_val == 1){
					screen_count++;
					if(screen_count>1)
						screen_count = 0;
						dh->pfnLcdPrintStr(0,0,"                ");
						dh->pfnLcdPrintStr(1,0,"                    ");
						dh->pfnLcdPrintStr(2,0,"                    ");
				}
				if(key_val==2){
					screen_count--;
					if(screen_count<0)
						screen_count = 1;
						dh->pfnLcdPrintStr(0,0,"                ");
						dh->pfnLcdPrintStr(1,0,"                    ");
						dh->pfnLcdPrintStr(2,0,"                    ");
				}
    }while((key_val==9||key_val==1||key_val==2||key_val==4));

    dh->pfnGetClock(&summary_data.stop_cal);
    SAVE_SUMMARY();
    dh->pfnPumpOff();
    dh->pfnValveClose();
    dh->pfnIntDisable01Hz();
    dh->pfnIntDisable32Hz();

    return key_val;
}

static void reset_summary(void){
    
    dh->pfnDebugPrint("reset_summary\n");
   
    summary_data.afp=0;
    summary_data.afr=0;
    summary_data.cv=0;
    summary_data.cv_parm.block_size=0;
    summary_data.cv_parm.sum=0;
    summary_data.cv_parm.sum_of_squares=0;
    summary_data.et=0;
    summary_data.ifp=0;
    summary_data.ifr=0;
    summary_data.start_cal.dd=0;
    summary_data.start_cal.mm=0;
    summary_data.start_cal.yy=0;
    summary_data.start_cal.h=0;
    summary_data.start_cal.m=0;
    summary_data.start_cal.s=0;
    summary_data.stop_cal.dd=0;
    summary_data.stop_cal.mm=0;
    summary_data.stop_cal.yy=0;
    summary_data.stop_cal.h=0;
    summary_data.stop_cal.m=0;
    summary_data.stop_cal.s=0;
    summary_data.tv=0;
    summary_data.new_file=1;
    summary_data.err_code=0;
    SAVE_SUMMARY();
}
void dusthawk_init(dusthawk_t *pvDh){
    dh=pvDh;
    
    RECALL_STATE();
    
    if(S>=SIZE){
        S=0;
        REMEMBER_THIS_STATE();
        calib_data.at.mul=1;
        calib_data.at.off=0;
        calib_data.ft.mul=1;
        calib_data.ft.off=0;
        calib_data.bp.mul=1;
        calib_data.bp.off=0;
        calib_data.fr.mul=1;
        calib_data.fr.off=0;
        calib_data.fp.mul=1;
        calib_data.fp.off=0;
        SAVE_CALIBRATION();

        user_data.comp=0;
        strcpy(user_data.f_id,"01234567");
        summary_data.new_file=1;
        user_data.rec_int=5;
        user_data.set_fr=16.7;
        SAVE_USERSETTING();
        reset_summary();
        auto_mode_data.auto_mode=0;
        auto_mode_data.sec_to_run=0;
        summary_data.start_cal.dd=0;
        auto_mode_data.start_cal.mm=0;
        auto_mode_data.start_cal.yy=0;
        auto_mode_data.start_cal.h=0;
        auto_mode_data.start_cal.m=0;
        auto_mode_data.start_cal.s=0;

        auto_mode_data.stop_cal.dd=0;
        auto_mode_data.stop_cal.mm=0;
        auto_mode_data.stop_cal.yy=0;
        auto_mode_data.stop_cal.h=0;
        auto_mode_data.stop_cal.m=0;
        auto_mode_data.stop_cal.s=0;
        SAVE_AUTOMODE();  
    }
    
    LOAD_CALIBRATION();
    LOAD_SUMMARY();
    LOAD_USERSETTING();
    LOAD_AUTOMODE();
}

void dusthawk_start(void){

    dh->pfnDebugPrint("[INFO]:Starting Dust Hawk..\n");

		// print the welcome message
		dh->pfnLcdPrintStr(0,0,"    LATA ENVIRO     ");
		dh->pfnLcdPrintStr(1,0,"       NOIDA        ");
		dh->pfnLcdPrintStr(2,0,"      911 MFC       ");
		dh->pfnLcdPrintStr(3,0,"                    ");

		// Delay a bit
		dh->pfnDelay(8000000);
		dh->pfnDelay(8000000);
		dh->pfnDelay(8000000);
		//dh->pfnDelay(8000000);
		//dh->pfnDelay(8000000);
		//dh->pfnDelay(8000000);
    dh->pfnLcdClear();

    if(dh->pfnMountDisk(0) != 0 ){
       dh->pfnLcdPrintStr(0,0,"    ERROR: DH001    ");
       dh->pfnLcdPrintStr(1,0,"  <.FATAL ERROR.>   ");
       dh->pfnLcdPrintStr(2,0," <.Please Restart.> ");
       WAIT_FOREVER();
    }

    if(dh->pfnCheckDisk(0) !=0 ){
        dh->pfnLcdPrintStr(0,0,"   ERROR: DH002     ");
        dh->pfnLcdPrintStr(1,0,"  NO DATA LOGGING   ");
        dh->pfnLcdPrintStr(2,0,"  <PRESS ANY KEY >  ");
        WAIT_FOR_ANY_KEY();
        WAIT_FOR_ANY_KEY();
    }
    
    else{
         dh->pfnCreateLogFile();
    }

    PID.Kp=25;
    PID.Ki=0.0;
    PID.Kd=0.0;
    arm_pid_init_f32(&PID,1);
    dh->pfnValveClose();
    //Restore Volatile State
    //Start The Finite State Machine
    while(1)
    {
        dh->pfnUSBCallback();
        Input=FSM[S].Task();
        S = FSM[S].Next[Input];
    }
}


