#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_i2c.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "driverlib/i2c.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"

#include "rtc.h"

#define RTC_PERIPH     SYSCTL_PERIPH_I2C0
#define RTC_GPIO			 SYSCTL_PERIPH_GPIOB	
#define RTC_PORT       GPIO_PORTB_BASE
#define RTC_BASE			 I2C0_BASE
#define RTC_PIN_SCL		 GPIO_PIN_2
#define RTC_PIN_SDA		 GPIO_PIN_3
#define RTC_SCL				 GPIO_PB2_I2C0SCL
#define RTC_SDA				 GPIO_PB3_I2C0SDA



void rtc_init(void)
{
    //enable I2C module 0
    SysCtlPeripheralEnable(RTC_PERIPH);
 
    //reset module
    SysCtlPeripheralReset(RTC_PERIPH);
     
    //enable GPIO peripheral that contains I2C 2
    SysCtlPeripheralEnable(RTC_GPIO);
 
    // Configure the pin muxing for I2C0 functions on port B2 and B3.
    GPIOPinConfigure(RTC_SCL);
    GPIOPinConfigure(RTC_SDA);
     
    // Select the I2C function for these pins.
    GPIOPinTypeI2CSCL(RTC_PORT, RTC_PIN_SCL);
    GPIOPinTypeI2C(RTC_PORT, RTC_PIN_SDA);
 
    // Enable and initialize the I2C0 master module.  Use the system clock for
    // the I2C0 module.  The last parameter sets the I2C data transfer rate.
    // If false the data rate is set to 100kbps and if true the data rate will
    // be set to 400kbps.
    I2CMasterInitExpClk(RTC_BASE, SysCtlClockGet(), false);
     
    //clear I2C FIFOs
    HWREG(RTC_BASE + I2C_O_FIFOCTL) = 80008000;
		
		
}

//sends an I2C command to the specified slave
static void I2CSend(uint8_t slave_addr, uint8_t *buff, uint32_t count)
{
    // Tell the master module what address it will place on the bus when
    // communicating with the slave.
    I2CMasterSlaveAddrSet(RTC_BASE, slave_addr, false);
     
    //put data to be sent into FIFO
    I2CMasterDataPut(RTC_BASE, buff[0]);
     
    //if there is only one argument, we only need to use the
    //single send I2C function
    if(count == 1)
    {
        //Initiate send of data from the MCU
        I2CMasterControl(RTC_BASE, I2C_MASTER_CMD_SINGLE_SEND);
         
        // Wait until MCU is done transferring.
        while(I2CMasterBusy(RTC_BASE));
			
    }
     
    //otherwise, we start transmission of multiple bytes on the
    //I2C bus
    else
    {
        //Initiate send of data from the MCU
        I2CMasterControl(RTC_BASE, I2C_MASTER_CMD_BURST_SEND_START);
         
        // Wait until MCU is done transferring.
        while(I2CMasterBusy(RTC_BASE));
         
        //send num_of_args-2 pieces of data, using the
        //BURST_SEND_CONT command of the I2C module
        for(uint8_t i = 1; i < (count - 1); i++)
        {
            //put next piece of data into I2C FIFO
            I2CMasterDataPut(RTC_BASE, buff[i]);
            //send next data that was just placed into FIFO
            I2CMasterControl(RTC_BASE, I2C_MASTER_CMD_BURST_SEND_CONT);
     
            // Wait until MCU is done transferring.
            while(I2CMasterBusy(RTC_BASE));
        }
     
        //put last piece of data into I2C FIFO
        I2CMasterDataPut(RTC_BASE, buff[count-1]);
        //send next data that was just placed into FIFO
        I2CMasterControl(RTC_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
        // Wait until MCU is done transferring.
        while(I2CMasterBusy(RTC_BASE));
         
    }
}

//read specified register on slave device
static uint8_t I2CReceive(uint8_t slave_addr, uint8_t reg)
{
    //specify that we are writing (a register address) to the
    //slave device
    I2CMasterSlaveAddrSet(RTC_BASE, slave_addr, false);
 
    //specify register to be read
    I2CMasterDataPut(RTC_BASE, reg);
 
    //send control byte and register address byte to slave device
    I2CMasterControl(RTC_BASE, I2C_MASTER_CMD_BURST_SEND_START);
     
    //wait for MCU to finish transaction
    while(I2CMasterBusy(RTC_BASE));
     
    //specify that we are going to read from slave device
    I2CMasterSlaveAddrSet(RTC_BASE, slave_addr, true);
     
    //send control byte and read from the register we
    //specified
    I2CMasterControl(RTC_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);
     
    //wait for MCU to finish transaction
    while(I2CMasterBusy(RTC_BASE));
     
    //return data pulled from the specified register
    return I2CMasterDataGet(RTC_BASE);
}



/*-----------------------------------------------------------------------------
 RTC_ByteWrite
-------------------------------------------------------------------------------*/

void RtcByteWrite(uint8_t addr, uint8_t data){
	uint8_t buff[2];
	buff[0]=addr;
	buff[1]=data;
  I2CSend(RTC_ADDR,buff,2);
}


/*-----------------------------------------------------------------------------
 RTC_ByteRead
-------------------------------------------------------------------------------*/
uint8_t RtcByteRead(uint8_t addr){
	return(uint8_t)I2CReceive(RTC_ADDR,addr);
}

static uint32_t BCDToDecimal(uint8_t bcdByte){
  return (((bcdByte & 0xF0) >> 4) * 10) + (bcdByte & 0x0F);
}

static uint8_t DecimalToBCD (uint32_t decimalByte){
  return (((decimalByte / 10) << 4) | (decimalByte % 10));
}

uint8_t RtcGet(uint8_t param){ 
	return (uint8_t)BCDToDecimal(RtcByteRead(param));
}

void RtcSet(uint8_t param,uint8_t value){
	   RtcByteWrite(param,DecimalToBCD(value));
}

void ds3231_enable_sqw(void){
	uint8_t reg_val;
	reg_val=RtcGet(CONTROL_REG);
	reg_val&=~(0x04);
	RtcSet(CONTROL_REG,reg_val);	
}

void ds3231_disable_sqw(void){
	uint8_t reg_val;
	reg_val=RtcGet(CONTROL_REG);
	reg_val|=(0x04);
	RtcSet(CONTROL_REG,reg_val);	
}

void ds3231_enable_32k(void){
	uint8_t reg_val;
	reg_val=RtcGet(STATUS_REG);
	reg_val|=(0x08);
	RtcSet(STATUS_REG,reg_val);
}

void ds3231_disable_32k(void){
	uint8_t reg_val;
	reg_val=RtcGet(STATUS_REG);
	reg_val&=~(0x08);
	RtcSet(STATUS_REG,reg_val);
}
