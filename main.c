#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "i2clib.h"
#include "main.h"

#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_gpio.h"
#include "inc/hw_i2c.h"

#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom_map.h"
#include <driverlib/timer.h>
#include "driverlib/gpio.h"
#include "driverlib/i2c.h"
#include "driverlib/interrupt.h"
#include "driverlib/adc.h"
#include "inc/tm4c123gh6pm.h"

#include "driverlib/uart.h"
#include "utils/uartstdio.h"

#include "libs/ssd1306/ssd1306.h"
#include "libs/ssd1306/ssd1306_info.h"
#include "libs/ssd1306/ssd1306_utils.h"
#include "libs/ssd1306/oled_font.h"
#include "libs/ssd1306/oled_font_source_pro.h"

#define ADDR_HTU21          0x40                // I2C address of the HTU21 sensor
#define CMD_MEAS_HUM_HM     0xE5
#define CMD_MEAS_HUM_NH     0xF5
#define CMD_MEAS_TMP_HM     0xE3
#define CMD_MEAS_TMP_NH     0xF3
#define CMD_READ_TMP        0xE0
#define CMD_RESET           0xFE
#define CMD_WRITE_USR       0xE6
#define CMD_READ_USR        0xE7
#define CMD_WRITE_HTR       0x51
#define CMD_READ_HTR        0x11
#define CMD_READ_ID_L0      0xFA
#define CMD_READ_ID_L1      0x0F
#define CMD_READ_ID_H0      0xFC
#define CMD_READ_ID_H1      0xC9
#define CMD_READ_FW_V0      0x84
#define CMD_READ_FW_V1      0xB8

#define MSK_RES1            0x80                // Bitmask for bit 7 of the HTU21 configuration register
#define MSK_VDDS            0x40                // Bitmask for bit 6 of the HTU21 configuration register
#define MSK_HTRE            0x04                // Bitmask for bit 2 of the HTU21 configuration register
#define MSK_RES0            0x01                // Bitmask for bit 0 of the HTU21 configuration register

uint32_t    wake_interval;

uint16_t    humidity;
uint16_t    temperature;
uint16_t    configuration;
uint8_t     resolution;
uint8_t     vdd_status;
uint8_t     heater_status;
uint32_t 		lightValue;

uint16_t    ConvertToC(uint16_t);
uint16_t    ConvertToF(uint16_t);
uint16_t    ConvertToRH(uint16_t);

void        ConfigureUSBUART0(void);
void        ConfigureBluetoothUART1(void);
void        ConfigureI2C0(void);
void        ConfigureLCD(void);
void 				ADC0_Init(void);

void        BluetoothPrint(char* const str);
void        print(char const*, ...);

void        TimerInterruptInit(void);
void        ResetTimer0A(void);

void        HUT21Send(uint8_t);
uint32_t    HTU21Receive(void);

void 				ADC0_Handler(void);

void        TIMER0A_ISR(void);

//*****************************************************************************

uint16_t ConvertToC(uint16_t raw) {
    return 175.72*raw/65536.0f-46.85;
}

uint16_t ConvertToF(uint16_t raw) {
    return ConvertToC(raw)*1.8+32;
}

uint16_t ConvertToRH(uint16_t raw) {
    return 125*raw/65536.0f-6;
}

void ConfigureUSBUART0(void) {
    //
    // Enable Peripheral Clocks
    //
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    //
    // Enable pin PA0 for UART0 U0RX
    //
    MAP_GPIOPinConfigure(GPIO_PA0_U0RX);
    MAP_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0);

    //
    // Enable pin PA1 for UART0 U0TX
    //
    MAP_GPIOPinConfigure(GPIO_PA1_U0TX);
    MAP_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_1);

    //
    // Use the internal 16MHz oscillator as the UART0 clock source.
    //
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

    //
    // Initialize the UART for console I/O.
    //
    UARTStdioConfig(0, 115200, 16000000);
}

void ConfigureBluetoothUART1(void) {

    //
    // Enable GPIO port C and corresponding UART
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);

    //
    // Enable UART Rx and Tx pins PC4 and PC5
    //
    GPIOPinConfigure(GPIO_PC4_U1RX);
    GPIOPinConfigure(GPIO_PC5_U1TX);
    GPIOPinTypeUART(GPIO_PORTC_BASE, GPIO_PIN_4 | GPIO_PIN_5);

    //
    // Configure clock on UART1
    //
    // UARTConfigSetExpClk(UART1_BASE, SysCtlClockGet(), 38400, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));    // HC-06
    UARTConfigSetExpClk(UART1_BASE, SysCtlClockGet(), 9600, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));        // HM-10

}

void ConfigureI2C0(void) {
    //
    // Enable Peripheral Clocks 
    //
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

    // Reset the I2C0 module
    MAP_SysCtlPeripheralReset(SYSCTL_PERIPH_I2C0);

    //
    // Enable pin PB2 for I2C0 I2C0SCL
    //
    MAP_GPIOPinConfigure(GPIO_PB2_I2C0SCL);
    MAP_GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);

    //
    // Enable pin PB3 for I2C0 I2C0SDA
    //
    MAP_GPIOPinConfigure(GPIO_PB3_I2C0SDA);
    MAP_GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);

    // Enable the clock for the I2C0 module.
    MAP_I2CMasterInitExpClk(I2C0_BASE, SysCtlClockGet(), true);

    // Reset I2C0 FIFO
    HWREG(I2C0_BASE + I2C_O_FIFOCTL) = 80008000;

}
void ADC0_Init(void)
{
	 
		SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);	//activate the clock of ADC0
		SysCtlDelay(2);	//insert a few cycles after enabling the peripheral to allow the clock to be fully activated.

		ADCSequenceDisable(ADC0_BASE, 3); //disable ADC0 before the configuration is complete

	
		ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);
		ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_CH0|ADC_CTL_IE|ADC_CTL_END);

		
		IntPrioritySet(INT_ADC0SS3, 0x00);  	 // configure ADC0 SS1 interrupt priority as 0
		IntEnable(INT_ADC0SS3);    				// enable interrupt 31 in NVIC (ADC0 SS1)
		ADCIntEnableEx(ADC0_BASE, ADC_INT_SS3);      // arm interrupt of ADC0 SS1
	
		ADCSequenceEnable(ADC0_BASE, 3); //enable ADC0
}

void ConfigureGPIOPortE(void) {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);                    // Activate the clock of GPIO_PORTE
    GPIOPinConfigure(GPIO_PCTL_PE3_AIN0);                        // Configure pin PE3 as analog input
    GPIOPinTypeGPIOInput(GPIO_PORTE_BASE, GPIO_PIN_3);                // Configure pin PE3 as input (might be redundant)
}




void BluetoothPrint(char* const str) {
    char* c = str;
    while(*c) {
        UARTCharPut(UART1_BASE, *c++);
    }
}

// Currently does not handle '%' on USB UART
void print(const char* format, ... ) {
    static char buf[256];
    va_list args;
    va_start( args, format );
    vsnprintf( buf, 256, format, args );
    va_end( args );

    UARTprintf(buf);                                                    // Print to stdio (default is UART0/USB)
    BluetoothPrint(buf);                                                // Print to Bluetooth (UART1)

    ssd1306ClearDisplay();                                              // Clear the ssd1306 LED
    ssd1306PrintString(buf, 0, 0, source_pro_set);                      // Print to ssd1360 LED (I2C0)
}

void TimerInterruptInit(void) {
    IntRegister(INT_TIMER0A, TIMER0A_ISR);
    IntPrioritySet(INT_TIMER0A, 0x00);
    IntEnable(INT_TIMER0A);
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    IntPendClear(INT_TIMER0A);
}

void ResetTimer0A() {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);                       // Enable the clock to TIMER0
    TimerDisable(TIMER0_BASE, TIMER_A);                                 // Disable TIMER0A while configuring
    TimerConfigure(TIMER0_BASE, TIMER_CFG_A_ONE_SHOT);                  // Set TIMER0A as periodic
    TimerLoadSet(TIMER0_BASE, TIMER_A, wake_interval - 1);              // Set the timer period
}

void HTU21Send(uint8_t reg) {

    //
    // Return if the HTU21 is not ready or connected
    //
    if(!SysCtlPeripheralReady(SYSCTL_PERIPH_I2C0)) {
        return;
    }

    I2CMasterSlaveAddrSet(I2C0_BASE, ADDR_HTU21, false);                // Put the HTU21 device address on the bus

    //specify register to be read
    I2CMasterDataPut(I2C0_BASE, reg);

    //send control byte and register address byte to slave device
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_SEND);

    //wait for MCU to finish transaction
    while(I2CMasterBusy(I2C0_BASE));
}

uint32_t HTU21Receive(void) {

    //
    // Return 0 if the HTU21 is not ready or connected
    //
    if(!SysCtlPeripheralReady(SYSCTL_PERIPH_I2C0)) {
        return 0;
    }

    uint16_t return_val[2];

    I2CMasterSlaveAddrSet(I2C0_BASE, ADDR_HTU21, true);

    //send control byte and read from the register we specified
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);

    //wait for MCU to finish transaction
    while(I2CMasterBusy(I2C0_BASE));

    //return data pulled from the specified register
    return_val[1] =  I2CMasterDataGet(I2C0_BASE);
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);

    //wait for MCU to finish transaction
    while(I2CMasterBusy(I2C0_BASE));
    return_val[0] =  I2CMasterDataGet(I2C0_BASE);

    return return_val[1] << 8 | return_val[0];
}

void ADC0_Handler(void)
{	
		ADCIntClear(ADC0_BASE, 3);
		ADCSequenceDataGet(ADC0_BASE, 3, &lightValue);
		UARTprintf("Current Value of photoresitor :%d", lightValue);
		UARTprintf("\n");
	
		if(lightValue < 100 )
		{
			UARTprintf("Its Dark\n");
		}
		else if(lightValue < 1000 )
		{
			UARTprintf("Its dim\n");
		}
		else
		{
			UARTprintf("Its Bright\n");
		}
	
}


uint8_t i = 0;
void TIMER0A_ISR(void) {
		
    TimerIntClear(TIMER0_BASE, TIMER_A);                                // Clear the interrupt flag

    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1, GPIO_PIN_1);

    ResetTimer0A();                                                     // Reload the timer
//    print("Woke from sleep. %i\n", i++);

    humidity = ConvertToRH( HTU21Receive() );                           // Receive the last humidity reading and convert to relative humidity
    HTU21Send( CMD_READ_TMP );                                          // Tell the HTU21 that we want to read the last temperature measurement
    temperature = ConvertToF( HTU21Receive() );                         // Receive the last temperature reading and convert to Fahrenheit
    HTU21Send( CMD_MEAS_HUM_NH );                                       // Tell the HTU21 to measure the humidity (this command also measures temperature)
		
		ADCProcessorTrigger(ADC0_BASE, 3);
		
    TimerEnable(TIMER0_BASE, TIMER_A);                                  // Restart the sleep timer
}

void ConfigureOutputPort(void) {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_1);
}

int main(void) {
    SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |SYSCTL_OSC_MAIN | SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480), 120000000);
    //SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);

    SysCtlPeripheralSleepDisable(SYSCTL_PERIPH_UART0);                  // Disable USB UART clock while in sleep mode
    SysCtlPeripheralSleepDisable(SYSCTL_PERIPH_UART1);                  // Disable Bluetooth UART clock while in sleep mode
    SysCtlPeripheralSleepDisable(SYSCTL_PERIPH_I2C0);                   // Disable HTU21 I2C clock while in sleep mode
    SysCtlPeripheralSleepDisable(SYSCTL_PERIPH_GPIOA);                  // Disable GPIOA (USB UART) clock while in sleep mode
    SysCtlPeripheralSleepDisable(SYSCTL_PERIPH_GPIOB);                  // Disable GPIOB (Bluetooth UART) clock while in sleep mode
    SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_TIMER0);                  // Enable TIMER0 (wake timer) clock while in sleep mode
    SysCtlPeripheralClockGating(true);

    wake_interval = 5 * SysCtlClockGet();                               // Set wake timer interval to 5s

    ConfigureOutputPort();
    ConfigureUSBUART0();                                                // Configure UART0 for USB stdio
    ConfigureBluetoothUART1();                                          // Configure UART1 for serial Bluetooth
    ConfigureI2C0();                                                    // Configure I2C0 for the HTU21 sensor and SSD1306 LED
		ConfigureGPIOPortE();
		ADC0_Init();																												

    ResetTimer0A();                                                     // Configure the timer
    TimerInterruptInit();                                               // Enable the timer interrupt

    IntMasterEnable();                                                  // Enable interrupts

    // Initialize the LED screen
    ssd1306Init();
    ssd1306TurnOn(true);
    ssd1306ClearDisplay();
    ssd1306AdjustContrast(25);

    print("Enabling sleep wake timer.\n");
    TimerEnable(TIMER0_BASE, TIMER_A);                                  // Start the wake timer

    while(1) {

        //
        // Print to all outputs (USB, Bluetooth, LED)
        //
        print("H: %2d T: %2d\n", humidity, temperature);
				
				SysCtlDelay(SysCtlClockGet()); //Cant get the screen to scroll, so delay for info to be displayed and print a new line
			
				//values for the if statements were test against a flashlight at various distances. not the most accurate scale but should give a general idea
			
				
				if(lightValue < 100 )
				{
					print("Its Dark\n");
				}
				else if(lightValue < 1000 )
				{
					print("Its dim\n");
				}
				else
				{
					print("Its Bright\n");
				}
        //
        // For some reason, this print statement does not execute fully when it calls BluetoothPrint().
        // Adding a breakpoint on UARTprint() or SysCtlSleep() allows it to print completely.
        //
        //print("Going to sleep for 5s...\n");

        GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1, ~GPIO_PIN_1);
        SysCtlSleep();
    }
    		
		return 0;

}
