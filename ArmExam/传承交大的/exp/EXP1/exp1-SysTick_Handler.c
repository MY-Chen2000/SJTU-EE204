
#include <stdint.h>
#include <stdbool.h>
#include "hw_memmap.h"
#include "debug.h"
#include "gpio.h"
#include "hw_i2c.h"
#include "hw_types.h"
#include "i2c.h"
#include "pin_map.h"
#include "sysctl.h"
#include "systick.h"
#include "interrupt.h"

#define SYSTICK_FREQUENCY		1000			//1000hz

#define   FASTFLASHTIME			(uint32_t) 500000
#define   SLOWFLASHTIME			(uint32_t) 4000000

uint32_t delay_time=FASTFLASHTIME;

void 		Delay(uint32_t value);
void 		S800_GPIO_Init(void);

//systick software counter define
volatile bool pressed_last=false,pressed_present=false;
volatile uint8_t	number_of_keypressed=0;

uint32_t ui32SysClock;

int main(void)
{
	volatile uint16_t	i2c_flash_cnt,gpio_flash_cnt;
	//use internal 16M oscillator, HSI
    ui32SysClock = SysCtlClockFreqSet((SYSCTL_OSC_INT |SYSCTL_USE_OSC), 16000000);		

  SysTickPeriodSet(ui32SysClock/SYSTICK_FREQUENCY);//T=ui32SysClock/SYSTICK_FREQUENCY*(1/ui32SysClock)=1ms
	
	SysTickEnable();
	SysTickIntEnable();
  IntMasterEnable();		

	
	S800_GPIO_Init();
	while (1)
	{
		switch(number_of_keypressed)
		{
			case 1:
			{
				GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0, GPIO_PIN_0);			// Turn off the LED.
				Delay(delay_time);
				GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0, 0x0);							// Turn on the LED.
				Delay(delay_time);
				break;
			}
			case 2:
			{
				GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0, 0x0);			// Turn off the LED.
				break;
			}
			case 3:
			{
				GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_PIN_1);			// Turn off the LED.
				Delay(delay_time);
				GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0x0);							// Turn on the LED.
				Delay(delay_time);
				break;
			}
			case 4:
			{
				GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0x0);			// Turn off the LED.
				break;
			}
			default:
				break;
		}
	}
}

void Delay(uint32_t value)
{
	uint32_t ui32Loop;
	for(ui32Loop = 0; ui32Loop < value; ui32Loop++){};
}

void S800_GPIO_Init(void)
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);						//Enable PortF
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF));			//Wait for the GPIO moduleF ready
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOJ);						//Enable PortJ	
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOJ)){};			//Wait for the GPIO moduleJ ready	
	
  GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_1);			//Set PF0 as Output pin
	GPIOPinTypeGPIOInput(GPIO_PORTJ_BASE,GPIO_PIN_0 | GPIO_PIN_1);//Set the PJ0,PJ1 as input pin
	GPIOPadConfigSet(GPIO_PORTJ_BASE,GPIO_PIN_0 | GPIO_PIN_1,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU);
}


/*
	Corresponding to the startup_TM4C129.s vector table systick interrupt program name
*/
void SysTick_Handler(void)
{
	pressed_last=pressed_present;
	
	if (GPIOPinRead(GPIO_PORTJ_BASE,GPIO_PIN_0)==0)
		pressed_present=true;
	else
		pressed_present=false;
	
	if ((pressed_last==false)&&(pressed_present==true))
		number_of_keypressed++;

	if (number_of_keypressed==5)
		number_of_keypressed=1;

}
