#include <stdint.h>
#include <stdbool.h>
#include "hw_memmap.h"
#include "debug.h"
#include "gpio.h"
#include "hw_types.h"
#include "pin_map.h"
#include "sysctl.h"


#define   FASTFLASHTIME			(uint32_t) 500000
#define   SLOWFLASHTIME			(uint32_t) 4000000

uint32_t delay_time=FASTFLASHTIME;
uint8_t btn_cnt=0;  //记录按键次数
bool press_sw1=false;

void 		Delay(uint32_t value);
void 		S800_GPIO_Init(void);


int main(void)
{
	S800_GPIO_Init();
	while(1)
  {
		
		if (GPIOPinRead(GPIO_PORTJ_BASE,GPIO_PIN_0)	== 0)	////USR_SW1-PJ0 pressed
			{				
				press_sw1 = true;	
			} 
		else         //USR_SW1-PJ0 released
			{
				if (press_sw1) 
					{ 	 
						btn_cnt++; 
						if (btn_cnt >= 4) btn_cnt = 0;	
					}
				press_sw1 = false;
			}
					
		

		switch(btn_cnt)
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
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOJ));			//Wait for the GPIO moduleJ ready	
	
	GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_0|GPIO_PIN_1);			//Set PF0 as Output pin
	
	GPIOPinTypeGPIOInput(GPIO_PORTJ_BASE,GPIO_PIN_0 | GPIO_PIN_1);//Set the PJ0,PJ1 as input pin
	
	GPIOPadConfigSet(GPIO_PORTJ_BASE,GPIO_PIN_0 | GPIO_PIN_1,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU);
}


