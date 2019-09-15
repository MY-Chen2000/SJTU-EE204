//initialize 通用初始化*******************88
#include <stdio.h>
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
#include "uart.h"
#include "hw_ints.h"
//*****************************************************************************
//
//I2C GPIO chip address and resigster define
//
//*****************************************************************************
#define TCA6424_I2CADDR 					0x22
#define PCA9557_I2CADDR						0x18

#define PCA9557_INPUT							0x00
#define	PCA9557_OUTPUT						0x01
#define PCA9557_POLINVERT					0x02
#define PCA9557_CONFIG						0x03

#define TCA6424_CONFIG_PORT0			0x0c
#define TCA6424_CONFIG_PORT1			0x0d
#define TCA6424_CONFIG_PORT2			0x0e

#define TCA6424_INPUT_PORT0				0x00
#define TCA6424_INPUT_PORT1				0x01
#define TCA6424_INPUT_PORT2				0x02

#define TCA6424_OUTPUT_PORT0			0x04
#define TCA6424_OUTPUT_PORT1			0x05
#define TCA6424_OUTPUT_PORT2			0x06

#define   FASTFLASHTIME			(uint32_t)50000
#define   SLOWFLASHTIME			(uint32_t)100000

//interrupt
#define SYSTICK_FREQUENCY		1000			//1000hz
#define	I2C_FLASHTIME				500				//500mS
#define GPIO_FLASHTIME			300				//300mS
//*****



int i=0,cmdflag = 3,lastcmdflag = 3, timeflag=1;
uint32_t delay_time;
volatile uint8_t result;
volatile uint8_t posedge=0;	
uint32_t state = 0, ui32SysClock,read_key_value;
uint8_t seg7[] = {0x3f,0x06,0x5b,0x4f,0x66,0x6d,0x7d,0x07,0x7f,0x6f,0x77,0x7c,0x58,0x5e,0x079,0x71,0x5c};
uint32_t ui32SysClock,ui32IntPriorityGroup,ui32IntPriorityMask;
uint32_t ui32IntPrioritySystick,ui32IntPriorityUart0;
uint8_t uart_receive_char, pointer = 0;
uint8_t buffer[16];


void 		S800_GPIO_Init(void);
uint8_t 	I2C0_WriteByte(uint8_t DevAddr, uint8_t RegAddr, uint8_t WriteData);
uint8_t 	I2C0_ReadByte(uint8_t DevAddr, uint8_t RegAddr);
void		S800_I2C0_Init(void);
void 		S800_UART_Init(void);
void Delay(uint32_t value);
void count(uint32_t *state);
void UARTStringPut(const char *cMessage);

uint8_t disp_buff[9];

//the disp_tab and disp_tab_7seg must be the Corresponding relation
//the last character should be the space(not display) character
char	const disp_tab[]			={'0','1','2','3','4','5',     //this could display char and its segment code   
															 '6','7','8','9','A','b',
															 'C','d','E','F',
															 'H','L','P','o',
															 '.','-','_',' '}; 
char const disp_tab_7seg[]	={0x3F,0x06,0x5B,0x4F,0x66,0x6D,  
															0x7D,0x07,0x7F,0x6F,0x77,0x7C,
															0x39,0x5E,0x79,0x71, 
															0x76,0x38,0x73,0x5c,
															0x80,0x40, 0x08,0x00}; 
char ASCII2Disp(char *buff);
//***************
															
//systick software counter define
volatile uint16_t systick_10ms_couter,systick_100ms_couter;
volatile uint8_t	systick_10ms_status,systick_100ms_status;
volatile uint8_t result,cnt,key_value,gpio_status;
volatile uint8_t rightshift = 0x01;
uint32_t ui32SysClock;
//***********************************************

//*********************
void GPIOJ_Handler(void);
void plus_one();
void decrease_one();
void flash();
void Display(void);
void processChar(int8_t c);
bool processCommand(int8_t* str);

//***********************add by cmy
uint8_t light= 0x04;
int flashcount=0;
volatile int initvalue=5950; 
int h2lbit[] = {10000,1000,100,10,1};
//*********************

//systick software counter define
uint8_t idx = 0, times = 100;
volatile uint16_t halfsec_counter, second_counter;
volatile uint8_t	halfsec_status, second_status, min = 0, sec = 0;
volatile uint8_t cnt,key_value,gpio_status;



uint8_t periods[] = {5, 10, 20, 2};
//***************************
char ASCII2Disp(char *buff)
{
	char * pcDisp;
	pcDisp 				=(char*)strchr(disp_tab,*buff);	
	if (pcDisp		== NULL)
		return 0x0;
	else
		return (disp_tab_7seg[pcDisp-disp_tab]);

}
void UARTStringPut(const char *cMessage)
{
	while(*cMessage!='\0')
		UARTCharPut(UART0_BASE,*(cMessage++));
}
void UARTStringPutNonBlocking(const char *cMessage)
{
	while(*cMessage!='\0')
		UARTCharPutNonBlocking(UART0_BASE,*(cMessage++));
}

void S800_UART_Init(void)
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);						//Enable PortA
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA));			//Wait for the GPIO moduleA ready

	GPIOPinConfigure(GPIO_PA0_U0RX);												// Set GPIO A0 and A1 as UART pins.
  GPIOPinConfigure(GPIO_PA1_U0TX);    			

  GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

	// Configure the UART for 115,200, 8-N-1 operation.
  UARTConfigSetExpClk(UART0_BASE, ui32SysClock,115200,(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |UART_CONFIG_PAR_NONE));
	UARTStringPut((uint8_t *)"\r\nHello, world!\r\n");
}
void Displaynum(int num)
{
	Delay(10);
	I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[(num/100000)%10]);
	I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,1);
	I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,0);
	Delay(10);
	I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[(num/10000)%10]);
	I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,2);
	I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,0);
	Delay(10);
	I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[(num/1000)%10]);
	I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,4);
	I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,0);
	Delay(10);
	I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[(num/100)%10]);
	I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,8);
	I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,0);
	Delay(10);
	I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[(num/10)%10]);
	I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,16);
	I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,0);
	Delay(10);
	I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[num%10]);
	I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,32);
	I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,0);
}

int main(void)
{
	volatile uint16_t	i2c_flash_cnt,gpio_flash_cnt;
	//use internal 16M oscillator, PIOSC
   //ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_16MHZ |SYSCTL_OSC_INT |SYSCTL_USE_OSC), 16000000);		
	//ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_16MHZ |SYSCTL_OSC_INT |SYSCTL_USE_OSC), 8000000);		
	//use external 25M oscillator, MOSC
   //ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |SYSCTL_OSC_MAIN |SYSCTL_USE_OSC), 25000000);		

	//use external 25M oscillator and PLL to 120M
   //ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |SYSCTL_OSC_MAIN | SYSCTL_USE_PLL |SYSCTL_CFG_VCO_480), 120000000);;		
	ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_16MHZ |SYSCTL_OSC_INT | SYSCTL_USE_PLL |SYSCTL_CFG_VCO_480), 20000000);
	
  SysTickPeriodSet(ui32SysClock/SYSTICK_FREQUENCY);
	SysTickEnable();
	SysTickIntEnable();																		//Enable Systick interrupt
	  

	S800_GPIO_Init();
	S800_I2C0_Init();
	S800_UART_Init();
	
	IntEnable(INT_UART0);
  UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);	//Enable UART0 RX,TX interrupt
  IntMasterEnable();		
	ui32IntPriorityMask				= IntPriorityMaskGet();
	IntPriorityGroupingSet(3);														//Set all priority to pre-emtption priority
	
	IntPrioritySet(INT_UART0,0x0);													//Set INT_UART0 to highest priority
	IntPrioritySet(FAULT_SYSTICK,0xe0);									//Set INT_SYSTICK to lowest priority
	
	ui32IntPriorityGroup			= IntPriorityGroupingGet();

	ui32IntPriorityUart0			= IntPriorityGet(INT_UART0);
	ui32IntPrioritySystick		= IntPriorityGet(FAULT_SYSTICK);
	
		halfsec_counter = 5 * times;
	second_counter = 10 * times;
	while (1)
	{
		if (second_status)
		{
			second_status = 0;
			switch (cmdflag)
	{
		
		case 1: 
			      timeflag=1;
			      decrease_one();
						break;
		case 2: 
			      timeflag=0;
		        plus_one();
		        break;
		case 3: 
			      timeflag=1;
			      plus_one();
		        break;
		case 4: 
			      timeflag=1;
			      initvalue=0;
		        cmdflag=3;
						break;
	}
			
		}
		if (halfsec_status)
		{
			halfsec_status = 0;
			flash();
		}
		Display();
	}

}

void Display(void)
{
	
	Delay(10);
	I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[(initvalue/1000)%10]);
	I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,1);
	I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,0);
	Delay(10);
	I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[(initvalue/100)%10]);
	I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,2);
	I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,0);
	Delay(10);
	I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[(initvalue/10)%10]);
	I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,4);
	I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,0);
	Delay(10);
	I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[initvalue%10]);
	I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,8);
	I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,0);
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
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);						//Enable PortN	
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPION));		
	
	IntMasterEnable();
	IntEnable(INT_GPIOJ_TM4C123);
	GPIOIntRegister(GPIO_PORTJ_BASE, GPIOJ_Handler);
	
  
	GPIOPinTypeGPIOInput(GPIO_PORTJ_BASE, GPIO_PIN_0 | GPIO_PIN_1);//Set the PJ0,PJ1 as input pin
	GPIOPadConfigSet(GPIO_PORTJ_BASE, GPIO_PIN_0 | GPIO_PIN_1, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
	
	 GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_0);			//Set PF0 as Output pin
  GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_0);			//Set PN0 as Output pin
  GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_1);	
	
	GPIOIntTypeSet(GPIO_PORTJ_BASE, GPIO_INT_PIN_0 | GPIO_INT_PIN_1, GPIO_FALLING_EDGE); // interrupts when pressed (rising edge for released)
	GPIOIntEnable(GPIO_PORTJ_BASE, GPIO_INT_PIN_0 | GPIO_INT_PIN_1);
}

void S800_I2C0_Init(void)
{
	uint8_t result;
  SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	GPIOPinConfigure(GPIO_PB2_I2C0SCL);
  GPIOPinConfigure(GPIO_PB3_I2C0SDA);
  GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
  GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);

	I2CMasterInitExpClk(I2C0_BASE,ui32SysClock, true);										//config I2C0 400k
	I2CMasterEnable(I2C0_BASE);	

	result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_CONFIG_PORT0,0x0ff);		//config port 0 as input
	result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_CONFIG_PORT1,0x0);			//config port 1 as output
	result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_CONFIG_PORT2,0x0);			//config port 2 as output 

	result = I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_CONFIG,0x00);					//config port as output
	result = I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_OUTPUT,0x0ff);				//turn off the LED1-8
	
}


uint8_t I2C0_WriteByte(uint8_t DevAddr, uint8_t RegAddr, uint8_t WriteData)
{
	uint8_t rop;
	while(I2CMasterBusy(I2C0_BASE)){};
	I2CMasterSlaveAddrSet(I2C0_BASE, DevAddr, false);
	I2CMasterDataPut(I2C0_BASE, RegAddr);
	I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);
	while(I2CMasterBusy(I2C0_BASE)){};
	rop = (uint8_t)I2CMasterErr(I2C0_BASE);

	I2CMasterDataPut(I2C0_BASE, WriteData);
	I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
	while(I2CMasterBusy(I2C0_BASE)){};

	rop = (uint8_t)I2CMasterErr(I2C0_BASE);
	return rop;
}

uint8_t I2C0_ReadByte(uint8_t DevAddr, uint8_t RegAddr)
{
	uint8_t value,rop;
	while(I2CMasterBusy(I2C0_BASE)){};	
	I2CMasterSlaveAddrSet(I2C0_BASE, DevAddr, false);
	I2CMasterDataPut(I2C0_BASE, RegAddr);
//	I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);		
	I2CMasterControl(I2C0_BASE,I2C_MASTER_CMD_SINGLE_SEND);
	while(I2CMasterBusBusy(I2C0_BASE));
	rop = (uint8_t)I2CMasterErr(I2C0_BASE);
	Delay(1);
	//receive data
	I2CMasterSlaveAddrSet(I2C0_BASE, DevAddr, true);
	I2CMasterControl(I2C0_BASE,I2C_MASTER_CMD_SINGLE_RECEIVE);
	while(I2CMasterBusBusy(I2C0_BASE));
	value=I2CMasterDataGet(I2C0_BASE);
		Delay(1);
	return value;
}

/*
	Corresponding to the startup_TM4C129.s vector table systick interrupt program name
*/

void plus_one()
{
		if (initvalue%10==9)
	{
		if((initvalue/10)%10==5){
			if((initvalue/100)%10==9){
				if((initvalue/1000)%10==5){
				  initvalue=0;
				}
				else initvalue=((initvalue/1000)+1)*1000;
			}
			else initvalue=((initvalue/100)+1)*100;
		}
		else initvalue=((initvalue/10)+1)*10;
	}
	else ++initvalue;
}

void decrease_one()
{
		if (initvalue%10==0)
	{
		if((initvalue/10)%10==0){
			if((initvalue/100)%10==0){
				if((initvalue/1000)%10==0){
				  initvalue=5959;
				}
				else initvalue=((initvalue/1000)-1)*1000+959;
			}
			else initvalue=((initvalue/100)-1)*100+59;
		}
		else initvalue=((initvalue/10)-1)*10+9;
	}
	else --initvalue;
}


void flash()
{
  flashcount++;
	if(flashcount==2)flashcount=0;
	if(flashcount==0){
	  result = I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_OUTPUT,~light);
	}
  else 	result = I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_OUTPUT,0xFF);
}

void GPIOJ_Handler(void)
{
	GPIOIntClear(GPIO_PORTJ_BASE, GPIO_PIN_0 | GPIO_PIN_1);
	if (!GPIOPinRead(GPIO_PORTJ_BASE,GPIO_PIN_0))
	{
		plus_one();
		second_counter = times * 2;
	}
	if (!GPIOPinRead(GPIO_PORTJ_BASE,GPIO_PIN_1))
	{
		min = (min + 1) % 60;
		halfsec_counter = times * 2;
	}
}
void SysTick_Handler(void)
{
	if (second_counter != 0){
		if(cmdflag!=2)--second_counter;
	}
	else
	{
		if (GPIOPinRead(GPIO_PORTJ_BASE,GPIO_PIN_0))
			second_counter = times * 10;
		else
			second_counter	= times * 2;
		second_status 	= 1;
	}
	if (halfsec_counter	!= 0){
		if(cmdflag!=2)--halfsec_counter;
	}
	else
	{
		if (GPIOPinRead(GPIO_PORTJ_BASE,GPIO_PIN_0))
			halfsec_counter	= times * 5;
		else
			halfsec_counter	= times * 1;
		halfsec_status 	= 1;
	}
}
//**********************bluetooth
/*
	Corresponding to the startup_TM4C129.s vector table UART0_Handler interrupt program name
*/
void UART0_Handler(void)
{
	int32_t uart0_int_status;
  uart0_int_status 		= UARTIntStatus(UART0_BASE, true);		// Get the interrrupt status.

  UARTIntClear(UART0_BASE, uart0_int_status);								//Clear the asserted interrupts

	while(UARTCharsAvail(UART0_BASE))    											// Loop while there are characters in the receive FIFO.
  {
		uart_receive_char = UARTCharGetNonBlocking(UART0_BASE);
		processChar(uart_receive_char);
	}
	processCommand(buffer);
  //*******return input*******
	/*while(UARTCharsAvail(UART0_BASE))    											// Loop while there are characters in the receive FIFO.
  {
		///Read the next character from the UART and write it back to the UART.
    UARTCharPutNonBlocking(UART0_BASE,UARTCharGetNonBlocking(UART0_BASE));
		GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_1,GPIO_PIN_1 );		
		Delay(1000);
	}
	GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_1,0 );	*/
		//************************
}
//******************************
void processChar(int8_t c)
{
	if ((c >= '0' && c <= '9') || (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || c == '+' || c == '-' || c == ':')
	{
		if (c >= 'a' && c <= 'z')
				c = c - 'a' + 'A';
		buffer[pointer++] = c;
	}
	else
	{
		pointer = 0;
		memset(buffer, 0, 16);
	}
	

	
	if (pointer >= 16)
	{
		pointer = 0;
		memset(buffer, 0, 16);
	}
}
bool processCommand(int8_t* str)
{
	
		if (strcmp(str, "AT+CLASS") == 0 )
	{
		UARTStringPut("CLASS F1703303");
		memset(buffer, 0, 16);
		pointer = 0;
	}
	else if (strcmp(str, "AT+STUDENTCODE") == 0)
	{
		UARTStringPut("CODE 517021910499");
		memset(buffer, 0, 16);
		pointer = 0;
	}
	if (strcmp(str, "A") == 0)
	{
		lastcmdflag=cmdflag;
		cmdflag=1;
		UARTStringPut((uint8_t *)"\r\nA\r\n");
		pointer = 0;
		memset(buffer, 0, 16);
		return true;
	}
	else if (strcmp(str, "B") == 0)
	{
	  lastcmdflag=cmdflag;
		cmdflag=2;
		UARTStringPut((uint8_t *)"\r\nB\r\n");
		pointer = 0;
		memset(buffer, 0, 16);
		return true;
	}
	else if (strcmp(str, "C") == 0)
	{
		lastcmdflag=cmdflag;
		cmdflag=3;
		UARTStringPut((uint8_t *)"\r\nC\r\n");
		pointer = 0;
		memset(buffer, 0, 16);
		return true;
	}
	else if (strcmp(str, "D") == 0)
	{
		lastcmdflag=cmdflag;
		cmdflag=4;
		UARTStringPut((uint8_t *)"\r\nD\r\n");
		pointer = 0;
		memset(buffer, 0, 16);
		return true;
	}
	return false;
}