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
#include "string.h"

#define SYSTICK_FREQUENCY		1000			//1000hz

#define	I2C_FLASHTIME				500				//500mS
#define GPIO_FLASHTIME			300				//300mS
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

#define   FASTFLASHTIME			(uint32_t) 300000
#define		MIDDFLASHTIME			(uint32_t) FASTFLASHTIME*10
#define   SLOWFLASHTIME			(uint32_t) FASTFLASHTIME*20


void 		Delay(uint32_t value);
uint32_t delay_time=FASTFLASHTIME;
void 		S800_GPIO_Init(void);
uint8_t 	I2C0_WriteByte(uint8_t DevAddr, uint8_t RegAddr, uint8_t WriteData);
uint8_t 	I2C0_ReadByte(uint8_t DevAddr, uint8_t RegAddr);
void		S800_I2C0_Init(void);
void 		S800_UART_Init(void);
//systick software counter define
volatile uint16_t systick_10ms_couter,systick_100ms_couter;
volatile uint8_t	systick_10ms_status,systick_100ms_status;

volatile uint8_t result,cnt,cnt2,key_value,gpio_status;
volatile uint8_t rightshift = 0x01,rightshift2=2;
uint32_t ui32SysClock,ui32IntPriorityGroup,ui32IntPriorityMask;
uint32_t ui32IntPrioritySystick,ui32IntPriorityUart0;
uint8_t tmp=0;
volatile uint8_t second1 = 0x04, second2=0x08;
volatile uint8_t colon = 0x04, cdata=15 ;
volatile uint8_t minute1 = 0x01, minute2=0x02;
volatile uint8_t sdata1=2, sdata2=1;
volatile uint8_t mdata1=0, mdata2=5;

uint8_t seg7[] = {0x3f,0x06,0x5b,0x4f,0x66,0x6d,0x7d,0x07,0x7f,0x6f,0x77,0x7c,0x58,0x5e,0x079,0x71,0x5c};
//0,1,2,3,4,5,6,7,8,9,A,B,C,d,E,F,o


//UART
uint8_t uart_receive_char, pointer = 0;
uint8_t buffer[16];
bool processCommand(int8_t* c);
void processChar(int8_t c);
void settime(int8_t th, int8_t tm, int8_t ts);
void inctime(int8_t th, int8_t tm, int8_t ts);
void gettime(void);
void Display(void);
void waiting(void);
uint8_t hour = 0, minute = 0, second = 0;
uint8_t state=1;

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


//*************中断优先级********************
	IntPrioritySet(INT_UART0,0x0e0);													//Set INT_UART0 to highest priority
	IntPrioritySet(FAULT_SYSTICK,0x0);									//Set INT_SYSTICK to lowest priority

//**********************************************************


	ui32IntPriorityGroup			= IntPriorityGroupingGet();

	ui32IntPriorityUart0			= IntPriorityGet(INT_UART0);
	ui32IntPrioritySystick		= IntPriorityGet(FAULT_SYSTICK);
	while (1)
	{
		if (systick_10ms_status)
		{
			systick_10ms_status		= 0;
			if (++gpio_flash_cnt	>= GPIO_FLASHTIME/10)
			{
				gpio_flash_cnt			= 0;
				if (gpio_status)
					GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0,GPIO_PIN_0 );
				else
					GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0,0);
				gpio_status					= !gpio_status;
			
			}
		}
		if (systick_100ms_status)
		{
			systick_100ms_status	= 0;
			if (++i2c_flash_cnt		>= I2C_FLASHTIME/100)
			{
				i2c_flash_cnt				= 0;

				//一位跑马灯
				cnt++;
				rightshift= rightshift<<1;

				if (cnt		  >= 0x8)
				{
					rightshift= 0x01;
					cnt 			= 0;
				}
				
				//两位跑马灯，要改systick
				/*
				if (GPIOPinRead(GPIO_PORTJ_BASE,GPIO_PIN_0))
				{
					cnt++;
					cnt2++;
					rightshift= rightshift<<1;
					rightshift2=rightshift2<<1;
				}

				if (cnt		  >= 0x8)
				{
					rightshift= 0x01;
					cnt 			= 0;
				}
				if (cnt2	  >= 0x8)
				{
					rightshift2= 0x01;
					cnt2 			= 0;
				}	
				*/
				
				
				/* 开灯关灯函数
				GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0, GPIO_PIN_0);			// Turn on the LED.
				GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0, 0x0);							// Turn off the LED.	
				delay_time=FASTFLASHTIME;	//改闪烁频率
				*/
				
				/*
				if(GPIOPinRead(GPIO_PORTJ_BASE,GPIO_PIN_0)==0)	//PJ0按下为0
				if(GPIOPinRead(GPIO_PORTJ_BASE,GPIO_PIN_1)==0)	//PJ1
				*/
				
				//倒数计时******************************
/*				if(sdata2==0){
					sdata2=10;
					if(sdata1==0){
						sdata1=6;
						if(mdata2==0){
							mdata2=10;
							if(mdata1==0){
								mdata1=6;
								}
							mdata1--;
							}
						mdata2--;
						}
					sdata1--;
					}
				sdata2--;*/
				//***************************************
				
				//正数计时*******************************
				/*
				sdata2++;
				if(sdata2==10){
					sdata1++;
					sdata2=0;
					if(sdata1==6){
						mdata2++;
						sdata1=0;
						if(mdata2==10){
							mdata2%=10;
							mdata1++;
							mdata1%=6;
							}
						}
					}
				*/
				//****************************************

			}
		}
	}



}

void Delay(uint32_t value)
{
	uint32_t ui32Loop;
	for(ui32Loop = 0; ui32Loop < value; ui32Loop++){};
}


void UARTStringPut(uint8_t *cMessage)
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
void S800_GPIO_Init(void)
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);						//Enable PortF
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF));			//Wait for the GPIO moduleF ready
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOJ);						//Enable PortJ	
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOJ));			//Wait for the GPIO moduleJ ready	
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);						//Enable PortN	
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPION));			//Wait for the GPIO moduleN ready		
	
  GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_0);			//Set PF0 as Output pin
  GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_0);			//Set PN0 as Output pin
  GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_1);			//Set PN1 as Output pin	

	GPIOPinTypeGPIOInput(GPIO_PORTJ_BASE,GPIO_PIN_0 | GPIO_PIN_1);//Set the PJ0,PJ1 as input pin
	GPIOPadConfigSet(GPIO_PORTJ_BASE,GPIO_PIN_0 | GPIO_PIN_1,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU);
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
void SysTick_Handler(void)
{
	//显示4位数码管********************************************************
	tmp++;
	tmp%=4;
	switch(tmp){
		case(0):
	result 							= I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,0);	//write port 1 	
	result 							= I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,minute1);	//write port 2			
	result 							= I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[mdata1]);	//write port 1 
	break;		
		case(1):
	result 							= I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,0);	//write port 1 	
	result 							= I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,minute2);	//write port 2			
	result 							= I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[mdata2]);	//write port 1 		
	break;		
		case(2):
	result 							= I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,0);	//write port 1 		
	result 							= I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,second1);	//write port 2		
	result 							= I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[sdata1]);	//write port 1 		
	break;
		case(3):
	result 							= I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,0);	//write port 1 		
	result 							= I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,second2);	//write port 2		
	result 							= I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[sdata2]);	//write port 1 
	break;
	}	
	
	//********************************************************************************
	
	//2位跑马灯
	/*
	if(systick_10ms_couter%2){
	result 							= I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,0);	//write port 1 		
	result 							= I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,rightshift);	//write port 2		
	result 							= I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[cnt+1]);	//write port 1 		
	result 							= I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_OUTPUT,~rightshift);	
	}else{
	result 							= I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,0);	//write port 1 	
	result 							= I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,rightshift2);	//write port 2			
	result 							= I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[cnt2+1]);	//write port 1 		

	result 							= I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_OUTPUT,~rightshift2);		
	}	
	*/
	
	//1位跑马灯,其实可以写在主函数里
	/*
	result 							= I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[cnt+1]);	//write port 1 				
	result 							= I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,rightshift);	//write port 2	
	result = I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_OUTPUT,~rightshift);	
	*/
	
	if (systick_100ms_couter	!= 0)
		systick_100ms_couter--;
	else
	{
		systick_100ms_couter	= SYSTICK_FREQUENCY/10;
		systick_100ms_status 	= 1;
	}
	
	if (systick_10ms_couter	!= 0)
		systick_10ms_couter--;
	else
	{
		systick_10ms_couter		= SYSTICK_FREQUENCY/100;
		systick_10ms_status 	= 1;
	}
	
	
	//按键就停止***********************************
	while (GPIOPinRead(GPIO_PORTJ_BASE,GPIO_PIN_0) == 0)
	{
		systick_100ms_status	= systick_10ms_status = 0;
		GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0,GPIO_PIN_0);	
	}
	//********************************************
	
	
	GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0,0);
}

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
		///Read the next character from the UART and write it back to the UART.
//    UARTCharPutNonBlocking(UART0_BASE,UARTCharGetNonBlocking(UART0_BASE));
		
		
		
	//接收串口信息*********************************************************	
		uart_receive_char = UARTCharGetNonBlocking(UART0_BASE);
		processChar(uart_receive_char);
		
	//*********************************************************************	
	
	
	}
	GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_1,GPIO_PIN_1 );		
	Delay(1000);
	GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_1,0 );	
	Delay(1000);
	
	//按下J1会亮灯并停止接收信号
	while (GPIOPinRead(GPIO_PORTJ_BASE,GPIO_PIN_1) == 0)
		GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_1,GPIO_PIN_1);	

	GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_1,0);
}

//***********************************************************************
//6位时钟****************************************************************
void Display(void)
{
	waiting();
	I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[hour/10]);
	I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,1);
	waiting();
	I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[hour%10]);
	I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,2);
	waiting();
	I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,0x40);
	I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,4);
	waiting();
	I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[minute/10]);
	I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,8);
	waiting();
	I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[minute%10]);
	I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,0x10);
	waiting();
	I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,0x40);
	I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,0x20);
	waiting();
	I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[second/10]);
	I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,0x40);
	waiting();
	I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[second%10]);
	I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,0x80);
}

void waiting(void)
{
	Delay(I2C_FLASHTIME*25);
}

//处理学号的

void processChar(int8_t c)
{
	switch(c){
		case 'A': state=1;
		case 'B': state=2;
		case 'C': state=3;
		case 'D': state=4;
	}
	
	if ((c >= '0' && c <= '9') || (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || c == '+')
	{
		if (c >= 'a' && c <= 'z')
			c -= 'a' - 'A';
		buffer[pointer++] = c;
	}
	else
		pointer =0;
	if (pointer > 15)
		pointer = 0;
	buffer[15] = 0;
	if (strcmp((char*)buffer, "AT+CLASS") == 0)
	{
		UARTStringPut("CLASS F1703304");
		memset(buffer, 0, 16);
		pointer = 0;
	}
	else if (strcmp((char*)buffer, "AT+STUDENTCODE") == 0)
	{
		UARTStringPut("CODE 517021910521");
		memset(buffer, 0, 16);
		pointer = 0;
	}
}



//处理时钟的
/*
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
*/

bool processCommand(int8_t* str)
{
	uint8_t th, tm, ts;
	if (strlen(str) == 11 && str[0] == 'S' && str[1] == 'E' && str[2] == 'T')
	{
		th = (str[3] - '0') * 10 + (str[4] - '0');
		tm = (str[6] - '0') * 10 + (str[7] - '0');
		ts = (str[9] - '0') * 10 + (str[10] - '0');
		settime(th, tm, ts);
		pointer = 0;
		memset(buffer, 0, 16);
		return true;
	}
	else if (strlen(str) == 11 && str[0] == 'I' && str[1] == 'N' && str[2] == 'C')
	{
		th = (str[3] - '0') * 10 + (str[4] - '0');
		tm = (str[6] - '0') * 10 + (str[7] - '0');
		ts = (str[9] - '0') * 10 + (str[10] - '0');
		inctime(th, tm, ts);
		pointer = 0;
		memset(buffer, 0, 16);
		return true;
	}
	else if (strlen(str) == 7 && str[0] == 'G' && str[1] == 'E' && str[2] == 'T' && str[3] == 'T' && str[4] == 'I' && str[5] == 'M' && str[6] == 'E')
	{
		gettime();
		pointer = 0;
		memset(buffer, 0, 16);
		return true;
	}
	return false;
}

//功能函数
void settime(int8_t th, int8_t tm, int8_t ts)
{
	hour = th;
	minute = tm;
	second = ts;
	inctime(0, 0, 0);
}
void inctime(int8_t th, int8_t tm, int8_t ts)
{
	second += ts;
	minute += tm;
	hour += th;
	minute += second / 60;
	second %= 60;
	hour += minute / 60;
	minute %= 60;
	hour %= 24;
}
void gettime(void)
{
	uint8_t s[9];
	s[0] = hour / 10 + '0';
	s[1] = hour % 10 + '0';
	s[2] = '-';
	s[3] = minute / 10 + '0';
	s[4] = minute % 10 + '0';
	s[5] = '-';
	s[6] = second / 10 + '0';
	s[7] = second % 10 + '0';
	s[8] = '\0';
	UARTStringPut(s);
}
