//initialize ͨ�ó�ʼ��*******************88
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
void		PF0_Flash(uint32_t key_value,uint32_t key_value2);

int main(void)
{

	
	
	uint32_t read_key_value2;
	uint32_t read_key_value;
	S800_GPIO_Init();
	S800_I2C0_Init();
	while(1)
  {
		read_key_value = GPIOPinRead(GPIO_PORTJ_BASE,GPIO_PIN_0)	;				//read the PJ0 key value
		
		read_key_value2 = GPIOPinRead(GPIO_PORTJ_BASE,GPIO_PIN_1)	;

		PF0_Flash(read_key_value,read_key_value2);

   }
}

void PF0_Flash(uint32_t key_value,uint32_t key_value2)
{
  	uint32_t delay_time;
		//if (key_value	== 0)						//USR_SW1-PJ0 pressed
		//	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0x3);			// Turn on the LED.
		//else													//USR_SW1-PJ0 released
		//	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0x0);	
		
		
		if (key_value2	== 0)						//USR_SW1-PJ0 pressed
			delay_time							= FASTFLASHTIME;
		else													//USR_SW1-PJ0 released
			delay_time							= SLOWFLASHTIME;
		
		GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0, GPIO_PIN_0);			// Turn on the LED.
		Delay(delay_time);
		GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0, 0x0);							// Turn off the LED.
		Delay(delay_time);
		
	
}

void Delay(uint32_t value)
{
	uint32_t ui32Loop;
	for(ui32Loop = 0; ui32Loop < value; ui32Loop++){
	if(GPIOPinRead(GPIO_PORTJ_BASE,GPIO_PIN_0))GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0x0);
	else GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0x3);
	
  };
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
	I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,0);
}
void S800_GPIO_Init(void)
{
	//SysCtlClockFreqSet((SYSCTL_XTAL_16MHZ |SYSCTL_OSC_INT |SYSCTL_USE_OSC), 16000000);
	SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |SYSCTL_OSC_MAIN |SYSCTL_USE_OSC), 25000000);
	//SysCtlClockFreqSet((SYSCTL_OSC_INT | SYSCTL_CFG_VCO_320 | SYSCTL_USE_PLL), 40000000);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);						//Enable PortF
	//����ԭ�ͣ�void SysCtlPeripheralEnable(uint32_t ui32Peripheral)
	
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF));			//Wait for the GPIO moduleF ready
	//����ԭ�ͣ�bool SysCtlPeripheralReady(uint32_t ui32Peripheral)
	//���ָ�������豻ʹ�ܳɹ�������true�����򷵻�false
	
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOJ);						//Enable PortJ	
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOJ));			//Wait for the GPIO moduleJ ready	
	
  GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_0);			//Set PF0 as Output pin
	GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1);	
	//����ԭ�ͣ�void GPIOPinTypeGPIOOutput(uint32_t ui32Port, uint8_t ui8Pins)
	//����GPIO�˿�����Ϊ������ţ�����ַ��ͣ�uint8_t������ui8PinsĳλΪ1����GPIO�˿ڶ�Ӧλ����Ϊ�������
	
	GPIOPinTypeGPIOInput(GPIO_PORTJ_BASE,GPIO_PIN_0 | GPIO_PIN_1);//Set the PJ0,PJ1 as input pin
	//����GPIO�˿�����Ϊ�������ţ���GPIOPinTypeGPIOOutput()���ơ�GPIO_PIN_0 | GPIO_PIN_1 = 00000011b
	
	GPIOPadConfigSet(GPIO_PORTJ_BASE,GPIO_PIN_0 | GPIO_PIN_1,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU);
	//����ԭ�ͣ�void GPIOPadConfigSet(uint32_t ui32Port, uint8_t ui8Pins, uint32_t ui32Strength, uint32_t ui32PinType)
	//GPIO�˿����á�uint32_t ui32Port��GPIO�˿ڻ���ַ
	//ui8Pins���˿�����λ��ϱ�ʾ����10000001b��ʾ���ö˿ڵ�D7��D0λ
	//ui32Strength���˿ڵ��������������������������Ч����ѡ�����GPIO_STRENGTH_2MA/4MA/8MA/8MA_SC/6MA/10MA/12MA
	//ui32PinType���������ͣ���ѡ�����GPIO_PIN_TYPE_STD�����죩��GPIO_PIN_TYPE_STD_WPU��������������GPIO_PIN_TYPE_STD_WPD��������������
	//GPIO_PIN_TYPE_OD����©����GPIO_PIN_TYPE_ANALOG��ģ�⣩��GPIO_PIN_TYPE_WAKE_HIGH���ߵ�ƽ�Ӷ��߻��ѣ���GPIO_PIN_TYPE_WAKE_LOW���ͣ�
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

