
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
#define TCA6424_OUTPUT_PORT2			0x06     //��ַ ���ù�




void 		Delay(uint32_t value);
void 		S800_GPIO_Init(void);
uint8_t 	I2C0_WriteByte(uint8_t DevAddr, uint8_t RegAddr, uint8_t WriteData);  //I2C��д
uint8_t 	I2C0_ReadByte(uint8_t DevAddr, uint8_t RegAddr);                      //I2C�ڶ�
void		S800_I2C0_Init(void);
volatile uint8_t result;


uint32_t ui32SysClock;     //ʱ��Ƶ��             
uint8_t seg7[] = {0x3f,0x06,0x5b,0x4f,0x66,0x6d,0x7d,0x07,0x7f,0x6f,0x77,0x7c,0x58,0x5e,0x079,0x71,0x5c};  //�߶ι���ʾ0~F

int num=1;//��������
uint8_t state=0x01;//ʹ��λ
uint8_t statenext=0x01;


int main(void)
{ 
	 //use internal 16M oscillator, PIOSC
   //ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_16MHZ |SYSCTL_OSC_INT |SYSCTL_USE_OSC), 16000000);		
	 //ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_16MHZ |SYSCTL_OSC_INT |SYSCTL_USE_OSC), 8000000);		
	 //use external 25M oscillator, MOSC
   //ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |SYSCTL_OSC_MAIN |SYSCTL_USE_OSC), 25000000);		
	 //use external 25M oscillator and PLL to 120M
   //ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |SYSCTL_OSC_MAIN | SYSCTL_USE_PLL |SYSCTL_CFG_VCO_480), 120000000);	
	 ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_16MHZ |SYSCTL_OSC_INT | SYSCTL_USE_PLL |SYSCTL_CFG_VCO_480), 20000000);	
	 //I2C0��ʼ��Ҫ��ʱ��
	
	S800_GPIO_Init();
	S800_I2C0_Init();

	
	
	while (1){
	  int i;
		for (i=0;i < 50;i++){		
	  result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[(num-1)%8+1]);    
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,state);		
    Delay(30000);
		
		statenext=state<<1;	
		if(state==0x80) {statenext=0x01;}
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[num%8+1]);    
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,statenext);		
		Delay(30000);
	}
	  num++;
    state=state<<1;
    if (num==9)	{num=1;state=0x01;}	
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
	
  GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);			//Set PF0123 as Output pin
	GPIOPinTypeGPIOInput(GPIO_PORTJ_BASE,GPIO_PIN_0 | GPIO_PIN_1);                            //Set the PJ0,PJ1 as input pin
	GPIOPadConfigSet(GPIO_PORTJ_BASE,GPIO_PIN_0 | GPIO_PIN_1,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU);
}


void S800_I2C0_Init(void)
{
	uint8_t result;
  SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);//��ʼ��i2cģ��
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);//ʹ��I2Cģ��0����������ΪI2C0SCL--PB2��I2C0SDA--PB3
	GPIOPinConfigure(GPIO_PB2_I2C0SCL);//����PB2ΪI2C0SCL
  GPIOPinConfigure(GPIO_PB3_I2C0SDA);//����PB3ΪI2C0SDA
  GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);//I2C��GPIO_PIN_2����SCL
  GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);//I2C��GPIO_PIN_3����SDA

	I2CMasterInitExpClk(I2C0_BASE,ui32SysClock, true);										//config I2C0 400k
	I2CMasterEnable(I2C0_BASE);	

	
	//PCA9557 I2C��չ8λGPIO��led��    TCA6424 I2C��չ24λGPIO��0�ڰ�����1���߶�����2��ʱ�� ����Ч
	result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_CONFIG_PORT0,0x0ff);		//config port 0 as input     ����1Ϊ�� 0Ϊ��
	result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_CONFIG_PORT1,0x0);			//config port 1 as output
	result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_CONFIG_PORT2,0x0);			//config port 2 as output 

	result = I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_CONFIG,0x00);					//config port as output
	result = I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_OUTPUT,0x0ff);				//turn off the LED1-8
	
}


uint8_t I2C0_WriteByte(uint8_t DevAddr, uint8_t RegAddr, uint8_t WriteData)
{
	uint8_t rop;
	while(I2CMasterBusy(I2C0_BASE)){};//���I2C0ģ��æ���ȴ�
		//
	I2CMasterSlaveAddrSet(I2C0_BASE, DevAddr, false);
		//��������Ҫ�ŵ������ϵĴӻ���ַ��false��ʾ����д�ӻ���true��ʾ�������ӻ�
		
	I2CMasterDataPut(I2C0_BASE, RegAddr);//����д�豸�Ĵ�����ַ
	I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);//ִ���ظ�д�����
	while(I2CMasterBusy(I2C0_BASE)){};
		
	rop = (uint8_t)I2CMasterErr(I2C0_BASE);//������

	I2CMasterDataPut(I2C0_BASE, WriteData);
	I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);//ִ���ظ�д�����������
	while(I2CMasterBusy(I2C0_BASE)){};

	rop = (uint8_t)I2CMasterErr(I2C0_BASE);//������

	return rop;//���ش������ͣ��޴���0
}

uint8_t I2C0_ReadByte(uint8_t DevAddr, uint8_t RegAddr)
{
	uint8_t value,rop;
	while(I2CMasterBusy(I2C0_BASE)){};	
	I2CMasterSlaveAddrSet(I2C0_BASE, DevAddr, false);
	I2CMasterDataPut(I2C0_BASE, RegAddr);
//	I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);		
	I2CMasterControl(I2C0_BASE,I2C_MASTER_CMD_SINGLE_SEND);//ִ�е���д�����
	while(I2CMasterBusBusy(I2C0_BASE));
	rop = (uint8_t)I2CMasterErr(I2C0_BASE);
	Delay(1);
	//receive data
	I2CMasterSlaveAddrSet(I2C0_BASE, DevAddr, true);//���ôӻ���ַ
	I2CMasterControl(I2C0_BASE,I2C_MASTER_CMD_SINGLE_RECEIVE);//ִ�е��ζ�����
	while(I2CMasterBusBusy(I2C0_BASE));
	value=I2CMasterDataGet(I2C0_BASE);//��ȡ��ȡ������
		Delay(1);
	return value; //���÷���
}

