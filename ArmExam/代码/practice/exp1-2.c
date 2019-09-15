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

uint32_t delay_time,key_value;
int i=0;

volatile uint8_t posedge=0;	
volatile uint8_t result;

void 		S800_GPIO_Init(void);
uint8_t 	I2C0_WriteByte(uint8_t DevAddr, uint8_t RegAddr, uint8_t WriteData);
uint8_t 	I2C0_ReadByte(uint8_t DevAddr, uint8_t RegAddr);
void		S800_I2C0_Init(void);
uint32_t state = 0, ui32SysClock;
uint8_t seg7[] = {0x3f,0x06,0x5b,0x4f,0x66,0x6d,0x7d,0x07,0x7f,0x6f,0x77,0x7c,0x58,0x5e,0x079,0x71,0x5c};
void Delay(uint32_t value);
void count(uint32_t *state);


//3.loop shift left and right******************************
uint8_t crol(uint8_t a,uint8_t b) //left
{
    uint8_t left = a<<b;
    uint8_t right=a>>( 8-b);
    uint8_t temp=left|right;
    return temp;
}
uint8_t cror(uint8_t a,uint8_t b) //right
{
    uint8_t right = a>>b;
    uint8_t left  = a<<( 8-b);
    uint8_t temp=left|right;
    return temp;
}
//*******************************************************
//4.let the number at position b of a be 1
uint8_t setbit(uint8_t a,uint8_t b){
	uint8_t one = 0x01;
	one = crol(one,b);
	a = a | one;
	return a;
}
//**********************************************************
void clean()
{
	uint8_t light=0x00;
	result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[i+1]);						//write port 1 				
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(light));					//write port 2
		
		result = I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_OUTPUT,~light);	
}
void oddFlowLight()
{
	uint8_t light=0x01;
	for(i=0;i<8;i=i+2){
	result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[i+1]);						//write port 1 				
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(light));					//write port 2
		
		result = I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_OUTPUT,~light);	
    
		light=crol(light,2);
	   
		Delay(1000000);
	}
}

void evenFlowLight()
{
	uint8_t light=0x02;
	for(i=1;i<8;i=i+2){
	result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[i+1]);						//write port 1 				
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(light));					//write port 2
		
		result = I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_OUTPUT,~light);	
    
		light=crol(light,2);
	   
		Delay(1000000);
	}
}

void leftToright ()
{
	
	uint8_t light=0x00;
	i=0;
	while(state<6){
		if(GPIOPinRead(GPIO_PORTJ_BASE,GPIO_PIN_1)==0)state=state+1;
		if(i==0){
			light=0x00;
			result = I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_OUTPUT,0xFF);	
		}
		else{
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(0x00));					//write port 2
		
		result = I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_OUTPUT,~light);
		}			
		if(state!=4){light=setbit(light,i);
		i=(i+1)%9;}
		Delay(1000000);
	}
	if(state==6)state=1;
}

void stopLight()
{
	
}

void count(uint32_t *state)
{
	//posedge 
			if(GPIOPinRead(GPIO_PORTJ_BASE,GPIO_PIN_1)==0) {
				while(GPIOPinRead(GPIO_PORTJ_BASE,GPIO_PIN_1)==0)Delay(1000);
				posedge=1;
			}
			if(posedge)
	{
		   *state = (*state + 1) % 4;
	}
   posedge=0;
	//
}

void display(uint32_t state)
{
	
	switch (state)
	{
		case 0: clean();
						Delay(1000);
						break;
		case 1: oddFlowLight();
		        Delay(1000);
						break;
		case 2: evenFlowLight();
		        break;
		case 3: leftToright();
						Delay(1000);
		        break;
	}
}


int main(void)
{
	
	ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_16MHZ |SYSCTL_OSC_INT |SYSCTL_USE_OSC), 16000000);		
	
	S800_GPIO_Init();
	S800_I2C0_Init();
	
	
	delay_time = SLOWFLASHTIME;
	while(1)
  {
		
		count(&state);
		
		display(state);
		
		
  }
}

void Delay(uint32_t value)
{
	uint32_t ui32Loop;
	for(ui32Loop = 0; ui32Loop < value; ui32Loop++){};
}

 
void S800_GPIO_Init(void)
{
	//SysCtlClockFreqSet((SYSCTL_XTAL_16MHZ |SYSCTL_OSC_INT |SYSCTL_USE_OSC), 16000000);
	SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |SYSCTL_OSC_MAIN |SYSCTL_USE_OSC), 25000000);
	//SysCtlClockFreqSet((SYSCTL_OSC_INT | SYSCTL_CFG_VCO_320 | SYSCTL_USE_PLL), 40000000);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);						//Enable PortF
	//函数原型：void SysCtlPeripheralEnable(uint32_t ui32Peripheral)
	
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF));			//Wait for the GPIO moduleF ready
	//函数原型：bool SysCtlPeripheralReady(uint32_t ui32Peripheral)
	//如果指定的外设被使能成功，返回true，否则返回false
	
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOJ);						//Enable PortJ	
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOJ));			//Wait for the GPIO moduleJ ready	
	
  GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_0);			//Set PF0 as Output pin
	GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1);	
	//函数原型：void GPIOPinTypeGPIOOutput(uint32_t ui32Port, uint8_t ui8Pins)
	//配置GPIO端口引脚为输出引脚，如果字符型（uint8_t）参数ui8Pins某位为1，则GPIO端口对应位配置为输出引脚
	
	GPIOPinTypeGPIOInput(GPIO_PORTJ_BASE,GPIO_PIN_0 | GPIO_PIN_1);//Set the PJ0,PJ1 as input pin
	//配置GPIO端口引脚为输入引脚，与GPIOPinTypeGPIOOutput()类似。GPIO_PIN_0 | GPIO_PIN_1 = 00000011b
	
	GPIOPadConfigSet(GPIO_PORTJ_BASE,GPIO_PIN_0 | GPIO_PIN_1,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU);
	//函数原型：void GPIOPadConfigSet(uint32_t ui32Port, uint8_t ui8Pins, uint32_t ui32Strength, uint32_t ui32PinType)
	//GPIO端口配置。uint32_t ui32Port：GPIO端口基地址
	//ui8Pins：端口引脚位组合表示，如10000001b表示配置端口的D7、D0位
	//ui32Strength：端口的输出驱动能力，对输入设置无效，可选项包括GPIO_STRENGTH_2MA/4MA/8MA/8MA_SC/6MA/10MA/12MA
	//ui32PinType：引脚类型，可选项包括GPIO_PIN_TYPE_STD（推挽）、GPIO_PIN_TYPE_STD_WPU（推挽上拉）、GPIO_PIN_TYPE_STD_WPD（推挽下拉）、
	//GPIO_PIN_TYPE_OD（开漏）、GPIO_PIN_TYPE_ANALOG（模拟）、GPIO_PIN_TYPE_WAKE_HIGH（高电平从冬眠唤醒）、GPIO_PIN_TYPE_WAKE_LOW（低）
}



void S800_I2C0_Init(void)
{
	uint8_t result;
  SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);//初始化i2c模块
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);//使用I2C模块0，引脚配置为I2C0SCL--PB2、I2C0SDA--PB3
	GPIOPinConfigure(GPIO_PB2_I2C0SCL);//配置PB2为I2C0SCL
  GPIOPinConfigure(GPIO_PB3_I2C0SDA);//配置PB3为I2C0SDA
  GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);//I2C将GPIO_PIN_2用作SCL
  GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);//I2C将GPIO_PIN_3用作SDA

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
	while(I2CMasterBusy(I2C0_BASE)){};//如果I2C0模块忙，等待
		//
	I2CMasterSlaveAddrSet(I2C0_BASE, DevAddr, false);
		//设置主机要放到总线上的从机地址。false表示主机写从机，true表示主机读从机
		
	I2CMasterDataPut(I2C0_BASE, RegAddr);//主机写设备寄存器地址
	I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);//执行重复写入操作
	while(I2CMasterBusy(I2C0_BASE)){};
		
	rop = (uint8_t)I2CMasterErr(I2C0_BASE);//调试用

	I2CMasterDataPut(I2C0_BASE, WriteData);
	I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);//执行重复写入操作并结束
	while(I2CMasterBusy(I2C0_BASE)){};

	rop = (uint8_t)I2CMasterErr(I2C0_BASE);//调试用

	return rop;//返回错误类型，无错返回0
}

uint8_t I2C0_ReadByte(uint8_t DevAddr, uint8_t RegAddr)
{
	uint8_t value,rop;
	while(I2CMasterBusy(I2C0_BASE)){};	
	I2CMasterSlaveAddrSet(I2C0_BASE, DevAddr, false);
	I2CMasterDataPut(I2C0_BASE, RegAddr);
//	I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);		
	I2CMasterControl(I2C0_BASE,I2C_MASTER_CMD_SINGLE_SEND);//执行单词写入操作
	while(I2CMasterBusBusy(I2C0_BASE));
	rop = (uint8_t)I2CMasterErr(I2C0_BASE);
	Delay(1);
	//receive data
	I2CMasterSlaveAddrSet(I2C0_BASE, DevAddr, true);//设置从机地址
	I2CMasterControl(I2C0_BASE,I2C_MASTER_CMD_SINGLE_RECEIVE);//执行单次读操作
	while(I2CMasterBusBusy(I2C0_BASE));
	value=I2CMasterDataGet(I2C0_BASE);//获取读取的数据
		Delay(1);
	return value;
}
