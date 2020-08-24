#include "IIC.h"

#define IIC_PORT_CLK_ENABLE() __HAL_RCC_GPIOA_CLK_ENABLE()
#define IIC_PORT GPIOA
#define SCL GPIO_PIN_10
#define SDA GPIO_PIN_11
//注：SCL、SDA应该属于同一组GPIO

void IIC_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	//使能外设 IO PORTC 时钟
	IIC_PORT_CLK_ENABLE();
	GPIO_InitStructure.Pin = SCL|SDA;
	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP ; //推挽输出
	GPIO_InitStructure.Pull=GPIO_PULLUP; 
	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(IIC_PORT, &GPIO_InitStructure);
	HAL_GPIO_WritePin(IIC_PORT,SCL,GPIO_PIN_SET);
	HAL_GPIO_WritePin(IIC_PORT,SDA,GPIO_PIN_SET);
	
}

void SDA_OUT()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	//使能外设 IO PORTC 时钟
	GPIO_InitStructure.Pin = SDA;
	GPIO_InitStructure.Pull=GPIO_PULLUP; 
	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP ; //推挽输出
	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(IIC_PORT, &GPIO_InitStructure);
}

void SDA_IN()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	//使能外设 IO PORT 时钟
	GPIO_InitStructure.Pull=GPIO_PULLUP; 
	GPIO_InitStructure.Pin = SDA;
	GPIO_InitStructure.Mode = GPIO_MODE_INPUT ; //上拉输入
	HAL_GPIO_Init(IIC_PORT, &GPIO_InitStructure);
}
//产生 IIC 起始信号

void IIC_Start(void)
{
	SDA_OUT(); //sda 线输出
	GPIO_SetBits(IIC_PORT,SDA);
	delay_us(2);
	GPIO_SetBits(IIC_PORT,SCL);
	delay_us(3);
	GPIO_ResetBits(IIC_PORT,SDA);//START:when CLK is high,DATA change form high to low
	delay_us(3);
	GPIO_ResetBits(IIC_PORT,SCL);//钳住 I2C 总线，准备发送或接收数据
}

//产生 IIC 停止信号
void IIC_Stop(void)
{
	SDA_OUT();//sda 线输出
	GPIO_ResetBits(IIC_PORT,SCL);
	GPIO_ResetBits(IIC_PORT,SDA);//STOP:when CLK is high DATA change form low to high
	delay_us(3);
	GPIO_SetBits(IIC_PORT,SCL);
	GPIO_SetBits(IIC_PORT,SDA);//发送 I2C 总线结束信号
	delay_us(3);
}

//等待应答信号到来
//返回值：0，接收应答失败
// 1，接收应答成功
u8 IIC_Wait_Ack(void)
{
	u16 ucErrTime=0;
	SDA_IN(); //SDA 设置为输入
	GPIO_SetBits(IIC_PORT,SDA);delay_us(1);
	GPIO_SetBits(IIC_PORT,SCL);delay_us(1);
	while(GPIO_ReadInputDataBit(IIC_PORT,SDA))
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			IIC_Stop();
			return 0;
		}
	}
	GPIO_ResetBits(IIC_PORT,SCL);//时钟输出 0
	return 1;
}

//产生 ACK 应答
void IIC_Ack(void)
{
	GPIO_ResetBits(IIC_PORT,SCL);
	SDA_OUT();
	GPIO_ResetBits(IIC_PORT,SDA);
	delay_us(2);
	GPIO_SetBits(IIC_PORT,SCL);
	delay_us(2);
	GPIO_ResetBits(IIC_PORT,SCL);
}
//不产生 ACK 应答
void IIC_NAck(void)
{
	GPIO_ResetBits(IIC_PORT,SCL);
	SDA_OUT();
	GPIO_SetBits(IIC_PORT,SDA);
	delay_us(2);
	GPIO_SetBits(IIC_PORT,SCL);
	delay_us(2);
	GPIO_ResetBits(IIC_PORT,SCL);
}
//IIC 发送一个字节
//返回从机有无应答
//1，有应答
//0，无应答
void IIC_Send_Byte(u8 txd)
{
	u8 t;
	SDA_OUT();
	GPIO_ResetBits(IIC_PORT,SCL);//拉低时钟开始数据传输
	for(t=0;t<8;t++)
	{
		switch((txd&0x80)>>7)
		{
			case 1: GPIO_SetBits(IIC_PORT,SDA);break;
			case 0:GPIO_ResetBits(IIC_PORT,SDA);break;
		}
		txd<<=1;
		delay_us(2); //对 TEA5767 这三个延时都是必须的
		GPIO_SetBits(IIC_PORT,SCL);
		delay_us(2);
		GPIO_ResetBits(IIC_PORT,SCL);
		delay_us(2);
	}
}

//读 1 个字节，ack=1 时，发送 ACK，ack=0，发送 nACK
u8 IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_IN();//SDA 设置为输入
	for(i=0;i<8;i++ )
	{
		GPIO_ResetBits(IIC_PORT,SCL);
		delay_us(2);
		GPIO_SetBits(IIC_PORT,SCL);
		receive<<=1;
		if(GPIO_ReadInputDataBit(IIC_PORT,SDA))receive++;
		delay_us(1);
		
	}
	if (!ack)
	IIC_NAck();//发送 nACK
	else
	IIC_Ack(); //发送 ACK
	return receive;
}
