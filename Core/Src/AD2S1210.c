//#include "AD2S1210.h"
//#include "./SYSTEM/sys/sys.h"
//#include "./SYSTEM/delay/delay.h"
//#include "./BSP/SPI/spi.h"

#include "AD2S1210.h"
#include "sys.h"
#include "delay.h"
#include "main.h"
// add the c file here


//---------------------------------
//void WriteToAD2S(unsigned char count,unsigned char *buf);
//---------------------------------
//Function that writes to the AD2S via the SPI Port. 
//--------------------------------------------------------------------------------
void SPIRead(unsigned char count, unsigned char *buf)
{
	unsigned	char	i = 0;
	unsigned	char	j = 0;
	unsigned	int  	iTemp = 0;
	unsigned	char  RotateData = 0;
	
	for(j=count; j>0; j--)
	{
		for(i=0; i<8; i++)
		{
		  SET_SCLK();					
			RotateData <<= 1;			//Rotate data
			delay_ms(1);
			iTemp = HAL_GPIO_ReadPin(SDO_GPIO_Port, SDO_Pin);
			CLR_SCLK();	
			if(iTemp)
			{
				RotateData |= 1;	
			}
			delay_ms(1);
		}
		*(buf + j - 1)= RotateData;	  	
	}
}	
 	 

void SPIWrite(unsigned char count, unsigned char *buf)
{
	unsigned	char	ValueToWrite = 0;
  unsigned	char	i = 0;
	unsigned	char	j = 0;
		
	for(i=count;i>0;i--)
 	{
	 	ValueToWrite = *(buf + i - 1);
		for(j=0; j<8; j++)
		{
			SET_SCLK();				
			if(0x80 == (ValueToWrite & 0x80))
			{
				SET_SDI();			//Send one to SDI pin
			}
			else
			{
				CLR_SDI();			//Send zero to SDI pin
			}
			delay_ms(1);
			CLR_SCLK();				
			delay_ms(1);
			ValueToWrite <<= 1;		//Rotate data
		}
	}
}



void AD2S1210GPIOInitiate()
{
	GPIO_InitTypeDef gpio_init_struct;
	
	DIR_GPIO_CLK_ENABLE();
	_RESET_GPIO_CLK_ENABLE();
	LOT_GPIO_CLK_ENABLE();
	DOS_GPIO_CLK_ENABLE();
	A1_GPIO_CLK_ENABLE();
	A0_GPIO_CLK_ENABLE();
	RES0_GPIO_CLK_ENABLE();
	RES1_GPIO_CLK_ENABLE();
	_RD_GPIO_CLK_ENABLE();
	_WR_GPIO_CLK_ENABLE();
	_SOE_GPIO_CLK_ENABLE();
	_SAMPLE_GPIO_CLK_ENABLE();
	_CS_GPIO_CLK_ENABLE();
	SCLK_GPIO_CLK_ENABLE();
	SDI_GPIO_CLK_ENABLE();
	SDO_GPIO_CLK_ENABLE();
	
	gpio_init_struct.Pin = DIR_Pin;
	gpio_init_struct.Mode = GPIO_MODE_OUTPUT_PP;
	gpio_init_struct.Pull = GPIO_PULLUP;
	gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(DIR_GPIO_Port, &gpio_init_struct);
	
	gpio_init_struct.Pin = _RESET_Pin;
	HAL_GPIO_Init(_RESET_GPIO_Port, &gpio_init_struct);
	
	gpio_init_struct.Pin = LOT_Pin;
	HAL_GPIO_Init(LOT_GPIO_Port, &gpio_init_struct);
	
	gpio_init_struct.Pin = DOS_Pin;
	HAL_GPIO_Init(DOS_GPIO_Port, &gpio_init_struct);
	
	gpio_init_struct.Pin = A1_Pin;
	HAL_GPIO_Init(A1_GPIO_Port, &gpio_init_struct);
	
	gpio_init_struct.Pin = A0_Pin;
	HAL_GPIO_Init(A0_GPIO_Port, &gpio_init_struct);
	
	gpio_init_struct.Pin = RES0_Pin;
	HAL_GPIO_Init(RES0_GPIO_Port, &gpio_init_struct);
	
	gpio_init_struct.Pin = RES1_Pin;
	HAL_GPIO_Init(RES1_GPIO_Port, &gpio_init_struct);
	
	gpio_init_struct.Pin = _RD_Pin;
	HAL_GPIO_Init(_RD_GPIO_Port, &gpio_init_struct);
	
	gpio_init_struct.Pin = _WR_Pin;
	HAL_GPIO_Init(_WR_GPIO_Port, &gpio_init_struct);
	
	gpio_init_struct.Pin = _SOE_Pin;
	HAL_GPIO_Init(_SOE_GPIO_Port, &gpio_init_struct);
	
	gpio_init_struct.Pin = _SAMPLE_Pin;
	HAL_GPIO_Init(_SAMPLE_GPIO_Port, &gpio_init_struct);
	
	gpio_init_struct.Pin = _CS_Pin;
	HAL_GPIO_Init(_CS_GPIO_Port, &gpio_init_struct);
	
	gpio_init_struct.Pin = SCLK_Pin;
	HAL_GPIO_Init(SCLK_GPIO_Port, &gpio_init_struct);
	
	gpio_init_struct.Pin = SDI_Pin;
	HAL_GPIO_Init(SDI_GPIO_Port, &gpio_init_struct);
	
	gpio_init_struct.Pin = SDO_Pin;
	gpio_init_struct.Mode = GPIO_MODE_INPUT;
	//gpio_init_struct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(SDO_GPIO_Port, &gpio_init_struct);
	
}

void AD2S1210Initiate()
{
//RESET->0 initially  
	CLR_RESET();  
	SET_SPL();
	delay_us(20);
	SET_RESET(); 
	delay_ms(10);
	CLR_SPL();
	delay_ms(1);
	SET_SPL();
}

void AD2S1210SelectMode(unsigned char mode)
{
	if (mode==POSITION)
	{
		CLR_A0();
		CLR_A1();
		delay_ms(1);		//Normal Mode position output
	}
	else if (mode==VELOCITY)
	{
		CLR_A0();
		SET_A1();
		delay_ms(1);		//Normal Mode velocity output
	}
	else if (mode==CONFIG)
	{
		SET_A0();
		SET_A1();
		delay_ms(1);		//Configuration Mode
	}
}

void WriteToAD2S1210(unsigned char address, unsigned char data)
{
	unsigned	char	buf;

	//write control register address
	buf = address;

	SET_SCLK();
	delay_ms(4);
	SET_CS();
	delay_ms(4);
	CLR_CS();
	delay_ms(4);
	
	SET_WR();
	delay_ms(4);
	CLR_WR();
	delay_ms(4);

	SPIWrite(1,&buf);	  	 
	
	SET_WR();
	delay_ms(4);	
	SET_CS();	
	//write control register address

	//write control register data
	buf = data;

	SET_SCLK();
	delay_ms(4);
	SET_CS();
	delay_ms(4);
	CLR_CS();
	delay_ms(4);
	
	SET_WR();
	delay_ms(4);
	CLR_WR();
	delay_ms(4);

	SPIWrite(1,&buf);	  	 

	SET_WR();
	delay_ms(4);	
	SET_CS();
	//write control register data
}

void ReadFromAD2S1210(unsigned char mode, unsigned char address, unsigned char * buf)
{
	if (mode==CONFIG)
	{
		
		//write control register address
		buf[0] = address;

		SET_SCLK();
		delay_ms(4);
		SET_CS();
		delay_ms(4);
		CLR_CS();
		delay_ms(4);
		
		SET_WR();
		delay_ms(4);
		CLR_WR();
		delay_ms(4);

		SPIWrite(1,buf);

		SET_WR();
		delay_ms(4);	
		SET_CS();
		//write control register address


		//read 1-byte register
		SET_SCLK();
			
		SET_CS();
		SET_WR();
		delay_ms(4);
	
		CLR_CS();
		delay_ms(4);
	
		CLR_SCLK();	
		delay_ms(1);
		
		CLR_WR();
		delay_ms(4);

		SPIRead(1,buf);	

		SET_WR();
		delay_ms(4);

		SET_CS();
		//read 1-byte register
	}
	else if (mode==POSITION||mode==VELOCITY)
	{
		SET_SPL();
		delay_ms(1);
		CLR_SPL();
		delay_ms(5);

		//read 3-byte register 
		SET_SCLK();
				
		SET_CS();
		SET_WR();
		delay_ms(4);
		
		CLR_CS();
		delay_ms(4);
	
		CLR_SCLK();	
		delay_ms(4);
			
		CLR_WR();
		delay_ms(4);
	
		SPIRead(3,buf);		//read data register
	
		SET_WR();
		delay_ms(4);
	
		SET_CS();
		//read 3-byte register


	}
}

void AD2S1210SoftReset(void)
{
	unsigned char buf=	SOFTRESET;
	SPIWrite (1,&buf);	  	//soft reset 
	delay_ms(10);
}