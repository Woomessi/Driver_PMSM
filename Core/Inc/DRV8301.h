#ifndef DRV8301_H
#define DRV8301_H

//#include "./SYSTEM/sys/sys.h"
#include "sys.h"

// add the header file here


#define NOCTW_GPIO_CLK_ENABLE()     do{ __HAL_RCC_GPIOD_CLK_ENABLE(); }while(0)


#define NFAULT_GPIO_CLK_ENABLE()     do{ __HAL_RCC_GPIOD_CLK_ENABLE(); }while(0)


#define NSCS_DRV8301_GPIO_CLK_ENABLE()     do{ __HAL_RCC_GPIOB_CLK_ENABLE(); }while(0)


#define SCLK_DRV8301_GPIO_CLK_ENABLE()     do{ __HAL_RCC_GPIOD_CLK_ENABLE(); }while(0)


#define SDO_DRV8301_GPIO_CLK_ENABLE()     do{ __HAL_RCC_GPIOD_CLK_ENABLE(); }while(0)


#define SDI_DRV8301_GPIO_CLK_ENABLE()     do{ __HAL_RCC_GPIOD_CLK_ENABLE(); }while(0)


#define DC_CAL_GPIO_CLK_ENABLE()     do{ __HAL_RCC_GPIOD_CLK_ENABLE(); }while(0)


#define EN_GATE_GPIO_CLK_ENABLE()     do{ __HAL_RCC_GPIOD_CLK_ENABLE(); }while(0) 

#define CURRENT1_7A 0x0000
#define CURRENT0_7A 0x0001
#define CURRENT0_25A 0x0002

#define GATE_RST_NORMAL 0x0000
#define GATE_RST_LATCH 0x0004

#define PHASE6PWM 0x0000
#define PHASE3PWM 0x0008

#define CURRENTLIMIT 0x0000
#define OCLATCHSTDOWN 0x0010
#define REPORTONLY 0x0020
#define OCDISABLED 0x0030

#define OCOTBOTH 0x0000
#define OTONLY 0x0001
#define OCONLY 0x0002
#define OCONLYRSVD 0x0003

#define GAIN10 0x0000
#define GAIN20 0x0004
#define GAIN40 0x0008
#define GAIN80 0x000C

#define CONNECTLOADPH1 0x0000
#define SHORTINPUTPH1 0x0010

#define CONNECTLOADPH2 0x0000
#define SHORTINPUTPH2 0x0020

#define CYCLEBYCYCLE 0x0000
#define OFFTIMECTRL 0x0040

#define DRV8301WRITE (0x00 << 15)
#define DRV8301READ (0x01 << 15)

#define REGSTATUS1 0x00
#define REGSTATUS2 0x01

#define REGCTRL1 0x02
#define REGCTRL2 0x03

typedef struct DRV8301CFG{
    uint16_t GATE_CURRENT;
    uint16_t GATE_RESET;
    uint16_t PWM_MODE;
    uint16_t OCP_MODE;
    uint16_t OC_ADJ_SET;
    uint16_t OCTW_MODE;
    uint16_t GAINVALUE;
    uint16_t DC_CAL_CH1;
    uint16_t DC_CAL_CH2;
    uint16_t OC_TOFF;
}DRV8301Cfg;




void enablegate(void);
uint16_t DRV8301IDread(void);
void DRV8301Init(DRV8301Cfg cfg);

void spi3init(void);
void chipselect(uint8_t status);
void spi3sck(uint8_t clk);
void spi3writeword(uint16_t data);
uint16_t spi3readword(void);

#endif