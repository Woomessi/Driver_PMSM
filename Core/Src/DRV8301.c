//#include "./BSP/DRV8301_HAL/DRV8301.h"
//#include "./SYSTEM/sys/sys.h"
//#include "./CMSIS/Include/core_cm4.h"
//#include "./SYSTEM/delay/delay.h"
//#include "./SYSTEM/usart/usart.h"

#include "DRV8301.h"
#include "sys.h"
#include "core_cm4.h"
#include "delay.h"
#include "main.h"
//#include "usart.h"

void spi3init(void){
    //使能SPI时钟
    NSCS_DRV8301_GPIO_CLK_ENABLE();
    SCLK_DRV8301_GPIO_CLK_ENABLE();
    SDO_DRV8301_GPIO_CLK_ENABLE();
    SDI_DRV8301_GPIO_CLK_ENABLE();

    //配置GPIO
    GPIO_InitTypeDef gpio_init_struct;

    gpio_init_struct.Pin = SDO_DRV8301_Pin;
    gpio_init_struct.Mode = GPIO_MODE_INPUT;
    gpio_init_struct.Pull = GPIO_NOPULL;
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(SDO_DRV8301_GPIO_Port, &gpio_init_struct);

    gpio_init_struct.Pin = NSCS_DRV8301_Pin;
    gpio_init_struct.Mode = GPIO_MODE_OUTPUT_PP;
    gpio_init_struct.Pull = GPIO_NOPULL;
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(NSCS_DRV8301_GPIO_Port, &gpio_init_struct);

    gpio_init_struct.Pin = SCLK_DRV8301_Pin;
    HAL_GPIO_Init(SCLK_DRV8301_GPIO_Port, &gpio_init_struct);

    gpio_init_struct.Pin = SDI_DRV8301_Pin;
    HAL_GPIO_Init(SDI_DRV8301_GPIO_Port, &gpio_init_struct);

    
    HAL_GPIO_WritePin(SCLK_DRV8301_GPIO_Port, SCLK_DRV8301_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(SDI_DRV8301_GPIO_Port, SDI_DRV8301_Pin, GPIO_PIN_RESET);

    HAL_GPIO_WritePin(NSCS_DRV8301_GPIO_Port, NSCS_DRV8301_Pin, GPIO_PIN_SET);

    //
    // RCC->AHB1ENR |= 0x01 << 2 | 0x01 << 3;

    // GPIOC->MODER &= ~(0x03 << 20 | 0x03 << 22 | 0x03 << 24);
    // GPIOD->MODER &= ~(0x03 << 4);

    // GPIOC->MODER |= 0x01 << 20 | 0x01 << 24;
    // GPIOD->MODER |= 0x01 << 4;

    // GPIOC->OTYPER &= ~(0x01 << 10 | 0x01 << 12);
    // GPIOD->OTYPER &= ~(0x01 << 2);

    // GPIOC->OSPEEDR &= ~(0x03 << 20 | 0x03 << 24);
    // GPIOD->OSPEEDR &= ~(0x03 << 4);
    
    // GPIOC->OSPEEDR |= 0x03 << 20 | 0x03 << 24;
    // GPIOD->OSPEEDR |= 0x03 << 4;

    // GPIOC->PUPDR &= ~(0x03 << 20 | 0x03 << 22 | 0x03 << 24);
    // GPIOD->PUPDR &= ~(0x03 << 4);

    // GPIOC->BSRRH |= 0x01 << 10 | 0x01 << 12;

    // GPIOD->BSRRL |= 0x01 << 2;
}

void chipselect(uint8_t status)
{
    if (status)
    {
        HAL_GPIO_WritePin(NSCS_DRV8301_GPIO_Port, NSCS_DRV8301_Pin, GPIO_PIN_SET);
    }
    else
    {
        HAL_GPIO_WritePin(NSCS_DRV8301_GPIO_Port, NSCS_DRV8301_Pin, GPIO_PIN_RESET);
    }

    // if (status)
    // {
    //     GPIOD->BSRRL |= 0x01 << 2;
    // }
    // else
    // {
    //     GPIOD->BSRRH |= 0x01 << 2;
    // }
}

void spi3sck(uint8_t clk)
{
    if (clk)
    {
        HAL_GPIO_WritePin(SCLK_DRV8301_GPIO_Port, SCLK_DRV8301_Pin, GPIO_PIN_SET);
    }
    else
    {
        HAL_GPIO_WritePin(SCLK_DRV8301_GPIO_Port, SCLK_DRV8301_Pin, GPIO_PIN_RESET);
    }

    // if (clk)
    // {
    //     GPIOC->BSRRL |= 0x01 << 10;
    // }
    // else
    // {
    //     GPIOC->BSRRH |= 0x01 << 10;
    // }
}

void spi3writeword(uint16_t data)
{
    uint8_t i;
    chipselect(0);
    for (i = 0; i < 16; i++)
    {
        spi3sck(1);
        if (data & 0x8000 >> i)
        {
            HAL_GPIO_WritePin(SDI_DRV8301_GPIO_Port, SDI_DRV8301_Pin, GPIO_PIN_SET);
        }
        else
        {
            HAL_GPIO_WritePin(SDI_DRV8301_GPIO_Port, SDI_DRV8301_Pin, GPIO_PIN_RESET);
        }
        spi3sck(0);
    }
    HAL_GPIO_WritePin(SDI_DRV8301_GPIO_Port, SDI_DRV8301_Pin, GPIO_PIN_RESET);
    chipselect(1);

    // uint8_t i;
    // chipselect(0);
    // for (i = 0; i < 16; i++)
    // {
    //     spi3sck(1);
    //     if (data & 0x8000 >> i)
    //     {
    //         GPIOC->BSRRL |= 0x01 << 12;
    //     }
    //     else
    //     {
    //         GPIOC->BSRRH |= 0x01 << 12;
    //     }
    //     spi3sck(0);
    // }
    // GPIOC->BSRRH |= 0x01 << 12;
    // chipselect(1);
}

uint16_t spi3readword(void){
    uint8_t i;
    uint16_t data = 0;
    chipselect(0);
    for(i = 0; i < 16; i++){
        spi3sck(1);
        spi3sck(0);
        if(HAL_GPIO_ReadPin(SDO_DRV8301_GPIO_Port, SDO_DRV8301_Pin)){
            data |= 0x8000 >> i;
        }
    }
    chipselect(1);
    return data;

    // uint8_t i;
    // uint16_t data = 0;
    // chipselect(0);
    // for(i = 0; i < 16; i++){
    //     spi3sck(1);
    //     spi3sck(0);
    //     if(GPIOC->IDR & 0x01 << 11){
    //         data |= 0x8000 >> i;
    //     }
    // }
    // chipselect(1);
    // return data;
}

void enablegate(void){
    EN_GATE_GPIO_CLK_ENABLE();
    GPIO_InitTypeDef gpio_init_struct;
    gpio_init_struct.Pin = EN_GATE_Pin;
    gpio_init_struct.Mode = GPIO_MODE_OUTPUT_PP;
    gpio_init_struct.Pull = GPIO_NOPULL;
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(EN_GATE_GPIO_Port, &gpio_init_struct);

    HAL_GPIO_WritePin(EN_GATE_GPIO_Port, EN_GATE_Pin, GPIO_PIN_SET);//高电平


    // RCC->AHB1ENR |= 0x01 << 6;
    // GPIOG->MODER &= ~(0x03 << 12);
    // GPIOG->MODER |= 0x01 << 12;
    // GPIOG->OTYPER &= ~(0x01 << 6);
    // GPIOG->OSPEEDR &= ~(0x03 << 12);
    // GPIOG->PUPDR &= ~(0x03 << 12);
    // GPIOG->BSRRL |= 0x01 << 6;
}

void DRV8301Init(DRV8301Cfg cfg){
    enablegate();
    //timer6delayms(500);
    delay_ms(500);//需要延时500毫秒

    uint16_t wconfig = 0;
    uint16_t rconfig = 0;
    uint16_t regvalue = 0;

    wconfig = DRV8301WRITE | REGCTRL1 << 11 | cfg.GATE_CURRENT | cfg.GATE_RESET | cfg.PWM_MODE | cfg.OCP_MODE | cfg.OC_ADJ_SET;
    spi3writeword(wconfig);
    printf("wconfig1 = %d\r\n", wconfig);
    
    rconfig = DRV8301READ | REGCTRL1 << 11;
    spi3writeword(rconfig);
    regvalue = spi3readword();
    printf("regvalue1 = %d\r\n", regvalue);

    // if((regvalue & 0x07ff) != (wconfig &0x07ff)){
    //     return 1;
    // }

    wconfig = 0;
    wconfig = DRV8301WRITE | REGCTRL2 << 11 | cfg.OCTW_MODE | cfg.GAINVALUE | cfg.DC_CAL_CH1 | cfg.DC_CAL_CH2 | cfg.OC_TOFF << 6;
    spi3writeword(wconfig);
    printf("wconfig2 = %d\r\n", wconfig);

    rconfig = DRV8301READ | REGCTRL2 << 11;
    spi3writeword(rconfig);
    regvalue = spi3readword();
    printf("regvalue2 = %d\r\n", regvalue);

    // if((regvalue & 0x007f) != (wconfig & 0x007f)){
    //     return 2;
    // }
    // return 0;
}

uint16_t DRV8301IDread(void){
    uint16_t command = 0;
    uint16_t regvalue = 0;

    command = DRV8301READ | REGSTATUS2 << 11;
    printf("command = %d\r\n", command);
    spi3writeword(command);
    regvalue = spi3readword();

    return regvalue & 0x0f;
}