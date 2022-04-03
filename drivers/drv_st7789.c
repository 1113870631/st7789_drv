#include "stdlib.h"
#include "drv_common.h"
#include "drv_st7789.h"
#include "board.h"
#include "stm32f1xx_hal_conf.h"

       /*  cs1 F13
         *  cs2
         *  RES C5
         *  DC  B1
         *  BLK F11
         *  MISO
         *
         * */

#define PORT_DC GPIOB
#define PIN_DC GPIO_PIN_1

#define PORT_RES GPIOC
#define PIN_RES GPIO_PIN_5

#define PORT_BLK GPIOF
#define PIN_BLK GPIO_PIN_11

#define PORT_CS GPIOF
#define PIN_CS GPIO_PIN_13

#define CS1(n)   n ? HAL_GPIO_WritePin(PORT_CS, PIN_CS,GPIO_PIN_RESET):\
                     HAL_GPIO_WritePin(PORT_CS, PIN_CS,GPIO_PIN_SET)

#define BLK(n)   n ? HAL_GPIO_WritePin(PORT_BLK, PIN_BLK,GPIO_PIN_SET):\
                     HAL_GPIO_WritePin(PORT_BLK, PIN_BLK,GPIO_PIN_RESET)

#define RES(n)   n ? HAL_GPIO_WritePin(PORT_RES, PIN_RES,GPIO_PIN_RESET):\
                     HAL_GPIO_WritePin(PORT_RES, PIN_RES,GPIO_PIN_SET)
//low command
#define DC(n)   n ? HAL_GPIO_WritePin(PORT_DC, PIN_DC,GPIO_PIN_SET):\
                     HAL_GPIO_WritePin(PORT_DC, PIN_DC,GPIO_PIN_RESET)


SPI_HandleTypeDef hspi1;


static void st7789_spi_gpio_init(){
    GPIO_InitTypeDef GPIO_InitStruct = {0};
            __HAL_RCC_SPI1_CLK_ENABLE();

            __HAL_RCC_GPIOA_CLK_ENABLE();
            /**SPI1 GPIO Configuration
            PA5     ------> SPI1_SCK
            PA7     ------> SPI1_MOSI
            */
            GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_7;
            GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
            GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
            HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

};
static void st7789_spi_init(){
    hspi1.Instance = SPI1;
            hspi1.Init.Mode = SPI_MODE_MASTER;
            hspi1.Init.Direction = SPI_DIRECTION_1LINE;
            hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
            hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
            hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
            hspi1.Init.NSS = SPI_NSS_SOFT;
            hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
            hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
            hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
            hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
            hspi1.Init.CRCPolynomial = 10;
            if (HAL_SPI_Init(&hspi1) != HAL_OK)
            {
              Error_Handler();
            }

};
static void st7789_extera_gpio_init(){
    /*  cs1 F13
            *  cs2
            *  RES C5
            *  DC  B1
            *  BLK F11
            *  MISO
            *
            * */
           GPIO_InitTypeDef GPIO_InitStruct = {0};
           __HAL_RCC_GPIOF_CLK_ENABLE();
           __HAL_RCC_GPIOC_CLK_ENABLE();
           __HAL_RCC_GPIOB_CLK_ENABLE();

           GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_13;
           GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
           GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
           HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

           GPIO_InitStruct.Pin = GPIO_PIN_1;
           GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
           GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
           HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

           GPIO_InitStruct.Pin = GPIO_PIN_5;
           GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
           GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
           HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

};


void st7789_gpio_reset(){
    //RES  正常高电平
    HAL_GPIO_WritePin(PORT_RES, PIN_RES,GPIO_PIN_RESET);
    HAL_Delay(240);
    HAL_GPIO_WritePin(PORT_RES, PIN_RES,GPIO_PIN_SET);
}


void st7789_hard_init(){
    st7789_spi_gpio_init();
    st7789_spi_init();
    st7789_extera_gpio_init();
    //RES  正常高电平
    RES(0);
    //BLK  高电平打开背光
    BLK(1);
    //CS1  低选中
    CS1(0);
};


/*RDID1 (DAh): Read ID1
 *
 * */
void st7789_s_command(uint8_t command){

     DC(0);
    //发送
     HAL_SPI_Transmit(&hspi1,&command,1,65535);

};
void st7789_r_data(uint8_t*  data){
    //接收
    HAL_SPI_Receive(&hspi1,data,1,65535);
}


void st7789_s_16data(uint16_t  data){

     DC(1);
     //发送
     uint8_t tmp1=data>>8;
     HAL_SPI_Transmit(&hspi1,&tmp1,1,65535);
     HAL_SPI_Transmit(&hspi1,&data,1,65535);
};
void st7789_s_8data(uint8_t  data){

     DC(1);
     //发送
     HAL_SPI_Transmit(&hspi1,&data,1,65535);
};

void st7789_s_data(uint8_t  *data,uint16_t size){

     DC(1);
     //发送
     HAL_SPI_Transmit(&hspi1,&data,size,65535);
};



void st7789_command_init(){
    st7789_gpio_reset();

       /* 关闭睡眠模式 */
        CS1(1);
       st7789_s_command(0x11);
       HAL_Delay(120);

       /* 开始设置显存扫描模式，数据格式等 */

       st7789_s_command(0x36);
       st7789_s_8data(0x00);
       /* RGB 5-6-5-bit格式  */
       st7789_s_command(0x3A);
       st7789_s_8data(0x65);
       /* porch 设置 */
       st7789_s_command(0xB2);
       st7789_s_8data(0x0C);
       st7789_s_8data(0x0C);
       st7789_s_8data(0x00);
       st7789_s_8data(0x33);
       st7789_s_8data(0x33);
       /* VGH设置 */
       st7789_s_command(0xB7);
       st7789_s_8data(0x72);
       /* VCOM 设置 */
       st7789_s_command(0xBB);
       st7789_s_8data(0x3D);
       /* LCM 设置 */
       st7789_s_command(0xC0);
       st7789_s_8data(0x2C);
       /* VDV and VRH 设置 */
       st7789_s_command(0xC2);
       st7789_s_8data(0x01);
       /* VRH 设置 */
       st7789_s_command(0xC3);
       st7789_s_8data(0x19);
       /* VDV 设置 */
       st7789_s_command(0xC4);
       st7789_s_8data(0x20);
       /* 普通模式下显存速率设置 60Mhz */
       st7789_s_command(0xC6);
       st7789_s_8data(0x0F);
       /* 电源控制 */
       st7789_s_command(0xD0);
       st7789_s_8data(0xA4);
       st7789_s_8data(0xA1);
       /* 电压设置 */
       st7789_s_command(0xE0);
       st7789_s_8data(0xD0);
       st7789_s_8data(0x04);
       st7789_s_8data(0x0D);
       st7789_s_8data(0x11);
       st7789_s_8data(0x13);
       st7789_s_8data(0x2B);
       st7789_s_8data(0x3F);
       st7789_s_8data(0x54);
       st7789_s_8data(0x4C);
       st7789_s_8data(0x18);
       st7789_s_8data(0x0D);
       st7789_s_8data(0x0B);
       st7789_s_8data(0x1F);
       st7789_s_8data(0x23);
       /* 电压设置 */
       st7789_s_command(0xE1);
       st7789_s_8data(0xD0);
       st7789_s_8data(0x04);
       st7789_s_8data(0x0C);
       st7789_s_8data(0x11);
       st7789_s_8data(0x13);
       st7789_s_8data(0x2C);
       st7789_s_8data(0x3F);
       st7789_s_8data(0x44);
       st7789_s_8data(0x51);
       st7789_s_8data(0x2F);
       st7789_s_8data(0x1F);
       st7789_s_8data(0x1F);
       st7789_s_8data(0x20);
       st7789_s_8data(0x23);
       /* 显示开 */
       st7789_s_command(0x21);
       st7789_s_command(0x29);

};

void st7789_read_id1(uint8_t*  ID){
    CS1(1);
    st7789_s_command(0xDA);
    st7789_r_data(ID);
    CS1(0);
};


int st7789_connect_check(){
    int tmp=0;
    st7789_read_id1(&tmp);

    if(tmp==133)
        return 1;
    else
        return 0;
}


void LCD_Address_Set(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{

    /* 指定X方向操作区域 */
    CS1(1);
    st7789_s_command(0x2a);
    st7789_s_8data(x1 >> 8);
    st7789_s_8data(x1);
    st7789_s_8data(x2 >> 8);
    st7789_s_8data(x2);

    /* 指定Y方向操作区域 */
    st7789_s_command(0x2b);
    st7789_s_8data(y1 >> 8);
    st7789_s_8data(y1);
    st7789_s_8data(y2 >> 8);
    st7789_s_8data(y2);

    /* 发送该命令，LCD开始等待接收显存数据 */
    st7789_s_command(0x2C);

}


#define LCD_TOTAL_BUF_SIZE  (240*240*2)
#define LCD_Buf_Size 1152
static uint8_t lcd_buf[LCD_Buf_Size];

void LCD_Clear(uint16_t color)
{
    uint16_t i, j;
    uint8_t data[2] = {0};  //color是16bit的，每个像素点需要两个字节的显存

    /* 将16bit的color值分开为两个单独的字节 */
    data[0] = color >> 8;
    data[1] = color;

    /* 显存的值需要逐字节写入 */
    for(j = 0; j < LCD_Buf_Size / 2; j++)
    {
        lcd_buf[j * 2] =  data[0];
        lcd_buf[j * 2 + 1] =  data[1];
    }
    /* 指定显存操作地址为全屏幕 */
    LCD_Address_Set(0, 0, LCD_W - 1, LCD_H - 1);

    /* 将显存缓冲区的数据全部写入缓冲区 */
    for(i = 0; i < (LCD_TOTAL_BUF_SIZE / LCD_Buf_Size); i++)
    {
        st7789_s_data(lcd_buf,LCD_TOTAL_BUF_SIZE);
    }
}




