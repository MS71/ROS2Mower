#include <stdlib.h>
#include <stdint.h>
#include "io_i2c.h"

extern "C" {
#include "EPD_1in54.h"
#include "GUI_Paint.h"
}

#define WHITE          0xFF

int EPD_BUSY_PIN = 0;
int EPD_RST_PIN = 1;
int EPD_DC_PIN = 2;
int EPD_CS_PIN = 3;

volatile int epfd = -1;
uint8_t epgpios = 0;

UBYTE DEV_Module_Init(void)
{
	epfd = io_i2c_open(12);
	//printf("DEV_Module_Init() %d\n",epfd);
	return 0;
}

/*------------------------------------------------------------------------------------------------------*/
void DEV_Digital_Write(UWORD Pin, UBYTE Value)
{
	//printf("DEV_Digital_Write(%d,%d)\n",Pin,Value);
	epgpios &= ~(1<<Pin);
	epgpios |= ((Value&1)<<Pin);
	io_i2c_write_bytes(epfd,&epgpios,1);
}

UBYTE DEV_Digital_Read(UWORD Pin)
{
    uint8_t tmp;
    io_i2c_read_bytes(epfd, &tmp, 1);
    //printf("DEV_Digital_Read(%d) => %d\n", Pin, ((tmp >> Pin) & 1));
    return ((tmp >> Pin) & 1);
}

void DEV_SPI_WriteByte(UBYTE Value)
{
 	//printf("DEV_SPI_WriteByte(%02x)\n",Value);
	uint8_t tmp[2];
	tmp[0] = epgpios;
	tmp[1] = Value;
	io_i2c_write_bytes(epfd,tmp,sizeof(tmp));
}

void DEV_SPI_Write_nByte(uint8_t* pData, uint32_t Len)
{
 	//printf("DEV_SPI_Write_nByte(Len=%d)\n",Len);
	uint8_t tmp[1+Len];
	tmp[0] = epgpios;
	memcpy(&tmp[1],pData,Len);
	io_i2c_write_bytes(epfd,tmp,sizeof(tmp));
}

void EPD_1IN54_SendCommand(UBYTE Value)
{
	uint8_t tmp[2];
	tmp[0] = epgpios | (1<<6);
	tmp[1] = Value;
	io_i2c_write_bytes(epfd,tmp,sizeof(tmp));
}

void EPD_1IN54_SendData(UBYTE Value)
{
	uint8_t tmp[2];
	tmp[0] = epgpios | (1<<7);
	tmp[1] = Value;
	io_i2c_write_bytes(epfd,tmp,sizeof(tmp));
}

void DEV_Delay_ms(UDOUBLE xms)
{
	usleep(1000*xms);
}

void DEV_Module_Exit(void)
{
	io_i2c_close(epfd);
}

int main()
{
    if(DEV_Module_Init() != 0) {
		return -1;
    }

    printf("e-Paper Init and Clear...\r\n");
    EPD_1IN54_Init(EPD_1IN54_FULL);
    EPD_1IN54_Clear();
    //DEV_Delay_ms(500);
	
    //Create a new image cache
    UBYTE *BlackImage;
    /* you have to edit the startup_stm32fxxx.s file and set a big enough heap size */
    UWORD Imagesize = ((EPD_1IN54_WIDTH % 8 == 0)? (EPD_1IN54_WIDTH / 8 ): (EPD_1IN54_WIDTH / 8 + 1)) * EPD_1IN54_HEIGHT;
    if((BlackImage = (UBYTE *)malloc(Imagesize)) == NULL) {
        printf("Failed to apply for black memory...\r\n");
        return -1;
    }
	
    printf("Paint_NewImage\r\n");
    Paint_NewImage(BlackImage, EPD_1IN54_WIDTH, EPD_1IN54_HEIGHT, 0, WHITE);

#if 1	
	printf("Drawing\r\n");
    //1.Select Image
    Paint_SelectImage(BlackImage);
    Paint_Clear(WHITE);

    // 2.Drawing on the image
    Paint_DrawPoint(5, 10, BLACK, DOT_PIXEL_1X1, DOT_STYLE_DFT);
    Paint_DrawPoint(5, 25, BLACK, DOT_PIXEL_2X2, DOT_STYLE_DFT);
    Paint_DrawPoint(5, 40, BLACK, DOT_PIXEL_3X3, DOT_STYLE_DFT);
    Paint_DrawPoint(5, 55, BLACK, DOT_PIXEL_4X4, DOT_STYLE_DFT);

    Paint_DrawLine(20, 10, 70, 60, BLACK, DOT_PIXEL_1X1, LINE_STYLE_SOLID);
    Paint_DrawLine(70, 10, 20, 60, BLACK, DOT_PIXEL_1X1, LINE_STYLE_SOLID);
    Paint_DrawLine(170, 15, 170, 55, BLACK, DOT_PIXEL_1X1, LINE_STYLE_DOTTED);
    Paint_DrawLine(150, 35, 190, 35, BLACK, DOT_PIXEL_1X1, LINE_STYLE_DOTTED);

    Paint_DrawRectangle(20, 10, 70, 60, BLACK, DOT_PIXEL_1X1, DRAW_FILL_EMPTY);
    Paint_DrawRectangle(85, 10, 130, 60, BLACK, DOT_PIXEL_1X1, DRAW_FILL_FULL);

    Paint_DrawCircle(170, 35, 20, BLACK, DOT_PIXEL_1X1, DRAW_FILL_EMPTY);
    Paint_DrawCircle(170, 85, 20, BLACK, DOT_PIXEL_1X1, DRAW_FILL_FULL);
    Paint_DrawString_EN(5, 85, "waveshare", &Font20, BLACK, WHITE);
    Paint_DrawNum(5, 110, 123456789, &Font20, BLACK, WHITE);

    Paint_DrawString_CN(5, 135,"���abc", &Font12CN, BLACK, WHITE);
    Paint_DrawString_CN(5, 155, "΢ѩ����", &Font24CN, WHITE, BLACK);

    EPD_1IN54_Display(BlackImage);
	
	DEV_Delay_ms(2000);
#endif
	 
	printf("Partial refresh\r\n");
    EPD_1IN54_Init(EPD_1IN54_PART);
    Paint_SelectImage(BlackImage);
    Paint_Clear(WHITE);

    PAINT_TIME sPaint_time;
    sPaint_time.Hour = 12;
    sPaint_time.Min = 34;
    sPaint_time.Sec = 56;
    UBYTE num = 100;
    for (;;) {
        sPaint_time.Sec = sPaint_time.Sec + 1;
        if (sPaint_time.Sec == 60) {
            sPaint_time.Min = sPaint_time.Min + 1;
            sPaint_time.Sec = 0;
            if (sPaint_time.Min == 60) {
                sPaint_time.Hour =  sPaint_time.Hour + 1;
                sPaint_time.Min = 0;
                if (sPaint_time.Hour == 24) {
                    sPaint_time.Hour = 0;
                    sPaint_time.Min = 0;
                    sPaint_time.Sec = 0;
                }
            }
        }
        Paint_ClearWindows(0, 0, 0 + Font24.Width * 7, 0 + Font24.Height, WHITE);
        Paint_DrawTime(0, 0, &sPaint_time, &Font24, WHITE, BLACK);
        num = num - 1;
        if(num == 0) {
            break;
        }
        EPD_1IN54_Display(BlackImage);
        //DEV_Delay_ms(500);//Analog clock 1s
	}
		
	   printf("Clear...\r\n");
    //EPD_1IN54_Init(EPD_1IN54_FULL);
    //EPD_1IN54_Clear();

    printf("Goto Sleep...\r\n");
    EPD_1IN54_Sleep();
    free(BlackImage);
    BlackImage = NULL;

    // close 5V
    printf("close 5V, Module enters 0 power consumption ...\r\n");
    DEV_Module_Exit();
}

#if 0
int main()
{
	int fd = io_i2c_open(12);
	uint8_t buf[] = {0x11,0x22,0x33,0x44};
	io_i2c_write_bytes(fd,buf,sizeof(buf));
	io_i2c_close(fd);
}
#endif
