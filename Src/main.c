/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2020 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"
#include "usb_host.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>

int  init_ram();
#include "st7789.h"
#include "stm32_adafruit_sd.h"

void setBK_imp(int k)
{

}
#define SD_TIMEOUT 30 * 1000

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
DMA_HandleTypeDef hdma_spi2_tx;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SPI2_Init(void);
static void MX_SPI1_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
void MX_USB_HOST_Process(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#if 1
#include "usbh_def.h"
#include "usbh_hid.h"
#include "usbh_hid_keybd.h"
extern USBH_HandleTypeDef hUsbHostFS;
#endif
void retarget_put_char(char p)
{
	HAL_UART_Transmit(&huart1, &p, 1, 0xffffff); // send message via UART
}
int _write(int fd, char* ptr, int len)
{
    (void)fd;
    int i = 0;
    while (ptr[i] && (i < len))
    {
    	if (ptr[i] == '\r')
    	{

    	}
    	else
    	{
			retarget_put_char((int)ptr[i]);
			if (ptr[i] == '\n')
			{
				retarget_put_char((int)'\r');
			}
    	}
        i++;
    }
    return len;
}

// display utils
#if 0
void LCD_Draw_Char(char Character, int16_t X, int16_t Y, uint16_t Colour, uint16_t Size, uint16_t Background_Colour)
{
	printf("!!!LCD_Draw_Char: %c\n",Character);
}
void  LCD_Draw_Text2(const char* Text, int16_t X, int16_t Y, uint16_t Colour, uint16_t SizeX, uint16_t SizeY,uint16_t Background_Colour)
{
	printf("!!!LCD_Draw_Text2: %s\n",Text);
}
void  LCD_Draw_Text(const char* Text, int16_t X, int16_t Y, uint16_t Colour, uint16_t Size, uint16_t Background_Colour)
{
	printf("!!!LCD_Draw_Text: %s\n",Text);
}
#endif
volatile uint16_t keyMatrix  = 0;
uint16_t scanJSK()
{
	return keyMatrix;
}
void clearKey()
{
	keyMatrix = 0;
}

int  volume_perc = 50;

void setVolume0(int percent)
{
	volume_perc = percent;
	if(volume_perc<0)volume_perc = 0;
	if(volume_perc>100)volume_perc = 100;
	percent = volume_perc;
}

int getVolume0()
{
	return volume_perc;
}

#if 1
#define H_SIZE_BITS (6)
#define H_SIZE (1<<H_SIZE_BITS)
#define R_SIZE_BITS (11)
#define R_SIZE (1<<R_SIZE_BITS)
#else
#define H_SIZE_BITS (5)
#define H_SIZE (1<<H_SIZE_BITS)
#define R_SIZE_BITS (6)
#define R_SIZE (1<<R_SIZE_BITS)
#endif

#define PAL     626
#define NTSC    526

#define LINES PAL

//uint16_t VideoBuffer0s[N_SIZE];
//TIM_InitStruct.Autoreload = 12; 84000000/13       = 6.461538MHz

// line_time*cpu_freq/(Autoreload+1)
// 0.000064sec*84000000/13  = 413.53
#define N_TICKS  	(412)

//N_TICKS - BORDER and aligned
#define N_SIZE  	(N_TICKS)

// start pixels position
#define NS_SIZE 	(104)

// last pixels position =start+sinclair SCREENW +8 pixels border*2 must be <= N_TICKS
#define NL_SIZE 	(NS_SIZE+256+16)

// odd even synhro position
#define HALF_TICKS 	(N_TICKS/2)

//2.3*84mhz/13 = 14.86
#define T2_3us      (15)
//4.7*84mhz/13 = 30.36
#define T4_7us      (30)

uint16_t  VideoBuffer[N_SIZE*2];
uint16_t* VideoBuffer0 = &VideoBuffer[0];
uint16_t* VideoBuffer0m =&VideoBuffer[N_SIZE];

#define MAX_LEVEL  7
#define ZERO_LEVEL 0
#define BASE_LEVEL 3
#define SYNC_LEVEL 0
//f= 1/(2*PI*R*C);
//R= 300 Om / F = 5000000Hz ;
//C =1/ (F*2*Pi*R)

/*
#define MAX_LEVEL  8
#define ZERO_LEVEL 0
#define BASE_LEVEL 3
#define SYNC_LEVEL 0
*/
//volatile int complete = 3;
//volatile int oddEven = 0;



// 0 - line def with data, 1 - line def   2 - line1 3 -line3 4-line313 5-line 318 6-Line623 7-Line624

uint8_t policyT[PAL];


inline funcDef(uint16_t *pnt)
{
	uint32_t* pnt32 = pnt;
	for(int k=0;k<T4_7us/2;k++)
	{
		pnt32[k] = (SYNC_LEVEL<<16) | SYNC_LEVEL;
	}
	for(int k=T4_7us/2;k<N_SIZE/2;k++)
	{
		pnt32[k] = (BASE_LEVEL<<16) | BASE_LEVEL;
		//pnt[k] = BASE_LEVEL;
	}
	pnt[N_SIZE-1] = BASE_LEVEL;
}
inline  funcDefT(uint16_t *pnt)
{
	funcDef(pnt);
	/*(
	for(int k=0;k<T4_7us;k++)
	{
		pnt[k] = SYNC_LEVEL;
	}
	for(int k=T4_7us;k<NS_SIZE;k++)
	{
		pnt[k] = BASE_LEVEL;
	}
	for(int k=NL_SIZE;k<N_SIZE;k++)
	{
		pnt[k] = BASE_LEVEL;
	}
	*/
}
void funcL1(uint16_t *pnt)
{
	for(int k=0;k<N_SIZE;k++)
	{
		pnt[k] = BASE_LEVEL;
	}
	for(int k=0;k<HALF_TICKS-T4_7us;k++)
	{
		pnt[k] = SYNC_LEVEL;
	}
	for(int k=N_SIZE-T4_7us;k<N_SIZE;k++)
	{
		pnt[k] = SYNC_LEVEL;
	}
}

void funcL3(uint16_t *pnt)
{
	for(int k=0;k<N_SIZE;k++)
	{
		pnt[k] = BASE_LEVEL;
	}
	for(int k=0;k<HALF_TICKS-T4_7us;k++)
	{
		pnt[k] = SYNC_LEVEL;
	}
	for(int k=HALF_TICKS;k<HALF_TICKS+T2_3us;k++)
	{
		pnt[k] = SYNC_LEVEL;
	}
}

void funcL313(uint16_t *pnt)
{
	for(int k=0;k<N_SIZE;k++)
	{
		pnt[k] = BASE_LEVEL;
	}
	for(int k=0;k<T2_3us;k++)
	{
		pnt[k] = SYNC_LEVEL;
	}
	for(int k=HALF_TICKS;k<N_SIZE-T4_7us;k++)
	{
		pnt[k] = SYNC_LEVEL;
	}
}

void funcL318(uint16_t *pnt)
{
	for(int k=0;k<N_SIZE;k++)
	{
		pnt[k] = BASE_LEVEL;
	}
	for(int k=0;k<T2_3us;k++)
	{
		pnt[k] = SYNC_LEVEL;
	}
}
void funcL623(uint16_t *pnt)
{
	for(int k=0;k<N_SIZE;k++)
	{
		pnt[k] = BASE_LEVEL;
	}
	for(int k=0;k<T4_7us;k++)
	{
		pnt[k] = SYNC_LEVEL;
	}
	for(int k=HALF_TICKS;k<HALF_TICKS+T2_3us;k++)
	{
		pnt[k] = SYNC_LEVEL;
	}
}
void funcL624(uint16_t *pnt)
{
	for(int k=0;k<N_SIZE;k++)
	{
		pnt[k] = BASE_LEVEL;
	}
	for(int k=0;k<T2_3us;k++)
	{
		pnt[k] = SYNC_LEVEL;
	}
	for(int k=HALF_TICKS;k<HALF_TICKS+T2_3us;k++)
	{
		pnt[k] = SYNC_LEVEL;
	}
}

enum lineType {LinWD,lineND,line1,line3,line313,line318,line623,line624};
volatile int lineN = 0;
int 		 vBaseLine = 0;
int initTables()
{
	int WW= 0;
	lineN = 0;
    for(int k=0;k<PAL;k++)
    {
    	policyT[k] = LinWD;
    }

    if(LINES==PAL)
    {
    	policyT[623]   = line623;
    	policyT[624]   = line624;
    	policyT[625]   = line624;
    	policyT[1]     = line1;
    	policyT[2]     = line1;
    	policyT[3]     = line3;
    	policyT[4]     = line624;
    	policyT[5] 	   = line624;
    	policyT[311]   = line624;
    	policyT[312]   = line624;
    	policyT[313]   = line313;
    	policyT[314]   = line1;
    	policyT[315]   = line1;
    	policyT[316]   = line624;
    	policyT[317]   = line624;
    	policyT[318]   = line318;
    	vBaseLine = 23+25;
    	for(int k=0;k<23+25;k++)
    	{
    		if(policyT[k]==LinWD)
    			policyT[k] = lineND;
    		if(policyT[k+LINES/2]==LinWD)
    			policyT[k+LINES/2] = lineND;
    	}

    	for(int k=LINES/2-25;k<LINES/2;k++)
    	{
    		if(policyT[k]==LinWD)
    			policyT[k] = lineND;
    		if(policyT[k+LINES/2]==LinWD)
    			policyT[k+LINES/2] = lineND;
    	}
    }
    else
    {
    	policyT[1]   = line624;
    	policyT[2]   = line624;
    	policyT[3]   = line624;
    	policyT[4]   = line1;
    	policyT[5]   = line1;
    	policyT[6]   = line1;
    	policyT[7]   = line624;
    	policyT[8]   = line624;
    	policyT[9]   = line624;

    	policyT[263]   = line623;
    	policyT[264]   = line624;
    	policyT[265]   = line624;
    	policyT[266]   = line313;
    	policyT[267]   = line1;
    	policyT[268]   = line1;
    	policyT[269]   = line3;
    	policyT[270]   = line1;
    	policyT[271]   = line1;
    	policyT[272]   = line318;
    	vBaseLine = 23;
    	for(int k=0;k<23;k++)
    	{
    		if(policyT[k]==LinWD)
    			policyT[k] = lineND;
    		if(policyT[k+LINES/2]==LinWD)
    			policyT[k+LINES/2] = lineND;
    	}
    }
    for(int k=0;k<LINES;k++)
    {
    	if(policyT[k] == LinWD) WW++;
    }
    return WW;
}
volatile int prevFull0;
volatile int prevFull1;
int testPattern = 1;

const int stable[] = {0,1,2,3,4,5,6,7,6,5,4,3,2,1,0};
const int tables[] = {0,1,1,2,1,2,2,3};
//uint8_t  BKL0a[512];
//uint32_t BKL1a[512*4];
extern const uint32_t BKL1a[512][4];
uint32_t delta0 = 0;
uint32_t delta1 = 0;
uint32_t delta2 = 0;
uint8_t getBorderColor(int line);

void HAL_TIM_PeriodElapsedCallback3(int hl)
{
			  uint32_t t_start = TIM3->CNT;
			  uint16_t * pnt0 = hl?VideoBuffer0m:VideoBuffer0;
			  lineN++;
			  if(lineN==LINES)
			  {
				  lineN = 1;
			  }

			  int pol = policyT[lineN];
			  if((prevFull0!=pol) || (prevFull1!=pol))
			  {
				  switch(pol)
				  {
					  case  line1: funcL1(pnt0);break;
					  case  line3: funcL3(pnt0);break;
					  case  line313: funcL313(pnt0);break;
					  case  line318: funcL318(pnt0);break;
					  case  line623: funcL623(pnt0);break;
					  case  line624: funcL624(pnt0);break;
					  case  LinWD:   funcDefT(pnt0);break;
					  case  lineND:   funcDef(pnt0);break;
				  }
			  }
			  prevFull0 = prevFull1;
			  prevFull1 = pol;
			  int flagTime = ((HAL_GetTick()/400)&1)*0x100;
			  int tline = -1;
			  if(pol==LinWD)
			  {
				  int rLine = ((lineN-1) % (LINES/2))-(vBaseLine-1);
				  if(testPattern)
				  {
					  pnt0 += NS_SIZE;

					  if(rLine<80)
					  {
						  for(int k=0;k<NL_SIZE-NS_SIZE-4;k++)
						  {
							  pnt0[k] = ((k&1)?ZERO_LEVEL:(MAX_LEVEL/2))+BASE_LEVEL;
						  }
					  }
					  else if(rLine<160)
					  {
						  for(int k=0;k<NL_SIZE-NS_SIZE-4;k++)
						  {
							  pnt0[k] = (stable[(k/8)&0xf])+BASE_LEVEL;
						  }
					  }
					  else
					  {
						  for(int k=0;k<NL_SIZE-NS_SIZE-4;k++)
						  {
							  pnt0[k] = ((k&4)?ZERO_LEVEL:(MAX_LEVEL/3))+BASE_LEVEL;
						  }
					  }
					  /*
					  if(rLine==0)
					  {
						  for(int k=0;k<NL_SIZE-NS_SIZE-4;k++)
							  pnt0[k] = ((k&4)?ZERO_LEVEL:(MAX_LEVEL))+BASE_LEVEL;

					  }
					  if(rLine==239)
					  {
						  for(int k=0;k<NL_SIZE-NS_SIZE-4;k++)
							  pnt0[k] = ((k&4)?ZERO_LEVEL:MAX_LEVEL)+BASE_LEVEL;

					  }
					  */
				  }
				  else  // real data from spectrum
				  {
					  pnt0 += NS_SIZE;
					  //SINCLAIR_FLAGS.border = SINCLAIR_FLAGS.borderArray[rLine];
					  //SINCLAIR_FLAGS.borderArray_OLD[rLine] =SINCLAIR_FLAGS.border;
					  uint8_t border = getBorderColor(rLine);
					  int c10 = tables[border&7];

					  int scale = (0xA0*MAX_LEVEL/3)>>8;
					  uint16_t CLR1  =  scale*(c10)+BASE_LEVEL;
					  uint32_t CLR_F = (CLR1<<16)|CLR1  ;
					  uint32_t *pnt32 = (uint32_t *)pnt0;
					  for(int k=0;k<8;k++)
					  {
						  pnt0[k] = CLR1;
					  }

					  tline = rLine-(240-192)/2;
					  if(tline>=0&&tline<192)
					  {

						  int yb =  tline / 8;
						  int yt =  tline & 7;
						  int y  =  tline;// yt+yb*8;
						  uint16_t vr = ((y<<8)&0x0700)|((y<<2)&0xe0)|((y<<5)&0x1800);
						  uint8_t* VRAM = get_VIDEO_RAM()+vr;
						  uint8_t* ARAM = get_ATTR_RAM()+yb*32;
						  uint32_t *pnt = &pnt0[8];
						  for(int xb=0;xb<32;xb++)
						  {
							  //  uint16_t CLR0B = BKL0a[ARAM[xb]|flagTime];
							    uint32_t* BKL1a0 = &BKL1a[ARAM[xb]|flagTime][0];
								//uint16_t CLR0  = CLR0B&0xf;
								//uint16_t CLR1  = CLR0B>>4;
								uint8_t  val   = VRAM[xb];
								pnt[0] = BKL1a0[(val>>6)&0x3];
								pnt[1] = BKL1a0[(val>>4)&0x3];
								pnt[2] = BKL1a0[(val>>2)&0x3];
								pnt[3] = BKL1a0[(val)&0x3];
								pnt+=4;
#if 0
#if 0
                                for(int bit=0;bit<8;bit++)
                                {
                                	pnt[bit] = (val&(0x80>>bit))?CLR1:CLR0;
                                }
#else
								pnt[0] = (val&0x80)?CLR1:CLR0;
								pnt[1] = (val&0x40)?CLR1:CLR0;
								pnt[2] = (val&0x20)?CLR1:CLR0;
								pnt[3] = (val&0x10)?CLR1:CLR0;
								pnt[4] = (val&0x08)?CLR1:CLR0;
								pnt[5] = (val&0x04)?CLR1:CLR0;
								pnt[6] = (val&0x02)?CLR1:CLR0;
								pnt[7] = (val&0x01)?CLR1:CLR0;
#endif
								pnt += 8;
#endif
						  }

						 // uint8_t* ppnt = get_VIDEO_RAM()+ 32*tline;
						 // for(int k=8;k<NL_SIZE-NS_SIZE-4-8;k++)
						 // {
						//	  int bit = k&0x7;
						//	  pnt0[k] = (ppnt[(k-8)/8]>>bit)+BASE_LEVEL;
						 // }
					  }
					  else
					  {
						  uint32_t *pnt32z = (uint32_t *)pnt0;
						  for(int k=8/2;k<(NL_SIZE-NS_SIZE-8)/2;k++)
						  {
							  pnt32z[k] = (CLR1<<16)|CLR1;;
						  }
						  //for(int k=8/2;k<(NL_SIZE-NS_SIZE-4-8)/2;k++)
						  {
							  //pnt[k] = CLR_F;
						  }
					  }
					  uint32_t *pnt32z = (uint32_t *)pnt0;
					  for(int k=(NL_SIZE-NS_SIZE-8)/2;k<(NL_SIZE-NS_SIZE)/2;k++)
					  {
						  pnt32z[k] = (CLR1<<16)|CLR1;
						  //pnt0[k] = pnt0[k] = CLR1;ZERO_LEVEL+BASE_LEVEL+2;
					  }
				  }
			  }
			  if(tline==0)
			  delta0 =  TIM3->CNT-t_start;
			  if(tline==192/2)
			  delta1 =  TIM3->CNT-t_start;
			  if(tline==192)
			  delta2 =  TIM3->CNT-t_start;
}

#ifdef TV_OUT
/*
void LCD_Draw_Char(char Character, int16_t X, int16_t Y, uint16_t Colour, uint16_t Size, uint16_t Background_Colour)
{
	printf("!!!LCD_Draw_Char: %c\n",Character);
}
void  LCD_Draw_Text2(const char* Text, int16_t X, int16_t Y, uint16_t Colour, uint16_t SizeX, uint16_t SizeY,uint16_t Background_Colour)
{
	printf("!!!LCD_Draw_Text2: %s\n",Text);
}
void  LCD_Draw_Text(const char* Text, int16_t X, int16_t Y, uint16_t Colour, uint16_t Size, uint16_t Background_Colour)
{
	printf("!!!LCD_Draw_Text: %s\n",Text);
}
*/
int LCD_getWidth()
{
	return 256;
}
int LCD_getHeight()
{
	return 192;
}
void setPixel(int x,int y,int color,uint8_t attr)
{
	if(x>=0&&x<256&&y>=0&&y<192)
	{
	  int tline = y;
	  int yb =  tline / 8;
	  int yt =  tline & 7;
	  uint16_t vr = ((y<<8)&0x0700)|((y<<2)&0xe0)|((y<<5)&0x1800);
	  uint8_t* VRAM = get_VIDEO_RAM()+vr;
	  uint8_t* ARAM = get_ATTR_RAM()+yb*32;
	  int xb = x/8;
	  ARAM[xb] = attr;
	  if(color)
	  {
		  VRAM[xb] |= 0x80 >>(x&7);
	  }
	  else
	  {
		  VRAM[xb] &= ~(0x80 >>(x&7));
	  }
	}
}
void LCD_fillRect(uint16_t x1, uint16_t y1, uint16_t w, uint16_t h, uint16_t color)
{

      if(color==BLACK)
      {
    	  for(int y=y1;y<y1+h;y++)
    	  {
    		  for(int x=x1;x<x1+w;x++)
    		  {
    			  setPixel(x,y,1,7*8+0);
    		  }
    	  }
      }
      else
      {
    	  for(int y=y1;y<y1+h;y++)
    	  {
    		  for(int x=x1;x<x1+w;x++)
    		  {
    			  setPixel(x,y,0,7*8+0);
    		  }
    	  }
      }

}
void LCD_Write8x8line16(uint16_t x1,uint16_t y1,uint16_t * data)
{
}
#endif

void clearFullScreen()
{
	LCD_fillRect(0, 0, LCD_getWidth(), LCD_getHeight(), BLACK);
}

uint16_t VoiceBuff1[H_SIZE];
uint8_t VoiceBuff1T[R_SIZE];

volatile int sound_head = 0;
volatile int sound_tail = 0;

int sound_size()
{
	return (sound_head+R_SIZE-sound_tail)&(R_SIZE-1);
}
inline int iwaitFor()
{
	int cntWait = 0;
	//TimMasterHandle.Instance = TIM2;
	while(((sound_head+R_SIZE-sound_tail)&(R_SIZE-1))>(R_SIZE-16))
	{

		    // Disable HAL tick interrupt
		   // __HAL_TIM_DISABLE_IT(&TimMasterHandle, TIM_IT_CC2);

		    // Request to enter SLEEP mode
		    //HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);

		    // Enable HAL tick interrupt
		    //__HAL_TIM_ENABLE_IT(&TimMasterHandle, TIM_IT_CC2);
				//HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
		//HAL_Delay(1);
		cntWait++;
	}
	return cntWait;
}
int waitFor()
{
	return iwaitFor();
}
void push_pair(uint16_t Left,uint16_t Right)
{
	iwaitFor();
	VoiceBuff1T[sound_head] = Right>>2;
	sound_head = (sound_head+1)&(R_SIZE-1);
}
void cleanAudioBuffer()
{
	waitFor();
	for(int k=0;k<R_SIZE;k++)
	{
#ifdef DAC_OUTPUT
			push_pair(2048,2048);
#else
			push_pair(MAX_VOLUME/2,MAX_VOLUME/2);
#endif
	}
}
volatile int cnt0 = 0;
volatile int cnt1 = 0;
void TIM4_TC7()
{
	cnt0++;
	int k;
	for(k=H_SIZE/2;k<H_SIZE;k++)
	{
		VoiceBuff1[k] =((uint16_t)(VoiceBuff1T[sound_tail]))<<2;
		sound_tail = (sound_tail+1)&(R_SIZE-1);
	}
}
void TIM4_HT7()
{
	cnt1++;
	int k;
	for(k=0;k<H_SIZE/2;k++)
	{
		VoiceBuff1[k] = ((uint16_t) (VoiceBuff1T[sound_tail]))<<2;
		sound_tail = (sound_tail+1)&(R_SIZE-1);
	}
}


void loop()
{
}
extern int ustate;

uint8_t* SD_BUFF = (uint8_t* ) &VideoBuffer[0];
uint8_t bbuf[512];
void *	m_malloc (size_t __size)
{
	if(__size == 512)
	{
		return bbuf;
	}
	else
	{
		//printf("alloc size = %d\n",__size);

		return malloc(__size);
	}
}
void m_free (void *pnt)
{
	if(pnt==bbuf)
	{

	}
	else
		free(pnt);
}

#define IMAGE_WIDTH  240
RGB_typedef *RGB_matrix;
/*
uint16_t color_convertRGB_to161(uint8_t R,uint8_t G,uint8_t B)
{

	return __REV16(((R>>3)<<11)|((G>>2)<<5)|(B>>3));
}
*/
void LCD_sendLineRect(uint16_t y1,uint8_t * data);

void TIM1_TC2()
{
	HAL_TIM_PeriodElapsedCallback3(1);
}
void TIM1_HT2()
{
	HAL_TIM_PeriodElapsedCallback3(0);
}
void TIM1_TE2()
{
}

#if 1
void init_timerLL1()
{
	  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA2);

	  LL_DMA_ConfigAddresses(DMA2, LL_DMA_STREAM_2, (uint32_t)&VideoBuffer[0], (uint32_t)&TIM1->CCR2, LL_DMA_GetDataTransferDirection(DMA2, LL_DMA_STREAM_2));
	  LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_2, N_SIZE*2);
	  LL_DMA_EnableIT_TC(DMA2, LL_DMA_STREAM_2);
	  LL_DMA_EnableIT_HT(DMA2, LL_DMA_STREAM_2);


	  /***************************/
	  /* Enable the DMA transfer */
	  /***************************/
	  LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_2);


	  LL_TIM_EnableDMAReq_CC2(TIM1);

	  LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH2);

	  LL_TIM_EnableAllOutputs(TIM1);
	  LL_TIM_EnableCounter(TIM1);




}
#endif

void init_timerLL4()
{
	  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA2);
	  LL_DMA_ConfigAddresses(DMA1, LL_DMA_STREAM_7, (uint32_t)&VoiceBuff1[0], (uint32_t)&TIM4->CCR3, LL_DMA_GetDataTransferDirection(DMA1, LL_DMA_STREAM_7));
	  LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_7, H_SIZE);
	  LL_DMA_EnableIT_TC(DMA1, LL_DMA_STREAM_7);
	  LL_DMA_EnableIT_HT(DMA1, LL_DMA_STREAM_7);


	  /***************************/
	  /* Enable the DMA transfer */
	  /***************************/
	  LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_7);


	  LL_TIM_EnableDMAReq_CC3(TIM4);

	  LL_TIM_CC_EnableChannel(TIM4, LL_TIM_CHANNEL_CH3);

	  LL_TIM_EnableAllOutputs(TIM4);
	  LL_TIM_EnableCounter(TIM4);




}
extern uint8_t usb_mode;
extern USBH_HandleTypeDef hUsbHostFS;
uint8_t kstate = 0;

int dummyFunction(struct SYS_EVENT* ev)
{
	printf("dummy mess   = %x\r\n",ev->message);
	printf("dummy param1 = %x\r\n",ev->param1);
	printf("dummy param2 = %x\r\n",ev->param2);
	return 0;
}

dispathFunction currFunc = &dummyFunction;
#define stackFuncSize 16
dispathFunction stackFunc[stackFuncSize];


int stackFuncP = 0;

void pushMenuFunc(void*fnk)
{
	if(stackFuncP<stackFuncSize-2)
	{
		stackFunc[stackFuncP] = fnk;
		stackFuncP++;
	}
}

dispathFunction popMenuFunc()
{
	if(stackFuncP>0)
	{
		stackFuncP--;
	}
	currFunc = stackFunc[stackFuncP];
	return currFunc;
}

void setCurrentFunc(void*fnk)
{
	pushMenuFunc(currFunc);
	currFunc = fnk;
}

char *mess = "Hello Dima\r";
int readDirIntoList1(char* currentDir)
{
	FILINFO 	MyFileInfo;
	DIR 		MyDirectory;
	FRESULT 	res;
	res = f_opendir(&MyDirectory,currentDir);
	int files = 0;
	if(res == FR_OK)
	{
		for (;;)
		{
			      res = f_readdir(&MyDirectory, &MyFileInfo);
			      if(res != FR_OK || MyFileInfo.fname[0] == 0)
				break;
			      //~ if(numFiles<LCD_getHeight()/CHAR_HEIGHT-2)
			      {
				     // if(files<NUM_LINES)
				      {
					      //strcpy(Lines[files].cacheLine,MyFileInfo.fname);
				    	  printf("file:: %7d %x	%s \r\n",MyFileInfo.fsize,MyFileInfo.fattrib,MyFileInfo.fname);
				    	  //putStr(MyFileInfo.fattrib,MyFileInfo.fname);
					      files++;
				      }
				      //~ const uint16_t yScr = (240-192)/2-4;
				      //~ const uint16_t xScr = (320-256)/2-4;
				      //~ int lp = (snumFiles-baseline);
				      //~ if(lp>=0&&lp<192/8-1)
				      //~ {
						//~ LCD_Draw_Text(MyFileInfo.fname,xScr,yScr+(snumFiles-baseline)*8, GREEN, 1, (selection==snumFiles)?YELLOW:BLACK);
				      //~ }
			      }
		}
      }
	else if(res==FR_NO_FILESYSTEM)
	{
		printf("FR_NO_FILESYSTEM\n");
		//~ LCD_Draw_Text("format?",10,130, GREEN, 2, BLACK);
	}
	else
	{
		//~ LCD_Draw_Text("open fail",10,130, GREEN, 2, BLACK);
	}
    f_closedir(&MyDirectory);
    return files;
}
int capsFl = 9;
void init_Sinclair();
int menu_dispatch(struct SYS_EVENT* ev);
int z48_z128_dispatch(struct SYS_EVENT* ev);
//uint8_t kbs[256];
HID_KEYBD_Info_TypeDef prev_state_s;
void setTapeSpeed(int ts);

void kscan0()
{
        MX_USB_HOST_Process();
    	if(usb_mode==HOST_USER_CLASS_ACTIVE)
    	{
			if(capsFl == 9)
			{
				kstate = 0x80;
				capsFl = 0;
			}
    		HID_KEYBD_Info_TypeDef * ki=USBH_HID_GetKeybdInfo(&hUsbHostFS);
    		//int reps = 1;
    		//while(ki && reps>0)
    		if(ki)
    		{
    			//reps --;
				prev_state_s = *ki;
				printf("%d %d %d %x : %x %x %x %x %x%x%x%x%x%x%x%x\n",
												delta0,
												delta1,
												delta2,
												ki->state,
												ki->keys[0],
												ki->keys[1],
												ki->keys[2],
												ki->keys[3],
												ki->lctrl,
												ki->lshift,
												ki->lalt,
												ki->lgui,
												ki->rctrl,
												ki->rshift,
												ki->ralt,
												ki->rgui);

#if 0
				char c = USBH_HID_GetASCIICode(ki);
				printf("%x : %x %x %x %x %x%x%x%x%x%x%x%x\n",
												ki->state,
												ki->keys[0],
												ki->keys[1],
												ki->keys[2],
												ki->keys[3],
												ki->lctrl,
												ki->lshift,
												ki->lalt,
												ki->lgui,
												ki->rctrl,
												ki->rshift,
												ki->ralt,
												ki->rgui);
#endif
				for(int k=0;k<1;k++)
				{
					switch(ki->keys[k])
					{
				    	case  KEY_RIGHTARROW: 	keyMatrix = K_RIGHT;break;
					    case  KEY_LEFTARROW: 	keyMatrix = K_LEFT;break;
					    case  KEY_DOWNARROW: 	keyMatrix = K_DOWN;break;
						case  KEY_UPARROW: 		keyMatrix = K_UP;break;
						case  KEY_F10: 			setTapeSpeed(0);break;
						case  KEY_F11: 			setTapeSpeed(1);break;
						case  KEY_F12: 			setTapeSpeed(2);break;

						case  0x0:  			keyMatrix = 0;break;
						case  KEY_ENTER:  		keyMatrix = K_FIRE;break;//enterbreak;
						case  KEY_SPACEBAR: 	keyMatrix = K_SPACE;break;
						case  KEY_ESCAPE:		keyMatrix = K_ESC;break;
						case  KEY_F7:  			setScreenOffset(0);break;
						case  KEY_F8:  			setScreenOffset(1);break;
						case  KEY_F9:  			setScreenOffset(2);break;
						case  KEY_F6:
										initRam();
										ay_reset();
										z80_reset(1);
										SKEY_QUE_init();
										setScreenOffset(0);
									break;
					};
				}

				if(ki->keys[0]==0x39)
				{
					if(capsFl&2)
					{
						capsFl&=~2;
					}
					else
					{
						capsFl|=2;
					}
					kstate = 0x80 | capsFl;
				}
				if(ki->keys[0]==0x53)
				{
					if(capsFl&1)
					{
						capsFl&=~1;
					}
					else
					{
						capsFl|=1;
					}
					kstate = 0x80 | capsFl;
				}
				if(ki->keys[0]==0x47)
				{
					if(capsFl&4)
					{
						capsFl&=~4;
					}
					else
					{
						capsFl|=4;
					}
					kstate = 0x80 | capsFl;
				}
			//	ki = USBH_HID_GetKeybdInfo(&hUsbHostFS);
		}
		if(kstate&0x80)
		{
		    	kstate &= 0x7f;
		    	int tmOut = 1000;
		    	while(tmOut>0&&USBH_HID_SetReport(&hUsbHostFS,0x2,0,&kstate,1)!=USBH_OK)
		    	{
		    		tmOut--;
		    	};
		}
	}
	else
	{
		  capsFl = 9;
	}
}

//~ volatile int cntP  = 0;

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_SPI2_Init();
  MX_SPI1_Init();
  MX_RTC_Init();
  MX_FATFS_Init();
  MX_USB_HOST_Init();
  MX_TIM4_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
   printf("\n\nProgram start\n");
  // init_ram();
   init_Sinclair();
	if(HAL_TIM_Base_Start(&htim3)!= HAL_OK)
	{
		printf("set TIM1 TIM_CHANNEL_3 error\r\n");
	}
	{
		uint32_t tstart= TIM3->CNT;
		HAL_Delay(10);
		tstart= TIM3->CNT-tstart;
		printf("TIM3->CNT in 10 ms= %d\n",tstart);
	}
   SD_CardInfo CardInfo;
   uint8_t sd_state= BSP_SD_Init();
   printf("sd_state = %x\n",sd_state);
   BSP_SD_GetCardInfo(&CardInfo);
	int WW= initTables();
   printf("W2 =%d\n",NL_SIZE-NS_SIZE);
   printf("H=%d %d\n",WW/2,WW);

   printf("hsd.CardCapacity =%d\n",CardInfo.CardCapacity);
   //printf("hsd.SdCard.BlockNbr=%d\n",hsd.SdCard.BlockNbr);
   //printf("hsd.SdCard.BlockSize=%d\n",hsd.SdCard.BlockSize);
   //printf("hsd.SdCard.CardType=%d\n",hsd.SdCard.CardType);
   //printf("hsd.SdCard.CardVersion=%d\n",hsd.SdCard.CardVersion);
   //printf("hsd.SdCard.Class=%d\n",hsd.SdCard.Class);
   printf("hsd.CardBlockSize =%d\n",CardInfo.CardBlockSize);
   printf("hsd.SdCard.LogBlockNbr=%d\n",CardInfo.LogBlockNbr);
   printf("hsd.SdCard.LogBlockSize=%d\n",CardInfo.LogBlockSize);

 /*
   printf("CardInfo.BlockNbr  = 0x%x\n",CardInfo.BlockNbr);
   printf("CardInfo.BlockSize = 0x%x\n",CardInfo.BlockSize);
   printf("CardInfo.CardType  = 0x%x\n",CardInfo.CardType);
   printf("CardInfo.CardVersion  = 0x%x\n",CardInfo.CardVersion);
   printf("CardInfo.Class  = %x\n",CardInfo.Class);
   printf("CardInfo.LogBlockNbr  = 0x%x\n",CardInfo.LogBlockNbr);
   printf("CardInfo.LogBlockSize  = 0x%x\n",CardInfo.LogBlockSize);
   printf("CardInfo.RelCardAdd  = 0x%x\n",CardInfo.RelCardAdd);
 */
   uint64_t size_k = CardInfo.LogBlockNbr;
   size_k = size_k*CardInfo.LogBlockSize;


   printf("CardInfo size %d MB\n",(int)(size_k/1024/1024));
  printf("freq= %d %d %d %d\n",HAL_RCC_GetSysClockFreq(),HAL_RCC_GetHCLKFreq(),HAL_RCC_GetPCLK1Freq(),HAL_RCC_GetPCLK2Freq());
  {
	  int32_t tk = HAL_GetTick();
	  int stat = 0;
	  int kk;
	  for(kk=0;(kk<2*64)&&!stat;kk++)
	  {
  			stat = BSP_SD_ReadBlocks(&SD_BUFF[0],kk,1, SD_TIMEOUT);
	  }
	  tk = HAL_GetTick()-tk;
	  printf("Read %d blocks (bytes = %d) in %d ms st = %d\n",kk,kk*512,tk,stat);
	//  paint();
  }
  f_mount(NULL,USERPath, 0);
	if(f_mount(&USERFatFS,USERPath, 0) != FR_OK)
	{
		printf("f_mount Error!\r\n");
		return 0;
	}
#if 0
	char baseDir[0x20];
	strcpy(baseDir,"/");
	int rfiles =  readDirIntoList1(baseDir);
	strcpy(baseDir,"/Z48");
	printf("Z48:\n------");
	rfiles =  readDirIntoList1(baseDir);
	strcpy(baseDir,"/Z128");
	printf("Z128:\n------");
	rfiles =  readDirIntoList1(baseDir);
#endif
//   init_Display();
#ifndef TV_OUT
      {
	      LCD_init();
	      LCD_setRotation(PORTRAIT_FLIP);
   		uint32_t trt = HAL_GetTick();
   		int k;
   		//  printf("RID4 %x\n",ili9341_ReadID4());
   		//  printf("RID4 %x\n",ili9341_ReadID4());
   		//setBK_imp(99);
   		for(k=0;k<10;k++)
   		{
   			LCD_fillRect(0, 0, LCD_getWidth(), LCD_getHeight(), rand());
   			//HAL_GPIO_TogglePin(BOARD_LED_GPIO_Port,BOARD_LED_Pin);
   		}
   		trt = (HAL_GetTick()-trt);
   		printf("SCR_FULL %d uS\n",trt*100);
        LCD_fillRect(0, 0, LCD_getWidth(), LCD_getHeight(),BLACK);
      }
#endif
      /* Open the JPG image with read access */
	    printf("initTimers\n");
	    init_timerLL1();
	    init_timerLL4();
		printf("setCurrentFunc(&menu_dispatch)\n");
		volatile struct SYS_EVENT event;
		event.message = MESS_OPEN;
		event.param1  = 0x1234;
		event.param2  = 0x5678;
	#warning "setCurrentFunc(&menu_dispatch),no sinclair on start!"
	//	setModeFile(1);
	//	setCurrentFunc(&z48_z128_dispatch);
		HAL_Delay(3000);
		testPattern = 0;
		setCurrentFunc(&menu_dispatch);



  //    init_ram();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
   while (1)
  {
	  	  if(keyMatrix&&(event.message==MESS_IDLE))
	  	  {
	  		  event.param1 = keyMatrix;
	  		  event.param2 = HAL_GetTick();
	  		  event.message = MESS_KEYBOARD;
	  		  //printf("key = %x\n",keyMatrix);
	  		 // printf("cnt t3 %d %x %x %x \r\n ",event.param2,keyMatrix,TIM3->CNT,TIM1->CNT);
	  	  }
	  	  else
	  	  {
	  		  HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
	  	  }
	  	  (*currFunc)(&event);
	  	  if(event.message==MESS_CLOSE)
	  	  {
	  		event.message = MESS_REPAINT;
	  		printf("stackFuncP B =%d\r\n",stackFuncP);
	  		popMenuFunc();
	  		printf("stackFuncP A b=%d\r\n",stackFuncP);
	  		//currFunc = &menu_dispatch;
	  	  }
    /* USER CODE END WHILE */
    MX_USB_HOST_Process();

    /* USER CODE BEGIN 3 */
    kscan0();
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /**Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /**Initialize RTC Only 
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */
    
  /* USER CODE END Check_RTC_BKUP */

  /**Initialize RTC and set the Time and Date 
  */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 0x1;
  sDate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  LL_TIM_InitTypeDef TIM_InitStruct = {0};
  LL_TIM_OC_InitTypeDef TIM_OC_InitStruct = {0};
  LL_TIM_BDTR_InitTypeDef TIM_BDTRInitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM1);

  /* TIM1 DMA Init */
  
  /* TIM1_CH2 Init */
  LL_DMA_SetChannelSelection(DMA2, LL_DMA_STREAM_2, LL_DMA_CHANNEL_6);

  LL_DMA_SetDataTransferDirection(DMA2, LL_DMA_STREAM_2, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);

  LL_DMA_SetStreamPriorityLevel(DMA2, LL_DMA_STREAM_2, LL_DMA_PRIORITY_VERYHIGH);

  LL_DMA_SetMode(DMA2, LL_DMA_STREAM_2, LL_DMA_MODE_CIRCULAR);

  LL_DMA_SetPeriphIncMode(DMA2, LL_DMA_STREAM_2, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA2, LL_DMA_STREAM_2, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(DMA2, LL_DMA_STREAM_2, LL_DMA_PDATAALIGN_HALFWORD);

  LL_DMA_SetMemorySize(DMA2, LL_DMA_STREAM_2, LL_DMA_MDATAALIGN_WORD);

  LL_DMA_EnableFifoMode(DMA2, LL_DMA_STREAM_2);

  LL_DMA_SetFIFOThreshold(DMA2, LL_DMA_STREAM_2, LL_DMA_FIFOTHRESHOLD_FULL);

  LL_DMA_SetMemoryBurstxfer(DMA2, LL_DMA_STREAM_2, LL_DMA_MBURST_INC4);

  LL_DMA_SetPeriphBurstxfer(DMA2, LL_DMA_STREAM_2, LL_DMA_PBURST_SINGLE);

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  TIM_InitStruct.Prescaler = 0;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 12;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  TIM_InitStruct.RepetitionCounter = 0;
  LL_TIM_Init(TIM1, &TIM_InitStruct);
  LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH2);
  TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM1;
  TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.CompareValue = 0;
  TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
  TIM_OC_InitStruct.OCNPolarity = LL_TIM_OCPOLARITY_HIGH;
  TIM_OC_InitStruct.OCIdleState = LL_TIM_OCIDLESTATE_LOW;
  TIM_OC_InitStruct.OCNIdleState = LL_TIM_OCIDLESTATE_LOW;
  LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH2, &TIM_OC_InitStruct);
  LL_TIM_OC_DisableFast(TIM1, LL_TIM_CHANNEL_CH2);
  LL_TIM_SetTriggerOutput(TIM1, LL_TIM_TRGO_RESET);
  LL_TIM_DisableMasterSlaveMode(TIM1);
  TIM_BDTRInitStruct.OSSRState = LL_TIM_OSSR_DISABLE;
  TIM_BDTRInitStruct.OSSIState = LL_TIM_OSSI_DISABLE;
  TIM_BDTRInitStruct.LockLevel = LL_TIM_LOCKLEVEL_OFF;
  TIM_BDTRInitStruct.DeadTime = 0;
  TIM_BDTRInitStruct.BreakState = LL_TIM_BREAK_DISABLE;
  TIM_BDTRInitStruct.BreakPolarity = LL_TIM_BREAK_POLARITY_HIGH;
  TIM_BDTRInitStruct.AutomaticOutput = LL_TIM_AUTOMATICOUTPUT_DISABLE;
  LL_TIM_BDTR_Init(TIM1, &TIM_BDTRInitStruct);
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  /**TIM1 GPIO Configuration  
  PA9   ------> TIM1_CH2 
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_9;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_1;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 84-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  LL_TIM_InitTypeDef TIM_InitStruct = {0};
  LL_TIM_OC_InitTypeDef TIM_OC_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM4);

  /* TIM4 DMA Init */
  
  /* TIM4_CH3 Init */
  LL_DMA_SetChannelSelection(DMA1, LL_DMA_STREAM_7, LL_DMA_CHANNEL_2);

  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_STREAM_7, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);

  LL_DMA_SetStreamPriorityLevel(DMA1, LL_DMA_STREAM_7, LL_DMA_PRIORITY_MEDIUM);

  LL_DMA_SetMode(DMA1, LL_DMA_STREAM_7, LL_DMA_MODE_CIRCULAR);

  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_STREAM_7, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_STREAM_7, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(DMA1, LL_DMA_STREAM_7, LL_DMA_PDATAALIGN_HALFWORD);

  LL_DMA_SetMemorySize(DMA1, LL_DMA_STREAM_7, LL_DMA_MDATAALIGN_WORD);

  LL_DMA_EnableFifoMode(DMA1, LL_DMA_STREAM_7);

  LL_DMA_SetFIFOThreshold(DMA1, LL_DMA_STREAM_7, LL_DMA_FIFOTHRESHOLD_FULL);

  LL_DMA_SetMemoryBurstxfer(DMA1, LL_DMA_STREAM_7, LL_DMA_MBURST_INC4);

  LL_DMA_SetPeriphBurstxfer(DMA1, LL_DMA_STREAM_7, LL_DMA_PBURST_SINGLE);

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  TIM_InitStruct.Prescaler = 0;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 1903;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  LL_TIM_Init(TIM4, &TIM_InitStruct);
  LL_TIM_OC_EnablePreload(TIM4, LL_TIM_CHANNEL_CH3);
  TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM1;
  TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.CompareValue = 0;
  TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
  LL_TIM_OC_Init(TIM4, LL_TIM_CHANNEL_CH3, &TIM_OC_InitStruct);
  LL_TIM_OC_DisableFast(TIM4, LL_TIM_CHANNEL_CH3);
  LL_TIM_SetTriggerOutput(TIM4, LL_TIM_TRGO_RESET);
  LL_TIM_DisableMasterSlaveMode(TIM4);
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
  /**TIM4 GPIO Configuration  
  PB8   ------> TIM4_CH3 
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_8;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_2;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
  /* DMA1_Stream7_IRQn interrupt configuration */
  NVIC_SetPriority(DMA1_Stream7_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),1, 0));
  NVIC_EnableIRQ(DMA1_Stream7_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  NVIC_SetPriority(DMA2_Stream2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(DMA2_Stream2_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SPI1_SEC_CS_Pin|LCD_CMD_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LCD_BL_Pin|LCD_RESET_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 PA2 PA10 
                           PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10 
                          |GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI1_SEC_CS_Pin */
  GPIO_InitStruct.Pin = SPI1_SEC_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(SPI1_SEC_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI1_CS_Pin */
  GPIO_InitStruct.Pin = SPI1_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI1_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB10 
                           PB3 PB4 PB5 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10 
                          |GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_BL_Pin LCD_RESET_Pin */
  GPIO_InitStruct.Pin = LCD_BL_Pin|LCD_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_CMD_Pin */
  GPIO_InitStruct.Pin = LCD_CMD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(LCD_CMD_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
