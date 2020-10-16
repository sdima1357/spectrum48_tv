#include "main.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#define READ_ID  	0x9f
#define RESET_EN 	0x66
#define RESET    	0x99
#define WRITE    	0x02
#define READ     	0x03
#define READ_FAST   0x0B
#define WRAP_TOGGLE 0xC0



extern SPI_HandleTypeDef hspi1;
uint8_t retv[128];
uint8_t tetv[128];
#define HRAM_CS_SET      SPI1_CS_GPIO_Port->BSRR = SPI1_CS_Pin;
#define HRAM_CS_RESET    SPI1_CS_GPIO_Port->BSRR = (uint32_t)SPI1_CS_Pin<<16U;

inline void SPIx_WriteF(uint8_t  Value)
{
	    *((__IO uint8_t*)&hspi1.Instance->DR) = Value;
	    while(((hspi1.Instance->SR) & SPI_FLAG_TXE) != SPI_FLAG_TXE)
	    {
		    //~ busy_counter++;
	    }
}


inline void SPIx_ReadF(uint8_t * Value)
{
	    while(((hspi1.Instance->SR) & SPI_FLAG_RXNE) != SPI_FLAG_RXNE)
	    {
	    }
	    *Value =  *((__IO uint8_t*)&hspi1.Instance->DR);
}
inline void SPIx_WriteF16(uint16_t  Value)
{
	    *((__IO uint16_t*)&hspi1.Instance->DR) = Value;
	    while(((hspi1.Instance->SR) & SPI_FLAG_TXE) != SPI_FLAG_TXE)
	    {
		    //~ busy_counter++;
	    }
}
inline void SPIx_ReadF16(uint16_t * Value)
{
	    while(((hspi1.Instance->SR) & SPI_FLAG_RXNE) != SPI_FLAG_RXNE)
	    {
		    //~ busy_counter++;
	    }
	    *Value =  *((__IO uint16_t*)&hspi1.Instance->DR);
}

void hram_write32F(uint32_t addr , void* pnt)
{
	tetv[0] = WRITE;
	tetv[1] = (addr>>16) & 0xff;
	tetv[2] = (addr>>8) & 0xff;
	tetv[3] = (addr>>0) & 0b11100000;
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port,SPI1_CS_Pin,GPIO_PIN_RESET);
	HRAM_CS_RESET;
	uint8_t dummy;
	SPIx_WriteF(tetv[0]);
	SPIx_WriteF(tetv[1]);
	SPIx_WriteF(tetv[2]);
	SPIx_WriteF(tetv[3]);

	uint8_t *bpnt = pnt;

	SPIx_WriteF(bpnt[0]);
	SPIx_WriteF(bpnt[1]);
	SPIx_WriteF(bpnt[2]);
	SPIx_WriteF(bpnt[3]);

	SPIx_WriteF(bpnt[4]);
	SPIx_WriteF(bpnt[5]);
	SPIx_WriteF(bpnt[6]);
	SPIx_WriteF(bpnt[7]);


	SPIx_WriteF(bpnt[8]);
	SPIx_WriteF(bpnt[9]);
	SPIx_WriteF(bpnt[10]);
	SPIx_WriteF(bpnt[11]);

	SPIx_WriteF(bpnt[12]);
	SPIx_WriteF(bpnt[13]);
	SPIx_WriteF(bpnt[14]);
	SPIx_WriteF(bpnt[15]);

	SPIx_WriteF(bpnt[16+0]);
	SPIx_WriteF(bpnt[16+1]);
	SPIx_WriteF(bpnt[16+2]);
	SPIx_WriteF(bpnt[16+3]);

	SPIx_WriteF(bpnt[16+4]);
	SPIx_WriteF(bpnt[16+5]);
	SPIx_WriteF(bpnt[16+6]);
	SPIx_WriteF(bpnt[16+7]);


	SPIx_WriteF(bpnt[16+8]);
	SPIx_WriteF(bpnt[16+9]);
	SPIx_WriteF(bpnt[16+10]);
	SPIx_WriteF(bpnt[16+11]);

	SPIx_WriteF(bpnt[16+12]);
	SPIx_WriteF(bpnt[16+13]);
	SPIx_WriteF(bpnt[16+14]);
	SPIx_WriteF(bpnt[16+15]);

	//while(((hspi1.Instance->SR) & SPI_FLAG_RXNE) != SPI_FLAG_RXNE){};
	SPIx_ReadF(&dummy);
	HRAM_CS_SET;

}
void hram_read32F(uint32_t addr , void* pnt)
{
	tetv[0] = READ_FAST;
	tetv[1] = (addr>>16) & 0xff;
	tetv[2] = (addr>>8) & 0xff;
	tetv[3] = (addr>>0) & 0b11100000;
	HRAM_CS_RESET;
	uint8_t dummy;
	SPIx_WriteF(tetv[0]);	SPIx_ReadF(&dummy);
	SPIx_WriteF(tetv[1]);	SPIx_ReadF(&dummy);
	SPIx_WriteF(tetv[2]);	SPIx_ReadF(&dummy);
	SPIx_WriteF(tetv[3]);   SPIx_ReadF(&dummy);
	SPIx_WriteF(dummy);		SPIx_ReadF(&dummy);
	uint8_t *bpnt = pnt;
#define k 0
	SPIx_WriteF(dummy);		SPIx_ReadF(&bpnt[k+0]);
	SPIx_WriteF(dummy);		SPIx_ReadF(&bpnt[k+1]);
	SPIx_WriteF(dummy);		SPIx_ReadF(&bpnt[k+2]);
	SPIx_WriteF(dummy);		SPIx_ReadF(&bpnt[k+3]);

	SPIx_WriteF(dummy);		SPIx_ReadF(&bpnt[k+4]);
	SPIx_WriteF(dummy);		SPIx_ReadF(&bpnt[k+5]);
	SPIx_WriteF(dummy);		SPIx_ReadF(&bpnt[k+6]);
	SPIx_WriteF(dummy);		SPIx_ReadF(&bpnt[k+7]);
#undef k
#define k 8
	SPIx_WriteF(dummy);		SPIx_ReadF(&bpnt[k+0]);
	SPIx_WriteF(dummy);		SPIx_ReadF(&bpnt[k+1]);
	SPIx_WriteF(dummy);		SPIx_ReadF(&bpnt[k+2]);
	SPIx_WriteF(dummy);		SPIx_ReadF(&bpnt[k+3]);

	SPIx_WriteF(dummy);		SPIx_ReadF(&bpnt[k+4]);
	SPIx_WriteF(dummy);		SPIx_ReadF(&bpnt[k+5]);
	SPIx_WriteF(dummy);		SPIx_ReadF(&bpnt[k+6]);
	SPIx_WriteF(dummy);		SPIx_ReadF(&bpnt[k+7]);
#undef k
#define k 16
	SPIx_WriteF(dummy);		SPIx_ReadF(&bpnt[k+0]);
	SPIx_WriteF(dummy);		SPIx_ReadF(&bpnt[k+1]);
	SPIx_WriteF(dummy);		SPIx_ReadF(&bpnt[k+2]);
	SPIx_WriteF(dummy);		SPIx_ReadF(&bpnt[k+3]);

	SPIx_WriteF(dummy);		SPIx_ReadF(&bpnt[k+4]);
	SPIx_WriteF(dummy);		SPIx_ReadF(&bpnt[k+5]);
	SPIx_WriteF(dummy);		SPIx_ReadF(&bpnt[k+6]);
	SPIx_WriteF(dummy);		SPIx_ReadF(&bpnt[k+7]);
#undef k
#define k 24
	SPIx_WriteF(dummy);		SPIx_ReadF(&bpnt[k+0]);
	SPIx_WriteF(dummy);		SPIx_ReadF(&bpnt[k+1]);
	SPIx_WriteF(dummy);		SPIx_ReadF(&bpnt[k+2]);
	SPIx_WriteF(dummy);		SPIx_ReadF(&bpnt[k+3]);

	SPIx_WriteF(dummy);		SPIx_ReadF(&bpnt[k+4]);
	SPIx_WriteF(dummy);		SPIx_ReadF(&bpnt[k+5]);
	SPIx_WriteF(dummy);		SPIx_ReadF(&bpnt[k+6]);
	SPIx_WriteF(dummy);		SPIx_ReadF(&bpnt[k+7]);
#undef k
	//uint8_t dummy;
    //SPIx_ReadF(&dummy);
/*
    while(((hspi1.Instance->SR) & SPI_FLAG_BSY) != RESET)
	{
	}
*/
	//HAL_SPI_TransmitReceive(&hspi1,tetv,retv, 4, 1000);
	//HAL_SPI_TransmitReceive(&hspi1,pnt,retv, 32, 1000);
	HRAM_CS_SET;
	//HAL_GPIO_WritePin(SPI1_CS_GPIO_Port,SPI1_CS_Pin,GPIO_PIN_SET);
}
/*
void hram_write32(uint32_t addr , void* pnt)
{
	tetv[0] = WRITE;
	tetv[1] = (addr>>16) & 0xff;
	tetv[2] = (addr>>8) & 0xff;
	tetv[3] = (addr>>0) & 0b11100000;
	//HAL_GPIO_WritePin(SPI1_CS_GPIO_Port,SPI1_CS_Pin,GPIO_PIN_RESET);
	HRAM_CS_RESET;
	HAL_SPI_Transmit(&hspi1,tetv, 4, 1000);
	HAL_SPI_Transmit(&hspi1,pnt, 32,1000);
	//HAL_SPI_Transmit_DMA(&hspi1,pnt, 32);
	//while(hspi1.State!=HAL_SPI_STATE_READY){};
	HRAM_CS_SET;
	//HAL_GPIO_WritePin(SPI1_CS_GPIO_Port,SPI1_CS_Pin,GPIO_PIN_SET);
}
void hram_read32(uint32_t addr , void* pnt)
{
	tetv[0] = READ_FAST;
	tetv[1] = (addr>>16) & 0xff;
	tetv[2]= (addr>>8) & 0xff;
	tetv[3] = (addr>>0) & 0b11100000;
	HRAM_CS_RESET;
	//HAL_GPIO_WritePin(SPI1_CS_GPIO_Port,SPI1_CS_Pin,GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1,tetv, 5, 1000);
	//HAL_SPI_Receive_DMA(&hspi1,pnt, 32);
	HAL_SPI_Receive(&hspi1,pnt, 32,1000);
	//while(hspi1.State!=HAL_SPI_STATE_READY){};
	HRAM_CS_SET;
	//HAL_GPIO_WritePin(SPI1_CS_GPIO_Port,SPI1_CS_Pin,GPIO_PIN_SET);
}
*/
#if 0
volatile flag_tx = 0;
volatile flag_rx = 0;
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
	if(hspi==&hspi1)
	{
		flag_tx = 1;
		HRAM_CS_SET;
	}
}
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
	if(hspi==&hspi1)
	{
		flag_rx = 1;
		HRAM_CS_SET;
	}
}
#endif

void hram_write(uint32_t addr , void* pnt,int BYTES)
{
	tetv[0] = WRITE;
	tetv[1] = (addr>>16)& 0xff;
	tetv[2] = (addr>>8) ;
	tetv[3] = (addr>>0) ;
	//HAL_GPIO_WritePin(SPI1_CS_GPIO_Port,SPI1_CS_Pin,GPIO_PIN_RESET);
	HRAM_CS_RESET;
	//HAL_SPI_Transmit_DMA(&hspi1,tetv, 4);
	//while(hspi1.State!=HAL_SPI_STATE_READY){};
	HAL_SPI_Transmit(&hspi1,tetv, 4, 1000);
	//flag_tx = 0;
	//flag_rx = 0;
	//HAL_SPI_Transmit_DMA(&hspi1,pnt, BYTES);
	//while(hspi1.State!=HAL_SPI_STATE_READY){};
	HAL_SPI_Transmit(&hspi1,pnt, BYTES,1000);
	//while(flag_tx==0){};
	//HAL_DMA_PollForTransfer(hspi1.hdmatx,HAL_DMA_FULL_TRANSFER,1000);
	HRAM_CS_SET;
}
void hram_read(uint32_t addr , void* pnt,int BYTES)
{
	tetv[0] = READ_FAST;
	tetv[1] = (addr>>16)& 0xff;
	tetv[2] = (addr>>8) ;
	tetv[3] = (addr>>0) ;
	HRAM_CS_RESET;
	//HAL_GPIO_WritePin(SPI1_CS_GPIO_Port,SPI1_CS_Pin,GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1,tetv, 5, 1000);
	//HAL_SPI_Transmit_DMA(&hspi1,tetv, 5);
	//while(hspi1.State!=HAL_SPI_STATE_READY){};
	//flag_tx = 0;
	//flag_rx = 0;
	//HAL_SPI_Receive_DMA(&hspi1,pnt, BYTES);
	//while(hspi1.State!=HAL_SPI_STATE_READY){};
	HAL_SPI_Receive(&hspi1,pnt, BYTES,1000);
	//while(flag_rx==0){};
	//HAL_DMA_PollForTransfer(hspi1.hdmarx,HAL_DMA_FULL_TRANSFER,1000);
	HRAM_CS_SET;
}

//uint32_t block[32/sizeof(uint32_t)];
//uint32_t block_long[1024/sizeof(uint32_t)];

#define L     10
//#define M     4
#define M     3
#define AS    23

#define LMASK ((1<<L)-1)
#define MMASK ((1<<M)-1)

// Number of cache WAYS
#define W    2

// Number of cache lines
#define MN    (1<<M)

// Number of words in cache line
#define LN     (1<<L)

#define RAM_SIZEW 	(1<<AS)
#define RAM_SIZE_MASK 	(RAM_SIZEW-1)

// 8MBytes
#define HRAM_SIZE   (RAM_SIZEW)


#define ALIGN16 __attribute__((aligned(16)))

/********************************
Index
	AS-L-H		3		5
FGS	HBIT		|MBIT	| LBIT
WD
	00000000
				000
						00000
*/
#define true  1
#define false 0
struct sPSRAM
{

	ALIGN16 uint8_t  cache_lines[MN][W][LN];
	ALIGN16 uint16_t haddress[MN][W];   // contains address >>(L+M) so uint16_t enough but 32 more safety.
	uint8_t     dirtyBit[MN][W];   // if true write access  occurred . must to be save to memory!
	uint8_t     lastWay[MN];       // [0 - Ways-1] LRU ways
	uint32_t	missesR;
	uint32_t	missesW;

}PSR;
void PSRAM_init()
{
	PSR.missesR = 0;
	PSR.missesW = 0;

	for(int k=0;k<MN;k++)
	{
		PSR.lastWay[k] = 0;
		for(int w = 0;w<W;w++)
		{
			//PSR.haddress[k][w]= k*W + w;
			//PSR.haddress[k][w]=  w;
			PSR.haddress[k][w] = 0xffff;
			//PSR.dirtyBit[k][w] = true;
			PSR.dirtyBit[k][w] = false;
		}
	}
}
inline void PSRAM_saveCache(int mLine,int hLine,int iLastWayIndex)
{
		if(PSR.dirtyBit[mLine][iLastWayIndex])
		{
			int iLineAddr = PSR.haddress[mLine][iLastWayIndex];
			if(iLineAddr!=0xffff)
			{
			int saddress = (((iLineAddr)<<M)|mLine)<<L;
#ifdef DEBUG
			printf("RD WR%x\n",saddress);
#endif
			/*
			for(int k=0;k<LN;k++)
			{
				PSR.fullArray[saddress+k] =  PSR.cache_lines[mLine][iLastWayIndex][k];
			}
			*/
				hram_write(saddress,PSR.cache_lines[mLine][iLastWayIndex],LN);
			}
		}
		// read line
		{
			int saddress = ((hLine<<M)|mLine)<<L;
#ifdef DEBUG
			printf("RD RD%x\n",saddress);
#endif
			/*
			for(int k=0;k<LN;k++)
			{
				PSR.cache_lines[mLine][iLastWayIndex][k] = PSR.fullArray[saddress+k] ;
			}
			*/
			hram_read(saddress,PSR.cache_lines[mLine][iLastWayIndex],LN);
			PSR.haddress[mLine][iLastWayIndex] = hLine; // no dirty
			PSR.lastWay[mLine] = iLastWayIndex;
			PSR.dirtyBit[mLine][iLastWayIndex] = false;
		}
}
inline uint8_t PSRAM_read8(uint32_t address)
{
	//address &= RAM_SIZE_MASK;
	int mLine = (address>>L)&MMASK;
	int hLine  = (address>>L)>>M;

	int iLastWayIndex = PSR.lastWay[mLine];
	//int ww = 0;
	for(int ww=0; ww<W ; ww++)
	{
		int kIndex = (iLastWayIndex+ww)&(W-1);
		if(hLine == PSR.haddress[mLine][kIndex])
		{
			PSR.lastWay[mLine] = kIndex;
			return PSR.cache_lines[mLine][kIndex][address&LMASK];
		}
		//ww++;
	}

	//PSR.missesR++;

	iLastWayIndex = (iLastWayIndex+1)&(W-1);

	PSRAM_saveCache(mLine,hLine,iLastWayIndex);

	return PSR.cache_lines[mLine][iLastWayIndex][address&LMASK] ;
}
//#include <assert.h>

uint8_t PSRAM_read2468(uint32_t address)
{
	//assert(W==4);

	//address &= RAM_SIZE_MASK;
	int mLine = (address>>L)&MMASK;
	int hLine  = (address>>L)>>M;

	int iLastWayIndex = PSR.lastWay[mLine];

	//for(int ww = 0;ww<W;ww++)
	{
		if(hLine == PSR.haddress[mLine][(iLastWayIndex+0)&(W-1)])
		{
			PSR.lastWay[mLine] = (iLastWayIndex+0)&(W-1);
			return PSR.cache_lines[mLine][(iLastWayIndex+0)&(W-1)][address&LMASK];
		}
		if(hLine == PSR.haddress[mLine][(iLastWayIndex+1)&(W-1)])
		{
			PSR.lastWay[mLine] = (iLastWayIndex+1)&(W-1);
			return PSR.cache_lines[mLine][(iLastWayIndex+1)&(W-1)][address&LMASK];
		}
#if (W>=4)
		if(hLine == PSR.haddress[mLine][(iLastWayIndex+2)&(W-1)])
		{
			PSR.lastWay[mLine] = (iLastWayIndex+2)&(W-1);
			return PSR.cache_lines[mLine][(iLastWayIndex+2)&(W-1)][address&LMASK];
		}
		if(hLine == PSR.haddress[mLine][(iLastWayIndex+3)&(W-1)])
		{
			PSR.lastWay[mLine] = (iLastWayIndex+3)&(W-1);
			return PSR.cache_lines[mLine][(iLastWayIndex+3)&(W-1)][address&LMASK];
		}
#endif
#if (W>=6)
		if(hLine == PSR.haddress[mLine][(iLastWayIndex+4)&(W-1)])
		{
			PSR.lastWay[mLine] = (iLastWayIndex+4)&(W-1);
			return PSR.cache_lines[mLine][(iLastWayIndex+4)&(W-1)][address&LMASK];
		}
		if(hLine == PSR.haddress[mLine][(iLastWayIndex+5)&(W-1)])
		{
			PSR.lastWay[mLine] = (iLastWayIndex+5)&(W-1);
			return PSR.cache_lines[mLine][(iLastWayIndex+5)&(W-1)][address&LMASK];
		}
#endif
#if (W>=8)
		if(hLine == PSR.haddress[mLine][(iLastWayIndex+6)&(W-1)])
		{
			PSR.lastWay[mLine] = (iLastWayIndex+6)&(W-1);
			return PSR.cache_lines[mLine][(iLastWayIndex+6)&(W-1)][address&LMASK];
		}
		if(hLine == PSR.haddress[mLine][(iLastWayIndex+7)&(W-1)])
		{
			PSR.lastWay[mLine] = (iLastWayIndex+7)&(W-1);
			return PSR.cache_lines[mLine][(iLastWayIndex+7)&(W-1)][address&LMASK];
		}
#endif

	}

	//PSR.missesR++;

	iLastWayIndex = (PSR.lastWay[mLine]+1)&(W-1);

	PSRAM_saveCache(mLine,hLine,iLastWayIndex);
	return PSR.cache_lines[mLine][iLastWayIndex][address&LMASK] ;
}

inline void PSRAM_write8(uint32_t address,uint8_t data)
{
	//address &= RAM_SIZE_MASK;

	int mLine = (address>>L)&MMASK;
	int hLine  = (address>>L)>>M;

	int iLastWayIndex = PSR.lastWay[mLine];
	int ww = 0;
	while(ww<W)
	{
		if(hLine == PSR.haddress[mLine][(iLastWayIndex+ww)&(W-1)])
		{
			PSR.lastWay[mLine] = (iLastWayIndex+ww)&(W-1);
			PSR.dirtyBit[mLine][(iLastWayIndex+ww)&(W-1)] = true;
			PSR.cache_lines[mLine][(iLastWayIndex+ww)&(W-1)][address&LMASK] = data;
			return;
		}
		ww++;
	}

	//PSR.missesW++;
	iLastWayIndex = (iLastWayIndex+1)&(W-1);
	PSRAM_saveCache(mLine,hLine,iLastWayIndex);
	PSR.dirtyBit[mLine][iLastWayIndex] = true;
	PSR.cache_lines[mLine][iLastWayIndex][address&LMASK] = data;
}

uint16_t PSRAM_read16(uint32_t address)
{
	//address &= RAM_SIZE_MASK;
	if(address&1)
	{
		return PSRAM_read8(address)*PSRAM_read8(address+1)*256;
	}
	int mLine = (address>>L)&MMASK;
	int hLine  = (address>>L)>>M;

	int iLastWayIndex = PSR.lastWay[mLine];
	int ww = 0;
	while(ww<W)
	{
		if(hLine == PSR.haddress[mLine][(iLastWayIndex+ww)&(W-1)])
		{
			PSR.lastWay[mLine] = (iLastWayIndex+ww)&(W-1);
			return *(uint16_t*)&PSR.cache_lines[mLine][(iLastWayIndex+ww)&(W-1)][address&LMASK];
		}
		ww++;
	}

	//PSR.missesR++;

	iLastWayIndex = (iLastWayIndex+1)&(W-1);

	PSRAM_saveCache(mLine,hLine,iLastWayIndex);
	PSR.lastWay[mLine] = iLastWayIndex;
	return *(uint16_t*)&PSR.cache_lines[mLine][(iLastWayIndex+ww)&(W-1)][address&LMASK];
}


inline uint8_t hash(uint8_t num)
{
	//return (((num*3)>>8)+num+12)&0xff;
	return num;
}


int  init_ram()
{
	memset(tetv,0,128);
	memset(retv,0xff,128);

	printf("Hello!!!!!!!!!!!!!!!\n");
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port,SPI1_CS_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port,SPI1_CS_Pin,GPIO_PIN_SET);
	{
	//retv = SPI_transfer1(READ_ID);
	tetv[0] = RESET_EN;

	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port,SPI1_CS_Pin,GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&hspi1,tetv,retv, 1, 1000);
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port,SPI1_CS_Pin,GPIO_PIN_SET);

	tetv[0] = RESET;

	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port,SPI1_CS_Pin,GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&hspi1,tetv,retv, 1, 1000);
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port,SPI1_CS_Pin,GPIO_PIN_SET);


	//tetv[0] = WRAP_TOGGLE;
	//HAL_GPIO_WritePin(SPI1_CS_GPIO_Port,SPI1_CS_Pin,GPIO_PIN_RESET);
	//HAL_SPI_TransmitReceive(&hspi1,tetv,retv, 1, 1000);
	//HAL_GPIO_WritePin(SPI1_CS_GPIO_Port,SPI1_CS_Pin,GPIO_PIN_SET);


	//HAL_GPIO_WritePin(SPI1_CS_GPIO_Port,SPI1_CS_Pin,GPIO_PIN_SET);
	for(int i=0;i<10;i++)
		printf("%02x ",retv[i]);
	printf("\n");
	}

	for(int k=0;k<3;k++)
	{
	printf("\n");
	//retv = SPI_transfer1(READ_ID);
	tetv[0] = READ_ID;

	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port,SPI1_CS_Pin,GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&hspi1,tetv,retv, 10, 1000);
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port,SPI1_CS_Pin,GPIO_PIN_SET);

	for(int i=0;i<10;i++)
		printf("%02x ",retv[i]);
	printf("\n");

	}
	int address = 0xfff00;
	tetv[0] = WRITE;
	tetv[1] = (address>>16) & 0xff;
	tetv[2] = (address>>8) & 0xff;
	tetv[3] = (address>>0) & 0xff;
	for(int i=0;i<64;i++)
	{
		tetv[4+i] = i;
	}
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port,SPI1_CS_Pin,GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&hspi1,tetv,retv, 4, 1000);
	HAL_SPI_TransmitReceive(&hspi1,tetv+4,retv, 64, 1000);
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port,SPI1_CS_Pin,GPIO_PIN_SET);

	for(int k=0;k<2;k++)
	{

		tetv[0] = READ;
		tetv[1] = (address>>16) & 0xff;
		tetv[2] = (address>>8) & 0xff;
		tetv[3] = (address>>0) & 0xff;
		memset(retv,0xff,128);
		HAL_GPIO_WritePin(SPI1_CS_GPIO_Port,SPI1_CS_Pin,GPIO_PIN_RESET);
		HAL_SPI_TransmitReceive(&hspi1,tetv,retv, 4, 1000);
		HAL_SPI_TransmitReceive(&hspi1,tetv,retv, 64, 1000);
		HAL_GPIO_WritePin(SPI1_CS_GPIO_Port,SPI1_CS_Pin,GPIO_PIN_SET);
		for(int i=0;i<64+5;i++)
		{
			printf("%02x ",retv[i]);
		}
		printf("\n");
		tetv[0] = READ_FAST;
		tetv[1] = (address>>16) & 0xff;
		tetv[2] = (address>>8) & 0xff;
		tetv[3] = (address>>0) & 0xff;
		memset(retv,0xff,128);
		HAL_GPIO_WritePin(SPI1_CS_GPIO_Port,SPI1_CS_Pin,GPIO_PIN_RESET);
		HAL_SPI_TransmitReceive(&hspi1,tetv,retv, 5, 1000);
		HAL_SPI_TransmitReceive(&hspi1,tetv,retv, 64, 1000);
		HAL_GPIO_WritePin(SPI1_CS_GPIO_Port,SPI1_CS_Pin,GPIO_PIN_SET);
		for(int i=0;i<64+5;i++)
		{
			printf("%02x ",retv[i]);
		}
		printf("\n");


	}
		int l;

#if 0
	{
		uint32_t ticks = HAL_GetTick();
		//hram_write32(0,block);
		for(int k=0;k<HRAM_SIZE/32;k++)
		//for(int k=0;k<0x3;k++)
		{
			for(l=0;l<8;l++)
			{
				block[l] = l+k*8+7;
			}
			hram_write32(k*32,block);
		}
		ticks = HAL_GetTick()-ticks;
		printf("write32 take %d \n",ticks);
	}
	{
		uint32_t ticks = HAL_GetTick();
		int flagOk = 0;
		hram_read32(0,block);
		for(int k=0;k<HRAM_SIZE/32;k++)
		//for(int k=0;k<0x3;k++)
		{
			hram_read32(k*32,block);
			for(l=0;l<8;l++)
			{
				if(block[l] != l+k*8+7)
				{
					if(!flagOk)
					{
						printf("block[%02d]=%02x %x\n",l+k*8,block[l],k);
						flagOk = k+1;
					}
				}

			}
		}
		ticks = HAL_GetTick()-ticks;
		printf("read32 take %d Ok = %d bytes =%d\n",ticks,flagOk,HRAM_SIZE/32*32);
	}
	{
		uint32_t ticks = HAL_GetTick();
		//hram_write32(0,block);
		for(int k=0;k<(1<<23)/1024;k++)
		//for(int k=0;k<0x3;k++)
		{
			for(l=0;l<256;l++)
			{
				block_long[l] = l+k*256+13;
			}
			hram_write(k*1024,block_long,1024);
		}
		ticks = HAL_GetTick()-ticks;
		printf("write1024 take %d \n",ticks);
	}
	{
		uint32_t ticks = HAL_GetTick();
		int flagOk = 0;
		hram_read32(0,block);
		for(int k=0;k<HRAM_SIZE/1024;k++)
		//for(int k=0;k<0x3;k++)
		{
			hram_read(k*1024,block_long,1024);
			for(l=0;l<256;l++)
			{
				if(block_long[l] != l+k*256+13)
				{
					//printf("block[%02d]=%02x\n",l+k*8,block[l]);
					if(!flagOk)
	{
						printf("block_long[%02d]=%02x k=%x l=%x\n",l+k*256+13,block_long[l],k,l);
						flagOk = k+1;
			}
				}

			}
		}
		ticks = HAL_GetTick()-ticks;
		printf("read1024 take %d Ok = %d bytes =%d\n",ticks,flagOk,HRAM_SIZE/1024*1024);
	}
#endif
	PSRAM_init();
	printf("Asizeof(PSR)=%d\n",sizeof(PSR));
	printf("LMASK = %x\n",LMASK);
	printf("MMASK = %x\n",MMASK);
	printf("MN = %x\n",MN);
	printf("LN = %x\n",LN);
	{
		uint32_t ticks = HAL_GetTick();
		uint32_t summ = 0;
		for(int kk=0;kk<RAM_SIZEW;kk++)
		{
			summ+=hash(kk);
		}
		ticks = HAL_GetTick()-ticks;
		printf("summ in %d %x\n",ticks,summ);
	}
	{
		uint32_t ticks = HAL_GetTick();
		for(int kk=0;kk<RAM_SIZEW;kk++)
		{
			PSRAM_write8(kk,hash(kk));

			if(0)
			{
			int tda = (kk+(1<<(M+L)))&(RAM_SIZEW-1);
			PSRAM_write8(tda,hash(tda));
			}
			if(0)
			{
			int tda = (kk+(2<<(M+L)))&(RAM_SIZEW-1);
			PSRAM_write8(tda,hash(tda));
			}
			//ps->write(kk+(2<<(M+L)),hash(kk));
			//ps->write(kk+(3<<(M+L)),hash(kk));
			//ps->write(kk+(4<<(M+L)),hash(kk));
		}
		ticks = HAL_GetTick()-ticks;
		printf("WriteOK taken %d\n",ticks);
	}
	{
		uint32_t ticks = HAL_GetTick();
		uint32_t summ = 0;
		for(int kk=0;kk<RAM_SIZEW;kk++)
		{
			summ+=PSRAM_read8(kk);
		}
		ticks = HAL_GetTick()-ticks;
		printf("readOK in %d %x\n",ticks,summ);
	}
	if(1){
		uint32_t ticks = HAL_GetTick();
		uint32_t summ = 0;
		for(uint32_t kk=0;kk<RAM_SIZEW;kk++)
			{
			summ+=PSRAM_read8(kk&0xfffu);
		}
		ticks = HAL_GetTick()-ticks;
		printf("readOK mini in %d %x\n",ticks,summ);
	}
	if(1){
		uint32_t ticks = HAL_GetTick();
		for(int kk=0;kk<(RAM_SIZEW)-1;kk++)
		{
			uint8_t rk = PSRAM_read8(kk);
			if(rk!=hash(kk))
			{
				printf("Error in %x %x\n",kk,rk);
				return 0;
			}
			rk = PSRAM_read8(kk+1);
			if(rk!=hash(kk+1))
			{
				printf("Error in %x %x\n",kk+1,rk);
				return 0;
			}
		}
		ticks = HAL_GetTick()-ticks;
		printf("readOK in %d\n",ticks);
	}
	{
		uint32_t ticks = HAL_GetTick();
		for(int kk=0;kk<RAM_SIZEW;kk++)
		{
			uint8_t rk = PSRAM_read2468(kk);
			if(rk!=hash(kk))
			{
				printf("Error in %x %x\n",kk,rk);
				return 0;
			}
		}
		ticks = HAL_GetTick()-ticks;
		printf("readOK in %d\n",ticks);
	}
	{
		uint32_t ticks = HAL_GetTick();
		uint32_t summ = 0;
		for(int kk=0;kk<RAM_SIZEW/2;kk++)
		{
			uint16_t rk = PSRAM_read16(kk*2);
			summ+= rk;
		}
		ticks = HAL_GetTick()-ticks;
		printf("readOK in %d summ= %x\n",ticks,summ);
	}
#if 0
	{
		uint32_t ticks = HAL_GetTick();
		for(int kk=0;kk<RAM_SIZEW;kk+=2)
		{
			if((hash(kk)+hash(kk+1)*256)!=PSRAM_read16(kk))
			{
				printf("Error in %x \n",kk);
			}
		}
		ticks = HAL_GetTick()-ticks;
		printf("read16OK in %d\n",ticks);
	}
#endif
	printf("missesR = %d\n",PSR.missesR);
	printf("missesW = %d\n",PSR.missesW);


#if 0
write take 2963
read take 4083 Ok = 1
write take 2960
read take 4118 Ok = 1
#endif
	return 0;
}

