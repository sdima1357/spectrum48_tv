#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#define MAX_LEVEL  7
#define ZERO_LEVEL 0
#define BASE_LEVEL 3
#define SYNC_LEVEL 0 
const int tables[] = {0,1,1,2,1,2,2,3};
uint8_t  BKL0a[512];
uint32_t BKL1a[512][4];
void initTbl()
{
	for(int k=0;k<512;k++)
	{
		uint8_t attr = k&0xff;
		int  scale;

		int c10 = tables[attr&7];
		int c11 = tables[(attr>>3)&7];

		scale = (((attr&64)?0xff:0xA0)*MAX_LEVEL/3)>>8;

		uint16_t CLR0 = scale*(c11)+BASE_LEVEL;
		uint16_t CLR1 = scale*(c10)+BASE_LEVEL;
		BKL0a[k] = CLR1*16+CLR0;
		if((attr&128)&&(k&0x100))
		{
			BKL0a[k] = CLR0*16+CLR1;
		}
	}
	for(int bs=0;bs<4;bs++)
	{
		for(int k=0;k<512;k++)
		{
			uint16_t CLR0B = BKL0a[k];
			uint16_t CLR0  = CLR0B&0xf;
			uint16_t CLR1  = CLR0B>>4;
			BKL1a[k][bs] = (((bs&0x01)?CLR1:CLR0)<<16)|((bs&0x02)?CLR1:CLR0);
		}
	}
}
int main()
{
	initTbl();
	printf("#include <stdint.h> \nconst uint32_t BKL1a[512][4] = {\n");
	for(int k= 0;k<512;k++)
	{
		printf("{0x%08x,0x%08x,0x%08x,0x%08x}",BKL1a[k][0],BKL1a[k][1],BKL1a[k][2],BKL1a[k][3] );
		if(k!=511)
		{
			printf(",\n");
		}
		else
			printf("\n };");
	}
}
