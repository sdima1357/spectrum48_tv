#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
uint16_t rev16(uint16_t  prm)
{
	return  (prm>>8)| ((prm&0xff)<<8);
}
uint16_t color_convertRGB_to161(uint8_t Rc,uint8_t Gc,uint8_t Bc)
{

	return rev16(((Rc>>3)<<11)|((Gc>>2)<<5)|(Bc>>3));
}

uint16_t  BKLC[512][2];
void initTbl()
{
	for(int k=0;k<512;k++)
	{
		uint8_t attr = k&0xff;

		//uint8_t attr  = ATTR_RAM[tileAddr];
		uint8_t Br    = !!(attr&64);
		uint8_t scale = (Br?0xff:0xd7);

		uint8_t RInc= scale*!!(attr&2);
		uint8_t GInc= scale*!!(attr&4);
		uint8_t BInc= scale*!!(attr&1);

		uint8_t Rpap= scale*!!(attr&16);
		uint8_t Gpap= scale*!!(attr&32);
		uint8_t Bpap= scale*!!(attr&8);
		if( (attr&128)&&(k&0x100))
		{
			RInc= scale*!!(attr&16);
			GInc= scale*!!(attr&32);
			BInc= scale*!!(attr&8);

			Rpap= scale*!!(attr&2);
			Gpap= scale*!!(attr&4);
			Bpap= scale*!!(attr&1);
		}
		for(int bt=0;bt<2;bt++)
		{
			int Rc = (bt?(RInc):(Rpap));
			int Gc = (bt?(GInc):(Gpap));
			int Bc = (bt?(BInc):(Bpap));
			BKLC[k][bt] = color_convertRGB_to161(Rc,Gc,Bc);
		}
	}
}
int main()
{
	initTbl();
	printf("#include <stdint.h> \nconst uint16_t BKLC[512][2] = {\n");
	for(int k= 0;k<512;k++)
	{
		printf("{0x%08x,0x%08x}",BKLC[k][0],BKLC[k][1]);
		if(k!=511)
		{
			printf(",\n");
		}
		else
			printf("\n };");
	}
}
