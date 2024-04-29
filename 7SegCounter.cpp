#include <avr/io.h>

void display(unsigned char* pointarr[]){
	unsigned char wyswietlacz[4]={0b0111,0b1011,0b1101,0b1110};
	volatile unsigned int i,wait1,wait2;
	for(wait1=0;wait1<105;++wait1){
		for(i=0;i<4;++i){
			for(unsigned int wait2=0;wait2<1000;++wait2){
				PORTB=wyswietlacz[i];
				PORTD=~(*pointarr[i]);
			}
		}
	}
}

void incerement(unsigned char* pointarr[],unsigned char arr[]){
	++pointarr[0];
	if(*pointarr[0]==0){
		pointarr[0]=arr;
		++pointarr[1];
		if(*pointarr[1]==0){
			pointarr[1]=arr;
			++pointarr[2];
			if(*pointarr[2]==0){
				pointarr[2]=arr;
				++pointarr[3];
				if(*pointarr[3]==0) pointarr[3]=arr;
			}
		}
	}
}


int main(void)
{
	unsigned char arr[11]={0x7E, 0x30, 0x6D, 0x79, 0x33, 0x5B, 0x5F, 0x70, 0x7F, 0x7B,0};
		unsigned char* pointarr[4]={arr,arr,arr,arr};
		DDRB=0xFF;
		DDRD=0xFF;
    while (1) 
    {
		display(pointarr);
		incerement(pointarr,arr);
	}
}

