#include <avr/io.h>
#include <stdlib.h>
#include <stdbool.h>
#include <util/delay.h>
void display(unsigned char* pointarr[]){
	unsigned char disp[4]={0b0111,0b1011,0b1101,0b1110};
	volatile unsigned int i,wait1,wait2;
	for(wait1=0;wait1<105;++wait1){
		for(i=0;i<4;++i){
			for(unsigned int wait2=0;wait2<1000;++wait2){
				PORTC=disp[i];
				PORTD=~(*pointarr[i]);
			}
		}
	}
}

void counter(unsigned char* pointarr[],unsigned char arr[], volatile unsigned int value){
		pointarr[3]=&arr[value/1000];
		pointarr[2]=&arr[(value/100)%10];
		pointarr[1]=&arr[(value/10)%10];
		pointarr[0]=&arr[value%10];
}

void change (unsigned char* pointarr[],unsigned char arr[], volatile unsigned int press, volatile unsigned int temp){
	static bool was_press;
	if (!(PINB & (1 << PB7)))
	{
		_delay_ms(50);
		if(!(PINB & (1<<PB7)) && !was_press) counter(pointarr,arr,press);
		else counter(pointarr,arr,temp);
		was_press=true;
	}
	else was_press = false;
}


int main(void)
{
	unsigned char arr[11]={0x7E, 0x30, 0x6D, 0x79, 0x33, 0x5B, 0x5F, 0x70, 0x7F, 0x7B,0};
	unsigned char* pointarr[4]={arr,arr,arr,arr};
	DDRC=0xFF;
	DDRD=0xFF;
	DDRB &= ~(1 << PB7);
	PORTB |= (1 << PB7);
	
	while (1)
	{
		volatile unsigned int press = 2578;
		volatile unsigned int temp = 26;
		display(pointarr);
		change(pointarr, arr, press, temp);
	}
}
