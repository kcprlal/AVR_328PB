#include <avr/io.h>
#include <stdlib.h>
#include <stdbool.h>
#define F_CPU 8000000UL
#include <util/delay.h>

#define MOSI 3
#define SS0 2
#define MISO 4
#define CLOCK 5

#define BMP280_PRESS_MSB  ((uint8_t)0xf7)
#define BMP280_PRESS_LSB  ((uint8_t)0xf8)
#define BMP280_PRESS_XLSB ((uint8_t)0xf9)
#define BMP280_TEMP_MSB   ((uint8_t)0xfa)
#define BMP280_TEMP_LSB   ((uint8_t)0xfb)
#define BMP280_TEMP_XLSB  ((uint8_t)0xfc)

uint16_t dig_T1, dig_P1;
int16_t dig_T2, dig_T3, dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
int32_t t_fine;

void display(unsigned char* pointarr[]){
	unsigned char disp[4] = {0b0111, 0b1011, 0b1101, 0b1110};
	volatile unsigned int i, wait1, wait2;
	for (wait1 = 0; wait1 < 105; ++wait1) {
		for (i = 0; i < 4; ++i) {
			for (wait2 = 0; wait2 < 1000; ++wait2) {
				PORTC = disp[i];
				PORTD = ~(*pointarr[i]);
			}
		}
	}
}

void counter(unsigned char* pointarr[], unsigned char arr[], volatile unsigned int value){
	pointarr[3] = &arr[value / 1000];
	pointarr[2] = &arr[(value / 100) % 10];
	pointarr[1] = &arr[(value / 10) % 10];
	pointarr[0] = &arr[value % 10];
}

void change(unsigned char* pointarr[], unsigned char arr[], volatile unsigned int press, volatile unsigned int temp){
	static bool was_press;
	if (!(PINB & (1 << PB7))) {
		_delay_ms(50);
		if (!(PINB & (1 << PB7)) && !was_press)
		counter(pointarr, arr, press);
		else
		counter(pointarr, arr, temp);
		was_press = true;
		} else {
		was_press = false;
	}
}

static int32_t bmp280_temp32_compensate(int32_t adc_T){
	int32_t var1, var2, T;
	var1 = ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) * ((int32_t)dig_T2)) >> 11;
	var2 = (((((adc_T >> 4) - ((int32_t)dig_T1)) * ((adc_T >> 4) - ((int32_t)dig_T1))) >> 12) * ((int32_t)dig_T3)) >> 14;
	t_fine = var1 + var2;
	T = (t_fine * 5 + 128) >> 8;
	return T;
}

static uint32_t bmp280_press64_compensate(int32_t adc_P){
	int64_t var1, var2, p;
	var1 = ((int64_t)t_fine) - 128000;
	var2 = var1 * var1 * (int64_t)dig_P6;
	var2 = var2 + ((var1 * (int64_t)dig_P5) << 17);
	var2 = var2 + (((int64_t)dig_P4) << 35);
	var1 = ((var1 * var1 * (int64_t)dig_P3) >> 8) + ((var1 * (int64_t)dig_P2) << 12);
	var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)dig_P1) >> 33;
	if (var1 == 0) {
		return 0;
	}
	p = 1048576 - adc_P;
	p = (((p << 31) - var2) * 3125) / var1;
	var1 = (((int64_t)dig_P9) * (p >> 13)) >> 25;
	var2 = (((int64_t)dig_P8) * p) >> 19;
	p = ((p + var1 + var2) >> 8) + (((int64_t)dig_P7) << 4);
	return (uint32_t)p/25600;
}

uint8_t SPI_transfer(uint8_t data) {
	SPDR0 = data;
	while (!(SPSR0 & (1 << SPIF)));
	return SPDR0;
}

int bmp280_readreg(int reg){
	int8_t val;
	PORTB &= ~(1 << SS0);
	SPI_transfer(reg | 0x80);
	val = SPI_transfer(0x00);
	PORTB |= (1 << SS0);
	return val;
}

void bmp280_writereg(uint8_t reg, uint8_t value) {
    PORTB &= ~(1 << SS0); 
    SPI_transfer(reg & ~0x80); 
    SPI_transfer(value); 
    PORTB |= (1 << SS0); 
}

void SPI_Init(void) {
	DDRB &= ~(1 << MISO);
	DDRB |= (1 << MOSI) | (1 << CLOCK) | (1 << SS0);
	SPCR0 = (1 << SPE) | (1 << MSTR) | (1 << SPR0);
	SPCR0 &= ~(1 << SPR1);
	SPCR0 &= ~(0b00 << CPHA);
	PRR0 &= ~(1 << PRSPI0);
	PRR1 &= ~(1 << PRSPI1);
}

void bmp280_init(void) {
    bmp280_writereg(0xF4, 0b10110111); 
    _delay_ms(100); 
}

void bmp280_readcalibs(void){
	dig_T1 = bmp280_readreg(0x88) | (bmp280_readreg(0x89) << 8);
	dig_T2 = bmp280_readreg(0x8a) | (bmp280_readreg(0x8b) << 8);
	dig_T3 = bmp280_readreg(0x8c) | (bmp280_readreg(0x8d) << 8);

	dig_P1 = bmp280_readreg(0x8e) | (bmp280_readreg(0x8f) << 8);
	dig_P2 = bmp280_readreg(0x90) | (bmp280_readreg(0x91) << 8);
	dig_P3 = bmp280_readreg(0x92) | (bmp280_readreg(0x93) << 8);
	dig_P4 = bmp280_readreg(0x94) | (bmp280_readreg(0x95) << 8);
	dig_P5 = bmp280_readreg(0x96) | (bmp280_readreg(0x97) << 8);
	dig_P6 = bmp280_readreg(0x98) | (bmp280_readreg(0x99) << 8);
	dig_P7 = bmp280_readreg(0x9a) | (bmp280_readreg(0x9b) << 8);
	dig_P8 = bmp280_readreg(0x9c) | (bmp280_readreg(0x9d) << 8);
	dig_P9 = bmp280_readreg(0x9e) | (bmp280_readreg(0x9f) << 8);
}

read_pressure_and_temperature(volatile int32_t* pressure,volatile int32_t* temperature) {
	uint8_t msb, lsb, xlsb;

	msb = bmp280_readreg(BMP280_TEMP_MSB);
	lsb = bmp280_readreg(BMP280_TEMP_LSB);
	xlsb = bmp280_readreg(BMP280_TEMP_XLSB);
	*temperature = (msb << 12) | (lsb << 4) | (xlsb >> 4);

	msb = bmp280_readreg(BMP280_PRESS_MSB);
	lsb = bmp280_readreg(BMP280_PRESS_LSB);
	xlsb = bmp280_readreg(BMP280_PRESS_XLSB);
	*pressure = (msb << 12) | (lsb << 4) | (xlsb >> 4);
}

int main(void) {
	unsigned char arr[11] = {0x7E, 0x30, 0x6D, 0x79, 0x33, 0x5B, 0x5F, 0x70, 0x7F, 0x7B, 0};
	unsigned char* pointarr[4] = {arr, arr, arr, arr};
	CLKPR |=(1<<CLKPS0);
	SPI_Init();
	bmp280_init();
	bmp280_readcalibs();
	DDRD = 0xff;
	DDRC = 0xff;
	DDRB &= ~(1 << 7);
	PORTB |= (1 << 7);
	PORTB &= ~(1 << 2);
	volatile int32_t adc_T = 0;
	volatile int32_t adc_P = 0;
	while (1) {
		read_pressure_and_temperature(&adc_P, &adc_T);
		volatile uint32_t press = bmp280_press64_compensate(adc_P);
		volatile int32_t temp = bmp280_temp32_compensate(adc_T);
		change(pointarr, arr, press, temp);
		display(pointarr);
	}
}
