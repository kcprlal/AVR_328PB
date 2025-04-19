#include <avr/io.h>
#include <util/delay.h>
#include "bmp280.h"
#include "display.h"

unsigned char arr[11] = {0x7E, 0x30, 0x6D, 0x79, 0x33, 0x5B, 0x5F, 0x70, 0x7F, 0x7B, 0};
unsigned char* pointarr[4] = {arr, arr, arr, arr};

int main(void) {
    CLKPR |= (1 << CLKPS0);
    DDRD = 0xff;
    DDRC = 0xff;
    DDRE = 0xff;
    DDRB &= ~(1 << 7);
    PORTB |= (1 << 7);

    SPI_init();
    bmp280_init();
    bmp280_readcalibs();

    volatile int32_t adc_T = 0;
    volatile int32_t adc_P = 0;

    while (1) {
        read_pressure_and_temperature(&adc_P, &adc_T);
        volatile uint32_t press = bmp280_press64_compensate(adc_P);
        volatile int32_t temp = bmp280_temp32_compensate(adc_T);
        volatile int32_t alt = bmp280_altitude(press);
        change(pointarr, arr, press, temp, alt);
        if (minus) PORTE |= (1 << 3);
        else PORTE &= ~(1 << 3);
        display(pointarr);
    }
}
