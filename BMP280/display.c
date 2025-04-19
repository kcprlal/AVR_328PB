#include <avr/io.h>
#include <util/delay.h>
#include <stdbool.h>

bool minus = false;

void display(unsigned char* pointarr[]) {
    unsigned char disp[4] = {0b0111, 0b1011, 0b1101, 0b1110};
    volatile unsigned int i, wait1, wait2;
    for (wait1 = 0; wait1 < 105; ++wait1) {
        for (i = 0; i < 4; ++i) {
            for (wait2 = 0; wait2 < 1000; ++wait2) {
                PORTC = disp[i];
                PORTD = ~(*pointarr[i]);
                if (PINE & (1 << 1) && i == 2) PORTD &= ~(1 << 7);
            }
        }
    }
}

void counter(unsigned char* pointarr[], unsigned char arr[], volatile int32_t value) {
    if (value < 0) {
        minus = true;
        value *= -1;
    } else {
        minus = false;
    }

    pointarr[3] = &arr[value / 1000];
    pointarr[2] = &arr[(value / 100) % 10];
    pointarr[1] = &arr[(value / 10) % 10];
    pointarr[0] = &arr[value % 10];
}

void change(unsigned char* pointarr[], unsigned char arr[], volatile uint32_t press, volatile int32_t temp, volatile int32_t alt) {
    static bool was_press = false;
    static uint8_t state = 0;

    if (!(PINB & (1 << 7))) {
        _delay_ms(50);
        if (!(PINB & (1 << 7)) && !was_press) {
            state = (state + 1) % 3;
            was_press = true;
        }
    } else {
        was_press = false;
    }

    switch (state) {
        case 0:
            counter(pointarr, arr, press);
            PORTE = 0b001;
            break;
        case 1:
            counter(pointarr, arr, temp);
            PORTE = 0b010;
            break;
        case 2:
            counter(pointarr, arr, alt);
            PORTE = 0b100;
            break;
    }
}
