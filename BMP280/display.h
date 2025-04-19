#ifndef DISPLAY_H
#define DISPLAY_H

#include <stdint.h>
#include <stdbool.h>

extern bool minus;

void display(unsigned char* pointarr[]);
void counter(unsigned char* pointarr[], unsigned char arr[], volatile int32_t value);
void change(unsigned char* pointarr[], unsigned char arr[], volatile uint32_t press, volatile int32_t temp, volatile int32_t alt);

#endif
