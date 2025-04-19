#ifndef BMP280_H
#define BMP280_H

#include <stdint.h>

void SPI_init(void);
void bmp280_init(void);
void bmp280_readcalibs(void);
void bmp280_readreg(uint8_t reg, void *reg_data, uint8_t len);
void bmp280_writereg(uint8_t reg, uint8_t value);
int32_t bmp280_temp32_compensate();
uint32_t bmp280_press64_compensate();
uint32_t bmp280_altitude(uint32_t press);
void read_pressure_and_temperature(volatile int32_t* pressure, volatile int32_t* temperature);

#endif
