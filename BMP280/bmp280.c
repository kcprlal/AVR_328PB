#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>
#include <math.h>
#include "bmp280.h"

// Definicje pinów SPI i BMP280
#define SS0   2
#define MOSI  3
#define MISO  4
#define CLOCK 5

#define BMP280_PRESS_MSB   ((uint8_t)0xf7)
#define BMP280_PRESS_LSB   ((uint8_t)0xf8)
#define BMP280_PRESS_XLSB  ((uint8_t)0xf9)
#define BMP280_TEMP_MSB    ((uint8_t)0xfa)
#define BMP280_TEMP_LSB    ((uint8_t)0xfb)
#define BMP280_TEMP_XLSB   ((uint8_t)0xfc)

// Kalibracyjne rejestry
uint16_t dig_T1, dig_P1;
int16_t dig_T2, dig_T3;
int16_t dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;

int32_t t_fine;

void SPI_init(void) {
    DDRB &= ~(1 << MISO);
    DDRB |= (1 << MOSI) | (1 << CLOCK) | (1 << SS0);
    SPCR0 = (1 << SPE) | (1 << MSTR) | (1 << SPR0);
    SPCR0 &= ~(1 << SPR1);
    SPCR0 &= ~(0b00 << CPHA);
    PRR0 &= ~(1 << PRSPI0);
    PRR1 &= ~(1 << PRSPI1);
}

uint8_t SPI_transfer(uint8_t data) {
    SPDR0 = data;
    while (!(SPSR0 & (1 << SPIF)));
    return SPDR0;
}

void bmp280_readreg(uint8_t reg, void *reg_data, uint8_t len){
    PORTB &= ~(1 << SS0);
    SPI_transfer(reg | 0x80);
    while(len--) {
        *(uint8_t*)reg_data = SPI_transfer(0x00);
        reg_data = (uint8_t*)reg_data + 1;
    }
    PORTB |= (1 << SS0);
}

void bmp280_writereg(uint8_t reg, uint8_t value) {
    PORTB &= ~(1 << SS0);
    SPI_transfer(reg & ~0x80);
    _delay_ms(10);
    SPI_transfer(value);
    _delay_ms(10);
    PORTB |= (1 << SS0);
}

void bmp280_init(void) {
    bmp280_writereg(0xF4, 0b10110111); // Normal mode, oversampling
    _delay_ms(100);
}

void bmp280_readcalibs(void) {
    bmp280_readreg(0x88, &dig_T1, sizeof(dig_T1));
    bmp280_readreg(0x8A, &dig_T2, sizeof(dig_T2));
    bmp280_readreg(0x8C, &dig_T3, sizeof(dig_T3));

    bmp280_readreg(0x8E, &dig_P1, sizeof(dig_P1));
    bmp280_readreg(0x90, &dig_P2, sizeof(dig_P2));
    bmp280_readreg(0x92, &dig_P3, sizeof(dig_P3));
    bmp280_readreg(0x94, &dig_P4, sizeof(dig_P4));
    bmp280_readreg(0x96, &dig_P5, sizeof(dig_P5));
    bmp280_readreg(0x98, &dig_P6, sizeof(dig_P6));
    bmp280_readreg(0x9A, &dig_P7, sizeof(dig_P7));
    bmp280_readreg(0x9C, &dig_P8, sizeof(dig_P8));
    bmp280_readreg(0x9E, &dig_P9, sizeof(dig_P9));
}

int32_t bmp280_temp32_compensate() {
    int32_t var1, var2, T;
    uint8_t tempr[3];
    bmp280_readreg(BMP280_TEMP_MSB, tempr, 3);
    int32_t adc_T = ((int32_t)(tempr[0]) << 12) | ((int32_t)(tempr[1]) << 4) | ((int32_t)(tempr[2]) >> 4);

    var1 = ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) * ((int32_t)dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((int32_t)dig_T1)) *
              ((adc_T >> 4) - ((int32_t)dig_T1))) >> 12) *
            ((int32_t)dig_T3)) >> 14;

    t_fine = var1 + var2;
    T = (t_fine * 5 + 128) >> 8;
    return T;
}

uint32_t bmp280_press64_compensate() {
    int64_t var1, var2, p;
    uint8_t pressr[3];
    bmp280_readreg(BMP280_PRESS_MSB, pressr, 3);
    int32_t adc_P = ((int32_t)(pressr[0]) << 12) | ((int32_t)(pressr[1]) << 4) | ((int32_t)(pressr[2]) >> 4);

    var1 = ((int64_t)t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)dig_P6;
    var2 = var2 + ((var1 * (int64_t)dig_P5) << 17);
    var2 = var2 + (((int64_t)dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)dig_P3) >> 8) + ((var1 * (int64_t)dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)dig_P1) >> 33;

    if (var1 == 0) return 0;

    p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)dig_P9) * (p >> 13)) >> 25;
    var2 = (((int64_t)dig_P8) * p) >> 19;

    p = ((p + var1 + var2) >> 8) + (((int64_t)dig_P7) << 4);
    return (uint32_t)(p / 25600);
}

uint32_t bmp280_altitude(uint32_t press) {
    float alt = 44330 * (1.0 - pow((float)press / 1013.25, 0.1903));
    return (uint32_t)alt;
}

void read_pressure_and_temperature(volatile int32_t* pressure, volatile int32_t* temperature) {
    uint8_t msb, lsb, xlsb;

    bmp280_readreg(BMP280_TEMP_MSB, &msb, 1);
    bmp280_readreg(BMP280_TEMP_LSB, &lsb, 1);
    bmp280_readreg(BMP280_TEMP_XLSB, &xlsb, 1);
    *temperature = (msb << 12) | (lsb << 4) | (xlsb >> 4);

    bmp280_readreg(BMP280_PRESS_MSB, &msb, 1);
    bmp280_readreg(BMP280_PRESS_LSB, &lsb, 1);
    bmp280_readreg(BMP280_PRESS_XLSB, &xlsb, 1);
    *pressure = (msb << 12) | (lsb << 4) | (xlsb >> 4);
}
