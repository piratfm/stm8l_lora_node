#ifndef _main_h_
#define _main_h_

#include "stdint.h"
#include "stdbool.h"


#define UART1_PORT    GPIOA
#define UART1_TXD_PIN GPIO_Pin_2
#define UART1_RXD_PIN GPIO_Pin_3
#define UART1_TXDEXTI_PIN EXTI_Pin_2
#define UART1_RXDEXTI_PIN EXTI_Pin_3

#define TCXO_EN_PORT GPIOC
#define TCXO_EN_PIN GPIO_Pin_4

#define SX1276_NRESET_PORT GPIOC
#define SX1276_NRESET_PIN  GPIO_Pin_3

//pe42421 switch:
#define SX1276_TX_PORT GPIOD
#define SX1276_TX_PIN  GPIO_Pin_2

#define SX1276_RX_PORT GPIOD
#define SX1276_RX_PIN  GPIO_Pin_3


#define SPI1_PORT GPIOB
#define SPI1_MISO_PIN  GPIO_Pin_7
#define SPI1_MOSI_PIN  GPIO_Pin_6
#define SPI1_SCK_PIN  GPIO_Pin_5
#define SPI1_NSS_PIN  GPIO_Pin_4

#define I2C_PORT GPIOC
#define I2C_SCL_PIN GPIO_Pin_5
#define I2C_SDA_PIN GPIO_Pin_6

#define AUX_PORT GPIOD
#define AUX_PIN GPIO_Pin_0
#define AUXEXTI_PIN EXTI_Pin_0

#ifndef true
#define true TRUE
#define false FALSE
#endif

extern bool sensor_is_bme280;
extern bool sensor_is_bmp280;
extern bool sensor_is_bmp180;

void hal_delayMs (uint32_t time);

bool isUsartEnabled();

void i2c_init();
void i2c_scan();
void i2c_start_condition();
void i2c_stop_condition();
bool i2c_write_byte( uint8_t B, bool start, bool stop );
uint8_t i2c_read_byte( bool ack, bool stop );
bool i2c_write_byte_data( uint8_t address, uint8_t reg, uint8_t data );
uint8_t i2c_read_byte_data( uint8_t address, uint8_t reg );
uint16_t i2c_read_short_data( uint8_t address, uint8_t reg, bool swap );
uint32_t i2c_read_3bytes_data( uint8_t address, uint8_t reg );

void BMxx80_init();
void BMxx80_read_data(int16_t *t, uint16_t *p, uint8_t *h);

#endif // _main_h_
