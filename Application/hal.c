/*******************************************************************************
 * Copyright (c) 2015 Matthijs Kooijman
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the Eclipse Public License v1.0
 * which accompanies this distribution, and is available at
 * http://www.eclipse.org/legal/epl-v10.html
 *
 * This the HAL to run LMIC on top of the Arduino environment.
 *******************************************************************************/

#include "lmic.h"
#include "hal.h"
#include "main.h"

#include "stm8l15x.h"
#include "stm8l15x_it.h"    /* SDCC patch: required by SDCC for interrupts */
#include "stdio.h"

#define interrupts enableInterrupts
#define noInterrupts disableInterrupts


// -----------------------------------------------------------------------------
// I/O

static void hal_io_init () {
  GPIO_ResetBits(SX1276_NRESET_PORT, SX1276_NRESET_PIN);
  GPIO_ResetBits(SX1276_TX_PORT, SX1276_TX_PIN);
  GPIO_ResetBits(SX1276_RX_PORT, SX1276_RX_PIN);
  
  GPIO_ExternalPullUpConfig(SX1276_NRESET_PORT, SX1276_NRESET_PIN, ENABLE);
  
  GPIO_Init(SX1276_NRESET_PORT, SX1276_NRESET_PIN, GPIO_Mode_Out_OD_HiZ_Slow);
  
  GPIO_Init(SX1276_TX_PORT, SX1276_TX_PIN, GPIO_Mode_Out_PP_High_Slow);
  GPIO_Init(SX1276_RX_PORT, SX1276_RX_PIN, GPIO_Mode_Out_PP_High_Slow);
}

// val == 1  => tx 1
void hal_pin_rxtx (u1_t val) {
    if(val==0) {
        GPIO_ResetBits(SX1276_TX_PORT, SX1276_TX_PIN);
        GPIO_SetBits(SX1276_RX_PORT, SX1276_RX_PIN);
    } else if(val==1) {
        GPIO_ResetBits(SX1276_RX_PORT, SX1276_RX_PIN);
        GPIO_SetBits(SX1276_TX_PORT, SX1276_TX_PIN);
    } else {
        GPIO_ResetBits(SX1276_RX_PORT, SX1276_RX_PIN);
        GPIO_ResetBits(SX1276_TX_PORT, SX1276_RX_PIN);
    }
}

// set radio RST pin to given value (or keep floating!)
void hal_pin_rst (u1_t val) {
    if(val == 0 || val == 1) { // drive pin
      if(val)
        GPIO_SetBits(SX1276_NRESET_PORT, SX1276_NRESET_PIN);
      else
        GPIO_ResetBits(SX1276_NRESET_PORT, SX1276_NRESET_PIN);
    } else {
      //GPIO_Init(SX1276_NRESET_PORT, SX1276_NRESET_PIN, GPIO_Mode_In_FL_No_IT);
      GPIO_SetBits(SX1276_NRESET_PORT, SX1276_NRESET_PIN);
    }
}

static void hal_io_check() {
#if 0
    uint8_t i;
    for (i = 0; i < NUM_DIO; ++i) {
        if (lmic_pins.dio[i] == LMIC_UNUSED_PIN)
            continue;

        if (dio_states[i] != digitalRead(lmic_pins.dio[i])) {
            dio_states[i] = !dio_states[i];
            if (dio_states[i])
                radio_irq_handler(i);
        }
    }
#endif
    if ( radio_has_irq() )
        radio_irq_handler(0);
}

// -----------------------------------------------------------------------------
// SPI

static void hal_spi_init () {
  printf("Init spi...\n\r");
  CLK_PeripheralClockConfig(CLK_Peripheral_SPI1, ENABLE);

  /* Set the MOSI,MISO and SCK at high level */
  GPIO_ExternalPullUpConfig(SPI1_PORT, SPI1_SCK_PIN | \
                            SPI1_MOSI_PIN | SPI1_MISO_PIN, ENABLE);

  /* Configure CS as Output push-pull, used as sx1276 Chip select */
  GPIO_Init(SPI1_PORT, SPI1_NSS_PIN, GPIO_Mode_Out_PP_High_Slow);

  /* SPI configuration */
  SPI_Init(SPI1, SPI_FirstBit_MSB, SPI_BaudRatePrescaler_2, SPI_Mode_Master,
           SPI_CPOL_Low, SPI_CPHA_1Edge, SPI_Direction_2Lines_FullDuplex,
           SPI_NSS_Soft, 0x07);
  /* Enable SPI  */
  SPI_Cmd(SPI1, ENABLE);
}

void hal_pin_nss (u1_t val) {
  if(val)
    GPIO_SetBits(SPI1_PORT, SPI1_NSS_PIN);
  else
    GPIO_ResetBits(SPI1_PORT, SPI1_NSS_PIN);
}

// perform SPI transaction with radio
u1_t hal_spi (u1_t out) {
  /* Loop while DR register in not emplty */
  while (SPI_GetFlagStatus(SPI1, SPI_FLAG_TXE) == RESET);
  /* Send byte through the SPI peripheral */
  SPI_SendData(SPI1, out);
  /* Wait to receive a byte */
  while (SPI_GetFlagStatus(SPI1, SPI_FLAG_RXNE) == RESET);
  /* Return the byte read from the SPI bus */
  return SPI_ReceiveData(SPI1);
}

// -----------------------------------------------------------------------------
// TIME
#define TIM4_PERIOD       7
__IO uint32_t osTicks;
__IO uint32_t TimingDelay;

static void hal_time_init () {
  // Nothing to do
  printf("Init ticks...\n\r");
  CLK_PeripheralClockConfig(CLK_Peripheral_TIM4, ENABLE);
  TIM4_TimeBaseInit(TIM4_Prescaler_128, TIM4_PERIOD);
  TIM4_ClearFlag(TIM4_FLAG_Update);
  TIM4_ITConfig(TIM4_IT_Update, ENABLE);
  osTicks=0;
  TimingDelay=0;
  TIM4_Cmd(ENABLE);
}

u4_t hal_ticks () {
    // Because micros() is scaled down in this function, micros() will
    // overflow before the tick timer should, causing the tick timer to
    // miss a significant part of its values if not corrected. To fix
    // this, the "overflow" serves as an overflow area for the micros()
    // counter. It consists of three parts:
    //  - The US_PER_OSTICK upper bits are effectively an extension for
    //    the micros() counter and are added to the result of this
    //    function.
    //  - The next bit overlaps with the most significant bit of
    //    micros(). This is used to detect micros() overflows.
    //  - The remaining bits are always zero.
    //
    // By comparing the overlapping bit with the corresponding bit in
    // the micros() return value, overflows can be detected and the
    // upper bits are incremented. This is done using some clever
    // bitwise operations, to remove the need for comparisons and a
    // jumps, which should result in efficient code. By avoiding shifts
    // other than by multiples of 8 as much as possible, this is also
    // efficient on AVR (which only has 1-bit shifts).
    static uint8_t overflow = 0;

    // Scaled down timestamp. The top US_PER_OSTICK_EXPONENT bits are 0,
    // the others will be the lower bits of our return value.
    //uint32_t scaled = micros() >> US_PER_OSTICK_EXPONENT;
    uint32_t scaled = osTicks;
    // Most significant byte of scaled
    uint8_t msb = scaled >> 24;
    // Mask pointing to the overlapping bit in msb and overflow.
    const uint8_t mask = (1 << (7 - US_PER_OSTICK_EXPONENT));
    // Update overflow. If the overlapping bit is different
    // between overflow and msb, it is added to the stored value,
    // so the overlapping bit becomes equal again and, if it changed
    // from 1 to 0, the upper bits are incremented.
    overflow += (msb ^ overflow) & mask;

    // Return the scaled value with the upper bits of stored added. The
    // overlapping bit will be equal and the lower bits will be 0, so
    // bitwise or is a no-op for them.
    return scaled | ((uint32_t)overflow << 24);

    // 0 leads to correct, but overly complex code (it could just return
    // micros() unmodified), 8 leaves no room for the overlapping bit.
    //static_assert(US_PER_OSTICK_EXPONENT > 0 && US_PER_OSTICK_EXPONENT < 8, "Invalid US_PER_OSTICK_EXPONENT value");
#if US_PER_OSTICK_EXPONENT <= 0 || US_PER_OSTICK_EXPONENT >= 8
#error "Invalid US_PER_OSTICK_EXPONENT value"
#endif
}

// Returns the number of ticks until time. Negative values indicate that
// time has already passed.
static s4_t delta_time(u4_t time) {
    return (s4_t)(time - hal_ticks());
}

void hal_delayMs (u4_t time) {
  TimingDelay = ms2osticks(time);
  while (TimingDelay != 0);
}

void hal_waitUntil (u4_t time) {
#if 0
    s4_t delta = delta_time(time);
    // From delayMicroseconds docs: Currently, the largest value that
    // will produce an accurate delay is 16383.
    while (delta > (16000 / US_PER_OSTICK)) {
        delayMs(16);
        delta -= (16000 / US_PER_OSTICK);
    }
    if (delta > 0)
        delayMicroseconds(delta * US_PER_OSTICK);
#endif
  
#if 0
  s4_t delta = delta_time(time);
  if(delta < 0)
    return;
  
  TimingDelay = delta;
  while (TimingDelay != 0);
#endif
  
#if 1
  while (osTicks < time);
#endif
}

// check and rewind for target time
u1_t hal_checkTimer (u4_t time) {
    // No need to schedule wakeup, since we're not sleeping
    return delta_time(time) <= 0;
}

static uint8_t irqlevel = 0;

void hal_disableIRQs () {
    //noInterrupts();
    //printf("noInterrupts()\n\r");
    irqlevel++;
}

void hal_enableIRQs () {
    if(--irqlevel == 0) {
        //interrupts();
        //printf("interrupts()\n\r");

        // Instead of using proper interrupts (which are a bit tricky
        // and/or not available on all pins on AVR), just poll the pin
        // values. Since os_runloop disables and re-enables interrupts,
        // putting this here makes sure we check at least once every
        // loop.
        //
        // As an additional bonus, this prevents the can of worms that
        // we would otherwise get for running SPI transfers inside ISRs
        hal_io_check();
    }
}


void hal_sleep () {
  hal_io_check();
  // Not implemented
}

// -----------------------------------------------------------------------------
void hal_printf_init() {
  SYSCFG_REMAPPinConfig(REMAP_Pin_USART1TxRxPortA, ENABLE);
  CLK_PeripheralClockConfig(CLK_Peripheral_USART1, ENABLE);
  GPIO_ExternalPullUpConfig(UART1_PORT, UART1_TXD_PIN, ENABLE);
  GPIO_ExternalPullUpConfig(UART1_PORT, UART1_RXD_PIN, ENABLE);

  USART_Init(USART1, 115200,
             USART_WordLength_8b,
             USART_StopBits_1,
             USART_Parity_No,
             (USART_Mode_TypeDef)(USART_Mode_Tx | USART_Mode_Rx));
  USART_Cmd(USART1, ENABLE);
  printf("\n\rSTM8 LoRa node\n\r");
}


void hal_init () {
    // configure radio I/O and interrupt handler
    hal_io_init();
    if(isUsartEnabled()) {
      // printf support
      hal_printf_init();
    } else {
      GPIO_SetBits(UART1_PORT, UART1_TXD_PIN);
      GPIO_Init(UART1_PORT, UART1_TXD_PIN, GPIO_Mode_In_PU_IT);
      GPIO_Init(UART1_PORT, UART1_RXD_PIN, GPIO_Mode_In_PU_IT);
      //EXTI_SetPinSensitivity(AUXEXTI_PIN, EXTI_Trigger_Rising_Falling);
      EXTI_SetPinSensitivity(UART1_TXDEXTI_PIN, EXTI_Trigger_Rising);//2
      EXTI_SetPinSensitivity(UART1_RXDEXTI_PIN, EXTI_Trigger_Rising);//3
    }
    // configure radio SPI
    hal_spi_init();
    // configure timer and interrupt handler
    hal_time_init();
    enableInterrupts();
}

void hal_failed (const char *file, u2_t line) {
  printf("assert failed: %s:%d\n\r", file, line);
  while (1) {}
}