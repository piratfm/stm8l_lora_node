/* Includes ------------------------------------------------------------------*/
#include "stm8l15x.h"
#include "stm8l15x_it.h"    /* SDCC patch: required by SDCC for interrupts */
#include <stdio.h>
#include "lmic.h"
#include "cayenne_lpp.h"
#include "main.h"

#define DELAY_IN_SECONDS    30


__no_init __eeprom u1_t APPEUI[8];
__no_init __eeprom u1_t APPKEY[16];
__no_init __eeprom u1_t DEVEUI[8];
/*
    ******1 - disable UART and enable GPIO inputs on TXD/RXD lines.
    *****1* - disable LoRa ACK's from the gateways.
*/
__no_init __eeprom uint8_t dev_flags;
__no_init __eeprom uint32_t delay_minutes;

__no_init __eeprom uint32_t DEVADDR;
__no_init __eeprom u1_t NWKSKEY[16];
__no_init __eeprom u1_t APPSKEY[16];


bool config_mode = false;
bool isUsartEnabled() {
  return (config_mode || !(dev_flags&0x01));
}

int __eeprom_wait_for_last_operation(void) {
  FLASH_Status_TypeDef status = FLASH_WaitForLastOperation(FLASH_MemType_Data);
  return !!(status & (FLASH_Status_Write_Protection_Error | FLASH_Status_Successful_Operation));
}

void __eeprom_program_byte(unsigned char __near * dst, unsigned char v) {
  FLASH_ProgramByte((uint32_t)dst, (uint8_t)v);
}

void __eeprom_program_long(unsigned char __near * dst, unsigned long v) {
  FLASH_ProgramWord((uint32_t)dst, (uint32_t)v);
}


cayenne_lpp_t lpp = { 0 };

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 30;

extern __IO uint32_t osTicks;
extern __IO uint32_t TimingDelay;
__IO uint8_t extTriggered = 0;

/** @addtogroup STM8L15x_StdPeriph_Examples
  * @{
  */

/**
  * @addtogroup USART_Printf
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
#ifdef _RAISONANCE_
#define PUTCHAR_PROTOTYPE int putchar (char c)
#define GETCHAR_PROTOTYPE int getchar (void)
#elif defined (_COSMIC_)
#define PUTCHAR_PROTOTYPE char putchar (char c)
#define GETCHAR_PROTOTYPE char getchar (void)
#elif defined (_SDCC_)                    /* SDCC patch: same types as stdio.h */
 #if SDCC_VERSION >= 30600                  /* putchar() was changed for >=3.6.0, see sdcc manual */
  #define PUTCHAR_PROTOTYPE int putchar (int c)
 #else
  #define PUTCHAR_PROTOTYPE void putchar (char c)
 #endif 
 #define GETCHAR_PROTOTYPE int getchar (void)
#else /* _IAR_ */
#define PUTCHAR_PROTOTYPE int putchar (int c)
#define GETCHAR_PROTOTYPE int getchar (void)
#endif /* _RAISONANCE_ */

/* Private functions ---------------------------------------------------------*/
//Unique ID for STM8L151
#ifdef STM8L15X_MD
#define         ID1                                 ( 0x4925 )
#define         ID2                                 ( 0x4926 )
#define         ID3                                 ( 0x4927 )
#define         ID4                                 ( 0x4928 )
#define         ID5                                 ( 0x4929 )
#define         ID6                                 ( 0x492A )
#define         ID7                                 ( 0x492B )
#define         ID8                                 ( 0x492C )
#define         ID9                                 ( 0x492D )
#define         IDA                                 ( 0x492E )
#define         IDB                                 ( 0x492F )
#define         IDC                                 ( 0x4930 )
#define         VREFINT                             ( 0x4910 )
#endif

void os_getArtEui (u1_t* buf) {
  if(dev_flags & 0x04)
    return;
//  memcpy_P(buf, (void const *)APPEUI, 8);
  buf[0] = APPEUI[7];
  buf[1] = APPEUI[6];
  buf[2] = APPEUI[5];
  buf[3] = APPEUI[4];
  buf[4] = APPEUI[3];
  buf[5] = APPEUI[2];
  buf[6] = APPEUI[1];
  buf[7] = APPEUI[0];
}


void getDevEui (u1_t* buf) {
    buf[0] = ( ( *( uint8_t* )ID1 )+ ( *( uint8_t* )ID9 ) );
    buf[1] = ( ( *( uint8_t* )ID2 )+ ( *( uint8_t* )IDA ) );
    buf[2] = ( ( *( uint8_t* )ID3 )+ ( *( uint8_t* )IDB ) );
    buf[3] = ( ( *( uint8_t* )ID4 )+ ( *( uint8_t* )IDC ) );
    buf[4] = ( ( *( uint8_t* )ID5 ) );
    buf[5] = ( ( *( uint8_t* )ID6 ) );
    buf[6] = ( ( *( uint8_t* )ID7 ) );
    buf[7] = ( ( *( uint8_t* )ID8 ) );
}

// This should also be in little endian format, see above.
void os_getDevEui (u1_t* buf) {
  if(dev_flags & 0x04)
    return;
  
  buf[0] = DEVEUI[7];
  buf[1] = DEVEUI[6];
  buf[2] = DEVEUI[5];
  buf[3] = DEVEUI[4];
  buf[4] = DEVEUI[3];
  buf[5] = DEVEUI[2];
  buf[6] = DEVEUI[1];
  buf[7] = DEVEUI[0];
  
  printf("DevEUI: %02x %02x %02x %02x %02x %02x %02x %02x\r\n",
           buf[7], buf[6], buf[5], buf[4], buf[3], buf[2], buf[1], buf[0]);
}
// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from ttnctl can be copied as-is.
// The key shown here is the semtech default key.
void os_getDevKey (u1_t* buf) {
  if(dev_flags & 0x04)
    return;
  memcpy_P(buf, (void const *)APPKEY, 16);
}

static osjob_t sendjob;


uint16_t readVoltageValue() {
  uint32_t refV = (((uint32_t)0x600) | ( *( uint8_t* )VREFINT )) * 3000;

  ADC_DeInit(ADC1);
  CLK_PeripheralClockConfig(CLK_Peripheral_ADC1, ENABLE);
  ADC_Init(ADC1, ADC_ConversionMode_Continuous, ADC_Resolution_12Bit, ADC_Prescaler_2);
  ADC_SamplingTimeConfig(ADC1, ADC_Group_SlowChannels, ADC_SamplingTime_384Cycles);
  ADC_Cmd(ADC1, ENABLE);
  ADC_SchmittTriggerConfig(ADC1, ADC_Channel_Vrefint, DISABLE);
  ADC_ChannelCmd(ADC1, ADC_Channel_Vrefint, ENABLE);
  ADC_SoftwareStartConv(ADC1);
  while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == 0)
  {}
  /* Get conversion value */
  uint16_t ADCData = ADC_GetConversionValue(ADC1);
  ADC_Cmd(ADC1, DISABLE);
  CLK_PeripheralClockConfig(CLK_Peripheral_ADC1, DISABLE);
  //uint32_t volt = refV * 100 / ADCData;
  uint32_t volt = refV / ADCData;
  //printf("R: %02X V1: %lu | V2: %u | V3: %lu\r\n", ( *( uint8_t* )VREFINT ), refV, ADCData, volt);
  return volt/10;
}

void writeCounterValue(uint32_t cnt) {
  uint32_t *EEPROMArray = (uint32_t *)FLASH_DATA_EEPROM_START_PHYSICAL_ADDRESS;
  uint8_t i=0;
  uint8_t max_idx=0;
  uint32_t max=0;
  do {
    if(EEPROMArray[i] >= max) {
      max = EEPROMArray[i];
      max_idx=i;
    }
    i++;
  } while(i);
  max_idx++;
  
  FLASH_Unlock(FLASH_MemType_Data);
  FLASH_ProgramWord(FLASH_DATA_EEPROM_START_PHYSICAL_ADDRESS + 4*max_idx, cnt);
  FLASH_Lock(FLASH_MemType_Data);
}


void do_send(osjob_t* j){
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
         printf("OP_TXRXPEND, not sending\r\n");
    } else {
        
        cayenne_lpp_reset(&lpp);
        int16_t temp;            /**< 2 bytes, 0.1°C signed */
        uint16_t pressure;       /**< 2 bytes 0.1 hPa Unsigned */
        uint8_t humidity; /**< 1 byte, 0.5% unsigned */

        uint16_t voltage = readVoltageValue();  

        BMxx80_read_data(&temp, &pressure, &humidity);
        printf("T: %d | P: %u | H:%u | V: %u\r\n", temp, pressure, humidity, voltage);
        
        lpp.buffer[lpp.cursor++] = 1;
        lpp.buffer[lpp.cursor++] = CAYENNE_LPP_DIGITAL_INPUT;
        lpp.buffer[lpp.cursor++] = GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_0) ? 1 : 0;
   
        if(sensor_is_bme280 || sensor_is_bmp280 || sensor_is_bmp180) {
          lpp.buffer[lpp.cursor++] = 2;
          lpp.buffer[lpp.cursor++] = CAYENNE_LPP_TEMPERATURE;
          lpp.buffer[lpp.cursor++] = temp >> 8;
          lpp.buffer[lpp.cursor++] = temp;
          lpp.buffer[lpp.cursor++] = 3;
          lpp.buffer[lpp.cursor++] = CAYENNE_LPP_BAROMETRIC_PRESSURE;
          lpp.buffer[lpp.cursor++] = pressure >> 8;
          lpp.buffer[lpp.cursor++] = pressure;
        }
        
        lpp.buffer[lpp.cursor++] = 4;
        lpp.buffer[lpp.cursor++] = CAYENNE_LPP_ANALOG_INPUT;
        lpp.buffer[lpp.cursor++] = voltage >> 8;
        lpp.buffer[lpp.cursor++] = voltage;
        
        if(!isUsartEnabled()) {
          lpp.buffer[lpp.cursor++] = 5;
          lpp.buffer[lpp.cursor++] = CAYENNE_LPP_DIGITAL_INPUT;
          lpp.buffer[lpp.cursor++] = GPIO_ReadInputDataBit(UART1_PORT, UART1_TXD_PIN) ? 1 : 0;
          lpp.buffer[lpp.cursor++] = 6;
          lpp.buffer[lpp.cursor++] = CAYENNE_LPP_DIGITAL_INPUT;
          lpp.buffer[lpp.cursor++] = GPIO_ReadInputDataBit(UART1_PORT, UART1_RXD_PIN) ? 1 : 0;
        }

        if(sensor_is_bme280) {
          lpp.buffer[lpp.cursor++] = 7;
          lpp.buffer[lpp.cursor++] = CAYENNE_LPP_RELATIVE_HUMIDITY;
          lpp.buffer[lpp.cursor++] = humidity;
        }

        // Prepare upstream data transmission at the next possible time.
        LMIC_setTxData2(1, lpp.buffer, lpp.cursor, (dev_flags & 0x02) ? 0 : 1);
        printf("Packet queued\r\n");
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

bool rejoinFailed = FALSE;
bool txComplete = FALSE;

void onEvent (ev_t ev) {
    printf("%lu: ", os_getTime());
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            printf("EV_SCAN_TIMEOUT\r\n");
            break;
        case EV_BEACON_FOUND:
            printf("EV_BEACON_FOUND\r\n");
            break;
        case EV_BEACON_MISSED:
            printf("EV_BEACON_MISSED\r\n");
            break;
        case EV_BEACON_TRACKED:
            printf("EV_BEACON_TRACKED\r\n");
            break;
        case EV_JOINING:
            printf("EV_JOINING\r\n");
            break;
        case EV_JOINED:
            printf("EV_JOINED\r\n");
            // Disable link check validation (automatically enabled
            // during join, but not supported by TTN at this time).
            //LMIC_setLinkCheckMode(0);
            uint8_t cnt;
            printf("DevADDR[msb...lsb]: %08lx \r\n", LMIC.devaddr);

            printf("NwkSKEY[msb...lsb]:");
            for(cnt=0;cnt<16;cnt++) {
              printf("%02X ", LMIC.nwkKey[cnt]);
            }
            printf("\r\n");

            printf("AppSKEY[msb...lsb]:");
            for(cnt=0;cnt<16;cnt++) {
              printf("%02X ", LMIC.artKey[cnt]);
            }
            printf("\r\n");
        
            break;
        case EV_RFU1:
            printf("EV_RFU1\r\n");
            break;
        case EV_JOIN_FAILED:
            printf("EV_JOIN_FAILED\r\n");
            break;
        case EV_REJOIN_FAILED:
            printf("EV_REJOIN_FAILED\r\n");
            rejoinFailed = TRUE;
            break;
        case EV_TXCOMPLETE:
          printf("EV_TXCOMPLETE (includes waiting for RX windows)\r\n");
            if (LMIC.txrxFlags & TXRX_ACK) {
              printf("Received ack\r\n");
            } else if(LMIC.opmode == 0x0900) {
              printf("Sending data timeout\r\n");
            }
            if (LMIC.dataLen)
              printf("Received %d bytes of payload\r\n", LMIC.dataLen);
            txComplete=TRUE;
            // Schedule next transmission
            //os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            break;
        case EV_LOST_TSYNC:
            printf("EV_LOST_TSYNC\r\n");
            break;
        case EV_RESET:
            printf("EV_RESET\r\n");
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            printf("EV_RXCOMPLETE\r\n");
            break;
        case EV_LINK_DEAD:
            printf("EV_LINK_DEAD\r\n");
            break;
        case EV_LINK_ALIVE:
            printf("EV_LINK_ALIVE\r\n");
            break;
         default:
            printf("Unknown event\r\n");
            break;
    }
}   

uint8_t hexPtr(uint8_t *ary, uint8_t cnt)
{
  uint8_t idx=0;
  uint8_t ch;
  do {
    ch = getchar();
    putchar(ch);

    uint8_t hX=0xFF;
    if(ch>='A'&&ch<='F') {
      hX = 10 + ch-'A';
    } else if(ch>='a'&&ch<='f') {
      hX = 10 + ch-'a';
    } else  if(ch>='0'&&ch<='9') {
      hX = ch-'0';
    }
    
    if(idx < cnt*2 && hX!=0xFF) {
      if(idx%2) {
        ary[idx/2] |= hX;
      } else {
        ary[idx/2] = hX << 4;
      }
      idx++;
    }
  } while(ch!='\r' && ch!='\n');

  return idx;
}

void keysInfo()
{
  uint8_t cnt;
  if(!(dev_flags & 0x04)) {
    printf("MODE: OTAA\r\n");
    printf("AppEUI[lsb...msb]:");
    for(cnt=0;cnt<8;cnt++)
      printf("%02X ", APPEUI[cnt]);
    printf("\r\n");
    
    printf("AppKEY[msb...lsb]:");
    for(cnt=0;cnt<16;cnt++)
      printf("%02X ", APPKEY[cnt]);
    printf("\r\n");

    printf("DevEUI[msb...lsb]:");
    for(cnt=0;cnt<8;cnt++)
      printf("%02X ", DEVEUI[cnt]);
    printf("\r\n");
  } else {
    printf("MODE: ADR\r\n");
    printf("DevADDR[msb...lsb]: %08lX \r\n", DEVADDR);
    
    printf("NwkSKEY[msb...lsb]:");
    for(cnt=0;cnt<16;cnt++) {
      printf("%02X ", NWKSKEY[cnt]);
    }
    printf("\r\n");

    printf("AppSKEY[msb...lsb]:");
    for(cnt=0;cnt<16;cnt++) {
      printf("%02X ", APPSKEY[cnt]);
    }
    printf("\r\n");
  }
  printf("logging interval: ~%lu minutes\r\n", delay_minutes);
}

void configModeFunc()
{
  uint8_t buff[16];
  uint8_t ch,cnt;
  uint16_t stat_int;

  printf("=============CONFIGURATION MODE==============\r\n");
  printf("Waiting for release of M0 pin...\r\n");
  while(GPIO_ReadInputDataBit(I2C_PORT, I2C_SCL_PIN)==0);
  printf("=============================================\r\n");
  printf("Please enter required data to register device\r\n");
  printf("in LoRa network. All values - are MSB first,\r\n");
  printf("just like it shown in TTN console.\r\n");
  printf("=============================================\r\n");
  FLASH_Unlock(FLASH_MemType_Data);

  
  printf("Disable OTAA activation method (y/N):");
  cnt = dev_flags & 0xFB;
  do {
    ch = getchar();
    putchar(ch);
    if(ch=='y' || ch=='Y')
      cnt |= 0x04;
  } while (ch != '\r' && ch != '\n');
  printf("\r\nOTAA is: %s\r\n", (cnt & 0x04) ? "DISABLED" : "ENABLED");
  dev_flags = cnt;
  
  
  if(!(cnt & 0x04)) {
  //otaa:
    printf("AppEUI:");
    cnt=hexPtr(buff, 8);
    printf("\r\n[%u]: ", cnt);
    for(cnt=0;cnt<8;cnt++) {
      printf("%02X ", buff[cnt]);
      APPEUI[cnt] = buff[cnt];
    }
    printf("\r\n");
    printf("AppKEY:");
    cnt=hexPtr(buff, 16);
    printf("\r\n[%u]: ", cnt);
    for(cnt=0;cnt<16;cnt++) {
      printf("%02X ", buff[cnt]);
      APPKEY[cnt] = buff[cnt];
    }
    printf("\r\n");

    printf("DevEUI[press enter for default: ");
    getDevEui(buff);
    for(cnt=0;cnt<8;cnt++)
      printf("%02X ", buff[cnt]);
    printf("]:");
    cnt=hexPtr(buff, 8);
    printf("\r\n[%u]: ", cnt);
    for(cnt=0;cnt<8;cnt++) {
      printf("%02X ", buff[cnt]);
      DEVEUI[cnt] = buff[cnt];
    }
    printf("\r\n");
  } else {
    //ADR mode
    printf("NwkSKEY:");
    cnt=hexPtr(buff, 16);
    printf("\r\n[%u]: ", cnt);
    for(cnt=0;cnt<16;cnt++) {
      printf("%02X ", buff[cnt]);
      NWKSKEY[cnt] = buff[cnt];
    }
    printf("\r\n");

    printf("AppSKEY:");
    cnt=hexPtr(buff, 16);
    printf("\r\n[%u]: ", cnt);
    for(cnt=0;cnt<16;cnt++) {
      printf("%02X ", buff[cnt]);
      APPSKEY[cnt] = buff[cnt];
    }
    printf("\r\n");
  
    printf("DevADDR:");
    cnt=hexPtr(buff, 4);
    printf("\r\n[%u]: ", cnt);
    for(cnt=0;cnt<4;cnt++) {
      printf("%02X ", buff[cnt]);
    }
    DEVADDR = ((uint32_t)buff[0]) << 24 | ((uint32_t)buff[1]) << 16 | ((uint32_t)buff[2]) << 8 | buff[3];
    printf(" -- %08lX ", DEVADDR);
    printf("\r\n");
  }
  
  printf("Statistics data push interval (in minutes)\r\n"
         "[default: 1, max: 65535]:");
  stat_int = 0;
  do {
    ch = getchar();
    if(ch>='0' && ch<='9') {
      putchar(ch);
      stat_int = stat_int*10 + (ch-'0');
    }
  } while (ch != '\r' && ch != '\n');
  
  if(stat_int==0)
    stat_int=1;
  printf("\r\nStatistics collection interval: %u minutes\r\n", stat_int);
  delay_minutes=stat_int;
  printf("\r\nStatistics collection interval: %lu minutes\r\n", delay_minutes);
  
  printf("Receive confirmation (ACKnowledgement) from gateways (Y/n):");
  cnt = dev_flags & 0xFD;
  do {
    ch = getchar();
    putchar(ch);
    if(ch=='n' || ch=='N')
      cnt |= 0x02;
  } while (ch != '\r' && ch != '\n');
  printf("\r\nACK is: %s\r\n", (cnt & 0x02) ? "DISABLED" : "ENABLED");
  dev_flags = cnt;
  
  printf("Disable UART pins and use them as additional input GPIO's\r\n"
         "for digital sensors (y/N):");
  
  cnt = dev_flags & 0xFE;
  do {
    ch = getchar();
    putchar(ch);
    if(ch=='y' || ch=='Y')
      cnt |= 0x01;
  } while (ch != '\r' && ch != '\n');
  printf("\r\nUART is: %s\r\n", (cnt & 0x01) ? "DISABLED" : "ENABLED");
  dev_flags = cnt;

  FLASH_Lock(FLASH_MemType_Data);
  printf("=============CONFIGURATION FINISHED==============\r\n");
  config_mode=false;
}
    


/***********************************************************************************/
/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
void main(void)
{
  /*High speed internal clock prescaler: 1*/
  CLK_SYSCLKDivConfig(CLK_SYSCLKDiv_1);

  // 64 / 38000 * 2^16/2 ~ 55s per counter tick
  CLK_RTCClockConfig(CLK_RTCCLKSource_LSI, CLK_RTCCLKDiv_64);
  /* Wait for LSI clock to be ready */
  while (CLK_GetFlagStatus(CLK_FLAG_LSIRDY) == RESET);
  CLK_PeripheralClockConfig(CLK_Peripheral_RTC, ENABLE);
  /* Configures the RTC */
  RTC_WakeUpClockConfig(RTC_WakeUpClock_CK_SPRE_16bits);
  RTC_ITConfig(RTC_IT_WUT, ENABLE);

  // emable TCXO in case if we have additional I2C devices connected there.
  GPIO_SetBits(TCXO_EN_PORT, TCXO_EN_PIN);
  GPIO_Init(TCXO_EN_PORT, TCXO_EN_PIN, GPIO_Mode_Out_PP_High_Slow);


  GPIO_SetBits(AUX_PORT, AUX_PIN);
  GPIO_Init(AUX_PORT, AUX_PIN, GPIO_Mode_In_PU_IT);
  //EXTI_SetPinSensitivity(AUXEXTI_PIN, EXTI_Trigger_Rising_Falling);
  EXTI_SetPinSensitivity(AUXEXTI_PIN, EXTI_Trigger_Rising);

  /*initialize unused GPIO in low power*/
#if 1
  GPIO_Init(GPIOA, GPIO_Pin_4|GPIO_Pin_5, GPIO_Mode_Out_PP_Low_Slow);
  GPIO_Init(GPIOB, GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3, GPIO_Mode_Out_PP_Low_Slow);
  GPIO_Init(GPIOC, GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2, GPIO_Mode_Out_PP_Low_Slow);
  GPIO_Init(GPIOD, GPIO_Pin_1|GPIO_Pin_4, GPIO_Mode_Out_PP_Low_Slow);
#endif

  i2c_init();
  uint32_t cnt=0x200000; //7 seconds
  while(GPIO_ReadInputDataBit(I2C_PORT, I2C_SCL_PIN)==0 && --cnt);

  config_mode=(!cnt || (!(dev_flags & 0x04) && DEVEUI[0]==0 && DEVEUI[1]==0 && DEVEUI[2]==0 && DEVEUI[3]==0 &&
              DEVEUI[4]==0 && DEVEUI[5]==0 && DEVEUI[6]==0 && DEVEUI[7]==0)
                    || ((dev_flags & 0x04) && DEVADDR ==0));

  // LMIC init
  os_init();

  keysInfo();

  if(config_mode) {
    configModeFunc();
    keysInfo();
    WWDG_SWReset();
  }

  
  //i2c_scan();
  BMxx80_init();
//  int16_t temp;
//  uint16_t pressure;
//  BMP180_read_data(&temp, &pressure);
//  printf("T: %d | P: %u\r\n", temp, pressure);

  // Reset the MAC state. Session and pending data transfers will be discarded.
  printf("Reset the MAC state....\r\n");
  LMIC_reset();
  LMIC_setLinkCheckMode(1);
  //LMIC_init();
  //LMIC_setClockError(MAX_CLOCK_ERROR * 20 / 100);

  //LMIC_startJoining();
  
  LMIC_setClockError(MAX_CLOCK_ERROR * 30 / 100);

  // Set up the channels used by the Things Network, which corresponds
  // to the defaults of most gateways. Without this, only three base
  // channels from the LoRaWAN specification are used, which certainly
  // works, so it is good for debugging, but can overload those
  // frequencies, so be sure to configure the full frequency range of
  // your network here (unless your network autoconfigures them).
  // Setting up channels should happen after LMIC_setSession, as that
  // configures the minimal channel set.
  
  LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
  LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band
  // TTN defines an additional channel at 869.525Mhz using SF9 for class B
  // devices' ping slots. LMIC does not have an easy way to define set this
  // frequency and support for class B is spotty and untested, so this
  // frequency is not configured here.

  // required for downlink
  LMIC.dn2Dr = SF9;

  // Set data rate and transmit power (note: txpow seems to be ignored by the library)
  LMIC_setDrTxpow(DR_SF7, 20);
  
  if(dev_flags & 0x04) {
    LMIC_setAdrMode(true);
    LMIC_setSession (0x01, DEVADDR, (uint8_t *)NWKSKEY, (uint8_t *)APPSKEY);
    rejoinFailed = FALSE;
  }

  // Start job (sending automatically starts OTAA too)
  do_send(&sendjob);

  //uint8_t to_sleep=0;
  while(1) {
    os_runloop_once();
    
    if(rejoinFailed) {
      rejoinFailed = FALSE;
      if(!(dev_flags & 0x04))
        LMIC_startJoining();
    }
    
    if(txComplete) {
      printf("To Sleep!!!!\r\n");
      extTriggered = 0;
      
      //disable switch and TCXO
      hal_pin_rxtx(2);
      
      //do not lower nreser, because it taking 5mA!
      GPIO_ResetBits(TCXO_EN_PORT, TCXO_EN_PIN);

      printf("======= HALT  \r\n");

      TIM4_Cmd(DISABLE);
      SPI_Cmd(SPI1, DISABLE);
      if(isUsartEnabled()) {
        USART_Cmd(USART1, DISABLE);
        CLK_PeripheralClockConfig(CLK_Peripheral_USART1, DISABLE);
      }
      CLK_PeripheralClockConfig(CLK_Peripheral_TIM4, DISABLE);
      CLK_PeripheralClockConfig(CLK_Peripheral_SPI1, DISABLE);
      
      /* Switch to LSI as system clock source */
      /* system clock prescaler: 1*/
      //CLK_SYSCLKDivConfig(CLK_SYSCLKDiv_1);
      CLK_SYSCLKSourceConfig(CLK_SYSCLKSource_LSI);
      CLK_SYSCLKSourceSwitchCmd(ENABLE);
      while (CLK_GetFlagStatus(CLK_FLAG_LSIRDY) == 0);
      CLK_HSICmd(DISABLE);
      CLK_HSEConfig(CLK_HSE_OFF);
 

      /* Set STM8 in low power */
      PWR_UltraLowPowerCmd(ENABLE);
      
      RTC_SetWakeUpCounter(delay_minutes); //55s
      RTC_WakeUpCmd(ENABLE);

      EXTI_ClearITPendingBit(EXTI_IT_Pin0);
      halt();

      RTC_WakeUpCmd(DISABLE);
      
      /* Switch to HSI as system clock source */
      /* system clock prescaler: 1*/
      //CLK_SYSCLKDivConfig(CLK_SYSCLKDiv_1);
      CLK_HSICmd(ENABLE);
      while (CLK_GetFlagStatus(CLK_FLAG_HSIRDY) == 0);
      CLK_SYSCLKSourceConfig(CLK_SYSCLKSource_HSI);
      CLK_SYSCLKSourceSwitchCmd(ENABLE);

      osTicks += sec2osticks(55*delay_minutes);

      GPIO_SetBits(TCXO_EN_PORT, TCXO_EN_PIN);

      CLK_PeripheralClockConfig(CLK_Peripheral_TIM4, ENABLE);
      CLK_PeripheralClockConfig(CLK_Peripheral_SPI1, ENABLE);
      if(isUsartEnabled()) {
        CLK_PeripheralClockConfig(CLK_Peripheral_USART1, ENABLE);
        USART_Cmd(USART1, ENABLE);
      }
      TIM4_Cmd(ENABLE);
      SPI_Cmd(SPI1, ENABLE);
      
      printf("======= WORK  %s\r\n", extTriggered ? "extTriggered" : "RTC");
      hal_pin_rxtx(0);
      
      do_send(&sendjob);
      txComplete=FALSE;
    }
      
  }
}



















/**
  * @brief Retargets the C library printf function to the USART.
  * @param[in] c Character to send
  * @retval char Character sent
  * @par Required preconditions:
  * - None
  */
PUTCHAR_PROTOTYPE
{
  if(isUsartEnabled()) {
    /* Write a character to the USART */
    USART_SendData8(USART1, c);
    /* Loop until the end of transmission */
    while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
  }

  return (c);
}
/**
  * @brief Retargets the C library scanf function to the USART.
  * @param[in] None
  * @retval char Character to Read
  * @par Required preconditions:
  * - None
  */
GETCHAR_PROTOTYPE
{
  int c = 0;
  if(isUsartEnabled()) {
    /* Loop until the Read data register flag is SET */
    while (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET);
    c = USART_ReceiveData8(USART1);
  }
  return (c);
}

#ifdef  USE_FULL_ASSERT
  /**
    * @brief  Reports the name of the source file and the source line number
    *   where the assert_param error has occurred.
    * @param  file: pointer to the source file name
    * @param  line: assert_param error line source number
    * @retval None
    */
  void assert_failed(uint8_t* file, uint32_t line)
  {
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

    /* Infinite loop */
    while (1)
    {}
  }
#endif
/**
  * @}
  */

/**
  * @}
  */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
