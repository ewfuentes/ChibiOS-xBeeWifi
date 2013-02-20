#include "ch.h"
#include "hal.h"
#include "XBEES6.h"

#define GPIOE_DOUT 10
#define GPIOE_nRESET 9

#define GPIOB_nATTN 	11
#define GPIOB_SPI2_SS   12
#define GPIOB_SPI2_SCLK 13
#define GPIOB_SPI2_MISO 14
#define GPIOB_SPI2_MOSI 15 

#define BUFFER_SIZE 256

volatile uint8_t dataAvailable = TRUE;
volatile uint16_t dataTxSize = 0;

static SPIConfig spi2Config = {
		NULL, //callback
		GPIOB, //Slave Select Port
		GPIOB_SPI2_SS, //Slave Select Pin
		SPI_CR1_BR_2 //Set Communication speed for 5.25 MHz
};

static void extAttention(EXTDriver *extp, expchannel_t channel){
	(void) extp;
	(void) channel;

	dataAvailable = TRUE;
}

static EXTConfig dataAvailableEXTConfig = {
  {
    {EXT_CH_MODE_DISABLED, NULL}, 	// 0
    {EXT_CH_MODE_DISABLED, NULL}, 	// 1
    {EXT_CH_MODE_DISABLED, NULL},	// 2
    {EXT_CH_MODE_DISABLED, NULL},	// 3
    {EXT_CH_MODE_DISABLED, NULL},	// 4
    {EXT_CH_MODE_DISABLED, NULL},	// 5
    {EXT_CH_MODE_DISABLED, NULL},	// 6
    {EXT_CH_MODE_DISABLED, NULL},	// 7
    {EXT_CH_MODE_DISABLED, NULL},	// 8
    {EXT_CH_MODE_DISABLED, NULL},	// 9
    {EXT_CH_MODE_DISABLED, NULL},	// 10
    {EXT_MODE_GPIOB | EXT_CH_MODE_FALLING_EDGE | EXT_CH_MODE_AUTOSTART, extAttention},	// 11
    {EXT_CH_MODE_DISABLED, NULL},	// 12
    {EXT_CH_MODE_DISABLED, NULL},	// 13
    {EXT_CH_MODE_DISABLED, NULL},	// 14
    {EXT_CH_MODE_DISABLED, NULL},	// 15
    {EXT_CH_MODE_DISABLED, NULL},	// PVD
    {EXT_CH_MODE_DISABLED, NULL},	// RTC Alarm
    {EXT_CH_MODE_DISABLED, NULL},	// USB OTG FS
    {EXT_CH_MODE_DISABLED, NULL},	// Ethernet Wake
    {EXT_CH_MODE_DISABLED, NULL},	// USB OTG HS
    {EXT_CH_MODE_DISABLED, NULL},	// RTC Tamper and Timestamp
    {EXT_CH_MODE_DISABLED, NULL}	// RTC Wakeup
  }
};



void resetXbee(void){
	palSetPadMode(GPIOB, GPIOB_SPI2_MOSI, PAL_MODE_ALTERNATE(5) | PAL_STM32_OSPEED_HIGHEST);
	palSetPadMode(GPIOB, GPIOB_SPI2_MISO, PAL_MODE_ALTERNATE(5) | PAL_STM32_OSPEED_HIGHEST);
	palSetPadMode(GPIOB, GPIOB_SPI2_SCLK, PAL_MODE_ALTERNATE(5) | PAL_STM32_OSPEED_HIGHEST);
	palSetPadMode(GPIOB, GPIOB_SPI2_SS, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST);
	palSetPadMode(GPIOB, GPIOB_nATTN, PAL_MODE_INPUT_PULLDOWN);

	

	// palSetPadMode(GPIOE,GPIOE_DOUT, PAL_MODE_INPUT);
	palSetPadMode(GPIOE,GPIOE_DOUT, PAL_MODE_OUTPUT_PUSHPULL);
	palSetPadMode(GPIOE,GPIOE_nRESET, PAL_MODE_OUTPUT_PUSHPULL);

	extStart(&EXTD1, &dataAvailableEXTConfig);

	palClearPad(GPIOE, GPIOE_nRESET);
	chThdSleepMilliseconds(100);
	// palSetPadMode(GPIOE,GPIOE_DOUT, PAL_MODE_OUTPUT_PUSHPULL);
	palClearPad(GPIOE, GPIOE_DOUT);
	chThdSleepMilliseconds(200);
	palSetPad(GPIOE,GPIOE_nRESET);
	chThdSleepMilliseconds(100);
	
	while(!dataAvailable){
		asm("nop");
	}
	
}


static WORKING_AREA(xbeeS6LowLevelArea,1024);
static WORKING_AREA(xbeeS6HighLevelArea,1024);

msg_t xbeeLowLevelThread(void *arg){
	(void) arg;
	
	uint8_t rxBuff[256];

	while(TRUE){
		//If there is data to receive, try to get it
		if (dataAvailable){
			spiAcquireBus(&SPID2);
			spiStart(&SPID2,&spi2Config);

			while(palReadPad(GPIOB,GPIOB_nATTN) == 0){
				spiSelect(&SPID2);
				spiReceive(&SPID2,1,&rxBuff);
				//if 0x7E is received, this marks the start of a message
				//Otherwise the byte gets discarded
				if (rxBuff[0] == 0x7E){
					//grab the length of the packet
					spiReceive(&SPID2,2,&rxBuff[1]);
					uint16_t length = (rxBuff[1]<<8) + rxBuff[2];
					//receive length plus one more byte for the checksum
					spiReceive(&SPID2,length + 1,&rxBuff[3]);
				}
				spiUnselect(&SPID2);
				
			}
			spiReleaseBus(&SPID2);
			dataAvailable = FALSE;
		}
		// If there is data available send it out
		if (dataTxSize > 0){

		}
	}
}

msg_t xbeeHighLevelThread(void *arg){
	(void) arg;
}

msg_t xbeeInitThread(void *arg){
	(void) arg;

	resetXbee();

	chThdCreateStatic(xbeeS6LowLevelArea,sizeof(xbeeS6LowLevelArea),
                    NORMALPRIO,xbeeLowLevelThread,NULL);
	chThdCreateStatic(xbeeS6HighLevelArea,sizeof(xbeeS6HighLevelArea),
                    NORMALPRIO,xbeeHighLevelThread,NULL);	

	return 0;
}