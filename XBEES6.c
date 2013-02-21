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

#define MAILBOX_SIZE 10
#define MESSAGE_POOL_SIZE MAILBOX_SIZE * 2
Mailbox unusedMessagesMB;
Mailbox toLowLevelMB;
Mailbox toHighLevelMB;


msg_t unusedMessagesMBBuf[MESSAGE_POOL_SIZE];
msg_t toLowLevelMBBuf[MAILBOX_SIZE];
msg_t toHighLevelMBBuf[MAILBOX_SIZE];

static XbeeMsg messagePool[MESSAGE_POOL_SIZE];

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
//Private Function Prototypes
void resetXbee(void);
msg_t xbeeLowLevelThread(void *arg);
msg_t xbeeHighLevelThread(void *arg);
void processModemStatus(XbeeMsg *msg);
void sendATCommand(void);
void sendFrame(XbeeMsg *msg);

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
	chRegSetThreadName("xBeeLow");
	static uint8_t rxBuff[256];
	msg_t msgPtr = (msg_t)NULL;
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
					spiReceive(&SPID2,2,&rxBuff[0]);
					uint16_t length = (rxBuff[0]<<8) + rxBuff[1];
					//receive length plus one more byte for the checksum
					spiReceive(&SPID2,length + 1,&rxBuff[2]);
					uint16_t i;
					msgPtr = (msg_t)NULL;
					msg_t result = chMBFetch(&unusedMessagesMB,&msgPtr,TIME_IMMEDIATE);
					if (result == RDY_OK){
						
						XbeeMsg *toHighLevel = (XbeeMsg *) msgPtr;
						
						toHighLevel->length = (rxBuff[0]<<8) + rxBuff[1];
						uint8_t checksum = 0;
						for (i = 0; i < toHighLevel->length; i++){
							toHighLevel->data[i] = rxBuff[i+2];
							checksum += rxBuff[i+2];
						}
						
						toHighLevel->checksum = rxBuff[toHighLevel->length+2];
						checksum += toHighLevel->checksum;

						if (checksum == 0xFF){
							chMBPost(&toHighLevelMB,((msg_t)toHighLevel),TIME_IMMEDIATE);
						}
					}
				}
				spiUnselect(&SPID2);
			}
			spiReleaseBus(&SPID2);
			dataAvailable = FALSE;
		}
		// If there is data available send it out
		msgPtr = (msg_t)NULL;
		if (chMBFetch(&toLowLevelMB,&msgPtr,TIME_IMMEDIATE) == RDY_OK){
			XbeeMsg *toSend = (XbeeMsg *)msgPtr;
			spiAcquireBus(&SPID2);
			spiStart(&SPID2,&spi2Config);
			spiSelect(&SPID2);

			uint8_t delim = 0x7E;
			uint8_t length[] = {toSend->length>>8, toSend->length};
			spiSend(&SPID2, 1, &delim);
			spiSend(&SPID2, 2, &length);
			spiSend(&SPID2, toSend->length, &(toSend->data));
			spiSend(&SPID2, 1, &(toSend->checksum));

			spiUnselect(&SPID2);
			spiReleaseBus(&SPID2);
		}
		chThdSleepMilliseconds(10);
	}
}

void sendFrame(XbeeMsg *msg){
	msg_t msgPtr;
	if (chMBFetch(&unusedMessagesMB,&msgPtr,TIME_IMMEDIATE) == RDY_OK){
		XbeeMsg *toFill = (XbeeMsg *) msgPtr;
		//calculating checksum and filling the buffer
		uint16_t i = 0;
		toFill->checksum = 0;
		toFill->length = msg->length;
		for (i = 0; i< msg->length; i++){
			toFill->checksum += msg->data[i];
			toFill->data[i] = msg->data[i];
		}
		toFill->checksum = 0xFF - toFill->checksum;
		chMBPost(&toLowLevelMB,((msg_t)toFill),TIME_IMMEDIATE);
	}

}

void sendATCommand(){
	XbeeMsg msg;
	msg.length = 4;
	msg.data[0] = 0x08;
	msg.data[1] = 0x01;
	msg.data[2] = 'A';
	msg.data[3] = 'S';

	sendFrame(&msg);
}

void processModemStatus(XbeeMsg *msg){
	uint8_t status = msg->data[1];
	if (status == 0x00){//Hardware Reset or Power Up
		sendATCommand();
	}else if (status == 0x01){//Watchdog timer reset

	}else if (status == 0x02){//Joined

	}else if (status == 0x03){//No longer joined to access point

	}else if (status == 0x04){//IP configuration error

	}else if (status == 0x82){//Send or join command issued without first connecting from access point

	}else if (status == 0x83){//Access point not found

	}else if (status == 0x84){//PSK not configured

	}else if (status == 0x87){//SSID not found

	}else if (status == 0x88){//Failed to join with security enabled

	}else if (status == 0x8A){//Invalid Channel

	}else if (status == 0x8E){//Failed to join access point0

	}
}



void processXbeeMessage(XbeeMsg *msg){
	(void) msg;
	uint8_t frameType = msg->data[0];
	if (frameType == 0x00){//Tx64 Request

	}else if (frameType == 0x08){//AT Command
	
	}else if (frameType == 0x09){//AT Command - Queue Parameter Value

	}else if (frameType == 0x07){//Remote Command Request

	}else if (frameType == 0x20){//TX IPv4

	}else if (frameType == 0x80){//Rx64 Indicator

	}else if (frameType == 0x88){//AT Command Response

	}else if (frameType == 0x89){//TX Status

	}else if (frameType == 0x8A){//Modem Status
		processModemStatus(msg);
	}else if (frameType == 0x8F){//IO Data Sample Rx Indicator

	}else if (frameType == 0x87){//Remote Command Response

	}else if (frameType == 0xB0){//RX IPv4

	}
	//Recycle the message container
	chMBPost(&unusedMessagesMB,(msg_t)msg,TIME_IMMEDIATE);
}

msg_t xbeeHighLevelThread(void *arg){
	(void) arg;
	chRegSetThreadName("xBeeHigh");
	msg_t msgPtr=0;
	while(TRUE){
		if (chMBFetch(&toHighLevelMB, &msgPtr, 5000)){
			XbeeMsg *msg = (XbeeMsg *)msgPtr;
			processXbeeMessage(msg);

		}
	}
}

msg_t xbeeInitThread(void *arg){
	(void) arg;

	resetXbee();

	chMBInit(&unusedMessagesMB,unusedMessagesMBBuf,MESSAGE_POOL_SIZE);
	chMBInit(&toHighLevelMB, toHighLevelMBBuf, MAILBOX_SIZE);
	chMBInit(&toLowLevelMB, toLowLevelMBBuf, MAILBOX_SIZE);

	uint8_t i= 0;
	for (i = 0; i < MESSAGE_POOL_SIZE; i++){
		chMBPost(&unusedMessagesMB,(msg_t)(&messagePool[i]),TIME_IMMEDIATE);
	}

	chThdCreateStatic(xbeeS6LowLevelArea,sizeof(xbeeS6LowLevelArea),
                    NORMALPRIO,xbeeLowLevelThread,NULL);
	chThdCreateStatic(xbeeS6HighLevelArea,sizeof(xbeeS6HighLevelArea),
                    NORMALPRIO,xbeeHighLevelThread,NULL);	

	return 0;
}