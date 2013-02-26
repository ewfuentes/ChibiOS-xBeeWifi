#include "ch.h"
#include "hal.h"
#include "XBEES6.h"
#include <string.h>

#define GPIOE_DOUT 10
#define GPIOE_nRESET 9

#define GPIOB_nATTN 	11
#define GPIOB_SPI2_SS   12
#define GPIOB_SPI2_SCLK 13
#define GPIOB_SPI2_MISO 14
#define GPIOB_SPI2_MOSI 15 

#define BUFFER_SIZE 256

#define FRAME_TX64				0x00
#define FRAME_AT_COMMAND		0x08
#define FRAME_AT_COMMAND_QUEUE	0x09
#define FRAME_AT_REMOTE_COMMAND	0x07
#define FRAME_TX_IPV4			0x20
#define FRAME_RX64				0x80
#define FRAME_AT_COMMAND_RESP	0x88
#define FRAME_TX_STATUS			0x89
#define FRAME_MODEM_STATUS		0x8A
#define FRAME_IO_SAMPLE_RX		0x8F
#define FRAME_AT_REMOTE_RESP	0x87
#define FRAME_RX_IPV4			0xB0

volatile uint8_t dataAvailable = TRUE;
volatile uint16_t dataTxSize = 0;
volatile uint8_t connection = CNXN_NOT_CONNECTED;

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
void processATCommandResponse(XbeeMsg *msg);
void scanWifiNetworks(void);
void sendFrame(XbeeMsg *msg);
void connectToNetwork(void);
void getIPaddress(void);
char ipAddress[20];
void processRXFrame(XbeeMsg *msg);


uint8_t connectionStatus(){
	return connection;
}

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
			spiSelect(&SPID2);
			while(palReadPad(GPIOB,GPIOB_nATTN) == 0){
				
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
				
			}
			spiUnselect(&SPID2);
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

			chMBPost(&unusedMessagesMB,(msg_t) msgPtr,TIME_IMMEDIATE);
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

void scanWifiNetworks(){
	XbeeMsg msg;
	msg.length = 4;
	msg.data[0] = 0x08;
	msg.data[1] = 0x01;
	msg.data[2] = 'A';
	msg.data[3] = 'S';

	sendFrame(&msg);
}

void txPacket(IPAddr *ip, uint8_t *data,uint16_t length){
	XbeeMsg msg;
	msg.length = length + 12;
	msg.data[0] = 0x20;
	msg.data[1] = 0x01;
	msg.data[2] = ip->IP.byte.IP1;
	msg.data[3] = ip->IP.byte.IP2;
	msg.data[4] = ip->IP.byte.IP3;
	msg.data[5] = ip->IP.byte.IP4;
	msg.data[6] = (ip->port>>8) & 0xFF;
	msg.data[7] = ip->port & 0xFF;
	msg.data[8] = 0x26;
	msg.data[9] = 0x16;
	msg.data[10] = 0;
	msg.data[11] = 0;
	uint8_t i = 0;
	for (i = 0; i < length; i++){
		msg.data[i+12] = data[i];
	}

	sendFrame(&msg);
}

void connectToNetwork(){
	//Set SSID
	XbeeMsg msg;
	msg.data[0] = FRAME_AT_COMMAND;
	msg.data[1] = 1;
	msg.data[2] = 'I';
	msg.data[3] = 'D';
	char *ssid = SSID_TO_CONNECT;
	uint8_t i;
	for (i = 0; ssid[i] != '\0'; i++){
		msg.data[i+4] = ssid[i];
	}
	msg.length = i + 4;

	sendFrame(&msg);

	//Set Encryption
	msg.data[1] = 2;
	msg.data[2] = 'E';
	msg.data[3] = 'E';
	msg.data[4] = SECURITY;
	msg.length = 5;

	sendFrame(&msg);

	//Set Security Key
	msg.data[1] = 3;
	msg.data[2] = 'P';
	msg.data[3] = 'K';
	char *pk = PASSWORD;
	for (i = 0; pk[i] != '\0'; i++){
		msg.data[i+4] = pk[i];
	}
	msg.length = i + 4;

	sendFrame(&msg);
}

void getIPaddress(){
	XbeeMsg msg;
	msg.length = 4;
	msg.data[0] = 0x08;
	msg.data[1] = 0x01;
	msg.data[2] = 'M';
	msg.data[3] = 'Y';

	sendFrame(&msg);
}

void processModemStatus(XbeeMsg *msg){
	uint8_t status = msg->data[1];
	if (status == 0x00){//Hardware Reset or Power Up
		connection = CNXN_SEARCHING;
		scanWifiNetworks();
	}else if (status == 0x01){//Watchdog timer reset

	}else if (status == 0x02){//Joined
		connection = CNXN_CONNECTED;
		getIPaddress();
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

void processATCommandResponse(XbeeMsg *msg){
	char command[] = {(char)msg->data[2],(char) msg->data[3], '\0'};
	uint8_t status = msg->data[4];
	if (strcmp((char *)command,"DL") == 0){
		//Destination Address Low
		//Range: 0.0.0.0 - 255.255.255.255
		//Default: 255.255.255.255
	}else if (strcmp((char *)command,"MY") == 0){
		//IP Network Address
		//Range: 0.0.0.0 - 255.255.255.255
		//Default: 0.0.0.0
		if (connectionStatus() == CNXN_CONNECTED){
			msg->data[msg->length]= '\0';
			strcpy(ipAddress,(char *)&msg->data[5]);
		}

	}else if (strcmp((char *)command,"MK") == 0){
		//IP Address Mask
		//Range: 0.0.0.0 - 255.255.255.255
		//Default: 0.0.0.0
	}else if (strcmp((char *)command,"GW") == 0){
		//Gateway IP Address
		//Range: 0.0.0.0 - 255.255.255.255
		//Default: 0.0.0.0
	}else if (strcmp((char *)command,"SH") == 0){
		//Serial Number High
		//Range: 0 - 0xFFFFFFFF [Read Only]
		//Default: Factory Set
	}else if (strcmp((char *)command,"SL") == 0){
		//Serial Number Low 
		//Range: 0 - 0xFFFFFFFF [Read Only]
		//Default: Factory Set
	}else if (strcmp((char *)command,"NI") == 0){
		//Node Identifier
		//Range 20-Byte printable ascii string
		//Default: ASCII space character (0x20)
	}else if (strcmp((char *)command,"DE") == 0){
		//Destination Port
		//Range: 0 - 0xFFFF
		//Default: 0x2616
	}else if (strcmp((char *)command,"C0") == 0){
		//Serial Communication Port
		//Range: 0 - 0xFFFF
	}else if (strcmp((char *)command,"DD") == 0){
		//Device Type Identifier
		//Range: 0 - 0xFFFFFFFF
		//Default: 0x50000
	}else if (strcmp((char *)command,"NP") == 0){
		//Max RF Payload Bytes
		//Range: 0 - 0xFFFF [Read Only]
	}else if (strcmp((char *)command,"ID") == 0){
		//SSID
		//Range: 31 bytes of printable ASCII
		//Default: NULL
	}else if (strcmp((char *)command,"AH") == 0){
		//Network Type
		//Range: 0 - IBSS Joiner, 1 - IBSS Creator, 2 - Infrastructure
		//Default: 2 - Infrastructure
	}else if (strcmp((char *)command,"IP") == 0){
		//IP Protocol
		//Range: 0 - UDP, 1 - TCP
		//Default: 0 - UDP
	}else if (strcmp((char *)command,"MA") == 0){
		//IP Addressing Mode
		//Range: 0 - DHCP, 1 - Static
		//Default: 0 - DHCP
	}else if (strcmp((char *)command,"TM") == 0){
		//TCP Timeout
		//Range: 0-0xFF (x 100 msec)
		//Default: 0x0A
	}else if (strcmp((char *)command,"EE") == 0){
		//Encryption Enable
		//Range: 0 - No Security, 1 - WPA, 2 - WPA2, 3 - WEP
		//Default: 0 - No Security
	}else if (strcmp((char *)command,"PK") == 0){
		//Security Key
		//Range: 0-31 ASCII Characters for WPA and WPA2
		//		 5 or 13 ASCII Characters for WEP
	}else if (strcmp((char *)command,"PL") == 0){
		//Power Level
		//Range: 0,1,2 - 7 dBm, 3 - 10 dBm, 4 - 15 dBm
		//Default: 4 - 15 dBm
	}else if (strcmp((char *)command,"CH") == 0){
		//Channel
		//Range: 1 - 0xE [Read Only]
	}else if (strcmp((char *)command,"BR") == 0){
		//Bit Rate of IBSS Creator
		//Range: 0 - Autorate, 1 - 1 Mbps, 2 - 2 Mbps, 3 - 5.5 Mbps
		//		 4 - 11 Mbps, 5 - 6 Mbps, 6 - 9 Mbps,  7 - 12 Mbps
		//		 8 - 18 Mbps, 9 - 24 Mbps, 0xA - 36 Mbps, 0xB - 48 Mbps
		//		 0xC - 54 Mbps, 0xD - MCS0, 0xE - MCS1, 0xF - MCS2
		//		 0x10 - MCS3, 0x11 - MCS4, 0x12 - MCS5, 0x13 - MCS6
		// 		 0x14 - MCS7
		//Default: 0 - Auto-rate
	}else if (strcmp((char *)command,"AP") == 0){
		//API Enable
		//Range: 0 - Transparent, 1 - API Enabled, 2 - API w/ Escape Chars
		//Default: 1 - API Enabled
	}else if (strcmp((char *)command,"BD") == 0){
		//Interface Data Rate
		//Range: 0 - 1200 bps, 1 - 2400 bps, 2 - 4800 bps, 3 - 9600 bps
		//		 4 - 19200 bps, 5 - 38400 bps, 6 - 57600 bps, 7 - 155200 bps
		//		 8 - 230400 bps, 0x100 - 0xE1000 Non Standard rate in bps
		//Default: 3 - 9600 bps
	}else if (strcmp((char *)command,"NB") == 0){
		//Serial Parity
		//Range: 0 - None, 1 - Even, 2 - Odd
		//Default: 0 - None
	}else if (strcmp((char *)command,"SB") == 0){
		//Stop Bits
		//Range: 0 - 1 stop bit, 1 - 2 Stop Bits
		//Default: 0 - 1 stop bit
	}else if (strcmp((char *)command,"RO") == 0){
		//Packetization Timeout
	}else if (strcmp((char *)command,"FT") == 0){
		//Flow Control Threshold
	}else if (strcmp((char *)command,"D7") == 0){
		//DIO7 Configuration
	}else if (strcmp((char *)command,"D6") == 0){
		//DIO6 Configuration
	}else if (strcmp((char *)command,"IS") == 0){
		//Force Sample
	}else if (strcmp((char *)command,"IR") == 0){
		//IO Sample Rate
	}else if (strcmp((char *)command,"IC") == 0){
		//IO Digital Change Detection
	}else if (strcmp((char *)command,"IF") == 0){
		//Sample from Sleep Rate
	}else if (strcmp((char *)command,"P0") == 0){
		//DIO10 Configuration
	}else if (strcmp((char *)command,"P1") == 0){
		//DIO11 Configuration
	}else if (strcmp((char *)command,"P2") == 0){
		//DIO12 Configuration
	}else if (strcmp((char *)command,"P3") == 0){
		//DOUT Configuration
	}else if (strcmp((char *)command,"P4") == 0){
		//DIN Configuration
	}else if (strcmp((char *)command,"D0") == 0){
		//AD0/DIO0 Configuration
	}else if (strcmp((char *)command,"D1") == 0){
		//AD1/DIO1 Configuration
	}else if (strcmp((char *)command,"D2") == 0){
		//AD2/DIO2 Configuration
	}else if (strcmp((char *)command,"D3") == 0){
		//AD3/DIO3 Configuration
	}else if (strcmp((char *)command,"D4") == 0){
		//DIO4 Configuration
	}else if (strcmp((char *)command,"D5") == 0){
		//DIO5 Configuration
	}else if (strcmp((char *)command,"D8") == 0){
		//DIO8 Configuration
	}else if (strcmp((char *)command,"D9") == 0){
		//DIO9 Configuration
	}else if (strcmp((char *)command,"LT") == 0){
		//Assoc LED Blink Time
	}else if (strcmp((char *)command,"PR") == 0){
		//Pull Up Resistor
	}else if (strcmp((char *)command,"PD") == 0){
		//Pull Direction
	}else if (strcmp((char *)command,"AV") == 0){
		//Analog Voltage Reference
	}else if (strcmp((char *)command,"M0") == 0){
		//PWM0 Duty Cycle
	}else if (strcmp((char *)command,"M1") == 0){
		//PWM1 Duty Cycle
	}else if (strcmp((char *)command,"VR") == 0){
		//Firmware Version
	}else if (strcmp((char *)command,"HV") == 0){
		//Hardware Version
	}else if (strcmp((char *)command,"AI") == 0){
		//Association Indication
	}else if (strcmp((char *)command,"AS") == 0){
		//Active Scan
		if (status == 0x00){
			uint8_t scanType = msg->data[5];
			uint8_t _reserved = msg->data[6];
			uint8_t securityType = msg->data[7];
			uint8_t sigStrength = msg->data[8];
			char ssidName[msg->length-9 + 1];
			uint16_t i = 9;
			for (i = 9; i<msg->length; i++){
				ssidName[i-9] = msg->data[i];
			}
			ssidName[i-9] = '\0';
			if (strcmp(ssidName,SSID_TO_CONNECT) == 0){
				connection = CNXN_SSID_FOUND;
				connectToNetwork();
			}

			(void)scanType;
			(void)_reserved;
			(void)securityType;
			(void)sigStrength;
		}else if(status == 0x01){
			scanWifiNetworks();
		}
	}else if (strcmp((char *)command,"TP") == 0){
		//Temperature
	}else if (strcmp((char *)command,"CK") == 0){
		//Configuration Code
	}else if (strcmp((char *)command,"%V") == 0){
		//Supply Voltage
	}else if (strcmp((char *)command,"DB") == 0){
		//RSSI (Signal Strength)
	}else if (strcmp((char *)command,"CT") == 0){
		//Command Mode Timeout
	}else if (strcmp((char *)command,"CN") == 0){
		//Exit Command Mode
	}else if (strcmp((char *)command,"GT") == 0){
		//Guard Times
	}else if (strcmp((char *)command,"CC") == 0){
		//Command Mode Character
	}else if (strcmp((char *)command,"SM") == 0){
		//Sleep Mode
	}else if (strcmp((char *)command,"SP") == 0){
		//Sleep Period
	}else if (strcmp((char *)command,"SO") == 0){
		//Sleep Options
	}else if (strcmp((char *)command,"WH") == 0){
		//Wake Host
	}else if (strcmp((char *)command,"ST") == 0){
		//Wake Time
	}else if (strcmp((char *)command,"AC") == 0){
		//Apply Changes
	}else if (strcmp((char *)command,"WR") == 0){
		//Write
	}else if (strcmp((char *)command,"RE") == 0){
		//Restore Defaults
	}else if (strcmp((char *)command,"FR") == 0){
		//Software Reset
	}else if (strcmp((char *)command,"NR") == 0){
		//Network Reset
	}
}

volatile uint8_t payload[128];
volatile uint8_t dataRx = FALSE;
volatile uint16_t dataLen;
void processRXFrame(XbeeMsg *msg){
	IPAddr source;
	source.IP.byte.IP1 = msg->data[1];
	source.IP.byte.IP2 = msg->data[2];
	source.IP.byte.IP3 = msg->data[3];
	source.IP.byte.IP4 = msg->data[4];
	source.port = (msg->data[7]<<8) + msg->data[8];
	(void)source;
	uint8_t protocol = msg->data[9];
	(void)protocol;
	uint16_t i;
	for (i = 0; i + 11< msg->length; i++){
		payload[i] = msg->data[i+11];
	}
	dataLen = i;
	for (;i<128; i++){
		payload[i] = 0;
	}
	dataRx = TRUE;
}

void processXbeeMessage(XbeeMsg *msg){
	(void) msg;
	uint8_t frameType = msg->data[0];
	if (frameType == FRAME_TX64){//Tx64 Request

	}else if (frameType == FRAME_AT_COMMAND){//AT Command
	
	}else if (frameType == FRAME_AT_COMMAND_QUEUE){//AT Command - Queue Parameter Value

	}else if (frameType == FRAME_AT_REMOTE_COMMAND){//Remote Command Request

	}else if (frameType == FRAME_TX_IPV4){//TX IPv4

	}else if (frameType == FRAME_RX64){//Rx64 Indicator

	}else if (frameType == FRAME_AT_COMMAND_RESP){//AT Command Response
		processATCommandResponse(msg);
	}else if (frameType == FRAME_TX_STATUS){//TX Status

	}else if (frameType == FRAME_MODEM_STATUS){//Modem Status
		processModemStatus(msg);
	}else if (frameType == FRAME_IO_SAMPLE_RX){//IO Data Sample Rx Indicator

	}else if (frameType == FRAME_AT_REMOTE_COMMAND){//Remote Command Response

	}else if (frameType == FRAME_RX_IPV4){//RX IPv4
		processRXFrame(msg);
	}
	
}

msg_t xbeeHighLevelThread(void *arg){
	(void) arg;
	chRegSetThreadName("xBeeHigh");
	msg_t msgPtr=0;
	while(TRUE){
		msgPtr = (msg_t)NULL;
		if (chMBFetch(&toHighLevelMB, &msgPtr, 10) == RDY_OK){
			XbeeMsg *msg = (XbeeMsg *)msgPtr;
			processXbeeMessage(msg);

			//Recycle the message container		
			chMBPost(&unusedMessagesMB,(msg_t)msg,TIME_IMMEDIATE);
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