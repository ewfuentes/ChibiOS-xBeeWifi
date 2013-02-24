#ifndef _XBEES6_H_
#define _XBEES6_H_

#define SECURITY_TYPE_NONE 	0
#define SECURITY_TYPE_WPA	1
#define SECURITY_TYPE_WPA2	2
#define SECURITY_TYPE_WEP	3

#define CNXN_NOT_CONNECTED	0
#define CNXN_SEARCHING 		1
#define CNXN_SSID_FOUND		2
#define CNXN_CONNECTED 		3

#define SSID_TO_CONNECT "Bring Me Cookies!"
#define PASSWORD "conner3colony"

#define SECURITY SECURITY_TYPE_WPA2
msg_t xbeeInitThread(void *arg);
uint8_t connectionStatus(void);



typedef struct{
	uint16_t length;
	uint8_t checksum;
	uint8_t _reserved;
	uint8_t data[128];
} XbeeMsg;

typedef struct{
	union{
		struct{
			uint8_t IP1;
			uint8_t IP2;
			uint8_t IP3;
			uint8_t IP4;
		}byte;
		uint32_t num;
	}IP;
	uint16_t port;
} IPAddr;

void txPacket(IPAddr *ip, uint8_t *data,uint16_t length);
#endif /* _XBEES6_H_ */