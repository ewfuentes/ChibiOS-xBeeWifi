#ifndef _XBEES6_H_
#define _XBEES6_H_

msg_t xbeeInitThread(void *arg);

typedef struct{
	uint16_t length;
	uint8_t checksum;
	uint8_t _reserved;
	uint8_t data[128];
} XbeeMsg;

#endif /* _XBEES6_H_ */