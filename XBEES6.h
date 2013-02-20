#ifndef _XBEES6_H_
#define _XBEES6_H_

msg_t xbeeInitThread(void *arg);

typedef struct{
	uint8_t processed;
	uint8_t frameType;
	uint16_t length;
	uint8_t data[100];
} XBeeMsg;

#endif /* _XBEES6_H_ */