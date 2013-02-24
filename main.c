#include "ch.h"
#include "hal.h"
#include "XBEES6.h"
#include <stdio.h>

#define TARGET_IP1 192
#define TARGET_IP2 168
#define TARGET_IP3 1
#define TARGET_IP4 121
#define TARGET_IP {TARGET_IP1, TARGET_IP2, TARGET_IP3,TARGET_IP4}
#define TARGET_PORT 0x7530

static WORKING_AREA(xbeeS6WorkingArea,128);

extern char ipAddress[20];
extern volatile uint8_t dataRx;

IPAddr ip = {
  {TARGET_IP},
  TARGET_PORT,
};

int main(void) {

  halInit();
  chSysInit();
  chRegSetThreadName("MAIN");
  chThdCreateStatic(xbeeS6WorkingArea,sizeof(xbeeS6WorkingArea),
                    NORMALPRIO,xbeeInitThread,NULL);

  int32_t delay = 10;

  uint16_t count = 0;

  char out [7];
  

  while (TRUE) {
    if (connectionStatus() == CNXN_NOT_CONNECTED){
      delay = 50;
    }else if(connectionStatus() == CNXN_SEARCHING){
      delay = 100;
    }else if(connectionStatus() == CNXN_SSID_FOUND){
      delay = 250;
    }else if(connectionStatus() == CNXN_CONNECTED){
      delay = 500;
      uint8_t len = snprintf(out,7,"%X",count);
      out[len] = '\r';
      out[len+1] = '\n';
      txPacket(&ip,(uint8_t *)out, len+2);
      if (dataRx == TRUE){
        dataRx = FALSE;

        ipAddress[18] = (char)'\r';
        ipAddress[19] = (char)'\n';
        txPacket(&ip,(uint8_t *)ipAddress,20);  
      }
      
    }
    palSetPad(GPIOD, GPIOD_LED4);
    chThdSleepMilliseconds(delay);
    palClearPad(GPIOD, GPIOD_LED4);
    chThdSleepMilliseconds(delay);
    count++;
  }
}
