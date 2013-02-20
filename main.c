#include "ch.h"
#include "hal.h"
#include "XBEES6.h"

static WORKING_AREA(xbeeS6WorkingArea,128);


int main(void) {

  halInit();
  chSysInit();
  chRegSetThreadName("MAIN");
  chThdCreateStatic(xbeeS6WorkingArea,sizeof(xbeeS6WorkingArea),
                    NORMALPRIO,xbeeInitThread,NULL);

  int32_t delay = 5000;

  while (TRUE) {
    palSetPad(GPIOD, GPIOD_LED4);
    chThdSleepMilliseconds(delay);
    palClearPad(GPIOD, GPIOD_LED4);
    chThdSleepMilliseconds(delay);
  }
}
