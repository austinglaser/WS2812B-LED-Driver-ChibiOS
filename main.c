/*
    ChibiOS/RT - Copyright (C) 2006-2013 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

#include "ch.h"
#include "hal.h"
#include "test.h"
#include "LEDDriver.h"

#define N_LEDS 20


/*
 * Application entry point.
 */
int main(void) {

  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  halInit();
  chSysInit();
  uint8_t *o_fb;

  /*
   * Initialize LedDriver - 150 leds in chain, GPIOA pin 1
   */
  ledDriverInit(N_LEDS, GPIOA, 0b00000010, &o_fb);
  testPatternFB(o_fb);

  /*
   * Activates the serial driver 1 using the driver default configuration.
   * PA9(TX) and PA10(RX) are routed to USART1.
   */
  sdStart(&SD1, NULL);
  palSetPadMode(GPIOA, 9, PAL_MODE_ALTERNATE(7));
  palSetPadMode(GPIOA, 10, PAL_MODE_ALTERNATE(7));

  const Color red   = {24,  0,  0};
  const Color green = { 0, 24,  0};
  const Color blue  = { 0,  0, 24};
  const Color white = {24, 24, 24};
  const Color black = { 0,  0,  0};

  //setColorRGBLocation(black, 0, 0, 0);
  //setColorRGBLocation(black, 0, 0, 1);
  //setColorRGBLocation(black, 0, 0, 2);
  //setColorRGBLocation(black, 0, 1, 0);
  //setColorRGBLocation(black, 0, 1, 1);
  //setColorRGBLocation(black, 0, 1, 2);
  //setColorRGBLocation(black, 0, 2, 0);
  //setColorRGBLocation(black, 0, 2, 1);
  //setColorRGBLocation(black, 0, 2, 2);
  //setColorRGBLocation(black, 0, 3, 0);
  //setColorRGBLocation(black, 1, 0, 0);
  //setColorRGBLocation(black, 1, 0, 1);
  //setColorRGBLocation(black, 1, 0, 2);
  //setColorRGBLocation(black, 1, 1, 0);
  //setColorRGBLocation(black, 1, 1, 1);
  //setColorRGBLocation(black, 1, 1, 2);
  //setColorRGBLocation(black, 1, 2, 0);
  //setColorRGBLocation(black, 1, 2, 1);
  //setColorRGBLocation(black, 1, 2, 2);
  //setColorRGBLocation(black, 1, 3, 0);

  while (TRUE) {
    // sides
    int i, j;
    for (i = 0; i < 4; i++) {
      for (j = 0; j < 3; j++) {
        setColorRGBLocation(green, 0, i, j);
        setColorRGBLocation(blue, 1, i, j);
      }
    }

    chThdSleepMilliseconds(1000);

    // regions
    for (i = 0; i < 4; i++) {
      for (j = 0; j < 3; j++) {
        if (i == 0) {
          setColorRGBLocation(red, 0, i, j);
          setColorRGBLocation(red, 1, i, j);
        }
        if (i == 1) {
          setColorRGBLocation(green, 0, i, j);
          setColorRGBLocation(green, 1, i, j);
        }
        if (i == 2) {
          setColorRGBLocation(blue, 0, i, j);
          setColorRGBLocation(blue, 1, i, j);
        }
        if (i == 3) {
          setColorRGBLocation(white, 0, i, j);
          setColorRGBLocation(white, 1, i, j);
        }
      }
    }

    chThdSleepMilliseconds(1000);

    // indices
    for (i = 0; i < 4; i++) {
      for (j = 0; j < 3; j++) {
        if (j == 0) {
          setColorRGBLocation(red, 0, i, j);
          setColorRGBLocation(red, 1, i, j);
        }
        if (j == 1) {
          setColorRGBLocation(green, 0, i, j);
          setColorRGBLocation(green, 1, i, j);
        }
        if (j == 2) {
          setColorRGBLocation(blue, 0, i, j);
          setColorRGBLocation(blue, 1, i, j);
        }
      }
    }

    chThdSleepMilliseconds(1000);

    // off
    for (i = 0; i < 4; i++) {
      for (j = 0; j < 3; j++) {
        setColorRGBLocation(black, 0, i, j);
        setColorRGBLocation(black, 1, i, j);
      }
    }

    chThdSleepMilliseconds(1000);
  }

}
