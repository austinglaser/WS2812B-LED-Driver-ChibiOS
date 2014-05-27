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

#include <stdio.h>
#include <string.h>

#include "ch.h"
#include "hal.h"
#include "test.h"
#include "math.h"
#include "arm_math.h"
#include "shell.h"
#include "chprintf.h"
#include "led_driver.h"

#define N_LEDS          20
#define FFT_LENGTH      1024
#define TWO_PI          (M_PI * 2.0f)
#define SAMPLING_RATE   46391.75

#define ADC_GRP1_NUM_CHANNELS   1
#define ADC_GRP1_BUF_DEPTH      (FFT_LENGTH*2)

static adcsample_t adc_samples[ADC_GRP1_NUM_CHANNELS * ADC_GRP1_BUF_DEPTH];
float32_t avg;

size_t nx = 0, ny = 0;
static void adccallback(ADCDriver * adcp, adcsample_t * buffer, size_t n)
{
  (void)adcp;

  if (adc_samples == buffer) nx += n;
  else                       ny += n;
} 
static void adcerrorcallback(ADCDriver * adcp, adcerror_t err)
{
  (void) adcp;
  (void) err;
}

static const ADCConversionGroup adcgrpcfg1 = {
  TRUE,                         /* Circular       */
  ADC_GRP1_NUM_CHANNELS,        /* Num Channels   */
  adccallback,                  /* End callback   */
  adcerrorcallback,             /* Error callback */
  0,                            /* CFGR           */
  ADC_TR(0, 4095),              /* TR1            */
  0,                            /* CR             */
  {                             /* SMPR[2]        */
    0,
    ADC_SMPR2_SMP_AN12(ADC_SMPR_SMP_181P5)
  },
  {                             /* SQR[4]         */
    ADC_SQR1_NUM_CH(ADC_GRP1_NUM_CHANNELS) | ADC_SQR1_SQ1_N(ADC_CHANNEL_IN12),
    0,
    0,
    0
  }
};


float32_t average_array(adcsample_t * buffer, size_t len)
{
  float32_t sum = 0;
  uint32_t i;
  for (i = 0; i < len; i++) sum += buffer[i];

  return sum/((float32_t) len);
}

const color_rgb_t red   = {24,  0,  0};
const color_rgb_t green = { 0, 24,  0};
const color_rgb_t blue  = { 0,  0, 24};
const color_rgb_t white = {24, 24, 24};
const color_rgb_t black = { 0,  0,  0};

float32_t samples[FFT_LENGTH*2];
float32_t fft_mag[FFT_LENGTH/2];

static WORKING_AREA(wathread_leds, 512);
  __attribute__ ((__noreturn__))
static msg_t thread_leds(void *arg)
{
  (void) arg;
  chRegSetThreadName("leds");

  /*
   * Use TX1 as a timing output
   */
  palSetPadMode(GPIOA, GPIOA_TX1, PAL_MODE_OUTPUT_PUSHPULL);
  palSetPad(GPIOA, GPIOA_TX1);

  /* 
   * FPU INIT
   */
  const uint32_t ifftFlag = 0;
  const uint32_t doBitReverse = 1;
  arm_cfft_radix2_instance_f32 S;
  float32_t max_value = 0;
  uint32_t max_index = 0;

  arm_cfft_radix2_init_f32(&S, FFT_LENGTH, ifftFlag, doBitReverse);

  /*
   * Main loop
   */
  while (TRUE) {
    palSetPad(GPIOA, GPIOA_TX1);                        //indicate loop beginning

    // read in sample values
    int i, j;
    for (i = 0; i < FFT_LENGTH; i++) {
      samples[i*2]     = ((float32_t) adc_samples[i]) - 2792.0; //real
      samples[i*2 + 1] = 0.0;                                   //complex
    }

    // perform fft
    arm_cfft_radix2_f32(&S, samples);
    palClearPad(GPIOA, GPIOA_TX1);                      //indicate end of fft stage
    arm_cmplx_mag_f32(samples, fft_mag, FFT_LENGTH/2);  //calculate overall magnitude

    // find max value and location between ~226 and ~2000 hz
    arm_max_f32(fft_mag + 20, 85, &max_value, &max_index);

    // figure out float32_ting magnitude
    float32_t magnitude = (max_value/1024.0);

    color_hsv_t max_color = {(((float32_t) max_index - 20.0)/45.0), 1.0, magnitude/8.0};

    /*
    float last = max_value;
    for (i = max_index; last >= fft_mag[i]; i++) {
      last = fft_mag[i];
      fft_mag[i] = 0.0;
    }

    last = max_value;
    for (i = max_index - 1; last >= fft_mag[i]; i--) {
      last = fft_mag[i];
      fft_mag[i] = 0.0;
    }
    */
    fft_mag[max_index] = 0.0;

    color_hsv_t sub_color[3];

    for (i = 0; i < 3; i++)  {
      // find max value and location between ~226 and ~2000 hz
      arm_max_f32(fft_mag + 40, 85, &max_value, &max_index);

      // figure out float32_ting magnitude
      magnitude = (max_value/1024.0);

      sub_color[i].hue = (((float32_t) max_index - 20.0)/45.0);
      sub_color[i].sat = 1.0;
      sub_color[i].val = magnitude/8.0;

      /*
        last = max_value;
        for (i = max_index; last >= fft_mag[i]; i++) {
          last = fft_mag[i];
          fft_mag[i] = 0.0;
        }

        last = max_value;
        for (i = max_index - 1; last >= fft_mag[i]; i--) {
          last = fft_mag[i];
          fft_mag[i] = 0.0;
        }
        */
      fft_mag[max_index] = 0.0;
    }

    color_hsv_t interp_color[3][2];

    interp_color[0][0].hue = (sub_color[0].hue * 2.0 + sub_color[1].hue + max_color.hue * 2.0)/5.0;
    interp_color[0][1].hue = (sub_color[0].hue * 2.0 + sub_color[2].hue + max_color.hue * 2.0)/5.0;
    interp_color[1][0].hue = (sub_color[1].hue * 2.0 + sub_color[2].hue + max_color.hue * 2.0)/5.0;
    interp_color[1][1].hue = (sub_color[1].hue * 2.0 + sub_color[0].hue + max_color.hue * 2.0)/5.0;
    interp_color[2][0].hue = (sub_color[2].hue * 2.0 + sub_color[0].hue + max_color.hue * 2.0)/5.0;
    interp_color[2][1].hue = (sub_color[2].hue * 2.0 + sub_color[1].hue + max_color.hue * 2.0)/5.0;

    interp_color[0][0].sat = (sub_color[0].sat * 2.0 + sub_color[1].sat + max_color.sat * 2.0)/5.0;
    interp_color[0][1].sat = (sub_color[0].sat * 2.0 + sub_color[2].sat + max_color.sat * 2.0)/5.0;
    interp_color[1][0].sat = (sub_color[1].sat * 2.0 + sub_color[2].sat + max_color.sat * 2.0)/5.0;
    interp_color[1][1].sat = (sub_color[1].sat * 2.0 + sub_color[0].sat + max_color.sat * 2.0)/5.0;
    interp_color[2][0].sat = (sub_color[2].sat * 2.0 + sub_color[0].sat + max_color.sat * 2.0)/5.0;
    interp_color[2][1].sat = (sub_color[2].sat * 2.0 + sub_color[1].sat + max_color.sat * 2.0)/5.0;

    interp_color[0][0].val = (sub_color[0].val * 2.0 + sub_color[1].val + max_color.val * 2.0)/5.0;
    interp_color[0][1].val = (sub_color[0].val * 2.0 + sub_color[2].val + max_color.val * 2.0)/5.0;
    interp_color[1][0].val = (sub_color[1].val * 2.0 + sub_color[2].val + max_color.val * 2.0)/5.0;
    interp_color[1][1].val = (sub_color[1].val * 2.0 + sub_color[0].val + max_color.val * 2.0)/5.0;
    interp_color[2][0].val = (sub_color[2].val * 2.0 + sub_color[0].val + max_color.val * 2.0)/5.0;
    interp_color[2][1].val = (sub_color[2].val * 2.0 + sub_color[1].val + max_color.val * 2.0)/5.0;

    set_color_hsv_location(max_color, 0, 3, 0);
    set_color_hsv_location(max_color, 1, 3, 0);

    for (i = 0; i < 3; i++) {
      set_color_hsv_location(sub_color[i], 0, i, 2);
      set_color_hsv_location(sub_color[i], 1, i, 2);

      for (j = 0; j < 2; j++) {
        set_color_hsv_location(interp_color[i][j], 0, i, j);
        set_color_hsv_location(interp_color[i][j], 1, i, j);
      }
    }

    push_buffer_leds();
  }
}


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

  /*
   * Initialize adc thread
   */
  chThdCreateStatic(wathread_leds, sizeof(wathread_leds), NORMALPRIO, thread_leds, NULL);

  uint8_t *o_fb;
  uint16_t gpio_pin1_mask = 0b0000000000000010;
  
  /* 
   * Set gain for mic pre-amp (high = 40 dB, low = 50 dB, float32_ting = 60 dB)
   */
  palSetPadMode(GPIOB, GPIOB_GAIN, PAL_MODE_INPUT);           // float32_ting input = 60 dB
  //palSetPadMode(GPIOB, GPIOB_GAIN, PAL_MODE_OUTPUT_PUSHPULL);   // drive high or low
  //palSetPad(GPIOB, GPIOB_GAIN);                                 // drive high = 40 dB
  //palsClearPad(GPIOB, GPIOB_GAIN);                            // drive low = 50 dB


  /*
   * Initialize LedDriver - 20 leds in chain, GPIOA pin 1
   */
  led_driver_init(N_LEDS, GPIOA, gpio_pin1_mask, &o_fb);

  /*
   * ADC Initialization
   */
  adcStart(&ADCD3, NULL);

  /*
   * Start on continuous conversion
   */
  adcStartConversion(&ADCD3, &adcgrpcfg1, adc_samples, ADC_GRP1_BUF_DEPTH);

  /*
   * Activates the serial driver 1 using the driver default configuration.
   * PA9(TX) and PA10(RX) are routed to USART1.
   */
  sdStart(&SD1, NULL);
  palSetPadMode(GPIOA, 9, PAL_MODE_ALTERNATE(7));
  palSetPadMode(GPIOA, 10, PAL_MODE_ALTERNATE(7));


  while (TRUE) {
    /*
    // sides
    int i, j;
    for (i = 0; i < 4; i++) {
    for (j = 0; j < 3; j++) {
    set_color_rgb_location(green, 0, i, j);
    set_color_rgb_location(blue, 1, i, j);
    }
    }

    chThdSleepMilliseconds(1000);

    // regions
    for (i = 0; i < 4; i++) {
    for (j = 0; j < 3; j++) {
    if (i == 0) {
    set_color_rgb_location(red, 0, i, j);
    set_color_rgb_location(red, 1, i, j);
    }
    if (i == 1) {
    set_color_rgb_location(green, 0, i, j);
    set_color_rgb_location(green, 1, i, j);
    }
    if (i == 2) {
    set_color_rgb_location(blue, 0, i, j);
    set_color_rgb_location(blue, 1, i, j);
    }
    if (i == 3) {
    set_color_rgb_location(white, 0, i, j);
    set_color_rgb_location(white, 1, i, j);
    }
    }
    }

    chThdSleepMilliseconds(1000);

    // indices
    for (i = 0; i < 4; i++) {
    for (j = 0; j < 3; j++) {
    if (j == 0) {
    set_color_rgb_location(red, 0, i, j);
    set_color_rgb_location(red, 1, i, j);
    }
    if (j == 1) {
    set_color_rgb_location(green, 0, i, j);
    set_color_rgb_location(green, 1, i, j);
    }
    if (j == 2) {
    set_color_rgb_location(blue, 0, i, j);
    set_color_rgb_location(blue, 1, i, j);
    }
    }
    }

    chThdSleepMilliseconds(1000);

    // off
    for (i = 0; i < 4; i++) {
    for (j = 0; j < 3; j++) {
    set_color_rgb_location(black, 0, i, j);
    set_color_rgb_location(black, 1, i, j);
    }
    }

    chThdSleepMilliseconds(1000);
     */
  }

}
