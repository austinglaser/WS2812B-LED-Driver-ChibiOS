/*
 * LEDDriver.c
 *
 *  Created on: Aug 26, 2013
 *      Author: Omri Iluz
 */

#include "led_driver.h"

static uint8_t *fb;
static uint8_t *dummy_fb;
static int sLeds;
static GPIO_TypeDef *sPort;
static uint32_t sMask;
uint8_t* dma_source;

void set_color(uint8_t color, uint8_t *buf,uint32_t mask){
  int i;
  for (i=0;i<8;i++){
    buf[i]=((color<<i)&0b10000000?0x0:mask);
  }
}

void _correct_gamma(color_rgb_t * c);
void _correct_white(color_rgb_t * c);

void set_color_rgb(color_rgb_t c, uint8_t *buf, uint32_t mask){
  //_correct_gamma(&c);
  //_correct_white(&c);

  set_color(c.green,buf, mask);
  set_color(c.red,buf+8, mask);
  set_color(c.blue,buf+16, mask);
}

int gamma_lut[] = { 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
                    0,  0,  0,  0,  0,  0,  1,  1,  1,  1,  1,  1,  1,  2,  2,  2,
                    2,  2,  2,  3,  3,  3,  3,  3,  4,  4,  4,  4,  5,  5,  5,  5,
                    6,  6,  6,  7,  7,  7,  8,  8,  8,  9,  9,  9,  10, 10, 11, 11,
                    11, 12, 12, 13, 13, 13, 14, 14, 15, 15, 16, 16, 17, 17, 18, 18,
                    19, 19, 20, 21, 21, 22, 22, 23, 23, 24, 25, 25, 26, 27, 27, 28,
                    29, 29, 30, 31, 31, 32, 33, 34, 34, 35, 36, 37, 37, 38, 39, 40,
                    40, 41, 42, 43, 44, 45, 46, 46, 47, 48, 49, 50, 51, 52, 53, 54,
                    55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70,
                    71, 72, 73, 74, 76, 77, 78, 79, 80, 81, 83, 84, 85, 86, 88, 89,
                    90, 91, 93, 94, 95, 96, 98, 99,100,102,103,104,106,107,109,110,
                    111,113,114,116,117,119,120,121,123,124,126,128,129,131,132,134,
                    135,137,138,140,142,143,145,146,148,150,151,153,155,157,158,160,
                    162,163,165,167,169,170,172,174,176,178,179,181,183,185,187,189,
                    191,193,194,196,198,200,202,204,206,208,210,212,214,216,218,220,
                    222,224,227,229,231,233,235,237,239,241,244,246,248,250,252,255
                  };
  
void _correct_gamma(color_rgb_t * c)
{
  c->red   = gamma_lut[c->red];
  c->green = gamma_lut[c->green];
  c->blue  = gamma_lut[c->blue];
}
void _correct_white(color_rgb_t * c)
{
  c->red = ((uint8_t) ((float)c->red)*0.75);
  //c->green = ((uint8_t) ((float)c->green)*1.0);
  c->blue = ((uint8_t) ((float)c->blue)*0.72);
}

/**
 * @brief   Initialize Led Driver
 * @details Initialize the Led Driver based on parameters.
 *          Following initialization, the frame buffer would automatically be
 *          exported to the supplied port and pins in the right timing to drive
 *          a chain of WS2812B controllers
 * @note    The function assumes the controller is running at 72Mhz
 * @note    Timing is critical for WS2812. While all timing is done in hardware
 *          need to verify memory bandwidth is not exhausted to avoid DMA delays
 *
 * @param[in] leds      length of the LED chain controlled by each pin
 * @param[in] port      which port would be used for output
 * @param[in] mask      Which pins would be used for output, each pin is a full chain
 * @param[out] o_fb     initialized frame buffer
 *
 */
void led_driver_init(int leds, GPIO_TypeDef *port, uint32_t mask, uint8_t **o_fb) {
  sLeds=leds;
  sPort=port;
  sMask=mask;
  palSetGroupMode(port, sMask, 0, PAL_MODE_OUTPUT_PUSHPULL|PAL_STM32_OSPEED_HIGHEST|PAL_STM32_PUDR_FLOATING);

  // configure pwm timers -
  // timer 2 as master, active for data transmission and inactive to disable transmission during reset period (50uS)
  // timer 3 as slave, during active time creates a 1.25 uS signal, with duty cycle controlled by frame buffer values
  static PWMConfig pwmc2 = {72000000 / 90, /* 800Khz PWM clock frequency. 1/90 of PWMC3   */
                            (72000000 / 90) * 0.05, /*Total period is 50ms (20FPS), including sLeds cycles + reset length for ws2812b and FB writes  */
                            NULL,
                            { {PWM_OUTPUT_ACTIVE_HIGH, NULL},
                              {PWM_OUTPUT_DISABLED, NULL},
                              {PWM_OUTPUT_DISABLED, NULL},
                              {PWM_OUTPUT_DISABLED, NULL}},
                              TIM_CR2_MMS_2, /* master mode selection */
                              0, };
  /* master mode selection */
  static PWMConfig pwmc3 = {72000000,/* 72Mhz PWM clock frequency.   */
                            90, /* 90 cycles period (1.25 uS per period @72Mhz       */
                            NULL,
                            { {PWM_OUTPUT_ACTIVE_HIGH, NULL},
                              {PWM_OUTPUT_ACTIVE_HIGH, NULL},
                              {PWM_OUTPUT_ACTIVE_HIGH, NULL},
                              {PWM_OUTPUT_ACTIVE_HIGH, NULL}},
                              0,
                              0,
  };
  dma_source = chHeapAlloc(NULL, 1);
  fb = chHeapAlloc(NULL, ((sLeds) * 24)+10);
  dummy_fb = chHeapAlloc(NULL, ((sLeds) * 24)+10);
  *o_fb=fb;
  int j;
  for (j = 0; j < (sLeds) * 24; j++) {
    fb[j] = 0;
    dummy_fb[j] = 0;
  }
  dma_source[0] = sMask;
  // DMA stream 2, triggered by channel3 pwm signal. if FB indicates, reset output value early to indicate "0" bit to ws2812
  dmaStreamAllocate(STM32_DMA1_STREAM2, 10, NULL, NULL);
  dmaStreamSetPeripheral(STM32_DMA1_STREAM2, &(sPort->BSRR.H.clear));
  dmaStreamSetMemory0(STM32_DMA1_STREAM2, fb);
  dmaStreamSetTransactionSize(STM32_DMA1_STREAM2, (sLeds) * 24);
  dmaStreamSetMode(
      STM32_DMA1_STREAM2,
      STM32_DMA_CR_DIR_M2P | STM32_DMA_CR_MINC | STM32_DMA_CR_PSIZE_BYTE
      | STM32_DMA_CR_MSIZE_BYTE | STM32_DMA_CR_CIRC | STM32_DMA_CR_PL(2));
  // DMA stream 3, triggered by pwm update event. output high at beginning of signal
  dmaStreamAllocate(STM32_DMA1_STREAM3, 10, NULL, NULL);
  dmaStreamSetPeripheral(STM32_DMA1_STREAM3, &(sPort->BSRR.H.set));
  dmaStreamSetMemory0(STM32_DMA1_STREAM3, dma_source);
  dmaStreamSetTransactionSize(STM32_DMA1_STREAM3, 1);
  dmaStreamSetMode(
      STM32_DMA1_STREAM3, STM32_DMA_CR_TEIE |
      STM32_DMA_CR_DIR_M2P | STM32_DMA_CR_PSIZE_BYTE | STM32_DMA_CR_MSIZE_BYTE
      | STM32_DMA_CR_CIRC | STM32_DMA_CR_PL(3));
  // DMA stream 6, triggered by channel1 update event. reset output value late to indicate "1" bit to ws2812.
  // always triggers but no affect if dma stream 2 already change output value to 0
  dmaStreamAllocate(STM32_DMA1_STREAM6, 10, NULL, NULL);
  dmaStreamSetPeripheral(STM32_DMA1_STREAM6, &(sPort->BSRR.H.clear));
  dmaStreamSetMemory0(STM32_DMA1_STREAM6, dma_source);
  dmaStreamSetTransactionSize(STM32_DMA1_STREAM6, 1);
  dmaStreamSetMode(
      STM32_DMA1_STREAM6,
      STM32_DMA_CR_DIR_M2P | STM32_DMA_CR_PSIZE_BYTE | STM32_DMA_CR_MSIZE_BYTE
      | STM32_DMA_CR_CIRC | STM32_DMA_CR_PL(3));
  pwmStart(&PWMD2, &pwmc2);
  pwmStart(&PWMD3, &pwmc3);
  // set pwm3 as slave, triggerd by pwm2 oc1 event. disables pwmd2 for synchronization.
  PWMD3.tim->SMCR |= TIM_SMCR_SMS_0 | TIM_SMCR_SMS_2 | TIM_SMCR_TS_0;
  PWMD2.tim->CR1 &= ~TIM_CR1_CEN;
  // set pwm values.
  // 28 (duty in ticks) / 90 (period in ticks) * 1.25uS (period in S) = 0.39 uS
  pwmEnableChannel(&PWMD3, 2, 28);
  // 58 (duty in ticks) / 90 (period in ticks) * 1.25uS (period in S) = 0.806 uS
  pwmEnableChannel(&PWMD3, 0, 58);
  // active during transfer of 90 cycles * sLeds * 24 bytes * 1/90 multiplier
  pwmEnableChannel(&PWMD2, 0, 90 * sLeds * 24 / 90);
  // stop and reset counters for synchronization
  PWMD2.tim->CNT = 0;
  // Slave (TIM3) needs to "update" immediately after master (TIM2) start in order to start in sync.
  // this initial sync is crucial for the stability of the run
  PWMD3.tim->CNT = 89;
  PWMD3.tim->DIER |= TIM_DIER_CC3DE | TIM_DIER_CC1DE | TIM_DIER_UDE;
  dmaStreamEnable(STM32_DMA1_STREAM3);
  dmaStreamEnable(STM32_DMA1_STREAM6);
  dmaStreamEnable(STM32_DMA1_STREAM2);
  // all systems go! both timers and all channels are configured to resonate
  // in complete sync without any need for CPU cycles (only DMA and timers)
  // start pwm2 for system to start resonating
  PWMD2.tim->CR1 |= TIM_CR1_CEN;
}


void test_pattern_fb(uint8_t *fb){
  int i;
  color_rgb_t tmpC = {rand()%256, rand()%256, rand()%256};
  for (i=0;i<sLeds;i++){
    set_color_rgb(tmpC,fb+24*i, sMask);
  }
}

const uint8_t front_lut[4][3] = {
                                  { 0,  5,  3},
                                  {13, 17, 15},
                                  { 6, 11,  8},
                                  {18,  0,  0}
                                };
const uint8_t  back_lut[4][3] = {
                                  {16, 12, 14},
                                  { 4,  1,  2},
                                  {10,  7,  9},
                                  {19,  0,  0}
                                };

void set_color_rgb_location(color_rgb_t c, uint8_t back_nfront, uint8_t section, uint8_t index)
{
  if (section < 3  && index > 2)  return;
  if (section == 3 && index > 0)  return;
  if (section > 3)                return;

  if (!back_nfront) set_color_rgb(c, dummy_fb + 24*front_lut[section][index], sMask);
  else              set_color_rgb(c, dummy_fb +  24*back_lut[section][index], sMask);
}

void set_color_rgb_array(color_rgb_t c, uint8_t index)
{
  if (index >= sLeds) return;

  set_color_rgb(c, dummy_fb + 24*index, sMask);
}

void push_buffer_leds(void) {
  memcpy(fb, dummy_fb, 24*sLeds + 10);
}

void set_color_hsv_location(color_hsv_t c, uint8_t back_nfront, uint8_t section, uint8_t index)
{
  color_rgb_t tempc = convert_color_hsv_to_rgb(c);

  set_color_rgb_location(tempc, back_nfront, section, index);
}

void set_color_hsl_location(color_hsl_t c, uint8_t back_nfront, uint8_t section, uint8_t index)
{
  color_rgb_t tempc = convert_color_hsl_to_rgb(c);

  set_color_rgb_location(tempc, back_nfront, section, index);
}

color_rgb_t convert_color_hsv_to_rgb(color_hsv_t c_hsv)
{
  float red    = c_hsv.val;
  float green  = c_hsv.val;
  float blue   = c_hsv.val;

  float hue    = c_hsv.hue;
  float sat    = c_hsv.sat;
  float val    = c_hsv.val;

  if (sat > 0.0) {
    hue *= 6.0;      // sector 0 to 5
    uint32_t sextant = floorf(hue);;
    float fract = hue - sextant;      // fractional part of h

    float p = val * ( 1 - sat );
    float q = val * ( 1 - sat * fract );
    float t = val * ( 1 - sat * ( 1 - fract ) );

    switch(sextant) {
      case 0:
        red = val;
        green = t;
        blue = p;
        break;
      case 1:
        red = q;
        green = val;
        blue = p;
        break;
      case 2:
        red = p;
        green = val;
        blue = t;
        break;
      case 3:
        red = p;
        green = q;
        blue = val;
        break;
      case 4:
        red = t;
        green = p;
        blue = val;
        break;
      default:    // case 5:
        red = val;
        green = p;
        blue = q;
        break;
    }
  }

  color_rgb_t c_rgb = {((uint8_t) (red*255.0)), ((uint8_t) (green*255.0)), ((uint8_t) (blue*255.0))};
  return c_rgb;
}

color_rgb_t convert_color_hsl_to_rgb(color_hsl_t c_hsl)
{
  float red    = c_hsl.lum;
  float green  = c_hsl.lum;
  float blue   = c_hsl.lum;

  color_rgb_t c_rgb = {((uint8_t) red*255.0), ((uint8_t) green*255.0), ((uint8_t) blue*255.0)};
  return c_rgb;
}

color_hsv_t average_color_hsv(color_hsv_t * c_arr, size_t n)
{
  color_hsv_t c_avg = {0, 0, 0};

  size_t i;
  for (i = 0; i < n; i++) {
    c_avg.hue += c_arr[i].hue;
    c_avg.sat += c_arr[i].sat;
    c_avg.val += c_arr[i].val;
  }

  c_avg.hue /= n;
  c_avg.sat /= n;
  c_avg.val /= n;

  return c_avg;
}
