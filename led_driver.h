/*
 * LEDDriver.h
 *
 *  Created on: Aug 26, 2013
 *      Author: Omri Iluz
 */

#ifndef LEDDRIVER_H_
#define LEDDRIVER_H_

#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "hal.h"

#define sign(x) (( x > 0 ) - ( x < 0 ))

typedef struct color_rgb_t color_rgb_t;
struct color_rgb_t {
  uint8_t red;
  uint8_t green;
  uint8_t blue;
};

typedef struct color_hsl_t color_hsl_t;
struct color_hsl_t {
  float hue;
  float sat;
  float lum;
};

typedef struct color_hsv_t color_hsv_t;
struct color_hsv_t {
  float hue;
  float sat;
  float val;
};

void led_driver_init(int leds, GPIO_TypeDef *port, uint32_t mask, uint8_t **o_fb);
void set_color_rgb(color_rgb_t c, uint8_t *buf, uint32_t mask);
void set_color_hsl(color_hsl_t c, uint8_t *buf, uint32_t mask);
void set_color_hsv(color_hsv_t c, uint8_t *buf, uint32_t mask);
void test_pattern_fb(uint8_t *fb);

void set_color_rgb_array(color_rgb_t c, uint8_t index);

void set_color_rgb_location(color_rgb_t c, uint8_t back_nfront, uint8_t section, uint8_t index);
void set_color_hsl_location(color_hsl_t c, uint8_t back_nfront, uint8_t section, uint8_t index);
void set_color_hsv_location(color_hsv_t c, uint8_t back_nfront, uint8_t section, uint8_t index);

void push_buffer_leds(void);

color_rgb_t convert_color_hsl_to_rgb(color_hsl_t c);
color_rgb_t convert_color_hsv_to_rgb(color_hsv_t c);

color_hsv_t average_color_hsv(color_hsv_t * c_arr, size_t n);

#endif /* LEDDRIVER_H_ */
