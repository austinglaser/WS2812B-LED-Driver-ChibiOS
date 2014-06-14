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
#include "chsys.h"
#include "test.h"
#include "math.h"
#include "arm_math.h"
#include "shell.h"
#include "chprintf.h"
#include "led_driver.h"
#include "MPU9150.h"

#define N_LEDS 240
/*===========================================================================*/
/* Timer related stuff.                                                      */
/*===========================================================================*/

GPTConfig i2c_timer = {
  1000000,
  NULL,
  0         /* DIER */
};

/*===========================================================================*/
/* I2C related stuff.                                                        */
/*===========================================================================*/

// Hardware-specific support functions that MUST be customized:
#define I2CSPEED 100
void I2C_delay(void) 
{
  gptPolledDelay(&GPTD1, 10); //wait 10 us
}

bool read_SCL(void) // Set SCL as input and return current level of line, 0 or 1
{
  palWritePad(GPIOB, GPIOB_IMU_SCL, 1);

  return palReadPad(GPIOB, GPIOB_IMU_SCL);
}
bool read_SDA(void) // Set SDA as input and return current level of line, 0 or 1
{
  palWritePad(GPIOB, GPIOB_IMU_SDA, 1);

  return palReadPad(GPIOB, GPIOB_IMU_SDA);
}
void clear_SCL(void) // Actively drive SCL signal low
{
  palWritePad(GPIOB, GPIOB_IMU_SCL, 0);
}
void clear_SDA(void) // Actively drive SDA signal low
{
  palWritePad(GPIOB, GPIOB_IMU_SDA, 0);
}

bool started = false; // global data
int i2c_start_cond(void) {
  if (started) { // if started, do a restart cond
    // set SDA to 1
    read_SDA();
    I2C_delay();
    while (read_SCL() == 0) {  // Clock stretching
      // You should add timeout to this loop
    }
    // Repeated start setup time, minimum 4.7us
    I2C_delay();
  }
  if (read_SDA() == 0) {
    return 1;
  }
  // SCL is high, set SDA from 1 to 0.
  clear_SDA();
  I2C_delay();
  clear_SCL();
  started = true;

  return 0;
}

int i2c_stop_cond(void){
  // set SDA to 0
  clear_SDA();
  I2C_delay();
  // Clock stretching
  while (read_SCL() == 0) {
    // add timeout to this loop.
  }
  // Stop bit setup time, minimum 4us
  I2C_delay();
  // SCL is high, set SDA from 0 to 1
  if (read_SDA() == 0) {
    return -1;
  }
  I2C_delay();
  started = false;

  return 0;
}

// Write a bit to I2C bus
int i2c_write_bit(bool bit) {
  if (bit) {
    read_SDA();
  } else {
    clear_SDA();
  }
  I2C_delay();
  while (read_SCL() == 0) { // Clock stretching
    // You should add timeout to this loop
  }
  // SCL is high, now data is valid
  // If SDA is high, check that nobody else is driving SDA
  if (bit && read_SDA() == 0) {
    return 1;
  }
  I2C_delay();
  clear_SCL();

  return 0;
}

// Read a bit from I2C bus
bool i2c_read_bit(void) {
  bool bit;
  // Let the slave drive data
  read_SDA();
  I2C_delay();
  while (read_SCL() == 0) { // Clock stretching
    // You should add timeout to this loop
  }
  // SCL is high, now data is valid
  bit = read_SDA();
  I2C_delay();
  clear_SCL();
  return bit;
}

// Write a byte to I2C bus. Return 0 if ack by the slave.
bool i2c_write_byte(bool send_start,
    bool send_stop,
    unsigned char byte) {
  unsigned bit;
  bool nack;
  if (send_start) {
    i2c_start_cond();
  }
  for (bit = 0; bit < 8; bit++) {
    i2c_write_bit((byte & 0x80) != 0);
    byte <<= 1;
  }
  nack = i2c_read_bit();
  if (send_stop) {
    i2c_stop_cond();
  }
  return nack;
}

// Read a byte from I2C bus
unsigned char i2c_read_byte(bool nack, bool send_stop) {
  unsigned char byte = 0;
  unsigned bit;
  for (bit = 0; bit < 8; bit++) {
    byte = (byte << 1) | i2c_read_bit();
  }
  i2c_write_bit(nack);
  if (send_stop) {
    i2c_stop_cond();
  }
  return byte;
}

// writes d to register ra. returns 1 if successful, 0 otherwise
bool mpu9150_write_register(uint8_t ra, uint8_t data[], size_t n)
{
  if (i2c_write_byte(1, 0, (0x68 << 1))) return 1; // send address byte. send start, no stop
  if (i2c_write_byte(0, 0, ra))          return 1; // send register address

  size_t i;
  for (i = 0; i < n; i++) if (i2c_write_byte(0, i == (n - 1), data[i])) return 1; // send data

  return 0; // success
}

bool mpu9150_read_register(uint8_t ra, uint8_t data[], size_t n)
{
  if (i2c_write_byte(1, 0, (0x68 << 1)))        return 1; // send address byte. send start, no stop
  if (i2c_write_byte(0, 0, ra))                 return 1; // send register address
  if (i2c_write_byte(1, 0, (0x68 << 1) | 0x01)) return 1; // send address byte. send start, no stop

  size_t i;
  for (i = 0; i < n; i++) data[i] = i2c_read_byte(i == (n - 1), i == (n - 1));

  return 0; // success
}

/*===========================================================================*/
/* USB related stuff.                                                        */
/*===========================================================================*/

/*
 * Endpoints to be used for USBD1.
 */
#define USBD1_DATA_REQUEST_EP           1
#define USBD1_DATA_AVAILABLE_EP         1
#define USBD1_INTERRUPT_REQUEST_EP      2

/*
 * DP resistor control is not possible on the STM32F3-Discovery, using stubs
 * for the connection macros.
 */
#define usb_lld_connect_bus(usbp)
#define usb_lld_disconnect_bus(usbp)

/*
 * Serial over USB Driver structure.
 */
static SerialUSBDriver SDU1;

/*
 * USB Device Descriptor.
 */
static const uint8_t vcom_device_descriptor_data[18] = {
  USB_DESC_DEVICE       (0x0110,        /* bcdUSB (1.1).                    */
      0x02,          /* bDeviceClass (CDC).              */
      0x00,          /* bDeviceSubClass.                 */
      0x00,          /* bDeviceProtocol.                 */
      0x40,          /* bMaxPacketSize.                  */
      0x0483,        /* idVendor (ST).                   */
      0x5740,        /* idProduct.                       */
      0x0200,        /* bcdDevice.                       */
      1,             /* iManufacturer.                   */
      2,             /* iProduct.                        */
      3,             /* iSerialNumber.                   */
      1)             /* bNumConfigurations.              */
};

/*
 * Device Descriptor wrapper.
 */
static const USBDescriptor vcom_device_descriptor = {
  sizeof vcom_device_descriptor_data,
  vcom_device_descriptor_data
};

/* Configuration Descriptor tree for a CDC.*/
static const uint8_t vcom_configuration_descriptor_data[67] = {
  /* Configuration Descriptor.*/
  USB_DESC_CONFIGURATION(67,            /* wTotalLength.                    */
      0x02,          /* bNumInterfaces.                  */
      0x01,          /* bConfigurationValue.             */
      0,             /* iConfiguration.                  */
      0xC0,          /* bmAttributes (self powered).     */
      50),           /* bMaxPower (100mA).               */
  /* Interface Descriptor.*/
  USB_DESC_INTERFACE    (0x00,          /* bInterfaceNumber.                */
      0x00,          /* bAlternateSetting.               */
      0x01,          /* bNumEndpoints.                   */
      0x02,          /* bInterfaceClass (Communications
                        Interface Class, CDC section
                        4.2).                            */
      0x02,          /* bInterfaceSubClass (Abstract
                        Control Model, CDC section 4.3).   */
      0x01,          /* bInterfaceProtocol (AT commands,
                        CDC section 4.4).                */
      0),            /* iInterface.                      */
  /* Header Functional Descriptor (CDC section 5.2.3).*/
  USB_DESC_BYTE         (5),            /* bLength.                         */
  USB_DESC_BYTE         (0x24),         /* bDescriptorType (CS_INTERFACE).  */
  USB_DESC_BYTE         (0x00),         /* bDescriptorSubtype (Header
                                           Functional Descriptor.           */
  USB_DESC_BCD          (0x0110),       /* bcdCDC.                          */
  /* Call Management Functional Descriptor. */
  USB_DESC_BYTE         (5),            /* bFunctionLength.                 */
  USB_DESC_BYTE         (0x24),         /* bDescriptorType (CS_INTERFACE).  */
  USB_DESC_BYTE         (0x01),         /* bDescriptorSubtype (Call Management
                                           Functional Descriptor).          */
  USB_DESC_BYTE         (0x00),         /* bmCapabilities (D0+D1).          */
  USB_DESC_BYTE         (0x01),         /* bDataInterface.                  */
  /* ACM Functional Descriptor.*/
  USB_DESC_BYTE         (4),            /* bFunctionLength.                 */
  USB_DESC_BYTE         (0x24),         /* bDescriptorType (CS_INTERFACE).  */
  USB_DESC_BYTE         (0x02),         /* bDescriptorSubtype (Abstract
                                           Control Management Descriptor).  */
  USB_DESC_BYTE         (0x02),         /* bmCapabilities.                  */
  /* Union Functional Descriptor.*/
  USB_DESC_BYTE         (5),            /* bFunctionLength.                 */
  USB_DESC_BYTE         (0x24),         /* bDescriptorType (CS_INTERFACE).  */
  USB_DESC_BYTE         (0x06),         /* bDescriptorSubtype (Union
                                           Functional Descriptor).          */
  USB_DESC_BYTE         (0x00),         /* bMasterInterface (Communication
                                           Class Interface).                */
  USB_DESC_BYTE         (0x01),         /* bSlaveInterface0 (Data Class
                                           Interface).                      */
  /* Endpoint 2 Descriptor.*/
  USB_DESC_ENDPOINT     (USBD1_INTERRUPT_REQUEST_EP|0x80,
      0x03,          /* bmAttributes (Interrupt).        */
      0x0008,        /* wMaxPacketSize.                  */
      0xFF),         /* bInterval.                       */
  /* Interface Descriptor.*/
  USB_DESC_INTERFACE    (0x01,          /* bInterfaceNumber.                */
      0x00,          /* bAlternateSetting.               */
      0x02,          /* bNumEndpoints.                   */
      0x0A,          /* bInterfaceClass (Data Class
                        Interface, CDC section 4.5).     */
      0x00,          /* bInterfaceSubClass (CDC section
                        4.6).                            */
      0x00,          /* bInterfaceProtocol (CDC section
                        4.7).                            */
      0x00),         /* iInterface.                      */
  /* Endpoint 3 Descriptor.*/
  USB_DESC_ENDPOINT     (USBD1_DATA_AVAILABLE_EP,       /* bEndpointAddress.*/
      0x02,          /* bmAttributes (Bulk).             */
      0x0040,        /* wMaxPacketSize.                  */
      0x00),         /* bInterval.                       */
  /* Endpoint 1 Descriptor.*/
  USB_DESC_ENDPOINT     (USBD1_DATA_REQUEST_EP|0x80,    /* bEndpointAddress.*/
      0x02,          /* bmAttributes (Bulk).             */
      0x0040,        /* wMaxPacketSize.                  */
      0x00)          /* bInterval.                       */
};

/*
 * Configuration Descriptor wrapper.
 */
static const USBDescriptor vcom_configuration_descriptor = {
  sizeof vcom_configuration_descriptor_data,
  vcom_configuration_descriptor_data
};

/*
 * U.S. English language identifier.
 */
static const uint8_t vcom_string0[] = {
  USB_DESC_BYTE(4),                     /* bLength.                         */
  USB_DESC_BYTE(USB_DESCRIPTOR_STRING), /* bDescriptorType.                 */
  USB_DESC_WORD(0x0409)                 /* wLANGID (U.S. English).          */
};

/*
 * Vendor string.
 */
static const uint8_t vcom_string1[] = {
  USB_DESC_BYTE(38),                    /* bLength.                         */
  USB_DESC_BYTE(USB_DESCRIPTOR_STRING), /* bDescriptorType.                 */
  'S', 0, 'T', 0, 'M', 0, 'i', 0, 'c', 0, 'r', 0, 'o', 0, 'e', 0,
  'l', 0, 'e', 0, 'c', 0, 't', 0, 'r', 0, 'o', 0, 'n', 0, 'i', 0,
  'c', 0, 's', 0
};

/*
 * Device Description string.
 */
static const uint8_t vcom_string2[] = {
  USB_DESC_BYTE(56),                    /* bLength.                         */
  USB_DESC_BYTE(USB_DESCRIPTOR_STRING), /* bDescriptorType.                 */
  'C', 0, 'h', 0, 'i', 0, 'b', 0, 'i', 0, 'O', 0, 'S', 0, '/', 0,
  'R', 0, 'T', 0, ' ', 0, 'V', 0, 'i', 0, 'r', 0, 't', 0, 'u', 0,
  'a', 0, 'l', 0, ' ', 0, 'C', 0, 'O', 0, 'M', 0, ' ', 0, 'P', 0,
  'o', 0, 'r', 0, 't', 0
};

/*
 * Serial Number string.
 */
static const uint8_t vcom_string3[] = {
  USB_DESC_BYTE(8),                     /* bLength.                         */
  USB_DESC_BYTE(USB_DESCRIPTOR_STRING), /* bDescriptorType.                 */
  '0' + CH_KERNEL_MAJOR, 0,
  '0' + CH_KERNEL_MINOR, 0,
  '0' + CH_KERNEL_PATCH, 0
};

/*
 * Strings wrappers array.
 */
static const USBDescriptor vcom_strings[] = {
  {sizeof vcom_string0, vcom_string0},
  {sizeof vcom_string1, vcom_string1},
  {sizeof vcom_string2, vcom_string2},
  {sizeof vcom_string3, vcom_string3}
};

/*
 * Handles the GET_DESCRIPTOR callback. All required descriptors must be
 * handled here.
 */
static const USBDescriptor *get_descriptor(USBDriver *usbp,
    uint8_t dtype,
    uint8_t dindex,
    uint16_t lang) {

  (void)usbp;
  (void)lang;
  switch (dtype) {
    case USB_DESCRIPTOR_DEVICE:
      return &vcom_device_descriptor;
    case USB_DESCRIPTOR_CONFIGURATION:
      return &vcom_configuration_descriptor;
    case USB_DESCRIPTOR_STRING:
      if (dindex < 4)
        return &vcom_strings[dindex];
  }
  return NULL;
}

/**
 * @brief   IN EP1 state.
 */
static USBInEndpointState ep1instate;

/**
 * @brief   OUT EP1 state.
 */
static USBOutEndpointState ep1outstate;

/**
 * @brief   EP1 initialization structure (both IN and OUT).
 */
static const USBEndpointConfig ep1config = {
  USB_EP_MODE_TYPE_BULK,
  NULL,
  sduDataTransmitted,
  sduDataReceived,
  0x0040,
  0x0040,
  &ep1instate,
  &ep1outstate,
  1,
  NULL
};

/**
 * @brief   IN EP2 state.
 */
static USBInEndpointState ep2instate;

/**
 * @brief   EP2 initialization structure (IN only).
 */
static const USBEndpointConfig ep2config = {
  USB_EP_MODE_TYPE_INTR,
  NULL,
  sduInterruptTransmitted,
  NULL,
  0x0010,
  0x0000,
  &ep2instate,
  NULL,
  1,
  NULL
};

/*
 * Handles the USB driver global events.
 */
static void usb_event(USBDriver *usbp, usbevent_t event) {

  switch (event) {
    case USB_EVENT_RESET:
      return;
    case USB_EVENT_ADDRESS:
      return;
    case USB_EVENT_CONFIGURED:
      chSysLockFromIsr();

      /* Enables the endpoints specified into the configuration.
         Note, this callback is invoked from an ISR so I-Class functions
         must be used.*/
      usbInitEndpointI(usbp, USBD1_DATA_REQUEST_EP, &ep1config);
      usbInitEndpointI(usbp, USBD1_INTERRUPT_REQUEST_EP, &ep2config);

      /* Resetting the state of the CDC subsystem.*/
      sduConfigureHookI(&SDU1);

      chSysUnlockFromIsr();
      return;
    case USB_EVENT_SUSPEND:
      return;
    case USB_EVENT_WAKEUP:
      return;
    case USB_EVENT_STALLED:
      return;
  }
  return;
}

/*
 * USB driver configuration.
 */
static const USBConfig usbcfg = {
  usb_event,
  get_descriptor,
  sduRequestsHook,
  NULL
};

/*
 * Serial over USB driver configuration.
 */
static const SerialUSBConfig serusbcfg = {
  &USBD1,
  USBD1_DATA_REQUEST_EP,
  USBD1_DATA_AVAILABLE_EP,
  USBD1_INTERRUPT_REQUEST_EP
};

BaseSequentialStream *USBout = (BaseSequentialStream *)&SDU1;
BaseChannel *USBin = (BaseChannel *)&SDU1;

//const I2CConfig mpu9150_config = { 
//  STM32_TIMINGR_PRESC(15U) |
//  STM32_TIMINGR_SCLDEL(4U) | STM32_TIMINGR_SDADEL(2U) |
//  STM32_TIMINGR_SCLH(15U)  | STM32_TIMINGR_SCLL(21U),
//  0,
//  0
//};

const I2CConfig mpu9150_config = { 
  STM32_TIMINGR_PRESC(14U) |
    STM32_TIMINGR_SCLDEL(5U) | STM32_TIMINGR_SDADEL(3U) |
    STM32_TIMINGR_SCLH(18U)  | STM32_TIMINGR_SCLL(23U),
  0,
  0
};


static uint8_t *o_fb;
color_rgb_t led[N_LEDS];
static uint16_t gpio_pin1_mask = 0b0000000000000010;

static const color_rgb_t red = {100, 0, 0};
static const color_rgb_t green = {0, 100, 0};
static const color_rgb_t blue = {0, 0, 100};
static const color_rgb_t black = {0, 0, 0};

/*
   static WORKING_AREA(wathread_leds, 512);
   __attribute__ ((__noreturn__))
   static msg_t thread_leds(void *arg)
   {

   (void) arg;
   while (TRUE) {
//chprintf(USBout, "red\r\n");
set_color_rgb(red, o_fb, gpio_pin1_mask);
chThdSleepMilliseconds(1000);

//chprintf(USBout, "green\r\n");
set_color_rgb(green, o_fb, gpio_pin1_mask);
chThdSleepMilliseconds(1000);

//chprintf(USBout, "blue\r\n");
set_color_rgb(blue, o_fb, gpio_pin1_mask);
chThdSleepMilliseconds(1000);
}
}
 */

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
   * Initializes a serial-over-USB CDC driver.
   */
  sduObjectInit(&SDU1);
  sduStart(&SDU1, &serusbcfg);

  /*
   * Activates the USB driver and then the USB bus pull-up on D+.
   * Note, a delay is inserted in order to not have to disconnect the cable
   * after a reset.
   */
  usbDisconnectBus(serusbcfg.usbp);
  chThdSleepMilliseconds(1500);
  usbStart(serusbcfg.usbp, &usbcfg);
  usbConnectBus(serusbcfg.usbp);

  /*
   * Initialize timer for i2c
   */
  gptStart(&GPTD1, &i2c_timer);

  /*  
   * Starting the I2C driver 1.
   */
  i2cStart(&I2CD1, &mpu9150_config);


  /*
   * Initialize LedDriver - 20 leds in chain, GPIOA pin 1
   */
  led_driver_init(N_LEDS, GPIOA, gpio_pin1_mask, &o_fb);

  uint8_t data[6];
  color_rgb_t c;
  bool success;

  /*
   * Wake up mpu9150
   */
  data[0] = 0x02;

  success = mpu9150_write_register(0x6B, data, 1);

  int i;
  for (i = 0; i < N_LEDS; i++) {
    led[i] = black;
    set_color_rgb(black, o_fb + 24*i, gpio_pin1_mask);
  }


  while (TRUE) {

    success = mpu9150_read_register(0x3B, data, 6);
    if (!success) {
      //int32_t r = (int32_t) data[0] << 8 | data[1];
      //int32_t g = (int32_t) data[2] << 8 | data[3];
      //int32_t b = (int32_t) data[4] << 8 | data[5];

      int8_t r = (int8_t) data[0];
      int8_t g = (int8_t) data[2];
      int8_t b = (int8_t) data[4];

      //if (r < 0) r *= -1;
      //if (g < 0) g *= -1;
      //if (b < 0) b *= -1;

      if (r < 0) r = 0;
      if (g < 0) g *= -1;
      else       g = 0;
      if (b < 0) b = 0;

      //c.red = r*255/32768;
      //c.green = g*255/32768;
      //c.blue = b*255/32768;

      c.red = r/4;
      c.green = g/4;
      c.blue = b/4;

      for (i = N_LEDS - 1; i > 0; i--) led[i] = led[i - 1];
      led[0] = c;
    }

    for (i = 0; i < N_LEDS; i++) set_color_rgb_array(led[i], i);
    push_buffer_leds();

    //success = mpu9150_read_register(0x3B, data, 6);
    //if (success) chprintf(USBout, "FAILED\t\t");
    //else         chprintf(USBout, "SUCCESS\t\t");
    //chprintf(USBout, "x: %d\t", (int32_t) data[0] << 8 | data[1]);
    //chprintf(USBout, "y: %d\t", (int32_t) data[2] << 8 | data[3]);
    //chprintf(USBout, "z: %d\t", (int32_t) data[4] << 8 | data[5]);
    //chprintf(USBout, "\r\n");

    //chThdSleepMilliseconds(100);
  }
}
