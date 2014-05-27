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

#define N_LEDS          20
#define FFT_LENGTH      1024
#define TWO_PI          (M_PI * 2.0f)
#define SAMPLING_RATE   46391.75

#define ADC_GRP1_NUM_CHANNELS   1
#define ADC_GRP1_BUF_DEPTH      (FFT_LENGTH*2)

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

static adcsample_t adc_samples[ADC_GRP1_NUM_CHANNELS * ADC_GRP1_BUF_DEPTH];
static adcsample_t buf_samples[ADC_GRP1_NUM_CHANNELS * ADC_GRP1_BUF_DEPTH];
float32_t avg;

size_t nx = 0, ny = 0;
static void adccallback(ADCDriver * adcp, adcsample_t * buffer, size_t n)
{
  (void)adcp;

  if (adc_samples == buffer) memcpy(buf_samples, adc_samples, sizeof(adcsample_t)*ADC_GRP1_NUM_CHANNELS * ADC_GRP1_BUF_DEPTH);
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
      samples[i*2]     = ((((float32_t) buf_samples[i]) - 2700.0)); //real
      samples[i*2 + 1] = 0.0;                                       //complex
    }

    // perform fft
    arm_cfft_radix2_f32(&S, samples);
    palClearPad(GPIOA, GPIOA_TX1);                      //indicate end of fft stage
    arm_cmplx_mag_f32(samples, fft_mag, FFT_LENGTH/2);  //calculate overall magnitude

    /*
    chprintf(USBout, "sampledata = [");
    for (i = 0; i < FFT_LENGTH/2; i++) {
      chprintf(USBout, "%D,", (int64_t) fft_mag[i]);
    }
    chprintf(USBout,"];\r\n");
    */

    // find max value and location between ~226 and ~2000 hz
    arm_max_f32(fft_mag + 20, 85, &max_value, &max_index);

    // figure out float32_ting magnitude
    float32_t magnitude = (max_value/1024.0);

    color_hsv_t max_color = {(((float32_t) max_index - 20.0)/65.0), 1.0, magnitude};

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
      arm_max_f32(fft_mag + 20, 85, &max_value, &max_index);

      // figure out float32_ting magnitude
      magnitude = (max_value/1024.0);

      sub_color[i].hue = (((float32_t) max_index - 20.0)/65.0);
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
   * Initialize adc thread
   */
  chThdCreateStatic(wathread_leds, sizeof(wathread_leds), NORMALPRIO, thread_leds, NULL);

  uint8_t *o_fb;
  uint16_t gpio_pin1_mask = 0b0000000000000010;

  /* 
   * Set gain for mic pre-amp (high = 40 dB, low = 50 dB, float32_ting = 60 dB)
   */
  //palSetPadMode(GPIOB, GPIOB_GAIN, PAL_MODE_INPUT);           // float32_ting input = 60 dB
  palSetPadMode(GPIOB, GPIOB_GAIN, PAL_MODE_OUTPUT_PUSHPULL);   // drive high or low
  palSetPad(GPIOB, GPIOB_GAIN);                                 // drive high = 40 dB
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
    chThdSleepMilliseconds(1000);
  }

}
