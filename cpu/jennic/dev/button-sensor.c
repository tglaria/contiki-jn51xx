/*
 * Copyright (c) 2009-2010
 * Telecooperation Office (TecO), Universitaet Karlsruhe (TH), Germany.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 * 3. Neither the name of the Universitaet Karlsruhe (TH) nor the names
 *    of its contributors may be used to endorse or promote products
 *    derived from this software without specific prior written
 *    permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Author(s): Philipp Scholl <scholl@teco.edu>
 */
#include "lib/sensors.h"
#include "dev/button-sensor.h"
#include "dev/irq.h"
#include "dev/leds.h"
#include <stdbool.h>
#include <AppHardwareApi.h>

#include "stdio.h"

#define DEBUG 0
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

#ifndef JENNIC_CONF_BUTTON_PIN
# define PINS 0x00 // no PINS as input per default to avoid current draw
#else
# define PINS JENNIC_CONF_BUTTON_PIN
#endif

// calculate the mapping from PINS to BUTTON_X macro for the value() function
// at compile-time! __builtin_ffs() gives the index of the first significant
// bit, so BUTTONS are ordered by the PIN number.
#define BUTTON0_MASK __builtin_ffs(PINS)
#define BUTTON1_MASK __builtin_ffs(BUTTON0_MASK)
#define BUTTON2_MASK __builtin_ffs(BUTTON1_MASK)
#define BUTTON3_MASK __builtin_ffs(BUTTON2_MASK)
#define BUTTON4_MASK __builtin_ffs(BUTTON3_MASK)
#define BUTTON5_MASK __builtin_ffs(BUTTON4_MASK)
#define BUTTON6_MASK __builtin_ffs(BUTTON5_MASK)
#define BUTTON7_MASK __builtin_ffs(BUTTON6_MASK)

static uint32_t volatile irq_val=0,
                             val=0;

PROCESS(debounce_process, "button debounce");
#define DEBOUNCE_TIME (CLOCK_SECOND/100)

void irq(irq_t s)
{
  // this "if" is here, so that the compiler removes all the code if there is no
  // input PIN defined as a button!
  if (__builtin_popcount(PINS)) {

  /* configure for next event from the PIN */
  uint32 u32Rising, u32Falling;
  irq_val = u32AHI_DioReadInput();

  /* configure to complementary event */
  u32Rising = (~irq_val)&((uint32)PINS);
  u32Falling = irq_val&((uint32)PINS);
  vAHI_DioInterruptEdge(u32Rising, u32Falling);

  /* let the debounce process know about this irq */
  process_poll(&debounce_process);

  }
}

static int
value(int type)
{
  switch (type & __builtin_popcount(PINS)) {
  case BUTTON_ALL: return val&PINS;
  case BUTTON_0:   return val&BUTTON0_MASK;
  case BUTTON_1:   return val&BUTTON1_MASK;
  case BUTTON_2:   return val&BUTTON2_MASK;
  case BUTTON_3:   return val&BUTTON3_MASK;
  case BUTTON_4:   return val&BUTTON4_MASK;
  case BUTTON_5:   return val&BUTTON5_MASK;
  case BUTTON_6:   return val&BUTTON6_MASK;
  case BUTTON_7:   return val&BUTTON7_MASK;
  default:         return 0;
  }
}

static int
configure(int type, int value)
{
  static struct irq_handle handle = { .callback=irq, .irqsrc=PINS };

  // this "if" is here, so that the compiler removes all the code if there is no
  // input PIN defined as a button!
  if (__builtin_popcount(PINS)) {

  switch(type) {
  case SENSORS_HW_INIT:
    vAHI_DioSetDirection(PINS, 0x00);
    val = (u32AHI_DioReadInput() & PINS);
    PRINTF("Button_Sensor SENSORS_HW_INIT\n\r");
    return 1;

  case SENSORS_ACTIVE:
    if (value) {
      irq_add(&handle);
      irq(PINS); /* call irq routing once to initialise */
      process_start(&debounce_process, NULL);
      vAHI_DioInterruptEnable(PINS, 0x00);
    }
    else {
      vAHI_DioInterruptEnable(0x00, PINS);
      irq_remove(&handle);
      process_exit(&debounce_process);
    }
    return 1;
  }

  }

  return 0;
}

static void *
status(int type)
{
  switch(type) {
  case SENSORS_ACTIVE:
  case SENSORS_READY:
    break;
  }

  return NULL;
}

PROCESS_THREAD(debounce_process, ev, data)
{
  static struct etimer et;

  // this "if" is here, so that the compiler removes all the code if there is no
  // input PIN defined as a button!
  if (__builtin_popcount(PINS)) {

  PROCESS_BEGIN();

  etimer_set(&et, DEBOUNCE_TIME);
  while (ev != PROCESS_EVENT_EXIT)
  {
    PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_POLL || ev == PROCESS_EVENT_TIMER);

    if (ev==PROCESS_EVENT_POLL) etimer_reset(&et);
    else if (ev==PROCESS_EVENT_TIMER && val != irq_val) {
      val = irq_val;
      sensors_changed(&button_sensor);
    }
  }

  PROCESS_END();

  }
  return 0;
}

SENSORS_SENSOR(button_sensor, BUTTON_SENSOR,
               value, configure, status);
