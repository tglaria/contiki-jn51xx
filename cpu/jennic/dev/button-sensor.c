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

#ifndef JENNIC_CONF_BUTTON_PIN
# define PIN IRQ_DIO9
#else
# define PIN JENNIC_CONF_BUTTON_PIN
#endif

static bool volatile current_value = false;
static bool _value = false;

PROCESS(debounce_process, "button debounce");
#define DEBOUNCE_TIME (CLOCK_SECOND/100)

void irq(irq_t s)
{
  /* configure for next event from this PIN */
  if (u32AHI_DioReadInput()&PIN)
    vAHI_DioInterruptEdge(0x00, PIN);
  else
    vAHI_DioInterruptEdge(PIN, 0x00);

  current_value = u32AHI_DioReadInput()&PIN;
  process_poll(&debounce_process);
}

static int
value(int type)
{
  return _value;
}

static int
configure(int type, int value)
{
  static irq_handle_t handle = { .callback=irq, .irqsrc=PIN };

  switch(type) {
  case SENSORS_HW_INIT:
    vAHI_DioSetDirection(0x00, PIN);
    _value = u32AHI_DioReadInput() & PIN;
    return 1;

  case SENSORS_ACTIVE:
    if (value) {
      irq_add(&handle);
      irq(PIN); /* call irq routing once to initialise */
      process_start(&debounce_process, NULL);
      vAHI_DioInterruptEnable(PIN, 0x00);
    }
    else {
      vAHI_DioInterruptEnable(0x00, PIN);
      irq_remove(&handle);
      process_exit(&debounce_process);
    }
    return 1;
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
  PROCESS_BEGIN();

  etimer_set(&et, DEBOUNCE_TIME);
  while (ev != PROCESS_EVENT_EXIT)
  {
    PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_POLL || ev == PROCESS_EVENT_TIMER);

    if (ev==PROCESS_EVENT_POLL) etimer_reset(&et);
    else if (ev==PROCESS_EVENT_TIMER && _value != current_value) {
      _value = current_value;
      sensors_changed(&button_sensor);
    }
  }

  PROCESS_END();
}

SENSORS_SENSOR(button_sensor, BUTTON_SENSOR,
               value, configure, status);
