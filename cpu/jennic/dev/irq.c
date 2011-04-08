/*
 * Copyright (c) 2009
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
 *
 * Wrapper for interrupt handling.
 *
 */
#include <stdbool.h>
#include "contiki.h"
#include "lib/list.h"
#include "sensors.h"
#include "irq.h"
#include "dev/leds.h"
#include <AppHardwareApi.h>
#include "gdb2.h"

LIST(irqs);

#define AP_SAMPLE_READY 0x03

static volatile bool  adc_enabled = false;
static volatile irq_t current_channel = 0;
static void adc_prepare_next();
static void adc_enable();

static void
common_handle(u32_t dev, u32_t irqsrc)
{
  struct irq_handle *item = NULL;

  for (item = list_head(irqs); item; item = item->next)
    if (item->irqsrc & irqsrc || item->irqsrc & current_channel)
      item->callback(item->irqsrc);

  if (dev == E_AHI_DEVICE_ANALOGUE)
  {
    adc_prepare_next();
    u16AHI_AdcRead(); /* make sure the result has been consumed */
  }
}

void
irq_init()
{
  list_init(irqs);
  vAHI_APRegisterCallback(common_handle);
  vAHI_SysCtrlRegisterCallback(common_handle);
}

void
irq_add(const struct irq_handle *handle)
{
  GDB2_PUTS("irq: add\n");
  list_add(irqs, handle);

  /* is this is not a adc irq, then adc will be disabled from the irq again */
  if (!adc_enabled) { adc_enable(); adc_prepare_next(); }
}

void
irq_remove(const struct irq_handle *handle)
{
  GDB2_PUTS("irq: remove\n");
  list_remove(irqs, handle);
}

static void
adc_enable()
{
  vAHI_ApConfigure(E_AHI_AP_REGULATOR_ENABLE, E_AHI_AP_INT_ENABLE,
                   E_AHI_AP_SAMPLE_8, E_AHI_AP_CLOCKDIV_500KHZ,
                   E_AHI_AP_INTREF);

  while(!bAHI_APRegulatorEnabled())
    ; /* wait until adc powered up */

  adc_enabled = true;
}

static uint32_t
to_adcsrc(uint32_t x)
{
  uint32_t i=0;

  if (x >= IRQ_ADC1 && x <= IRQ_ADC_VOLT)
  {
    for (i=0; i<6; i++)
      if (x & (IRQ_ADC1<<i))
        return i;
  }

  return 0;
}

static void
adc_prepare_next() {
  static struct irq_handle *item = NULL;
  u8_t i;

  for(i=0; i < list_length(irqs); i++)
  {
    if (item == NULL)
      item = list_head(irqs);

    if (item->irqsrc >= IRQ_ADC1 && item->irqsrc <= IRQ_ADC_VOLT)
    {
      current_channel = item->irqsrc;

      vAHI_AdcEnable(E_AHI_ADC_SINGLE_SHOT,
                     item->adc_input_range,
                     to_adcsrc(item->irqsrc));
      vAHI_AdcStartSample();

      item = item->next;
      break;
    }

    item = item->next;
  }

  /* no more adc irq handles */
  if (i == list_length(irqs))
  {
    adc_enabled = false;
    GDB2_PUTS("adc: disabled\n");
  }
}
