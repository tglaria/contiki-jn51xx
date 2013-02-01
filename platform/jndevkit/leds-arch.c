/*
 * Copyright (c) 2008
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

#include "dev/leds.h"
#include <AppHardwareApi.h>
#include "contiki-conf.h"

static uint8_t leds;

/*---------------------------------------------------------------------------*/
void
leds_arch_init(void)
{
  leds_arch_set(0);
}

/*---------------------------------------------------------------------------*/
unsigned char
leds_arch_get(void)
{
  return leds;
}
/*---------------------------------------------------------------------------*/
void
leds_arch_set(unsigned char c)
{
  leds = c;

  if ((c==LEDS_ALL) || (c&LEDS_RED && c&LEDS_GREEN))
    vAHI_DioSetDirection(0, E_AHI_DIO16_INT|
                            E_AHI_DIO17_INT);
  else if (c&LEDS_RED)
    vAHI_DioSetDirection(E_AHI_DIO16_INT, E_AHI_DIO17_INT);
  else if (c&LEDS_GREEN)
    vAHI_DioSetDirection(E_AHI_DIO17_INT, E_AHI_DIO16_INT);
  else
    vAHI_DioSetDirection(E_AHI_DIO17_INT|
                         E_AHI_DIO16_INT, 0);
}

