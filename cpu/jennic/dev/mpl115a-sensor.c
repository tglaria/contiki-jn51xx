/*
 * Copyright (c) 2011
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
 * Driver for MPL115a2.
 */
#include "lib/sensors.h"
#include "dev/pressure-sensor.h"
#include "dev/temperature-sensor.h"
#include "dev/i2c.h"
#include <stdbool.h>
#include <AppHardwareApi.h>

#define MPL_ADDR        0xC0
#define MPL_PRESSURE    0x00
#define MPL_TEMPERATURE 0x02
#define MPL_COEFFICIENT 0x04
#define MPL_START_PRESS 0x10
#define MPL_START_TEMP  0x11
#define MPL_START_BOTH  0x12

static uint8_t rregn(uint8_t r, uint8_t *p, size_t n)
{
  return i2c(MPL_ADDR,&r,1,p,n,I2C_REPEATED_START|I2C_END_OF_TRANS);
}

static uint16_t rreg16(uint8_t r)
{
  uint16_t c=0;
  i2c(MPL_ADDR,&r,1,&c,2,I2C_REPEATED_START|I2C_END_OF_TRANS);
  return c;
}

static bool wreg(uint8_t r, uint8_t c)
{
  uint8_t buf[2] = {r,c};
  return i2c(MPL_ADDR,&buf,2,NULL,0,I2C_REPEATED_START|I2C_END_OF_TRANS);
}

static int*
tstatus(int type)
{
  switch(type) {
  case SENSORS_ACTIVE:
  case SENSORS_READY:
    return 1; // TODO
  }

  return NULL;
}

static int
tvalue(int type)
{
  switch(type) {
  case TEMPERATURE_VALUE_MILLICELSIUS:
    wreg(MPL_START_TEMP, 0x01);
    clock_delay(CLOCK_SECOND/333);
    /* original formula with floats:
     *  25 + (rreg16(MPL_TEMPERATURE) - 498)/-5.35 */
    return 25000 + ((((rreg16(MPL_TEMPERATURE)>>6) - 498)*1000000 / -5350));
  }

  return 0;
}

static int
tconfigure(int type, int value)
{
  switch(type) {
  case SENSORS_HW_INIT:
  case SENSORS_ACTIVE:
    i2c_init();
    return wreg(0x00,0x00); // check if there
  }

  return 0;
}

static int*
pstatus(int type)
{
  switch(type) {
  case SENSORS_ACTIVE:
  case SENSORS_READY:
    return 1; // TODO
  }

  return NULL;
}

static struct {
  int16_t sia0,sib1,sib2,sic12,sic11,sic22;
} __attribute((__packed__)) coeff;

static struct {
  int16_t uiPadc,uiTadc;
} __attribute((__packed__)) p;

static int
pvalue(int type)
{
  int32_t siPcomp,
          lt1, lt2, lt3, si_c11x1, si_a11, si_c12x2,
          si_a1, si_c22x2, si_a2, si_a1x1, si_y1, si_a2x2;

  /* conversion code is based on
   * https://github.com/misenso/MPL115A2-Arduino-Library/blob/master/MPL115A2.cpp
   */
  switch(type) {
  case PRESSURE_VALUE_PASCAL:
    wreg(MPL_START_BOTH, 0x01);
    clock_delay(CLOCK_SECOND/333);
    rregn(MPL_PRESSURE,&p,sizeof(p));

    /* coeefficient 9 equation compensation */
    p.uiPadc = p.uiPadc >> 6;
    p.uiTadc = p.uiTadc >> 6;

    // Step 1 c11x1 = c11 * Padc
    lt1 = (signed long) coeff.sic11;
    lt2 = (signed long) p.uiPadc;
    lt3 = lt1*lt2;
    si_c11x1 = (signed long) lt3;

    // Step 2 a11 = b1 + c11x1
    lt1 = ((signed long)coeff.sib1)<<14;
    lt2 = (signed long) si_c11x1;
    lt3 = lt1 + lt2;
    si_a11 = (signed long)(lt3>>14);

    // Step 3 c12x2 = c12 * Tadc
    lt1 = (signed long) coeff.sic12;
    lt2 = (signed long) p.uiTadc;
    lt3 = lt1*lt2;
    si_c12x2 = (signed long)lt3;

    // Step 4 a1 = a11 + c12x2
    lt1 = ((signed long)si_a11<<11);
    lt2 = (signed long)si_c12x2;
    lt3 = lt1 + lt2;
    si_a1 = (signed long) lt3>>11;

    // Step 5 c22x2 = c22*Tadc
    lt1 = (signed long)coeff.sic22;
    lt2 = (signed long)p.uiTadc;
    lt3 = lt1 * lt2;
    si_c22x2 = (signed long)(lt3);

    // Step 6 a2 = b2 + c22x2
    lt1 = ((signed long)coeff.sib2<<15);
    lt2 = ((signed long)si_c22x2>1);
    lt3 = lt1+lt2;
    si_a2 = ((signed long)lt3>>16);

    // Step 7 a1x1 = a1 * Padc
    lt1 = (signed long)si_a1;
    lt2 = (signed long)p.uiPadc;
    lt3 = lt1*lt2;
    si_a1x1 = (signed long)(lt3);

    // Step 8 y1 = a0 + a1x1
    lt1 = ((signed long)coeff.sia0<<10);
    lt2 = (signed long)si_a1x1;
    lt3 = lt1+lt2;
    si_y1 = ((signed long)lt3>>10);

    // Step 9 a2x2 = a2 * Tadc
    lt1 = (signed long)si_a2;
    lt2 = (signed long)p.uiTadc;
    lt3 = lt1*lt2;
    si_a2x2 = (signed long)(lt3);

    // Step 10 pComp = y1 + a2x2
    lt1 = ((signed long)si_y1<<10);
    lt2 = (signed long)si_a2x2;
    lt3 = lt1+lt2;

    // Fixed point result with rounding
    siPcomp = ((signed long)lt3>>13);

    // decPcomp is defined as a floating point number
    // Conversion to decimal value from 1023 ADC count value
    // ADC counts are 0 to 1023, pressure is 50 to 115kPa respectively
    return (((65000/1023)*siPcomp)+50000);
  }

  return 0;
}

static int
pconfigure(int type, int value)
{
  switch(type) {
  case SENSORS_HW_INIT:
  case SENSORS_ACTIVE:
    i2c_init();
    /* load compensation coeefficients */
    return rregn(MPL_COEFFICIENT,&coeff,sizeof(coeff));
  }

  return 0;
}

SENSORS_SENSOR(temperature_sensor, TEMPERATURE_SENSOR,
               tvalue,tconfigure,tstatus);
SENSORS_SENSOR(pressure_sensor, PRESSURE_SENSOR,
               pvalue,pconfigure,pstatus);
