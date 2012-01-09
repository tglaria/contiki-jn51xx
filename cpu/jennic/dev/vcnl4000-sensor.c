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
 * Driver for Vishay VCNL4000.
 */
#include "lib/sensors.h"
#include "dev/lightlevel-sensor.h"
#include "dev/proximity-sensor.h"
#include "dev/i2c.h"
#include <stdbool.h>
#include <AppHardwareApi.h>

//#ifndef JENNIC_CONF_VCNL2550_CONVERT
//# define CONVERT_LUX 1
//#else
//# define CONVERT_LUX JENNIC_CONF_VCNL2550_CONVERT
//#endif

#define VCNL_ADDR        0x26
#define VCNL_CMD_REG     0x80
#define VCNL_PRD_ID      0x81
#define VCNL_IR_CURRENT  0x83
#define VCNL_AMB_PAR     0x84
#define VCNL_AMB_RES     0x85
#define VCNL_PROX_RES    0x87
#define VCNL_PROX_FREQ   0x89
#define VCNL_PROX_MODU   0x8A

#define MEASURE_AMBIENT   0x10
#define MEASURE_PROXIMITY 0x08
#define AMBIENT_DR        0x40
#define PROXIMITY_DR      0x20

#define VCNL_3M125        0
#define VCNL_1M5625       1
#define VCNL_781K25       2
#define VCNL_390K625      3

static uint8_t rreg(uint8_t r)
{
  uint8_t c=0;
  i2c(VCNL_ADDR,&r,1,&c,1,I2C_REPEATED_START|I2C_END_OF_TRANS);
  return c;
}

static uint16_t rreg16(uint8_t r)
{
  uint16_t c=0;
  i2c(VCNL_ADDR,&r,1,&c,2,I2C_REPEATED_START|I2C_END_OF_TRANS);
  return c;
}

static bool wreg(uint8_t r, uint8_t c)
{
  uint8_t buf[] = {r,c};
  return i2c(VCNL_ADDR,buf,2,NULL,0,I2C_REPEATED_START|I2C_END_OF_TRANS);
}

static int*
lstatus(int type)
{
  switch(type) {
  case SENSORS_ACTIVE:
    return (int*) (rreg(VCNL_AMB_PAR) & (1<<7));
  case SENSORS_READY:
    return (int*) (rreg(VCNL_CMD_REG) & (1<<6));
  }

  return NULL;
}

static int
lvalue(int type)
{
  switch(type) {
  case LIGHT_VALUE_VISIBLE_CENTILUX:
    wreg(VCNL_CMD_REG, ((1<<4)|(1<<3))); // start a measurement
    while (!(rreg(VCNL_CMD_REG) & (1<<6)))
      ;
    /* from http://www.vishay.com/docs/83395/vcnl4000_demo_kit.pdf
     *
     * response to different light sources is different average scaling factor
     * is 4 counts per lux to convert counts to LUX and not to loose precision
     * convert it to centi (10**-2) */
    return rreg16(VCNL_AMB_RES) * 100/4;
  }

  return 0;
}

static int
lconfigure(int type, int value)
{
  switch(type) {
  case SENSORS_HW_INIT:
  case SENSORS_ACTIVE:
    if (value) /* 32 samples average, continous mode */
    {
      wreg(VCNL_AMB_PAR, 0x0d);
      wreg(VCNL_IR_CURRENT, 2); // 20mA
      wreg(VCNL_CMD_REG, ((1<<4)|(1<<3))); // start a measurement
      return 1;
    }
    else
      return wreg(VCNL_AMB_PAR, 0x0d);
  }

  return 0;
}

static int*
pstatus(int type)
{
  switch(type) {
  case SENSORS_ACTIVE:
    return (int*) (rreg(VCNL_IR_CURRENT)); // == zero if not
  case SENSORS_READY:
    return (int*) (rreg(VCNL_CMD_REG) & PROXIMITY_DR);
  }

  return NULL;
}

static int
pvalue(int type)
{
  switch(type) {
  case PROXIMITY_VALUE:
    wreg(VCNL_CMD_REG, MEASURE_PROXIMITY); // start a measurement
    while (!(rreg(VCNL_CMD_REG)&PROXIMITY_DR))
      ;
    return rreg16(VCNL_PROX_RES);
  }

  return 0;
}

static int
pconfigure(int type, int value)
{
  switch(type) {
  case SENSORS_HW_INIT:
  case SENSORS_ACTIVE:
    if (value)
    {
      wreg(VCNL_IR_CURRENT, 10); // 20*10mA = 200mA
      wreg(VCNL_PROX_FREQ, VCNL_781K25);   // 781kHz
      /* dead an delay time as documented in Vishay Datasheet pg. 9 */
      return wreg(VCNL_PROX_MODU, 0x81);
    }
    else
      return wreg(VCNL_IR_CURRENT, 0); // off?!?
  }

  return 0;
}

SENSORS_SENSOR(lightlevel_sensor, LIGHTLEVEL_SENSOR,
               lvalue,lconfigure,lstatus);
SENSORS_SENSOR(proximity_sensor, PROXIMITY_SENSOR,
               pvalue,pconfigure,pstatus);
