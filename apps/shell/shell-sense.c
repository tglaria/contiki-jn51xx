/*
 * Copyright (c) 2004, Adam Dunkels.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#include <string.h>
#include <stddef.h>
#include <stdbool.h>

#include "contiki.h"
#include "shell.h"
#include "contiki-net.h"
#include "lib/sensors.h"

/*---------------------------------------------------------------------------*/
PROCESS(shell_sense_process, "sense");
PROCESS(shell_sense_list_process, "sense_list");
SHELL_COMMAND(sense_list_command,
              "sense_list",
              "sense_list: list available sensors",
              &shell_sense_list_process);
SHELL_COMMAND(sense_command,
              "sense",
              "sense <sample-rate-in-hz> <num-samples> <sensor-name> <which-value>:\n"
              " read num samples at sample rate from sensor value or\n"
              "sense <sample-rate-in-hz> <num-samples> <sensor-name>:\n"
              " read num sample at sample rate from first sensor value or\n"
              "sense <sample-rate-in-hz> <sensor-name>:\n"
              " read continiously from sensor at sample rate\n"
              "sense <sample-rate-in-hz> <sensor-name> <which-value>:\n"
              " read continiously from sensor value at sample rate\n"
              "sense <sensor> <which-value>: read one sample\n"
              "sense <sensor>: read one sample\n",
              &shell_sense_process);

/*---------------------------------------------------------------------------*/
// for multiple instances static vars need to be allocated somewhere else

PROCESS_THREAD(shell_sense_list_process, ev, data)
{
  struct sensors_sensor *sensor;
  PROCESS_BEGIN();
  PROCESS_PAUSE();

  for (sensor = sensors_first(); sensor; sensor = sensors_next(sensor))
    shell_output_str(&sense_list_command, sensor->type, "\n");

  PROCESS_EXIT();
  PROCESS_END();
}

#define DEFAULT_TIMEOUT CLOCK_SECOND

PROCESS_THREAD(shell_sense_process, ev, data)
{
  static bool   waiting;
  static struct sensors_sensor *chosen;
  static struct etimer timeout;
  struct shell_input *input;
  struct sensors_sensor *sensor;
  char *next = NULL, c;
  static u32_t samples, sample_rate, which_value, i;
  static clock_time_t starttime;

  PROCESS_BEGIN();

  chosen = NULL;
  samples = 1;
  sample_rate = which_value = 0;

  if (!process_is_running(&sensors_process))
    process_start(&sensors_process, NULL);

  if(data == NULL) {
    shell_output_str(&sense_command, "sense: missing args", "");
    PROCESS_EXIT();
  }

  sample_rate = shell_strtolong(data, &next);
  data = next;

  if(next == '\0') { /* no number given */
    shell_output_str(&sense_command, "sense: missing args", "");
    PROCESS_EXIT();
  }

  samples = shell_strtolong(data, &next);
  if (data==next && sample_rate == 0)
    samples = 1; /* no number given and no sample rate set */
  data = next;

  /* consume argument separator */
  while (*next && isblank(*next))
    next++;

  if (!*next) {
    shell_output_str(&sense_command, "sense: sensor name missing", "");
    PROCESS_EXIT();
  }

  /* find position after sensor name */
  data = next;
  while (*next && !isblank(*next))
    next++;

  if (*next) { *next = '\0'; next++; };

  chosen = sensors_find(data);

  if(chosen == NULL) {
    shell_output_str(&sense_command, "sense: sensor not found", data);
    PROCESS_EXIT();
  }

  /* jump to next number in argument string */
  while (*next && !isdigit(*next))
    next++;

  if (*next) {
    data = next;
    which_value = shell_strtolong(data, &next);
  }

  if (!chosen->configure(SENSORS_ACTIVE, 1))
  {
    shell_output_str(&sense_command, "sense: unable to activate sensor", "");
    PROCESS_EXIT();
  }

  starttime = clock_time();
  for (i=0; i<samples || samples==0; i++) {
    char buf[24];
    int value, n;

    /* if sample rate is bigger than CLOCK_SECOND interval will be zero but
     * that is okay */
    etimer_set(&timeout, CLOCK_SECOND/sample_rate);
    PROCESS_YIELD_UNTIL ((ev == sensors_event && data == chosen) ||
                          ev == PROCESS_EVENT_TIMER ||
                          ev == shell_event_input);

    if (ev==shell_event_input)
      break;

    value = chosen->value(which_value);
    n = snprintf(buf, sizeof(buf), "%d\n", value);
    shell_output(&sense_command, buf, n, NULL, 0);
  }

  chosen->configure(SENSORS_ACTIVE, 0);

  PROCESS_PAUSE();

  { /* print some statistics */
    char buf[48];
    int n;
    n = snprintf(buf, sizeof(buf), "%d samples spanning %d ms\n", i, clock_time()-starttime);
    shell_output(&sense_command, buf,n,NULL,0);
  }


  PROCESS_EXIT();
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
void
shell_sense_init(void)
{
  shell_register_command(&sense_command);
  shell_register_command(&sense_list_command);
}
/*---------------------------------------------------------------------------*/
