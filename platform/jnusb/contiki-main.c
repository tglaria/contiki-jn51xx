#include "contiki-net.h"
#include "net/ieee802.h"
#include "gdb2.h"
#include "init.h"
#include "jts.h"

PROCINIT(&etimer_process);

void
init_net(void)
{
  /* load link-layer address */
  memcpy(uip_lladdr.addr, pvAppApiGetMacAddrLocation(), sizeof(uip_lladdr.addr));
  netstack_init();
}

void
AppColdStart(void)
{
  /* default startup */
  init_hardware();
  process_init();
  procinit_init();
  init_net();
  jts_init();

  /* application startup */
  autostart_start(autostart_processes);

  /* enable watchdog on JN5148, there is none on JN5139 */
  watchdog_start();


  /* default main loop */
  while(1)
  {
    process_run();
    etimer_request_poll();
    watchdog_periodic();
  }
}

void
AppWarmStart(void)
{
  AppColdStart();
}

void
uip_log(char *msg)
{
  GDB2_PUTS(msg);
}
