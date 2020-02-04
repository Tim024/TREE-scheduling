#include "contiki.h"
#include "node-id.h"
#include "sys/log.h"
#include "net/ipv6/uip-ds6-route.h"
#include "net/ipv6/uip-sr.h"
#include "net/mac/tsch/tsch.h"
#include "net/mac/tsch/tsch-log.h"
#include "net/routing/routing.h"
#include "net/ipv6/simple-udp.h"
#include "lib/random.h"
/*For pwrtrc*/
#include "os/sys/compower.h"
#include "os/sys/energest.h"
#include "contiki-lib.h"


#define DEBUG DEBUG_PRINT
#include "net/ipv6/uip-debug.h"

/* Log configuration */
#include "os/sys/log.h"
#define LOG_MODULE "TREE-NODE"
#define LOG_LEVEL LOG_LEVEL_DBG

/*---------------------------------------------------------------------------*/
PROCESS(node_process, "TREE-RPL Node");
AUTOSTART_PROCESSES(&node_process);

static struct simple_udp_connection udp_conn;

/*---------------------------------------------------------------------------*/
static void
udp_rx_callback(struct simple_udp_connection *c,
         const uip_ipaddr_t *sender_addr,
         uint16_t sender_port,
         const uip_ipaddr_t *receiver_addr,
         uint16_t receiver_port,
         const uint8_t *data,
         uint16_t datalen)
{
  //char* msg;
  //memcpy(msg, data, 32);
  /* If tagging of traffic class is enabled tc will print number of
     transmission - otherwise it will be 0 */
  LOG_INFO("PACKETRX %s (tc:%d) from ", (char*)data,
           uipbuf_get_attr(UIPBUF_ATTR_MAX_MAC_TRANSMISSIONS));
  LOG_INFO_6ADDR(sender_addr);
  LOG_INFO_("\n");
}
/*---------------------------------------------------------------------------*/
static inline unsigned long
to_seconds(uint64_t time)
{
  return (unsigned long)(time / ENERGEST_SECOND);
}
/*---------------------------------------------------------------------------*/
static void
powertrace_print(char *str)
{
  energest_flush();

  LOG_INFO_LLADDR(&linkaddr_node_addr);
  LOG_INFO_(" ENERGEST CPU %llu LPM %llu DEEPLPM %llu Totaltime %llu LISTEN %llu TRANSMIT %llu SEC %d ID:%u\n",
         (energest_type_time(ENERGEST_TYPE_CPU)),
         (energest_type_time(ENERGEST_TYPE_LPM)),
         (energest_type_time(ENERGEST_TYPE_DEEP_LPM)),
         (ENERGEST_GET_TOTAL_TIME()),
         (energest_type_time(ENERGEST_TYPE_LISTEN)),
         (energest_type_time(ENERGEST_TYPE_TRANSMIT)),
        ENERGEST_SECOND, node_id);
}
/*---------------------------------------------------------------------------*/
char buffer[32];
PROCESS_THREAD(node_process, ev, data)
{
  static int is_coordinator;

  static struct etimer periodic_timer;
  static unsigned count = 0;
  uip_ipaddr_t dest_ipaddr;

  PROCESS_BEGIN();

  //is_coordinator = 0;
  //41104 = lille m3-151
  //41331 = m3-152
  //154 b071 45169
  //firefly 9 36923

  is_coordinator = 0;//(node_id == 1 || node_id == 45169 || node_id == 36923);

  if(is_coordinator) {
    NETSTACK_ROUTING.root_start();
  }
  NETSTACK_MAC.on();

  /* Initialize UDP connection */
  simple_udp_register(&udp_conn, 1234, NULL,
                      1234, udp_rx_callback);


  etimer_set(&periodic_timer, (4+random_rand() % 3)*CLOCK_SECOND);
  while(1) {
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&periodic_timer));

    powertrace_print("yolo");

    if(!is_coordinator && NETSTACK_ROUTING.node_is_reachable() && NETSTACK_ROUTING.get_root_ipaddr(&dest_ipaddr)) {

      sprintf(buffer, "ID:%u-%u", node_id,count);
      /* Send to DAG root */
      LOG_INFO("PACKETTX %s to ", buffer);
      LOG_INFO_6ADDR(&dest_ipaddr);
      LOG_INFO_("\n");
      /* Set the number of transmissions to use for this packet -
         this can be used to create more reliable transmissions or
         less reliable than the default. Works end-to-end if
         UIP_CONF_TAG_TC_WITH_VARIABLE_RETRANSMISSIONS is set to 1.
       */
      uipbuf_set_attr(UIPBUF_ATTR_MAX_MAC_TRANSMISSIONS, 8);
      simple_udp_sendto(&udp_conn, &buffer, sizeof(buffer), &dest_ipaddr);
      count++;

    } else {
      LOG_INFO("Not reachable yet\n");
    }

      etimer_reset(&periodic_timer);

  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
