
#ifndef __PROJECT_CONF_H__
#define __PROJECT_CONF_H__


// /* USB serial takes space, free more space elsewhere */
// #define SICSLOWPAN_CONF_FRAG 0
// #define UIP_CONF_BUFFER_SIZE 160

/*******************************************************/
/******************* Configure TSCH ********************/
/*******************************************************/


// #define TSCH_CONF_INIT_SCHEDULE_FROM_EB 0

#undef NBR_TABLE_CONF_MAX_NEIGHBORS
#define NBR_TABLE_CONF_MAX_NEIGHBORS 64 //Max neighbors
// #define TSCH_QUEUE_CONF_NUM_PER_NEIGHBOR 16 //Max 8 packets per neighbor
// #define QUEUEBUF_CONF_NUM 20 //Max 10 packets in queue total
#undef TSCH_SCHEDULE_CONF_MAX_LINKS
#define TSCH_SCHEDULE_CONF_MAX_LINKS 64

#undef QUEUEBUF_CONF_NUM
#define QUEUEBUF_CONF_NUM 32

#undef NETSTACK_MAX_ROUTE_ENTRIES
#define NETSTACK_MAX_ROUTE_ENTRIES 128

/*******************************************************/
/************* Platform dependent configuration ********/
/*******************************************************/

/* USB serial takes space, free more space elsewhere */
//#define SICSLOWPAN_CONF_FRAG 0
//#define UIP_CONF_BUFFER_SIZE 160

//#define UIP_CONF_IPV6_CHECKS 1

//#define SICSLOWPAN_CONF_COMPRESSION SICSLOWPAN_COMPRESSION_6LORH

/* Enables energest */
#define ENERGEST_CONF_ON 1

/* IEEE802.15.4 PANID */
#define IEEE802154_CONF_PANID 0x81a2
#define TSCH_CONF_JOIN_MY_PANID_ONLY 1

//#define TSCH_HOPPING_SEQUENCE_MY_SEQUENCE (uint8_t[]){17, 23, 15, 25, 19, 13, 21}//(uint8_t[]){17, 23, 15, 25, 19, 11, 13, 21}
//#define TSCH_HOPPING_SEQUENCE_MY_SEQUENCE (uint8_t[]){13}
#define TSCH_CONF_DEFAULT_HOPPING_SEQUENCE TSCH_HOPPING_SEQUENCE_16_16 //TSCH_HOPPING_SEQUENCE_MY_SEQUENCE
//#define TSCH_CONF_DEFAULT_HOPPING_SEQUENCE TSCH_HOPPING_SEQUENCE_MY_SEQUENCE

#define TSCH_CONF_AUTOSTART 0

#define FRAME802154_CONF_VERSION FRAME802154_IEEE802154_2015
/* Need more DIOs from RPL for more EBs */

/* RPL Trickle timer tuning */
//#define RPL_CONF_DIO_INTERVAL_MIN 12 /* 4.096 s */

//#define RPL_CONF_DIO_INTERVAL_DOUBLINGS 2 /* Max factor: x4. 4.096 s * 4 = 16.384 s */

//#define TSCH_CONF_EB_PERIOD (4 * CLOCK_SECOND)
//#define TSCH_CONF_KEEPALIVE_TIMEOUT (24 * CLOCK_SECOND)

#define WITH_TSCH_SECURITY 0

/* Logging */
#define LOG_CONF_LEVEL_RPL                         LOG_LEVEL_WARN
#define LOG_CONF_LEVEL_TCPIP                       LOG_LEVEL_WARN
#define LOG_CONF_LEVEL_IPV6                        LOG_LEVEL_WARN
#define LOG_CONF_LEVEL_6LOWPAN                     LOG_LEVEL_WARN
#define LOG_CONF_LEVEL_MAC                         LOG_LEVEL_ERR
#define LOG_CONF_LEVEL_FRAMER                      LOG_LEVEL_WARN//LOG_LEVEL_DBG
#define TSCH_LOG_CONF_PER_SLOT                     0//1

//#include "../common-conf.h"

#endif /* __PROJECT_CONF_H__ */
