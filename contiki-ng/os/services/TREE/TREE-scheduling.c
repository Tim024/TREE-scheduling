/*
 * Copyright (c) 2018, Eindhoven University of Technology.
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
 *
 */

/**
 * \file
 *         TREE scheduling algorithm
 *
 * \author Tim van der Lee <t.lee@tue.nl>
 */

#include "contiki.h"
#include "TREE-scheduling.h"
#include "TREE-infq.h"
#include "node-id.h"
#include "net/queuebuf.h"
#include "net/packetbuf.h"
#include "net/ipv6/uip-icmp6.h"
#include "net/routing/routing.h"
#include "net/mac/tsch/tsch.h"
#include "net/mac/tsch/tsch-schedule.h"
#include "net/mac/tsch/tsch-queue.h"
#if ROUTING_CONF_RPL_LITE
#include "net/routing/rpl-lite/rpl.h"
#elif ROUTING_CONF_RPL_CLASSIC //Do not work ?
#include "net/routing/rpl-classic/rpl.h"
#endif

/* Log configuration */
#include "sys/log.h"
#define LOG_MODULE "TREE"
#define LOG_LEVEL LOG_LEVEL_INFO

/* Sends a control message to the queue */
static int send_control_message(const linkaddr_t * dest, uint8_t type, uint16_t timeslot, uint16_t channel_offset);

/* The scheduling background process */
PROCESS(tree_scheduling_process, "TREE-scheduling process");

static struct tsch_slotframe *slotframe_data;
static struct tsch_slotframe *slotframe_bc;
static struct tsch_slotframe *slotframe_eb;

//Counter for how long we wait to receive acknowledgment of control packet
static int waitack_counter = 0;

/* Message buffer for control messages */
struct message message_buffer;
/* Action info and state */
linkaddr_t action_address;
uint16_t action_timeslot = 0;
uint16_t action_channel = 0;
uint8_t action_state = ACTION_IDLE;

/*---------------------------------------------------------------------------*/
/* Priorityframes are FRAME802154_CMDFRAME */
void
control_packet_input()
{
  NETSTACK_FRAMER.parse();

  struct message msg;
  memcpy(&msg, packetbuf_dataptr(), sizeof(struct message));
  //struct message * msg = (struct message *) packetbuf_dataptr(); -> ALIGN exception

  linkaddr_t addr;
  linkaddr_copy(&addr,packetbuf_addr(PACKETBUF_ADDR_SENDER));
  uint8_t type = msg.type;
  uint16_t ts = msg.timeslot;
  uint16_t co = msg.channel_offset;

  if(action_state == ACTION_IDLE){
    linkaddr_copy(&action_address,&addr);
    action_timeslot = ts;
    action_channel = co;
    if(type == TYPE_ADD){
      if(tsch_schedule_get_link_by_timeslot(slotframe_data, ts) == NULL){
        LOG_DBG("type ADD rx, I send add ok\n");
        if(1 == send_control_message(&action_address,TYPE_ADD_OK,ts,co))
          action_state = ACTION_ADD_WAITACK;
      }
    } else if (type == TYPE_ADD_OK){
      LOG_DBG("type ADD OK rx\n");
      action_state = ACTION_ADDLINK_TX;
    }
  }
  if(action_state == ACTION_SEND_ADD && type == TYPE_ADD_OK){
    //If we wanted to request a link with this neighbor, ignore demand and add the link
    if (linkaddr_cmp(&addr,&action_address)){
      action_timeslot = ts;
      action_channel = co;
      LOG_DBG("type ADD OK rx overwriting send ADD\n");
      action_state = ACTION_ADDLINK_TX;
    }
  }

  LOG_DBG("input control %u len %u--  %u.%u.%u.%u -- ts %u co %u typ %u\n",action_state,packetbuf_datalen(),addr.u8[0],addr.u8[1],addr.u8[2],addr.u8[3],ts,co,type);
}
/*---------------------------------------------------------------------------*/
void
control_packet_sent(void *ptr, int status, int transmissions)
{
  //struct message * message = (struct message *) ptr;
  //LOG_DBG("control sent %u ts %u co %u typ %u\n",status, message->timeslot,message->channel_offset,message->type);

  /* If state == ACTION_ADD_WAITACK, update accordingly */
  if(action_state == ACTION_ADD_WAITACK){
    if(status == MAC_TX_OK){
      LOG_DBG("control: ADD OK acknowledged\n");
      action_state = ACTION_ADDLINK_RX; //sometime acknowledged but not added
      waitack_counter = 0;
    }
  }
}
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
void
tree_callback_nbr_removed(const linkaddr_t *n)
{
#if TREE_BASIC_CELL
/* Remove unused cell */
  //tsch_schedule_remove_link_by_timeslot(slotframe_data, TREE_HASH(&n->addr));
#endif
  LOG_DBG("TO BE IMPLEMENTED? nb rm\n");
}
/*---------------------------------------------------------------------------*/
void
tree_callback_nbr_added(const linkaddr_t *n)
{
  struct tsch_neighbor * nb = tsch_queue_get_nbr(n);
  if(nb == NULL || linkaddr_cmp(n,&tsch_broadcast_address) || linkaddr_cmp(n,&tsch_eb_address)){
    return;
  }

#if TREE_BASIC_CELL
/* One Tx per neighbor */
tsch_schedule_add_link(slotframe_data,
    LINK_OPTION_TX | LINK_OPTION_SHARED,
    LINK_TYPE_NORMAL, n,
    TREE_HASH(n)%TREE_SF_SIZE, 2);
#endif
  LOG_DBG("nbr add %u.%u creating cell at %u\n",n->u8[0],n->u8[1], TREE_HASH(n));
}
/*---------------------------------------------------------------------------*/
static int control_packet_seqno = 1;
/* Sends a control message to the queue */
static int
send_control_message(const linkaddr_t * dest, uint8_t type, uint16_t timeslot, uint16_t channel_offset)
{

  if(!tsch_is_associated || linkaddr_cmp(dest,&tsch_broadcast_address) || linkaddr_cmp(dest,&linkaddr_null)) {
    mac_call_sent_callback(control_packet_sent, &message_buffer, MAC_TX_ERR, 1);
    return 0;
  }

  message_buffer.type = type;
  message_buffer.timeslot = timeslot;
  message_buffer.channel_offset = channel_offset;

  packetbuf_clear();
  packetbuf_copyfrom(&message_buffer,sizeof(struct message));

  packetbuf_set_attr(PACKETBUF_ATTR_FRAME_TYPE, FRAME802154_CMDFRAME);
  packetbuf_set_addr(PACKETBUF_ADDR_SENDER, &linkaddr_node_addr);
  packetbuf_set_addr(PACKETBUF_ADDR_RECEIVER, dest);

  //NETSTACK_MAC.send(control_packet_sent, &message_buffer);
  int ret = MAC_TX_DEFERRED;
  int hdr_len = 0;
  uint8_t max_transmissions = TSCH_MAC_MAX_FRAME_RETRIES + 1;
  if(++control_packet_seqno == 0) {
    control_packet_seqno++;
  }
  packetbuf_set_attr(PACKETBUF_ATTR_MAC_SEQNO, control_packet_seqno);
  packetbuf_set_attr(PACKETBUF_ATTR_MAC_ACK, 1);
  packetbuf_set_attr(PACKETBUF_ATTR_FRAME_TYPE, FRAME802154_CMDFRAME);
#if LLSEC802154_ENABLED
  if(tsch_is_pan_secured) {
    /* Set security level, key id and index */
    packetbuf_set_attr(PACKETBUF_ATTR_SECURITY_LEVEL, TSCH_SECURITY_KEY_SEC_LEVEL_OTHER);
    packetbuf_set_attr(PACKETBUF_ATTR_KEY_ID_MODE, FRAME802154_1_BYTE_KEY_ID_MODE); /* Use 1-byte key index */
    packetbuf_set_attr(PACKETBUF_ATTR_KEY_INDEX, TSCH_SECURITY_KEY_INDEX_OTHER);
  }
#endif /* LLSEC802154_ENABLED */

  if((hdr_len = NETSTACK_FRAMER.create()) < 0) {
    LOG_ERR("! can't send packet due to framer error\n");
    ret = MAC_TX_ERR;
  } else {
    struct tsch_packet *p = NULL;
    /* Enqueue packet */
#if TREE_WITH_PRIORITY_CONTROL
  struct tsch_neighbor * n = tsch_queue_add_nbr(dest);
  if(n != NULL) {
    //LOG_DBG("control: dequeue everything\n");
    struct tsch_packet * tmp_buffer[TSCH_QUEUE_NUM_PER_NEIGHBOR];
    int16_t get_counter = 0;
    int16_t put_counter = 0;

    if(!tsch_is_locked()) {
       // LOG_DBG("DEBUG from %u\n",tsch_queue_packet_count(dest));
      if(tsch_get_lock()){
        int16_t get_index = ringbufindex_get(&n->tx_ringbuf);
        /* Fill the tmp buffer packets with all packets in queue */
        while (get_index != -1){
           // LOG_DBG("DEBUG Dequeueing tmp_buffer[%u]=n->tx_array[%u]\n",get_counter,get_index);
          tmp_buffer[get_counter] = n->tx_array[get_index];
          get_counter ++;
          get_index = ringbufindex_get(&n->tx_ringbuf);
        }

        if (get_counter == TSCH_QUEUE_NUM_PER_NEIGHBOR - 1){
          get_counter --;
          LOG_WARN("removing packet to put control packet %u %u\n",get_counter,TSCH_QUEUE_NUM_PER_NEIGHBOR);
          tsch_queue_free_packet(tmp_buffer[get_counter]);
        }

        tsch_release_lock();
      }



      /* Fill the queue with tmp_buffer and with the priority packet */
       // LOG_DBG("DEBUG add control packet %u\n",tsch_queue_packet_count(dest));

      p = tsch_queue_add_packet(dest, max_transmissions, control_packet_sent, &message_buffer);

       // LOG_DBG("DEBUG added control packet %u\n",tsch_queue_packet_count(dest));

      if(tsch_get_lock()){
        int16_t put_index = ringbufindex_peek_put(&n->tx_ringbuf);
        while(put_index != -1 && put_counter != get_counter) {
           // LOG_DBG("DEBUG Requeueing n->tx_array[%u]=tmp_buffer[%u]\n",put_index,put_counter);
          ringbufindex_put(&n->tx_ringbuf);
          n->tx_array[put_index] = tmp_buffer[put_counter];
          put_counter ++;
          put_index = ringbufindex_peek_put(&n->tx_ringbuf);
        }

        if (put_index == -1 && put_counter == get_counter-1){
          LOG_WARN("removing packet %u %u\n",put_counter,get_counter);
          tsch_queue_free_packet(tmp_buffer[get_counter]);
        }

        tsch_release_lock();
      }

       // LOG_DBG("DEBUG to %u\n",tsch_queue_packet_count(dest));


    } else {
      LOG_ERR("!couldn't get lock\n");
      return 0;
    }
  } else {
  LOG_ERR("!null neighbor\n");
  return 0;
  }
#else
    p = tsch_queue_add_packet(dest, max_transmissions, control_packet_sent, &message_buffer);
#endif /* TREE_WITH_PRIORITY_CONTROL */
    if(p == NULL) {
      ret = MAC_TX_ERR;
    } else {
      p->header_len = hdr_len;
    }
  }
  if(ret != MAC_TX_DEFERRED) {
    mac_call_sent_callback(control_packet_sent, &message_buffer, ret, 1);
    return 0;
  }
  LOG_DBG("Send control to %u %u !\n",dest->u8[7],dest->u8[6]);
  struct tsch_neighbor * n = tsch_queue_get_nbr(dest);
  n->ctrl_packet_in_queue = 1;
  return 1;
}
/*---------------------------------------------------------------------------*/
void
tree_callback_packet_added_in_queue(const linkaddr_t *addr, struct tsch_neighbor * n)
{
  /* Update PHI */
  if(n != n_eb && n != n_broadcast){
    //if(mac_status == MAC_TX_OK){
    uint16_t previous_phi = n->phi;
    n->phi = update_phi(n->phi,tsch_queue_packet_count(addr),n->dedicated_tx_links_count);
    LOG_DBG("update PHI for %u:%u %u->%u QUEUE %u/%u L %u\n", addr->u8[6], addr->u8[7], previous_phi, n->phi, tsch_queue_packet_count(addr), QUEUEBUF_NUM-tsch_queue_global_packet_count(), n->dedicated_tx_links_count);

    /* Send a request for unicast link if PHI is too high */
    if(!check_phi(n->phi) && action_state == ACTION_IDLE && n->ctrl_packet_in_queue == 0){
      LOG_DBG("pls send ADD to %u:%u PHI %u\n",addr->u8[6],addr->u8[7],n->phi);
      action_state = ACTION_SEND_ADD;
      linkaddr_copy(&action_address,addr);
    }
    //}
  }
}
/*---------------------------------------------------------------------------*/
void
tree_callback_slot_rx(struct input_packet * packet, int is_frame_valid, struct tsch_link* link)
{
  if (!is_frame_valid){
    link->psi = update_psi_err(link->psi);
    return;
  }
  link->psi = update_psi(link->psi);
}

/*---------------------------------------------------------------------------*/
void
tree_callback_slot_tx(const linkaddr_t * dest, uint8_t mac_status, struct tsch_link * link)
{

  /* Update PSI according to transmission status */
  if(mac_status != MAC_TX_OK){
    link->psi = update_psi_err(link->psi);
    return;
  }
  struct tsch_neighbor* n = tsch_queue_get_nbr(dest);
  if(n != NULL) {
    if(n->ctrl_packet_in_queue == 1){
      n->ctrl_packet_in_queue = 0;
      // n->phi = TREE_PHI_RESET;
    }
  }

  link->psi = update_psi(link->psi);

}
/*---------------------------------------------------------------------------*/
void
tree_callback_new_time_source(const struct tsch_neighbor *old, const struct tsch_neighbor *new)
{
#if TREE_EB_ON_RPL_PARENT
  uint16_t old_ts = old != NULL ? TREE_HASH(&old->addr)%TREE_SF_EB_SIZE : 0xffff;
  uint16_t new_ts = new != NULL ? TREE_HASH(&new->addr)%TREE_SF_EB_SIZE : 0xffff;

  if(new_ts == old_ts) {
    return;
  }

  if(old_ts != 0xffff) {
    /* Stop listening to the old time source's EBs */
    if(old_ts == TREE_HASH(&linkaddr_node_addr)%TREE_SF_EB_SIZE) {
      /* This was the same timeslot as slot. Reset original link options */
      tsch_schedule_add_link(slotframe_eb, LINK_OPTION_TX | LINK_OPTION_SHARED, LINK_TYPE_ADVERTISING_ONLY,
        &tsch_broadcast_address, old_ts, 1);
    } else {
      /* Remove slot */
      tsch_schedule_remove_link_by_timeslot(slotframe_eb, old_ts);
    }
  }
  if(new_ts != 0xffff) {
    uint8_t link_options = LINK_OPTION_RX | LINK_OPTION_SHARED;
    if(new_ts == TREE_HASH(&linkaddr_node_addr)%TREE_SF_EB_SIZE) {
      /* This is also our timeslot, add necessary flags */
      link_options |= LINK_OPTION_TX;
    }
    /* Listen to the time source's EBs */
    tsch_schedule_add_link(slotframe_eb, link_options, LINK_TYPE_ADVERTISING_ONLY,
      &tsch_broadcast_address, new_ts, 1);
  }
#endif
}
/*---------------------------------------------------------------------------*/
void
tree_process_pending()
{
  /* Check action to be done */
  if(action_state != ACTION_IDLE){
    struct tsch_neighbor * n;
    n = tsch_queue_add_nbr(&action_address); //get ?
    if(action_state == ACTION_REMOVELINK){
      if (1 == tsch_schedule_remove_link_by_timeslot(slotframe_data,action_timeslot)){
        LOG_INFO("ACTION REMOVELINK %d\n",action_timeslot);
      }
      action_state = ACTION_IDLE;
    }
    if (action_state == ACTION_ADDLINK_TX){
      n->phi = TREE_PHI_RESET;
      struct tsch_link * li = tsch_schedule_add_link(slotframe_data, LINK_OPTION_TX, LINK_TYPE_NORMAL, &action_address, action_timeslot, action_channel);
      if (li == NULL){
        LOG_ERR("Could not add link TX %d\n",action_timeslot);
      } else {
        li->psi = TREE_PSI_RESET;
        LOG_INFO("ACTION ADDLINK TX ts %d co %d addr %u:%u:%u:%u\n", action_timeslot, action_channel, action_address.u8[4], action_address.u8[5], action_address.u8[6], action_address.u8[7]);
      }
      action_state = ACTION_IDLE;
    } else if (action_state == ACTION_ADDLINK_RX){
      n->phi = TREE_PHI_RESET;
      struct tsch_link * li = tsch_schedule_add_link(slotframe_data, LINK_OPTION_RX, LINK_TYPE_NORMAL, &action_address, action_timeslot, action_channel);
      if (li == NULL){
        LOG_ERR("Could not add link RX %d\n",action_timeslot);
      } else {
        li->psi = TREE_PSI_RESET;
        LOG_INFO("ACTION ADDLINK RX ts %d co %d addr %u:%u:%u:%u\n", action_timeslot, action_channel, action_address.u8[4], action_address.u8[5], action_address.u8[6], action_address.u8[7]);
      }
      action_state = ACTION_IDLE;
    } else if (action_state == ACTION_ADD_WAITACK){
      LOG_DBG("ACTION WAIT ACK %d/180\n",waitack_counter);
      waitack_counter ++;
      if(waitack_counter > 180){
        action_state = ACTION_IDLE;
        waitack_counter = 0;
      }
    } else if (action_state == ACTION_SEND_ADD){
      //n->phi = TREE_PHI_RESET;
      /* Finds a free timeslot in slotframe_data */
      uint16_t timeslot = 0;
      uint16_t channel = 0;
      int stop = 0;
      while(stop == 0){
        timeslot = random_rand() % TREE_SF_SIZE;
        channel = random_rand() % 16;
        if(tsch_schedule_get_link_by_timeslot(slotframe_data, timeslot) == NULL){
          stop = 1;
        }
      }
      LOG_INFO("ACTION SEND ADD\n")
      /* Sends the control message */;
      send_control_message(&action_address,TYPE_ADD,timeslot,channel);
      action_state = ACTION_IDLE;
    }
  }
}
/*---------------------------------------------------------------------------*/
void
tree_callback_packet_ready() //Select SF0 when possible
{
  uint16_t slotframe = 0xffff;
  uint16_t timeslot = 0xffff;

  linkaddr_t addr;
  linkaddr_copy(&addr,packetbuf_addr(PACKETBUF_ADDR_RECEIVER));

  if(packetbuf_attr(PACKETBUF_ATTR_FRAME_TYPE) == FRAME802154_BEACONFRAME){
    slotframe = TREE_SF_EB_HANDLE;
  } else {
    struct tsch_neighbor * nb = tsch_queue_get_nbr(&addr);
    if (nb != NULL){
      if (nb->dedicated_tx_links_count > 0 && !(nb->is_broadcast)){
        slotframe = TREE_SF_HANDLE;
        //printf("packet ready assigning sf handle %u ...\n", TREE_SF_HANDLE);
      }// else { printf("packet ready not enough links or is broadcast %u %u \n",nb->dedicated_tx_links_count, nb->is_broadcast); }
    } //else { printf("packet ready null neighbor...\n"); }
  }



#if TSCH_WITH_LINK_SELECTOR
  packetbuf_set_attr(PACKETBUF_ATTR_TSCH_SLOTFRAME, slotframe);
  packetbuf_set_attr(PACKETBUF_ATTR_TSCH_TIMESLOT, timeslot);
#endif
  //LOG_DBG("packet ready on SF%u TS%u ",slotframe,timeslot);
  //L//OG_DBG_LLADDR(&addr);
  //LOG_DBG_("\n");
  return;
}
/*---------------------------------------------------------------------------*/
void
tree_init(void)
{

  slotframe_data = tsch_schedule_add_slotframe(TREE_SF_HANDLE, TREE_SF_SIZE);
  slotframe_bc = tsch_schedule_add_slotframe(TREE_SF_BC_HANDLE, TREE_SF_BC_SIZE);
  slotframe_eb = tsch_schedule_add_slotframe(TREE_SF_EB_HANDLE, TREE_SF_EB_SIZE);

#if TREE_EB_ON_RPL_PARENT
  tsch_schedule_add_link(slotframe_eb,
      LINK_OPTION_TX | LINK_OPTION_SHARED,
      LINK_TYPE_ADVERTISING_ONLY, &tsch_broadcast_address,
      TREE_HASH(&linkaddr_node_addr)%TREE_SF_EB_SIZE, 1);
#else
  tsch_schedule_add_link(slotframe_eb,
      LINK_OPTION_TX | LINK_OPTION_RX| LINK_OPTION_SHARED,
      LINK_TYPE_ADVERTISING_ONLY, &tsch_broadcast_address,
      0, 1);
#endif /* TREE_EB_ON_RPL_PARENT */

#if TREE_BASIC_CELL
  /* One Tx per neighbor (receiver-based) */
  tsch_schedule_add_link(slotframe_data,
      LINK_OPTION_RX| LINK_OPTION_SHARED,
      LINK_TYPE_NORMAL, &tsch_broadcast_address,
      TREE_HASH(&linkaddr_node_addr)%TREE_SF_SIZE, 2);
#endif /* TREE_BASIC_CELL */

  /* Always one common broadcast cell */
  tsch_schedule_add_link(slotframe_bc,
      LINK_OPTION_TX | LINK_OPTION_RX | LINK_OPTION_SHARED ,
      LINK_TYPE_NORMAL, &tsch_broadcast_address,
      0, 0);

  tsch_schedule_print();

  process_start(&tree_scheduling_process, NULL);
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(tree_scheduling_process, ev, data)
{
  PROCESS_BEGIN();

  static struct etimer et;

  etimer_set(&et, TREE_EVAP_PERIOD);

  static int counter = 0;

  while(1){
    PROCESS_WAIT_UNTIL(etimer_expired(&et));
    /* Evaporate all PHI for neighbors */
    struct tsch_neighbor * n;
    for(n = get_neighbor_list_head(); n != NULL; n = n->next){
      if (n == n_eb || n == n_broadcast) continue; //unsupported
      n->phi = evaporate(n->phi);
    }

    /* Evaporate all PSI for links */
    struct tsch_link * l;
    for (l = list_head(slotframe_data->links_list); l != NULL; l = list_item_next(l)){
      l->psi = evaporate(l->psi);

      /* Delete when PSI is too low, if the link is not shared and if state == idle */
      if(!check_psi(l->psi) &&
          action_state == ACTION_IDLE &&
          !(l->link_options & LINK_OPTION_SHARED)){

        action_state = ACTION_REMOVELINK;
        linkaddr_copy(&action_address,&l->addr);
        action_timeslot = l->timeslot;
        action_channel = l->channel_offset;
      }
    }

    counter = counter;
    /* Debug print*/
    counter ++;
    if(counter > 30){
      counter = 0;
      printf("TREE: PHI");
      for(n = get_neighbor_list_head(); n != NULL; n = n->next){
        if (n == n_eb || n == n_broadcast) continue;
        printf(" %x:%x:%x:%x:%x:%x:%x:%x %u %d %u", n->addr.u8[0], n->addr.u8[1], n->addr.u8[2], n->addr.u8[3], n->addr.u8[4], n->addr.u8[5], n->addr.u8[6], n->addr.u8[7], n->phi,  tsch_queue_packet_count(&n->addr), n->dedicated_tx_links_count);
      }
      printf(" PSI");
      uint16_t link_counter = 0;
      for (l = list_head(slotframe_data->links_list); l != NULL; l = list_item_next(l)){
        link_counter ++ ;
        printf(" %u %lu", l->timeslot, l->psi);
      }
      printf(" STATE %u\n",action_state);
      //printf("TIM DBG %u, %u\n",TREE_EB_ON_RPL_PARENT,TREE_HASH(&linkaddr_node_addr));
      //tsch_schedule_print();
      if(link_counter > TSCH_SCHEDULE_MAX_LINKS-TSCH_SCHEDULE_MAX_LINKS/4){
        LOG_WARN("Too many links allocated (%u/%u)!\n",link_counter,TSCH_SCHEDULE_MAX_LINKS);
      }
      counter ++;
    }

    // if(!tsch_is_locked()) {
    //   uint16_t total_packet_mem = 0;
    //   for(n = get_neighbor_list_head(); n != NULL; n = n->next){
    //     total_packet_mem += ringbufindex_elements(&n->tx_ringbuf);
    //     // printf("DEBUG NB %u SZ %u ",n->addr.u8[1],ringbufindex_elements(&n->tx_ringbuf));
    //   }
    //   if (total_packet_mem != tsch_queue_global_packet_count()){
    //     printf("DEBUG mismatch %u %u",total_packet_mem,tsch_queue_global_packet_count());
    //   }
    //   printf("\n");
    // }
    etimer_reset(&et);
  }

  PROCESS_END();
}
