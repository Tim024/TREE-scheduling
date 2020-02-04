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
 *
 * \author Tim van der Lee <t.lee@tue.nl>
 */

#ifndef __SCHEDULING_H__
#define __SCHEDULING_H__

#include "lib/ringbufindex.h"
#include "net/mac/tsch/tsch.h"
#include "net/mac/tsch/tsch-conf.h"
#include "net/mac/tsch/tsch-schedule.h"
#include "net/mac/tsch/tsch-slot-operation.h"
#include "TREE-conf.h"

/* Message structure for interaction with neighbors */
#define TYPE_ADD 1
#define TYPE_ADD_OK 2
struct message{
  uint16_t timeslot[TREE_CELLS_PER_COORD];
  uint16_t channel_offset[TREE_CELLS_PER_COORD];
  uint8_t type;
};// __attribute__ ((aligned(1))); did not work with JN516x

/* States of the algorithm */
#define ACTION_IDLE         1
#define ACTION_SEND_ADD     2
#define ACTION_ADD_WAITACK  3
#define ACTION_ADDLINK_TX   4
#define ACTION_ADDLINK_RX   5
#define ACTION_REMOVELINK   6

/******** Callbacks ********/

/* Act on state of the algorithm, to be called with tsch_process_pending*/
void tree_process_pending();
/* Callback for control packet input */
void control_packet_input();
/* MAC callback for control packet sent */
void control_packet_sent(void *ptr, int status, int transmissions);
/* Callbacks for tsch-slot-operation */
void tree_callback_slot_rx(struct input_packet * packet, int is_frame_valid, struct tsch_link* link);
void tree_callback_slot_tx(const linkaddr_t * dest, uint8_t mac_status, struct tsch_link * link);

/* Callbacks for tsch-queue */
void tree_callback_new_time_source(const struct tsch_neighbor *old, const struct tsch_neighbor *new);
void tree_callback_packet_ready();
void tree_callback_packet_added_in_queue(const linkaddr_t *addr, struct tsch_neighbor * n);

/* Callbacks when neighbor are added or removed */
void tree_callback_nbr_added(const linkaddr_t *n);
void tree_callback_nbr_removed(const linkaddr_t *n);

/******** Functions ********/
/* Initialize the slotframes */
void tree_init();

#endif /* __SCHEDULING_H__ */
