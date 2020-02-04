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
 *         TREE scheduling algorithm configuration
 *
 * \author Tim van der Lee <t.lee@tue.nl>
 */

#ifndef __TREE_CONF_H__
#define __TREE_CONF_H__

#define TREE_WITH_RPL 1


//TODO Test ROUTING_CONF_RPL_LITE

/*********** Configuration related to information quantities (I.Q.) **********/
/*****************************************************************************/

/* PHI (I.Q. per neighbor) maximum value, determining cell allocation. */
#ifdef TREE_CONF_PHI_MAX
#define TREE_PHI_MAX TREE_CONF_PHI_MAX
#else
#define TREE_PHI_MAX 9802
#endif /*TREE_CONF_PHI_MAX*/

/* PSI (I.Q. per link) minimum value, determining cell deallocation. */
#ifdef TREE_CONF_PSI_MIN
#define TREE_PSI_MIN TREE_CONF_PSI_MIN
#else
#define TREE_PSI_MIN 30
#endif /*TREE_CONF_PSI_MIN*/

/* PHI (I.Q. per neighbor) increment value */
#ifdef TREE_CONF_PHI_INCR_VALUE
#define TREE_PHI_INCR_VALUE TREE_CONF_PHI_INCR_VALUE
#else
#define TREE_PHI_INCR_VALUE 2000
#endif /*TREE_CONF_PHI_INCR_VALUE*/

/* PSI (I.Q. per link) increment value */
#ifdef TREE_CONF_PSI_INCR_VALUE
#define TREE_PSI_INCR_VALUE TREE_CONF_PSI_INCR_VALUE
#else
#define TREE_PSI_INCR_VALUE 2000
#endif /*TREE_CONF_PSI_INCR_VALUE*/

/* PSI (I.Q. per link) punishment factor */
#ifdef TREE_CONF_PSI_PUNISHMENT_FACTOR
#define TREE_PSI_PUNISHMENT_FACTOR TREE_CONF_PSI_PUNISHMENT_FACTOR
#else
#define TREE_PSI_PUNISHMENT_FACTOR 0.3
#endif /*TREE_CONF_PSI_PUNISHMENT_FACTOR*/

/* I.Q. Evaporation period */
#ifdef TREE_CONF_EVAP_PERIOD
#define TREE_EVAP_PERIOD TREE_CONF_EVAP_PERIOD
#else
#define TREE_EVAP_PERIOD 0.5*CLOCK_SECOND
#endif /*TREE_CONF_EVAP_PERIOD*/

/* I.Q. Evaporation coefficient */
#ifdef TREE_CONF_EVAP_COEF
#define TREE_EVAP_COEF TREE_CONF_EVAP_COEF
#else
#define TREE_EVAP_COEF 0.01
#endif /*TREE_CONF_EVAP_COEF*/

/*********** Configuration related to TREE algorithm **********/

/* Number of cells to allocate by coordination message */
#ifdef TREE_CONF_CELLS_PER_COORD
#define TREE_CELLS_PER_COORD TREE_CONF_CELLS_PER_COORD
#else
#define TREE_CELLS_PER_COORD 5
#endif /*TREE_CONF_CELLS_PER_COORD*/


/* Priority for control messages */
#ifdef TREE_CONF_WITH_PRIORITY_CONTROL
#define TREE_WITH_PRIORITY_CONTROL TREE_CONF_WITH_PRIORITY_CONTROL
#else
#define TREE_WITH_PRIORITY_CONTROL 1
#endif /*TREE_CONF_WITH_PRIORITY_CONTROL*/

/* EB based on parent*/
#ifdef TREE_CONF_EB_ON_RPL_PARENT
#define TREE_EB_ON_RPL_PARENT TREE_CONF_EB_ON_RPL_PARENT
#else
#define TREE_EB_ON_RPL_PARENT TREE_WITH_RPL
#endif /*EB_ON_RPL_PARENT_CONF*/

/* Includes basic cells with 1 TX per neighbor and 1 RX per node
 * Useful in dense environments*/
#ifdef TREE_CONF_BASIC_CELL
#define TREE_BASIC_CELL TREE_CONF_BASIC_CELL
#else
#define TREE_BASIC_CELL 1
#endif /*TREE_CONF_BASIC_CELL*/

/* Slotframe EB size */
#ifdef TREE_CONF_SF_EB_SIZE
#define TREE_SF_EB_SIZE TREE_CONF_SF_EB_SIZE
#else
#define TREE_SF_EB_SIZE 157
#endif /*TREE_CONF_SF_EB_SIZE*/

/* Main slotframe size 16*13
Multiple of 16 so that it learns to play with channel hopping
Same size as other slotframe to learn other also
*/
#ifdef TREE_CONF_SF_SIZE
#define TREE_SF_SIZE TREE_CONF_SF_SIZE
#else
#define TREE_SF_SIZE 208
#endif /*TREE_CONF_SF_SIZE*/

/* Broadcast slotframe size */
#ifdef TREE_CONF_SF_BC_SIZE
#define TREE_SF_BC_SIZE TREE_CONF_SF_BC_SIZE
#else
#define TREE_SF_BC_SIZE 157
#endif /*TREE_CONF_SF_BC_SIZE*/

/* Broadcast slotframe size */
#ifdef TREE_CONF_SF_BC_HANDLE
#define TREE_SF_BC_HANDLE TREE_CONF_SF_BC_HANDLE
#else
#define TREE_SF_BC_HANDLE 1
#endif /*TREE_CONF_SF_BC_HANDLE*/

/* Main slotframe handle (defines priority with EB) */
#ifdef TREE_CONF_SF_HANDLE
#define TREE_SF_HANDLE TREE_CONF_SF_HANDLE
#else
#define TREE_SF_HANDLE 2
#endif /*TREE_CONF_SF_HANDLE*/

/* EB slotframe handle (defines priority with main slotframe) */
#ifdef TREE_CONF_SF_EB_HANDLE
#define TREE_SF_EB_HANDLE TREE_CONF_SF_EB_HANDLE
#else
#define TREE_SF_EB_HANDLE 0
#endif /*TREE_CONF_SF_EB_HANDLE*/

/* The hash function used to assign initial timeslots to a given node
* (based on its link-layer address) */
#ifdef TREE_CONF_HASH
#define TREE_HASH                  TREE_CONF_HASH
#else
#define TREE_HASH(addr)            (((addr)->u8[7]+(addr)->u8[6])+1)
#endif /* TREE_CONF_HASH */


#endif /*__TREE_CONF_H__*/
