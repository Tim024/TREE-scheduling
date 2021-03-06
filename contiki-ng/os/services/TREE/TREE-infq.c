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
 *         TREE scheduling algorithm information quantities
 *
 * \author Tim van der Lee <t.lee@tue.nl>
 */

#include "TREE-infq.h"

/* Log configuration */
#include "sys/log.h"
#define LOG_MODULE "TREE-INFQ"
#define LOG_LEVEL LOG_LEVEL_DBG

/*---------------------------------------------------------------------------*/
uint32_t
update_psi(uint32_t old_value)
{
  return old_value + TREE_PSI_INCR_VALUE;
}
/*---------------------------------------------------------------------------*/
uint32_t
update_psi_err(uint32_t old_value)
{
  return old_value*TREE_PSI_PUNISHMENT_FACTOR;
}
/*---------------------------------------------------------------------------*/
uint32_t
update_phi(uint32_t old_value, int qsize, int num_links)
{
  //LOG_DBG("qs %d nl %d\n",qsize-1,num_links+1);
  int broadcast_cells = 1;
#if TREE_BASIC_CELL
  broadcast_cells += 1;
#endif
  return old_value + TREE_PHI_INCR_VALUE*((qsize-1)*QFACTOR + 1)/(num_links+broadcast_cells);
}
/*---------------------------------------------------------------------------*/
uint32_t
evaporate(uint32_t old_value)
{
  uint32_t res = old_value*(1-TREE_EVAP_COEF);
  return res;
}
/*---------------------------------------------------------------------------*/
int
check_psi(uint32_t value)
{
  return value > TREE_PSI_MIN;
}
/*---------------------------------------------------------------------------*/
int
check_phi(uint32_t value)
{
  return value < TREE_PHI_MAX;
}
