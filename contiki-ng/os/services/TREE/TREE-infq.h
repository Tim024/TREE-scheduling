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


#include "TREE-conf.h"
#include "lib/random.h"
#include "net/mac/tsch/tsch.h"
#include "contiki.h"


/* Reset values for PSI and PHI */
#define TREE_PHI_RESET TREE_PHI_MAX/2
#define TREE_PSI_RESET 5000
//#define QFACTOR 0.3 /* In the paper, this should be equal to f_q \times k */
#define QFACTOR 0.3 /* In the paper, this should be equal to f_q \times k */


uint32_t update_psi(uint32_t old_value);
uint32_t update_psi_err(uint32_t old_value);
uint32_t update_phi(uint32_t old_value, int qsize, int num_links);
uint32_t evaporate(uint32_t value);
int check_psi(uint32_t value);
int check_phi(uint32_t value);
