/*-
* Copyright (c) 2017-2019 Razor, Inc.
*	All rights reserved.
*
* See the file LICENSE for redistribution information.
*/

#ifndef __SIM_FEC_H__
#define __SIM_FEC_H__

#include "sim_proto.h"
#include "cf_skiplist.h"
#include "cf_list.h"

struct __sim_session;
typedef struct __sim_session sim_session_t;

typedef struct
{
	uint32_t			base_id;
	uint32_t			max_ts;

	int64_t				evict_ts;

	skiplist_t*			segs_cache;
	skiplist_t*			flexes;
	skiplist_t*			recover_packets;
	base_list_t*		out;
} sim_receiver_fec_t;

sim_receiver_fec_t*		sim_fec_create(sim_session_t* s);
void sim_fec_destroy(sim_session_t* s, sim_receiver_fec_t* f);
void sim_fec_reset(sim_session_t* s, sim_receiver_fec_t* f);
void sim_fec_active(sim_session_t* s, sim_receiver_fec_t* f);
void sim_fec_put_fec_packet(sim_session_t* s, sim_receiver_fec_t* f, sim_fec_t* fec);
void sim_fec_put_segment(sim_session_t* s, sim_receiver_fec_t* f, sim_segment_t* seg);
void sim_fec_evict(sim_session_t* s, sim_receiver_fec_t* f, int64_t now_ts);

#endif // !__SIM_FEC_H__
