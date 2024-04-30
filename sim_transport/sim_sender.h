/*-
* Copyright (c) 2017-2018 Razor, Inc.
*	All rights reserved.
*
* See the file LICENSE for redistribution information.
*/

#ifndef __SIM_SENDER_H__
#define __SIM_SENDER_H__

#include "cf_skiplist.h"
#include "sim_proto.h"
#include "razor_callback.h"
#include "flex_fec_sender.h"

struct __sim_session;
typedef struct __sim_session sim_session_t;

typedef struct __sim_sender
{
	int                actived;
	uint32_t           base_packet_id;			/*receiver received max continuous packets id*/

	uint32_t           packet_id_seed;			/* meta segment packet id begin from 1*/
	uint32_t           send_id_seed;			/*send packets id used for cc*/
	uint32_t           frame_id_seed;    // send frame id auto increase, begin from 1
	uint32_t           transport_seq_seed;

	int64_t            first_ts;			/* first frame timestamp */

	skiplist_t*        segs_cache;  /* meta segment list */
	skiplist_t*        fecs_cache;  /* fec segment list for pace sender */
	skiplist_t*        ack_cache;   /* sent segments need ack list */

	razor_sender_t*    cc;					/* congestion controller */

	sim_session_t*     s;

	flex_fec_sender_t* flex;
	base_list_t*       out_fecs;  /* fec packets generating buffer */

	size_t             splits_size;
	uint16_t*          splits;
} sim_sender_t;

sim_sender_t* sim_sender_create(sim_session_t* s, int transport_type, int padding, int fec);
void sim_sender_destroy(sim_session_t* s, sim_sender_t* sender);
void sim_sender_reset(sim_session_t* s, sim_sender_t* sender, int transport_type, int padding, int fec);
int sim_sender_active(sim_session_t* s, sim_sender_t* sender);

int sim_sender_put(sim_session_t* s, sim_sender_t* sender, uint8_t payload_type, uint8_t ftype, const uint8_t* data, size_t size);
int sim_sender_ack(sim_session_t* s, sim_sender_t* sender, sim_segment_ack_t* ack);
void sim_sender_timer(sim_session_t* s, sim_sender_t* sender, uint64_t now_ts);
void sim_sender_update_rtt(sim_session_t* s, sim_sender_t* r);
void sim_sender_feedback(sim_session_t* s, sim_sender_t* sender, sim_feedback_t* feedback);
void sim_sender_set_bitrates(sim_session_t* s, sim_sender_t* sender, uint32_t min_bitrate, uint32_t start_bitrate, uint32_t max_bitrate);
void sim_clean_ack_cache(sim_session_t* s, sim_sender_t* sender);

#endif //!__SIM_SENDER_H__
