/*
* Copyright (c) 2017-2018 Razor, Inc.
*	All rights reserved.
*
* See the file LICENSE for redistribution information.
*/

#ifndef __SIM_RECEIVER_H__
#define __SIM_RECEIVER_H__

#include <stdint.h>
#include "cf_skiplist.h"
#include "sim_proto.h"
#include "sim_fec.h"

class VCMJitterBuffer;

struct __razor_receiver;
typedef struct __razor_receiver razor_receiver_t;

struct __sim_session;
typedef struct __sim_session sim_session_t;

typedef struct
{
	uint32_t			fid;  // frame id
	uint32_t			last_seq;
	uint32_t			ts;  // frame send ts
	int					frame_type;  // I P

	int					seg_count;   // received segs
	int					seg_number;  // total segment count of one frame
	sim_segment_t**		segments;
}sim_frame_t;

typedef struct __sim_frame_cache
{
	uint32_t			size;
	uint32_t			min_seq;  /* min sequeceid(packet id) */
	uint32_t			min_fid;  /* min frame id(excluse self, begin from 0) */
	uint32_t			max_fid;  /* max frame id(begin from 1) */
	uint32_t			play_frame_ts; // last playing buffer cleaned ts 
	uint32_t			max_ts;  // max segment timestamp

	uint32_t			frame_ts;		/* playing ts, update ervery 5ms */
	uint64_t			play_ts;		/* The timestamp of the current system clock.*/

	uint32_t			frame_timer;	/* The interval between frames*/
	uint32_t			wait_timer;		/* jitter length，unit ms*/

	int					state;
	int					loss_flag;

	float				f;  // play speed factor 1.0

	skiplist_t*			discard_loss;

	sim_frame_t*		frames; //ring buffer for frame buffer, size = CACHE_SIZE
}sim_frame_cache_t;

typedef struct __sim_receiver
{
	uint32_t			base_uid;
	uint32_t			base_seq;
	uint32_t			max_seq;
	uint32_t			max_ts;        // max received packet ts

	skiplist_t*			loss;
	int					loss_count;				/*The frequency of packet loss within a unit of time.*/

	sim_frame_cache_t*	cache;

	uint64_t			ack_ts;        // last ack GET_SYS_MS
	uint64_t			cache_ts;      // jitter calculate timer
	uint64_t			active_ts;

	uint8_t				acked_count;
	uint32_t			ackeds[ACK_NUM];

	uint64_t			fir_ts;

	int					actived;

	/*about FIR*/
	uint32_t			fir_seq;			/*The request for a keyframe message seq*/
	int					fir_state;			

	int					cc_type;

	razor_receiver_t*	cc;
	sim_session_t*		s;

	sim_receiver_fec_t* recover;

	VCMJitterBuffer* jitter;
} sim_receiver_t;

sim_receiver_t* sim_receiver_create(sim_session_t* s, int transport_type);
void sim_receiver_destroy(sim_session_t* s, sim_receiver_t* r);
void sim_receiver_reset(sim_session_t* s, sim_receiver_t* r, int transport_type);
int sim_receiver_active(sim_session_t* s, sim_receiver_t* r, uint32_t uid);
int sim_receiver_put(sim_session_t* s, sim_receiver_t* r, sim_segment_t* seg);
int sim_receiver_put_fec(sim_session_t* s, sim_receiver_t* r, sim_fec_t* fec);
int sim_receiver_padding(sim_session_t* s, sim_receiver_t* r, uint16_t transport_seq, uint32_t send_ts, size_t data_size);
int sim_receiver_get(sim_session_t* s, sim_receiver_t* r, uint8_t* data, size_t* sizep, uint8_t* payload_type);
void sim_receiver_timer(sim_session_t* s, sim_receiver_t* r, int64_t now_ts);
void sim_receiver_update_rtt(sim_session_t* s, sim_receiver_t* r);
uint32_t sim_receiver_cache_delay(sim_session_t* s, sim_receiver_t* r);

#endif //!__SIM_RECEIVER_H__
