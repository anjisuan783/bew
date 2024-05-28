/*-
* Copyright (c) 2017-2018 Razor, Inc.
*	All rights reserved.
*
* See the file LICENSE for redistribution information.
*/

#include "sim_receiver.h"

#include <algorithm>
#include <limits>
#include <assert.h>

#include "webrtc/base/mod_ops.h"
#include "sim_session.h"
#include "jitter_buffer.h"
#include "call_stats.h"

//#define DISABLE_JITTER

#define USE_JITTER_ESTMITOR

enum {
	buffer_waiting = 0,  // init state
	buffer_playing
};

enum {
	fir_normal,
	fir_flightting,
};

#define CACHE_SIZE 1024
#define INDEX(i)	((i) % CACHE_SIZE)
#define MAX_EVICT_DELAY_MS 6000
#define MIN_EVICT_DELAY_MS 3000

const uint32_t kMaxJitterDelayNoLost = 10;

const int kMaxNackRetries = 10;

namespace webrtc {
template <typename T, T M>
inline bool AheadOrAt(T a, T b) {
  static_assert(std::is_unsigned<T>::value,
                "Type must be an unsigned integer.");
  const T maxDist = M / 2;
  if (!(M & 1) && MinDiff<T, M>(a, b) == maxDist)
    return b < a;
  return ForwardDiff<T, M>(b, a) <= maxDist;
}

template <typename T>
inline bool AheadOrAt(T a, T b) {
  static_assert(std::is_unsigned<T>::value,
                "Type must be an unsigned integer.");
  const T maxDist = std::numeric_limits<T>::max() / 2 + T(1);
  if (a - b == maxDist)
    return b < a;
  return ForwardDiff(b, a) < maxDist;
}

// Test if the sequence number |a| is ahead of sequence number |b|. (a newer than b)
//
// If |M| is an even number and the two sequence numbers are at max distance
// from each other, then the sequence number with the highest value is
// considered to be ahead.
template <typename T, T M>
inline bool AheadOf(T a, T b) {
  static_assert(std::is_unsigned<T>::value,
                "Type must be an unsigned integer.");
  return a != b && AheadOrAt<T, M>(a, b);
}

template <typename T>
inline bool AheadOf(T a, T b) {
  static_assert(std::is_unsigned<T>::value,
                "Type must be an unsigned integer.");
  return a != b && AheadOrAt(a, b);
}
}

static void sim_receiver_send_fir(sim_session_t* s, sim_receiver_t* r);

/*************************************play buffer ******************************************************/
static sim_frame_cache_t* open_real_video_cache(sim_session_t* s)
{
	sim_frame_cache_t* c = (sim_frame_cache_t*)calloc(1, sizeof(sim_frame_cache_t));
	c->wait_timer = 50;
	c->state = buffer_waiting;
	c->min_seq = 0;
	c->frame_timer = 100;
	c->f = 1.0f;
	c->size = CACHE_SIZE;
	c->frames = (sim_frame_t*)calloc(CACHE_SIZE, sizeof(sim_frame_t));

	c->discard_loss = skiplist_create(idu32_compare, NULL, NULL);
	return c;
}

static inline void real_video_clean_frame(sim_session_t* session, sim_frame_cache_t* c, sim_frame_t* frame)
{
	skiplist_iter_t* iter;

	int i;
	if (frame->seg_number == 0)
		return;

	for (i = 0; i < frame->seg_number; ++i){
		if (frame->segments[i] != NULL){
			c->min_seq = frame->segments[i]->packet_id - frame->segments[i]->index + frame->segments[i]->total - 1;
			free(frame->segments[i]);
			frame->segments[i] = NULL;
		}
	}

	free(frame->segments);

	c->play_frame_ts = frame->ts;
	frame->ts = 0;
	frame->frame_type = 0;
	frame->seg_number = 0;
	frame->seg_count = 0;
	frame->nack_count = 0;
	frame->size = 0;

	c->min_fid = frame->fid;

	while (skiplist_size(c->discard_loss) > 0){
		iter = skiplist_first(c->discard_loss);
		if (iter->key.u32 <= c->min_seq)
			skiplist_remove(c->discard_loss, iter->key);
		else
			break;
	}
}

static int evict_gop_frame(sim_session_t* s, sim_frame_cache_t* c)
{
	uint32_t i;
	sim_frame_t* frame = NULL;
	
	uint32_t key_frame_id = 0;
	// look for key frame, from the sencod, first one must be evict
	for (i = c->min_fid + 2; i < c->max_fid; ++i){
		frame = &c->frames[INDEX(i)];
		if (frame->frame_type == 1){
			key_frame_id = i;
			break;
		}
	}

	if (key_frame_id == 0)
		return -1;

	sim_debug("evict_gop_frame, from frame = %u, to frame = %u!! \n", c->min_fid + 1, key_frame_id);
	for (i = c->min_fid + 1; i < key_frame_id; i++){
		c->frame_ts = c->frames[INDEX(i)].ts;
		real_video_clean_frame(s, c, &c->frames[INDEX(i)]);
	}

	c->min_fid = key_frame_id - 1;

	return 0;
}

static inline int real_video_cache_check_frame_full(sim_frame_t* frame)
{
	return (frame->seg_number == frame->seg_count) ? 0 : -1;
}

static void real_video_cache_evict_discard(sim_session_t* s, sim_frame_cache_t* c)
{
	uint32_t pos, missing_seq, i, j;
	sim_frame_t* frame;
	skiplist_item_t key;

	if (c->min_fid == c->max_fid)
		return;

	missing_seq = 0;

	pos = INDEX(c->min_fid + 1);
	frame = &c->frames[pos];
	if ((c->min_fid + 1 == frame->fid || frame->frame_type == 1) && real_video_cache_check_frame_full(frame) == 0)
		return;

	for (i = c->min_fid + 1; i <= c->max_fid; ++i){
		frame = &c->frames[INDEX(i)];
		if (frame->seg_number <= 0)
			continue;

		for (j = 0; j < frame->seg_number; ++j){
			if (frame->segments[j] != NULL){
				missing_seq = frame->segments[j]->packet_id + frame->segments[j]->total - frame->segments[j]->index;
				goto evict_lab;
			}
		}
	}

evict_lab:
	for (i = c->min_seq + 1; i <= missing_seq; ++i){
		key.u32 = i;
		if (skiplist_search(c->discard_loss, key) != NULL){
			if(evict_gop_frame(s, c) != 0)
				sim_receiver_send_fir(s, s->receiver);
			break;
		}
	}
}

static void real_video_cache_discard(sim_session_t* s, sim_frame_cache_t* c, uint32_t seq)
{
	skiplist_item_t key, val;

	if (seq <= c->min_seq)
		return;

	key.u32 = val.u32 = seq;
	skiplist_insert(c->discard_loss, key, val);

	real_video_cache_evict_discard(s, c);
}

static void close_real_video_cache(sim_session_t* s, sim_frame_cache_t* cache)
{
	uint32_t i;
	for (i = 0; i < cache->size; ++i)
		real_video_clean_frame(s, cache, &cache->frames[i]);

	skiplist_destroy(cache->discard_loss);

	free(cache->frames);
	free(cache);
}

static void reset_real_video_cache(sim_session_t* s, sim_frame_cache_t* c)
{
	uint32_t i;

	for (i = 0; i < c->size; ++i)
		real_video_clean_frame(s, c, &c->frames[i]);

	c->min_seq = 0;
	c->min_fid = 0;
	c->max_fid = 0;
	c->play_ts = 0;
	c->frame_ts = 0;
	c->max_ts = 100;
	c->frame_timer = 100;
	c->f = 1.0f;

	c->state = buffer_waiting;
	c->wait_timer = SU_MAX(50, s->stats->max_rtt_ms());

	skiplist_clear(c->discard_loss);
}

static void real_video_evict_frame(sim_session_t* s, sim_frame_cache_t* c, uint32_t fid)
{
	uint32_t pos, i;

	for (pos = c->max_fid + 1; pos <= fid; pos++)
		real_video_clean_frame(s, c, &c->frames[INDEX(pos)]);

	if (fid < c->min_fid + c->size)
		return;

	for (pos = c->min_fid + 1; pos < c->max_fid; ++pos){
		if (c->frames[INDEX(pos)].frame_type == 1)
			break;
	}

	for (i = c->min_fid + 1; i < pos; ++i)
		real_video_clean_frame(s, c, &c->frames[INDEX(i)]);
}

static int real_video_cache_put(sim_session_t* s, sim_frame_cache_t* c, sim_segment_t* seg)
{
	sim_frame_t* frame;
	int ret = -1;
	int i;
	bool duplicate = false;

	// valid check
	if (seg->index >= seg->total || seg->fid <= c->min_fid){
		return ret;
	}

	// here comes a new frame.  TODO rewind case?
	if (seg->fid > c->max_fid){
		if (c->max_fid > 0) {
			// clean the old slots
			real_video_evict_frame(s, c, seg->fid);
		} else if (c->min_fid == 0 && c->max_fid == 0) {
			// initial state
			c->min_fid = seg->fid - 1;
			c->play_frame_ts = seg->timestamp;
		}

		// calcute frame interval, limited 16 <= frame_timer << 200
		int ts_diff = seg->timestamp - c->max_ts;
		int frame_diff = seg->fid - c->max_fid;
		if (c->max_fid >= 0 && frame_diff > 0 && ts_diff > 0){
			c->frame_timer = ts_diff / frame_diff;
			c->frame_timer = SU_MAX(10, SU_MIN(200, c->frame_timer));
		}
		c->max_ts = seg->timestamp;
		c->max_fid = seg->fid;
	}

	frame = &(c->frames[INDEX(seg->fid)]);
	frame->fid = seg->fid;
	frame->frame_type = seg->ftype;
	frame->ts = seg->timestamp;

	if (frame->seg_number == 0){ // first seg of the frame
		frame->seg_count = 1;
		frame->seg_number = seg->total;
		frame->segments = (sim_segment_t**)calloc(frame->seg_number, sizeof(seg));
		frame->segments[seg->index] = seg;
		frame->nack_count = 0;
		frame->size = seg->data_size;
		ret = 0;
	} else { // next seg of the frame
		if (frame->segments[seg->index] == NULL){
			frame->segments[seg->index] = seg;
			frame->seg_count++;
			frame->size += seg->data_size;
			ret = 0;
		} else { // else duplicate segment will be free
			duplicate = true;
		}
	}
#ifdef USE_JITTER_ESTMITOR
	if (!duplicate && 0 == real_video_cache_check_frame_full(frame) && frame->nack_count <= 0) {
		// update jitter
		int64_t now = GET_SYS_MS();
		s->receiver->jitter->UpdateJitterEstimate(now, frame->ts * 90, frame->size);
		uint32_t f_delay = s->receiver->jitter->EstimatedJitterMs();
		s->receiver->cache->wait_timer = f_delay;
		sim_debug("real_video_cache_put fid=%u f_delay=%u fsize=%u\n", frame->fid, f_delay, frame->size);
	}
#endif
	if (!duplicate && 0 == real_video_cache_check_frame_full(frame)) {
		sim_debug("real_video_cache_put fid=%u fsize=%u\n", frame->fid, frame->size);
	}
	sim_debug("buffer put video frame, frame=%u, packet_id=%u\n", seg->fid, seg->packet_id);
	return ret;
}

static void real_video_cache_check_playing(sim_session_t* s, sim_frame_cache_t* c)
{
	if (c->max_fid <= c->min_fid)
		return ;
		
	uint32_t space = SU_MAX(c->wait_timer, c->frame_timer);
	if (c->max_ts >= c->play_frame_ts + space && c->max_fid >= c->min_fid + 1){
		c->state = buffer_playing;

		c->play_ts = GET_SYS_MS();
		c->frame_ts = c->max_ts - c->frame_timer * (c->max_fid - c->min_fid - 1);
		/*sim_debug("buffer playing, frame ts = %u, min_ts = %u, c->frame_timer = %u, max ts = %u\n", c->frame_ts, min_ts, c->frame_timer, max_ts);*/
	}
}

static inline void real_video_cache_sync_timestamp(sim_session_t* s, sim_frame_cache_t* c)
{
	uint64_t cur_ts = GET_SYS_MS();
	uint64_t diff = cur_ts - c->play_ts;
	if (diff >= 5) { // update playing ts
		c->frame_ts += (uint32_t)(diff * c->f);
		c->play_ts = cur_ts;
	}
}

static uint32_t real_video_ready_ms(sim_session_t* s, sim_frame_cache_t* c)
{
	sim_frame_t* frame;
	uint32_t i, min_ready_ts, max_ready_ts, ret, count;
	count = 0;
	ret = c->frame_timer;
	min_ready_ts = 0; // first frame ts
	max_ready_ts = 0; // last frame ts
	for (i = c->min_fid + 1; i <= c->max_fid; ++i){
		frame = &c->frames[INDEX(i)];
		if (0 == real_video_cache_check_frame_full(frame)){
			if (min_ready_ts == 0)
				min_ready_ts = frame->ts;
			max_ready_ts = frame->ts;
			++count;
		} else
			break;
	}

	if (min_ready_ts > 0)
		ret = max_ready_ts - min_ready_ts + c->frame_timer;

	if (count > 1) {
		//sim_debug("real_video_ready_ms more than one frame ready=%u ts=%ums\n", count, ret);
	}

	return ret;
}

/*fir request*/
static int real_video_check_fir(sim_session_t* s, sim_frame_cache_t* c)
{
	uint32_t pos;
	sim_frame_t* frame;

	pos = INDEX(c->min_fid + 1);
	frame = &c->frames[pos];

	if (real_video_cache_check_frame_full(frame) == 0)
		return -1;

	if (c->play_frame_ts + MAX_EVICT_DELAY_MS * 3 / 5 > c->max_ts)
		return -1;

	return 0;
}

static int real_video_cache_get(sim_session_t* s, sim_frame_cache_t* c, uint8_t* data, size_t* sizep, uint8_t* payload_type, int loss)
{
	uint32_t space;
	size_t size, buffer_size;
	int ret, i;
	sim_frame_t* frame;
	uint32_t play_ready_ts;
	uint8_t type;

	buffer_size = *sizep;
	*sizep = 0;
	*payload_type = 0;

	type = 0;
	size = 0;
	ret = -1;

	if (c->state == buffer_waiting) {
		real_video_cache_check_playing(s, c);
	}

	if (c->state != buffer_playing)
		goto err;

	space = SU_MAX(c->wait_timer, c->frame_timer);

	// first_ts - last_ts + interval
	play_ready_ts = real_video_ready_ms(s, c);

	c->f = 1.0f;
	if (loss != 0 && play_ready_ts < SU_MIN(space, 4 * c->frame_timer))
		c->f = 0.6f;
	else if (play_ready_ts > c->frame_timer * 4 && play_ready_ts > MIN_EVICT_DELAY_MS / 2)
		c->f = 3.0f;
	else if (play_ready_ts >= space && play_ready_ts >= SU_MAX(kMaxJitterDelayNoLost, c->frame_timer))
		c->f = 1.2f;

	real_video_cache_sync_timestamp(s, c);

	if (c->min_fid == c->max_fid)
		goto err;

	// 4xjitter len < clean time < 3s
	if (c->play_frame_ts + SU_MAX(MIN_EVICT_DELAY_MS, SU_MIN(MAX_EVICT_DELAY_MS, 4 * c->wait_timer)) < c->max_ts){
		evict_gop_frame(s, c);
	}

	//real_video_cache_evict_discard(s, c);
	frame = &c->frames[INDEX(c->min_fid + 1)];
	if ((c->min_fid + 1 == frame->fid || frame->frame_type == 1) && real_video_cache_check_frame_full(frame) == 0){
#ifndef DISABLE_JITTER	
		if (play_ready_ts > space && frame->ts > c->frame_ts + SU_MAX(500, 2 * space)) {
			// faster play ts
			c->frame_ts = frame->ts - space;
		} else if (frame->ts <= c->frame_ts) {
#endif
			for (i = 0; i < frame->seg_number; ++i){
				if (size + frame->segments[i]->data_size <= buffer_size && frame->segments[i]->data_size <= SIM_VIDEO_SIZE){
					memcpy(data + size, frame->segments[i]->data, frame->segments[i]->data_size);
					size += frame->segments[i]->data_size;
					type = frame->segments[i]->payload_type;
				} else{
					size = 0;
					break;
				}
			}
			c->frame_ts = frame->ts;
			real_video_clean_frame(s, c, frame);
			ret = 0;
			//sim_debug("real_video_cache_get fid=%u size=%u \n", frame->fid, size);
#ifndef DISABLE_JITTER
		}
#endif
	} else {
		size = 0;
		ret = -1;
	}

err:
	*sizep = size;
	*payload_type = type;

	return ret;
}

inline static uint32_t real_video_cache_delay(sim_session_t* s, sim_frame_cache_t* c)
{
	if (c->max_fid <= c->min_fid)
		return 0;

	return c->max_ts - c->play_frame_ts;
}

/*********************************************video reciever processing***************************************/
typedef struct
{
	int64_t				loss_ts;  // lost time now
	int64_t				ts;				// nack time
	int					  count;      // nack count
	uint32_t      fid;
}sim_loss_t;

static void loss_free(skiplist_item_t key, skiplist_item_t val, void* args)
{
	if (val.ptr != NULL)
		free(val.ptr);
}

static void send_sim_feedback(void* handler, const uint8_t* payload, int payload_size)
{
	sim_header_t header;
	sim_feedback_t feedback;
	sim_receiver_t* r = (sim_receiver_t*)handler;
	sim_session_t* s = r->s;

	if (payload_size > SIM_FEEDBACK_SIZE){
		/*sim_error("feedback size > SIM_FEEDBACK_SIZE\n");*/
		return;
	}

	INIT_SIM_HEADER(header, SIM_FEEDBACK, s->uid);

	feedback.base_packet_id = r->base_seq;

	feedback.feedback_size = payload_size;
	memcpy(feedback.feedback, payload, payload_size);

	sim_encode_msg(&s->sstrm, &header, &feedback);
	sim_session_network_send(s, &s->sstrm);

	/*sim_debug("sim send SIM_FEEDBACK, feedback size = %u\n", payload_size);*/
}

#define FIR_DELAY_TIME 2000
static void sim_receiver_send_fir(sim_session_t* s, sim_receiver_t* r)
{
	sim_fir_t fir;
	sim_header_t header;

	int64_t cur_ts = GET_SYS_MS();
	if (r->fir_ts + FIR_DELAY_TIME < cur_ts){
		INIT_SIM_HEADER(header, SIM_FIR, s->uid);
		fir.fir_seq = (r->fir_state == fir_flightting) ? r->fir_seq : (++r->fir_seq);

		sim_encode_msg(&s->sstrm, &header, &fir);
		sim_session_network_send(s, &s->sstrm);

		r->fir_ts = cur_ts;
	}
}
#define NACK_MAX_TICK 3000
static void sim_receiver_update_loss(sim_session_t* s, sim_receiver_t* r, sim_segment_t* seg)
{
	uint32_t i, space;
	skiplist_item_t key, val;
	skiplist_iter_t* iter;
	int64_t now_ts;

	uint32_t seq = seg->packet_id;
	uint32_t fid = seg->fid;
	key.u32 = seq;
	skiplist_remove(r->loss, key);

	now_ts = GET_SYS_MS();
	if (r->max_ts + NACK_MAX_TICK < seg->timestamp || s->stats->avg_rtt_ms() >= CACHE_MAX_DELAY){
		// 3s no segment received, or rtt > CACHE_MAX_DELAY use fir instead of nack
		skiplist_clear(r->loss);
		sim_receiver_send_fir(s, r);
		return ;
	}

	// insert nack list
	for (i = r->max_seq + 1; i < seq; ++i){
		key.u32 = i;
		iter = skiplist_search(r->loss, key);
		if (iter == NULL) { 
			sim_loss_t* l = (sim_loss_t*)calloc(1, sizeof(sim_loss_t));
			l->ts = -1;
			l->loss_ts = now_ts;
			l->count = 0;
			l->fid = fid;
			val.ptr = l;

			skiplist_insert(r->loss, key, val);
		}
		/*sim_debug("add loss, seq = %u\n", key.u32);*/
	}
}

static inline void sim_receiver_send_ack(sim_session_t* s, sim_receiver_t* r, sim_segment_ack_t* ack)
{
	uint32_t count, i;
	sim_header_t header;
	INIT_SIM_HEADER(header, SIM_SEG_ACK, s->uid);

	ack->ack_num = 0;

	// nack hole existed
	if (r->base_seq < r->max_seq){
		// calculate ack list
		count = r->acked_count >= ACK_NUM ? ACK_NUM : r->acked_count;
		for (i = 0; i < count; ++i){
			if (r->base_seq < r->ackeds[i])
				ack->acked[ack->ack_num++] = (uint16_t)(r->ackeds[i] - r->base_seq);
		}
	}

	r->acked_count = 0;

	sim_encode_msg(&s->sstrm, &header, ack);
	sim_session_network_send(s, &s->sstrm);
}

static void video_real_update_base(sim_session_t* s, sim_receiver_t* r)
{
	skiplist_iter_t* iter;
	skiplist_item_t key;
	uint32_t min_seq;

	min_seq = r->cache->min_seq;
	for (key.u32 = r->base_seq + 1; key.u32 <= min_seq; ++key.u32)
		skiplist_remove(r->loss, key);

	if (skiplist_size(r->loss) == 0) {
		r->base_seq = r->max_seq;
	} else {
		iter = skiplist_first(r->loss);
		r->base_seq = (iter->key.u32 > 0) ? (iter->key.u32 - 1) : r->base_seq;
	}
}

#define ACK_REAL_TIME	20
#define ACK_HB_TIME		200
static void video_real_ack(sim_session_t* s, sim_receiver_t* r, int heartbeat, uint32_t seq)
{
	uint64_t cur_ts;
	sim_segment_ack_t ack;
	skiplist_iter_t* iter;
	skiplist_item_t key;
	sim_loss_t* l;
	uint32_t delay, space_factor;
	skiplist_t* outofdate_nacklist = nullptr;
	int max_count = 0;

	uint32_t numbers[NACK_NUM];
	int i, evict_count = 0;
	
	cur_ts = GET_SYS_MS();
	
	bool sequence_based = heartbeat == 0;
	// heartbeat trigger ack every 200ms
	// packet arriving trigger ack every 20ms
	space_factor = s->stats->max_rtt_ms();
	if (sequence_based || (r->ack_ts + space_factor < cur_ts)){		
		ack.acked_packet_id = seq; // calculate rtt using ack on fec mode
		video_real_update_base(s, r);

		ack.base_packet_id = r->base_seq;
		ack.nack_num = 0;
		if (r->loss->size > 1000)
			sim_info("video_real_ack, lost size=%u \n", r->loss->size);
			
		SKIPLIST_FOREACH(r->loss, iter) {
			// out of date nack
			if (iter->key.u32 <= r->base_seq)
				continue;

			l = (sim_loss_t*)iter->val.ptr;

			if ((sequence_based && -1 == l->ts && webrtc::AheadOf(r->max_seq, iter->key.u32)) ||
					(/*time_based && -1!= l->ts && */l->ts + space_factor < cur_ts)) {
				l->count++;
				if (l->count >= kMaxNackRetries) {
					if (!outofdate_nacklist) {
						outofdate_nacklist = skiplist_create(idu32_compare, NULL, NULL);
					}
					key.u32 = iter->key.u32;
					skiplist_item_t v;
					v.u32 = iter->key.u32;
					skiplist_insert(outofdate_nacklist, key, v);
					continue;
				}
				ack.nack[ack.nack_num++] = iter->key.u32 - r->base_seq;
				l->ts = cur_ts;
				r->loss_count++;

				// update frame nack count
				sim_frame_t* frame = &(r->cache->frames[INDEX(l->fid)]);
				if (frame->seg_count > 0) {
					frame->nack_count++;
				}
			}

			if (l->count > max_count)
				max_count = l->count;

			if (ack.nack_num >= NACK_NUM) {
				sim_info("video_real_ack, ack.nack_num >= NACK_NUM \n");
				break;
			}
		}

		if (s->stats->avg_rtt_ms() >= CACHE_MAX_DELAY && ack.nack_num > 0){
			skiplist_clear(r->loss);
			sim_receiver_send_fir(s, r);

			ack.nack_num = 0;
		}

		if (outofdate_nacklist) {
			if (skiplist_size(r->loss) > 0) {
				SKIPLIST_FOREACH(outofdate_nacklist, iter) {
					skiplist_remove(r->loss, iter->key);
				}
			}
			skiplist_destroy(outofdate_nacklist);
		}

		if (ack.nack_num) {
			std::ostringstream os;
			os <<(sequence_based? "sequence_based":"time_based");
			os << "nack num=" <<  (int)ack.nack_num << " [";
			for (uint8_t i = 0; i < ack.nack_num; ++i) {
				os << ack.base_packet_id + ack.nack[i] << ",";
			}
			os << "]";
			sim_debug("video_real_ack, %s \n", os.str().c_str());
		}
		
		sim_receiver_send_ack(s, r, &ack);
	
		r->ack_ts = cur_ts;
#ifndef USE_JITTER_ESTMITOR
		/*calculate jitter*/
		if (max_count >= 1) {
			delay = (max_count + 7) * s->stats->avg_rtt_ms() >> 3;
		} else {
			delay = SU_MAX(kMaxJitterDelayNoLost, s->stats->avg_rtt_ms()>> 2);
		}

		r->cache->wait_timer = (r->cache->wait_timer * 7 + delay) >> 3;
		r->cache->wait_timer = SU_MAX(r->cache->frame_timer, r->cache->wait_timer);
#endif
	}
}

static int sim_receiver_internal_put(sim_session_t* s, sim_receiver_t* r, sim_segment_t* seg)
{
	uint32_t seq;

	// first frame must be I frame
	if (r->max_seq == 0 && seg->ftype == 0)
		return -1;

	seq = seg->packet_id;
	if (r->actived == 0 || seg->packet_id <= r->base_seq)
		return -1;

	// calculate base_seq. execute once at the very beginning
	if (r->max_seq == 0 && seg->packet_id > seg->index){
		r->max_seq = seg->packet_id - seg->index - 1;
		r->base_seq = seg->packet_id - seg->index - 1;
		r->max_ts = seg->timestamp;
	}

	sim_receiver_update_loss(s, r, seg);
	if (real_video_cache_put(s, r->cache, seg) != 0)
		return -1;

	if (seg->ftype == 1)
		r->fir_state = fir_normal;

	r->max_seq = SU_MAX(r->max_seq, seq);
	r->max_ts = SU_MAX(r->max_ts, seg->timestamp);

	r->ackeds[r->acked_count++ % ACK_NUM] = seg->packet_id;

	return 0;
}

static void sim_receiver_recover(sim_session_t* s, sim_receiver_t* r)
{
	skiplist_iter_t* iter;
	skiplist_t* recover_map;
	sim_segment_t* seg, *in_seg;

	recover_map = r->recover->recover_packets;
	while (skiplist_size(recover_map) > 0){
		iter = skiplist_first(recover_map);
		seg = (sim_segment_t*)iter->val.ptr;

		in_seg = (sim_segment_t*)malloc(sizeof(sim_segment_t));
		*in_seg = *seg;
		//in_seg->recv_ts = recv_ts;

		skiplist_remove(recover_map, iter->key);

		if (sim_receiver_internal_put(s, r, in_seg) == 0){
			//sim_debug("fec recover video segment, packet id = %u\n", in_seg->packet_id);
			sim_fec_put_segment(s, r->recover, in_seg);
		} else
			free(in_seg);
	}
}

/////////////////////////////////////////////////////////////////////////////
// receiver module api
sim_receiver_t* sim_receiver_create(sim_session_t* s, int transport_type)
{
	sim_receiver_t* r = (sim_receiver_t*)calloc(1, sizeof(sim_receiver_t));

	r->loss = skiplist_create(idu32_compare, loss_free, NULL);
	r->cache = open_real_video_cache(s);
	r->cache_ts = GET_SYS_MS();
	r->s = s;
	r->fir_state = fir_normal;

	/*create receive cc*/
	r->cc_type = transport_type;
	r->cc = razor_receiver_create(r->cc_type, MIN_BITRATE, MAX_BITRATE, SIM_SEGMENT_HEADER_SIZE, r, send_sim_feedback);

	r->recover = sim_fec_create(s);

	r->jitter = new JitterWrapper();

	return r;
}

void sim_receiver_destroy(sim_session_t* s, sim_receiver_t* r)
{
	if (r == NULL)
		return;

	assert(r->cache && r->loss);
	skiplist_destroy(r->loss);
	close_real_video_cache(s, r->cache);

	/*destroy cc*/
	if (r->cc != NULL){
		razor_receiver_destroy(r->cc);
		r->cc = NULL;
	}

	if (r->recover != NULL){
		sim_fec_destroy(s, r->recover);
		r->recover = NULL;
	}

	if (r->jitter) {
		delete r->jitter;
	}

	free(r);
}

void sim_receiver_reset(sim_session_t* s, sim_receiver_t* r, int transport_type)
{
	reset_real_video_cache(s, r->cache);
	skiplist_clear(r->loss);

	r->base_uid = 0;
	r->base_seq = 0;
	r->actived = 0;
	r->max_seq = 0;
	r->max_ts = 0;
	r->ack_ts = GET_SYS_MS();
	r->active_ts = r->ack_ts;
	r->loss_count = 0;
	r->fir_state = fir_normal;
	r->fir_seq = 0;

	if (r->recover != NULL)
		sim_fec_reset(s, r->recover);

	/*recreate a cc*/
	if (r->cc != NULL){
		razor_receiver_destroy(r->cc);
		r->cc = NULL;
	}

	r->cc_type = transport_type;
	r->cc = razor_receiver_create(r->cc_type, MIN_BITRATE, MAX_BITRATE, SIM_SEGMENT_HEADER_SIZE, r, send_sim_feedback);

	r->acked_count = 0;
}

int sim_receiver_active(sim_session_t* s, sim_receiver_t* r, uint32_t uid)
{
	const int64_t kLowRttNackMs = 20;
	const int64_t kHighRttNackMs = 100;

	if (r->actived == 1)
		return -1;

	r->actived = 1;
	r->cache->frame_timer = 50; /*default frames interval ms, e.g fps=25*/

	r->base_uid = uid;
	r->active_ts = GET_SYS_MS();
	r->fir_state = fir_normal;

	if (r->recover != NULL)
		sim_fec_active(s, r->recover);

	if (r->jitter) {
		r->jitter->SetNackMode(JitterWrapper::kProtectionNackFEC);
	}

	return 0;
}

int sim_receiver_put(sim_session_t* s, sim_receiver_t* r, sim_segment_t* seg)
{
	int rc;

	if (r->cc != NULL)
		r->cc->on_received(r->cc, seg->transport_seq, seg->timestamp + seg->send_ts, seg->data_size + SIM_SEGMENT_HEADER_SIZE, seg->remb);

	rc = sim_receiver_internal_put(s, r, seg);
	if (rc != 0)
		return rc;

	//FEC recover
	if (r->recover != NULL && seg->fec_id > 0){
		sim_fec_put_segment(s, r->recover, seg);
		sim_receiver_recover(s, r);
	}

	video_real_ack(s, r, 0, seg->packet_id);

	return rc;
}

int sim_receiver_put_fec(sim_session_t* s, sim_receiver_t* r, sim_fec_t* fec)
{
	if (r->cc != NULL)
		r->cc->on_received(r->cc, fec->transport_seq, fec->send_ts, fec->fec_data_size + SIM_SEGMENT_HEADER_SIZE, 1);

	if (r->recover != NULL){
		sim_fec_put_fec_packet(s, r->recover, fec);
		sim_receiver_recover(s, r);
	}

	return 0;
}

int sim_receiver_padding(sim_session_t* s, sim_receiver_t* r, uint16_t transport_seq, uint32_t send_ts, size_t data_size)
{
	if (r->cc != NULL)
		r->cc->on_received(r->cc, transport_seq, send_ts, data_size + SIM_SEGMENT_HEADER_SIZE, 1);

	return 0;
}

/* get a frame*/
int sim_receiver_get(sim_session_t* s, sim_receiver_t* r, uint8_t* data, size_t* sizep, uint8_t* payload_type)
{
	if (r == NULL || r->actived == 0)
		return -1;

	return real_video_cache_get(s, r->cache, data, sizep, payload_type, skiplist_size(r->loss));
}

void sim_receiver_timer(sim_session_t* s, sim_receiver_t* r, int64_t now_ts)
{
	video_real_ack(s, r, 1, 0);

#ifndef USE_JITTER_ESTMITOR
	/*Attempt to reduce the buffering wait time by shrinking it.*/
	if (r->cache_ts + SU_MAX(s->stats->avg_rtt_ms(), 500) < now_ts){
		if (r->loss_count == 0)  // no packet lost reduce jitter, TODO (exclude fec case)
			r->cache->wait_timer = SU_MAX(r->cache->wait_timer * 7 >> 3, s->stats->avg_rtt_ms() >> 1);
		else if (r->cache->wait_timer > 2 * s->stats->avg_rtt_ms())
			r->cache->wait_timer = SU_MAX(r->cache->wait_timer * 15 / 16, s->stats->avg_rtt_ms());

		r->cache_ts = now_ts;
		r->loss_count = 0;
	}
#endif
	/*cc heart beat*/
	if (r->cc != NULL)
		r->cc->heartbeat(r->cc);

	if (r->recover != NULL)
		sim_fec_evict(s, r->recover, now_ts);
}

void sim_receiver_update_rtt(sim_session_t* s, sim_receiver_t* r)
{
	if (r->cc != NULL)
		r->cc->update_rtt(r->cc, s->stats->avg_rtt_ms());
}

uint32_t sim_receiver_cache_delay(sim_session_t* s, sim_receiver_t* r)
{
	return real_video_cache_delay(s, r->cache);
}
