/*-
* Copyright (c) 2017-2018 Razor, Inc.
*	All rights reserved.
*
* See the file LICENSE for redistribution information.
*/

#include "sim_sender.h"
#include "sim_session.h"
#include "exp_filter.h"
#include "call_stats.h"

#include <assert.h>

#define MAX_SEND_COUNT		10
#define DEFAULT_SPLITS_SIZE 128

static void sim_bitrate_change(void* trigger, uint32_t bitrate, uint8_t fraction_loss, uint32_t rtt)
{
	sim_session_t* s = (sim_session_t*)trigger;
	sim_sender_t* sender = s->sender;

	uint32_t overhead_bitrate, per_packets_second, payload_bitrate, video_bitrate_kbps;
	double loss;  // max 50%
	uint32_t packet_size_bit = (SIM_SEGMENT_HEADER_SIZE + SIM_VIDEO_SIZE) * 8;

	per_packets_second = (bitrate + packet_size_bit - 1) / packet_size_bit;

	overhead_bitrate = per_packets_second * SIM_SEGMENT_HEADER_SIZE * 8;
	payload_bitrate = bitrate - overhead_bitrate;

	// fast increasing and slow dereasing
	s->loss_fraction = (fraction_loss < s->loss_fraction) ? (s->loss_fraction * 15 + fraction_loss) >> 4 : 
                      (fraction_loss + s->loss_fraction) >> 1;

	loss = SU_MIN(s->loss_fraction / 255.0, 0.5);

	s->exp_filter->Apply(1.0, loss);

	video_bitrate_kbps = (uint32_t)((1.0 - loss) * payload_bitrate) / 1000;

	// TODO (20% bandwidth for fec is not a good idea, dynamic fec bitrate is better)
	#pragma message ("20% bandwidth for fec is not a good idea, dynamic fec bitrate is better.")
	if (sender->flex != NULL)
		video_bitrate_kbps = video_bitrate_kbps * 4 / 5;

	s->change_bitrate_cb(s->event, video_bitrate_kbps, loss > 0 ? 1 : 0);

	//sim_debug("bitrate = %ukb/s, video_bitrate_kbps = %ukb/s lost = %f fraction_loss=%u\n", 
	//		bitrate / 1000, video_bitrate_kbps, loss * 100, fraction_loss);
}

static void sim_send_packet(void* handler, uint32_t send_id, int fec, size_t size, int padding)
{
	sim_header_t header;
	skiplist_iter_t* it;
	skiplist_item_t key;

	sim_sender_t* sender = (sim_sender_t*)handler;
	sim_session_t* s = sender->s;
	int64_t now_ts = GET_SYS_MS();

	if (padding == 1){
		sim_pad_t pad;
		pad.transport_seq = sender->transport_seq_seed++;
		pad.send_ts = (uint32_t)(now_ts - sender->first_ts);
		pad.data_size = SU_MIN(size, SIM_VIDEO_SIZE);

		if (sender->cc != NULL)
			sender->cc->on_send(sender->cc, pad.transport_seq, pad.data_size + SIM_SEGMENT_HEADER_SIZE);

		INIT_SIM_HEADER(header, SIM_PAD, s->uid);
		sim_encode_msg(&s->sstrm, &header, &pad);

		sim_session_network_send(s, &s->sstrm);

		if(s->loss_fraction == 0)
			s->loss_fraction = 1;
	} else if (fec == 0){
		key.u32 = send_id;
		it = skiplist_search(sender->segs_cache, key);
		if (it == NULL){
			sim_debug("send packet to network failed, send_id = %u\n", send_id);
			return;
		}

		sim_segment_t* seg = (sim_segment_t*)it->val.ptr;
		seg->transport_seq = sender->transport_seq_seed++;
		seg->send_ts = (uint16_t)(now_ts - sender->first_ts - seg->timestamp);

		if (sender->cc != NULL)
			sender->cc->on_send(sender->cc, seg->transport_seq, seg->data_size + SIM_SEGMENT_HEADER_SIZE);

		INIT_SIM_HEADER(header, SIM_SEG, s->uid);
		sim_encode_msg(&s->sstrm, &header, seg);

		sim_session_network_send(s, &s->sstrm);

		//sim_debug("send packet id=%u, transport_seq = %u ts=%u\n", seg->packet_id, seg->transport_seq, seg->timestamp);
	} else if (fec == 1){
		key.u32 = send_id;
		it = skiplist_search(sender->fecs_cache, key);
		if (it == NULL){
			sim_debug("send fec to network failed, send_id = %u\n", send_id);
			return;
		}
		sim_fec_t* fec_packet = (sim_fec_t*)it->val.ptr;
		fec_packet->transport_seq = sender->transport_seq_seed++;
		fec_packet->send_ts = (uint32_t)(now_ts - sender->first_ts);

		if (sender->cc != NULL)
			sender->cc->on_send(sender->cc, fec_packet->transport_seq, fec_packet->fec_data_size + SIM_SEGMENT_HEADER_SIZE);

		INIT_SIM_HEADER(header, SIM_FEC, s->uid);
		sim_encode_msg(&s->sstrm, &header, fec_packet);

		sim_session_network_send(s, &s->sstrm);
	}
}

static void free_video_seg(skiplist_item_t key, skiplist_item_t val, void* args)
{
	if (val.ptr != NULL)
		free(val.ptr);
}

sim_sender_t* sim_sender_create(sim_session_t* s, int transport_type, int padding, int fec)
{
	int cc_type;
	sim_sender_t* sender = (sim_sender_t*)calloc(1, sizeof(sim_sender_t));
	sender->first_ts = -1;

	sender->fecs_cache = skiplist_create(idu32_compare, free_video_seg, s);
	sender->segs_cache = skiplist_create(idu32_compare, free_video_seg, s);
	sender->ack_cache = skiplist_create(idu32_compare, NULL, s);

	cc_type = transport_type;
	if (cc_type < gcc_transport || cc_type > remb_transport)
		cc_type = gcc_congestion;

	sender->cc = razor_sender_create(cc_type, padding, s, sim_bitrate_change, sender, sim_send_packet, 1000);

	sender->s = s;

	if (fec == 1)
		sender->flex = flex_fec_sender_create();

	sender->out_fecs = create_list();

	sender->splits_size = DEFAULT_SPLITS_SIZE;
	sender->splits = (uint16_t*)malloc(sender->splits_size * sizeof(uint16_t));

	return sender;
}

void sim_sender_destroy(sim_session_t* s, sim_sender_t* sender)
{
	if (sender == NULL)
		return;

	if (sender->fecs_cache != NULL){
		skiplist_destroy(sender->fecs_cache);
		sender->fecs_cache = NULL;
	}

	if (sender->segs_cache != NULL){
		skiplist_destroy(sender->segs_cache);
		sender->segs_cache = NULL;
	}

	if (sender->ack_cache != NULL){
		skiplist_destroy(sender->ack_cache);
		sender->ack_cache = NULL;
	}

	if (sender->cc != NULL){
		razor_sender_destroy(sender->cc);
		sender->cc = NULL;
	}

	if (sender->flex != NULL){
		flex_fec_sender_destroy(sender->flex);
		sender->flex = NULL;
	}

	if (sender->out_fecs != NULL){
		destroy_list(sender->out_fecs);
		sender->out_fecs = NULL;
	}

	if (sender->splits != NULL) {
		free(sender->splits);
		sender->splits = NULL;
	}

	free(sender);
}

#define MAX_PACE_QUEUE_DELAY 300
void sim_sender_reset(sim_session_t* s, sim_sender_t* sender, int transport_type, int padding, int fec)
{
	int cc_type;

	sender->actived = 0;
	sender->base_packet_id = 0;
	sender->send_id_seed = 0;
	sender->packet_id_seed = 0;
	sender->frame_id_seed = 0;

	sender->first_ts = -1;
	sender->transport_seq_seed = 0;

	skiplist_clear(sender->fecs_cache);
	skiplist_clear(sender->segs_cache);
	skiplist_clear(sender->ack_cache);

	list_clear(sender->out_fecs);

	if (sender->cc != NULL){
		razor_sender_destroy(sender->cc);
		sender->cc = NULL;
	}

	if (sender->flex){
		flex_fec_sender_destroy(sender->flex);
		sender->flex = NULL;
	}

	cc_type = transport_type;
	if (cc_type < gcc_transport || cc_type > remb_transport)
		cc_type = gcc_congestion;

	sender->cc = razor_sender_create(cc_type, padding, s, sim_bitrate_change, sender, sim_send_packet, MAX_PACE_QUEUE_DELAY);

	if (fec == 1)
		sender->flex = flex_fec_sender_create();
}

int sim_sender_active(sim_session_t* s, sim_sender_t* sender)
{
	if (sender->actived == 1)
		return -1;

	sender->actived = 1;
	return 0;
}

static uint16_t sim_split_frame(sim_session_t* s, sim_sender_t* sender, size_t size, int segment_size)
{
	uint32_t ret, i;
	uint16_t remain_size, packet_size;

	if (size <= segment_size){
		ret = 1;
		sender->splits[0] = size;
	} else {
		ret = (size + segment_size - 1) / segment_size;
		if (ret > sender->splits_size){
			while (ret > sender->splits_size)
				sender->splits_size *= 2;
			sender->splits = (uint16_t*)realloc(sender->splits, sender->splits_size * sizeof(uint16_t));
		}
		packet_size = size / ret;
		remain_size = size % ret;

		for (i = 0; i < ret; i++){
			if (remain_size > 0){
				sender->splits[i] = packet_size + 1;
				remain_size--;
			} else
				sender->splits[i] = packet_size;
		}
	}

	return ret;
}

static void sim_sender_fec(sim_session_t* s, sim_sender_t* sender)
{
	// generating fec segments
	flex_fec_sender_update(sender->flex, s->loss_fraction, sender->out_fecs);
	if (s->loss_fraction && list_size(sender->out_fecs) == 0) {
		//sim_info("sim_sender_fec loss_fraction=%d, fec num=%d, meta count=%d, row=%d, col=%d\n", 
		//	(int)s->loss_fraction, (int)list_size(sender->out_fecs), (int)sender->flex->segs_count, sender->flex->row, sender->flex->col);
	}

	skiplist_item_t key, val;
	sim_fec_t* fec;
	while (list_size(sender->out_fecs) > 0) {
		fec = (sim_fec_t*)list_pop(sender->out_fecs);
		if (fec != NULL) {
			key.u32 = ++sender->send_id_seed;
			val.ptr = fec;
			skiplist_insert(sender->fecs_cache, key, val);
			fec->send_ts = (uint32_t)(GET_SYS_MS() - sender->first_ts);

			size_t fec_size = fec->fec_data_size + SIM_SEGMENT_HEADER_SIZE;
      s->fec_bitrate += fec_size;
			sender->cc->add_packet(sender->cc, key.u32, 1, fec_size);
		}
	}
}

int sim_sender_put(sim_session_t* s, sim_sender_t* sender, uint8_t payload_type, uint8_t ftype, const uint8_t* data, size_t size)
{	
	if (sender->cc == NULL)
		return -1;

	int64_t now_ts = GET_SYS_MS();
	const int segment_size = SIM_VIDEO_SIZE;
	const uint16_t total = sim_split_frame(s, sender, size, segment_size);

	uint32_t timestamp;  // calculate relevant send timestamp
	if (sender->first_ts == -1){
		timestamp = 0;
		sender->first_ts = now_ts;
	} else {
		timestamp = (uint32_t)(now_ts - sender->first_ts);
	}

	uint8_t* pos = (uint8_t*)data;
	++sender->frame_id_seed;
	sim_segment_t* seg;
	for (uint16_t i = 0; i < total; ++i) {
		seg = (sim_segment_t*)malloc(sizeof(sim_segment_t));

		seg->packet_id = ++sender->packet_id_seed;
		seg->send_id = ++sender->send_id_seed;
		seg->fid = sender->frame_id_seed;
		seg->timestamp = timestamp;
		seg->ftype = ftype;
		seg->payload_type = payload_type;
		seg->index = i;
		seg->total = total;

		seg->remb = 1;
		seg->fec_id = 0;
		seg->send_ts = 0;
		seg->transport_seq = 0;

		seg->data_size = sender->splits[i];
		memcpy(seg->data, pos, seg->data_size);
		pos += sender->splits[i];

		if (sender->flex != NULL){
			seg->fec_id = sender->flex->fec_id;
			flex_fec_sender_add_segment(sender->flex, seg);
		}

	  skiplist_item_t key, val;
		key.u32 = seg->send_id;
		val.ptr = seg;
		skiplist_insert(sender->segs_cache, key, val);

		key.u32 = seg->packet_id;
		skiplist_insert(sender->ack_cache, key, val);

		sender->cc->add_packet(sender->cc, seg->send_id, 0, seg->data_size + SIM_SEGMENT_HEADER_SIZE);

		if (sender->flex != NULL && sender->flex->segs_count >= 100)
			sim_sender_fec(s, sender);
	}
	if (sender->flex != NULL)
		sim_sender_fec(s, sender);

	sim_debug("sim_sender_put fid=%u size=%u\n", sender->frame_id_seed, size);

	return 0;
}

// remote out of date nack cache item
static inline void sim_sender_update_base(sim_session_t* s, sim_sender_t* sender, uint32_t base_packet_id)
{
	skiplist_iter_t* iter;
	sim_segment_t* seg;

	if (base_packet_id > sender->base_packet_id){
		sender->base_packet_id = base_packet_id;
		while (skiplist_size(sender->ack_cache) > 0) {
			iter = skiplist_first(sender->ack_cache);
			seg = (sim_segment_t*)iter->val.ptr;
			if (seg->packet_id > sender->base_packet_id)
				break;
			skiplist_remove(sender->ack_cache, iter->key);
		}
	}
}

int sim_sender_ack(sim_session_t* s, sim_sender_t* sender, sim_segment_ack_t* ack)
{
	int i;
	sim_segment_t* seg;
	skiplist_iter_t* iter;
	skiplist_item_t key;

	int64_t now_ts;

	if (ack->acked_packet_id > sender->packet_id_seed || ack->base_packet_id > sender->packet_id_seed) {
		// can't reach here
		assert(0);
		return -1;
	}

	now_ts = GET_SYS_MS();

	// skip heartbeat ack when calculating rtt
	if (0 != ack->acked_packet_id) {
		key.u32 = ack->acked_packet_id;
		iter = skiplist_search(sender->ack_cache, key);
		if (iter != NULL) {
			seg = (sim_segment_t*)iter->val.ptr;
			
			int64_t seg_send_ts = sender->first_ts + seg->timestamp + seg->send_ts;
			// calculate rtt using ack
			if (seg_send_ts <= now_ts) { 
				uint32_t diff = (uint32_t)(now_ts - seg_send_ts);
				//sim_debug("sim_sender_ack id=%u timestamp=%u, send_ts=%u, diff=%u\n", seg->packet_id, seg->timestamp, seg->send_ts, diff);
				sim_session_calculate_rtt(s, (uint32_t)(diff), now_ts);
			}
		}
	}

	sim_sender_update_base(s, sender, ack->base_packet_id);

	for (i = 0; i < ack->ack_num; ++i){
		key.u32 = ack->base_packet_id + ack->acked[i];
		iter = skiplist_remove(sender->ack_cache, key);
	}

	for (i = 0; i < ack->nack_num; ++i) {
		key.u32 = ack->base_packet_id + ack->nack[i];
		if (sender->base_packet_id >= key.u32) // duplicate nack
			continue;

		iter = skiplist_search(sender->ack_cache, key);
		if (iter != NULL){
			seg = (sim_segment_t*)iter->val.ptr;
			
			sim_debug("sim_sender_ack1, nack seq=%u\n", key.u32);
			// the time diff betwween sending this seg and sending the first seg + s->stats->avg_rtt_ms()/4 > now_ts
			// the seg hasn't arrived, ignoral nack
			if (seg->send_ts + sender->first_ts + (s->stats->avg_rtt_ms()>>1) > now_ts)
				continue;

			// ignore expired nack request
			if (sender->first_ts + seg->timestamp + CACHE_MAX_DELAY < now_ts)
				continue;

			// do not processing nack request if rtt >= CACHE_MAX_DELAY, TODO need optimization
			if (s->stats->avg_rtt_ms() < CACHE_MAX_DELAY) {
				size_t seg_size = seg->data_size + SIM_SEGMENT_HEADER_SIZE;
				s->nack_bitrate += seg_size;
				sender->cc->add_packet(sender->cc, seg->send_id, 0, seg_size);
				sim_debug("sim_sender_ack2, nack seq=%u\n", key.u32);
			}
		}
	}

	return 0;
}

void sim_clean_ack_cache(sim_session_t* s, sim_sender_t* sender)
{
	skiplist_clear(sender->ack_cache);
}

void sim_sender_feedback(sim_session_t* s, sim_sender_t* sender, sim_feedback_t* feedback)
{
	sim_sender_update_base(s, s->sender, feedback->base_packet_id);

	if (sender->cc != NULL)
		sender->cc->on_feedback(sender->cc, feedback->feedback, feedback->feedback_size);
}

void sim_sender_update_rtt(sim_session_t* s, sim_sender_t* sender)
{
	if (sender->cc != NULL){
		sender->cc->update_rtt(sender->cc, s->stats->avg_rtt_ms());
	}
}

void sim_sender_set_bitrates(sim_session_t* s, sim_sender_t* sender, uint32_t min_bitrate, uint32_t start_bitrate, uint32_t max_bitrate)
{
	if (sender->cc != NULL){
		sender->cc->set_bitrates(sender->cc, min_bitrate, start_bitrate, max_bitrate);
	}
}

#define MAX_CACHE_DELAY 6000
static void sim_sender_evict_cache(sim_session_t* s, sim_sender_t* sender, int64_t now_ts)
{
	skiplist_item_t key;
	skiplist_iter_t* iter;
	sim_segment_t* seg;
	sim_fec_t* fec;

	while (skiplist_size(sender->segs_cache) > 0){
		iter = skiplist_first(sender->segs_cache);
		seg = (sim_segment_t*)iter->val.ptr;
		if (seg->timestamp + sender->first_ts + MAX_CACHE_DELAY <= now_ts){
			key.u32 = seg->packet_id;
			skiplist_remove(sender->ack_cache, key);

			skiplist_remove(sender->segs_cache, iter->key);
		} else
			break;
	}

	while (skiplist_size(sender->fecs_cache) > 0){
		iter = skiplist_first(sender->fecs_cache);
		fec = (sim_fec_t*)iter->val.ptr;
		if (fec->send_ts + sender->first_ts + MAX_CACHE_DELAY / 3 < now_ts)
			skiplist_remove(sender->fecs_cache, iter->key);
		else
			break;
	}
}

void sim_sender_timer(sim_session_t* s, sim_sender_t* sender, uint64_t now_ts)
{
	if (sender->cc != NULL)
		sender->cc->heartbeat(sender->cc);

	sim_sender_evict_cache(s, sender, now_ts);
}
