/*-
* Copyright (c) 2017-2018 Razor, Inc.
*	All rights reserved.
*
* See the file LICENSE for redistribution information.
*/

#include "sim_session.h"

#include <assert.h>
#include <time.h>

#include "sim_receiver.h"
#include "sim_sender.h"
#include "exp_filter.h"
#include "call_stats.h"
#include "jitter_buffer.h"


//state
enum{
	session_idle = 0x00,
	session_connecting,
	session_connected,
	session_disconnected
};

//interrupt
enum{
	net_normal,
	net_interrupt,
	net_recover,
};

static void*	sim_session_loop_event(void* arg);
static void		sim_session_process(sim_session_t* s, bin_stream_t* strm, su_addr* addr);
static void		sim_session_heartbeat(sim_session_t* s, int64_t now_ts);

typedef void(*session_command_fn)(sim_session_t* s, uint64_t ts);
typedef void(*sim_session_processer)(sim_session_t* s, sim_header_t* header, bin_stream_t* strm, su_addr* addr);

#define MAX_TRY_TIME 100

const float kValueLossFraction = 0.99f;
const float kValueLossFractionMax = 1.0f;

static sim_session_processer* process_fn[MAX_MSG_ID];

sim_session_t* sim_session_create(uint16_t port, void* event, sim_notify_fn notify_cb, sim_change_bitrate_fn change_bitrate_cb, sim_state_fn state_cb)
{
	sim_session_t* s = (sim_session_t*)calloc(1, sizeof(sim_session_t));
	s->scid = rand();
	s->rcid = 0;
	s->uid = 0;
	s->stats = new CallStats;

	s->loss_fraction = 0;

	s->state = session_idle;
	s->interrupt = net_normal;

	s->min_bitrate = MIN_BITRATE;
	s->max_bitrate = MAX_BITRATE;
	s->start_bitrate = START_BITRATE;
	s->commad_ts = GET_SYS_MS();

	s->transport_type = gcc_transport;
	s->padding = 1;
	s->fec = 1;

	s->notify_cb = notify_cb;
	s->change_bitrate_cb = change_bitrate_cb;
	s->state_cb = state_cb;
	s->event = event;

	if (su_udp_create(NULL, port, &s->s) != 0){
		free(s);
		goto err;
	}

	su_socket_noblocking(s->s);

	s->mutex = su_create_mutex();
	bin_stream_init(&s->sstrm);

	s->exp_filter = new ExpFilter(kValueLossFraction, kValueLossFractionMax);

	//s->clock = Clock::GetRealTimeClock();

	s->run = 1;
	s->thr = su_create_thread(NULL, sim_session_loop_event, s);
err:
	return s;
}

void sim_session_destroy(sim_session_t* s)
{
	s->run = 0;
	while (s->run == 0){
		su_sleep(0, 10000);
	}

	if (s->sender != NULL){
		sim_sender_destroy(s, s->sender);
		s->sender = NULL;
	}

	if (s->receiver != NULL){
		sim_receiver_destroy(s, s->receiver);
		s->receiver = NULL;
	}

	su_destroy_mutex(s->mutex);
	bin_stream_destroy(&s->sstrm);
	su_socket_destroy(s->s);

	if (s->exp_filter)
		delete s->exp_filter;

	if (s->stats)
		delete s->stats;

	//if (s->clock)
	//	delete s->clock;
	
	free(s);
}

static void sim_session_reset(sim_session_t* s)
{
	s->uid = 0;
	s->state = session_idle;
	s->stats->reset();
	s->loss_fraction = 0;
	s->scid = rand();
	s->rcid = 0;
	s->rbitrate = 0;
	s->sbitrate = 0;
	s->rcount = 0;
	s->scount = 0;
	s->video_bytes = 0;
	s->max_frame_size = 0;
	s->fec_bitrate = 0;
	s->nack_bitrate = 0;

	s->stat_ts = GET_SYS_MS();
	s->resend = 0;
	s->commad_ts = s->stat_ts;
	s->interrupt = net_normal;
	s->transport_type = gcc_transport;
	s->padding = 1;
	s->fec = 1;
	
	s->fir_seq = 0;

	s->exp_filter->Reset(kValueLossFraction);

	if (s->sender != NULL){
		sim_sender_destroy(s, s->sender);
		s->sender = NULL;
	}

	if (s->receiver != NULL){
		sim_receiver_destroy(s, s->receiver);
		s->receiver = NULL;
	}
}

static void sim_session_send_connect(sim_session_t* s, int64_t now_ts)
{
	sim_header_t header;
	sim_connect_t body;

	INIT_SIM_HEADER(header, SIM_CONNECT, s->uid);
	body.cid = s->scid;
	body.token_size = 0;
	body.cc_type = (uint8_t)s->transport_type;
	/*?????????????????token*/

	sim_encode_msg(&s->sstrm, &header, &body);
	sim_session_network_send(s, &s->sstrm);

	msg_log(&s->peer, "send SIM_CONNECT to %s\n", ip);

	s->commad_ts = now_ts;
	s->resend++;
}

int sim_session_connect(sim_session_t* s, 
												uint32_t local_uid, 
												const char* peer_ip, 
												uint16_t peer_port, 
												int transport_type, 
												int padding, 
												int fec)
{
	int ret = -1;
	su_mutex_lock(s->mutex);

	if (s->state != session_idle){
		sim_error("session state is error! sate = %u\n", s->state);
		goto err;
	}

	su_set_addr(&s->peer, peer_ip, peer_port);
	msg_log(&s->peer, "connect reciver address = %s, state = session_connecting\n", ip);

	s->uid = local_uid;
	ret = 0;
	s->state = session_connecting;
	s->transport_type = transport_type;
	s->padding = padding;
	s->fec = fec;
	s->resend = 0;
	s->scid = rand();

	s->fir_seq = 0;

	s->loss_fraction = 0;

	sim_session_send_connect(s, GET_SYS_MS());

err:
	su_mutex_unlock(s->mutex);
	return ret;
}

static void sim_session_send_disconnect(sim_session_t* s, int64_t now_ts)
{
	sim_header_t header;
	sim_disconnect_t body;

	INIT_SIM_HEADER(header, SIM_DISCONNECT, s->uid);
	body.cid = s->scid;

	sim_encode_msg(&s->sstrm, &header, &body);
	sim_session_network_send(s, &s->sstrm);

	msg_log(&s->peer, "send SIM_DISCONNECT to %s\n", ip);

	s->commad_ts = now_ts;
	s->resend++;
}

int	sim_session_disconnect(sim_session_t* s)
{
	int ret = -1;

	su_mutex_lock(s->mutex);

	if (s->state == session_idle || s->state == session_disconnected){
		sim_error("session state is error! sate = %u\n", s->state);
		goto err;
	}

	s->resend = 0;
	s->commad_ts = GET_SYS_MS();

	s->state = session_disconnected;
	msg_log(&s->peer, "disconnect peer %s, state = session_disconnected\n", ip);

	sim_session_send_disconnect(s, s->commad_ts);

	ret = 0;
err:
	su_mutex_unlock(s->mutex);
	return ret;
}

int sim_session_send_video(sim_session_t* s, uint8_t payload_type, uint8_t ftype, const uint8_t* data, size_t size)
{
	int ret = -1;
	su_mutex_lock(s->mutex);

	if (s->state != session_connected || size <= 0)
		goto err;

	if (s->interrupt == net_interrupt)
		goto err;

	s->video_bytes += size;

	if (s->sender != NULL)
		ret = sim_sender_put(s, s->sender, payload_type, ftype, data, size);

	s->max_frame_size = SU_MAX(size, s->max_frame_size);

	//sim_debug("sim_session_send_video, video size=%u\n", size);

err:
	su_mutex_unlock(s->mutex);
	return ret;
}

int sim_session_recv_video(sim_session_t* s, uint8_t* data, size_t* sizep, uint8_t* payload_type)
{
	int ret = -1;
	su_mutex_lock(s->mutex);

	if (s->receiver != NULL)
		ret = sim_receiver_get(s, s->receiver, data, sizep, payload_type);

	su_mutex_unlock(s->mutex);
	return ret;
}

void sim_session_set_bitrates(sim_session_t* s, uint32_t min_bitrate, uint32_t start_bitrate, uint32_t max_bitrate)
{
	s->min_bitrate = (int)(min_bitrate * 1.07f);
	s->max_bitrate = (int)(max_bitrate * 1.07f);
	s->start_bitrate = (int)(start_bitrate * (SIM_SEGMENT_HEADER_SIZE + SIM_VIDEO_SIZE) * 1.07 / SIM_VIDEO_SIZE);
	s->start_bitrate = SU_MIN(max_bitrate, s->start_bitrate);

	if (s->sender != NULL)
		sim_sender_set_bitrates(s, s->sender, min_bitrate, s->start_bitrate, max_bitrate);
}

int sim_session_network_send(sim_session_t* s, bin_stream_t* strm)
{
	if (s->s < 0 || strm->used <= 0)
		return -1;
	
	s->sbitrate += strm->used;
	s->scount++;

	return su_udp_send(s->s, &s->peer, strm->data, strm->used);
}

// keep_rtt is real rtt
void sim_session_calculate_rtt(sim_session_t* s, uint32_t keep_rtt, int64_t now)
{
	s->stats->OnRttUpdate(keep_rtt, now);

	if (s->sender != NULL) {
		sim_sender_update_rtt(s, s->sender);
		s->sender->cc->update_rtt(s->sender->cc, keep_rtt);
	}

	if (s->receiver != NULL)
		sim_receiver_update_rtt(s, s->receiver);
}

/*
#define THREAD_BASE_PRIORITY_LOWRT  15  // value that gets a thread to LowRealtime-1
#define THREAD_BASE_PRIORITY_MAX    2   // maximum thread base priority boost

#define THREAD_PRIORITY_LOWEST          THREAD_BASE_PRIORITY_MIN
#define THREAD_PRIORITY_BELOW_NORMAL    (THREAD_PRIORITY_LOWEST+1)
#define THREAD_PRIORITY_NORMAL          0
#define THREAD_PRIORITY_HIGHEST         THREAD_BASE_PRIORITY_MAX
#define THREAD_PRIORITY_ABOVE_NORMAL    (THREAD_PRIORITY_HIGHEST-1)
*/
#ifdef WIN32
enum {
	LOWEST = THREAD_BASE_PRIORITY_MIN,
	BELOW_NORMAL = THREAD_PRIORITY_LOWEST,
	NORMAL = 0,
	ABOVE_NORMAL = THREAD_PRIORITY_HIGHEST - 1,
	HIGHEST = THREAD_BASE_PRIORITY_MAX
};
#endif

void SetThreadPriority(int priority) {
#ifdef WIN32
	HANDLE hThread = GetCurrentThread();
	BOOL success = SetThreadPriority(hThread, priority);

	assert(success);
#endif
}

static void* sim_session_loop_event(void* arg)
{
	sim_session_t* s = (sim_session_t*)arg;

	SetThreadPriority(HIGHEST);

	bin_stream_t rstrm;
	su_addr peer;
	int rc;
	int64_t now_ts, prev_ts;

	srand((uint32_t)time(NULL));
	prev_ts = now_ts = GET_SYS_MS();

	bin_stream_init(&rstrm);
	bin_stream_resize(&rstrm, 1500);

	while (s->run == 1){
		bin_stream_rewind(&rstrm, 1);

		rc = su_udp_recv(s->s, &peer, rstrm.data, rstrm.size, 5);
	
		if (rc >= SIM_HEADER_SIZE){
			rstrm.used = rc;

			su_mutex_lock(s->mutex);
			s->rbitrate += rc;
			s->rcount++;
			sim_session_process(s, &rstrm, &peer);
			su_mutex_unlock(s->mutex);
		}

		now_ts = GET_SYS_MS();

		if (now_ts >= prev_ts + 5){
			su_mutex_lock(s->mutex);
			sim_session_heartbeat(s, now_ts);
			su_mutex_unlock(s->mutex);

			prev_ts = now_ts;
		}
	}

	bin_stream_destroy(&rstrm);
	s->run = -1;

	return NULL;
}

static void process_sim_connect(sim_session_t* s, sim_header_t* header, bin_stream_t* strm, su_addr* addr)
{
	sim_header_t h;
	sim_connect_ack_t ack;
	sim_connect_t body;

	if (sim_decode_msg(strm, header, &body) != 0)
		return;

	msg_log(&s->peer, "recv SIM_CONNECT from %s\n", ip);

	INIT_SIM_HEADER(h, SIM_CONNECT_ACK, s->uid);
	ack.cid = body.cid;
	ack.result = 0;

	if (s->receiver == NULL){
		s->receiver = sim_receiver_create(s, body.cc_type);
		s->notify_cb(s->event, sim_start_play_notify, header->uid);
	} else {
		sim_receiver_reset(s, s->receiver, body.cc_type);
	}
	s->rcid = body.cid;

	sim_info("receiver actived!!!\n");
	sim_receiver_active(s, s->receiver, header->uid);

	sim_encode_msg(&s->sstrm, &h, &ack);
	sim_session_network_send(s, &s->sstrm);

	msg_log(addr, "send SIM_CONNECT_ACK to %s\n", ip);

	s->commad_ts = GET_SYS_MS();
	s->resend = 0;
}

static void process_sim_connect_ack(sim_session_t* s, sim_header_t* header, bin_stream_t* strm, su_addr* addr)
{
	sim_connect_ack_t ack;

	if (sim_decode_msg(strm, header, &ack) != 0)
		return;

	if (s->state != session_connecting)
		return;

	msg_log(&s->peer, "recv SIM_CONNECT_ACK from %s\n", ip);

	if (ack.result != 0){
		sim_info("connect failed, result = %u\n", ack.result);
		sim_session_reset(s);
	} else {
		s->state = session_connected;
		if (s->sender == NULL){
			s->sender = sim_sender_create(s, s->transport_type, s->padding, s->fec);
		} else {
			sim_sender_reset(s, s->sender, s->transport_type, s->padding, s->fec);
		}
		sim_sender_active(s, s->sender);

		s->interrupt = net_recover;

		sim_sender_set_bitrates(s, s->sender, s->min_bitrate, s->start_bitrate, s->max_bitrate);

		sim_info("sender actived!!!\n");

		sim_info("state = session_connected, create sim sender\n");
	}

	s->notify_cb(s->event, sim_connect_notify, ack.result);
}

static void process_sim_disconnect(sim_session_t* s, sim_header_t* header, bin_stream_t* strm, su_addr* addr)
{
	sim_header_t h;
	sim_disconnect_t body;
	sim_disconnect_ack_t ack;

	if(sim_decode_msg(strm, header, &body) != 0)
	return;

	msg_log(addr, "recv SIM_DISCONNECT from %s\n", ip);

	INIT_SIM_HEADER(h, SIM_DISCONNECT_ACK, s->uid);

	ack.cid = body.cid;
	ack.result = 0;

	sim_encode_msg(&s->sstrm, &h, &ack);
	sim_session_network_send(s, &s->sstrm);

	msg_log(addr, "send SIM_CONNECT_ACK to %s\n", ip);

	if (s->rcid == body.cid && s->receiver != NULL){
		s->notify_cb(s->event, sim_stop_play_notify, s->receiver->base_uid);
		sim_receiver_destroy(s, s->receiver);
		s->receiver = NULL;
	}
}

static void process_sim_disconnect_ack(sim_session_t* s, sim_header_t* header, bin_stream_t* strm, su_addr* addr)
{
	msg_log(&s->peer, "recv SIM_DISCONNECT_ACK from %s\n", ip);

	if (s->state == session_disconnected){
		sim_session_reset(s);
		s->notify_cb(s->event, sim_disconnect_notify, 0);
	}
}
static void process_sim_ping(sim_session_t* s, sim_header_t* header, bin_stream_t* strm, su_addr* addr)
{
	sim_header_t h;
	sim_ping_t ping;

	sim_decode_msg(strm, header, &ping);

	INIT_SIM_HEADER(h, SIM_PONG, s->uid);
	sim_encode_msg(&s->sstrm, &h, &ping);
	
	sim_session_network_send(s, &s->sstrm);
}

static void process_sim_pong(sim_session_t* s, sim_header_t* header, bin_stream_t* strm, su_addr* addr)
{
	sim_pong_t pong;
	sim_decode_msg(strm, header, &pong);

	int64_t now = GET_SYS_MS();
	int64_t diff_ts = now - pong.ts;
	//sim_session_calculate_rtt(s, SU_MAX(diff_ts, 5));
	sim_session_calculate_rtt(s, static_cast<uint32_t>(diff_ts), now);
}

static void process_sim_seg(sim_session_t* s, sim_header_t* header, bin_stream_t* strm, su_addr* addr)
{
	sim_segment_t* seg;
	if (s->receiver == NULL)
		return;

	seg = (sim_segment_t*)malloc(sizeof(sim_segment_t));
	if (sim_decode_msg(strm, header, seg) != 0 || sim_receiver_put(s, s->receiver, seg) != 0){
		free(seg);
	}
}

static void process_sim_seg_ack(sim_session_t* s, sim_header_t* header, bin_stream_t* strm, su_addr* addr)
{
	sim_segment_ack_t ack;

	if (sim_decode_msg(strm, header, &ack) != 0)
		return;

	if (s->sender != NULL)
		sim_sender_ack(s, s->sender, &ack);
}

static void process_sim_feedback(sim_session_t* s, sim_header_t* header, bin_stream_t* strm, su_addr* addr)
{
	sim_feedback_t feedback;

	if (sim_decode_msg(strm, header, &feedback) != 0)
		return;

	/*sim_debug("recv SIM_FEEDBACK, feedback size = %d\n", strm->used);*/

	if (s->sender != NULL)
		sim_sender_feedback(s, s->sender, &feedback);
}

static void process_sim_fir(sim_session_t* s, sim_header_t* header, bin_stream_t* strm, su_addr* addr)
{
	sim_fir_t fir;
	if (sim_decode_msg(strm, header, &fir) != 0)
		return;

	if (s->fir_seq < fir.fir_seq){
		s->fir_seq = fir.fir_seq;
		s->notify_cb(s->event, sim_fir_notify, 0);

		if (s->sender != NULL)
			sim_clean_ack_cache(s, s->sender);
	}
}

static void process_sim_pad(sim_session_t* s, sim_header_t* header, bin_stream_t* strm, su_addr* addr)
{
	sim_pad_t pad;
	if (sim_decode_msg(strm, header, &pad) != 0)
		return;

	if (s->receiver != NULL)
		sim_receiver_padding(s, s->receiver, pad.transport_seq, pad.send_ts, pad.data_size);	
}

static void process_sim_fec(sim_session_t* s, sim_header_t* header, bin_stream_t* strm, su_addr* addr)
{
	sim_fec_t* fec;
	fec = (sim_fec_t*)malloc(sizeof(sim_fec_t));
	if (sim_decode_msg(strm, header, fec) != 0){
		free(fec);
		return;
	}

	if (s->receiver != NULL)
		sim_receiver_put_fec(s, s->receiver, fec);
	else
		free(fec);
}


static void sim_session_process(sim_session_t* s, bin_stream_t* strm, su_addr* addr)
{
	sim_header_t header;

	if (sim_decode_header(strm, &header) != 0)
		return;

	if (header.mid < MIN_MSG_ID || header.mid > MAX_MSG_ID)
		return;

	su_addr_to_addr(addr, &s->peer);
	if (s->interrupt == net_interrupt){
		s->interrupt = net_recover;
		s->notify_cb(s->event, net_recover_notify, 0); /* notify appcatio layer network recover */
	}

	s->resend = 0;

	switch (header.mid){
	case SIM_CONNECT:
		process_sim_connect(s, &header, strm, addr);
		break;

	case SIM_CONNECT_ACK:
		process_sim_connect_ack(s, &header, strm, addr);
		break;

	case SIM_DISCONNECT:
		process_sim_disconnect(s, &header, strm, addr);
		break;

	case SIM_DISCONNECT_ACK:
		process_sim_disconnect_ack(s, &header, strm, addr);
		break;

	case SIM_PING:
		process_sim_ping(s, &header, strm, addr);
		break;

	case SIM_PONG:
		process_sim_pong(s, &header, strm, addr);
		break;

	case SIM_SEG:
		process_sim_seg(s, &header, strm, addr);
		break;

	case SIM_SEG_ACK:
		process_sim_seg_ack(s, &header, strm, addr);
		break;

	case SIM_FEEDBACK:
		process_sim_feedback(s, &header, strm, addr);
		break;

	case SIM_FIR:
		process_sim_fir(s, &header, strm, addr);
		break;

	case SIM_PAD:
		process_sim_pad(s, &header, strm, addr);
		break;

	case SIM_FEC:
		process_sim_fec(s, &header, strm, addr);
		break;
	}
}

// touched every 1s
static void sim_session_send_ping(sim_session_t* s, int64_t now_ts)
{
	sim_header_t header;
	sim_ping_t body;

	INIT_SIM_HEADER(header, SIM_PING, s->uid);
	body.ts = now_ts;

	sim_encode_msg(&s->sstrm, &header, &body);
	int ret = sim_session_network_send(s, &s->sstrm);
	assert(ret > 0);

	s->commad_ts = now_ts;
	s->resend++;

	/*exceed 3s, don't send package*/
	if (s->resend >= 4){
		if (s->sender != NULL && s->sender->cc != NULL)
			s->sender->cc->update_rtt(s->sender->cc, s->stats->avg_rtt_ms() + 20 * s->resend);
	}

	if (s->resend > 12){
		s->interrupt = net_interrupt;
		s->notify_cb(s->event, net_interrupt_notify, 0);
	}
}

#define TICK_DELAY_MS 1000
#define SIM_INFO_SIZE 1024
typedef void(*sim_session_command_func)(sim_session_t* s, int64_t now_ts);
static void sim_session_state_timer(sim_session_t* s, int64_t now_ts, sim_session_command_func fn, int type, int tick_delay)
{
	uint32_t delay;
	char info[SIM_INFO_SIZE];

	if (s->stat_ts + 1000 <= now_ts){
		delay = (uint32_t)(now_ts - s->stat_ts) * 1000;

		s->stat_ts = now_ts;

		uint32_t pacer_ms = 0;
		if (s->sender != NULL && s->sender->cc != NULL)
			pacer_ms = s->sender->cc->get_pacer_queue_ms(s->sender->cc);

		uint32_t cache_delay = 0;
		uint32_t jitter_ts = 0;
		uint32_t interval_ts = 0;
		if (s->receiver) {
			cache_delay = sim_receiver_cache_delay(s, s->receiver);
			jitter_ts = s->receiver->cache->wait_timer;
			interval_ts = s->receiver->cache->frame_timer;
		}

		if (s->state_cb != NULL){
			snprintf(info, SIM_INFO_SIZE,
				"video=%ukb/s, s=%ukb/s, r=%ukb/s, f=%ukb/s, n=%ukb/s, maxfsize=%uKB, pacer=%u, cache=%u interval=%u, jitter=%u, lost=%.2f, rtt_am=%lld-%lld",
					(uint32_t)(s->video_bytes * 1000 * 8 / delay), 
					(uint32_t)(s->sbitrate * 1000 * 8 / delay), 
					(uint32_t)(s->rbitrate * 1000 * 8 / delay), 
					(uint32_t)(s->fec_bitrate * 1000 * 8 / delay), 
					(uint32_t)(s->nack_bitrate * 1000 * 8 / delay), 
					(s->max_frame_size >> 10),
					pacer_ms, cache_delay, interval_ts, jitter_ts, s->exp_filter->filtered()*100,
					s->stats->avg_rtt_ms(), s->stats->max_rtt_ms());
			s->state_cb(s->event, info);
		}

		if (s->sender != NULL){
			sim_info("sim transport, scount=%u, rcount=%u, s=%ukb/s, r=%ukb/s, rtt=%lld-%lld, video=%ukb/s, pacer=%u, loss=%u\n",
					(uint32_t)(s->scount), 
					(uint32_t)(s->rcount), 
					(uint32_t)(s->sbitrate * 1000 * 8 / delay),
					(uint32_t)(s->rbitrate * 1000 * 8 / delay), 
					s->stats->avg_rtt_ms(), s->stats->max_rtt_ms(),
					(uint32_t)(s->video_bytes * 1000 * 8 / delay), 
					pacer_ms, s->loss_fraction);
		}

		s->rbitrate = 0;
		s->sbitrate = 0;
		s->scount = 0;
		s->rcount = 0;
		s->video_bytes = 0;
		s->max_frame_size = 0;
		s->fec_bitrate = 0;
		s->nack_bitrate = 0;
	}

	if (s->commad_ts + tick_delay <= now_ts){
		if (s->resend * tick_delay < 10000){
			fn(s, now_ts);
			if (s->resend == 2) {
				s->stats->OnRttUpdate(s->stats->avg_rtt_ms() + 500, now_ts);
				sim_debug("sim_session_state_timer rtt=%u\n", s->stats->avg_rtt_ms());
			}
		} else {
			if (s->receiver != NULL){
				s->notify_cb(s->event, sim_stop_play_notify, s->receiver->base_uid);
			}

			sim_session_reset(s);
			s->notify_cb(s->event, type, 1);
		}
	}
}

// every 5ms
static void sim_session_heartbeat(sim_session_t* s, int64_t now_ts)
{
	if (s->receiver != NULL)
		sim_receiver_timer(s, s->receiver, now_ts);

	switch (s->state){
	case session_connecting:
		sim_session_state_timer(s, now_ts, sim_session_send_connect, sim_connect_notify, TICK_DELAY_MS);
		break;

	case session_connected:
		if (s->sender != NULL)
			sim_sender_timer(s, s->sender, now_ts);
		break;

	case session_disconnected:
		sim_session_state_timer(s, now_ts, sim_session_send_disconnect, sim_disconnect_notify, TICK_DELAY_MS);
		break;

	default:
		;
	}

	if (s->sender != NULL || s->receiver != NULL)
		sim_session_state_timer(s, now_ts, sim_session_send_ping, sim_network_timout, TICK_DELAY_MS);
}
