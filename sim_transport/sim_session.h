/*-
* Copyright (c) 2017-2018 Razor, Inc.
*	All rights reserved.
*
* See the file LICENSE for redistribution information.
*/

#ifndef __sim_session_h_
#define __sim_session_h_

#include "cf_platform.h"
#include "cf_skiplist.h"
#include "sim_proto.h"
#include "cf_stream.h"
#include "razor_api.h"
#include "rate_stat.h"
#include "sim_external.h"
#include "flex_fec_receiver.h"

struct __sim_session;
typedef struct __sim_session sim_session_t;

struct __sim_receiver;
typedef struct __sim_receiver sim_receiver_t;

struct __sim_sender;
typedef struct __sim_sender sim_sender_t;

struct __sim_session
{
	su_socket		s;
	su_addr			peer;

	uint32_t		scid;				/*sender call id*/
	uint32_t		rcid;				/*receiver call id*/
	uint32_t		uid;				/*user ID*/

	uint32_t		rtt;
	uint32_t		rtt_var;			/*smooth rtt*/
	uint8_t			loss_fraction;		/* 0 ~ 255  100% = 255*/
	uint32_t		fir_seq;

	int				state;
	int				interrupt;

	int				transport_type;		/*cc type*/
	int				padding;			/*cc using padding mode*/
	int				fec;				/* enable FEC*/

	volatile int	run;				/*run flag */
	su_mutex		mutex;
	su_thread		thr;				/*thread ID*/

	sim_sender_t*	sender;
	sim_receiver_t*	receiver;

	int				resend;				/*command resend times*/
	int64_t			commad_ts;			/*command timestamp*/
	int64_t			stat_ts;

	uint64_t		rbandwidth;			/*receive bandwidth*/
	uint64_t		sbandwidth;			/*send bandwidth*/
	uint64_t		rcount;				/*received packets*/
	uint64_t		scount;				/*sent packets*/
	uint64_t		video_bytes;
	uint32_t		max_frame_size;		/*max frames in within a period*/

	int				min_bitrate;		/*target min bitrate, unit: bps*/
	int				max_bitrate;		/*target max bitrate, unit: bps*/
	int				start_bitrate;		/*target start bitrate unit: bps*/

	sim_notify_fn	notify_cb;
	sim_change_bitrate_fn change_bitrate_cb;
	sim_state_fn	state_cb;
	void*			event;

	bin_stream_t	sstrm;
};

#define MIN_BITRATE		80000					/*80kbps*/
#define MAX_BITRATE		16000000				/*16mbps*/
#define START_BITRATE	300000					/*300kbps*/

#define CACHE_MAX_DELAY 1500

sim_session_t* sim_session_create(uint16_t port, void* event, sim_notify_fn notify_cb, sim_change_bitrate_fn change_bitrate_cb, sim_state_fn state_cb);
void sim_session_destroy(sim_session_t* s);

int sim_session_connect(sim_session_t* s, uint32_t local_uid, const char* peer_ip, uint16_t peer_port, int transport_type, int padding, int fec);

int sim_session_disconnect(sim_session_t* s);

int sim_session_send_video(sim_session_t* s, uint8_t payload_type, uint8_t ftype, const uint8_t* data, size_t size);

int sim_session_recv_video(sim_session_t* s, uint8_t* data, size_t* sizep, uint8_t* payload_type);

void sim_session_set_bitrates(sim_session_t* s, uint32_t min_bitrate, uint32_t start_bitrate, uint32_t max_bitrate);

// api sendto wrapper
int sim_session_network_send(sim_session_t* s, bin_stream_t* strm);

void sim_session_calculate_rtt(sim_session_t* s, uint32_t keep_rtt);


void ex_sim_log(int level, const char* file, int line, const char *fmt, ...);

#define sim_debug(...)  				ex_sim_log(0, __FILE__, __LINE__, __VA_ARGS__)
#define sim_info(...)    				ex_sim_log(1, __FILE__, __LINE__, __VA_ARGS__)
#define sim_warn(...)					ex_sim_log(2, __FILE__, __LINE__, __VA_ARGS__)
#define sim_error(...)   				ex_sim_log(3, __FILE__, __LINE__, __VA_ARGS__)

#define msg_log(addr, ...)						\
do{												\
	char ip[IP_SIZE] = { 0 };					\
	su_addr_to_string((addr), ip, IP_SIZE);		\
	sim_info(__VA_ARGS__);						\
} while (0)

#endif



