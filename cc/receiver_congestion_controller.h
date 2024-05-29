/*-
* Copyright (c) 2017-2018 Razor, Inc.
*	All rights reserved.
*
* See the file LICENSE for redistribution information.
*/

#ifndef __receiver_congestion_controller_h_
#define __receiver_congestion_controller_h_

#include "remote_estimator_proxy.h"
#include "remote_bitrate_estimator.h"
#include "cc_loss_stat.h"
#include "razor_api.h"

typedef struct
{
	razor_receiver_t			receiver;

	int							min_bitrate;
	int							max_bitrate;

	estimator_proxy_t*			proxy;   // cc feedback
	remote_bitrate_estimator_t* rbe;

	cc_loss_statistics_t		loss_stat;

	/*feedback function*/
	void*						handler;
	send_feedback_func			send_cb;
}receiver_cc_t;


receiver_cc_t* receiver_cc_create(int min_bitrate, int max_bitrate, int packet_header_size, void* handler, send_feedback_func cb);
void receiver_cc_destroy(receiver_cc_t* cc);

void receiver_cc_heartbeat(receiver_cc_t* cc);
void receiver_cc_on_received(receiver_cc_t* cc, uint16_t seq, uint32_t timestamp, size_t size, int remb);

void receiver_cc_update_rtt(receiver_cc_t* cc, int32_t rtt);

void receiver_cc_set_min_bitrate(receiver_cc_t* cc, int min_bitrate);
void receiver_cc_set_max_bitrate(receiver_cc_t* cc, int max_bitrate);

#endif
