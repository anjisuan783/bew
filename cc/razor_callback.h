/*-
* Copyright (c) 2017-2018 Razor, Inc.
*	All rights reserved.
*
* See the file LICENSE for redistribution information.
*/

#ifndef __razor_callback_function_h__
#define __razor_callback_function_h__

/*The function of transmitting receiver feedback information is to relay the receiver's information back to the sender*/
typedef void(*send_feedback_func)(void* handler, const uint8_t* payload, int payload_size);

/*The function for modifying the sender's bandwidth involves the trigger, 
	which serves as the rate control allocation entity at the communication upper layer. 
	It configures the bitrates of various modules and adjusts the corresponding video encoder bitrate accordingly.*/
typedef void(*bitrate_changed_func)(void* trigger, uint32_t bitrate, uint8_t fraction_loss, uint32_t rtt);

/*The callback function for triggering packet transmission 
  in the sender's pacer is invoked with the following parameters:

handler: the object responsible for sending
packet_id: the identifier of the packet
retrans: a flag indicating if it is a retransmission
size: the length of the packet
	When this function is called, it will use the sequence number (seq) to locate the corresponding packet 
	in the transmission queue and proceed with packet transmission.*/
typedef void(*pace_send_func)(void* handler, uint32_t packet_id, int retrans, size_t size, int padding);

typedef int(*razor_log_func)(int level, const char* file, int line, const char* fmt, va_list vl);

/*********************************send side****************************************/
typedef struct __razor_sender razor_sender_t;

/*5-10ms*/
typedef void(*sender_heartbeat_func)(razor_sender_t* sender);

typedef void(*sender_set_bitrates)(razor_sender_t* sender, uint32_t min_bitrate, uint32_t start_bitrate, uint32_t max_bitrate);

typedef int(*sender_add_packet_func)(razor_sender_t* sender, uint32_t packet_id, int retrans, size_t size);

/*After transmitting a packet to the network, 
	the upper layer needs to pass the incrementing sequence ID and the size of the packet 
	to the congestion control object for recording, in order to facilitate reference and query.*/
typedef void(*sender_on_send_func)(razor_sender_t* sender, uint16_t transport_seq, size_t size);

/*Feed the feedback information received from the receiver into the congestion control object for appropriate processing.*/
typedef void(*sender_on_feedback_func)(razor_sender_t* sender, uint8_t* feedback, int feedback_size);

typedef void(*sender_update_rtt_func)(razor_sender_t* sender, int32_t rtt);

/*Retrieve the delay of the waiting transmission queue in the congestion object.*/
typedef int(*sender_get_pacer_queue_ms_func)(razor_sender_t* sender);

/*Retrieve the timestamp of the first transmitted packet.*/
typedef int64_t(*sender_get_first_ts)(razor_sender_t* sender);

struct __razor_sender
{
	int								type;
	int								padding;
	sender_heartbeat_func			heartbeat;
	sender_set_bitrates				set_bitrates;
	sender_add_packet_func			add_packet;
	sender_on_send_func				on_send;
	sender_on_feedback_func			on_feedback;
	sender_update_rtt_func			update_rtt;
	sender_get_pacer_queue_ms_func	get_pacer_queue_ms;
	sender_get_first_ts				get_first_timestamp;
};

/*************************************receive side********************************************/
typedef struct __razor_receiver razor_receiver_t;

typedef void(*receiver_heartbeat_func)(razor_receiver_t* receiver);

/*Upon receiving a packet, the receiver performs congestion calculation using the following parameters:
	transport_seq: the incrementing channel sequence number of the transmitted packet
	timestamp: the relative timestamp of the sender (calculated using video timestamp and send offset timestamp)
	size: the size of the packet data (including UDP header and application protocol header)
	remb: indicates whether the bitrate should be calculated using REMB method. A value of 0 means REMB is used, 
				while any other value means it is not. This value is derived from the received packet header.
*/
typedef void(*receiver_on_received_func)(razor_receiver_t* receiver, uint16_t transport_seq, uint32_t timestamp, size_t size, int remb);

typedef void(*receiver_update_rtt_func)(razor_receiver_t* receiver, int32_t rtt);

typedef void(*receiver_set_min_bitrate_func)(razor_receiver_t*receiver, uint32_t bitrate);

typedef void(*receiver_set_max_bitrate_func)(razor_receiver_t*receiver, uint32_t bitrate);

struct __razor_receiver
{
	int								type;
	receiver_heartbeat_func			heartbeat;				/*unit 5ms*/
	receiver_on_received_func		on_received;			
	receiver_update_rtt_func		update_rtt;
	receiver_set_max_bitrate_func	set_max_bitrate;
	receiver_set_min_bitrate_func	set_min_bitrate;
};

#endif
