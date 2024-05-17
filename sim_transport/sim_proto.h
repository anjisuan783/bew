/*-
* Copyright (c) 2017-2018 Razor, Inc.
*	All rights reserved.
*
* See the file LICENSE for redistribution information.
*/

#ifndef __sim_proto_h_001__
#define __sim_proto_h_001__

#include "cf_platform.h"
#include "cf_stream.h"

enum {
	MIN_MSG_ID	=	0x10,

	SIM_CONNECT,			
	SIM_CONNECT_ACK,	

	SIM_DISCONNECT,
	SIM_DISCONNECT_ACK,

	SIM_PING,
	SIM_PONG,

	SIM_SEG,
	SIM_SEG_ACK,
	
	SIM_FEEDBACK,
	SIM_FIR,				
	SIM_PAD,				
	SIM_FEC,				

	MAX_MSG_ID
};

#define protocol_ver			0x01

typedef struct
{
	uint8_t		ver;		  // protocal version, must be 1
	uint8_t		mid;			// message id
	uint32_t	uid;			// user id
}sim_header_t;

#define INIT_SIM_HEADER(h, msg_id, userid) \
	h.ver = protocol_ver;	\
	h.mid = (msg_id);		\
	h.uid = (userid)

#define SIM_HEADER_SIZE			6
#define SIM_VIDEO_SIZE			1280
#define SIM_TOKEN_SIZE			128
#define NACK_NUM				20
#define ACK_NUM					NACK_NUM
#define SIM_FEEDBACK_SIZE		1000
typedef struct
{
	uint32_t cid;						/* call id, random value, must match disconnect */
	uint16_t token_size;				/* token identify */
	uint8_t	 token[SIM_TOKEN_SIZE];
	uint8_t	 cc_type;					/* congestion control type */
}sim_connect_t;

typedef struct
{
	uint32_t cid;
	uint32_t result;
}sim_connect_ack_t;

typedef struct
{
	uint32_t cid;
}sim_disconnect_t;

typedef sim_connect_ack_t sim_disconnect_ack_t;

typedef struct
{
	uint32_t	packet_id;		  /* sequence id exclude fec packets */
	uint32_t	fid;					  /* frame id, start from 1 */
	uint32_t	timestamp;			/* the time when split segments, value = now - sender->first_ts */
	uint16_t	index;					/* segment id of one frame, start from 0 */
	uint16_t	total;					/* total splited packets of one frame */
	uint8_t		ftype;					/* frame type, 1 for key, 0 for delta */
	uint8_t		payload_type;		

	uint8_t		remb;					  /* use remb */
	uint16_t	fec_id;					/* fec object id, flex_fec_sender_t.fec_id */
	uint16_t	send_ts;				/* the time when sending the seg, d-value relevant to seg->timestamp */
	uint16_t	transport_seq;	/* sequence */
	
	uint32_t	send_id;        /* sequence id include fec packets */

	uint16_t	data_size;      /* segment meta length */
	uint8_t		data[SIM_VIDEO_SIZE]; /* segment meta data */
}sim_segment_t;

#define SIM_SEGMENT_HEADER_SIZE (SIM_HEADER_SIZE + 24)

typedef struct
{
	uint32_t	base_packet_id;			/* Max package ID of the receiver received by continuation*/
	uint32_t	acked_packet_id;		/*acked immediatly packet id for calculation rtt*/

	uint8_t		nack_num;
	uint16_t	nack[NACK_NUM];

	uint8_t		ack_num;
	uint16_t	acked[ACK_NUM];
}sim_segment_ack_t;

typedef struct
{
	int64_t	ts;							/*timestamp*/
}sim_ping_t, sim_pong_t;

typedef struct
{
	uint32_t	base_packet_id;						/*the same as sim_segment_t.base_packet_id */

	uint16_t	feedback_size;						/*feedback message*/
	uint8_t		feedback[SIM_FEEDBACK_SIZE];
}sim_feedback_t;

typedef struct
{
	uint32_t	fir_seq;
}sim_fir_t;

#define PADDING_DATA_SIZE  500
typedef struct
{
	uint32_t	send_ts;				/*now_ts - sender->first_ts*/
	uint16_t	transport_seq;

	size_t		data_size;
	uint8_t		data[PADDING_DATA_SIZE];
}sim_pad_t;

typedef struct
{
	uint32_t	seq;
	uint32_t	fid;
	uint32_t	ts;
	uint16_t	index;					/* segment index */
	uint16_t	total;					/*total segments*/
	uint8_t		ftype;				/* frame type */
	uint8_t		payload_type;	/* data length */
	uint16_t	size;					
}sim_fec_meta_t;

typedef struct
{
	uint16_t		fec_id;					/*fec id, auto increase */
	uint8_t			row;					
	uint8_t			col;					
	uint8_t			index;					/* row number or column index of fec spare maxtrix 
																 0x80 & index = 0 mean this is row index */
	uint16_t		count;					/* packet number of a meta matrix */
	uint32_t		base_id;				/* begin id */

	uint32_t	send_ts;					/* now_ts - sender->first_ts */
	uint16_t	transport_seq;		/* automatic increase */

	sim_fec_meta_t	fec_meta;

	uint16_t		fec_data_size;  /* fec packet length, the max size of the segments */
	uint8_t			fec_data[SIM_VIDEO_SIZE];
}sim_fec_t;

void sim_encode_msg(bin_stream_t* strm, sim_header_t* header, void* body);
int sim_decode_header(bin_stream_t* strm, sim_header_t* header);
int sim_decode_msg(bin_stream_t* strm, sim_header_t* header, void* body);
const char* sim_get_msg_name(uint8_t msg_id);

#endif
