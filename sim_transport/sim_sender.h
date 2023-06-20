/*-
* Copyright (c) 2017-2018 Razor, Inc.
*	All rights reserved.
*
* See the file LICENSE for redistribution information.
*/

/* nack request bandwith limiter*/
typedef struct
{
	uint32_t*		buckets;
	uint32_t		index;
	int64_t			oldest_ts;

	int				wnd_size;			/*statistics window*/
	uint32_t		wnd_bytes;
	uint32_t		threshold;			/*max sent bytes limited*/
}sim_sender_limiter_t;

struct __sim_sender
{
	int							actived;
	uint32_t					base_packet_id;			/*receiver received max continuous packets id*/

	uint32_t					packet_id_seed;			/* meta segment packet id*/
	uint32_t					send_id_seed;			/*send packets id used for cc*/
	uint32_t					frame_id_seed;    // send frame id auto increase
	uint32_t					transport_seq_seed;

	int64_t						first_ts;			/* first frame timestamp */

	skiplist_t*					segs_cache;  /* meta segment list for sending*/
	skiplist_t*					fecs_cache;  /* fec segment list for sending*/
	skiplist_t*					ack_cache;   /* sent segments need ack list */

	razor_sender_t*				cc;					/* congestion controller */

	sim_session_t*				s;

	flex_fec_sender_t*			flex;
	base_list_t*				out_fecs;

	size_t						splits_size;
	uint16_t*					splits;
};




