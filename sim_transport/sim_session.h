/*-
* Copyright (c) 2017-2018 Razor, Inc.
*	All rights reserved.
*
* See the file LICENSE for redistribution information.
*/

struct __sim_session
{
	su_socket		s;
	su_addr			peer;				

	uint32_t		scid;				/*sender call id*/
	uint32_t		rcid;				/*receiver call id*/
	uint32_t		uid;				/*local user ID*/

	uint32_t		rtt;				/*rtt*/
	uint32_t		rtt_var;			/*rtt error correct value*/
	uint8_t			loss_fraction;		/*lost rate, 0 ~ 255, 100% = 255*/
	uint32_t		fir_seq;

	int				state;				
	int				interrupt;			

	int				transport_type;		/*congestion control type*/
	int				padding;			/*use padding*/
	int				fec;				/*enable fec*/

	volatile int	run;				/*thread run flag*/
	su_mutex		mutex;				
	su_thread		thr;				/*thread ID*/

	sim_sender_t*	sender;				
	sim_receiver_t*	receiver;			

	int				resend;				/*resend times*/
	int64_t			commad_ts;			/*command timestamp*/
	int64_t			stat_ts;

	uint64_t		rbandwidth;			/*receive bandwidth*/
	uint64_t		sbandwidth;			/*send bandwidth*/
	uint64_t		rcount;				/*received packets nubmer*/
	uint64_t		scount;				/*sent packets number*/
	uint64_t		video_bytes;
	uint32_t		video_fps;
	uint32_t		max_frame_size;		/*max frame size within a period*/

	int				min_bitrate;		/*unit: bps*/
	int				max_bitrate;		/*unit: bps*/
	int				start_bitrate;		/*unit: bps*/

	sim_notify_fn	notify_cb;
	sim_change_bitrate_fn change_bitrate_cb;
	sim_state_fn	state_cb;
	void*			event;

	bin_stream_t	sstrm;
};

