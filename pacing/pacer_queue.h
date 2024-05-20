/*-
* Copyright (c) 2017-2018 Razor, Inc.
*	All rights reserved.
*
* See the file LICENSE for redistribution information.
*/
#ifndef __pacer_queue_h_
#define __pacer_queue_h_

#include "cf_platform.h"
#include "cf_skiplist.h"
#include "cf_list.h"

#define k_max_pace_queue_ms			250			/*pacer queue max delay*/

typedef struct
{
	uint32_t		seq;			/*packet sequence id*/
	int				retrans;		/*retransmition*/
	size_t			size;			/*packet size*/
	int64_t			que_ts;			/*timestamp when put into pacer queue*/
	int				sent;			/*sent flag*/
}packet_event_t;

typedef struct
{
	uint32_t		max_que_ms;		/*pacer max delay*/
	size_t			total_size;
	int64_t			oldest_ts;		/*oldest timestamp*/
	skiplist_t*		cache;			/*ordered by sequence id*/
	base_list_t*	l;				/*time ordered list*/
}pacer_queue_t;

void pacer_queue_init(pacer_queue_t* que, uint32_t que_ms);
void pacer_queue_destroy(pacer_queue_t* que);

int pacer_queue_push(pacer_queue_t* que, packet_event_t* ev);
/*获取que中最小seq的包，按顺序发出，这样防止出现大范围的抖动*/
packet_event_t*			pacer_queue_front(pacer_queue_t* que);
void pacer_queue_sent_by_id(pacer_queue_t* que, uint32_t id);
void pacer_queue_sent(pacer_queue_t* que, packet_event_t* ev);

int pacer_queue_empty(pacer_queue_t* que);
size_t pacer_queue_bytes(pacer_queue_t* que);
int64_t pacer_queue_oldest(pacer_queue_t* que);
/*计算que需要的码率*/
uint32_t pacer_queue_target_bitrate_kbps(pacer_queue_t* que, int64_t now_ts);

#endif


