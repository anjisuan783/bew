#ifndef __flex_fec_sender_h_
#define __flex_fec_sender_h_

#include "flex_fec_xor.h"
#include "cf_list.h"

typedef struct
{
	uint16_t		fec_id;  // maxtrix id
	uint8_t			row;
	uint8_t			col;
	uint32_t		base_id; // the min segment id
	int				  first;   // first segment flag for find the base_id from segments
	int64_t			fec_ts;
	uint16_t		seg_size;    // segs buffer size increase by DEFAULT_SIZE
	uint16_t		segs_count;  // total meta segments
	sim_segment_t** segs;    // meta segment pointor
	uint16_t		cache_size;
	sim_segment_t** cache;   // segments pointor cache for genarating fec packets
}flex_fec_sender_t;

flex_fec_sender_t*			flex_fec_sender_create();

void flex_fec_sender_destroy(flex_fec_sender_t* fec);
void flex_fec_sender_reset(flex_fec_sender_t* fec);

// add segment to buffer, and find out the base_id
void flex_fec_sender_add_segment(flex_fec_sender_t* fec, sim_segment_t* seg);
// generate fec data
void flex_fec_sender_update(flex_fec_sender_t* fec, const uint8_t protect_fraction, base_list_t* out_fecs);
void flex_fec_sender_release(flex_fec_sender_t* fec, base_list_t* out_fecs);
// return 1: fec column on; 0: fec column off 
int flex_fec_sender_num_packets(flex_fec_sender_t* fec, const uint8_t protect_fraction);

#endif
