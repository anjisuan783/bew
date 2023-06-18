#include "flex_fec_sender.h"
#include <math.h>

#define DEFAULT_SIZE 32
#define FEC_REPAIR_WINDOW 500
#define FEC_LOSS_THROLD 10

flex_fec_sender_t* flex_fec_sender_create()
{
	flex_fec_sender_t* fec = calloc(1, sizeof(flex_fec_sender_t));
	fec->seg_size = DEFAULT_SIZE;
	fec->segs = calloc(1, sizeof(sim_segment_t*) * DEFAULT_SIZE);

	fec->cache_size = DEFAULT_SIZE;
	fec->cache = calloc(1, sizeof(sim_segment_t*) * DEFAULT_SIZE);
	fec->fec_id = 1;
	fec->first = 1;
	return fec;
}

void flex_fec_sender_destroy(flex_fec_sender_t* fec)
{
	if (fec != NULL){
		if (fec->segs != NULL){
			free(fec->segs);
			fec->segs = NULL;
		}

		if (fec->cache != NULL){
			free(fec->cache);
			fec->cache = NULL;
		}

		free(fec);
	}
}

void flex_fec_sender_reset(flex_fec_sender_t* fec)
{
	fec->fec_id = 1;
	fec->first = 1;
	fec->col = fec->row = 0;
	fec->base_id = 0;
	fec->fec_ts = 0;
	fec->segs_count = 0;
}

/*from small to big packet size, no duplicated*/
void flex_fec_sender_add_segment(flex_fec_sender_t* fec, sim_segment_t* seg)
{
	int64_t now_ts = GET_SYS_MS();
	
	/*update fec time window*/
	if (fec->fec_ts == 0)
		fec->fec_ts = now_ts;
	else if (fec->fec_ts + FEC_REPAIR_WINDOW * 4 < now_ts) {
		// fec within 2 second
		fec->segs_count = 0;
		fec->base_id = 0;
		fec->first = 1;
		fec->fec_ts = now_ts;
	}

	// the min seg->packet_id is the fec->base_id
	if (fec->first == 1)
		fec->base_id = seg->packet_id;
	else
		fec->base_id = SU_MIN(seg->packet_id, fec->base_id);

	fec->first = 0;
	
	if (fec->segs_count >= fec->seg_size){
		while(fec->segs_count >= fec->seg_size) {
			fec->seg_size += DEFAULT_SIZE;
		}
		fec->segs = realloc(fec->segs, sizeof(sim_segment_t*) * fec->seg_size);
	}
	fec->segs[fec->segs_count++] = seg;
}

int flex_fec_sender_num_packets(flex_fec_sender_t* fec, const uint8_t protect_fraction)
{
	int colum, ret = 0;
	fec->col = 0;
	fec->row = 0;
	// parameter check
	if (fec->segs_count == 0 || protect_fraction == 0){	
		return ret;
	}

	// matix
	if (protect_fraction >= FEC_LOSS_THROLD && fec->segs_count >= 6){
		double f = sqrt(fec->segs_count);
		colum = (int)f;

		// error more than 0.1f
		if (colum + 0.1f < f)
			colum = 1 + (int)f;

		// 3 <= colum <= 20
		colum = SU_MIN(20, SU_MAX(3, colum));

		fec->row = fec->segs_count / colum;
		if ((fec->segs_count % colum) != 0)
			fec->row += 1;

		fec->col = fec->segs_count / fec->row;
		if (fec->segs_count % fec->row != 0)
			fec->col += 1;

		ret = 1;
		return ret;
	}
	// row only
	colum = (fec->segs_count * protect_fraction + (1 << 7)) >> 8;
	fec->row = 1;

	if (colum == 0)
		fec->col = fec->segs_count;
	else{
		fec->col = fec->segs_count / colum;
		if (fec->segs_count % colum > 0)
			fec->col += 1;

		fec->row = fec->segs_count / fec->col;
		if ((fec->segs_count % fec->col) != 0)
			fec->row += 1;
	}
	return ret;
}

static inline int flex_fec_sender_over(flex_fec_sender_t* fec, int64_t now_ts)
{
	if (fec->fec_ts + FEC_REPAIR_WINDOW < now_ts || fec->segs_count >= 6)
		return 0;
	return -1;
}

/* fec on a completed frame */
void flex_fec_sender_update(flex_fec_sender_t* fec, uint8_t protect_fraction, base_list_t* out_fecs)
{
	sim_fec_t* out;
	int row, col, count, row_first;

	int64_t now_ts = GET_SYS_MS();

	if (flex_fec_sender_over(fec, now_ts) != 0){
		return ;
	}

	int rc = flex_fec_sender_num_packets(fec, protect_fraction);

	if (fec->col > 1){
		/*row FEC packet
		-------------------------------------------
		| 1 | 2 | 3 | 4 | 5 | R0 = xor(1,2,3,4,5) |
		-------------------------------------------
		| 6 | 7 | 8 | 9 | 10| R1 = xor(6,7,8,9,10)|
		-------------------------------------------
		*/
		for (row = 0; row < fec->row; row++){
			count = fec->col;
			row_first = row * fec->col;
			if (count > fec->segs_count - row_first)
				count = fec->segs_count - row_first;

			if (count >= 1){
				out = malloc(sizeof(sim_fec_t));

				if (flex_fec_generate(&fec->segs[row_first], count, out) == 0){			
					out->fec_id = fec->fec_id;
					out->base_id = fec->base_id;
					out->col = fec->col;
					out->row = fec->row;
					out->index = row;
					out->count = fec->segs_count;

					list_push(out_fecs, out);
				}
				else
					free(out);
			}
		}

		/*colum FEC packet
		----------------------
		| 1 | 2 | 3 | 4 | 5  |
		----------------------
		| 6 | 7 | 8 | 9 | 10 |
		----------------------
		|C1 | C2| C3| C4| C5 |
		----------------------
		*/
		if (fec->row > 1 && rc == 1){
			if (fec->cache_size < fec->row){
				while (fec->cache_size < fec->row)
					fec->cache_size += DEFAULT_SIZE;
				fec->cache = realloc(fec->cache, sizeof(sim_segment_t*) * fec->cache_size);
			}

			for (col = 0; col < fec->col; col++){
				count = 0;
				for (row = 0; row < fec->row; row++){
					row_first = row * fec->col + col;
					if (row_first < fec->segs_count)
						fec->cache[count++] = fec->segs[row_first];
					else
						break;
				}

				if (count >= 1){
					out = malloc(sizeof(sim_fec_t));

					if (flex_fec_generate(fec->cache, count, out) == 0){
						out->fec_id = fec->fec_id;
						out->base_id = fec->base_id;
						out->col = fec->col;
						out->row = fec->row;
						out->index = 0x80 | col;
						out->count = fec->segs_count;

						list_push(out_fecs, out);
					}
					else
						free(out);
				}
			}
		}
	}

	fec->fec_ts = 0;
	fec->segs_count = 0;
	fec->base_id = 0;
	fec->first = 1;

	fec->fec_id++;
	if (fec->fec_id == 0)
		fec->fec_id = 1;
}

void flex_fec_sender_release(flex_fec_sender_t* fec, base_list_t* out_fecs)
{
	base_list_unit_t* iter;
	sim_fec_t* packet;

	LIST_FOREACH(out_fecs, iter){
		packet = iter->pdata;
		if (packet != NULL)
			free(packet);
	}

	list_clear(out_fecs);
}

