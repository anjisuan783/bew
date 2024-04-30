#ifndef __SIM_LIMITER_H__
#define __SIM_LIMITER_H__

#include <stdint.h>

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

void					sim_limiter_init(sim_sender_limiter_t* limiter, int windows_size_ms);
void					sim_limiter_destroy(sim_sender_limiter_t* limiter);
void					sim_limiter_set_max_bitrate(sim_sender_limiter_t* limiter, uint32_t bitrate);
int						sim_limiter_try(sim_sender_limiter_t* limiter, size_t size, int64_t now_ts);
void					sim_limiter_update(sim_sender_limiter_t* limiter, size_t size, int64_t now_ts);

#endif //!__SIM_LIMITER_H__
