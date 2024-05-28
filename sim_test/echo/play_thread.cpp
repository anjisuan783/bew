/*-
* Copyright (c) 2017-2018 Razor, Inc.
*	All rights reserved.
*
* See the file LICENSE for redistribution information.
*/

#include "play_thread.h"
#include "sim_external.h"
#include <assert.h>


extern int frame_log(int level, const char* file, int line, const char *fmt, ...);

VideoPlayhread::VideoPlayhread() { }

VideoPlayhread::~VideoPlayhread() { }

void VideoPlayhread::set_video_devices(CFVideoPlayer* play)
{
	play_ = play;
}

void VideoPlayhread::run()
{
	uint8_t *data;
	uint8_t payload_type;

	data = (uint8_t*)malloc(MAX_PIC_SIZE * sizeof(uint8_t));
	
	uint64_t last_frame_ts = -1;
	
	while (m_run_flag && play_) {
		size_t rc = MAX_PIC_SIZE;
		if (sim_recv_video(data, &rc, &payload_type) == 0) {
			uint64_t now = GET_SYS_MS();
			if (last_frame_ts == -1) {
				last_frame_ts = now;
			}
			if (now - last_frame_ts > 100) {
				frame_log(0, __FILE__, __LINE__, "VideoPlayhread read frame cost=%llu\n", now - last_frame_ts);
			}
			last_frame_ts = now;
			play_->write(data, rc, payload_type);
			continue;
		}
		Sleep(5);
	}

	free(data);
	m_run_flag = true;
}


