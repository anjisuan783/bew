/*-
* Copyright (c) 2017-2018 Razor, Inc.
*	All rights reserved.
*
* See the file LICENSE for redistribution information.
*/

#include "video_view.h"
#include <assert.h>

VideoViewThread::VideoViewThread() { }

VideoViewThread::~VideoViewThread() { }

void VideoViewThread::set_video_devices(CFVideoRecorder* rec, CFVideoPlayer* player)
{
	rec_ = rec;
	player_ = player;
}

void VideoViewThread::run()
{
	uint8_t *data;
	int rc = MAX_PIC_SIZE, size;
	int key;
	uint8_t payload_type;

	//DWORD_PTR cpuAffinityMask = 0x00000002;
	//DWORD_PTR previousAffinityMask = SetThreadAffinityMask(m_thread_handle, cpuAffinityMask);
	//assert(previousAffinityMask != 0);

	BOOL success = SetThreadPriority(m_thread_handle, THREAD_PRIORITY_HIGHEST);
	assert(success);

	data = (uint8_t*)malloc(rc * sizeof(uint8_t));

	while (m_run_flag){
		size = rec_->read(data, rc, key, payload_type);
		if (size > 0 && player_ != NULL){
			player_->write(data, size, payload_type);
		} else
			Sleep(1);
	}

	//rec_->close();
	free(data);
	m_run_flag = true;
}
