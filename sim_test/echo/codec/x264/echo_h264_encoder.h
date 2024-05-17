/*-
* Copyright (c) 2017-2018 Razor, Inc.
*	All rights reserved.
*
* See the file LICENSE for redistribution information.
*/

#ifndef __h264_encoder_h__
#define __h264_encoder_h__

#include <stdint.h>

#include "codec_common.h"

/*x264 inter-refresh */
class H264Encoder : public VideoEncoder
{
public:
	H264Encoder(PixelFormat fmt);
	~H264Encoder() override;

	bool encode(uint8_t *in,
				enum PixelFormat pix_fmt, 
				uint8_t *out, 
				int *out_size, 
				int *frame_type, 
				bool request_keyframe = false) override;

	int get_bitrate() const override;

protected:
	void config_param();
	void reconfig_encoder(uint32_t bitrate) override;
	bool open_encoder() override;
	void close_encoder() override;

private:
	SwsContext*		sws_context_;

	x264_picture_t	pic_out_;
	x264_picture_t	en_picture_;
	x264_t *		en_h_;
	x264_param_t	en_param_;

	PixelFormat  fmt_ = AV_PIX_FMT_NONE;
};

#endif
