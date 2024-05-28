/*-
* Copyright (c) 2017-2018 Razor, Inc.
*	All rights reserved.
*
* See the file LICENSE for redistribution information.
*/

#ifndef __echo_h265_encoder_h_
#define __echo_h265_encoder_h_

#include <stdint.h>
#include "codec_common.h"

class H265Encoder : public VideoEncoder
{
public:
	H265Encoder();
	~H265Encoder() override;

	bool encode(uint8_t *in, enum PixelFormat pix_fmt, uint8_t *out, int *out_size, int *frame_type, bool request_keyframe = false) override;

	int get_bitrate() const override;
protected:
	void config_param();
	void reconfig_encoder(uint32_t bitrate) override;
	bool open_encoder() override;
	void close_encoder() override;

private:
	uint8_t*		buff_;
	size_t			buff_size_;

	SwsContext*		sws_context_;

	x265_picture*	pic_out_;
	x265_picture*	en_picture_;
	x265_encoder*	en_h_;
	x265_param*	    en_param_;
};

#endif

