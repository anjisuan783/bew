/*-
* Copyright (c) 2017-2018 Razor, Inc.
*	All rights reserved.
*
* See the file LICENSE for redistribution information.
*/

#ifndef __echo_h264_decoder_h__
#define __echo_h264_decoder_h__

#include "codec_common.h"

class H264Decoder : public VideoDecoder
{
public:
	H264Decoder();
	~H264Decoder() override;

protected:
	void set_codec_id() override;
};

#endif


