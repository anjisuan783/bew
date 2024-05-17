/*-
* Copyright (c) 2017-2018 Razor, Inc.
*	All rights reserved.
*
* See the file LICENSE for redistribution information.
*/

#ifndef __echo_codec_common_h___
#define __echo_codec_common_h___

#ifdef _MSC_VER	/* MSVC */
#pragma warning(disable: 4996)
#endif

#include <stdint.h>

extern "C"
{
#include "x264.h"
#include "x265.h"
#include "libavcodec/avcodec.h"
#include "libavutil/avutil.h"
#include "libswscale/swscale.h"
#include "libavutil/pixfmt.h"
#include "libavformat/avformat.h"
#include "libswresample/swresample.h"
#include "libavutil/imgutils.h"
#include "libavutil/frame.h"
};

#include "rate_stat.h"

enum Resolution
{
	VIDEO_120P,
	VIDEO_240P,
	VIDEO_360P,
	VIDEO_480P,
	VIDEO_640P,
	VIDEO_720P,
	VIDEO_1080P,
};

#define KEY_FRAME_SEC 4

#define RESOLUTIONS_NUMBER 7

#define DEFAULT_FRAME_RATE 16

/*480P bitrate*/
#define MAX_VIDEO_BITRAE (800 * 1000)
#define MIN_VIDEO_BITARE (200 * 1000)
#define START_VIDEO_BITRATE (300 * 1000) 

typedef struct
{
	int			resolution;
	int			codec_width;
	int			codec_height;
	uint32_t	max_rate;
	uint32_t	min_rate;				
	uint32_t	start_rate;
}encoder_resolution_t;

extern encoder_resolution_t resolution_infos[RESOLUTIONS_NUMBER];

enum
{
	codec_raw = 32,
	codec_h263,
	codec_mpeg4,
	codec_h264,
	codec_h265,
	
	codec_vp8,
	codec_vp9
};

enum VSampleFormat {
	VSampleFormat_BGRA = 0,
	VSampleFormat_RGB = 1,
	VSampleFormat_I420 = 2,
};

void setup_codec(int codec_id);

class VideoEncoder
{
public:
	VideoEncoder();
	virtual ~VideoEncoder();

	bool init(int frame_rate, int src_width, int src_height, int dst_width, int dst_height);
	void destroy();
	
	void set_bitrate(uint32_t bitrate_kbps, int lost);

	int get_codec_width() const;
	int get_codec_height() const;

	int get_payload_type() const;

	virtual bool encode(uint8_t *in,
											enum PixelFormat pix_fmt, 
											uint8_t *out, 
											int *out_size, 
											int *frame_type, 
											bool request_keyframe = false) = 0;
	virtual int32_t get_bitrate() const = 0;

protected:
	void try_change_resolution();
	int find_resolution(uint32_t birate_kpbs);

	virtual bool open_encoder() = 0;
	virtual void reconfig_encoder(uint32_t bitrate) = 0;
	virtual void close_encoder() = 0;

protected:
	bool			inited_;

	int				payload_type_;
	unsigned int	src_width_;				// Input Width
	unsigned int	src_height_;			// Input Height

	int				frame_rate_;			// frame
	uint32_t		bitrate_kbps_;			

	int				max_resolution_;		
	int				curr_resolution_;		
	int				frame_index_;

	int64_t			up_ts_;

	rate_stat_t		rate_stat_;
};

class VideoDecoder
{
public:
	VideoDecoder();
	virtual ~VideoDecoder();

	bool init(VSampleFormat dst_fmt);
	void destroy();

	bool decode(uint8_t *in, int in_size, uint8_t **out, int &out_width, int& out_height, int32_t &pict_type);
	uint32_t get_width() const { return curr_width_; };
	uint32_t get_height() const { return curr_height_; };


protected:
	virtual void set_codec_id() = 0;

protected:
	bool				inited_;

	int					curr_width_;
	int					curr_height_;

	uint8_t*            buff_;
	int					buff_size_;

	AVFrame*			de_frame_;
	AVCodec*			de_codec_;
	AVCodecContext*		de_context_;

	AVPacket			avpkt_;
	AVPicture			outpic_;

	SwsContext*			sws_context_;

	enum AVCodecID		codec_id_;

	VSampleFormat fmt_ = VSampleFormat_I420;
};

#endif
