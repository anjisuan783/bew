/*-
* Copyright (c) 2017-2018 Razor, Inc.
*	All rights reserved.
*
* See the file LICENSE for redistribution information.
*/

#include "video_device.h"

#include <initguid.h>
#include <uuids.h>
#include <vector>
#include <dvdmedia.h>
#include <sstream>
#include <thread>
#include <chrono>
#include <assert.h>

#include "libyuv.h"
#include "webrtc\api\video\i420_buffer.h"
#include "webrtc\api\video\video_frame.h"

#include "echo_h264_encoder.h"
#include "echo_h264_decoder.h"

#include "echo_h265_encoder.h"
#include "echo_h265_decoder.h"

#include "gettimeofday.h"
#include "rtc_render.h"
#include "render\video_render_impl.h"

CFVideoRecorder::CFVideoRecorder(const std::wstring& dev_name) : frame_intval_(100),
	video_data_(NULL),
	video_data_size_(0),
	hwnd_(NULL),
	hwnd_hdc_(NULL),
	dev_(dev_name)
{
	info_.width = 320;
	info_.height = 240;

	info_.codec_width = 160;
	info_.codec_height = 120;

	info_.codec = codec_h264;
	info_.rate = 8;
	info_.pix_format = RGB24;

	encoder_ = NULL;
	open_ = false;
	intra_frame_ = false;
	encode_on_ = false;
}

CFVideoRecorder::~CFVideoRecorder()
{
	if (open_){
		this->close();
	}
}

void CFVideoRecorder::set_view_hwnd(HWND hwnd, const RECT& rect)
{
	AutoSpLock auto_lock(lock_);
	if (open_) {
		if (hwnd_hdc_ != NULL && hwnd_ != NULL)
			::ReleaseDC(hwnd_, hwnd_hdc_);

		hwnd_ = hwnd;
		hwnd_rect_ = rect;
		hwnd_hdc_ = ::GetDC(hwnd_);
	} else {
		hwnd_ = hwnd;
		hwnd_rect_ = rect;
	}
}

bool CFVideoRecorder::open()
{
	if (open_)
		return true;

	AutoSpLock auto_lock(lock_);

	std::vector<SCaptureDevice> cap_devs;
	DS_Utils::EnumCaptureDevice(cap_devs);

	int dev_size = static_cast<int>(cap_devs.size());
	if (0 == dev_size)
		return false;

	CComPtr<IBaseFilter> filter_cam;

	int i = 0;
	for (; i < dev_size; i++){
		std::wstring devName = cap_devs[i].FriendlyName;
		if (devName == dev_){
			filter_cam = cap_devs[i].pFilter;
			break;
		}
	}

	if (filter_cam == NULL)
		return false;

	SDeviceCapability request{info_.width, info_.height, info_.rate, kVideoRGB24};
	int idx = GetBestMatchedCapability(cap_devs[i], request);

	HRESULT hr = cam_graph_.InitInstance(
			filter_cam, 
		    MEDIASUBTYPE_RGB32,
			CLSID_NullRenderer, 
			cap_devs[i].capabilities[idx].width, 
			cap_devs[i].capabilities[idx].height);
	if (FAILED(hr))
		return false;

	// update info_
	if (!get_device_info())
		return false;

	info_.rate = cap_devs[i].capabilities[idx].maxFPS;
	info_.pix_format = YUV420;
	frame_intval_ = 1000 / info_.rate;

	if (FAILED(hr = cam_graph_.RunGraph()))
		return false;
	
	open_ = true;

	// Get the local window render hdc
	if (hwnd_ != NULL)
		hwnd_hdc_ = ::GetDC(hwnd_);
	// Init counter frequency
	QueryPerformanceFrequency(&counter_frequency_);
	// Init prev_timer_
	QueryPerformanceCounter(&prev_timer_);

	dib_.create(info_.width, info_.height, 32);

	video_data_size_ = info_.width * info_.height * 4;
	video_data_ = new uint8_t[video_data_size_];

	if (info_.codec == codec_h264)
		encoder_ = new H264Encoder((PixelFormat)info_.pix_format);
	//else
	//	encoder_ = new H265Encoder();

	if (!encoder_->init(info_.rate, info_.width, info_.height, info_.codec_width, info_.codec_height))
		return false;

	encode_on_ = true;

	return true;
}

void CFVideoRecorder::close()
{
	AutoSpLock auto_lock(lock_);
	if (open_){
		HRESULT hr = cam_graph_.StopGraph();

		if (video_data_ != NULL){
			delete[]video_data_;
			video_data_ = NULL;
			video_data_size_ = 0;
		}

		if (hwnd_hdc_ != NULL){
			::ReleaseDC(hwnd_, hwnd_hdc_);
			hwnd_hdc_ = NULL;
		}

		dib_.destroy();

		open_ = false;
	}
	
	if (encoder_){
		encoder_->destroy();
		delete encoder_;
		encoder_ = NULL;
	}

	if (nullptr != render_) {
		destroyRender(render_);
		render_ = nullptr;
	}
}

void CFVideoRecorder::rotate_pic()
{
	unsigned char* data = (unsigned char*)dib_.get_dib_bits();
	unsigned char* src = NULL;
	unsigned char* dst = NULL;

	unsigned int line_num = info_.width * 4;
	for (int i = info_.height - 1; i >= 0; i--){
		src = (unsigned char *)data + i * line_num;
		dst = video_data_ + (info_.height - i - 1) * line_num;
		memcpy(dst, src, line_num);
	}
}

bool CFVideoRecorder::capture_sample()
{
	bool ret = false;

	CComPtr<ISampleGrabber> grabber = cam_graph_.GetPictureGrabber();
	if (grabber == NULL)
		return ret;

	while (1){
		if (!open_)
			return ret;

		unsigned long data_size = video_data_size_;
		
		HRESULT hr = grabber->GetCurrentBuffer((long *)&data_size, (long *)dib_.get_dib_bits());
		if (SUCCEEDED(hr))
			break;

		Sleep(1);
	}

	rotate_pic();

	ret = true;
	
	return ret;
}

RECT AdaptDispplay(const RECT& org, float ratio) {
	RECT new_rect = org;

	LONG w = org.right - org.left;
	LONG h = org.bottom - org.top;

	LONG new_h = static_cast<LONG>((float)w / ratio);

	if (h > new_h) {
		LONG space = (h - new_h) / 2;
		new_rect.top += space;
		new_rect.bottom -= space;
	}

	return new_rect;
}

int CFVideoRecorder::read(void* data, uint32_t data_size, int& key_frame, uint8_t& payload_type)
{
	AutoSpLock auto_lock(lock_);

	int ret = 0, out_size = 0;

	if (!open_)
		return ret;

	key_frame = 0;

	LARGE_INTEGER cur_timer;
	QueryPerformanceCounter(&cur_timer);

	LARGE_INTEGER elapsed_million_seconds;
	elapsed_million_seconds.QuadPart = cur_timer.QuadPart - prev_timer_.QuadPart;
	elapsed_million_seconds.QuadPart *= 1000;
	elapsed_million_seconds.QuadPart /= counter_frequency_.QuadPart;

	if (frame_intval_ > elapsed_million_seconds.QuadPart)
		return ret;

	prev_timer_ = cur_timer;

	data_size = 0;
	if (capture_sample()) {
		//if (hwnd_hdc_ != NULL){
			
		//	RECT new_rect = AdaptDispplay(hwnd_rect_, (float)info_.width / info_.height);
		//	dib_.stretch_blt(hwnd_hdc_, new_rect.left, new_rect.top, new_rect.right - new_rect.left,
		//			new_rect.bottom - new_rect.top, 0, 0, info_.width, info_.height);
		//}

		if(nullptr == render_) {
			render_ = createVideoInternalRender(hwnd_);
		}

		const uint32_t& width = info_.width;
		const uint32_t& height = info_.height;

		rtc::scoped_refptr<webrtc::I420Buffer> i420Buf(webrtc::I420Buffer::Create(width, height));
		libyuv::ARGBToI420((const uint8_t*)video_data_, width << 2,
			i420Buf->MutableDataY(), i420Buf->StrideY(),
			i420Buf->MutableDataU(), i420Buf->StrideU(),
			i420Buf->MutableDataV(), i420Buf->StrideV(),
			width, height);

		//PutVideoData(i420Buf);

		if (encode_on_){
			if (encoder_->encode(i420Buf->MutableDataY(), (PixelFormat)YUV420, (uint8_t*)data, &out_size, &key_frame, intra_frame_) && out_size > 0){
				key_frame = (key_frame == 0x0001 ? 1 : 0);
				data_size = out_size;
				payload_type = encoder_->get_payload_type();

				char tmp[64] = { 0 };
				info_.codec_width = encoder_->get_codec_width();
				info_.codec_height = encoder_->get_codec_height();

				++frame_count_;
				intra_frame_ = false;
			}
		}
	}

	return data_size;
}

void CFVideoRecorder::on_change_bitrate(uint32_t bitrate_kbps, int lost)
{
	AutoSpLock auto_lock(lock_);

	if (encoder_ != NULL){
		encoder_->set_bitrate(bitrate_kbps, lost);
	}
}

void CFVideoRecorder::enable_encode()
{
	AutoSpLock auto_lock(lock_);
	encode_on_ = true;
}

void CFVideoRecorder::disable_encode()
{
	AutoSpLock auto_lock(lock_);
	encode_on_ = false;
}

void CFVideoRecorder::set_intra_frame()
{
	AutoSpLock auto_lock(lock_);
	intra_frame_ = true;
}

std::string	CFVideoRecorder::get_resolution()
{
	AutoSpLock auto_lock(lock_);

	char tmp[64] = { 0 };
	snprintf(tmp, 64, "%dx%d@%d", info_.codec_width, info_.codec_height, frame_count_);
	frame_count_ = 0;
	return tmp;
}

bool CFVideoRecorder::get_device_info()
{
	CComPtr<ISampleGrabber> grabber = cam_graph_.GetPictureGrabber();
	if (grabber == NULL)
		return false;

	HRESULT hr;
	if (FAILED(hr = grabber->SetBufferSamples(TRUE)))
		return false;

	AM_MEDIA_TYPE mt;
	if (FAILED(hr = grabber->GetConnectedMediaType(&mt)))
		return false;

	if (mt.majortype != MEDIATYPE_Video)
		return false;

	long actual_width = 0, actual_height = 0;

	if (mt.formattype == FORMAT_MPEGVideo){
		VIDEOINFOHEADER *pInfo = (VIDEOINFOHEADER *)mt.pbFormat;
		actual_width = pInfo->bmiHeader.biWidth;
		actual_height = pInfo->bmiHeader.biHeight;

	} else if (mt.formattype == FORMAT_MPEG2Video){
		VIDEOINFOHEADER2 *pInfo = (VIDEOINFOHEADER2 *)mt.pbFormat;
		actual_width = pInfo->bmiHeader.biWidth;
		actual_height = pInfo->bmiHeader.biHeight;

	} else if (mt.formattype == FORMAT_VideoInfo){
		VIDEOINFOHEADER *pInfo = (VIDEOINFOHEADER *)mt.pbFormat;
		actual_width = pInfo->bmiHeader.biWidth;
		actual_height = pInfo->bmiHeader.biHeight;

	} else if (mt.formattype == FORMAT_VideoInfo2){
		VIDEOINFOHEADER2 *pInfo = (VIDEOINFOHEADER2 *)mt.pbFormat;
		actual_width = pInfo->bmiHeader.biWidth;
		actual_height = pInfo->bmiHeader.biHeight;
	} else {
		return false;
	}
	if (info_.width != actual_width)
		info_.width = static_cast<uint32_t>(actual_width);

	if (info_.height != actual_height)
		info_.height = static_cast<uint32_t>(actual_height);

	return true;
}

int CFVideoRecorder::GetBestMatchedCapability(const SCaptureDevice& dev, const SDeviceCapability& requested) {
	int32_t bestformatIndex = -1;
	int32_t bestWidth = 0;
	int32_t bestHeight = 0;
	int32_t bestFrameRate = 0;
	RawVideoType bestRawType = kVideoUnknown;

	auto& _captureCapabilities = dev.capabilities;
	const int32_t numberOfCapabilies = static_cast<int32_t>(_captureCapabilities.size());
	// Loop through all capabilities
	for (int32_t tmp = 0; tmp < numberOfCapabilies; ++tmp) { 
		auto& capability = _captureCapabilities[tmp];

		const int32_t diffWidth = capability.width - requested.width;
		const int32_t diffHeight = capability.height - requested.height;
		const int32_t diffFrameRate = capability.maxFPS - requested.maxFPS;

		const int32_t currentbestDiffWith = bestWidth - requested.width;
		const int32_t currentbestDiffHeight = bestHeight - requested.height;
		const int32_t currentbestDiffFrameRate = bestFrameRate - requested.maxFPS;

		if ((diffHeight >= 0 && diffHeight <= abs(currentbestDiffHeight)) // Height better or equalt that previouse.
				|| (currentbestDiffHeight < 0 && diffHeight >= currentbestDiffHeight)) {
			if (diffHeight == currentbestDiffHeight) {// Found best height. Care about the width)
				if ((diffWidth >= 0 && diffWidth <= abs(currentbestDiffWith)) // Width better or equal
						|| (currentbestDiffWith < 0 && diffWidth >= currentbestDiffWith)) {
					if (diffWidth == currentbestDiffWith && diffHeight == currentbestDiffHeight) { // Same size as previously
						//Also check the best frame rate if the diff is the same as previouse
						if (((diffFrameRate >= 0 &&
									diffFrameRate <= currentbestDiffFrameRate) // Frame rate to high but better match than previouse and we have not selected IUV
								||
								(currentbestDiffFrameRate < 0 &&
									diffFrameRate >= currentbestDiffFrameRate)) // Current frame rate is lower than requested. This is better.
						) {
							if ((currentbestDiffFrameRate == diffFrameRate) // Same frame rate as previous  or frame rate allready good enough
									|| (currentbestDiffFrameRate >= 0)) {
								if (bestRawType != requested.rawType
										&& requested.rawType != kVideoUnknown
										&& (capability.rawType == requested.rawType
												|| capability.rawType == kVideoI420
												|| capability.rawType == kVideoYUY2
												|| capability.rawType == kVideoYV12))
								{
										bestRawType = capability.rawType;
										bestformatIndex = tmp;
								}
								// XXX: Disable it for rawType priority effect
								// // If width height and frame rate is full filled we can use the camera for encoding if it is supported.
								// if (capability.height == requested.height
								//     && capability.width == requested.width
								//     && capability.maxFPS >= requested.maxFPS)
								// {
								//   bestformatIndex = tmp;
								// }
							} else { // Better frame rate
								bestWidth = capability.width;
								bestHeight = capability.height;
								bestFrameRate = capability.maxFPS;
								bestRawType = capability.rawType;
								bestformatIndex = tmp;
							}
						}
					} else { // Better width than previously
						bestWidth = capability.width;
						bestHeight = capability.height;
						bestFrameRate = capability.maxFPS;
						bestRawType = capability.rawType;
						bestformatIndex = tmp;
					}
				}// else width no good
			} else { // Better height
				bestWidth = capability.width;
				bestHeight = capability.height;
				bestFrameRate = capability.maxFPS;
				bestRawType = capability.rawType;
				bestformatIndex = tmp;
			}
		}// else height not good
	}//end for

	if (bestformatIndex < 0)
    return -1;

	return bestformatIndex;
}

RTCResult CFVideoRecorder::PutVideoData(rtc::scoped_refptr<webrtc::I420Buffer>& i420Buf) {

	RTCVideoRender::convertVideoSink(render_)->OnFrame(webrtc::VideoFrame(i420Buf, (webrtc::VideoRotation)kVideoRotation_0, rtc::TimeMicros()));
 
	return kNoError;
}

///////////////////////////////////////////////////////////////////
// CFVideoPlayer
CFVideoPlayer::CFVideoPlayer(HWND hWnd)
{
	hwnd_ = hWnd;
	open_ = false;
}

CFVideoPlayer::~CFVideoPlayer()
{
	this->close();
}

bool CFVideoPlayer::open()
{
	if (open_)
		return true;

	if (hwnd_ == NULL){
		return false;
	}

	codec_type_ = codec_h264;
	decoder_ = new H264Decoder();
	if (!decoder_->init(VSampleFormat_I420)){
		return false;
	}

	open_ = true;

	return true;
}

void CFVideoPlayer::close()
{
	if (!open_) {
		return;
	}
	open_ = false;

	if (decoder_ != NULL){
		decoder_->destroy();
		delete decoder_;
		decoder_ = NULL;
	}

	if (nullptr != render_) {
		destroyRender(render_);
		render_ = nullptr;
	}
}

#define SU_MAX(a, b) ((a) > (b) ? (a) : (b))

int CFVideoPlayer::write(const void* data, uint32_t size, uint8_t payload_type)
{
	int width, height;

	int32_t pic_type;

	if (size <= 0){
		return 0;
	}

	if (payload_type != codec_type_){
		decoder_->destroy();
		delete decoder_;

		codec_type_ = payload_type;
		if (payload_type == codec_h264){
			decoder_ = new H264Decoder();
		} else{
			//decoder_ = new H265Decoder();
		}

		decoder_->init(VSampleFormat_I420);
	}

	if(decoder_ != NULL && !decoder_->decode((uint8_t *)data, size, &data_, width, height, pic_type))
		return 0;

	frame_count_++;

	if (decode_data_width_ != width || decode_data_height_ != height){
		// When first time write or the width or height changed
		decode_data_width_ = width;
		decode_data_height_ = height;
	}

	if(nullptr == render_) {
		render_ = createVideoInternalRender(hwnd_);
	}

	PutVideoData((void*)data_);

	return 0;
}

std::string	CFVideoPlayer::get_resolution()
{
	AutoSpLock auto_lock(lock_);
	char tmp[64] = { 0 };
	snprintf(tmp, 64, "%dx%d@%d", decode_data_width_, decode_data_height_, frame_count_);
	frame_count_ = 0;
	return tmp;
}

RTCResult CFVideoPlayer::PutVideoData(void *buffer) {
	const int& width = decode_data_width_;
	const int& height = decode_data_height_;
	uint32_t product = width * height;
	const int frameSize = product * 3 >> 1;

	RTCMediaFormat format;
	format.mediaType = kMediaTypeVideo;
	format.videoFmt.width = width;
	format.videoFmt.height = height;
	format.videoFmt.rotation = kVideoRotation_0;
	format.videoFmt.count = 3;
	format.videoFmt.offset[0] = 0;
	format.videoFmt.offset[1] = product;
	format.videoFmt.offset[2] = product + (product >> 2);
	format.videoFmt.stride[0] = width;
	format.videoFmt.stride[1] = width >> 1;
	format.videoFmt.stride[2] = width >> 1;
	format.timestamp = rtc::TimeMicros();

	const uint8_t * y = static_cast<const uint8_t*>(buffer) + format.videoFmt.offset[0];
	const uint8_t * u = static_cast<const uint8_t*>(buffer) + format.videoFmt.offset[1];
	const uint8_t * v = static_cast<const uint8_t*>(buffer) + format.videoFmt.offset[2];
	if (1) {
		rtc::scoped_refptr<webrtc::I420Buffer> i420Buf(webrtc::I420Buffer::Create(width, height));
		memcpy((void*)i420Buf->DataY(), y, product);
		memcpy((void*)i420Buf->DataU(), u, product >> 2);
		memcpy((void*)i420Buf->DataV(), v, product >> 2);
		RTCVideoRender::convertVideoSink(render_)->OnFrame(webrtc::VideoFrame(i420Buf, (webrtc::VideoRotation)format.videoFmt.rotation, format.timestamp));
	} else {
		rtc::scoped_refptr<webrtc::I420Buffer> scaled_buf = m_bufferPool.CreateBuffer(width, height);
		libyuv::I420Scale(y, format.videoFmt.stride[0],
				u, format.videoFmt.stride[1],
				v, format.videoFmt.stride[2],
				format.videoFmt.width, format.videoFmt.height,
				(uint8*)(scaled_buf->DataY()), scaled_buf->StrideY(),
				(uint8*)(scaled_buf->DataU()), scaled_buf->StrideU(),
				(uint8*)(scaled_buf->DataV()), scaled_buf->StrideV(),
				scaled_buf->width(), scaled_buf->height(),
				libyuv::kFilterBox);
		webrtc::VideoFrame scaled_frame(scaled_buf, (webrtc::VideoRotation)format.videoFmt.rotation, format.timestamp);

		RTCVideoRender::convertVideoSink(render_)->OnFrame(scaled_frame);
	}
	return kNoError;
}


/////////////////////////////////////////////////////////////////////////////
int get_camera_input_devices(std::vector<std::wstring>& vec_cameras)
{
	std::vector<SCaptureDevice> cap_devs;
	DS_Utils::EnumCaptureDevice(cap_devs);
	size_t size = cap_devs.size();
	for (size_t i = 0; i < size; ++i)
		vec_cameras.push_back(cap_devs[i].FriendlyName);

	return size;
}
