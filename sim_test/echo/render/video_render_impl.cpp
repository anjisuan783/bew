#include "video_render_impl.h"

#if defined(RTC_IOS)
#include "IOSVideoRender.h"
#elif defined(RTC_MAC)
#include "MacVideoRender.h"
#elif defined(RTC_ANDROID)
#include "AndroidVideoRender.h"
#elif defined(RTC_WIN)
#include "win_video_render.h"
#endif

// video render implement
rtc::VideoSinkInterface<webrtc::VideoFrame>* RTCVideoRender::convertVideoSink(IRTCRender *render) {
    if (nullptr == render) {
        return nullptr;
    }
    
    rtc::VideoSinkInterface<webrtc::VideoFrame> * videoSink = nullptr;
    RTCRenderType type = render->getType();
    switch (type) {
        case kRenderTypeVideoExternal:
        {
            RTCVideoExternalRenderImpl *externalRender = static_cast<RTCVideoExternalRenderImpl*>(render);
            videoSink = externalRender;
        }
            break;
            
        case kRenderTypeVideoInternal:
        {
            RTCVideoInternalRenderImpl *internalRender = static_cast<RTCVideoInternalRenderImpl*>(render);
            videoSink = internalRender;
        }
            break;
            
        default:
            break;
    }
    return videoSink;
}

// video external render implement
IRTCRender * createVideoExternalRender(IRTCRenderSink * sink) {
    return new RTCVideoExternalRenderImpl(sink);
}

RTCVideoExternalRenderImpl::RTCVideoExternalRenderImpl(IRTCRenderSink *sink)
: m_sink(sink), m_width(0), m_height(0) {
    
}

RTCVideoExternalRenderImpl::~RTCVideoExternalRenderImpl() {
    m_sink = nullptr;
}

void RTCVideoExternalRenderImpl::OnFrame(const webrtc::VideoFrame &frame) {
    m_width = frame.width();
    m_height = frame.height();
    
    if( m_sink ){
        rtc::scoped_refptr<webrtc::VideoFrameBuffer> frameBuffer;
        if (frame.video_frame_buffer()->native_handle() != nullptr) {
            frameBuffer = frame.video_frame_buffer()->NativeToI420Buffer();
        } else {
            frameBuffer = frame.video_frame_buffer();
        }
        
        const uint8_t * y = frameBuffer->DataY();
        const uint8_t * u = frameBuffer->DataU();
        const uint8_t * v = frameBuffer->DataV();
        
        RTCMediaFormat format;
        format.mediaType = kMediaTypeVideo;
        format.timestamp = frame.timestamp_us();
        format.videoFmt.type = kVideoTypeI420;
        format.videoFmt.width = m_width;
        format.videoFmt.height = m_height;
        format.videoFmt.count = 3;
        format.videoFmt.stride[0] = frameBuffer->StrideY();
        format.videoFmt.stride[1] = frameBuffer->StrideU();
        format.videoFmt.stride[2] = frameBuffer->StrideV();
        format.videoFmt.offset[0] = 0;
        format.videoFmt.offset[1] = uint32_t((intptr_t)u - (intptr_t)y);
        format.videoFmt.offset[2] = uint32_t((intptr_t)v - (intptr_t)y);
        format.videoFmt.rotation = (RTCVideoRotation)frame.rotation();
        
        int length = int((intptr_t)v + (frameBuffer->StrideV() * m_height / 2) - (intptr_t)y);
        
        m_sink->onRenderData((void*)y, length, format);
    }
}

RTCVideoInternalRenderImpl::RTCVideoInternalRenderImpl(void * window, uint32_t options)
: m_window(window) {
    switch(options) {
        case 1:
            m_gdi_render = true;
            break;
        case 3:
            m_gdi_render = true;
        case 2:
            m_yuv = true;
    }
}

RTCVideoInternalRenderImpl::~RTCVideoInternalRenderImpl() {
    m_window = nullptr;
}

void RTCVideoInternalRenderImpl::OnFrame(const webrtc::VideoFrame &frame) {
    
}

// video internal render implement
IRTCRender * createVideoInternalRender(void * window, uint32_t options) {
#if defined(RTC_IOS)
    return new RTCiOSVideoRender(window, options);
#elif defined(RTC_MAC)
    return new RTCMacVideoRender(window, options);
#elif defined(RTC_ANDROID)
    return new RTCAndroidVideoRender(window);
#elif defined(RTC_WIN)
    return new RTCWinVideoRender(window, options);
#else
    return new RTCVideoInternalRenderImpl(window);
#endif
}

RTCResult destroyRender(IRTCRender * render) {
    if (render == nullptr) {
        return kErrorInvalidParam;
    }
    RTCResult ret = kNoError;
    IRTCRender::RTCRenderType type = render->getType();
    switch (type) {
        case IRTCRender::kRenderTypeVideoInternal:
        {
            RTCVideoInternalRenderImpl *internalRender = static_cast<RTCVideoInternalRenderImpl*>(render);
            if (internalRender) {
                delete internalRender;
            } else {
                ret = kErrorInvalidParam;
            }
        }
            break;
            
        case IRTCRender::kRenderTypeVideoExternal:
        {
            RTCVideoExternalRenderImpl *externalRender = static_cast<RTCVideoExternalRenderImpl*>(render);
            if (externalRender) {
                delete externalRender;
            } else {
                ret = kErrorInvalidParam;
            }
        }
            break;
            
        default:
            ret = kErrorNotSupport;
            break;
    }
    return ret;
}
