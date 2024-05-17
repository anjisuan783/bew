#pragma once

#include "rtc_render.h"
#include "webrtc/media/base/videosinkinterface.h"
#include "webrtc/media/base/videoframe.h"

class RTCVideoRender : public IRTCRender, public rtc::VideoSinkInterface<webrtc::VideoFrame>  {
public:
    // convert tbrtc video render to webrtc video sink
    static rtc::VideoSinkInterface<webrtc::VideoFrame>* convertVideoSink(IRTCRender *render);
    
    RTCResult setVideoScalingMode(RTCVideoScalingMode mode) override { return kErrorNotSupport; }
};

class RTCVideoExternalRenderImpl : public RTCVideoRender  {
public:
    RTCVideoExternalRenderImpl(IRTCRenderSink* sink);
    ~RTCVideoExternalRenderImpl();
    
    RTCRenderType getType() override { return kRenderTypeVideoExternal; }
    
    void OnFrame(const webrtc::VideoFrame& frame) override;
    
protected:
    IRTCRenderSink *m_sink;
    int             m_width;
    int             m_height;
};

class RTCVideoInternalRenderImpl : public RTCVideoRender {
public:
    RTCVideoInternalRenderImpl(void * window, uint32_t options = 0);
    ~RTCVideoInternalRenderImpl();
    
    RTCRenderType getType() override { return kRenderTypeVideoInternal; }
    
    void OnFrame(const webrtc::VideoFrame& frame) override;
    
protected:
    void           *m_window;
    uint32_t        m_options;
};
