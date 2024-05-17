#pragma once

#include "rtc_error.h"
#include "rtc_def.h"

static const int64_t kDefaultExportFileSize   = -1;   // Default not limit export file size

/**
 * class of external audio/video render sink
 */
class IRTCRenderSink
{
public:
    //it is called as callback function. it shall be implemented by User
    RTCResult virtual onRenderData(void* buffer, int length, RTCMediaFormat& fmt) = 0;
    
    virtual ~IRTCRenderSink() {}
};


class IRTCRender {
public:
    enum RTCRenderType {
        kRenderTypeAudioInternal = 0,
        kRenderTypeAudioExternal,
        kRenderTypeVideoInternal,
        kRenderTypeVideoExternal,
    };
    virtual RTCRenderType getType() = 0;
    
    enum RTCVideoScalingMode {
        kVideoScaleFit = 0,
        kVideoScaleFullFill,
        kVideoScaleCropFill,
    };
    virtual RTCResult setVideoScalingMode(RTCVideoScalingMode mode) = 0;
    
    enum RTCVideoRenderOption {
        kVideoRenderFaceFollow = 1,
    };
    
protected:
    virtual ~IRTCRender() {}
};

// create audio render
//RTC_API_EXPORT IRTCRender * createAudioInternalRender(const char * filePath, int64_t maxFileSize = kDefaultExportFileSize);
//RTC_API_EXPORT IRTCRender * createAudioExternalRender(IRTCRenderSink * sink);

// create video render
// options == 1, use gdi render
/*RTC_API_EXPORT*/ IRTCRender * createVideoInternalRender(void * window, uint32_t options = 0);
/*RTC_API_EXPORT*/ IRTCRender * createVideoExternalRender(IRTCRenderSink * sink);

// destroy audio/video render
/*RTC_API_EXPORT*/ RTCResult    destroyRender(IRTCRender * render);
