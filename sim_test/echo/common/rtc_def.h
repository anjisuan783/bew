#pragma once

#include <stdint.h>


#ifdef  RTC_MAKE_SHARED_LIBRARY

#ifdef  WIN32
#if defined (_LIB)
#define RTC_API_EXPORT
#else 
#  define RTC_API_EXPORT   __declspec(dllexport)
#endif // _LIB
#elif __GNUC__ >= 4
#  define RTC_API_EXPORT   __attribute__ ((visibility("default")))
#else
#  define RTC_API_EXPORT
#endif

#else 

#ifdef  WIN32
#if defined (_LIB)
#define RTC_API_EXPORT
#else 
#  define RTC_API_EXPORT   __declspec(dllimport)
#endif // _LIB
#else
#  define RTC_API_EXPORT
#endif

#endif

static const int kMaxPlaneCount = 4;

/*!
* @brief Enumeration of audio mode for intended application.
*/
enum RTCAudioMode
{
    kAudioModeVoice         = 0,    ///< optimized for voice signals. 
    kAudioModeMusic48kbps   = 1,    ///< optimized for non-voice signals like music. Max average bitrate 48kbps.
    kAudioModeMusic64kbps   = 2,    ///< optimized for non-voice signals like music. Max average bitrate 64kbps.
    kAudioModeMusic96kbps   = 3,    ///< optimized for non-voice signals like music. Max average bitrate 96kbps.
    kAudioModeMusic128kbps  = 4,    ///< optimized for non-voice signals like music. Max average bitrate 128kbps.
};

/**
 * audio raw type
 */
enum RTCAudioType {
    kAudioTypePCM = 0,                     // 16bit signed integer linear PCM audio
};

/**
 * audio data's format
 */
struct RTCAudioFormat {
    RTCAudioType        type;                   //audio raw type
    uint32_t            channels;               //1 is mono, 2 is stereo
    uint32_t            sampleRate;             //sample rates per second, 44100 for example
    uint32_t            bytesPerSample;         //bytes of one sample, 2 bytes (16 bits) for example
};

/**
 * video raw type
 */
enum RTCVideoType {
    kVideoTypeI420 = 0,
};

// enum for clockwise rotation.
enum RTCVideoRotation {
    kVideoRotation_0 = 0,
    kVideoRotation_90 = 90,
    kVideoRotation_180 = 180,
    kVideoRotation_270 = 270,
};

/**
 * video data's format
 */
struct RTCVideoFormat {
    RTCVideoType        type;           //video raw type
    uint32_t            width;
    uint32_t            height;
    uint32_t            count;          //data array count, max value is 4; if data is planar, it's plane number; if not, it's 1. (I420 is 3)
    uint32_t            offset[kMaxPlaneCount]; //offset of a data beginning
    uint32_t            stride[kMaxPlaneCount]; //stride of a line, may be bigger than width; for RGB, only stride[0] is valid; for YUV(I420), stride[0]/stride[1]/stride[2] are all valid
    RTCVideoRotation    rotation;
};

/**
 * media type
 */
enum RTCMediaType {
    kMediaTypeAudio = 0,
    kMediaTypeVideo = 1,
};

/**
 * media data's format
 */
struct RTCMediaFormat
{
    RTCMediaType  mediaType;      //audio, video or other. It is used to differentiate the below union
    union {
        RTCAudioFormat audioFmt;
        RTCVideoFormat videoFmt;
    };
    uint64_t        timestamp;
};
