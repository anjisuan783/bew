#ifndef WEBRTC_API_VIDEO_ARGB_BUFFER_H_
#define WEBRTC_API_VIDEO_ARGB_BUFFER_H_

#include <memory>

#include "webrtc/api/video/video_frame_buffer.h"
#include "webrtc/system_wrappers/include/aligned_malloc.h"

namespace webrtc {

// Plain I420 buffer in standard memory.
class ArgbBuffer : public VideoFrameBuffer {
 public:
  static rtc::scoped_refptr<ArgbBuffer> Create(int width, int height);

  void InitializeData();

  int width() const override;
  int height() const override;
  const uint8_t* DataY() const override;
  const uint8_t* DataU() const override;
  const uint8_t* DataV() const override;

  int StrideY() const override;
  int StrideU() const override;
  int StrideV() const override;

  void* native_handle() const override;
  rtc::scoped_refptr<VideoFrameBuffer> NativeToI420Buffer() override;

  uint8_t* MutableDataY();
  uint8_t* MutableDataU();
  uint8_t* MutableDataV();
 protected:
  ArgbBuffer(int width, int height);

  ~ArgbBuffer() override = default;

 private:
  const int width_;
  const int height_;
  const std::unique_ptr<uint8_t, AlignedFreeDeleter> data_;
};

}  // namespace webrtc

#endif  // WEBRTC_API_VIDEO_ARGB_BUFFER_H_
