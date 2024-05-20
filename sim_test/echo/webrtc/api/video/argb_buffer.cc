#include "webrtc/api/video/argb_buffer.h"
#include "webrtc/base/checks.h"

namespace webrtc {

static const int kBufferAlignment = 64;

rtc::scoped_refptr<ArgbBuffer> ArgbBuffer::Create(int width, int height) {
  return new rtc::RefCountedObject<ArgbBuffer>(width, height);
}

ArgbBuffer::ArgbBuffer(int width, int height)
 : width_(width),
   height_(height),
   data_(static_cast<uint8_t*>(AlignedMalloc(
          width * height * 4,
          kBufferAlignment))) {
}

int ArgbBuffer::width() const {
  return width_;
}

int ArgbBuffer::height() const {
  return height_;
}

const uint8_t* ArgbBuffer::DataY() const {
  return data_.get();
}
const uint8_t* ArgbBuffer::DataU() const {
  RTC_NOTREACHED();  // Should not be called.
  return nullptr;
}
const uint8_t* ArgbBuffer::DataV() const {
  RTC_NOTREACHED();  // Should not be called.
  return nullptr;
}

int ArgbBuffer::StrideY() const {
  RTC_NOTREACHED();  // Should not be called.
  return 0;
}
int ArgbBuffer::StrideU() const {
  RTC_NOTREACHED();  // Should not be called.
  return 0;
}
int ArgbBuffer::StrideV() const {
  RTC_NOTREACHED();  // Should not be called.
  return 0;
}

void* ArgbBuffer::native_handle() const {
  return nullptr;
}

rtc::scoped_refptr<VideoFrameBuffer> ArgbBuffer::NativeToI420Buffer() {
  RTC_NOTREACHED();
  return nullptr;
}

uint8_t* ArgbBuffer::MutableDataY() {
  return const_cast<uint8_t*>(DataY());
}

uint8_t* ArgbBuffer::MutableDataU() {
  return const_cast<uint8_t*>(DataU());
}
uint8_t* ArgbBuffer::MutableDataV() {
  return const_cast<uint8_t*>(DataV());
}

}
