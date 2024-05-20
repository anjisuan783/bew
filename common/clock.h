#pragma once

#include <memory>

// January 1970, in NTP seconds.
const uint32_t kNtpJan1970 = 2208988800UL;

// Magic NTP fractional unit.
const double kMagicNtpFractionalUnit = 4.294967296E+9;

class NtpTime {
 public:
  static constexpr uint64_t kFractionsPerSecond = 0x100000000;
  NtpTime() : value_(0) {}
  explicit NtpTime(uint64_t value) : value_(value) {}
  NtpTime(uint32_t seconds, uint32_t fractions)
      : value_(seconds * kFractionsPerSecond + fractions) {}

  NtpTime(const NtpTime&) = default;
  NtpTime& operator=(const NtpTime&) = default;
  explicit operator uint64_t() const { return value_; }

  void Set(uint32_t seconds, uint32_t fractions) {
    value_ = seconds * kFractionsPerSecond + fractions;
  }
  void Reset() { value_ = 0; }

  int64_t ToMs() const {
    static constexpr double kNtpFracPerMs = 4.294967296E6;  // 2^32 / 1000.
    const double frac_ms = static_cast<double>(fractions()) / kNtpFracPerMs;
    return 1000 * static_cast<int64_t>(seconds()) +
           static_cast<int64_t>(frac_ms + 0.5);
  }
  // NTP standard (RFC1305, section 3.1) explicitly state value 0 is invalid.
  bool Valid() const { return value_ != 0; }

  uint32_t seconds() const { return static_cast<uint32_t>(value_ / kFractionsPerSecond); }
  uint32_t fractions() const { return value_ % kFractionsPerSecond; }

 private:
  uint64_t value_;
};

inline bool operator==(const NtpTime& n1, const NtpTime& n2) {
  return static_cast<uint64_t>(n1) == static_cast<uint64_t>(n2);
}
inline bool operator!=(const NtpTime& n1, const NtpTime& n2) {
  return !(n1 == n2);
}

// A clock interface that allows reading of absolute and relative timestamps.
class Clock {
 public:
  virtual ~Clock() {}

  // Return a timestamp in milliseconds relative to some arbitrary source; the
  // source is fixed for this clock.
  virtual int64_t TimeInMilliseconds() const = 0;

  // Return a timestamp in microseconds relative to some arbitrary source; the
  // source is fixed for this clock.
  virtual int64_t TimeInMicroseconds() const = 0;

  // Retrieve an NTP absolute timestamp in seconds and fractions of a second.
  virtual void CurrentNtp(uint32_t& seconds, uint32_t& fractions) = 0;

  // Retrieve an NTP absolute timestamp in milliseconds.
  virtual int64_t CurrentNtpInMilliseconds() = 0;

  // TODO(danilchap): Make pure virtual once implemented in derived classed
  // replacing CurrentNtp function.
  virtual NtpTime CurrentNtpTime();

  // Converts an NTP timestamp to a millisecond timestamp.
  static int64_t NtpToMs(uint32_t seconds, uint32_t fractions) {
    return NtpTime(seconds, fractions).ToMs();
  }

  // Returns an instance of the real-time system clock implementation.
  static Clock* GetRealTimeClock();
};
