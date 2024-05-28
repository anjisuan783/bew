#pragma once 

#include <stdint.h>

class VCMJitterEstimator;
class VCMInterFrameDelay;

class Clock;

class JitterWrapper {
 public:
  enum VCMVideoProtection {
    kProtectionNone,
    kProtectionNack,
    kProtectionFEC,
    kProtectionNackFEC,
  };

  JitterWrapper();
  ~JitterWrapper();

  void SetNackMode(VCMVideoProtection mode);
   // Returns the estimated jitter in milliseconds.
  uint32_t EstimatedJitterMs();

  void UpdateJitterEstimate(int64_t latest_packet_time_ms,
                            uint32_t timestamp,
                            unsigned int frame_size);
 private:
  // Filter for estimating jitter.
  VCMJitterEstimator* jitter_estimate_; 
  // Calculates network delays used for jitter calculations.
  VCMInterFrameDelay* inter_frame_delay_;
  VCMVideoProtection protection_mode_;
};
