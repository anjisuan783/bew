#pragma once 

#include <stdint.h>

class VCMJitterEstimator;
class VCMInterFrameDelay;

class Clock;

class VCMJitterBuffer {
 public:
  enum VCMNackMode { kNack, kNoNack };

  VCMJitterBuffer(Clock* clock);
  ~VCMJitterBuffer();

  // Initializes and starts jitter buffer.
  void Start();

  // Signals all internal events and stops the jitter buffer.
  void Stop();

  // Returns true if the jitter buffer is running.
  bool Running() const;

  // Empty the jitter buffer of all its data.
  void Flush();

  void SetNackMode(VCMNackMode mode,
                  int64_t low_rtt_nack_threshold_ms,
                  int64_t high_rtt_nack_threshold_ms);
   // Returns the estimated jitter in milliseconds.
  uint32_t EstimatedJitterMs();

  // Updates the round-trip time estimate.
  void UpdateRtt(int64_t rtt_ms);

  void UpdateJitterEstimate(int64_t latest_packet_time_ms,
                            uint32_t timestamp,
                            unsigned int frame_size,
                            bool incomplete_frame);
 private:
  bool WaitForRetransmissions();

  Clock* clock_;
  // If we are running (have started) or not.
  bool running_;
  int64_t rtt_ms_;
  // Filter for estimating jitter.
  VCMJitterEstimator* jitter_estimate_; 
  // Calculates network delays used for jitter calculations.
  VCMInterFrameDelay* inter_frame_delay_;

  int64_t low_rtt_nack_threshold_ms_;
  int64_t high_rtt_nack_threshold_ms_;

  // NACK and retransmissions.
  VCMNackMode nack_mode_ = kNoNack;
};
