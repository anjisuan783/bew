#include "jitter_buffer.h"

#include <assert.h>
#include <memory>
#include <vector>
#include <string>
#include <algorithm>

#include "session_log.h"
#include "cf_platform.h"

// Use this rtt if no value has been reported.
static const int64_t kDefaultRtt = 200;

// RollingAccumulator stores and reports statistics
// over N most recent samples.
//
// T is assumed to be an int, long, double or float.
template<typename T>
class RollingAccumulator {
 public:
  explicit RollingAccumulator(size_t max_count)
    : samples_(max_count) {
    Reset();
  }
  ~RollingAccumulator() {
  }

  size_t max_count() const {
    return samples_.size();
  }

  size_t count() const {
    return count_;
  }

  void Reset() {
    count_ = 0U;
    next_index_ = 0U;
    sum_ = 0.0;
    sum_2_ = 0.0;
    max_ = T();
    max_stale_ = false;
    min_ = T();
    min_stale_ = false;
  }

  void AddSample(T sample) {
    if (count_ == max_count()) {
      // Remove oldest sample.
      T sample_to_remove = samples_[next_index_];
      sum_ -= sample_to_remove;
      sum_2_ -= static_cast<double>(sample_to_remove) * sample_to_remove;
      if (sample_to_remove >= max_) {
        max_stale_ = true;
      }
      if (sample_to_remove <= min_) {
        min_stale_ = true;
      }
    } else {
      // Increase count of samples.
      ++count_;
    }
    // Add new sample.
    samples_[next_index_] = sample;
    sum_ += sample;
    sum_2_ += static_cast<double>(sample) * sample;
    if (count_ == 1 || sample >= max_) {
      max_ = sample;
      max_stale_ = false;
    }
    if (count_ == 1 || sample <= min_) {
      min_ = sample;
      min_stale_ = false;
    }
    // Update next_index_.
    next_index_ = (next_index_ + 1) % max_count();
  }

  T ComputeSum() const {
    return static_cast<T>(sum_);
  }

  double ComputeMean() const {
    if (count_ == 0) {
      return 0.0;
    }
    return sum_ / count_;
  }

  T ComputeMax() const {
    if (max_stale_) {
      RTC_DCHECK(count_ > 0) <<
                 "It shouldn't be possible for max_stale_ && count_ == 0";
      max_ = samples_[next_index_];
      for (size_t i = 1u; i < count_; i++) {
        max_ = std::max(max_, samples_[(next_index_ + i) % max_count()]);
      }
      max_stale_ = false;
    }
    return max_;
  }

  T ComputeMin() const {
    if (min_stale_) {
      RTC_DCHECK(count_ > 0) <<
                 "It shouldn't be possible for min_stale_ && count_ == 0";
      min_ = samples_[next_index_];
      for (size_t i = 1u; i < count_; i++) {
        min_ = std::min(min_, samples_[(next_index_ + i) % max_count()]);
      }
      min_stale_ = false;
    }
    return min_;
  }

  // O(n) time complexity.
  // Weights nth sample with weight (learning_rate)^n. Learning_rate should be
  // between (0.0, 1.0], otherwise the non-weighted mean is returned.
  double ComputeWeightedMean(double learning_rate) const {
    if (count_ < 1 || learning_rate <= 0.0 || learning_rate >= 1.0) {
      return ComputeMean();
    }
    double weighted_mean = 0.0;
    double current_weight = 1.0;
    double weight_sum = 0.0;
    const size_t max_size = max_count();
    for (size_t i = 0; i < count_; ++i) {
      current_weight *= learning_rate;
      weight_sum += current_weight;
      // Add max_size to prevent underflow.
      size_t index = (next_index_ + max_size - i - 1) % max_size;
      weighted_mean += current_weight * samples_[index];
    }
    return weighted_mean / weight_sum;
  }

  // Compute estimated variance.  Estimation is more accurate
  // as the number of samples grows.
  double ComputeVariance() const {
    if (count_ == 0) {
      return 0.0;
    }
    // Var = E[x^2] - (E[x])^2
    double count_inv = 1.0 / count_;
    double mean_2 = sum_2_ * count_inv;
    double mean = sum_ * count_inv;
    return mean_2 - (mean * mean);
  }

 private:
  size_t count_;
  size_t next_index_;
  double sum_;    // Sum(x) - double to avoid overflow
  double sum_2_;  // Sum(x*x) - double to avoid overflow
  mutable T max_;
  mutable bool max_stale_;
  mutable T min_;
  mutable bool min_stale_;
  std::vector<T> samples_;
};

//////////////////////////////////////////////////////////
// VCMInterFrameDelay
class VCMInterFrameDelay {
 public:
  explicit VCMInterFrameDelay(int64_t currentWallClock);

  // Resets the estimate. Zeros are given as parameters.
  void Reset(int64_t currentWallClock);

  // Calculates the delay of a frame with the given timestamp.
  // This method is called when the frame is complete.
  //
  // Input:
  //          - timestamp         : RTP timestamp of a received frame
  //          - *delay            : Pointer to memory where the result should be
  //          stored
  //          - currentWallClock  : The current time in milliseconds.
  //                                Should be -1 for normal operation, only used
  //                                for testing.
  // Return value                 : true if OK, false when reordered timestamps
  bool CalculateDelay(uint32_t timestamp,
                      int64_t* delay,
                      int64_t currentWallClock);

 private:
  // Controls if the RTP timestamp counter has had a wrap around
  // between the current and the previously received frame.
  //
  // Input:
  //          - timestmap         : RTP timestamp of the current frame.
  void CheckForWrapArounds(uint32_t timestamp);

  int64_t _zeroWallClock;  // Local timestamp of the first video packet received
  int32_t _wrapArounds;    // Number of wrapArounds detected
  // The previous timestamp passed to the delay estimate
  uint32_t _prevTimestamp;
  // The previous wall clock timestamp used by the delay estimate
  int64_t _prevWallClock;
  // Wrap-around compensated difference between incoming timestamps
  int64_t _dTS;
};

VCMInterFrameDelay::VCMInterFrameDelay(int64_t currentWallClock) {
  Reset(currentWallClock);
}

// Resets the delay estimate
void VCMInterFrameDelay::Reset(int64_t currentWallClock) {
  _zeroWallClock = currentWallClock;
  _wrapArounds = 0;
  _prevWallClock = 0;
  _prevTimestamp = 0;
  _dTS = 0;
}

// Calculates the delay of a frame with the given timestamp.
// This method is called when the frame is complete.
bool VCMInterFrameDelay::CalculateDelay(uint32_t timestamp,
                                        int64_t* delay,
                                        int64_t currentWallClock) {
  if (_prevWallClock == 0) {
    // First set of data, initialization, wait for next frame
    _prevWallClock = currentWallClock;
    _prevTimestamp = timestamp;
    *delay = 0;
    return true;
  }

  int32_t prevWrapArounds = _wrapArounds;
  CheckForWrapArounds(timestamp);

  // This will be -1 for backward wrap arounds and +1 for forward wrap arounds
  int32_t wrapAroundsSincePrev = _wrapArounds - prevWrapArounds;

  // Account for reordering in jitter variance estimate in the future?
  // Note that this also captures incomplete frames which are grabbed
  // for decoding after a later frame has been complete, i.e. real
  // packet losses.
  if ((wrapAroundsSincePrev == 0 && timestamp < _prevTimestamp) ||
      wrapAroundsSincePrev < 0) {
    *delay = 0;
    return false;
  }

  // Compute the compensated timestamp difference and convert it to ms and
  // round it to closest integer.
  _dTS = static_cast<int64_t>(
      (timestamp + wrapAroundsSincePrev * (static_cast<int64_t>(1) << 32) -
       _prevTimestamp) /
          90.0 +
      0.5);

  // frameDelay is the difference of dT and dTS -- i.e. the difference of
  // the wall clock time difference and the timestamp difference between
  // two following frames.
  *delay = static_cast<int64_t>(currentWallClock - _prevWallClock - _dTS);

  _prevTimestamp = timestamp;
  _prevWallClock = currentWallClock;

  return true;
}

// Investigates if the timestamp clock has overflowed since the last timestamp
// and keeps track of the number of wrap arounds since reset.
void VCMInterFrameDelay::CheckForWrapArounds(uint32_t timestamp) {
  if (timestamp < _prevTimestamp) {
    // This difference will probably be less than -2^31 if we have had a wrap
    // around
    // (e.g. timestamp = 1, _previousTimestamp = 2^32 - 1). Since it is cast to
    // a Word32,
    // it should be positive.
    if (static_cast<int32_t>(timestamp - _prevTimestamp) > 0) {
      // Forward wrap around
      _wrapArounds++;
    }
    // This difference will probably be less than -2^31 if we have had a
    // backward
    // wrap around.
    // Since it is cast to a Word32, it should be positive.
  } else if (static_cast<int32_t>(_prevTimestamp - timestamp) > 0) {
    // Backward wrap around
    _wrapArounds--;
  }
}

/////////////////////////////////////////////////////////
// VCMJitterEstimator
class VCMJitterEstimator {
 public:
  VCMJitterEstimator();
  virtual ~VCMJitterEstimator();
  
  // Updates the jitter estimate with the new data.
  //
  // Input:
  //          - frameDelay      : Delay-delta calculated by UTILDelayEstimate in
  //          milliseconds
  //          - frameSize       : Frame size of the current frame.
  //          - incompleteFrame : Flags if the frame is used to update the
  //          estimate before it
  //                              was complete. Default is false.
  void UpdateEstimate(int64_t frameDelayMS,
                      uint32_t frameSizeBytes,
                      bool incompleteFrame = false);

  // Returns the current jitter estimate in milliseconds and adds
  // also adds an RTT dependent term in cases of retransmission.
  //  Input:
  //          - rttMultiplier  : RTT param multiplier (when applicable).
  //
  // Return value                   : Jitter estimate in milliseconds
  virtual int GetJitterEstimate(double rttMultiplier);

  // A constant describing the delay from the jitter buffer
  // to the delay on the receiving side which is not accounted
  // for by the jitter buffer nor the decoding delay estimate.
  static const uint32_t OPERATING_SYSTEM_JITTER = 10;

 protected:
  // These are protected for better testing possibilities
  double _theta[2];  // Estimated line parameters (slope, offset), frameDelay=1/C + networkjitter
  double _varNoise;  // Variance of the time-deviation from the line

  virtual bool LowRateExperimentEnabled();

 private:
  // Resets the estimate to the initial state
  void Reset();
  // Updates the Kalman filter for the line describing
  // the frame size dependent jitter.
  //
  // Input:
  //          - frameDelayMS    : Delay-delta calculated by UTILDelayEstimate in
  //          milliseconds
  //          - deltaFSBytes    : Frame size delta, i.e.
  //                            : frame size at time T minus frame size at time
  //                            T-1
  void KalmanEstimateChannel(int64_t frameDelayMS, int32_t deltaFSBytes);

  // Updates the random jitter estimate, i.e. the variance
  // of the time deviations from the line given by the Kalman filter.
  //
  // Input:
  //          - d_dT              : The deviation from the kalman estimate
  //          - incompleteFrame   : True if the frame used to update the
  //          estimate
  //                                with was incomplete
  void EstimateRandomJitter(double d_dT, bool incompleteFrame);

  double NoiseThreshold() const;

  // Calculates the current jitter estimate.
  //
  // Return value                 : The current jitter estimate in milliseconds
  double CalculateEstimate();

  // Post process the calculated estimate
  void PostProcessEstimate();

  // Calculates the difference in delay between a sample and the
  // expected delay estimated by the Kalman filter.
  //
  // Input:
  //          - frameDelayMS    : Delay-delta calculated by UTILDelayEstimate in
  //          milliseconds
  //          - deltaFS         : Frame size delta, i.e. frame size at time
  //                              T minus frame size at time T-1
  //
  // Return value                 : The difference in milliseconds
  double DeviationFromExpectedDelay(int64_t frameDelayMS,
                                    int32_t deltaFSBytes) const;

  double GetFrameRate() const;

  // Constants, filter parameters
  const double _phi;
  const double _psi;
  const uint32_t _alphaCountMax;
  const double _thetaLow;
  const int32_t _numStdDevDelayOutlier;
  const int32_t _numStdDevFrameSizeOutlier;
  const double _noiseStdDevs;
  const double _noiseStdDevOffset;

  double _thetaCov[2][2];  // Estimate covariance
  double _Qcov[2][2];      // Process noise covariance
  double _avgFrameSize;    // Average frame size
  double _varFrameSize;    // Frame size variance
  double _maxFrameSize;    // Largest frame size received (descending
                           // with a factor _psi)
  uint32_t _fsSum;
  uint32_t _fsCount;

  int64_t _lastUpdateT;
  double _prevEstimate;     // The previously returned jitter estimate
  uint32_t _prevFrameSize;  // Frame size of the previous frame
  double _avgNoise;         // Average of the random jitter
  uint32_t _alphaCount;
  double _filterJitterEstimate;  // The filtered sum of jitter estimates

  uint32_t _startupCount;

  //VCMRttFilter _rttFilter;

  RollingAccumulator<uint64_t> fps_counter_;
  enum ExperimentFlag { kInit, kEnabled, kDisabled };
  ExperimentFlag low_rate_experiment_;
  
  VCMJitterEstimator& operator=(const VCMJitterEstimator& rhs);
};

enum { kStartupDelaySamples = 30 };
enum { kFsAccuStartupSamples = 5 };
enum { kMaxFramerateEstimate = 200 };

VCMJitterEstimator::VCMJitterEstimator()
    : _phi(0.97),
      _psi(0.9999),
      _alphaCountMax(400),
      _thetaLow(0.000001),
      _numStdDevDelayOutlier(15),
      _numStdDevFrameSizeOutlier(3),
      _noiseStdDevs(2.33),       // ~Less than 1% chance
                                 // (look up in normal distribution table)...
      _noiseStdDevOffset(30.0),  // ...of getting 30 ms freezes
      //_rttFilter(),
      fps_counter_(30),  // TODO(sprang): Use an estimator with limit based on
                         // time, rather than number of samples.
      low_rate_experiment_(kInit) {
  Reset();
}

VCMJitterEstimator::~VCMJitterEstimator() {}

// Resets the JitterEstimate
void VCMJitterEstimator::Reset() {
  _theta[0] = 1 / (512e3 / 8);
  _theta[1] = 0;
  _varNoise = 4.0;

  _thetaCov[0][0] = 1e-4;
  _thetaCov[1][1] = 1e2;
  _thetaCov[0][1] = _thetaCov[1][0] = 0;
  _Qcov[0][0] = 2.5e-10;
  _Qcov[1][1] = 1e-10;
  _Qcov[0][1] = _Qcov[1][0] = 0;
  _avgFrameSize = 500;
  _maxFrameSize = 500;
  _varFrameSize = 100;
  _lastUpdateT = -1;
  _prevEstimate = -1.0;
  _prevFrameSize = 0;
  _avgNoise = 0.0;
  _alphaCount = 1;
  _filterJitterEstimate = 0.0;
  _fsSum = 0;
  _fsCount = 0;
  _startupCount = 0;
  //_rttFilter.Reset();
  fps_counter_.Reset();
}

// Updates the estimates with the new measurements
void VCMJitterEstimator::UpdateEstimate(int64_t frameDelayMS,
                                        uint32_t frameSizeBytes,
                                        bool incompleteFrame /* = false */) {
  if (frameSizeBytes == 0) {
    return;
  }
  int deltaFS = frameSizeBytes - _prevFrameSize;

  sim_debug("UpdateEstimate frame_delay=%lld deltaFS=%d \n", frameDelayMS, deltaFS);

  // calculate first 5 frames for getting the _avgFrameSize quickly
  if (_fsCount < kFsAccuStartupSamples) {
    _fsSum += frameSizeBytes;
    _fsCount++;
  } else if (_fsCount == kFsAccuStartupSamples) {
    // Give the frame size filter
    _avgFrameSize = static_cast<double>(_fsSum) / static_cast<double>(_fsCount);
    _fsCount++;
  }
  if (!incompleteFrame || frameSizeBytes > _avgFrameSize) {
    double avgFrameSize = _phi * _avgFrameSize + (1 - _phi) * frameSizeBytes;
    // 68-95-99.7
    if (frameSizeBytes < _avgFrameSize + 2 * sqrt(_varFrameSize)) {
      // Only update the average frame size if this sample wasn't a key frame
      _avgFrameSize = avgFrameSize;
      sim_debug("UpdateEstimate _avgFrameSize=%lf frameSizeBytes=%u \n", _avgFrameSize, frameSizeBytes);
    }
    // Update the variance anyway since we want to capture cases where we only
    // get key frames.
    _varFrameSize = std::max<double>(
        _phi * _varFrameSize + (1 - _phi) * (frameSizeBytes - avgFrameSize) * (frameSizeBytes - avgFrameSize), 1.0);
  }

  // Update max frameSize estimate, fast up, slow down
  _maxFrameSize =
      std::max<double>(_psi * _maxFrameSize, static_cast<double>(frameSizeBytes));

  if (_prevFrameSize == 0) {
    _prevFrameSize = frameSizeBytes;
    return;
  }
  _prevFrameSize = frameSizeBytes;

  // Only update the Kalman filter if the sample is not considered
  // an extreme outlier. Even if it is an extreme outlier from a
  // delay point of view, if the frame size also is large the
  // deviation is probably due to an incorrect line slope.
  double deviation = DeviationFromExpectedDelay(frameDelayMS, deltaFS);

  if (fabs(deviation) < _numStdDevDelayOutlier * sqrt(_varNoise) ||
      frameSizeBytes > _avgFrameSize + _numStdDevFrameSizeOutlier * sqrt(_varFrameSize)) {
    // Update the variance of the deviation from the
    // line given by the Kalman filter
    EstimateRandomJitter(deviation, incompleteFrame);
    // Prevent updating with frames which have been congested by a large
    // frame, and therefore arrives almost at the same time as that frame.
    // This can occur when we receive a large frame (key frame) which
    // has been delayed. The next frame is of normal size (delta frame),
    // and thus deltaFS will be << 0. This removes all frame samples
    // which arrives after a key frame.
    if ((!incompleteFrame || deviation >= 0.0) &&
        static_cast<double>(deltaFS) > -0.25 * _maxFrameSize) {
      // Update the Kalman filter with the new data
      KalmanEstimateChannel(frameDelayMS, deltaFS);
    }
  } else { //outlier
    int nStdDev = (deviation >= 0) ? _numStdDevDelayOutlier : -_numStdDevDelayOutlier;
    EstimateRandomJitter(nStdDev * sqrt(_varNoise), incompleteFrame);
  }
  // Post process the total estimated jitter
  if (_startupCount >= kStartupDelaySamples) {
    PostProcessEstimate();
  } else {
    _startupCount++;
  }
}

// Updates Kalman estimate of the channel
// The caller is expected to sanity check the inputs.
void VCMJitterEstimator::KalmanEstimateChannel(int64_t frameDelayMS,
                                               int32_t deltaFSBytes) {
  double Mh[2];
  double hMh_sigma;
  double kalmanGain[2];
  double measureRes;
  double t00, t01;

  // Kalman filtering

  // Prediction
  // M = M + Q
  _thetaCov[0][0] += _Qcov[0][0];
  _thetaCov[0][1] += _Qcov[0][1];
  _thetaCov[1][0] += _Qcov[1][0];
  _thetaCov[1][1] += _Qcov[1][1];

  // Kalman gain
  // K = M*h'/(sigma2n + h*M*h') = M*h'/(1 + h*M*h')
  // h = [dFS 1]
  // Mh = M*h'
  // hMh_sigma = h*M*h' + R
  Mh[0] = _thetaCov[0][0] * deltaFSBytes + _thetaCov[0][1];
  Mh[1] = _thetaCov[1][0] * deltaFSBytes + _thetaCov[1][1];
  // sigma weights measurements with a small deltaFS as noisy and
  // measurements with large deltaFS as good
  if (_maxFrameSize < 1.0) {
    return;
  }
  double sigma = (300.0 * exp(-fabs(static_cast<double>(deltaFSBytes)) /
                              (1e0 * _maxFrameSize)) +
                  1) *
                 sqrt(_varNoise);
  if (sigma < 1.0) {
    sigma = 1.0;
  }
  hMh_sigma = deltaFSBytes * Mh[0] + Mh[1] + sigma;
  if ((hMh_sigma < 1e-9 && hMh_sigma >= 0) ||
      (hMh_sigma > -1e-9 && hMh_sigma <= 0)) {
    assert(false);
    return;
  }
  kalmanGain[0] = Mh[0] / hMh_sigma;
  kalmanGain[1] = Mh[1] / hMh_sigma;

  // Correction
  // theta = theta + K*(dT - h*theta)
  measureRes = frameDelayMS - (deltaFSBytes * _theta[0] + _theta[1]);
  _theta[0] += kalmanGain[0] * measureRes;
  _theta[1] += kalmanGain[1] * measureRes;

  if (_theta[0] < _thetaLow) {
    _theta[0] = _thetaLow;
  }

  // M = (I - K*h)*M
  t00 = _thetaCov[0][0];
  t01 = _thetaCov[0][1];
  _thetaCov[0][0] = (1 - kalmanGain[0] * deltaFSBytes) * t00 -
                    kalmanGain[0] * _thetaCov[1][0];
  _thetaCov[0][1] = (1 - kalmanGain[0] * deltaFSBytes) * t01 -
                    kalmanGain[0] * _thetaCov[1][1];
  _thetaCov[1][0] = _thetaCov[1][0] * (1 - kalmanGain[1]) -
                    kalmanGain[1] * deltaFSBytes * t00;
  _thetaCov[1][1] = _thetaCov[1][1] * (1 - kalmanGain[1]) -
                    kalmanGain[1] * deltaFSBytes * t01;

  // Covariance matrix, must be positive semi-definite
  assert(_thetaCov[0][0] + _thetaCov[1][1] >= 0 &&
         _thetaCov[0][0] * _thetaCov[1][1] -
                 _thetaCov[0][1] * _thetaCov[1][0] >=
             0 &&
         _thetaCov[0][0] >= 0);
}

// Calculate difference in delay between a sample and the
// expected delay estimated by the Kalman filter
double VCMJitterEstimator::DeviationFromExpectedDelay(
    int64_t frameDelayMS,
    int32_t deltaFSBytes) const {
  return frameDelayMS - (_theta[0] * deltaFSBytes + _theta[1]);
}

// Estimates the random jitter by calculating the variance of the
// sample distance from the line given by theta.
void VCMJitterEstimator::EstimateRandomJitter(double d_dT,
                                              bool incompleteFrame) {
  uint64_t now = GET_SYS_MS() * 1000;
  if (_lastUpdateT != -1) {
    fps_counter_.AddSample(now - _lastUpdateT);
  }
  _lastUpdateT = now;

  if (_alphaCount == 0) {
    assert(false);
    return;
  }
  double alpha =
      static_cast<double>(_alphaCount - 1) / static_cast<double>(_alphaCount);
  _alphaCount++;
  if (_alphaCount > _alphaCountMax)
    _alphaCount = _alphaCountMax;

  if (LowRateExperimentEnabled()) {
    // In order to avoid a low frame rate stream to react slower to changes,
    // scale the alpha weight relative a 30 fps stream.
    double fps = GetFrameRate();
    if (fps > 0.0) {
      double rate_scale = 30.0 / fps;
      // At startup, there can be a lot of noise in the fps estimate.
      // Interpolate rate_scale linearly, from 1.0 at sample #1, to 30.0 / fps
      // at sample #kStartupDelaySamples.
      if (_alphaCount < kStartupDelaySamples) {
        rate_scale =
            (_alphaCount * rate_scale + (kStartupDelaySamples - _alphaCount)) /
            kStartupDelaySamples;
      }
      alpha = pow(alpha, rate_scale);
    }
  }

  double avgNoise = alpha * _avgNoise + (1 - alpha) * d_dT;
  double varNoise = alpha * _varNoise + (1 - alpha) * (d_dT - _avgNoise) * (d_dT - _avgNoise);
  if (!incompleteFrame || varNoise > _varNoise) {
    _avgNoise = avgNoise;
    _varNoise = varNoise;
  }
  if (_varNoise < 1.0) {
    // The variance should never be zero, since we might get
    // stuck and consider all samples as outliers.
    _varNoise = 1.0;
  }
}
// 2.33 is normal distribution 99%
double VCMJitterEstimator::NoiseThreshold() const {
  double noiseThreshold = _noiseStdDevs * sqrt(_varNoise) - _noiseStdDevOffset;
  if (noiseThreshold < 1.0) {
    noiseThreshold = 1.0;
  }
  return noiseThreshold;
}

// Calculates the current jitter estimate from the filtered estimates
double VCMJitterEstimator::CalculateEstimate() {
  double ret = _theta[0] * (_maxFrameSize - _avgFrameSize) + NoiseThreshold();

  // A very low estimate (or negative) is neglected
  if (ret < 1.0) {
    if (_prevEstimate <= 0.01) {
      ret = 1.0;
    } else {
      ret = _prevEstimate;
    }
  }
  if (ret > 10000.0) {  // Sanity
    ret = 10000.0;
  }
  _prevEstimate = ret;
  return ret;
}

void VCMJitterEstimator::PostProcessEstimate() {
  _filterJitterEstimate = CalculateEstimate();
}

// Returns the current filtered estimate if available,
// otherwise tries to calculate an estimate.
int VCMJitterEstimator::GetJitterEstimate(double /*rttMultiplier*/) {
  double jitterMS = CalculateEstimate() + OPERATING_SYSTEM_JITTER;
  if (_filterJitterEstimate > jitterMS)
    jitterMS = _filterJitterEstimate;

  if (LowRateExperimentEnabled()) {
    static const double kJitterScaleLowThreshold = 5.0;
    static const double kJitterScaleHighThreshold = 10.0;
    double fps = GetFrameRate();
    // Ignore jitter for very low fps streams.
    if (fps < kJitterScaleLowThreshold) {
      if (fps == 0.0) {
        return static_cast<int>(jitterMS);
      }
      return 0;
    }

    // Semi-low frame rate; scale by factor linearly interpolated from 0.0 at
    // kJitterScaleLowThreshold to 1.0 at kJitterScaleHighThreshold.
    if (fps < kJitterScaleHighThreshold) {
      jitterMS =
          (1.0 / (kJitterScaleHighThreshold - kJitterScaleLowThreshold)) *
          (fps - kJitterScaleLowThreshold) * jitterMS;
    }
  }

  return static_cast<uint32_t>(jitterMS + 0.5);
}

bool VCMJitterEstimator::LowRateExperimentEnabled() {
  if (low_rate_experiment_ == kInit) {
	std::string group = "Disabled";
    if (group == "Disabled") {
      low_rate_experiment_ = kDisabled;
    } else {
      low_rate_experiment_ = kEnabled;
    }
  }
  return low_rate_experiment_ == kEnabled ? true : false;
}

double VCMJitterEstimator::GetFrameRate() const {
  if (fps_counter_.ComputeMean() == 0.0)
    return 0;

  double fps = 1000000.0 / fps_counter_.ComputeMean();
  // Sanity check.
  assert(fps >= 0.0);
  if (fps > kMaxFramerateEstimate) {
    fps = kMaxFramerateEstimate;
  }
  return fps;
}

///////////////////////////////////////////////////////
// VCMJitterBuffer
JitterWrapper::JitterWrapper()
  : jitter_estimate_(new VCMJitterEstimator),
    inter_frame_delay_(new VCMInterFrameDelay(GET_SYS_MS())) {
}

JitterWrapper::~JitterWrapper() {
  delete jitter_estimate_;
  delete inter_frame_delay_;
}

void JitterWrapper::SetNackMode(VCMVideoProtection mode) {
  protection_mode_ = mode;
}

uint32_t JitterWrapper::EstimatedJitterMs() {
  float rtt_mult = protection_mode_ == kProtectionNackFEC ? 0.0 : 1.0;
  return jitter_estimate_->GetJitterEstimate(rtt_mult);
}

// Should never be called with retransmitted frames, 
// they must be filtered out before this function is called.
void JitterWrapper::UpdateJitterEstimate(int64_t latest_packet_time_ms,
                                         uint32_t timestamp,
                                         unsigned int frame_size) {
  if (latest_packet_time_ms == -1) {
    return;
  }
  int64_t frame_delay;
  // Filter out frames which have been reordered in time by the network
  if (inter_frame_delay_->CalculateDelay(timestamp, &frame_delay, latest_packet_time_ms)) {
    // Update the jitter estimate with the new samples
    jitter_estimate_->UpdateEstimate(frame_delay, frame_size);
  }
}
