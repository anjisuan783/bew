#include <algorithm>
#include "call_stats.h"

namespace {
// Time interval for updating the observers.
const int64_t kUpdateIntervalMs = 1000;
// Weight factor to apply to the average rtt.
const float kWeightFactor = 0.3f;

void RemoveOldReports(int64_t now, std::list<CallStats::RttTime>* reports) {
  // A rtt report is considered valid for this long.
  const int64_t kRttTimeoutMs = 1500;
  while (!reports->empty() &&
         (now - reports->front().time) > kRttTimeoutMs) {
    reports->pop_front();
  }
}

int64_t GetMaxRttMs(std::list<CallStats::RttTime>* reports) {
  if (reports->empty())
    return -1;
  int64_t max_rtt_ms = 0;
  for (const CallStats::RttTime& rtt_time : *reports)
    max_rtt_ms = std::max(rtt_time.rtt, max_rtt_ms);
  return max_rtt_ms;
}

int64_t GetAvgRttMs(std::list<CallStats::RttTime>* reports) {
  if (reports->empty()) {
    return -1;
  }
  int64_t sum = 0;
  for (std::list<CallStats::RttTime>::const_iterator it = reports->begin();
       it != reports->end(); ++it) {
    sum += it->rtt;
  }
  return sum / reports->size();
}

void UpdateAvgRttMs(std::list<CallStats::RttTime>* reports, int64_t* avg_rtt) {
  int64_t cur_rtt_ms = GetAvgRttMs(reports);
  if (cur_rtt_ms == -1) {
    // Reset.
    *avg_rtt = -1;
    return;
  }
  if (*avg_rtt == -1) {
    // Initialize.
    *avg_rtt = cur_rtt_ms;
    return;
  }
  *avg_rtt = *avg_rtt * (1.0f - kWeightFactor) + cur_rtt_ms * kWeightFactor;
}
}  // namespace

CallStats::CallStats() {
  reset();
}

CallStats::~CallStats() {
}

void CallStats::reset() {
  last_process_time_ = -1;
  max_rtt_ms_ = -1;
  avg_rtt_ms_ = -1;
  sum_avg_rtt_ms_ = 0;
  num_avg_rtt_ = 0;
  time_of_first_rtt_ms_ = -1;
}

void CallStats::Process(int64_t now) {
  if (now < last_process_time_ + kUpdateIntervalMs)
    return;

  last_process_time_ = now;

  RemoveOldReports(now, &reports_);
  max_rtt_ms_ = GetMaxRttMs(&reports_);
  UpdateAvgRttMs(&reports_, &avg_rtt_ms_);

  // If there is a valid rtt, update all observers with the max rtt.
  if (max_rtt_ms_ >= 0) {
    // Sum for Histogram of average RTT reported over the entire call.
    sum_avg_rtt_ms_ += avg_rtt_ms_;
    ++num_avg_rtt_;
  }
}

int64_t CallStats::avg_rtt_ms() {
  return avg_rtt_ms_;
}

int64_t CallStats::max_rtt_ms() {
  return max_rtt_ms_;
}

void CallStats::OnRttUpdate(int64_t rtt, int64_t now_ms) {
  reports_.push_back(RttTime(rtt, now_ms));
  if (time_of_first_rtt_ms_ == -1)
    time_of_first_rtt_ms_ = now_ms;

  this->Process(now_ms);
}
