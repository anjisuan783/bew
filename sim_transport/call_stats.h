#pragma once 

#include <list>
#include <memory>

// CallStats keeps track of statistics for a call.
class CallStats {
 public:
  CallStats();
  ~CallStats();

  void reset();

  // Helper struct keeping track of the time a rtt value is reported.
  struct RttTime {
    RttTime(int64_t new_rtt, int64_t rtt_time)
        : rtt(new_rtt), time(rtt_time) {}
    const int64_t rtt;
    const int64_t time;
  };

  void OnRttUpdate(int64_t rtt, int64_t now_ms);
  int64_t avg_rtt_ms();
  int64_t max_rtt_ms();

 protected:
  void Process(int64_t now);

 private:
  // The last time 'Process' resulted in statistic update.
  int64_t last_process_time_;
  // The last RTT in the statistics update (zero if there is no valid estimate).
  int64_t max_rtt_ms_;
  int64_t avg_rtt_ms_;
  int64_t sum_avg_rtt_ms_;
  int64_t num_avg_rtt_;
  int64_t time_of_first_rtt_ms_;

  // All Rtt reports within valid time interval, oldest first.
  std::list<RttTime> reports_;
};
