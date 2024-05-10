#ifndef __EXP_FILTER_H__
#define __EXP_FILTER_H__

class ExpFilter {
 public:
  static const float kValueUndefined;

  explicit ExpFilter(float alpha, float max = kValueUndefined) : max_(max) {
    Reset(alpha);
  }

  // Resets the filter to its initial state, and resets filter factor base to
  // the given value |alpha|.
  void Reset(float alpha);

  // Applies the filter with a given exponent on the provided sample:
  // y(k) = min(alpha_^ exp * y(k-1) + (1 - alpha_^ exp) * sample, max_).
  float Apply(float exp, float sample);

  // Returns current filtered value.
  float filtered() const { return filtered_; }

  // Changes the filter factor base to the given value |alpha|.
  void UpdateBase(float alpha);

 private:
  float alpha_;     // Filter factor base.
  float filtered_;  // Current filter output.
  const float max_;
};

#endif //!__EXP_FILTER_H__
