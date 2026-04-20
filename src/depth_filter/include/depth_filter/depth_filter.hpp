#ifndef DEPTH_FILTER_DEPTH_FILTER_HPP_
#define DEPTH_FILTER_DEPTH_FILTER_HPP_

class DepthFilter
{
public:
  DepthFilter();

  void filter(double pressure);

  double get_filtered_depth() const { return depth_f_; }
  double get_filtered_speed() const { return speed_f_; }

private:
  double pressure_m_ = 0.0;  /// Raw measured pressure
  double pressure_f_ = 0.0;  /// Filtered pressure
  double depth_m_ = 0.0;  /// Depth calculated from raw pressure
  double depth_f_ = 0.0;  /// Filtered depth
  double speed_f_ = 0.0;  /// Filtered vertical speed
};

#endif  // DEPTH_FILTER_DEPTH_FILTER_HPP_