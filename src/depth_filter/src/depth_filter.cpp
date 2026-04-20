#include "depth_filter/depth_filter.hpp"

DepthFilter::DepthFilter()
{
}

void DepthFilter::filter(double pressure)
{
  // TODO: implement filtering logic here
  pressure_m_ = pressure;
}
