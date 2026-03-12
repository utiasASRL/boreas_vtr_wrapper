// Utils for radar that we don't want to include in other packages

# pragma once

#include <cstdint>
#include <opencv2/core.hpp>

namespace vtr {
namespace testing {

inline void load_radar_time_span(const cv::Mat &raw_data, int64_t &start_time, int64_t &final_time) {
  const uint N = raw_data.rows;  
  start_time = *((int64_t *)(raw_data.ptr<uchar>(0))) * 1000;
  final_time = *((int64_t *)(raw_data.ptr<uchar>(N - 1))) * 1000;
}

} // namespace testing
} // namespace vtr