#ifndef VISION_TUNE_UTILS_COMMON_UTILS_HPP_
#define VISION_TUNE_UTILS_COMMON_UTILS_HPP_

#include <chrono>

namespace vision_tune::utils
{

  inline std::chrono::nanoseconds hz_to_period(double hz) //ros2 timer용
  {
    if (hz <= 0.0)
    {
      return std::chrono::milliseconds(100);
    }

    return std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / hz));
  }

  inline int hz_to_msec(double hz) //qt timer용
  {
    if (hz <= 0.0)
    {
      return 100;
    }

    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(1.0 / hz));

    int msec = static_cast<int>(duration.count());

    if (msec <= 0)
    {
      return 1;
    }

    return msec;
  }

  template <typename T>
  inline T clamp_value(T value, T min_value, T max_value)
  {
    if (value < min_value)
    {
      return min_value;
    }
    if (value > max_value)
    {
      return max_value;
    }
    return value;
  }

} // namespace vision_tune::utils

#endif