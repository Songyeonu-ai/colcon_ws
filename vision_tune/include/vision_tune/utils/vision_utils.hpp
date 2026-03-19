#ifndef VISION_TUNE_UTILS_VISION_UTILS_HPP_
#define VISION_TUNE_UTILS_VISION_UTILS_HPP_

#include <vector>
#include <opencv2/opencv.hpp>

#include "vision_tune/utils/config_utils.hpp"

namespace vision_tune::utils
{

  struct detection_result
  {
    bool detected = false;
    float center_x = 0.0f;
    float center_y = 0.0f;
    float area = 0.0f;
    cv::Rect bbox;
  };

  cv::Mat make_bird_view(
      const cv::Mat &frame,
      const std::vector<cv::Point2f> &src_points,
      int output_width,
      int output_height);

  cv::Mat make_hsv_mask(
      const cv::Mat &bgr_image,
      const hsv_range &range);

  detection_result find_largest_blob(const cv::Mat &binary_image);

  cv::Mat draw_detection_result(
      const cv::Mat &base_image,
      const detection_result &result);

} // namespace vision_tune::utils

#endif