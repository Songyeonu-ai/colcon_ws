#include "vision_tune/utils/vision_utils.hpp"

namespace vision_tune::utils
{

  cv::Mat make_bird_view(
      const cv::Mat &frame,
      const std::vector<cv::Point2f> &src_points,
      int output_width,
      int output_height)
  {
    if (frame.empty() || src_points.size() != 4)
    {
      return cv::Mat();
    }

    std::vector<cv::Point2f> dst_points = {
        {0.0f, 0.0f},
        {static_cast<float>(output_width - 1), 0.0f},
        {0.0f, static_cast<float>(output_height - 1)},
        {static_cast<float>(output_width - 1), static_cast<float>(output_height - 1)}};

    cv::Mat transform_matrix = cv::getPerspectiveTransform(src_points, dst_points);

    cv::Mat bird_view;
    cv::warpPerspective(
        frame,
        bird_view,
        transform_matrix,
        cv::Size(output_width, output_height));

    return bird_view;
  }

  cv::Mat make_hsv_mask(const cv::Mat &bgr_image, const hsv_range &range)
  {
    if (bgr_image.empty())
    {
      return cv::Mat();
    }

    cv::Mat hsv_image;
    cv::cvtColor(bgr_image, hsv_image, cv::COLOR_BGR2HSV);

    cv::Mat mask;
    cv::inRange(
        hsv_image,
        cv::Scalar(range.h_low, range.s_low, range.v_low),
        cv::Scalar(range.h_high, range.s_high, range.v_high),
        mask);

    return mask;
  }

  detection_result find_largest_blob(const cv::Mat &binary_image)
  {
    detection_result result;

    if (binary_image.empty())
    {
      return result;
    }

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(
        binary_image,
        contours,
        cv::RETR_EXTERNAL,
        cv::CHAIN_APPROX_SIMPLE);

    double max_area = 0.0;
    cv::Rect best_rect;

    for (const auto &contour : contours)
    {
      double area = cv::contourArea(contour);
      if (area > max_area)
      {
        max_area = area;
        best_rect = cv::boundingRect(contour);
      }
    }

    if (max_area > 0.0)
    {
      result.detected = true;
      result.area = static_cast<float>(max_area);
      result.bbox = best_rect;
      result.center_x = best_rect.x + best_rect.width * 0.5f;
      result.center_y = best_rect.y + best_rect.height * 0.5f;
    }

    return result;
  }

  cv::Mat draw_detection_result(
      const cv::Mat &base_image,
      const detection_result &result)
  {
    if (base_image.empty())
    {
      return cv::Mat();
    }

    cv::Mat output = base_image.clone();

    if (result.detected)
    {
      cv::rectangle(output, result.bbox, cv::Scalar(0, 255, 0), 2);
      cv::circle(
          output,
          cv::Point(
              static_cast<int>(result.center_x),
              static_cast<int>(result.center_y)),
          5,
          cv::Scalar(0, 0, 255),
          -1);
    }

    return output;
  }

} // namespace vision_tune::utils