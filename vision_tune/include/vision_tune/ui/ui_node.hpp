#ifndef VISION_TUNE_UI_NODE_HPP_
#define VISION_TUNE_UI_NODE_HPP_

#include <QObject>
#include <QPixmap>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vision_tune/msg/process_result.hpp>
#include <vision_tune/msg/tuning_value.hpp>

class MainWindow;

class UiNode : public QObject, public rclcpp::Node
{
    Q_OBJECT

public:
    explicit UiNode(MainWindow *window);

Q_SIGNALS:
    void rawImageReceived(const QPixmap &pixmap);
    void resultImageReceived(const QPixmap &pixmap);

private:
    void publishTuning();
    void resultCallback(const vision_tune::msg::ProcessResult::SharedPtr msg);
    void rawImageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    void resultImageCallback(const sensor_msgs::msg::Image::SharedPtr msg);

    MainWindow *window_;

    rclcpp::Publisher<vision_tune::msg::TuningValue>::SharedPtr tuning_pub_;

    rclcpp::Subscription<vision_tune::msg::ProcessResult>::SharedPtr result_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr raw_image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr result_image_sub_;

    rclcpp::TimerBase::SharedPtr timer_;
};

#endif