#include "vision_tune/ui/ui_node.hpp"
#include "vision_tune/ui/main_window.hpp"

#include <QApplication>
#include <QTimer>
#include <QImage>
#include <QPixmap>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

UiNode::UiNode(MainWindow *window)
    : QObject(),
    rclcpp::Node("ui_node"),
    window_(window)
{
    tuning_pub_ = this->create_publisher<vision_tune::msg::TuningValue>(
        "/vision/tuning", 10);

    result_sub_ = this->create_subscription<vision_tune::msg::ProcessResult>(
        "/vision/result",
        10,
        std::bind(&UiNode::resultCallback, this, std::placeholders::_1));

    raw_image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera1/camera/image_raw",
        10,
        std::bind(&UiNode::rawImageCallback, this, std::placeholders::_1));

    result_image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/vision/result_image",
        10,
        std::bind(&UiNode::resultImageCallback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(33),
        std::bind(&UiNode::publishTuning, this));
}

void UiNode::publishTuning()
{
    const HSVConfig &cfg = window_->getHSVConfig();

    vision_tune::msg::TuningValue msg;

    msg.red_h_low = cfg.red.h_low;
    msg.red_h_high = cfg.red.h_high;
    msg.red_s_low = cfg.red.s_low;
    msg.red_s_high = cfg.red.s_high;
    msg.red_v_low = cfg.red.v_low;
    msg.red_v_high = cfg.red.v_high;

    msg.blue_h_low = cfg.blue.h_low;
    msg.blue_h_high = cfg.blue.h_high;
    msg.blue_s_low = cfg.blue.s_low;
    msg.blue_s_high = cfg.blue.s_high;
    msg.blue_v_low = cfg.blue.v_low;
    msg.blue_v_high = cfg.blue.v_high;

    msg.line_h_low = cfg.line.h_low;
    msg.line_h_high = cfg.line.h_high;
    msg.line_s_low = cfg.line.s_low;
    msg.line_s_high = cfg.line.s_high;
    msg.line_v_low = cfg.line.v_low;
    msg.line_v_high = cfg.line.v_high;

    tuning_pub_->publish(msg);
}

void UiNode::resultCallback(const vision_tune::msg::ProcessResult::SharedPtr msg)
{
    window_->updateResult(*msg);
}

void UiNode::rawImageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    try
    {
        cv::Mat frame = cv_bridge::toCvCopy(msg, "bgr8")->image;

        QImage qimg(
            frame.data,
            frame.cols,
            frame.rows,
            static_cast<int>(frame.step),
            QImage::Format_BGR888);

        QPixmap pixmap = QPixmap::fromImage(qimg.copy());
        emit rawImageReceived(pixmap);
    }
    catch (const cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "raw image cv_bridge exception: %s", e.what());
    }
}

void UiNode::resultImageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    try
    {
        cv::Mat frame = cv_bridge::toCvCopy(msg, "bgr8")->image;

        QImage qimg(
            frame.data,
            frame.cols,
            frame.rows,
            static_cast<int>(frame.step),
            QImage::Format_BGR888);

        QPixmap pixmap = QPixmap::fromImage(qimg.copy());
        emit resultImageReceived(pixmap);
    }
    catch (const cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "result image cv_bridge exception: %s", e.what());
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    QApplication app(argc, argv);

    MainWindow window;
    window.show();

    auto node = std::make_shared<UiNode>(&window);

    QObject::connect(node.get(), &UiNode::rawImageReceived,
                    &window, &MainWindow::updateRawImage);

    QObject::connect(node.get(), &UiNode::resultImageReceived,
                    &window, &MainWindow::updateResultImage);

    QTimer ros_timer;
    QObject::connect(&ros_timer, &QTimer::timeout, [&]()
                    {
        if (rclcpp::ok()) {
            rclcpp::spin_some(node);
        } });
    ros_timer.start(10);

    QObject::connect(&app, &QApplication::aboutToQuit, [&]()
                    { ros_timer.stop(); });

    int ret = app.exec();

    if (rclcpp::ok())
    {
        rclcpp::shutdown();
    }

    return ret;
}