#ifndef VISION_TUNE_MAIN_WINDOW_HPP_
#define VISION_TUNE_MAIN_WINDOW_HPP_

#include <QMainWindow>
#include <QFileDialog>
#include <QMessageBox>
#include <QPixmap>
#include <QTimer>
#include <QLabel>
#include <yaml-cpp/yaml.h>

#include <fstream>
#include "vision_tune/msg/process_result.hpp"
#include "vision_tune/utils/config_utils.hpp"

QT_BEGIN_NAMESPACE
namespace Ui
{
  class MainWindow;
}
QT_END_NAMESPACE

enum class VisionTarget
{
  red = 0,
  blue = 1,
  line = 2
};

class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
  explicit MainWindow(QWidget *parent = nullptr);
  ~MainWindow();

  const vision_tune::utils::hsv_config &get_hsv_config() const;
  vision_tune::utils::hsv_config &get_hsv_config_mutable();
  VisionTarget get_current_target() const;

  void update_result(const vision_tune::msg::ProcessResult &msg);
  bool save_yaml(const QString &file_path);
  bool load_yaml(const QString &file_path);
  void set_ui_update_hz(double hz);

public Q_SLOTS:
  void on_spinBox_h_low_valueChanged(int value);
  void on_spinBox_s_low_valueChanged(int value);
  void on_spinBox_v_low_valueChanged(int value);
  void on_spinBox_h_high_valueChanged(int value);
  void on_spinBox_s_high_valueChanged(int value);
  void on_spinBox_v_high_valueChanged(int value);

  void on_slider_h_low_valueChanged(int value);
  void on_slider_s_low_valueChanged(int value);
  void on_slider_v_low_valueChanged(int value);
  void on_slider_h_high_valueChanged(int value);
  void on_slider_s_high_valueChanged(int value);
  void on_slider_v_high_valueChanged(int value);

  void on_pushButton_save_clicked();
  void on_pushButton_load_clicked();
  void on_pushButton_set0_clicked();

  void on_radioButton_red_clicked();
  void on_radioButton_blue_clicked();
  void on_radioButton_line_clicked();

  void update_raw_image(const QPixmap &pixmap);
  void update_result_image(const QPixmap &pixmap);
  void update_bird_image(const QPixmap &pixmap);

private Q_SLOTS:
  void apply_ui_tick();

private:
  vision_tune::utils::hsv_range &current_hsv();
  const vision_tune::utils::hsv_range &current_hsv() const;

  void set_range_to_zero(vision_tune::utils::hsv_range &range);
  void queue_value_change(int h_low, int h_high, int s_low, int s_high, int v_low, int v_high);
  void queue_current_target_to_widgets();
  void queue_full_config_to_widgets();

  Ui::MainWindow *ui;
  QLabel *get_current_result_label();

  vision_tune::utils::hsv_config hsv_config_{};
  vision_tune::utils::hsv_config pending_widget_config_{};
  VisionTarget current_target_ = VisionTarget::red;
  VisionTarget pending_widget_target_ = VisionTarget::red;
  bool pending_widget_dirty_ = false;
  bool suppress_ui_sync_ = false;

  QPixmap pending_bird_pixmap_{};
  QPixmap pending_raw_pixmap_{};
  QPixmap pending_result_pixmap_{};
  vision_tune::msg::ProcessResult pending_result_msg_{};
  bool pending_raw_dirty_ = false;
  bool pending_result_image_dirty_ = false;
  bool pending_result_msg_dirty_ = false;
  bool pending_bird_image_dirty_ = false;

  QTimer *ui_apply_timer_ = nullptr;
  double ui_update_hz_ = 15.0;
};

#endif