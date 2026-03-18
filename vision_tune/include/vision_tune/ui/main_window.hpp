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

QT_BEGIN_NAMESPACE
namespace Ui
{
  class MainWindow;
}
QT_END_NAMESPACE

struct HSVRange
{
  int h_low = 0;
  int h_high = 0;
  int s_low = 0;
  int s_high = 0;
  int v_low = 0;
  int v_high = 0;
};

struct HSVConfig
{
  HSVRange red;
  HSVRange blue;
  HSVRange line;
};

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

  const HSVConfig &get_hsv_config() const;
  HSVConfig &get_hsv_config_mutable();
  VisionTarget get_current_target() const;

  void update_result(const vision_tune::msg::ProcessResult &msg);
  bool save_yaml(const QString &file_path);
  bool load_yaml(const QString &file_path);

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
  void apply_pending_ui();

private:
  HSVRange &current_hsv();
  const HSVRange &current_hsv() const;

  static void set_range_to_zero(HSVRange &range);
  void queue_value_change(int h_low, int h_high, int s_low, int s_high, int v_low, int v_high);
  void queue_current_target_to_widgets();
  void queue_full_config_to_widgets();

  Ui::MainWindow *ui;
  QLabel *get_current_result_label();

  HSVConfig hsv_config_{};
  VisionTarget current_target_ = VisionTarget::red;

  HSVConfig pending_widget_config_{};
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
};

#endif