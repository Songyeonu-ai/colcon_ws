#include "vision_tune/ui/main_window.hpp"
#include "vision_tune/utils/common_utils.hpp"
#include "ui_main_window.h"

#include <QDir>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent),
      ui(new Ui::MainWindow),
      pending_widget_config_(hsv_config_),
      pending_widget_target_(current_target_)
{
  ui->setupUi(this);

  ui_apply_timer_ = new QTimer(this);
  connect(ui_apply_timer_, &QTimer::timeout,
          this, &MainWindow::apply_ui_tick);

  set_ui_update_hz(15.0);

  queue_full_config_to_widgets();
  apply_ui_tick();
}

MainWindow::~MainWindow()
{
  delete ui;
}

void MainWindow::set_ui_update_hz(double hz)
{
  ui_update_hz_ = hz;

  if (ui_apply_timer_ == nullptr)
  {
    return;
  }

  ui_apply_timer_->stop();
  ui_apply_timer_->start(vision_tune::utils::hz_to_msec(ui_update_hz_));
}

//=========================UI 업데이트 관련=========================
void MainWindow::set_range_to_zero(vision_tune::utils::hsv_range &range)
{
  range = vision_tune::utils::hsv_range{};
}

void MainWindow::queue_value_change(int h_low, int h_high, int s_low, int s_high, int v_low, int v_high)
{
  vision_tune::utils::hsv_range &hsv = current_hsv();
  hsv.h_low = h_low;
  hsv.h_high = h_high;
  hsv.s_low = s_low;
  hsv.s_high = s_high;
  hsv.v_low = v_low;
  hsv.v_high = v_high;
  queue_current_target_to_widgets();
}

void MainWindow::queue_current_target_to_widgets()
{
  pending_widget_config_ = hsv_config_;
  pending_widget_target_ = current_target_;
  pending_widget_dirty_ = true;
}

void MainWindow::queue_full_config_to_widgets()
{
  pending_widget_config_ = hsv_config_;
  pending_widget_target_ = current_target_;
  pending_widget_dirty_ = true;
}

QLabel *MainWindow::get_current_result_label()
{
  switch (current_target_)
  {
  case VisionTarget::red:
    return ui->label_red;
  case VisionTarget::blue:
    return ui->label_blue;
  case VisionTarget::line:
    return ui->label_line;
  default:
    return ui->label_red;
  }
}
// ui업데이트 tick함수
void MainWindow::apply_ui_tick()
{
  if (pending_widget_dirty_)
  {
    suppress_ui_sync_ = true;

    const vision_tune::utils::hsv_range *hsv = nullptr;
    switch (pending_widget_target_)
    {
    case VisionTarget::red:
      hsv = &pending_widget_config_.red;
      ui->radioButton_red->setChecked(true);
      break;
    case VisionTarget::blue:
      hsv = &pending_widget_config_.blue;
      ui->radioButton_blue->setChecked(true);
      break;
    case VisionTarget::line:
      hsv = &pending_widget_config_.line;
      ui->radioButton_line->setChecked(true);
      break;
    }

    if (hsv != nullptr)
    {
      ui->slider_h_low->setValue(hsv->h_low);
      ui->slider_h_high->setValue(hsv->h_high);
      ui->slider_s_low->setValue(hsv->s_low);
      ui->slider_s_high->setValue(hsv->s_high);
      ui->slider_v_low->setValue(hsv->v_low);
      ui->slider_v_high->setValue(hsv->v_high);

      ui->spinBox_h_low->setValue(hsv->h_low);
      ui->spinBox_h_high->setValue(hsv->h_high);
      ui->spinBox_s_low->setValue(hsv->s_low);
      ui->spinBox_s_high->setValue(hsv->s_high);
      ui->spinBox_v_low->setValue(hsv->v_low);
      ui->spinBox_v_high->setValue(hsv->v_high);
    }

    suppress_ui_sync_ = false;
    pending_widget_dirty_ = false;
  }

  if (pending_raw_dirty_)
  {
    ui->label_raw->setPixmap(
        pending_raw_pixmap_.scaled(
            ui->label_raw->size(),
            Qt::KeepAspectRatio,
            Qt::SmoothTransformation));
    pending_raw_dirty_ = false;
  }

  if (pending_result_image_dirty_)
  {
    QLabel *target_label = get_current_result_label();

    if (target_label != nullptr)
    {
      target_label->setPixmap(
          pending_result_pixmap_.scaled(
              target_label->size(),
              Qt::KeepAspectRatio,
              Qt::SmoothTransformation));
    }

    pending_result_image_dirty_ = false;
  }

  if (pending_bird_image_dirty_)
  {
    ui->label_bird->setPixmap(
        pending_bird_pixmap_.scaled(
            ui->label_bird->size(),
            Qt::KeepAspectRatio,
            Qt::SmoothTransformation));
    pending_bird_image_dirty_ = false;
  }

  if (pending_result_msg_dirty_)
  {
    ui->label_detected->setText(pending_result_msg_.detected ? "true" : "false");
    ui->label_center_x->setText(QString::number(pending_result_msg_.center_x));
    ui->label_center_y->setText(QString::number(pending_result_msg_.center_y));
    ui->label_area->setText(QString::number(pending_result_msg_.area));
    pending_result_msg_dirty_ = false;
  }
}

void MainWindow::update_result(const vision_tune::msg::ProcessResult &msg)
{
  pending_result_msg_ = msg;
  pending_result_msg_dirty_ = true;
}

void MainWindow::update_raw_image(const QPixmap &pixmap)
{
  if (pixmap.isNull())
  {
    return;
  }
  pending_raw_pixmap_ = pixmap;
  pending_raw_dirty_ = true;
}

void MainWindow::update_result_image(const QPixmap &pixmap)
{
  if (pixmap.isNull())
  {
    return;
  }
  pending_result_pixmap_ = pixmap;
  pending_result_image_dirty_ = true;
}

void MainWindow::update_bird_image(const QPixmap &pixmap)
{
  if (pixmap.isNull())
  {
    return;
  }
  pending_bird_pixmap_ = pixmap;
  pending_bird_image_dirty_ = true;
}

//===================파일 저장 함수=======================

bool MainWindow::save_yaml(const QString &file_path)
{
  try
  {
    YAML::Node root;
    YAML::Node params;

    params["red"] = vision_tune::utils::to_yaml(hsv_config_.red);
    params["blue"] = vision_tune::utils::to_yaml(hsv_config_.blue);
    params["line"] = vision_tune::utils::to_yaml(hsv_config_.line);

    root["ui_node"]["ros__parameters"]["hsv"] = params;

    std::ofstream fout(file_path.toStdString());
    fout << root;
    fout.close();
    return true;
  }
  catch (const std::exception &e)
  {
    QMessageBox::critical(this, "save error", e.what());
    return false;
  }
}

bool MainWindow::load_yaml(const QString &file_path)
{
  try
  {
    YAML::Node root = YAML::LoadFile(file_path.toStdString());
    YAML::Node params = root["ui_node"]["ros__parameters"]["hsv"];

    if (!params)
    {
      QMessageBox::warning(this, "load error", "invalid yaml format");
      return false;
    }

    vision_tune::utils::from_yaml(params["red"], hsv_config_.red);
    vision_tune::utils::from_yaml(params["blue"], hsv_config_.blue);
    vision_tune::utils::from_yaml(params["line"], hsv_config_.line);

    queue_full_config_to_widgets();
    return true;
  }
  catch (const std::exception &e)
  {
    QMessageBox::critical(this, "load error", e.what());
    return false;
  }
}
//=========================UI 이벤트 핸들러=========================

void MainWindow::on_spinBox_h_low_valueChanged(int value)
{
  if (suppress_ui_sync_)
    return;
  queue_value_change(value, current_hsv().h_high, current_hsv().s_low, current_hsv().s_high, current_hsv().v_low, current_hsv().v_high);
}

void MainWindow::on_spinBox_s_low_valueChanged(int value)
{
  if (suppress_ui_sync_)
    return;
  queue_value_change(current_hsv().h_low, current_hsv().h_high, value, current_hsv().s_high, current_hsv().v_low, current_hsv().v_high);
}

void MainWindow::on_spinBox_v_low_valueChanged(int value)
{
  if (suppress_ui_sync_)
    return;
  queue_value_change(current_hsv().h_low, current_hsv().h_high, current_hsv().s_low, current_hsv().s_high, value, current_hsv().v_high);
}

void MainWindow::on_spinBox_h_high_valueChanged(int value)
{
  if (suppress_ui_sync_)
    return;
  queue_value_change(current_hsv().h_low, value, current_hsv().s_low, current_hsv().s_high, current_hsv().v_low, current_hsv().v_high);
}

void MainWindow::on_spinBox_s_high_valueChanged(int value)
{
  if (suppress_ui_sync_)
    return;
  queue_value_change(current_hsv().h_low, current_hsv().h_high, current_hsv().s_low, value, current_hsv().v_low, current_hsv().v_high);
}

void MainWindow::on_spinBox_v_high_valueChanged(int value)
{
  if (suppress_ui_sync_)
    return;
  queue_value_change(current_hsv().h_low, current_hsv().h_high, current_hsv().s_low, current_hsv().s_high, current_hsv().v_low, value);
}

void MainWindow::on_slider_h_low_valueChanged(int value)
{
  if (suppress_ui_sync_)
    return;
  queue_value_change(value, current_hsv().h_high, current_hsv().s_low, current_hsv().s_high, current_hsv().v_low, current_hsv().v_high);
}

void MainWindow::on_slider_s_low_valueChanged(int value)
{
  if (suppress_ui_sync_)
    return;
  queue_value_change(current_hsv().h_low, current_hsv().h_high, value, current_hsv().s_high, current_hsv().v_low, current_hsv().v_high);
}

void MainWindow::on_slider_v_low_valueChanged(int value)
{
  if (suppress_ui_sync_)
    return;
  queue_value_change(current_hsv().h_low, current_hsv().h_high, current_hsv().s_low, current_hsv().s_high, value, current_hsv().v_high);
}

void MainWindow::on_slider_h_high_valueChanged(int value)
{
  if (suppress_ui_sync_)
    return;
  queue_value_change(current_hsv().h_low, value, current_hsv().s_low, current_hsv().s_high, current_hsv().v_low, current_hsv().v_high);
}

void MainWindow::on_slider_s_high_valueChanged(int value)
{
  if (suppress_ui_sync_)
    return;
  queue_value_change(current_hsv().h_low, current_hsv().h_high, current_hsv().s_low, value, current_hsv().v_low, current_hsv().v_high);
}

void MainWindow::on_slider_v_high_valueChanged(int value)
{
  if (suppress_ui_sync_)
    return;
  queue_value_change(current_hsv().h_low, current_hsv().h_high, current_hsv().s_low, current_hsv().s_high, current_hsv().v_low, value);
}

void MainWindow::on_radioButton_red_clicked()
{
  current_target_ = VisionTarget::red;
  queue_current_target_to_widgets();
}

void MainWindow::on_radioButton_blue_clicked()
{
  current_target_ = VisionTarget::blue;
  queue_current_target_to_widgets();
}

void MainWindow::on_radioButton_line_clicked()
{
  current_target_ = VisionTarget::line;
  queue_current_target_to_widgets();
}

void MainWindow::on_pushButton_save_clicked()
{
  const QString path = QDir::homePath() + "/ros2_ws/src/vision_tune/config/";
  QDir().mkpath(path);

  QString file_path = QFileDialog::getSaveFileName(
      this,
      "save hsv yaml",
      path + "vision_hsv.yaml",
      "yaml files (*.yaml *.yml)");

  if (file_path.isEmpty())
  {
    return;
  }

  if (!file_path.endsWith(".yaml", Qt::CaseInsensitive) &&
      !file_path.endsWith(".yml", Qt::CaseInsensitive))
  {
    file_path += ".yaml";
  }

  if (save_yaml(file_path))
  {
    QMessageBox::information(this, "save", "yaml saved");
  }
}

void MainWindow::on_pushButton_load_clicked()
{
  const QString path = QDir::homePath() + "/ros2_ws/src/vision_tune/config/";
  QDir().mkpath(path);

  const QString file_path = QFileDialog::getOpenFileName(
      this,
      "load hsv yaml",
      path,
      "yaml files (*.yaml *.yml)");

  if (file_path.isEmpty())
  {
    return;
  }

  if (load_yaml(file_path))
  {
    QMessageBox::information(this, "load", "yaml loaded");
  }
}

void MainWindow::on_pushButton_set0_clicked()
{
  set_range_to_zero(hsv_config_.red);
  set_range_to_zero(hsv_config_.blue);
  set_range_to_zero(hsv_config_.line);
  queue_full_config_to_widgets();
}

//=================hsv관련====================
const vision_tune::utils::hsv_config &MainWindow::get_hsv_config() const
{
  return hsv_config_;
}

VisionTarget MainWindow::get_current_target() const
{
  return current_target_;
}

vision_tune::utils::hsv_range &MainWindow::current_hsv()
{
  switch (current_target_)
  {
  case VisionTarget::red:
    return hsv_config_.red;
  case VisionTarget::blue:
    return hsv_config_.blue;
  case VisionTarget::line:
    return hsv_config_.line;
  }

  return hsv_config_.red;
}

const vision_tune::utils::hsv_range &MainWindow::current_hsv() const
{
  switch (current_target_)
  {
  case VisionTarget::red:
    return hsv_config_.red;
  case VisionTarget::blue:
    return hsv_config_.blue;
  case VisionTarget::line:
    return hsv_config_.line;
  }

  return hsv_config_.red;
}