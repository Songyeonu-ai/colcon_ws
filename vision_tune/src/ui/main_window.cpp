#include "vision_tune/ui/main_window.hpp"
#include "ui_main_window.h"

#include <QDir>

namespace {
YAML::Node toYaml(const HSVRange &hsv)
{
    YAML::Node node;
    node["h_low"] = hsv.h_low;
    node["h_high"] = hsv.h_high;
    node["s_low"] = hsv.s_low;
    node["s_high"] = hsv.s_high;
    node["v_low"] = hsv.v_low;
    node["v_high"] = hsv.v_high;
    return node;
}

bool fromYaml(const YAML::Node &node, HSVRange &hsv)
{
    if (!node) {
        return false;
    }
    hsv.h_low = node["h_low"].as<int>();
    hsv.h_high = node["h_high"].as<int>();
    hsv.s_low = node["s_low"].as<int>();
    hsv.s_high = node["s_high"].as<int>();
    hsv.v_low = node["v_low"].as<int>();
    hsv.v_high = node["v_high"].as<int>();
    return true;
}
} // namespace

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    syncBufferToUi();
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::updateRawImage(const QPixmap &pixmap)
{
    if (pixmap.isNull()) return;

    ui->label_raw->setPixmap(
        pixmap.scaled(
            ui->label_raw->size(),
            Qt::KeepAspectRatio,
            Qt::SmoothTransformation
        )
    );
}

void MainWindow::updateResultImage(const QPixmap &pixmap)
{
    if (pixmap.isNull()) return;

    ui->label_red->setPixmap(
        pixmap.scaled(
            ui->label_red->size(),
            Qt::KeepAspectRatio,
            Qt::SmoothTransformation
        )
    );
}

const HSVConfig &MainWindow::getHSVConfig() const
{
    return hsv_config_;
}

HSVConfig &MainWindow::getHSVConfigMutable()
{
    return hsv_config_;
}

VisionTarget MainWindow::getCurrentTarget() const
{
    return current_target_;
}

HSVRange &MainWindow::currentHSV()
{
    switch (current_target_) {
    case VisionTarget::Red:
        return hsv_config_.red;
    case VisionTarget::Blue:
        return hsv_config_.blue;
    case VisionTarget::Line:
        return hsv_config_.line;
    }
    return hsv_config_.red;
}

const HSVRange &MainWindow::currentHSV() const
{
    switch (current_target_) {
    case VisionTarget::Red:
        return hsv_config_.red;
    case VisionTarget::Blue:
        return hsv_config_.blue;
    case VisionTarget::Line:
        return hsv_config_.line;
    }
    return hsv_config_.red;
}

void MainWindow::syncBufferToUi()
{
    suppress_ui_sync_ = true;
    const HSVRange &hsv = currentHSV();

    ui->slider_h_low->setValue(hsv.h_low);
    ui->slider_h_high->setValue(hsv.h_high);
    ui->slider_s_low->setValue(hsv.s_low);
    ui->slider_s_high->setValue(hsv.s_high);
    ui->slider_v_low->setValue(hsv.v_low);
    ui->slider_v_high->setValue(hsv.v_high);

    ui->spinBox_h_low->setValue(hsv.h_low);
    ui->spinBox_h_high->setValue(hsv.h_high);
    ui->spinBox_s_low->setValue(hsv.s_low);
    ui->spinBox_s_high->setValue(hsv.s_high);
    ui->spinBox_v_low->setValue(hsv.v_low);
    ui->spinBox_v_high->setValue(hsv.v_high);
    suppress_ui_sync_ = false;
}

void MainWindow::syncUiToBuffer()
{
    if (suppress_ui_sync_) {
        return;
    }

    HSVRange &hsv = currentHSV();
    hsv.h_low = ui->slider_h_low->value();
    hsv.h_high = ui->slider_h_high->value();
    hsv.s_low = ui->slider_s_low->value();
    hsv.s_high = ui->slider_s_high->value();
    hsv.v_low = ui->slider_v_low->value();
    hsv.v_high = ui->slider_v_high->value();
}

void MainWindow::setCurrentTarget(VisionTarget target)
{
    current_target_ = target;
    syncBufferToUi();
}

void MainWindow::setRangeToZero(HSVRange &range)
{
    range = HSVRange{};
}

void MainWindow::on_spinBox_h_low_valueChanged(int h_low)
{
    if (suppress_ui_sync_) return;
    ui->slider_h_low->setValue(h_low);
    currentHSV().h_low = h_low;
}

void MainWindow::on_spinBox_s_low_valueChanged(int s_low)
{
    if (suppress_ui_sync_) return;
    ui->slider_s_low->setValue(s_low);
    currentHSV().s_low = s_low;
}

void MainWindow::on_spinBox_v_low_valueChanged(int v_low)
{
    if (suppress_ui_sync_) return;
    ui->slider_v_low->setValue(v_low);
    currentHSV().v_low = v_low;
}

void MainWindow::on_spinBox_h_high_valueChanged(int h_high)
{
    if (suppress_ui_sync_) return;
    ui->slider_h_high->setValue(h_high);
    currentHSV().h_high = h_high;
}

void MainWindow::on_spinBox_s_high_valueChanged(int s_high)
{
    if (suppress_ui_sync_) return;
    ui->slider_s_high->setValue(s_high);
    currentHSV().s_high = s_high;
}

void MainWindow::on_spinBox_v_high_valueChanged(int v_high)
{
    if (suppress_ui_sync_) return;
    ui->slider_v_high->setValue(v_high);
    currentHSV().v_high = v_high;
}

void MainWindow::on_slider_h_low_valueChanged(int h_low)
{
    if (suppress_ui_sync_) return;
    ui->spinBox_h_low->setValue(h_low);
    currentHSV().h_low = h_low;
}

void MainWindow::on_slider_s_low_valueChanged(int s_low)
{
    if (suppress_ui_sync_) return;
    ui->spinBox_s_low->setValue(s_low);
    currentHSV().s_low = s_low;
}

void MainWindow::on_slider_v_low_valueChanged(int v_low)
{
    if (suppress_ui_sync_) return;
    ui->spinBox_v_low->setValue(v_low);
    currentHSV().v_low = v_low;
}

void MainWindow::on_slider_h_high_valueChanged(int h_high)
{
    if (suppress_ui_sync_) return;
    ui->spinBox_h_high->setValue(h_high);
    currentHSV().h_high = h_high;
}

void MainWindow::on_slider_s_high_valueChanged(int s_high)
{
    if (suppress_ui_sync_) return;
    ui->spinBox_s_high->setValue(s_high);
    currentHSV().s_high = s_high;
}

void MainWindow::on_slider_v_high_valueChanged(int v_high)
{
    if (suppress_ui_sync_) return;
    ui->spinBox_v_high->setValue(v_high);
    currentHSV().v_high = v_high;
}

void MainWindow::on_radioButton_red_clicked()
{
    setCurrentTarget(VisionTarget::Red);
}

void MainWindow::on_radioButton_blue_clicked()
{
    setCurrentTarget(VisionTarget::Blue);
}

void MainWindow::on_radioButton_line_clicked()
{
    setCurrentTarget(VisionTarget::Line);
}

void MainWindow::on_pushButton_set0_clicked()
{
    setRangeToZero(currentHSV());
    syncBufferToUi();
}

void MainWindow::on_pushButton_yolo_clicked()
{
}

void MainWindow::on_pushButton_save_clicked()
{
    QString default_dir = QDir::homePath() + "/colcon_ws/src/vision_tune/config";
    QDir().mkpath(default_dir);
    QString file_path = QFileDialog::getSaveFileName(
        this,
        "Save HSV YAML",
        default_dir + "/hsv_preset.yaml",
        "YAML Files (*.yaml *.yml)");

    if (file_path.isEmpty()) {
        return;
    }
    if (!file_path.endsWith(".yaml", Qt::CaseInsensitive) &&
        !file_path.endsWith(".yml", Qt::CaseInsensitive)) {
        file_path += ".yaml";
    }
    saveYaml(file_path);
}

void MainWindow::on_pushButton_load_clicked()
{
    QString default_dir = QDir::homePath() + "/colcon_ws/src/vision_tune/config";
    QDir().mkpath(default_dir);
    QString file_path = QFileDialog::getOpenFileName(
        this,
        "Load HSV YAML",
        default_dir,
        "YAML Files (*.yaml *.yml)");

    if (file_path.isEmpty()) {
        return;
    }
    loadYaml(file_path);
}

bool MainWindow::saveYaml(const QString &file_path)
{
    try {
        YAML::Node root;
        YAML::Node params;
        params["red"] = toYaml(hsv_config_.red);
        params["blue"] = toYaml(hsv_config_.blue);
        params["line"] = toYaml(hsv_config_.line);
        root["vision_tune"]["ros__parameters"] = params;

        std::ofstream fout(file_path.toStdString());
        fout << root;
        return true;
    } catch (const std::exception &e) {
        QMessageBox::critical(this, "Save Error", e.what());
        return false;
    }
}

bool MainWindow::loadYaml(const QString &file_path)
{
    try {
        YAML::Node root = YAML::LoadFile(file_path.toStdString());
        YAML::Node params = root["vision_tune"]["ros__parameters"];
        if (!params) {
            QMessageBox::warning(this, "Load Error", "Invalid YAML format");
            return false;
        }

        fromYaml(params["red"], hsv_config_.red);
        fromYaml(params["blue"], hsv_config_.blue);
        fromYaml(params["line"], hsv_config_.line);
        syncBufferToUi();
        return true;
    } catch (const std::exception &e) {
        QMessageBox::critical(this, "Load Error", e.what());
        return false;
    }
}

void MainWindow::updateResult(const vision_tune::msg::ProcessResult &msg)
{
    ui->label_detected->setText(msg.detected ? "true" : "false");
    ui->label_center_x->setText(QString::number(msg.center_x));
    ui->label_center_y->setText(QString::number(msg.center_y));
    ui->label_area->setText(QString::number(msg.area));
}
