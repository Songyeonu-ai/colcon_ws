#ifndef VISION_TUNE_MAIN_WINDOW_HPP_
#define VISION_TUNE_MAIN_WINDOW_HPP_

#include <QMainWindow>
#include <QFileDialog>
#include <QMessageBox>
#include <QPixmap>
#include <yaml-cpp/yaml.h>

#include <fstream>
#include "vision_tune/msg/process_result.hpp"

QT_BEGIN_NAMESPACE
namespace Ui {
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
    Red,
    Blue,
    Line
};

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

    const HSVConfig &getHSVConfig() const;
    HSVConfig &getHSVConfigMutable();
    VisionTarget getCurrentTarget() const;

    void updateResult(const vision_tune::msg::ProcessResult &msg);
    bool saveYaml(const QString &file_path);
    bool loadYaml(const QString &file_path);

public Q_SLOTS:
    void on_spinBox_h_low_valueChanged(int h_low);
    void on_spinBox_s_low_valueChanged(int s_low);
    void on_spinBox_v_low_valueChanged(int v_low);
    void on_spinBox_h_high_valueChanged(int h_high);
    void on_spinBox_s_high_valueChanged(int s_high);
    void on_spinBox_v_high_valueChanged(int v_high);

    void on_slider_h_low_valueChanged(int h_low);
    void on_slider_s_low_valueChanged(int s_low);
    void on_slider_v_low_valueChanged(int v_low);
    void on_slider_h_high_valueChanged(int h_high);
    void on_slider_s_high_valueChanged(int s_high);
    void on_slider_v_high_valueChanged(int v_high);

    void on_pushButton_save_clicked();
    void on_pushButton_load_clicked();
    void on_pushButton_set0_clicked();
    void on_pushButton_yolo_clicked();

    void on_radioButton_red_clicked();
    void on_radioButton_blue_clicked();
    void on_radioButton_line_clicked();

public slots:
    void updateRawImage(const QPixmap &pixmap);
    void updateResultImage(const QPixmap &pixmap);

private:
    HSVRange &currentHSV();
    const HSVRange &currentHSV() const;

    void syncBufferToUi();
    void syncUiToBuffer();
    void setCurrentTarget(VisionTarget target);
    void setRangeToZero(HSVRange &range);

    Ui::MainWindow *ui;
    HSVConfig hsv_config_{};
    VisionTarget current_target_ = VisionTarget::Red;
    bool suppress_ui_sync_ = false;
};

#endif
