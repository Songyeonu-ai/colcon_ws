/**
 * @file /include/ebimu/main_window.hpp
 *
 * @brief Qt based gui for %(package)s.
 *
 * @date August 2024
 **/

#ifndef ebimu_MAIN_WINDOW_H
#define ebimu_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QMainWindow>
#include "QIcon"
#include "qnode.hpp"
#include "ui_mainwindow.h"

namespace e2box_imu {


/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
  MainWindow(QWidget* parent = nullptr);
  ~MainWindow();
  QNode* qnode;
  E2BoxIMUNode* node;
  
  
public Q_SLOTS:
  void imu_callback();
  void on_pushButton_set_clicked();
  void on_pushButton_set_p90_clicked();
  void on_pushButton_set_n90_clicked();
  void on_pushButton_set_180_clicked();

private:
  Ui::MainWindowDesign* ui;
  void uiUpdate();
  void closeEvent(QCloseEvent* event);

  double past_yaw;

  
};
}
#endif  // ebimu_MAIN_WINDOW_H
