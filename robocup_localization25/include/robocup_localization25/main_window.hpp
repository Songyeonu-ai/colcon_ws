/**
 * @file /include/robocup_localization25/main_window.hpp
 *
 * @brief Qt based gui for %(package)s.
 *
 * @date August 2024
 **/

#ifndef robocup_localization25_MAIN_WINDOW_H
#define robocup_localization25_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QMainWindow>
#include <QGraphicsScene>
#include "QIcon"
#include "qnode.hpp"
#include "ui_mainwindow.h"

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
  MainWindow(QWidget *parent = nullptr);
  ~MainWindow();
  QNode *qnode;

  humanoid_interfaces::msg::Robocuplocalization25 localizationMsg;


  // For timer
  QTimer *m_Timer;

  // For Screen
  QPixmap buf;
  QGraphicsScene *scene;
  void Print_Screen();
  int cvt_Print_xy(float target);
  QPolygonF create_Print_robot(ROBOT robot);

  // For Ros Topic
  void publish_msg();

  // For UI
  int set_robot_flag = 0;
  int setting_flag = 0;

  void setting();

  // For udp
  void sel_ball();

private:
  Ui::MainWindowDesign *ui;
  void closeEvent(QCloseEvent *event);

public slots:
  void main();
  void mouseReleaseEvent(QMouseEvent *e);
  void mouseMoveEvent(QMouseEvent *e);
  void on_btn_free_set_clicked();
  void on_btn_objects_save_clicked();
  void on_btn_test_clicked();
  void on_btn_set_1_clicked();
  void on_btn_set_2_clicked();
  void on_btn_set_3_clicked();
  void on_btn_set_4_clicked();
  void on_btn_set_5_clicked();
  void on_btn_set_6_clicked();
  void on_btn_set_auto_clicked();
  void on_btn_ball_set_clicked();
  void on_btn_ball_del_clicked();
  void featureCalc();
};

#endif // robocup_localization25_MAIN_WINDOW_H
