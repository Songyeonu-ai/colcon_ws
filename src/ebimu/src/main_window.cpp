/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date August 2024
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include "../include/ebimu/main_window.hpp"
#include "../include/ebimu/qnode.hpp"

//double roll = 0.0;
//double pitch = 0.0;
//double yaw = 0.0;
extern rclcpp::Publisher<humanoid_interfaces::msg::ImuMsg>::SharedPtr imu_publisher_;
extern humanoid_interfaces::msg::ImuMsg imu_data_;

namespace e2box_imu {

extern double roll,pitch,yaw;

MainWindow::MainWindow(QWidget* parent) : QMainWindow(parent), ui(new Ui::MainWindowDesign)
{
  ui->setupUi(this);
  uiUpdate();
  past_yaw = 0;
  ui->dial->setRange(-180 +30, 180 -30);

  QIcon icon("://ros-icon.png");
  this->setWindowIcon(icon);

  qnode = new QNode();

  QObject::connect(qnode, SIGNAL(dial_update()), this, SLOT(imu_callback()));
  //QObject::connect(qnode, &QNode::dial_update, this, &MainWindow::imu_callback);
  QObject::connect(qnode, SIGNAL(rosShutDown()), this, SLOT(close()));
}

void MainWindow::closeEvent(QCloseEvent* event)
{
  QMainWindow::closeEvent(event);
}

MainWindow::~MainWindow()
{
  delete ui;
}
void MainWindow::imu_callback()
{
  if(past_yaw > 180) past_yaw -= 360;
  else if(past_yaw <= -180)  past_yaw += 360;
  yaw -= past_yaw;

  if(yaw > 180) yaw -= 360;
  else if(yaw <= -180)  yaw += 360;
  imu_data_.yaw=yaw;

  std::cout.precision(4);
  std::cout << std::endl;
  std::cout<<" YAW  | "<<imu_data_.yaw << std::endl;
  std::cout<<" ROLL | "<<imu_data_.roll << std::endl;
  std::cout<<" PITCH| "<<imu_data_.pitch << std::endl;
  std::cout << std::endl;
  
  imu_publisher_->publish(imu_data_);
  //node->imu_publisher_->publish(imu_data_);
  
  uiUpdate();
}

void MainWindow::uiUpdate()
{
  ui->dial->setValue(static_cast<int>(-yaw));//y
  ui->verticalSlider->setValue(static_cast<int>(pitch));//p
  ui->horizontalSlider->setValue(static_cast<int>(roll));//r  
}

void MainWindow::on_pushButton_set_clicked()
{
    std::cout<<"set"<<std::endl;
    past_yaw += yaw;
}
void MainWindow::on_pushButton_set_p90_clicked()
{
    std::cout<<"set +90"<<std::endl;
    past_yaw += yaw + 90;
}

void MainWindow::on_pushButton_set_n90_clicked()
{
    std::cout<<"set -90"<<std::endl;
    past_yaw += yaw - 90;
}

void MainWindow::on_pushButton_set_180_clicked()
{
    std::cout<<"set 180"<<std::endl;
    past_yaw += yaw + 180;
}
}
