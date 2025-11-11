/**
 * @file /include/ebimu/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ebimu_QNODE_HPP_
#define ebimu_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/
#ifndef Q_MOC_RUN
#include <rclcpp/rclcpp.hpp>
#endif
#include <QThread>
#include <thread>
#include <memory>
#include <string>

#include "serial_manager.hpp"
#include "e2box_imu_node.hpp"

#include <sensor_msgs/msg/imu.hpp>


namespace e2box_imu {
/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread
{
  Q_OBJECT
public:
  QNode();
  ~QNode();

protected:
  void run();

private:
  std::shared_ptr<rclcpp::Node> node;
  std::shared_ptr<rclcpp::Node> qnode;

  void imu_callback(const humanoid_interfaces::msg::ImuMsg::SharedPtr msg);
  std::shared_ptr<rclcpp::Subscription<humanoid_interfaces::msg::ImuMsg>> Imu_Sub;


Q_SIGNALS:
  void dial_update();
  void rosShutDown();
  void dial_callback();
};
} // namespace ebimu

#endif /* ebimu_QNODE_HPP_ */
