#include "../include/master_jo/master_jo_node.hpp"

#include <rclcpp/rclcpp.hpp>

namespace master_jo
{

  MasterJoNode::MasterJoNode(const rclcpp::NodeOptions &options)
      : rclcpp_lifecycle::LifecycleNode("master_jo_node", options)
  {
    RCLCPP_INFO(get_logger(), "Creating node");
    init_parameters();
    RCLCPP_INFO(get_logger(), "Parameters initialized");
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  MasterJoNode::on_configure(
      const rclcpp_lifecycle::State &previous_state)
  {
    RCLCPP_INFO(get_logger(), "Configuring node");
    get_parameters();

    ik_pub = create_publisher<humanoid_interfaces::msg::Master2IkMsg>("master2ik", 10);
    // timer_ = create_wall_timer(100ms, [this]() {
    //       ik_pub->publish(master->ik); cout<< "발행함" << endl;
    //     });
    vision_pub = create_publisher<humanoid_interfaces::msg::Master2vision25>(
        "master2vision", 10);
    gamecontrol_pub = create_publisher<humanoid_interfaces::msg::Gamecontrolreturndata>(
        "gamecontrolreturndata", 10);
    local_pub = create_publisher<humanoid_interfaces::msg::Master2localization25>(
        "master2local", 10);
    udp_pub = create_publisher<humanoid_interfaces::msg::Master2udp>("master2udp", 10);
    motionPub = create_publisher<humanoid_interfaces::msg::MotionOperator>(
        "motion_operator", 10);

    imu_sub = create_subscription<humanoid_interfaces::msg::ImuMsg>(
        "Imu", rclcpp::QoS(rclcpp::KeepLast(10)).reliable().best_effort(),
        std::bind(&MasterJoNode::imuCallback, this,
                  std::placeholders::_1));
    vision_sub = create_subscription<humanoid_interfaces::msg::HumanPjVision>(
        "/robit_mj/red_pixel_flag", 10,
        std::bind(&MasterJoNode::visionCallback, this,
                  std::placeholders::_1));
    ik_sub = create_subscription<humanoid_interfaces::msg::IkEndMsg>(
        "ik", 10,
        std::bind(&MasterJoNode::ikCallback, this, std::placeholders::_1));
    gamecontrol_sub =
        create_subscription<humanoid_interfaces::msg::Gamecontroldata>(
            "gamecontroldata", 10,
            std::bind(&MasterJoNode::gamecontrolCallback, this,
                      std::placeholders::_1));
    local_sub =
        create_subscription<humanoid_interfaces::msg::Robocuplocalization25>(
            "localization", 10,
            std::bind(&MasterJoNode::localCallback, this,
                      std::placeholders::_1));
    udp_sub = create_subscription<humanoid_interfaces::msg::Udp2master>(
        "udp", 10,
        std::bind(&MasterJoNode::udpCallback, this,
                  std::placeholders::_1));
    pid_sub = create_subscription<humanoid_interfaces::msg::Pidtuning>(
        "pid", 10,
        std::bind(&MasterJoNode::pidCallback, this,
                  std::placeholders::_1));

    try
    {
      master = std::make_shared<master_jo::MasterRcko>();
      master->kp = kp_;
      master->kd = kd_;
      master->ki = ki_;
      master->FRONT_MAX = FRONT_MAX_;
      master->X_MIN = X_MIN_;
      master->REAR_MAX = REAR_MAX_;
      master->RIGHT_MAX = RIGHT_MAX_;
      master->Y_MIN = Y_MIN_;
      master->ROUND_Y = ROUND_Y_;
      master->ROUND_YAW_MIN = ROUND_YAW_MIN_;
      master->LEFT_MAX = LEFT_MAX_;
      master->R_YAW_MAX = R_YAW_MAX_;
      master->L_YAW_MAX = L_YAW_MAX_;
      master->In = In_;
      master->Out = Out_;
      master->Back = Back_;
      master->Front = Front_;
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(get_logger(), "Failed to initialize Master: %s", e.what());
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
          CallbackReturn::FAILURE;
    }

    RCLCPP_INFO(get_logger(), "Parameters retrieved");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
        CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  MasterJoNode::on_activate(
      const rclcpp_lifecycle::State &previous_state)
  {
    RCLCPP_INFO(get_logger(), "Activating node");
    ik_pub->on_activate();
    vision_pub->on_activate();
    gamecontrol_pub->on_activate();
    local_pub->on_activate();
    udp_pub->on_activate();
    motionPub->on_activate();

    try
    {
      timer = create_wall_timer(
          std::chrono::milliseconds(100),
          std::bind(&MasterJoNode::robocup_master, this));
      RCLCPP_INFO(get_logger(), "Timer created successfully");
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(get_logger(), "Failed to enable: %s", e.what());
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
          CallbackReturn::FAILURE;
    }

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
        CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  MasterJoNode::on_deactivate(
      const rclcpp_lifecycle::State &previous_state)
  {
    RCLCPP_INFO(get_logger(), "Deactivating node");
    ik_pub->on_deactivate();
    vision_pub->on_deactivate();
    gamecontrol_pub->on_deactivate();
    local_pub->on_deactivate();
    udp_pub->on_deactivate();
    motionPub->on_deactivate();

    timer->cancel();
    player.reset();
    if (player == nullptr)
    {
      RCLCPP_INFO(get_logger(), "Player reset");
    }
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
        CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  MasterJoNode::on_cleanup(const rclcpp_lifecycle::State &previous_state)
  {
    RCLCPP_INFO(get_logger(), "Cleaning up node");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
        CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  MasterJoNode::on_shutdown(
      const rclcpp_lifecycle::State &previous_state)
  {
    RCLCPP_INFO(get_logger(), "Shutting down node");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
        CallbackReturn::SUCCESS;
  }

  rcl_interfaces::msg::SetParametersResult
  MasterJoNode::on_parameter_change(
      const std::vector<rclcpp::Parameter> &parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto &param : parameters)
    {
      RCLCPP_INFO(get_logger(), "Parameter '%s' changed",
                  param.get_name().c_str());
    }

    return result;
  }

  void MasterJoNode::init_parameters()
  {
    declare_parameter("Kp", 0.0);
    declare_parameter("Kd", 0.0);
    declare_parameter("Ki", 0.0);
    declare_parameter("FRONT_MAX", 0);
    declare_parameter("REAR_MAX", 0);
    declare_parameter("RIGHT_MAX", 0);
    declare_parameter("LEFT_MAX", 0);
    declare_parameter("R_YAW_MAX", 0);
    declare_parameter("L_YAW_MAX", 0);
    declare_parameter("X_MIN", 0);
    declare_parameter("Y_MIN", 0);
    declare_parameter("ROUND_YAW_MIN", 0);
    declare_parameter("ROUND_Y", 0);
    declare_parameter("In", 0);
    declare_parameter("Out", 0);
    declare_parameter("Back", 0);
    declare_parameter("Front", 0);
  }

  void MasterJoNode::get_parameters()
  {
    RCLCPP_INFO(get_logger(), "Getting parameters");
    kp_ = get_parameter("Kp").as_double();
    kd_ = get_parameter("Kd").as_double();
    ki_ = get_parameter("Ki").as_double();
    FRONT_MAX_ = get_parameter("FRONT_MAX").as_int();
    REAR_MAX_ = get_parameter("REAR_MAX").as_int();
    RIGHT_MAX_ = get_parameter("RIGHT_MAX").as_int();
    LEFT_MAX_ = get_parameter("LEFT_MAX").as_int();
    R_YAW_MAX_ = get_parameter("R_YAW_MAX").as_int();
    L_YAW_MAX_ = get_parameter("L_YAW_MAX").as_int();
    X_MIN_ = get_parameter("X_MIN").as_int();
    Y_MIN_ = get_parameter("Y_MIN").as_int();
    ROUND_YAW_MIN_ = get_parameter("ROUND_YAW_MIN").as_int();
    ROUND_Y_ = get_parameter("ROUND_Y").as_int();
    In_ = get_parameter("In").as_int();
    Out_ = get_parameter("Out").as_int();
    Back_ = get_parameter("Back").as_int();
    Front_ = get_parameter("Front").as_int();
    std::cout << "Kp: " << kp_ << std::endl;
    std::cout << "Kd: " << kd_ << std::endl;
    std::cout << "Ki: " << ki_ << std::endl;
    std::cout << "FRONT_MAX: " << FRONT_MAX_ << std::endl;
    std::cout << "REAR_MAX: " << REAR_MAX_ << std::endl;
    std::cout << "RIGHT_MAX: " << RIGHT_MAX_ << std::endl;
    std::cout << "LEFT_MAX: " << LEFT_MAX_ << std::endl;
    std::cout << "R_YAW_MAX: " << R_YAW_MAX_ << std::endl;
    std::cout << "L_YAW_MAX: " << L_YAW_MAX_ << std::endl;
    std::cout << "X_MIN: " << X_MIN_ << std::endl;
    std::cout << "Y_MIN: " << Y_MIN_ << std::endl;
    std::cout << "ROUND_YAW_MIN: " << ROUND_YAW_MIN_ << std::endl;
    std::cout << "ROUND_Y: " << ROUND_Y_ << std::endl;
    std::cout << "In: " << In_ << std::endl;
    std::cout << "Out: " << Out_ << std::endl;
    std::cout << "Back: " << Back_ << std::endl;
    std::cout << "Front: " << Front_ << std::endl;
  }

  void MasterJoNode::imuCallback(
      const humanoid_interfaces::msg::ImuMsg::SharedPtr msg)
  {
    master->imu.pitch = msg->pitch; // ㅇㅋ
    master->imu.roll = msg->roll;
    master->imu.yaw = msg->yaw;
  }

  void MasterJoNode::visionCallback(
      const humanoid_interfaces::msg::HumanPjVision::SharedPtr msg)
  {
    master->vision.flag = msg->flag;
  }

  void MasterJoNode::ikCallback(
      const humanoid_interfaces::msg::IkEndMsg::SharedPtr msg)
  {
    master->ikEnd.ikend = msg->ikend; // ㅇㅋ
  }

  void MasterJoNode::gamecontrolCallback(
      const humanoid_interfaces::msg::Gamecontroldata::SharedPtr msg)
  {
    master->gameControlData.robotnum = msg->robotnum; // ㅇㅋ
    master->gameControlData.state = msg->state;
    master->gameControlData.readytime = msg->readytime;
    master->gameControlData.penalty = msg->penalty;
    master->gameControlData.myside = msg->myside;
    master->gameControlData.iskickoff = msg->iskickoff;
    master->gameControlData.position = msg->position;
    master->gameControlData.secondstate = msg->secondstate;
    master->gameControlData.secondinfo = msg->secondinfo;
    master->gameControlData.myteam = msg->myteam;
  }

  void MasterJoNode::localCallback(
      const humanoid_interfaces::msg::Robocuplocalization25::SharedPtr msg)
  {
    master->local.robot_x = msg->robot_x; // ㅇㅋ
    master->local.robot_y = msg->robot_y;
    master->local.ball_x = msg->ball_x;
    master->local.ball_y = msg->ball_y;
    master->local.ball_speed_x = msg->ball_speed_x;
    master->local.ball_speed_y = msg->ball_speed_y;
    master->local.obstacle0_x = msg->obstacle0_x;
    master->local.obstacle0_y = msg->obstacle0_y;
    master->local.obstacle1_x = msg->obstacle1_x;
    master->local.obstacle1_y = msg->obstacle1_y;
    master->local.obstacle2_x = msg->obstacle2_x;
    master->local.obstacle2_y = msg->obstacle2_y;
    master->local.obstacle3_x = msg->obstacle3_x;
    master->local.obstacle3_y = msg->obstacle3_y;
    master->local.obstacle_x = msg->obstacle_x;
    master->local.obstacle_y = msg->obstacle_y;
  }

  void MasterJoNode::udpCallback(
      const humanoid_interfaces::msg::Udp2master::SharedPtr msg)
  {
    int i = static_cast<int>(msg->robotnum - 1); // ㅇ?ㅋi를 얼마나 해야할지..
    master->udp[i].robotnum = msg->robotnum;
    master->udp[i].robotcase = msg->robotcase;
    master->udp[i].localx = msg->localx;
    master->udp[i].localy = msg->localy;
    master->udp[i].localyaw = msg->localyaw;
    master->udp[i].balldist = msg->balldist;

    master->udp[3].balldist = 0;
  }

  void MasterJoNode::motionCallback(
      const humanoid_interfaces::msg::MotionOperator::SharedPtr msg)
  {
    master->motionEnd.motion_num = msg->motion_num; // ㅇㅋ
    master->motionEnd.motion_end = msg->motion_end;
    player->publish_motion_complete = true;
  }

  void MasterJoNode::pidCallback(
      const humanoid_interfaces::msg::Pidtuning::SharedPtr msg)
  {
    master->kp = msg->kp;
    master->kd = msg->kd;
    master->ki = msg->ki;
  }

  void MasterJoNode::robocup_master()
  {
    using namespace std;
    if (!testFlag)
    {
      position = POSITION_FW; // master->gameControlData.position;
      state = master->gameControlData.state;
    }

    if (!player)
    {
      switch (position)
      {
      case POSITION_FW:
        RCLCPP_INFO(get_logger(), "POSITION_FW");
        player = std::make_shared<master_jo::Forward>(master);
        break;
      default:
        if (position != POSITION_FW)
        {
          RCLCPP_INFO(get_logger(), "NO PLAYER SET (NOT FW)");
        }
        break;
      }
    }

    if (player)
    {
      // else player->walkStart(master->FRONT_MAX, 0, 0);

      if (!player->falldownExeption())
      {
        player->selectGoalPost();
        player->penaltyControl(isPenalty);
        player->selectRobotState(isPenalty);
        player->gameStateControl(!isPenalty, state);
      }

      player->publishMsg();//pub
      master->ik.x_length = player->x;
      master->ik.y_length = player->y;
      master->ik.yaw = player->yaw;

      ik_pub->publish(master->ik);
      cout<<"ik_pubished"<<endl;            // ㅇㅋ
      vision_pub->publish(master->master2vision);              // ㅇㅋ
      gamecontrol_pub->publish(master->gameControlReturnData); // ㅇㅋ
      local_pub->publish(master->master2local);                // ㅇㅋ
      udp_pub->publish(master->master2udp);                    // ㅇㅋ
      if (player->publish_motion_flag)
      {
        motionPub->publish(master->motion); // ㅇㅋ
        player->publish_motion_complete = false;
        player->publish_motion_flag = false;
      }
    }
  }

}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<master_jo::MasterJoNode>();

  auto configure_result = node->trigger_transition(
      lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  if (configure_result.id() !=
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
  {
    RCLCPP_ERROR(node->get_logger(), "Failed to configure udp.");
    return 1;
  }

  auto activate_result = node->trigger_transition(
      lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  if (activate_result.id() !=
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
  {
    RCLCPP_ERROR(node->get_logger(), "Failed to activate udp.");
    return 1;
  }

  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}