#include "../include/motion_operator/motion_operator.hpp"

namespace motion_operator {

MOTION::MOTION(std::string KickRAddr, std::string KickLAddr,
               std::string FrontRiseAddr, std::string BackRiseAddr,
               std::string RightRiseAddr, std::string LeftRiseAddr) {
  KickRAddr_ = KickRAddr;
  KickLAddr_ = KickLAddr;
  FrontRiseAddr_ = FrontRiseAddr;
  BackRiseAddr_ = BackRiseAddr;
  RightRiseAddr_ = RightRiseAddr;
  LeftRiseAddr_ = LeftRiseAddr;
}

MOTION::~MOTION() { steps.clear(); }

bool MOTION::read_motion(std::string motion_path) {
  YAML::Node file = YAML::LoadFile(motion_path);

  // TODO : YAML 파일의 motor id와 현재 활성화된 motor id 비교

  // YAML 파일의 motor id와 현재 활성화된 motor id 비교
  // std::vector<int> file_motor_ids = file[1]["motor
  // id"].as<std::vector<int>>(); std::vector<int> current_motor_ids; for (int i
  // = 0; i < dxl_status.size(); i++) {
  //   current_motor_ids.push_back(dxl_status[i].id);
  // }

  // if (file_motor_ids != current_motor_ids) {
  //   std::cout << "Invalid motor id" << std::endl;
  //   std::cout << "File motor id: ";
  //   for (int i = 0; i < file_motor_ids.size(); i++) {
  //     std::cout << file_motor_ids[i] << " ";
  //   }
  //   std::cout << std::endl << std::endl;
  //   std::cout << "current motor id : ";
  //   for (int i = 0; i < current_motor_ids.size(); i++) {
  //     std::cout << current_motor_ids[i] << " ";
  //   }
  //   return false;
  // }

  std::string type = file[2]["type"].as<std::string>();

  steps.clear();
  if (type == "motion") {
    for (std::size_t i = 3; i < file.size(); ++i) {
      Step step;
      step.time = file[i]["time"].as<int>();
      step.delay = file[i]["delay"].as<int>();
      step.repeat = file[i]["repeat"].as<int>();
      step.name = file[i]["name"].as<std::string>();
      step.selected = file[i]["selected"].as<bool>();
      for (const auto &dxl : file[i]["dxl"]) {
        Dynamixel_postition pos;
        pos.id = dxl["id"].as<int>();
        pos.position = dxl["position"].as<double>();
        step.dxl.push_back(pos);
      }
      steps.push_back(step);
    }
    return true;
  } else {
    std::cout << "Invalid motion type" << std::endl;
    return false;
  }
}

bool MOTION::read_step(std::string motion_path) {
  YAML::Node file = YAML::LoadFile(motion_path);

  // // YAML 파일의 motor id와 현재 활성화된 motor id 비교
  // std::vector<int> file_motor_ids = file[1]["motor
  // id"].as<std::vector<int>>(); std::vector<int> current_motor_ids; for (int i
  // = 0; i < dxl_status.size(); i++) {
  //   current_motor_ids.push_back(dxl_status[i].id);
  // }

  // if (file_motor_ids != current_motor_ids) {
  //   std::cout << "Invalid motor id" << std::endl;
  //   std::cout << "File motor id: ";
  //   for (int i = 0; i < file_motor_ids.size(); i++) {
  //     std::cout << file_motor_ids[i] << " ";
  //   }
  //   std::cout << std::endl << std::endl;
  //   std::cout << "current motor id : ";
  //   for (int i = 0; i < current_motor_ids.size(); i++) {
  //     std::cout << current_motor_ids[i] << " ";
  //   }
  //   return false;
  // }

  std::string type = file[2]["type"].as<std::string>();

  steps.clear();
  if (type == "step") {
    for (std::size_t i = 3; i < file.size(); ++i) {
      Step step;
      step.time = file[i]["time"].as<int>();
      step.delay = file[i]["delay"].as<int>();
      step.repeat = file[i]["repeat"].as<int>();
      step.name = file[i]["name"].as<std::string>();
      step.selected = file[i]["selected"].as<bool>();
      for (const auto &dxl : file[i]["dxl"]) {
        Dynamixel_postition pos;
        pos.id = dxl["id"].as<int>();
        pos.position = dxl["position"].as<double>();
        step.dxl.push_back(pos);
      }

      init = step;
    }
  } else {
    std::cout << "Invalid motion type" << std::endl;
    return false;
  }
  return true;
}

std::vector<double> MOTION::set_velocity(std::vector<double> goal_position,
                                         std::vector<double> current_position,
                                         int time) {
  std::vector<double> goal_velocity_;
  for (int i = 0; i < goal_position.size(); i++) {
    double velocity =
        calc_velocity(goal_position[i], current_position[i], time);
    goal_velocity_.push_back(velocity);
  }

  return goal_velocity_;
}

double MOTION::calc_velocity(double goal_position, double current_position,
                             int time) {
  double velocity = fabs(goal_position - current_position) / (time * 0.001);
  return velocity;
}

std::vector<double> MOTION::get_goal_position(int step_num) {
  std::vector<double> goal_position;
  for (int i = 0; i < steps[step_num].dxl.size(); i++) {
    goal_position.push_back(steps[step_num].dxl[i].position);
  }
  return goal_position;
}

std::vector<double> MOTION::get_before_position(int step_num) {
  if (step_num == 0) {
    std::vector<double> before_position;
    for (int i = 0; i < init.dxl.size(); i++) {
      before_position.push_back(init.dxl[i].position);
    }
    return before_position;
  } else {
    std::vector<double> before_position;
    for (int i = 0; i < steps[step_num - 1].dxl.size(); i++) {
      before_position.push_back(steps[step_num - 1].dxl[i].position);
    }
    return before_position;
  }
}
}  // namespace motion_operator
