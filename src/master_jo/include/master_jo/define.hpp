#ifndef DEFINE_HPP
#define DEFINE_HPP

#define MAX_ROBOT_NUM 4

#define MOTION_RASEUP_FRONT 2
#define MOTION_RASEUP_REAR 3
#define MOTION_RASEUP_RIGHT 4
#define MOTION_RASEUP_LEFT 5
#define MOTION_KICK_FRONT_R 0
#define MOTION_KICK_FRONT_L 1
#define MOTION_KICK_SIDE_R 0
#define MOTION_KICK_SIDE_L 1
// true save motion (without first init)
#define MOTION_ARM_RIGHT 0
#define MOTION_ARM_LEFT 1

// fall down
#define NOT_FALLDOWN 0
#define FALLDOWN 1
#define FALLDOWN_FRONT 2
#define FALLDOWN_REAR 3
#define FALLDOWN_RIGHT 4
#define FALLDOWN_LEFT 5

// my side
#define LEFT 0
#define RIGHT 1

// target object
#define BALL 0
#define COORDINATE 1

// angle mode
#define OPPONENT 0
#define ABSOLUTE 1
#define RELATIVE 2

// kick mode
#define NONE 0
#define FRONT_KICK 1
#define FRONT_RIGHT 2
#define FRONT_LEFT 3
#define SIDE_KICK 4
#define SIDE_RIGHT 5
#define SIDE_LEFT 6

// robot state
#define ROBOT_STATE_NONE 0
#define ROBOT_STATE_NOBALL 1
#define ROBOT_STATE_SUPPORT 2
#define ROBOT_STATE_CONTROLL 3
#define ROBOT_STATE_ATTACK 4
#define ROBOT_STATE_DEFENCE 5
#define ROBOT_STATE_PICKUP 6

#define KEEPER_STATE_KEEPERZONE 14
#define KEEPER_STATE_CENTER 3
#define NORMAL 4
#define KEEPER_STATE_QUICK_BACK 5

// position
#define POSITION 0
#define POSITION_GK 1
#define POSITION_FW 2

// vision
#define VISION_SAVE_CAMERA_B -1
#define VISION_SAVE_CAMERA_F -2
#define VISION_SCAN_MODE_INIT 0
#define VISION_SCAN_MODE_OPER 1
#define VISION_SCAN_MODE_BALL 2
#define VISION_SCAN_MODE_LINE 3
#define VISION_SCAN_MODE_CNTR 4

#endif // DEFINE_HPP
