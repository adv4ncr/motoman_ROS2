// Copyright 2020 Norwegian University of Science and Technology.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef MOTOMAN_HARDWARE__SIMPLE_MESSAGE_HPP_
#define MOTOMAN_HARDWARE__SIMPLE_MESSAGE_HPP_

// PORTS
#define TCP_PORT_MOTION						50240
#define TCP_PORT_STATE						50241
#define TCP_PORT_IO							  50242
#define TCP_PORT_MOTION_COMMAND   50243
#define UDP_PORT_REALTIME_MOTION  50244

namespace motoman_hardware::simple_message
{
#pragma pack(push, 1) // prevent data structure padding

constexpr int ROS_MAX_JOINT = 10;
constexpr int MOT_MAX_GR = 4;

struct Prefix
{
  int length;
};

enum class MsgType
{
  ROS_MSG_MOTO_MOTION_CTRL = 2001,
  ROS_MSG_MOTO_MOTION_REPLY = 2002,

  MOTO_REALTIME_MOTION_JOINT_STATE_EX = 2030,
  MOTO_REALTIME_MOTION_JOINT_COMMAND_EX = 2031
};

enum class CommType
{
  INVALID = 0,
  TOPIC = 1,
  SERVICE_REQUEST = 2,
  SERVICE_REPLY = 3
};

enum class ReplyType
{
  INVALID = 0,
  SUCCESS = 1,
  FAILURE = 2
};

struct Header
{
  MsgType msg_type;
  CommType comm_type;
  ReplyType reply_type;
};

enum class SmCommandType
{
	ROS_CMD_CHECK_MOTION_READY = 200101,
	ROS_CMD_CHECK_QUEUE_CNT = 200102,
	ROS_CMD_STOP_MOTION = 200111,
	ROS_CMD_START_SERVOS = 200112, // starts the servo motors
	ROS_CMD_STOP_SERVOS = 200113, // stops the servo motors and motion
	ROS_CMD_RESET_ALARM = 200114, // clears the error in the current controller
	ROS_CMD_START_TRAJ_MODE = 200121, // Enable robot motion
	ROS_CMD_STOP_TRAJ_MODE = 200122, // Disable robot motion
	ROS_CMD_DISCONNECT = 200130,
	ROS_CMD_START_RT_MODE = 200140, // Start real-time mode
	ROS_CMD_STOP_RT_MODE = 200141, // Stop real-time mode
	ROS_CMD_CHECK_TIMEOUT_CNT = 200142, // Check the number of late packages in RT mode
};

struct SmBodyMotoMotionCtrl	// ROS_MSG_MOTO_MOTION_CTRL = 2001
{
	int groupNo;  			        // Robot/group ID;  0 = 1st robot 
	int sequence;				        // Optional message tracking number that will be echoed back in the response.
	SmCommandType command;		  // Desired command
	float data[ROS_MAX_JOINT];	// Command data - for future use  
};

typedef enum
{
	ROS_RESULT_SUCCESS = 0,
	ROS_RESULT_TRUE = 0,
	ROS_RESULT_BUSY = 1,
	ROS_RESULT_FAILURE = 2,
	ROS_RESULT_FALSE = 2,
	ROS_RESULT_INVALID = 3,
	ROS_RESULT_ALARM = 4,
	ROS_RESULT_NOT_READY = 5,
	ROS_RESULT_MP_FAILURE = 6
} SmResultType;

struct SmBodyMotoMotionReply	// ROS_MSG_MOTO_MOTION_REPLY = 2002
{
	int groupNo;  				      // Robot/group ID;  0 = 1st robot 
	int sequence;				        // Optional message tracking number that will be echoed back in the response.
	int command;				        // Reference to the received message command or type
	SmResultType result;		    // High level result code
	int subcode;				        // More detailed result code (optional)
	float data[ROS_MAX_JOINT];	// Reply data - for future use 
};

enum class MotoRealTimeMotionMode
{
  IDLE = 0,
  JOINT_POSITION = 1,
  JOINT_VELOCITY = 2
};

struct MotoRealTimeMotionJointStateExData
{
  int groupno;               // Robot/group ID;  0 = 1st robot
  float pos[ROS_MAX_JOINT];  // Feedback joint positions in radian. Base to Tool joint order
  float vel[ROS_MAX_JOINT];  // Feedback joint velocities in radian/sec.
};

struct MotoRealTimeMotionJointStateEx
{
  int message_id;  // Message id that the external control must echo back in the command
  MotoRealTimeMotionMode mode;
  int number_of_valid_groups;
  MotoRealTimeMotionJointStateExData joint_state_ex_data[MOT_MAX_GR];
};

struct MotoRealTimeMotionJointCommandExData
{
  int groupno;                   // Robot/group ID;  0 = 1st robot
  float command[ROS_MAX_JOINT];  // Command data. Either joint positions or velocities dependent on the current mode.
};

struct MotoRealTimeMotionJointCommandEx
{
  int message_id;  // Message id that the external control must echo back in the command
  int number_of_valid_groups;
  MotoRealTimeMotionJointCommandExData joint_command_ex_data[MOT_MAX_GR];
};

union Body
{
  SmBodyMotoMotionCtrl motionCtrl;
  SmBodyMotoMotionReply motionReply;

  MotoRealTimeMotionJointStateEx joint_state_ex;
  MotoRealTimeMotionJointCommandEx joint_command_ex;
};

struct SimpleMessage
{
  Prefix prefix;
  Header header;
  Body body;
};

#pragma pack(pop)

}  // namespace motoman_hardware::simple_message

#endif  // MOTOMAN_HARDWARE__SIMPLE_MESSAGE_HPP_
