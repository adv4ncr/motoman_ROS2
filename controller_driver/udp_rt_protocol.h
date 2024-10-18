#ifndef UDP_RT_PROTOCOL_H
#define UDP_RT_PROTOCOL_H

//#define REALTIME_MOTION_BUFFER_SIZE_MAX  1024
#define REALTIME_MOTION_TIMEOUT_COUNTER_MAX 20
#define REALTIME_MOTION_TIMEOUT_MS 4 // in milliseconds
#define REALTIME_MOTION_UDP_PORT 50244
#define TCP_PORT_MOTION_COMMAND   50243

#define RT_ROBOT_JOINTS_MAX 8
#define RT_ROBOT_GROUPS 1

#ifdef __cplusplus

#include <assert.h>
#pragma pack(push, 1)

namespace motoman_hardware::udp_rt_message
{

#else
// Static assert using https://www.pixelbeat.org/programming/gcc/static_assert.html
#define ASSERT_CONCAT_(a, b) a##b
#define ASSERT_CONCAT(a, b) ASSERT_CONCAT_(a, b)
/* These can't be used after statements in c89. */
#ifdef __COUNTER__
  #define static_assert(e,m) \
    ;enum { ASSERT_CONCAT(static_assert_, __COUNTER__) = 1/(int)(!!(e)) }
#else
  /* This can't be used twice on the same line so ensure if using in headers
   * that the headers are not included twice (by wrapping in #ifndef...#endif)
   * Note it doesn't cause an issue when used on same line of separate modules
   * compiled with gcc -combine -fwhole-program.  */
  #define static_assert(e,m) \
    ;enum { ASSERT_CONCAT(assert_line_, __LINE__) = 1/(int)(!!(e)) }
#endif
#endif // __cplusplus



	

	// create enum using typedef:
	//typedef enum __attribute__((__packed__)) {...} MyEnum;

	// ------------------ MESSAGE STATES ------------------
	enum RtMsgState
	{
		// Nominal operation states. Used as status indicators. Do not set via interface.
		STATE_UNDEFINED = 0,
		STATE_INITIALIZE = 1,
		STATE_START = 2,
		STATE_IDLE = 3,
		STATE_RUN = 4,
		STATE_EXIT = 9,
		STATE_RUN_STALL = 10,
		STATE_RUN_OUT = 11,
		STATE_RUN_IN = 12,
		STATE_IO_OPERATION = 20, // #TODO

		// Error handling
		STATE_ERROR_GENERAL = 100,
		STATE_ERROR_CONTROLLER = 101,
		STATE_ERROR_HOST = 102,

		// Exit condition
		STATE_FINAL =255

	} __attribute__((__packed__));
	//typedef enum RtMsgState RtMsgState;

	// ------------------ MESSAGE CODES ------------------
	//
	// On the controller (NotReadyCode) UNSPECIFIED starts at 5000, ALARM = 5001 ... #TODO

	enum RtMsgCode
	{
		CODE_UNDEFINED = 0,
		CODE_CONFIRM = 1,

		CODE_MAX_INC_ERROR = 10,
		CODE_MAX_ACC_ERROR = 11,

		CODE_UNSPECIFIED = 100,
		CODE_ALARM,
		CODE_ERROR,
		CODE_ESTOP,
		CODE_NOT_PLAY,
		CODE_NOT_REMOTE,
		CODE_SERVO_OFF,
		CODE_HOLD,
		CODE_NOT_STARTED,
		CODE_WAITING_ROS,
		CODE_SKILLSEND,
		CODE_PFL_ACTIVE,
		CODE_INC_MOVE_ERROR,
	} __attribute__((__packed__));

#ifdef __UNUSED__

	#define ERROR_ENUM(VARIANT)	\
	VARIANT(UNSPECIFIED) \
	VARIANT(ALARM) \
	VARIANT(ERROR) \
	VARIANT(ESTOP) \
	VARIANT(NOT_PLAY) \
	VARIANT(NOT_REMOTE) \
	VARIANT(SERVO_OFF) \
	VARIANT(HOLD) \
	VARIANT(NOT_STARTED) \
	VARIANT(WAITING_ROS) \
	VARIANT(SKILLSEND) \
	VARIANT(PFL_ACTIVE) \
	VARIANT(INC_MOVE_ERROR) 

	#define ERROR_ENUM_VARIANT(NAME) NAME,
	enum RtMsgCode
	{
		ERROR_ENUM(ERROR_ENUM_VARIANT)
	} __attribute__((__packed__));

	// Enable error plotting
	#define ERROR_ENUM_STRING(NAME) case NAME: return #NAME;
	const char *error_name(RtMsgCode code) {
		switch(code) 
		{
			ERROR_ENUM(ERROR_ENUM_STRING)
			default: assert(!"Unknown error");
		}
		return 0;
	}

	// Error value by name
	#define ERROR_ENUM_FROM_STRING(NAME) if (strcmp(string, #NAME) == 0) return NAME;
	RtMsgCode error_from_string(const char *string) 
	{
		ERROR_ENUM(ERROR_ENUM_FROM_STRING)
		assert(!"Unknown error");
		return UNSPECIFIED;
	}

#endif // __UNUSED__

	// ------------------ MESSAGE HEADER ------------------
	struct _RtMsgHeader
	{
		enum RtMsgState msg_state;
		enum RtMsgCode msg_code;
		unsigned char msg_sequence;
		//unsigned char _;
	} __attribute__((packed, aligned(4)));
	typedef struct _RtMsgHeader RtMsgHeader;

	// ------------------ MESSAGE BODY ------------------
	struct _RtMsgBodyCommand
	{
		float pos[RT_ROBOT_JOINTS_MAX];
		float INC_FACTOR;
		float ACC_FACTOR;
	} __attribute__((packed));
	typedef struct _RtMsgBodyCommand RtMsgBodyCommand;

	struct _RtMsgBodyState
	{
		float pos_cmd[RT_ROBOT_JOINTS_MAX];
		float pos_set[RT_ROBOT_JOINTS_MAX];
		float pos_fb[RT_ROBOT_JOINTS_MAX];
		float vel_cmd[RT_ROBOT_JOINTS_MAX];
		float vel_set[RT_ROBOT_JOINTS_MAX];
		float vel_fb[RT_ROBOT_JOINTS_MAX];
		float acc_cmd[RT_ROBOT_JOINTS_MAX];
		float acc_set[RT_ROBOT_JOINTS_MAX];

		// Debug
		unsigned char dbg_axs_sync_state[RT_ROBOT_JOINTS_MAX];
		float dbg_loop_time;

	} __attribute__((packed));
	typedef struct _RtMsgBodyState RtMsgBodyState;

	union RtMsgBody
	{
		RtMsgBodyCommand command[RT_ROBOT_GROUPS];
		RtMsgBodyState state[RT_ROBOT_GROUPS];
	};

	// ------------------ MESSAGE ------------------
	struct _RtMsg
	{
		RtMsgHeader header;
		union RtMsgBody body;
	} __attribute__((packed, aligned(4)));
	typedef struct _RtMsg RtMsg;
	
	// struct _RtMsgCommand
	// {
	// 	RtMsgHeader header;
	// 	RtMsgBodyCommand body;
	// } __attribute__((packed));
	// typedef struct _RtMsgCommand RtMsgCommand;
	
	// struct _RtMsgState
	// {
	// 	RtMsgHeader header;
	// 	RtMsgBodyState body;
	// } __attribute__((packed));
	// typedef struct _RtMsgState RtMsgState;

#ifdef __cplusplus
#pragma pack(pop)
#define RT_MSG_CMD_SIZE (sizeof(udp_rt_message::RtMsgHeader) + sizeof(udp_rt_message::RtMsgBodyCommand)*RT_ROBOT_GROUPS)
#define RT_MSG_STATE_SIZE (sizeof(udp_rt_message::RtMsgHeader) + sizeof(udp_rt_message::RtMsgBodyState)*RT_ROBOT_GROUPS)
#else
#define RT_MSG_CMD_SIZE (sizeof(RtMsgHeader) + sizeof(RtMsgBodyCommand)*RT_ROBOT_GROUPS)
#define RT_MSG_STATE_SIZE (sizeof(RtMsgHeader) + sizeof(RtMsgBodyState)*RT_ROBOT_GROUPS)
#endif // __cplusplus

// assert enum size
static_assert(sizeof(enum RtMsgState) == 1, "RtMsgState must be 1");
static_assert(sizeof(enum RtMsgCode) == 1, "RtMsgCode must be 1");
static_assert(sizeof(RtMsgHeader) == 4, "RtMsgHeader must be 4");
static_assert(sizeof(RtMsg) == RT_MSG_STATE_SIZE, "RtMsg not aligned");

#ifdef __cplusplus
} // motoman_hardware::udp_rt_message
#endif // __cplusplus

#endif // UDP_RT_PROTOCOL_H
