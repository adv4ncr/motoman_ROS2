// RealTimeMotionServer.c
/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2020, Norwegian University of Science and Technology (NTNU)
 * All rights reserved.
 *
 * Redistribution and use in binary form, with or without modification,
 * is permitted provided that the following conditions are met:
 *
 *       * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *       * Neither the name of NTNU, nor the names
 *       of its contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "stdbool.h"
#include "udp_rt_protocol.h"
#include "MotoROS.h"


//-----------------------
// Function Declarations
//-----------------------
// Main Task:
void Ros_RealTimeMotionServer_StartNewConnection(Controller* controller, int sd);
void Ros_RealTimeMotionServer_StopConnection(Controller* controller, int connectionIndex);

// WaitForSimpleMsg Task:
void Ros_RealTimeMotionServer_WaitForSimpleMsg(Controller* controller, int connectionIndex);
int Ros_RealTimeMotionServer_SimpleMsgProcess(Controller* controller, SimpleMsg* receiveMsg, SimpleMsg* replyMsg);
int Ros_RealTimeMotionServer_MotionCtrlProcess(Controller* controller, SimpleMsg* receiveMsg, SimpleMsg* replyMsg);

// Start the real time control client which communicates with an external
// control server
int Ros_RealTimeMotionServer_RealTimeMotionClientStart(Controller* controller);
// Stop the real time control client which communicates with an external control
// server
BOOL Ros_RealTimeMotionServer_RealTimeMotionClientStop(Controller* controller);

// Start the main control loop in real time mode
//void Ros_RealTimeMotionServer_IncMoveLoopStart(Controller* controller);
//void Ros_RealTimeMotionServer_IncMoveLoopStart2(Controller* controller);
static void Ros_RealTimeMotionServer_IncMoveLoopStart(Controller* controller);

// Simple messages
//int Ros_RealTimeMotionServer_SimpleMsg_State(SimpleMsg* stateMsg, CtrlGroup* ctrlGroup, int sequence);

int Ros_RealTimeMotionServer_MotoRealTimeMotionJointStateEx(
		Controller* controller, int messageId, MotoRealTimeMotionMode mode,
		SimpleMsg* sendMsg);

// Handle Controller
BOOL Ros_RealTimeMotionServer_StartTrajMode(Controller* controller);
BOOL Ros_RealTimeMotionServer_StopTrajMode(Controller* controller);
BOOL Ros_RealTimeMotionServer_StopMotion(Controller* controller);

// Utility functions
STATUS Ros_RealTimeMotionServer_DisableEcoMode(Controller* controller);
void Ros_RealTimeMotionServer_PrintError(USHORT err_no, char* msgPrefix);
void Ros_RealTimeMotionServer_PrintPrevPos(Controller* controller);
void Ros_RealTimeMotionServer_PrintParams(Controller* controller);

//-----------------------
// Function implementation
//-----------------------

void Ros_RealTimeMotionServer_StartNewConnection(Controller* controller, int sd)
{
	int connectionIndex;

	printf("[RT] Starting new connection to the Real-Time Motion Server\r\n");

	// Look for next available connection slot. There is only one.
	for (connectionIndex = 0; connectionIndex < MAX_MOTION_CONNECTIONS; connectionIndex++)
	{
		if (controller->sdMotionConnections[connectionIndex] == INVALID_SOCKET)
		{
			controller->sdMotionConnections[connectionIndex] = sd;
			break;
		}
	}

	// Abort if already connected
	if (connectionIndex == MAX_MOTION_CONNECTIONS) {
		puts("[RT] motion server already connected... not accepting last attempt.");
		mpClose(sd);
		return;
	}

	// If not started, start the WaitForSimpleMsg task
	if (controller->tidMotionConnections[connectionIndex] == INVALID_TASK) {
		printf("[RT] Creating new task: tidMotionConnections (connectionIndex = %d)\n", connectionIndex);

		// start new task for this specific connection
		controller->tidMotionConnections[connectionIndex] =
				mpCreateTask(MP_PRI_TIME_NORMAL, MP_STACK_SIZE,
										 (FUNCPTR)Ros_RealTimeMotionServer_WaitForSimpleMsg,
										 (int)controller, connectionIndex, 0, 0, 0, 0, 0, 0, 0, 0);

		if (controller->tidMotionConnections[connectionIndex] != ERROR) {
			Ros_Controller_SetIOState(
					IO_FEEDBACK_MOTIONSERVERCONNECTED,
					TRUE);  // set feedback signal indicating success
		} else {
			puts("[RT] Could not create new task in the motion server.  Check robot parameters.");
			mpClose(sd);
			controller->sdMotionConnections[connectionIndex] = INVALID_SOCKET;
			controller->tidMotionConnections[connectionIndex] = INVALID_TASK;
			Ros_Controller_SetIOState(IO_FEEDBACK_FAILURE, TRUE);
			mpSetAlarm(8004, "MOTOROS FAILED TO CREATE TASK", 6);
			return;
		}
	}
}

//-----------------------------------------------------------------------
// Close a connection along with all its associated task
//-----------------------------------------------------------------------
void Ros_RealTimeMotionServer_StopConnection(Controller* controller, int connectionIndex)
{
	int tid;

	printf("[RT] Closing Real Time Motion Server Connection\r\n");

	//close this connection
	mpClose(controller->sdMotionConnections[connectionIndex]);
	//mark connection as invalid
	controller->sdMotionConnections[connectionIndex] = INVALID_SOCKET;

	//set feedback signal
	Ros_Controller_SetIOState(IO_FEEDBACK_MOTIONSERVERCONNECTED, FALSE);

	// Stop message receiption task
	tid = controller->tidMotionConnections[connectionIndex];
	controller->tidMotionConnections[connectionIndex] = INVALID_TASK;
	printf("[RT] Real Time Motion Server Connection Closed\r\n");

	mpDeleteTask(tid);
}


int Ros_RealTimeMotionServer_GetExpectedByteSizeForMessageType(SimpleMsg* receiveMsg, int recvByteSize)
{
	int minSize = sizeof(SmPrefix) + sizeof(SmHeader);
	int expectedSize;

	switch (receiveMsg->header.msgType)
	{
	case ROS_MSG_MOTO_MOTION_CTRL:
		expectedSize = minSize + sizeof(SmBodyMotoMotionCtrl);
		break;
	case ROS_MSG_MOTO_MOTION_REPLY:
		expectedSize = minSize + sizeof(SmBodyMotoMotionReply);
		break;
	default: //invalid message type
		return -1;
	}
	return expectedSize;
}

//-----------------------------------------------------------------------
// Task that waits to receive new SimpleMessage and then processes it
//-----------------------------------------------------------------------
void Ros_RealTimeMotionServer_WaitForSimpleMsg(Controller* controller, int connectionIndex)
{
	printf("[RT] wait for simple message.\r\n"); // #TODO remove
	SimpleMsg receiveMsg;
	SimpleMsg replyMsg;
	int byteSize = 0, byteSizeResponse = 0;
	int minSize = sizeof(SmPrefix) + sizeof(SmHeader);
	int expectedSize;
	int ret = 0;
	BOOL bDisconnect = FALSE;
	int partialMsgByteCount = 0;
	BOOL bSkipNetworkRecv = FALSE;

	while(!bDisconnect) //keep accepting messages until connection closes
	{
		Ros_Sleep(0);	//give it some time to breathe, if needed

		if (!bSkipNetworkRecv) //if I don't already have an extra complete packet buffered from the previous recv
		{
			if (partialMsgByteCount) //partial (incomplete) message already received
			{
				//Receive message from the PC
				memset((&receiveMsg) + partialMsgByteCount, 0x00, sizeof(SimpleMsg) - partialMsgByteCount);
				byteSize = mpRecv(controller->sdMotionConnections[connectionIndex], (char*)((&receiveMsg) + partialMsgByteCount), sizeof(SimpleMsg) - partialMsgByteCount, 0);
				if (byteSize <= 0)
					break; //end connection

				byteSize += partialMsgByteCount;
				partialMsgByteCount = 0;
			}
			else //get whole message
			{
				//Receive message from the PC
				memset(&receiveMsg, 0x00, sizeof(receiveMsg));
				byteSize = mpRecv(controller->sdMotionConnections[connectionIndex], (char*)(&receiveMsg), sizeof(SimpleMsg), 0);
				if (byteSize <= 0)
					break; //end connection
			}
		}
		else
		{
			byteSize = partialMsgByteCount;
			partialMsgByteCount = 0;
			bSkipNetworkRecv = FALSE;
		}

		// Determine the expected size of the message
		expectedSize = -1;
		if(byteSize >= minSize)
		{
			expectedSize = Ros_RealTimeMotionServer_GetExpectedByteSizeForMessageType(&receiveMsg, byteSize);

			if (expectedSize == -1)
			{
				printf("[RT] Unknown Message Received (%d)\r\n", receiveMsg.header.msgType);
				Ros_SimpleMsg_MotionReply(&receiveMsg, ROS_RESULT_INVALID, ROS_RESULT_INVALID_MSGTYPE, &replyMsg, 0);
			}
			else if (byteSize >= expectedSize) // Check message size
			{
				// Process the simple message
				ret = Ros_RealTimeMotionServer_SimpleMsgProcess(controller, &receiveMsg, &replyMsg);
				if (ret == 1) //error during processing
				{
					bDisconnect = TRUE;
				}
				else if (byteSize > expectedSize) // Received extra data in single message
				{
					////Special case where ROS_MSG_MOTO_JOINT_TRAJ_PT_FULL_EX message could have different lengths - this never happens in RT mode
					//if (receiveMsg.header.msgType == ROS_MSG_MOTO_JOINT_TRAJ_PT_FULL_EX &&
					//	byteSize == (int)(minSize + sizeof(SmBodyJointTrajPtFullEx)))
					//{
					//	// All good
					//	partialMsgByteCount = 0;
					//}
					//else
					//{

					// Preserve the remaining bytes and treat them as the start of a new message
					Db_Print("[RT] MessageReceived(%d bytes): expectedSize=%d, processing rest of bytes (%d, %d, %d)\r\n", byteSize, expectedSize, sizeof(receiveMsg), receiveMsg.body.jointTrajData.sequence, ((int*)((char*)&receiveMsg + expectedSize))[5]);
					partialMsgByteCount = byteSize - expectedSize;
					memmove(&receiveMsg, (char*)&receiveMsg + expectedSize, partialMsgByteCount);

					//Did I receive multiple full messages at once that all need to be processed before listening for new data?
					if (partialMsgByteCount >= minSize)
					{
						expectedSize = Ros_RealTimeMotionServer_GetExpectedByteSizeForMessageType(&receiveMsg, partialMsgByteCount);
						bSkipNetworkRecv = (partialMsgByteCount >= expectedSize); //does my modified receiveMsg buffer contain a full message to process?
					}
					//}
				}
				else // All good
					partialMsgByteCount = 0;
			}
			else // Not enough data to process the command
			{
				Db_Print("[RT]MessageReceived(%d bytes): expectedSize=%d\r\n", byteSize, expectedSize);
				Ros_SimpleMsg_MotionReply(&receiveMsg, ROS_RESULT_INVALID, ROS_RESULT_INVALID_MSGSIZE, &replyMsg, 0);
			}
		}
		else // Didn't even receive a command ID
		{
			Db_Print("[RT] Unknown Data Received (%d bytes)\r\n", byteSize);
			Ros_SimpleMsg_MotionReply(&receiveMsg, ROS_RESULT_INVALID, ROS_RESULT_INVALID_MSGSIZE, &replyMsg, 0);
		}

		//Send reply message
		byteSizeResponse = mpSend(controller->sdMotionConnections[connectionIndex], (char*)(&replyMsg), replyMsg.prefix.length + sizeof(SmPrefix), 0);
		if (byteSizeResponse <= 0)
			break;	// Close the connection
	}

	Ros_Sleep(50);	// Just in case other associated task need time to clean-up.

	//close this connection
	Ros_RealTimeMotionServer_StopConnection(controller, connectionIndex);
}

//-----------------------------------------------------------------------
// Checks the type of message and processes it accordingly
// Return -1=Failure; 0=Success; 1=CloseConnection;
//-----------------------------------------------------------------------
int Ros_RealTimeMotionServer_SimpleMsgProcess(Controller* controller, SimpleMsg* receiveMsg, SimpleMsg* replyMsg)
{
	int ret = 0;
	int invalidSubcode = 0;

	switch(receiveMsg->header.msgType)
	{
	case ROS_MSG_MOTO_MOTION_CTRL:
		ret = Ros_RealTimeMotionServer_MotionCtrlProcess(controller, receiveMsg, replyMsg);
		break;

	//-----------------------
	default:
		printf("[RT] Invalid message type: %d\n", receiveMsg->header.msgType);
		invalidSubcode = ROS_RESULT_INVALID_MSGTYPE;
		break;
	}

	// Check Invalid Case #TODO can be moved to default
	if(invalidSubcode != 0)
	{
		Ros_SimpleMsg_MotionReply(receiveMsg, ROS_RESULT_INVALID, invalidSubcode, replyMsg, 0);
		ret = -1;
	}

	return ret;
}

//-----------------------------------------------------------------------
// Processes message of type: ROS_MSG_MOTO_MOTION_CTRL
// Return -1=Failure; 0=Success; 1=CloseConnection;
//-----------------------------------------------------------------------
int Ros_RealTimeMotionServer_MotionCtrlProcess(Controller* controller, SimpleMsg* receiveMsg, SimpleMsg* replyMsg)
{
	SmBodyMotoMotionCtrl* motionCtrl;

	//printf("In MotionCtrlProcess\r\n");

	// Check the command code
	motionCtrl = &receiveMsg->body.motionCtrl;
	Db_Print("[RT] MotionCtrl: groupNo: %d sequence: %d command: %d\n", motionCtrl->groupNo, motionCtrl->sequence, motionCtrl->command);

	switch(motionCtrl->command)
	{
		case ROS_CMD_START_RT_MODE:
		{
			int Ret = Ros_RealTimeMotionServer_RealTimeMotionClientStart(controller);
			// Reply msg
			if(Ret == 0)
				Ros_SimpleMsg_MotionReply(receiveMsg, ROS_RESULT_SUCCESS, 0, replyMsg, receiveMsg->body.motionCtrl.groupNo);
			else if(Ret == 1)
				Ros_SimpleMsg_MotionReply(receiveMsg, ROS_RESULT_BUSY, 0, replyMsg, receiveMsg->body.motionCtrl.groupNo);
			else
				Ros_SimpleMsg_MotionReply(receiveMsg, ROS_RESULT_FAILURE, 0, replyMsg, receiveMsg->body.motionCtrl.groupNo);
			break;
		}
		case ROS_CMD_STOP_RT_MODE:
		{
			// Stop Motion
			BOOL bRet = Ros_RealTimeMotionServer_RealTimeMotionClientStop(controller);
			// Reply msg
			if(bRet)
				Ros_SimpleMsg_MotionReply(receiveMsg, ROS_RESULT_SUCCESS, 0, replyMsg, receiveMsg->body.motionCtrl.groupNo);
			else
				Ros_SimpleMsg_MotionReply(receiveMsg, ROS_RESULT_FAILURE, 0, replyMsg, receiveMsg->body.motionCtrl.groupNo);
			break;
		}
		default:
			printf("[RT] invalid motionCtrl command: %d", motionCtrl->command);
			break;
	}
	return 0;
}

//-----------------------------------------------------------------------
// Start the real time control client which communicates with an external
// control server
// Return -1=Failure; 0=Success; 1=AlreadyRunning;
//-----------------------------------------------------------------------
int Ros_RealTimeMotionServer_RealTimeMotionClientStart(
		Controller* controller) {
	if (controller->tidIncMoveThread == INVALID_TASK) {
		puts("[RT] Creating new task: RealTimeMotionServer_IncMoveLoop");

		controller->tidIncMoveThread =
				mpCreateTask(MP_PRI_IP_CLK_TAKE, MP_STACK_SIZE,
										 (FUNCPTR)Ros_RealTimeMotionServer_IncMoveLoopStart, // #TODO
										 (int)controller, 0, 0, 0, 0, 0, 0, 0, 0, 0);
		if (controller->tidIncMoveThread == ERROR) {
			puts("[RT] Failed to create task for real time motion.  Check robot parameters.");
			controller->tidIncMoveThread = INVALID_TASK;
			Ros_Controller_SetIOState(IO_FEEDBACK_FAILURE, TRUE);
			mpSetAlarm(8004, "MOTOROS FAILED TO CREATE TASK", 4);
			return -1;
		}
		return 0;
	}
	puts("[RT] RealTimeMotionServer_IncMoveLoop task already started!");
	return 1;
}

//-----------------------------------------------------------------------
// Stop the real time control client which communicates with an external
// control server
//-----------------------------------------------------------------------
BOOL Ros_RealTimeMotionServer_RealTimeMotionClientStop(Controller* controller) {
	BOOL bStopped = FALSE;
	// Signal that the motion should be stopped
	controller->bStopMotion = TRUE;
	// Check that the motion has in fact been stopped
	static int checkCnt;
	for (checkCnt = 0; checkCnt < MOTION_STOP_TIMEOUT; checkCnt++) {
		if (controller->tidIncMoveThread == INVALID_TASK) {
			bStopped = TRUE;
			// All motion should be stopped at this point, so turn of the flag
			controller->bStopMotion = FALSE;
			break;
		}
		Ros_Sleep(1);
	}
	return bStopped;
}

// ------------------ TYPE DECLARATIONS  ------------------

// Requested motion states
enum Ros_RealTimeMotionServer_Motion_t
{
	RT_MOTION_UNDEFINED = 0,
	RT_MOTION_SYNC = 1,
	RT_MOTION_STOP = 2,
	RT_MOTION_STALL,
	RT_MOTION_OUT_OF_SYNC,
	RT_MOTION_ERROR,
} __attribute__((__packed__));

// Internal synchronisation states
enum Ros_RealTimeMotionServer_SycStatus_t
{
    RT_SYNCSTATE_INIT_0 = 0,
    RT_SYNCSTATE_INIT_1 = 1,
    RT_SYNCSTATE_OUT_OF_SYNC = 2,
    RT_SYNCSTATE_SYNCHRONIZING = 3,
    RT_SYNCSTATE_IN_SYNC = 4,
} __attribute__((__packed__));

// Axis motion data for one control group
typedef struct
{
    long prevPosCmd[RT_ROBOT_JOINTS_MAX];
    long prevPosSet[RT_ROBOT_JOINTS_MAX];
    long prevIncCmd[RT_ROBOT_JOINTS_MAX];
    long prevIncSet[RT_ROBOT_JOINTS_MAX];
    long prevAccCmd[RT_ROBOT_JOINTS_MAX];
    long prevAccSet[RT_ROBOT_JOINTS_MAX];
    double velSet[RT_ROBOT_JOINTS_MAX];
	enum Ros_RealTimeMotionServer_SycStatus_t state[RT_ROBOT_JOINTS_MAX];
} MoveData;


// ------------------ STATIC GLOBAL VARIABLES  ------------------
static UINT8 groupNo;							// Robot group number
static UINT8 axisNo;							// Robot axis number
//static long pulsePos[RT_ROBOT_GROUPS][RT_ROBOT_JOINTS_MAX];			// Pulse position array
//static long pulsePosCmd[RT_ROBOT_GROUPS][RT_ROBOT_JOINTS_MAX];		// Pulse position command array
//static long pulseInc[RT_ROBOT_GROUPS][RT_ROBOT_JOINTS_MAX];			// Pulse increment array
static MoveData mData;
static MP_EXPOS_DATA incData;					// Incremental move data
//static enum Ros_RealTimeMotionServer_Motion_t motionRequest, motionResponse;

// connection variables
static struct sockaddr_in serverAddr, clientAddr;
static int sizeofClientAddr;
//static int bytesSend, bytesRecv;				// UDP sent / received bytes
static struct timeval readTimeout;				// Define UDP read timeout
static struct fd_set readFds;					// struct for handling socket file descriptor state
static int udp_socket_fd;						// UDP server socket
static RtMsg rtMsgRecv, rtMsgSend;				// Message variable

#define MAX_INC_FALLBACK 0.1
#define MAX_ACC_FALLBACK 0.01

// Message queue
typedef struct
{
	#define MSG_CODE_QUEUE_SIZE 100
	enum RtMsgCode code[MSG_CODE_QUEUE_SIZE];
	int counter;
} MsgCodeQueue;

static MsgCodeQueue msgCodeQueue;

// Add fail codes to the message code queue
void _add_msg_code_queue(enum RtMsgCode code)
{
	if(msgCodeQueue.counter < MSG_CODE_QUEUE_SIZE)
	{
		msgCodeQueue.code[msgCodeQueue.counter] = code;
		msgCodeQueue.counter++;
	}
	else { Db_Print("[RT] _add_msg_code_queue: QUEUE FULL\n"); }
}

// Get code from message code queue and set to RtMsg header code
void _get_next_msg_code_queue(enum RtMsgCode * msg_code)
{
	if(msgCodeQueue.counter > 0)
	{
		msgCodeQueue.counter -= 1;
		(*msg_code) = msgCodeQueue.code[msgCodeQueue.counter];
	}
	else
	{
		(*msg_code) = CODE_UNDEFINED;
	}
}

// Get motion limits (increments) from UDP message
void _get_motion_limits(RtMsg * msg, UINT8 grpNo, float * INC_MAX, float * ACC_MAX)
{
	// Check and set inc limit
	if(msg->body.command[grpNo].INC_FACTOR > 0. && msg->body.command[grpNo].INC_FACTOR <= 1.)
	{
		(*INC_MAX) = msg->body.command[grpNo].INC_FACTOR;
	}
	else
	{
		(*INC_MAX) = MAX_INC_FALLBACK;
		_add_msg_code_queue(CODE_MAX_INC_ERROR);
	}

	// Check and set acc limit
	if(msg->body.command[grpNo].ACC_FACTOR > 0. && msg->body.command[grpNo].ACC_FACTOR <= 1.)
	{
		(*ACC_MAX) = msg->body.command[grpNo].ACC_FACTOR;
	}
	else
	{
		(*ACC_MAX) = MAX_ACC_FALLBACK;
		_add_msg_code_queue(CODE_MAX_ACC_ERROR);
	}
}

// Test variables
static float sw_current;
static bool DEMO_MODE = false;

// #TODO remove
static MoveData _mDataLog; // #TODO remove
static void _update_log_msg_data()
{
	for(axisNo=0; axisNo<RT_ROBOT_JOINTS_MAX; axisNo++)
	{
		_mDataLog.prevPosCmd[axisNo] = mData.prevPosCmd[axisNo];
		_mDataLog.prevPosSet[axisNo] = mData.prevPosSet[axisNo];
		_mDataLog.prevIncCmd[axisNo] = mData.prevIncCmd[axisNo] * 250;			// Conversion step - s
		_mDataLog.prevIncSet[axisNo] = mData.prevIncSet[axisNo] * 250;			// ...
		_mDataLog.prevAccCmd[axisNo] = mData.prevAccCmd[axisNo] * 250 * 250;	// Conversion step² - s²
 		_mDataLog.prevAccSet[axisNo] = mData.prevAccSet[axisNo] * 250 * 250;	// ...
		_mDataLog.velSet[axisNo] = mData.velSet[axisNo];
		_mDataLog.state[axisNo] = mData.state[axisNo];
	}
}

static void Ros_RealTimeMotionServer_SetState(enum RtMsgState new_state, enum RtMsgState *state, bool *entry)
{
	Db_Print("[RT] SET STATE: %d\n", new_state);
	*entry = true;
	*state = new_state;
}

static BOOL _Ros_RealTimeMotionServer_SendUdpMsg()
{
	static int bytesSend;
	bytesSend = mpSendTo(udp_socket_fd, (char*)&rtMsgSend, RT_MSG_STATE_SIZE, 0, (struct sockaddr*)&clientAddr, sizeof(clientAddr));
	if (bytesSend < 0)
	{
		printf("[RT] mpSendTo ERRNO: %d\n", errno);
		return FALSE;
	}
	return TRUE;
}

static BOOL _Ros_RealTimeMotionServer_GetUdpMsg()
{
	static int bytesRecv;

	FD_ZERO(&readFds);
	FD_SET(udp_socket_fd, &readFds);

	// clear message
	memset(&rtMsgRecv, CLEAR, RT_MSG_CMD_SIZE);

	// wait for message
	if (mpSelect(udp_socket_fd + 1, &readFds, NULL, NULL, &readTimeout) > 0)
	{
		// receive message
		bytesRecv = mpRecvFrom(udp_socket_fd, (char*)&rtMsgRecv, RT_MSG_CMD_SIZE, 0, (struct sockaddr *)&clientAddr, &sizeofClientAddr);
		if (bytesRecv < 0) 
		{
			printf("[RT] mpRecvFrom ERRNO: %d\n", errno);
			return FALSE;
		}
		return TRUE;
	}

	// timeout 
	return FALSE;
}

static BOOL _Ros_RealTimeMotionServer_SetStateMsg(Controller* controller, 
	const enum RtMsgState *msg_state, const unsigned char *sequence )
{
	static long _pulse_values[MAX_PULSE_AXES];

	// initialize memory
	memset(&rtMsgSend, CLEAR, RT_MSG_STATE_SIZE);

	// set header
	rtMsgSend.header.msg_state = *msg_state;
	_get_next_msg_code_queue(&rtMsgSend.header.msg_code);
	rtMsgSend.header.msg_sequence = *sequence;
	//rtMsgSend.header.msg_

	// set number of valid groups
	// sendMsg->body.realTimeMotionJointStateEx.numberOfValidGroups =
	// 		controller->numGroup;

	// Populate the state of all control groups
	for (groupNo = 0; groupNo < controller->numGroup; groupNo++) 
	{
		// sendMsg->body.realTimeMotionJointStateEx.jointStateData[groupNo].groupNo =
		// 		groupNo;

		// ------------------ POSITION ------------------

		// Commanded position
		Ros_CtrlGroup_ConvertToRosPos(controller->ctrlGroups[groupNo], _mDataLog.prevPosCmd, rtMsgSend.body.state[groupNo].pos_cmd);

		// Set position
		if (DEMO_MODE)
		{
			Ros_CtrlGroup_ConvertToRosPos(controller->ctrlGroups[groupNo], _mDataLog.prevPosSet, rtMsgSend.body.state[groupNo].pos_set);
		}
		else
		{
			if (!Ros_CtrlGroup_GetPulsePosCmd(controller->ctrlGroups[groupNo], _pulse_values)) 
			{
				Db_Print("[RT] Ros_CtrlGroup_GetPulsePosCmd error\n");
				return FALSE;
			}
			Ros_CtrlGroup_ConvertToRosPos(controller->ctrlGroups[groupNo], _pulse_values, rtMsgSend.body.state[groupNo].pos_set);
		}


		// Feedback position
		if (!Ros_CtrlGroup_GetFBPulsePos(controller->ctrlGroups[groupNo], _pulse_values)) 
		{
			Db_Print("[RT] Ros_CtrlGroup_GetFBPulsePos error\n");
			return FALSE;
		}
		Ros_CtrlGroup_ConvertToRosPos(controller->ctrlGroups[groupNo], _pulse_values, rtMsgSend.body.state[groupNo].pos_fb);


		// ------------------ VELOCITY ------------------

		// // Set speed -> useless function, delayed and parse results
		// if (!Ros_CtrlGroup_GetServoSpeed(controller->ctrlGroups[groupNo], _pulse_values)) 
		// {
		// 	Db_Print("[RT] Ros_CtrlGroup_GetServoSpeed error\n");
		// 	return FALSE;
		// }
		// if (!DEMO_MODE)
		// {
		// 	Ros_CtrlGroup_ConvertToRosPos(controller->ctrlGroups[groupNo], _pulse_values, rtMsgSend.body.state[groupNo].vel_set);
		// }

		// Commanded speed
		Ros_CtrlGroup_ConvertToRosPos(controller->ctrlGroups[groupNo], _mDataLog.prevIncCmd, rtMsgSend.body.state[groupNo].vel_cmd);
		
		// Set speed
		Ros_CtrlGroup_ConvertToRosPos(controller->ctrlGroups[groupNo], _mDataLog.prevIncSet, rtMsgSend.body.state[groupNo].vel_set);

		// Feedback speed
		if(!Ros_CtrlGroup_GetFBServoSpeed(controller->ctrlGroups[groupNo], _pulse_values))
		{
			Db_Print("[RT] Ros_CtrlGroup_GetFBServoSpeed error\n");
			return FALSE;	
		}
		Ros_CtrlGroup_ConvertToRosPos(controller->ctrlGroups[groupNo], _pulse_values, rtMsgSend.body.state[groupNo].vel_fb);
		
		
		// ------------------ ACCELERATION ------------------
		
		// Commanded acceleration
		Ros_CtrlGroup_ConvertToRosPos(controller->ctrlGroups[groupNo], _mDataLog.prevAccCmd, rtMsgSend.body.state[groupNo].acc_cmd);

		// Set acceleration
		Ros_CtrlGroup_ConvertToRosPos(controller->ctrlGroups[groupNo], _mDataLog.prevAccSet, rtMsgSend.body.state[groupNo].acc_set);


		// ------------------ ROBOT STATE ------------------

		// #TODO necessary after testing?
		for(axisNo = 0; axisNo < RT_ROBOT_JOINTS_MAX; axisNo++)
		{
			rtMsgSend.body.state[groupNo].dbg_axs_sync_state[axisNo] = (unsigned char)mData.state[axisNo];
		}
		
		rtMsgSend.body.state[groupNo].dbg_loop_time = sw_current;


	}

	return TRUE;
}

/*
Set robot motion.
Returns: Ros_RealTimeMotionServer_Motion_t
*/
static UINT8 _Ros_RealTimeMotionServer_SetMotion(Controller* controller, enum Ros_RealTimeMotionServer_Motion_t motionType)
{
	const UINT8 groupNo = 0;	// Only for 1 controller group for now

	// Maximum values
	static float MAX_INC_FACTOR = MAX_INC_FALLBACK, MAX_INC_FACTOR_PREV = MAX_INC_FALLBACK;
	static float MAX_ACC_FACTOR = MAX_ACC_FALLBACK, MAX_ACC_FACTOR_PREV = MAX_ACC_FALLBACK;

	// Local move data variables
	static long posCmd[RT_ROBOT_JOINTS_MAX];
	static long _pos_cmd = 0;
	static long _inc_cmd = 0;
	static long _inc_set = 0;
	static UINT16 _INC_MAX = 0;
	static long _acc_cmd = 0;
	static long _acc_set = 0;
	static UINT16 _ACC_MAX = 0;
	static double _acc_clc = 0;
	static double _acc_itc = 0;
	static double _numerator, _denominator;

	// Return value
	static enum Ros_RealTimeMotionServer_Motion_t RET;
	RET = RT_MOTION_UNDEFINED;
	
	// ------------------ SET COMMAND INCREMENT ------------------

	if(motionType == RT_MOTION_SYNC)
	{	
		// Convert command [RAD] values to [PULSE]
		Ros_CtrlGroup_ConvertToMotoPos(controller->ctrlGroups[groupNo], rtMsgRecv.body.command[groupNo].pos, posCmd);
		
		// Get new motion limits
		_get_motion_limits(&rtMsgRecv, groupNo, &MAX_INC_FACTOR, &MAX_ACC_FACTOR);
		if(MAX_INC_FACTOR != MAX_INC_FACTOR_PREV)
		{
			Db_Print("[RT] set MAX_INC_FACTOR: %f\n", MAX_INC_FACTOR);
			MAX_INC_FACTOR_PREV = MAX_INC_FACTOR;
		}
		if(MAX_ACC_FACTOR != MAX_ACC_FACTOR_PREV)
		{
			Db_Print("[RT] set MAX_ACC_FACTOR: %f\n", MAX_ACC_FACTOR);
			MAX_ACC_FACTOR_PREV = MAX_ACC_FACTOR;
		}
	}

	// ------------------ CALCULATE MOTION ------------------
	
	for (axisNo = 0; axisNo < controller->ctrlGroups[groupNo]->numAxes; axisNo++)
	{

		// Set commanded position
		switch (motionType)
		{
		case RT_MOTION_SYNC:

			// Position by new message
			_pos_cmd = posCmd[axisNo];
			// Set desired increment
			_inc_cmd = _pos_cmd - mData.prevPosCmd[axisNo];
			_inc_set = _pos_cmd - mData.prevPosSet[axisNo];
			break;

		case RT_MOTION_STALL:

			// Position by last increment
			_pos_cmd = mData.prevPosCmd[axisNo] + mData.prevIncCmd[axisNo];
			// Set desired increment
			_inc_cmd = _pos_cmd - mData.prevPosCmd[axisNo];
			_inc_set = _pos_cmd - mData.prevPosSet[axisNo];
			break;

		case RT_MOTION_STOP:

			// Position by last SET position (0 increment)
			_pos_cmd = mData.prevPosSet[axisNo];
			_inc_cmd = 0;
			_inc_set = 0;
			break;

		default:
			printf("[RT] SetMotion ERROR: undefined requested state %d\r\n", motionType);
			return RT_MOTION_ERROR;
			break;
		}

		// Synchronize only if RT_SYNCSTATE not in INIT_X
		if(mData.state[axisNo] > RT_SYNCSTATE_INIT_1)
		{
			// Set desired acceleration
			_acc_cmd = _inc_cmd - mData.prevIncCmd[axisNo];
			_acc_set = _inc_set - mData.prevIncSet[axisNo];

			// Get maximum values
			_INC_MAX = (UINT16) (controller->ctrlGroups[groupNo]->maxInc.maxIncrement[axisNo] * MAX_INC_FACTOR);
			_ACC_MAX = (UINT16) (2 * controller->ctrlGroups[groupNo]->maxInc.maxIncrement[axisNo] * MAX_ACC_FACTOR);

			// --------- Approximate Sync ---------
			
			// First assumption
			static bool _in_sync;
			_in_sync = true;

			// Check increment limit
			if (abs(_inc_set) > _INC_MAX)
			{
				_in_sync = false;
				_inc_set = _inc_set > 0 ? _INC_MAX : -_INC_MAX;
				_acc_set = _inc_set - mData.prevIncSet[axisNo];
			}

			// Check acceleration limit
			if (abs(_acc_set) > _ACC_MAX)
			{
				_in_sync = false;
				_acc_set = _acc_set > 0 ? _ACC_MAX : -_ACC_MAX;
				_inc_set = mData.prevIncSet[axisNo] + _acc_set;
			}

			// Check acceleration limit after one extrapolated step
			if (abs(_inc_cmd-_inc_set) > _ACC_MAX)
			{
				_in_sync = false;
			}

			// Set state
			if (_in_sync) {mData.state[axisNo] = RT_SYNCSTATE_IN_SYNC;}
			else if (mData.state[axisNo] == RT_SYNCSTATE_IN_SYNC)
			{
				mData.state[axisNo] = RT_SYNCSTATE_OUT_OF_SYNC;
			}

			// --------- Out of sync ---------
			if (mData.state[axisNo] == RT_SYNCSTATE_OUT_OF_SYNC)
			{
				_acc_clc = _inc_set > _inc_cmd ? -_ACC_MAX : _ACC_MAX;
				//_acc_clc *= 1.0;		// #TODO set factor here?

				
				// Check synchronization condition
				
				// Prevent division by 0; Calculate denominator first
				_denominator = 2*((mData.prevPosSet[axisNo]+_inc_set) - (_pos_cmd));

				if (_denominator != 0)
				{
					_numerator = (_inc_set + _acc_set/2.) - (_inc_cmd + _acc_cmd/2.);
					_acc_itc = _numerator*_numerator / _denominator;	// interception acceleration
				}
				else {_acc_itc = 0;}

				// Check synchronisation start condition
				if((_acc_clc > 0 && _acc_itc > _acc_clc) || (_acc_clc < 0 && _acc_itc < _acc_clc))
				{
					mData.state[axisNo] = RT_SYNCSTATE_SYNCHRONIZING;
					// Set start velocity for synchronisation phase
					mData.velSet[axisNo] = (_inc_set + mData.prevIncSet[axisNo])/2.0;
				}
			}

			// --------- Synchronizing ---------
			if (mData.state[axisNo] == RT_SYNCSTATE_SYNCHRONIZING)
			{
				// Prevent division by 0; Calculate denominator first
				_denominator = 2*((mData.prevPosSet[axisNo]) - (mData.prevPosCmd[axisNo]));

				if (_denominator != 0)
				{
					_numerator = (mData.velSet[axisNo]) - (_inc_cmd - _acc_cmd/2.);
					_acc_itc = _numerator*_numerator / _denominator;	// interception acceleration
				}
				else {_acc_itc = 0;}

				// Exit if not expedient
				// Condition: ((_inc_set > _inc_cmd) XOR (_acc_itc < 0)) OR _acc_itc < 1
				if ( (((mData.velSet[axisNo] + _acc_itc/2.) > _inc_cmd) != (_acc_itc < 0)) || abs(_acc_itc) < 1)
				{
					//printf("[RT] [EXIT_SYC] axisNo: %d", axisNo);
					mData.state[axisNo] = RT_SYNCSTATE_OUT_OF_SYNC;
				}
				// Else set new interception values
				else 
				{
					_inc_set = mData.velSet[axisNo] + _acc_itc/2.;
					_acc_set = _inc_set - mData.prevIncSet[axisNo];
					mData.velSet[axisNo] += _acc_itc;

					// Check max. increment
					if (abs(_inc_set) > _INC_MAX)
					{
						_inc_set = _inc_set > 0 ? _INC_MAX : -_INC_MAX;
						_acc_set = _inc_set - mData.prevIncSet[axisNo];
						mData.velSet[axisNo] = _inc_set + _acc_set/2;
					}
					// Check max. acceleration
					if (abs(_acc_set) > _ACC_MAX)
					{
						_acc_set = _acc_set > 0 ? _ACC_MAX : -_ACC_MAX;
						_inc_set = mData.prevIncSet[axisNo] + _acc_set;
						mData.velSet[axisNo] = _inc_set + _acc_set/2;
					}
				}
			}
		}
		else // mData.state[axisNo] <= RT_SYNCSTATE_INIT_1
		{
			// Set increment by last increment
			_inc_set = mData.prevIncSet[axisNo];
			// Next state (diry solution)
			mData.state[axisNo]++;
		}

		// --------- Set robot increment ---------

		// Set increment to actual robot move variable
		//if(axisNo == 5) // TEST #TODO REMOVE
		//{
		incData.grp_pos_info[groupNo].pos[axisNo] = _inc_set;
		//}

		// --------- Set previous values ---------
		mData.prevPosCmd[axisNo] = _pos_cmd;
		mData.prevIncCmd[axisNo] = _inc_cmd;
		mData.prevPosSet[axisNo] += _inc_set;
		mData.prevIncSet[axisNo] = _inc_set;
		mData.prevAccCmd[axisNo] = _acc_cmd;
		mData.prevAccSet[axisNo] = _acc_set;


	}

	// ------------------ SET RETURN VALUE ------------------
	for (axisNo = 0; axisNo < controller->ctrlGroups[groupNo]->numAxes; axisNo++)
	{
		switch (motionType)
		{
		case RT_MOTION_SYNC:
			if(RET < RT_MOTION_OUT_OF_SYNC && mData.state[axisNo] != RT_SYNCSTATE_IN_SYNC) 
				{ RET = RT_MOTION_OUT_OF_SYNC; }
			else if(RET < RT_MOTION_SYNC)
				{ RET = RT_MOTION_SYNC; }
			break;

		case RT_MOTION_STOP:
			if(RET < RT_MOTION_OUT_OF_SYNC && mData.state[axisNo] != RT_SYNCSTATE_IN_SYNC) 
				{ RET = RT_MOTION_OUT_OF_SYNC; }
			else if(RET < RT_MOTION_STOP && mData.prevIncSet == 0) 
				{ RET = RT_MOTION_STOP; }
			else if(RET < RT_MOTION_SYNC)
				{ RET = RT_MOTION_SYNC; }
			break;

		case RT_MOTION_STALL:
			if(RET < RT_MOTION_OUT_OF_SYNC && mData.state[axisNo] != RT_SYNCSTATE_IN_SYNC) 
				{ RET = RT_MOTION_OUT_OF_SYNC; }
			else if(RET < RT_MOTION_STALL)
				{ RET = RT_MOTION_STALL; }
			break;
		
		default:
			break;
		}
	}

	// ------------------ TEST PRINT #TODO remove ------------------
	// TEST #TODO REMOVE
	static BOOL _printInc;
	_printInc = FALSE;
	for (axisNo = 0; axisNo < controller->ctrlGroups[groupNo]->numAxes; axisNo++)
		if(mData.prevIncSet[axisNo] != 0) _printInc = TRUE;
	if(_printInc)
		printf("[RT] P: %ld %ld %ld %ld %ld %ld\n",
			mData.prevIncSet[0], mData.prevIncSet[1], mData.prevIncSet[2], 
			mData.prevIncSet[3], mData.prevIncSet[4], mData.prevIncSet[5]);



	// ------------------ MOVE ROBOT ------------------

	if (Ros_Controller_IsMotionReady(controller) && !controller->bStopMotion) 
	{
		static int mpRet;
		mpRet = mpExRcsIncrementMove(&incData);
		// Error
		if (mpRet != 0) {
			if (mpRet == -3) 
			{
				printf("[RT] mpExRcsIncrementMove ERROR: %d (ctrl_grp = % ld)\r\n",
					mpRet, incData.ctrl_grp);
			}
			else 
			{
				printf("[RT] mpExRcsIncrementMove ERROR: %d\r\n", mpRet);
			}
			RET = RT_MOTION_ERROR;
		}
		// Normal end
		else {}
	}

	return RET;
}


//-----------------------------------------------------------------------
// Real time Incremental Move Loop -- main
//-----------------------------------------------------------------------
static void Ros_RealTimeMotionServer_IncMoveLoopStart(Controller* controller)
{
	// Consistency check of defined udp_rt_protocol macros and physical setup
	if(controller->numGroup != RT_ROBOT_GROUPS)
	{
		printf("[RT] %d control groups configured. Supported: %d\n", controller->numGroup, RT_ROBOT_GROUPS);
		puts("[RT] Ask your friendly local software engineer to upgrade the code to support multiple control groups");
		goto exitTask;
	}
	for (groupNo = 0; groupNo < controller->numGroup; groupNo++) 
	{
		if(controller->ctrlGroups[groupNo]->numAxes > RT_ROBOT_JOINTS_MAX)
		{
			printf("[RT] Control group %d has %d axes. Supported: %d\n",
				groupNo, controller->ctrlGroups[groupNo]->numAxes, RT_ROBOT_JOINTS_MAX);
			puts("[RT] Change udp_rt_protocol.h and recompile");
			goto exitTask;
		}
	}

	// ------------------ MANAGE VARIABLES ------------------

	// Declare scoped static variables
	//static int i;
	static bool msgReceived;							// Received message from host
	//static enum RtMsgCode msgCode = CODE_UNDEFINED;		// Message code sent to host
	static unsigned short timeoutCounter = 0;			// Missed messages
	static unsigned char sequence = 0;					// Current message sequence
	static enum Ros_RealTimeMotionServer_Motion_t motionRet = RT_MOTION_UNDEFINED;
	// state machine variables
	static enum RtMsgState rtState;
	static bool stateEntry;

	// Stop watch #TODO remove after testing
	void * stopWatch = mpStopWatchCreate(251);	// stopWatch
	mpStopWatchReset(stopWatch);				// stopWatch
	//mpStopWatchStart(stopWatch);				// stopWatch
	sw_current = 0;
	//static UINT8 sw_counter  = 0;
	//static float sw_results[250];
	//static double sw_final_result = 0;
	//memset(sw_results, 0, 250);

	

	
	// Clear data structures
	for (groupNo = 0; groupNo < controller->numGroup; groupNo++)
	{
		memset(&mData, 0, sizeof(MoveData));
		memset(&_mDataLog, 0, sizeof(MoveData));	// #TODO remove

		// Clear msg code queue
		memset(&msgCodeQueue, 0, sizeof(MsgCodeQueue));	
	}

	// Set movedata
	memset(&incData, CLEAR, sizeof(MP_EXPOS_DATA));
	for (groupNo = 0; groupNo < controller->numGroup; groupNo++)
	{
		incData.ctrl_grp |= (0x01 << groupNo);
		incData.grp_pos_info[groupNo].pos_tag.data[0] = Ros_CtrlGroup_GetAxisConfig(controller->ctrlGroups[groupNo]);
		//incData.grp_pos_info[i].pos_tag.data[3] = MP_INC_ANGLE_DTYPE; // Control with pulse increments
		incData.grp_pos_info[groupNo].pos_tag.data[3] = MP_INC_PULSE_DTYPE;
		// #TODO not set: tool info[2], user coordinate sys[4], 
	}

	// Set STATE_INITIALIZE
	Ros_RealTimeMotionServer_SetState(STATE_INITIALIZE, &rtState, &stateEntry);

	// ------------------ SERVER HANDLING ------------------

	// UDP server socket
	if ((udp_socket_fd = mpSocket(AF_INET, SOCK_DGRAM, 0)) < 0)
	{
		printf("[RT] UDP socket creation failed. ERRNO: %d\n", errno);
		goto exitTask;
	}

	// Set up UDP server structure
	sizeofClientAddr = sizeof(clientAddr);
	memset(&serverAddr, CLEAR, sizeof(serverAddr));
	memset(&clientAddr, CLEAR, sizeof(clientAddr));

	serverAddr.sin_family = AF_INET;
	serverAddr.sin_addr.s_addr = INADDR_ANY;
	serverAddr.sin_port = mpHtons(REALTIME_MOTION_UDP_PORT);

	// Bind server port
	if (mpBind(udp_socket_fd, (struct sockaddr *)&serverAddr, sizeof(serverAddr)) < 0)
	{
		printf("[RT] UDP bind failed. ERRNO: %d\n", errno);
		goto exitTask;
	}

	
	// ------------------ GET CLIENT INFORMATION ------------------
	
	// Set 10 sec timeout
	readTimeout.tv_sec = 10; readTimeout.tv_usec = 0;
	
	// Receive msg to set client ip
	if (!_Ros_RealTimeMotionServer_GetUdpMsg())
	{
		puts("[RT] failed to receive client informaiton");
		goto exitTask;
	}
	else // Successful
	{
		static char _ip_str[INET_ADDRSTRLEN];
		mpInetNtoaB(clientAddr.sin_addr, _ip_str);
		printf("[RT] setting client IP: %s\n", _ip_str);
	}

	
	// ------------------ START CONTROLLER ------------------

	// Ensure that the motion is not stopped on the controller
	controller->bStopMotion = FALSE;
	
	// Start Trajectory mode by starting the INIT_ROS job on the controller
	static BOOL bRet; // #TODO remove bool, move function to if()
	bRet = Ros_RealTimeMotionServer_StartTrajMode(controller);
	
	DEMO_MODE = !bRet;
	if(DEMO_MODE) { printf("[RT] ----------- DEMO_MODE activated -----------\n"); }
	bRet = TRUE; // TEST #TODO REMOVE

	// clear message
	memset(&rtMsgSend, CLEAR, RT_MSG_STATE_SIZE);
	rtMsgSend.header.msg_state = rtState;
	
	// Controller started successfully
	if (bRet) 
	{
		Db_Print("[RT] controller active.\n");
		rtMsgSend.header.msg_code = CODE_CONFIRM;
		_Ros_RealTimeMotionServer_SendUdpMsg();
	}
	// Exit on unsuccessful attempt
	else 
	{
		Db_Print("[RT] failed to start controller. Exit.\n");
		rtMsgSend.header.msg_code = Ros_Controller_GetNotReadySubcode(controller) - (ROS_RESULT_NOT_READY_UNSPECIFIED - CODE_UNSPECIFIED);
		_Ros_RealTimeMotionServer_SendUdpMsg();
		goto exitTask;
	}
	
	// Set interpolation cycle read timeout
	readTimeout.tv_sec = 0;
	readTimeout.tv_usec = REALTIME_MOTION_TIMEOUT_MS * 1000;

	// Send initial message
	_Ros_RealTimeMotionServer_SetStateMsg(controller, &rtState, &sequence);
	_Ros_RealTimeMotionServer_SendUdpMsg();

	// Set idle state
	Ros_RealTimeMotionServer_SetState(STATE_IDLE, &rtState, &stateEntry);

	Db_Print("[RT] Starting control loop with cycle time: %u ms\n", controller->interpolPeriod);

	// ------------------ MAIN RT LOOP  ------------------

	while ( // exit condition here
		//timeoutCounter < REALTIME_MOTION_TIMEOUT_COUNTER_MAX &&
		//runCounter < testRunSec && // #TEST1
		//Ros_Controller_IsMotionReady(controller) && // #TEST1
		rtState != STATE_FINAL &&	// Exit in final state
		!controller->bStopMotion
		)
	{

		// Sync with the interpolation clock
		if (mpClkAnnounce(MP_INTERPOLATION_CLK) < 0) 
		{ 
			puts("[RT] TASK TIMEOUT\n");
			goto exitTask;
		}

		// --------- receive cmd from pc ---------
		msgReceived = _Ros_RealTimeMotionServer_GetUdpMsg();
		
		mpStopWatchStart(stopWatch);	// stopWatch

		if(!msgReceived)
		{	
			// failed to receive message
			if(timeoutCounter % 125 == 0) { Db_Print("[RT] GetUdpMsg timeout %d\n", timeoutCounter); }

			timeoutCounter++;
		}
		else
		{
			timeoutCounter = 0;
		}



		// reset message code
		// msgCode = CODE_UNDEFINED;

		// --------- switch state machine ---------
		switch (rtState)
		{
		case STATE_IDLE:
			// Robot stagnation, waiting for msgs
			if(stateEntry)
			{
				// Set initial joint angles
				for(groupNo = 0; groupNo < controller->numGroup; groupNo++)
				{
					if(!Ros_CtrlGroup_GetPulsePosCmd(controller->ctrlGroups[groupNo], mData.prevPosCmd))
					{ Db_Print("[RT] Failed to get PulsePosCmd\n"); }
					if(!Ros_CtrlGroup_GetPulsePosCmd(controller->ctrlGroups[groupNo], mData.prevPosSet))
					{ Db_Print("[RT] Failed to get PulsePosCmd\n"); }
				}
				//Db_Print("[RT] state entry %d", rtState);
				stateEntry = false;
			}

			// Exit
			if(msgReceived)
			{
				Ros_RealTimeMotionServer_SetState(STATE_RUN, &rtState, &stateEntry);
			}
			break;

		case STATE_RUN:
			// Nominal robot operation
			if(stateEntry)
			{
				// Skip all old messages
				if(rtMsgRecv.header.msg_sequence != sequence)
				{
					static UINT8 _i;
					for(_i = 0u; _i < 255 * sizeof(rtMsgRecv.header.msg_sequence); _i++)
					{
						Db_Print("[RT] Fixing sequence. Recv: %d. Actual: %d\n", 
							rtMsgRecv.header.msg_sequence, sequence);
						_Ros_RealTimeMotionServer_GetUdpMsg();
						if(rtMsgRecv.header.msg_sequence == sequence) {break;}
						else if(_i == 254) 
						{
							printf("[RT] failed to get latest message. Sequence: %d MsgSequence: %d\n",
								sequence, rtMsgRecv.header.msg_sequence);
							goto exitTask;
						}
					}
				}
				stateEntry = false;
			}

			// Exit on missing msg
			if(!msgReceived)
			{
				motionRet = _Ros_RealTimeMotionServer_SetMotion(controller, RT_MOTION_STALL);
				Ros_RealTimeMotionServer_SetState(STATE_RUN_STALL, &rtState, &stateEntry);
			}
			else
			{
				motionRet = _Ros_RealTimeMotionServer_SetMotion(controller, RT_MOTION_SYNC);
			}

			break;

		case STATE_EXIT:
			// Exit state handling
			if(stateEntry)
			{
				stateEntry = false;
			}

			// exit rt loop
			Ros_RealTimeMotionServer_SetState(STATE_FINAL, &rtState, &stateEntry);
			break;

		case STATE_RUN_STALL:
			// Keep incrementing for some cycles and wait for new msgs
			if(stateEntry)
			{
				stateEntry = false;
			}

			motionRet = _Ros_RealTimeMotionServer_SetMotion(controller, RT_MOTION_STALL);

			if(msgReceived)
			{
				Ros_RealTimeMotionServer_SetState(STATE_RUN, &rtState, &stateEntry);
			}
			else if(timeoutCounter > 100) // Random value, test #TODO
			{
				Ros_RealTimeMotionServer_SetState(STATE_RUN_OUT, &rtState, &stateEntry);
			}
			break;
		
		case STATE_RUN_OUT:
			// Decelerate to 0
			if(stateEntry)
			{
				stateEntry = false;
			}

			motionRet = _Ros_RealTimeMotionServer_SetMotion(controller, RT_MOTION_STOP);

			if(msgReceived)
			{
				Ros_RealTimeMotionServer_SetState(STATE_RUN, &rtState, &stateEntry);
			}
			else if(motionRet == RT_MOTION_STOP)
			{

				Ros_RealTimeMotionServer_SetState(STATE_IDLE, &rtState, &stateEntry);
			}

			break;
		
		// case STATE_RUN_IN:
		// 	// Accelerate to synchronized motion
		// 	if(stateEntry)
		// 	{
		// 		// Is there a better way to handle this? E.g. small input buffer on host #TODO
		// 		// Skip all old msgs
		// 		static UINT8 _tryCounter;
		// 		_tryCounter = 0;
		// 		while (rtMsgRecv.header.msg_sequence != sequence)
		// 		{
		// 			Db_Print("[RT] Fixing sequence. Recv: %d. Actual: %d\n", 
		// 				rtMsgRecv.header.msg_sequence, sequence);
					
		// 			if(_tryCounter > 100) // Random value, test #TODO
		// 			{
		// 				printf("[RT] failed to get latest message. Sequence: %d MsgSequence: %d\n",
		// 					sequence, rtMsgRecv.header.msg_sequence);
		// 				goto exitTask;
		// 			}
		// 			_Ros_RealTimeMotionServer_GetUdpMsg();
		// 			_tryCounter++;
		// 		}

		// 		stateEntry = false;
		// 	}


		// 	// Exit on all axes in sync
		// 	if(_Ros_RealTimeMotionServer_SetMotion(controller, RT_MOTION_ACCELERATE) == RT_MOTION_SYNC)
		// 	{
		// 		Ros_RealTimeMotionServer_SetState(STATE_RUN, &rtState, &stateEntry);
		// 	}
		// 	// Exit on missing msg
		// 	else if(!msgReceived)
		// 	{
		// 		Ros_RealTimeMotionServer_SetState(STATE_RUN_STALL, &rtState, &stateEntry);
		// 	}

		default:
			// catching all unused states, should never happen with rtState
			printf("[RT] UNDEFINED STATE %d. EXIT.\n", rtState);
			goto exitTask;
			break;
		}

		
		// sw_results[sw_counter++] = sw_current_iter;
		// if(sw_current_iter > 0.2)
		// {
		// 	printf("[RT] SW peak: %f ms\n\r", sw_current_iter);
		// }
		
		// if(sw_counter == 250)
		// {
		// 	sw_final_result = 0;
		// 	for(sw_counter = 0; sw_counter<250; sw_counter++)
		// 	{
		// 		sw_final_result += sw_results[sw_counter];
		// 	}
		// 	sw_final_result /= 250.0;
		// 	printf("[RT] SW final result: %lf ms\n\r", sw_final_result);
		// 	sw_counter = 0;
		// 	memset(sw_results, 0, 250);
		// }

		mpStopWatchStop(stopWatch);		// stopWatch
		sw_current = mpStopWatchGetTime(stopWatch);

		// --------- send state to pc ---------

		// #TODO remove
		_update_log_msg_data();

		sequence++;
		_Ros_RealTimeMotionServer_SetStateMsg(controller, &rtState, &sequence);
		_Ros_RealTimeMotionServer_SendUdpMsg();





	}

exitTask:

	// TEST #TODO REMOVE
	Ros_RealTimeMotionServer_PrintParams(controller);

	// Stop servos
	Ros_RealTimeMotionServer_StopTrajMode(controller);
	// Close UDP socket
	mpClose(udp_socket_fd);
	// Delete this task
	controller->tidIncMoveThread = INVALID_TASK;
	puts("[RT] Deleting RealTimeMotion server inc task");
	mpDeleteSelf;

}




//-----------------------------------------------------------------------
// Set SimpleMessage state message
//-----------------------------------------------------------------------
int Ros_RealTimeMotionServer_MotoRealTimeMotionJointStateEx(
		Controller* controller, int messageId, MotoRealTimeMotionMode mode,
		SimpleMsg* sendMsg) 
{
	int bRet;
	long pulsePos[MAX_PULSE_AXES];
	long pulseSpeed[MAX_PULSE_AXES];
	int groupNo;

	// initialize memory
	memset(sendMsg, 0x00, sizeof(SimpleMsg));

	// set prefix: length of message excluding the prefix
	sendMsg->prefix.length = sizeof(SmHeader) + sizeof(SmBodyMotoRealTimeMotionJointStateEx);

	// set header information
	sendMsg->header.msgType = ROS_MSG_MOTO_REALTIME_MOTION_JOINT_STATE_EX;
	sendMsg->header.commType = ROS_COMM_TOPIC;
	sendMsg->header.replyType = ROS_REPLY_INVALID;

	// set number of valid groups
	sendMsg->body.realTimeMotionJointStateEx.numberOfValidGroups =
			controller->numGroup;

	// set unique message id
	sendMsg->body.realTimeMotionJointStateEx.messageId = messageId;

	// set control mode (idle, position, velocity)
	sendMsg->body.realTimeMotionJointStateEx.mode = mode;

	// Populate the state of all control groups
	for (groupNo = 0; groupNo < controller->numGroup; groupNo++) 
	{
		sendMsg->body.realTimeMotionJointStateEx.jointStateData[groupNo].groupNo =
				groupNo;

		// feedback position
		bRet = Ros_CtrlGroup_GetFBPulsePos(controller->ctrlGroups[groupNo], pulsePos);
		if (bRet != TRUE) {
			return 0;
		}
		Ros_CtrlGroup_ConvertToRosPos(
				controller->ctrlGroups[groupNo], pulsePos,
				sendMsg->body.realTimeMotionJointStateEx.jointStateData[groupNo].pos);

		// servo speed
		bRet = Ros_CtrlGroup_GetFBServoSpeed(controller->ctrlGroups[groupNo], pulseSpeed);
		if (bRet == TRUE) {
			Ros_CtrlGroup_ConvertToRosPos(
				controller->ctrlGroups[groupNo], pulseSpeed,
				sendMsg->body.realTimeMotionJointStateEx.jointStateData[groupNo].vel);
		}
	}

	return sendMsg->prefix.length + sizeof(SmPrefix);
}


//-----------------------------------------------------------------------
// Attempts to start playback of a job to put the controller in RosMotion mode
//-----------------------------------------------------------------------
BOOL Ros_RealTimeMotionServer_StartTrajMode(Controller* controller)
{
	int ret;
	MP_STD_RSP_DATA rData;
	MP_START_JOB_SEND_DATA sStartData;
	int checkCount;
	int grpNo;
	STATUS status;

	
	Db_Print("[RT] Attempting controller start ...\r\n");

	// Update status
	Ros_Controller_StatusUpdate(controller);

	// Reset PFL Activation Flag
	if (controller->bPFLduringRosMove)
		controller->bPFLduringRosMove = FALSE;

	// Reset Inc Move Error
	if (controller->bMpIncMoveError)
		controller->bMpIncMoveError = FALSE;

	// Check if already in the proper mode
	if (Ros_Controller_IsMotionReady(controller))
		return TRUE;

	// Check if currently in operation, we don't want to interrupt current operation
	if (Ros_Controller_IsOperating(controller))
		return FALSE;

	// Check for condition that need operator manual intervention	
	if (Ros_Controller_IsEStop(controller))
	{
		Db_Print("[RT] Controller ESTOP\r\n");
		return FALSE;
	}
	if (Ros_Controller_IsHold(controller))
	{
		Db_Print("[RT] Controller HOLD\r\n");
		return FALSE;
	}
	if (!Ros_Controller_IsRemote(controller))
	{
		Db_Print("[RT] Controller not in REMOTE mode\r\n");
		return FALSE;
	}

	// Check for condition that can be fixed remotely
	if (Ros_Controller_IsError(controller))
	{
		// Cancel error
		memset(&rData, 0x00, sizeof(rData));
		ret = mpCancelError(&rData);
		if (ret != 0) goto updateStatus;
	}

	// Check for condition that can be fixed remotely
	if (Ros_Controller_IsAlarm(controller))
	{
		// Reset alarm
		memset(&rData, 0x00, sizeof(rData));
		ret = mpResetAlarm(&rData);
		if (ret == 0)
		{
			// wait for the Alarm reset confirmation
			int checkCount;
			for (checkCount = 0; checkCount < MOTION_START_TIMEOUT; checkCount += MOTION_START_CHECK_PERIOD)
			{
				// Update status
				Ros_Controller_StatusUpdate(controller);

				if (Ros_Controller_IsAlarm(controller) == FALSE)
					continue;

				Ros_Sleep(MOTION_START_CHECK_PERIOD);
			}
			if (Ros_Controller_IsAlarm(controller)) goto updateStatus;
		}
		else goto updateStatus;
	}

	// Servo On
	if (Ros_Controller_IsServoOn(controller) == FALSE)
	{
		MP_SERVO_POWER_SEND_DATA sServoData;
		memset(&sServoData, 0x00, sizeof(sServoData));

		status = Ros_RealTimeMotionServer_DisableEcoMode(controller);
		if (status == NG)
		{
			goto updateStatus;
		}

		sServoData.sServoPower = 1;  // ON
		memset(&rData, 0x00, sizeof(rData));
		ret = mpSetServoPower(&sServoData, &rData);
		if ((ret == 0) && (rData.err_no == 0))
		{
			// wait for the Servo On confirmation
			int checkCount;
			for (checkCount = 0; checkCount < MOTION_START_TIMEOUT; checkCount += MOTION_START_CHECK_PERIOD)
			{
				// Update status
				Ros_Controller_StatusUpdate(controller);

				if (Ros_Controller_IsServoOn(controller) == TRUE)
					break;

				Ros_Sleep(MOTION_START_CHECK_PERIOD);
			}
			if (Ros_Controller_IsServoOn(controller) == FALSE)
				goto updateStatus;
		}
		else
		{
			Ros_RealTimeMotionServer_PrintError(rData.err_no, "Can't turn on servo because:");
			goto updateStatus;
		}
	}

	// make sure that there is no data in the queues
	if (Ros_MotionServer_HasDataInQueue(controller)) {

		Db_Print("StartTrajMode clearing leftover data in queue\r\n");

		Ros_MotionServer_ClearQ_All(controller);

		if (Ros_MotionServer_HasDataInQueue(controller))
			printf("WARNING: StartTrajMode has data in queue\r\n");
	}

	// have to initialize the prevPulsePos that will be used when interpolating the traj
	for (grpNo = 0; grpNo < MP_GRP_NUM; ++grpNo)
	{
		if (controller->ctrlGroups[grpNo] != NULL)
		{
			Ros_CtrlGroup_GetPulsePosCmd(controller->ctrlGroups[grpNo], controller->ctrlGroups[grpNo]->prevPulsePos);
		}
	}

	// Start Job
	memset(&rData, 0x00, sizeof(rData));
	memset(&sStartData, 0x00, sizeof(sStartData));
	sStartData.sTaskNo = 0;
	memcpy(sStartData.cJobName, MOTION_INIT_ROS_JOB, MAX_JOB_NAME_LEN);
	ret = mpStartJob(&sStartData, &rData);
	if ((ret != 0) || (rData.err_no != 0))
	{
		Ros_RealTimeMotionServer_PrintError(rData.err_no, "Can't start job because:");
		goto updateStatus;
	}

	// wait for the Motion Ready
	for (checkCount = 0; checkCount < MOTION_START_TIMEOUT; checkCount += MOTION_START_CHECK_PERIOD)
	{
		// Update status
		Ros_Controller_StatusUpdate(controller);

		if (Ros_Controller_IsMotionReady(controller))
			return(TRUE);

		Ros_Sleep(MOTION_START_CHECK_PERIOD);
	}

updateStatus:
	// Update status
	Ros_Controller_StatusUpdate(controller);

	return (Ros_Controller_IsMotionReady(controller));
}


//-----------------------------------------------------------------------
// Set I/O signal matching the WAIT instruction to allow the controller 
// to resume job execution
//-----------------------------------------------------------------------
BOOL Ros_RealTimeMotionServer_StopTrajMode(Controller* controller)
{
	// Don't change mode if queue is not empty
	if (Ros_MotionServer_HasDataInQueue(controller))
	{
		//printf("Failed: Ros_MotionServer_HasDataInQueue is true\r\n");
		return FALSE;
	}

	// Stop motion
	if (!Ros_RealTimeMotionServer_StopMotion(controller))
	{
		//printf("Failed: Ros_RealTimeMotionServer_StopMotion is false\r\n");
		return FALSE;
	}

	// Set I/O signal
	Ros_Controller_SetIOState(IO_FEEDBACK_MP_INCMOVE_DONE, TRUE);

	return TRUE;
}


//-----------------------------------------------------------------------
// Stop motion by stopping message processing and clearing the queue
//-----------------------------------------------------------------------
BOOL Ros_RealTimeMotionServer_StopMotion(Controller* controller)
{
	// NOTE: for the time being, stop motion will stop all motion for all control group 
	BOOL bRet = TRUE;
	BOOL bStopped;
	int checkCnt;
	int groupNo;

	// Stop any motion from being processed further
	controller->bStopMotion = TRUE;

	// Check that background processing of message has been stopped
	for (checkCnt = 0; checkCnt < MOTION_STOP_TIMEOUT; checkCnt++)
	{
		bStopped = TRUE;
		for (groupNo = 0; groupNo < controller->numGroup; groupNo++)
			bStopped &= !controller->ctrlGroups[groupNo]->hasDataToProcess;
		if (bStopped)
			break;
		else
			Ros_Sleep(1);
	}

	// Clear queues
	if (Ros_MotionServer_HasDataInQueue(controller)) 
	{

		Db_Print("[RT] stop motion - clearing leftover data in queue\r\n");
		bRet = Ros_MotionServer_ClearQ_All(controller);
		if (Ros_MotionServer_HasDataInQueue(controller)) printf("[RT] WARNING: StartTrajMode has data in queue\r\n");
	}

	// All motion should be stopped at this point, so turn of the flag
	controller->bStopMotion = FALSE;

	if (checkCnt >= MOTION_STOP_TIMEOUT)
		printf("[RT] WARNING: Message processing not stopped before clearing queue\r\n");

	return(bStopped && bRet);
}


//-----------------------------------------------------------------------
// Utility function: Disable eco mode on controller
//-----------------------------------------------------------------------
STATUS Ros_RealTimeMotionServer_DisableEcoMode(Controller* controller)
{
	MP_SERVO_POWER_SEND_DATA sServoData;
	MP_STD_RSP_DATA rData;
	int ret;

	if (Ros_Controller_IsEcoMode(controller) == TRUE)
	{
		//toggle servos to disable energy-savings mode
		sServoData.sServoPower = 0;  // OFF
		memset(&sServoData, 0x00, sizeof(sServoData));
		memset(&rData, 0x00, sizeof(rData));
		ret = mpSetServoPower(&sServoData, &rData);
		if ((ret == 0) && (rData.err_no == 0))
		{
			// wait for the Servo/Eco OFF confirmation
			int checkCount;
			for (checkCount = 0; checkCount < MOTION_START_TIMEOUT; checkCount += MOTION_START_CHECK_PERIOD)
			{
				// Update status
				Ros_Controller_StatusUpdate(controller);

				if (Ros_Controller_IsEcoMode(controller) == FALSE)
					break;

				Ros_Sleep(MOTION_START_CHECK_PERIOD);
			}
		}
		else
		{
			Ros_RealTimeMotionServer_PrintError(rData.err_no, "Can't disable energy-savings mode because:");
			return NG;
		}
	}

	if (Ros_Controller_IsEcoMode(controller) == FALSE) return OK;
	else return NG;
}


//-----------------------------------------------------------------------
// Utility function: Print Error
//-----------------------------------------------------------------------
void Ros_RealTimeMotionServer_PrintError(USHORT err_no, char* msgPrefix)
{
	char errMsg[ERROR_MSG_MAX_SIZE];
	memset(errMsg, 0x00, ERROR_MSG_MAX_SIZE);
	Ros_Controller_ErrNo_ToString(err_no, errMsg, ERROR_MSG_MAX_SIZE);
	printf("%s %s\r\n", msgPrefix, errMsg);
}


//-----------------------------------------------------------------------
// Utility function: Print Inc Pos
//-----------------------------------------------------------------------
void Ros_RealTimeMotionServer_PrintPrevPos(Controller* controller)
{
	UINT8 grpNo;
	for (grpNo = 0; grpNo < controller->numGroup; ++grpNo)
	{
		if (controller->ctrlGroups[grpNo] != NULL)
		{
			Ros_CtrlGroup_GetPulsePosCmd(controller->ctrlGroups[grpNo], controller->ctrlGroups[grpNo]->prevPulsePos);
			CtrlGroup* ctrlGroup = controller->ctrlGroups[grpNo];
			printf("[RT] ctrlGroup->prevPulsePos: %ld, %ld, %ld, %ld, %ld, %ld, %ld, %ld\r\n",
				ctrlGroup->prevPulsePos[0], ctrlGroup->prevPulsePos[1], ctrlGroup->prevPulsePos[2],
				ctrlGroup->prevPulsePos[3], ctrlGroup->prevPulsePos[4], ctrlGroup->prevPulsePos[5],
				ctrlGroup->prevPulsePos[6], ctrlGroup->prevPulsePos[7]);
		}
	}
}


//-----------------------------------------------------------------------
// Utility function: Print Parameters
//-----------------------------------------------------------------------
void Ros_RealTimeMotionServer_PrintParams(Controller* controller)
{
	UINT8 grpNo, i;
	for (grpNo = 0; grpNo < controller->numGroup; ++grpNo)
	{
		if (controller->ctrlGroups[grpNo] != NULL)
		{
			Ros_CtrlGroup_GetPulsePosCmd(controller->ctrlGroups[grpNo], controller->ctrlGroups[grpNo]->prevPulsePos);
			CtrlGroup* ctrlGroup = controller->ctrlGroups[grpNo];


			printf("[PRMTR] groupNo: %d groupId: %d numAxes: %d tool: %d\n", 
				ctrlGroup->groupNo, ctrlGroup->groupId, ctrlGroup->numAxes, ctrlGroup->tool);
			//printf("[PRMTR] groupId: %d\n", ctrlGroup->groupId);
			//printf("[PRMTR] numAxes: %d\n", ctrlGroup->numAxes);
			//for (i = 0; i < MAX_PULSE_AXES; i++) printf("[PRMTR] pulseToRad[%d]: %f\n", i, ctrlGroup->pulseToRad.PtoR[i]);
			//for (i = 0; i < MAX_PULSE_AXES; i++) printf("[PRMTR] pulseToMeter[%d]: %f\n", i, ctrlGroup->pulseToMeter.PtoM[i]);
			//for (i = 0; i < MAX_PULSE_AXES; i++)
			//{
			//	printf("[PRMTR] correctionData[%d]: bValid: %d ulSourceAxis: %d ulCorrectionAxis: %d fCorrectionRatio: %f\n", i, 
			//		ctrlGroup->correctionData.correction[i].bValid,
			//		ctrlGroup->correctionData.correction[i].ulSourceAxis,
			//		ctrlGroup->correctionData.correction[i].ulCorrectionAxis,
			//		ctrlGroup->correctionData.correction[i].fCorrectionRatio
			//	);
			//}
			for (i = 0; i < MAX_PULSE_AXES; i++) printf("[PRMTR] maxIncrement[%d]: %u\n", i, ctrlGroup->maxInc.maxIncrement[i]);
			//for (i = 0; i < MP_GRP_AXES_NUM; i++) printf("[PRMTR] maxSpeed[%d]: %f\n", i, ctrlGroup->maxSpeed[i]);
			//printf("[PRMTR] tool: %d\n", ctrlGroup->tool);
			//for (i = 0; i < MAX_PULSE_AXES; i++) printf("[PRMTR] axisType[%d]: %d\n", i, ctrlGroup->axisType.type[i]);
			
			printf("[PRMTR] percentage limit: %f\n", GP_getGovForIncMotion(grpNo));

			static DH_PARAMETERS dh;
			static UINT8 link;
			GP_getDhParameters(grpNo, &dh);
			for (link = 0; link < MAX_PULSE_AXES; link++)
			{
				printf("[PRMTR] link %d: ", link);
				printf("theta: %f ", dh.link[link].theta);
				printf("d: %f ", dh.link[link].d);
				printf("a: %f ", dh.link[link].a);
				printf("alpha: %f\n", dh.link[link].alpha);
			}

			
			
		}
	}
}

