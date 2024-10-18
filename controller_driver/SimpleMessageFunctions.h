// SimpleMessageFunctions.h
//
/*
* Software License Agreement (BSD License) 
*
* Copyright (c) 2013, Yaskawa America, Inc.
* All rights reserved.
*
* Redistribution and use in binary form, with or without modification,
* is permitted provided that the following conditions are met:
*
*       * Redistributions in binary form must reproduce the above copyright
*       notice, this list of conditions and the following disclaimer in the
*       documentation and/or other materials provided with the distribution.
*       * Neither the name of the Yaskawa America, Inc., nor the names 
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

#ifndef SIMPLE_MSG_FUNCTIONS_H
#define SIMPLE_MSG_FUNCTIONS_H

#include "SimpleMessageDefinition.h"

//-------------------
// Function Section
//-------------------

extern int Ros_SimpleMsg_JointFeedback(CtrlGroup* ctrlGroup, SimpleMsg* sendMsg);
extern void Ros_SimpleMsg_JointFeedbackEx_Init(int numberOfGroups, SimpleMsg* sendMsg);
extern int Ros_SimpleMsg_JointFeedbackEx_Build(int groupIndex, SimpleMsg* src_msgFeedback, SimpleMsg* dst_msgExtendedFeedback);

extern int Ros_SimpleMsg_MotionReply(SimpleMsg* receiveMsg, int result, int subcode, SimpleMsg* replyMsg, int ctrlGrp);
extern int Ros_SimpleMsg_IoReply(int result, int subcode, SimpleMsg* replyMsg);

//Uncomment the DEBUG definition to enable debug-messages at runtime
#define DEBUG  1

#ifdef DEBUG
// #warning Dont forget to disable the DEBUG flag
// function to dump data structure for debugging
extern void Ros_SimpleMsg_DumpTrajPtFull(SmBodyJointTrajPtFull* data);
#endif

#endif // SIMPLE_MSG_FUNCTIONS_H
