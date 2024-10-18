#ifndef MOTOMAN_HARDWARE__SIMPLE_MESSAGE_HPP_
#define MOTOMAN_HARDWARE__SIMPLE_MESSAGE_HPP_

#include <cstdint>
#define TCP_PORT_MOTION						50240
#define TCP_PORT_STATE						50241
#define TCP_PORT_IO							50242
#define TCP_PORT_MOTION_COMMAND   			50243
#define MAX_PULSE_AXES			(8)	/* Maximum pulse axes */


namespace motoman_hardware::simple_message 
{

using INT8 = int8_t;
using UINT8 = uint8_t;
using INT16 = int16_t;
using UINT16 = uint16_t;
using INT32 = int32_t;
using UINT32 = uint32_t;
using ULONG = unsigned long;
using BOOL = bool;

#include "ParameterTypes.h"
#include "SimpleMessageDefinition.h"



} // namespace motoman_hardware::simple_message

// Define useful roboter constants

enum class ROBOT_SIGNAL 
{
    MS_OUT01 = 81320,
    MS_OUT02,
    MS_OUT03,
    MS_OUT04,
    MS_OUT05,
    // ...
    MS_OUT64 = 81397,
    FS_OUT01 = 81400,
    FS_OUT02,
    FS_OUT03,
    FS_OUT04,
    FS_OUT05,
    // ...
    FS_OUT64 = 81477,
};

#endif  // MOTOMAN_HARDWARE__SIMPLE_MESSAGE_HPP_
