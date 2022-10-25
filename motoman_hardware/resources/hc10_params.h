
#ifndef HC10_PARAMS_H
#define HC10_PARAMS_H

#include <cstdint>

//constexpr uint8_t groupNo = 0; // sequence group number
//constexpr uint8_t groupId = 0; // control group ID - MP_R1_GID
constexpr uint8_t numAxes = 6; // number of axis in the control group

// conversion ratio between pulse and radian
constexpr float pulseToRad[8] =
{
    153054.453125,
    153054.453125,
    134496.984375,
    92770.187500,
    92770.187500,
    92770.187500,
    0.000000,
    0.000000
};

// maximum increment per interpolation cycle
constexpr uint32_t maxIncrement[8] = 
{
    1389,
    1389,
    1690,
    1165,
    1619,
    1619,
    0,
    0
};

// maximum joint speed in radian/sec (rotational) or meter/sec (linear)
constexpr float maxSpeed[8] =
{
    2.268800,
    2.268800,
    3.141334,
    3.139478,
    4.362932,
    4.362932,
    0.000000,
    0.000000
};

// Indicates whether axis is rotary or linear. AXIS_ROTATION=0, AXIS_LINEAR=1, AXIS_INVALID=2
constexpr uint8_t axisType[8] =
{
    0,
    0,
    0,
    0,
    0,
    0,
    2,
    2
};

#endif // HC10_PARAMS_H


// groupNo: 0
// groupId: 0
// numAxes: 6
// pulseToRad[0]: 153054.453125
// pulseToRad[1]: 153054.453125
// pulseToRad[2]: 134496.984375
// pulseToRad[3]: 92770.187500
// pulseToRad[4]: 92770.187500
// pulseToRad[5]: 92770.187500
// pulseToRad[6]: 0.000000
// pulseToRad[7]: 0.000000
// pulseToMeter[0]: 0.000000
// pulseToMeter[1]: 0.000000
// pulseToMeter[2]: 0.000000
// pulseToMeter[3]: 0.000000
// pulseToMeter[4]: 0.000000
// pulseToMeter[5]: 0.000000
// pulseToMeter[6]: 0.000000
// pulseToMeter[7]: 0.000000
// correctionData[0]: bValid: 0 ulSourceAxis: -1 ulCorrectionAxis: -1 fCorrectionRatio: Inf
// correctionData[1]: bValid: 0 ulSourceAxis: -1 ulCorrectionAxis: -1 fCorrectionRatio: Inf
// correctionData[2]: bValid: 0 ulSourceAxis: -1 ulCorrectionAxis: -1 fCorrectionRatio: Inf
// correctionData[3]: bValid: 0 ulSourceAxis: -1 ulCorrectionAxis: -1 fCorrectionRatio: Inf
// correctionData[4]: bValid: 0 ulSourceAxis: -1 ulCorrectionAxis: -1 fCorrectionRatio: Inf
// correctionData[5]: bValid: 0 ulSourceAxis: -1 ulCorrectionAxis: -1 fCorrectionRatio: Inf
// correctionData[6]: bValid: 0 ulSourceAxis: -1 ulCorrectionAxis: -1 fCorrectionRatio: Inf
// correctionData[7]: bValid: 0 ulSourceAxis: -1 ulCorrectionAxis: -1 fCorrectionRatio: Inf
// maxIncrement[0]: 1389
// maxIncrement[1]: 1389
// maxIncrement[2]: 1690
// maxIncrement[3]: 1165
// maxIncrement[4]: 1619
// maxIncrement[5]: 1619
// maxIncrement[6]: 0
// maxIncrement[7]: 0
// maxSpeed[0]: 2.268800
// maxSpeed[1]: 2.268800
// maxSpeed[2]: 3.141334
// maxSpeed[3]: 3.139478
// maxSpeed[4]: 4.362932
// maxSpeed[5]: 4.362932
// maxSpeed[6]: 0.000000
// maxSpeed[7]: 0.000000
// tool: 0
// axisType[0]: 0
// axisType[1]: 0
// axisType[2]: 0
// axisType[3]: 0
// axisType[4]: 0
// axisType[5]: 0
// axisType[6]: 2
// axisType[7]: 2