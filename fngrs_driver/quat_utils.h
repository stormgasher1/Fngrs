#define _USE_MATH_DEFINES

#include "openvr_driver.h"
#include <iostream>
#include <cmath>

double DegToRad(int degrees);
double RadToDeg(double rad);

vr::HmdMatrix34_t& createRotationMatrix(vr::HmdMatrix34_t& matrix, unsigned axisId, float angle);
vr::HmdMatrix34_t& MultiplyMatrix(vr::HmdMatrix34_t& result, const vr::HmdMatrix34_t& a, const vr::HmdMatrix34_t& b);
vr::HmdQuaternion_t GetRotation(vr::HmdMatrix34_t matrix);
vr::HmdQuaternion_t MultiplyQuaternion(vr::HmdQuaternion_t q, vr::HmdQuaternion_t r);

double GetYaw(vr::HmdQuaternion_t q);
double GetPitch(vr::HmdQuaternion_t q);
double GetRoll(vr::HmdQuaternion_t q);

double GetYOffsetFromRoll(vr::HmdQuaternion_t q, double offset, bool right_hand);
double GetXOffsetFromRoll(vr::HmdQuaternion_t q, double offset, bool right_hand);