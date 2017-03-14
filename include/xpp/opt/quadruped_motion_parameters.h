/**
 @file    quadruped_motion_parameterss.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jan 12, 2017
 @brief   Brief description
 */

#ifndef XPP_QUADRUPED_MOTION_PARAMETERS_H_
#define XPP_QUADRUPED_MOTION_PARAMETERS_H_

#include <xpp/opt/motion_parameters.h>

namespace xpp {
namespace opt {

class QuadrupedMotionParameters : public MotionParameters {
public:
  QuadrupedMotionParameters();
  static MotionTypePtr MakeMotion(opt::MotionTypeID);
};

class Walk : public QuadrupedMotionParameters {
public:
  Walk();
};

class Trott : public QuadrupedMotionParameters {
public:
  Trott();
};

class Pace : public QuadrupedMotionParameters {
public:
  Pace();
};

class Bound : public QuadrupedMotionParameters {
public:
  Bound();
};

class PushRecovery : public QuadrupedMotionParameters {
public:
  PushRecovery();
};

} // namespace opt
} // namespace xpp

#endif /* XPP_QUADRUPED_MOTION_PARAMETERS_H_ */