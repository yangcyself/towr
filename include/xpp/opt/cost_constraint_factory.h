/**
 @file    constraint_factory.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jul 19, 2016
 @brief   Declares factory class to build constraints.
 */

#ifndef XPP_XPP_OPT_INCLUDE_XPP_OPT_COST_CONSTRAINT_FACTORY_H_
#define XPP_XPP_OPT_INCLUDE_XPP_OPT_COST_CONSTRAINT_FACTORY_H_

#include <xpp/utils/state.h>
#include "variable_set.h"
#include "motion_structure.h"
#include "com_motion.h"
#include "motion_parameters.h"
#include <memory>

namespace xpp {
namespace opt {

class AConstraint;
class ACost;

/** Builds all types of constraints/costs for the user.
  *
  * Implements the factory method, hiding object creation from the client.
  * The client specifies which object it wants, and this class is responsible
  * for the object creation. Factory method is like template method pattern
  * for object creation.
  */
class CostConstraintFactory {
public:
  using ConstraintPtr = std::shared_ptr<AConstraint>;
  using CostPtr       = std::shared_ptr<ACost>;
  using Vector2d      = Eigen::Vector2d;
  using State2d       = xpp::utils::StateLin2d;
  using MotionTypePtr = std::shared_ptr<MotionParameters>;
  using ComMotionPtr  = std::shared_ptr<ComMotion>;

  CostConstraintFactory ();
  virtual ~CostConstraintFactory ();

  void Init(const ComMotionPtr&, const MotionStructure&,
            const MotionTypePtr& params, const State2d& initial_state,
            const State2d& final_state);

  // optimization variables with initial values
  VariableSet SplineCoeffVariables();
  VariableSet ContactVariables(const Vector2d initial_pos);
  VariableSet ConvexityVariables();
  VariableSet CopVariables();

  CostPtr GetCost(CostName name);
  ConstraintPtr GetConstraint(ConstraintName name);

private:
  MotionStructure motion_structure;
  MotionTypePtr params;
  ComMotionPtr com_motion;
  State2d initial_state_;
  State2d final_state_;

  // constraints
  ConstraintPtr MakeInitialConstraint();
  ConstraintPtr MakeFinalConstraint();
  ConstraintPtr MakeJunctionConstraint();
  ConstraintPtr MakeConvexityConstraint();
  ConstraintPtr MakeSupportAreaConstraint();
  ConstraintPtr MakeDynamicConstraint();
  ConstraintPtr MakeRangeOfMotionBoxConstraint();
  ConstraintPtr MakeFinalStanceConstraint();
  ConstraintPtr MakeObstacleConstraint();
  ConstraintPtr MakePolygonCenterConstraint();

  // costs
  CostPtr ComMotionCost_();

  CostPtr ToCost(const ConstraintPtr& constraint);
};

} /* namespace opt */
} /* namespace xpp */

#endif /* XPP_XPP_OPT_INCLUDE_XPP_OPT_COST_CONSTRAINT_FACTORY_H_ */