/**
 @file    node_values.h
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Jul 19, 2017
 @brief   Brief description
 */

#ifndef XPP_OPT_INCLUDE_XPP_OPT_VARIABLES_NODE_VALUES_H_
#define XPP_OPT_INCLUDE_XPP_OPT_VARIABLES_NODE_VALUES_H_

#include <string>
#include <vector>

#include <xpp/cartesian_declarations.h>
#include <xpp/opt/constraints/composite.h>
#include <xpp/opt/polynomial.h>

#include "spline.h"

namespace xpp {
namespace opt {

/** Holds position and velocity of nodes used to generate a cubic Hermite spline.
 */
class NodeValues : public Component {
public:
  using Node     = CubicHermitePoly::Node;
  using Side     = CubicHermitePoly::Side;
  using VecNodes = std::vector<Node>;
  using VecTimes = std::vector<double>;

  using PolyType = CubicHermitePoly;
  using VecPoly  = std::vector<std::shared_ptr<PolyType>>;


  struct NodeInfo {
    int id_;
    MotionDerivative deriv_;
    int dim_;
  };


  NodeValues (bool is_motion, const Node& initial_value, const VecTimes&, const std::string& name);
  virtual ~NodeValues ();

  /**
   * ordered (x0,  y0,  z0,
   *          xd0, yd0, zd0,
   *          x1,  y1,  z1,
   *          xd1, yd1, zd1,...
   */
  VectorXd GetValues () const override;
  void SetValues (const VectorXd& x) override;

//  VecBound GetBounds () const override;


  Jacobian GetJacobian(int poly_id, double t_local) const;
  VecPoly GetCubicPolys() const { return cubic_polys_; };

  void UpdatePolynomials();
  VecTimes GetDurations() const { return timings_; };


private:
  std::vector<NodeInfo> GetNodeInfo(int idx) const;
  int GetNodeId(int poly_id, Side) const;

  std::vector<Node> nodes_;
  VecTimes timings_; // zmp_ for now constant
  VecPoly cubic_polys_;
  int n_dim_;

  int n_polys_per_multi_poly_phase;
};


// zmp_ might be able to remove this class
class HermiteSpline : public Spline {
public:
  using NodeValueT = std::shared_ptr<NodeValues>;


  HermiteSpline(const OptVarsPtr& opt_vars, const std::string& spline_base_id);
  virtual ~HermiteSpline();

  virtual bool DoVarAffectCurrentState(const std::string& poly_vars, double t_current) const override;
  Jacobian GetJacobian(double t_global,  MotionDerivative dxdt) const override;

private:
  virtual VecTimes GetDurations() const override { return node_values_->GetDurations(); };
  NodeValueT node_values_;
};




} /* namespace opt */
} /* namespace xpp */

#endif /* XPP_OPT_INCLUDE_XPP_OPT_VARIABLES_NODE_VALUES_H_ */
