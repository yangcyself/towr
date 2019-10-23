
#ifndef TOWR_CONSTRAINTS_CENTER_OF_BODY_CONSTRAINT_H_
#define TOWR_CONSTRAINTS_CENTER_OF_BODY_CONSTRAINT_H_

#include <ifopt/constraint_set.h>
#include <towr/variables/spline.h>
#include <towr/variables/spline_holder.h>
#include <towr/variables/euler_converter.h>
#include <towr/variables/nodes_variables_phase_based.h>
#include <towr/terrain/height_map.h>

namespace towr {

/**
 * @brief Ensures the endeffectors always lays on or above terrain height.
 *
 * When using interior point solvers such as IPOPT to solve the problem, this
 * constraint also keeps the foot nodes far from the terrain, causing a leg
 * lifting during swing-phase. This is convenient.
 *
 * Attention: This is enforced only at the spline nodes.
 *
 * @ingroup Constraints
 */
class CenterOfBodyConstraint : public ifopt::ConstraintSet {
public:
  using Vector3d = Eigen::Vector3d;

  /**
   * @brief Constructs a terrain constraint.
   * @param terrain  The terrain height value and slope for each position x,y.
   * @param ee_motion_id The name of the endeffector variable set.
   */
  CenterOfBodyConstraint (const HeightMap::Ptr& terrain, std::string ee_motion_id, const SplineHolder& spline_holder);
  virtual ~CenterOfBodyConstraint () = default;

  void InitVariableDependedQuantities(const VariablesPtr& x) override;
  int p_is_in_tri(double px,double py, double ax,double ay, double bx, double by, double cx, double cy, double disc_rate)const;
  int check_fall(int cx,int cy,int feet_num, int* ground_feets) const;
  void add_to_arrange_res(int *used, int *arrange_list,int arrange_res[][20],int &arrange_len, int n)const;
  void dfs(int p, int n, int *used, int *arrange_list, int arrange_res[][20], int &arrange_len)const;

  VectorXd GetX() const;
  VectorXd GetY() const;

  VectorXd GetValues() const override;
  VecBound GetBounds() const override;
  void FillJacobianBlock (std::string var_set, Jacobian&) const override;

private:
  NodesVariablesPhaseBased::Ptr ee_motion_; ///< the position of the endeffector.
  HeightMap::Ptr terrain_;    ///< the height map of the current terrain.
  NodeSpline::Ptr base_linear_;     ///< the linear position of the base.

  std::string ee_motion_id_;  ///< the name of the endeffector variable set.
  std::vector<int> node_ids_; ///< the indices of the nodes constrained.
};

} /* namespace towr */

#endif /* TOWR_CONSTRAINTS_CENTER_OF_BODY_CONSTRAINT_H_ */