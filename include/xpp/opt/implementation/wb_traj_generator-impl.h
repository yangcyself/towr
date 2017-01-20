/**
@file    hyq_spliner.cpp
@author  Alexander Winkler (winklera@ethz.ch)
@date    Oct 21, 2014
@brief   Splines body position, orientation and swing leg
 */

#include <xpp/opt/wb_traj_generator.h>
#include <kindr/rotations/Rotation.hpp>

namespace xpp {
namespace opt {

using namespace xpp::utils;


WBTrajGenerator::WBTrajGenerator()
{
  leg_lift_height_ = 0.0;
}

WBTrajGenerator::~WBTrajGenerator()
{
}

void
WBTrajGenerator::Init (const PhaseVec& phase_info, const ComMotionS& com_spline,
                       const VecFoothold& footholds, double des_height,
                       const SplineNode& curr_state, double lift_height)
{
  // get endeffector size from current node
  kNEE = curr_state.GetEECount();
  leg_lift_height_ = lift_height;
  com_motion_ = com_spline;

  t_start_ = curr_state.GetTime();
  nodes_.push_back(curr_state);
  nodes_.back().SetTime(0.0); // internally, the motion starts at t=0

  BuildNodeSequence(phase_info, footholds, des_height);

  CreateAllSplines();
}

void
WBTrajGenerator::BuildNodeSequence(const PhaseVec& phase_info,
                                   const VecFoothold& footholds,
                                   double des_robot_height)
{
  int phase_id = 0;
  for (const auto& curr_phase : phase_info) {
    // starting point is previous state
    SplineNode prev_node = nodes_.back();
    SplineNode goal_node(prev_node.GetEECount());

    EEXppPos pos_W = prev_node.GetEEPos();
    for (auto c : curr_phase.swing_goal_contacts_) {
      pos_W.At(c.ee).x() = footholds.at(c.id).x();
      pos_W.At(c.ee).y() = footholds.at(c.id).y();
      pos_W.At(c.ee).z() = 0.0;
    }

    SplineNode::ContactState contact_state(prev_node.GetEECount());
    contact_state.SetAll(false);
    for (auto c : curr_phase.GetAllContacts())
      contact_state.At(c.ee) = true;

    goal_node.SetEEState(utils::kPos, pos_W);
    goal_node.SetContactState(contact_state);
    // vel, acc of endeffector always zero at nodes

    // adjust roll, pitch, yaw depending on footholds
    BaseState base = prev_node.GetBase();
    kindr::EulerAnglesXyzPD yprIB(0.0, 0.0, 0.0);
    kindr::RotationQuaternionPD qIB(yprIB);
    base.ang.q = qIB.toImplementation();

    // adjust global z position of body depending on footholds
    base.lin.p.z() = des_robot_height + goal_node.GetZAvg();
    goal_node.SetBase(base);

    goal_node.SetTime(prev_node.GetTime() + curr_phase.duration_); // time to reach this node
    goal_node.SetPercentPhase(0.0);
    goal_node.SetCurrentPhase(phase_id++);

    nodes_.push_back(goal_node);
  }
}

void WBTrajGenerator::CreateAllSplines()
{
  z_spliner_.clear();
  ori_spliner_.clear();
  ee_spliner_.clear();

  SplinerOri ori;
  ZPolynomial z_height;
  EESplinerArray feet(kNEE);

  for (int n=1; n<nodes_.size(); ++n) {
    SplineNode from = nodes_.at(n-1);
    SplineNode to   = nodes_.at(n);

    BuildPhase(from, to, z_height, ori, feet);

    z_spliner_.push_back(z_height);
    ori_spliner_.push_back(ori);
    ee_spliner_.push_back(feet);
  }
}

void
WBTrajGenerator::BuildPhase(const SplineNode& from, const SplineNode& to,
                                 ZPolynomial& z_poly,
                                 SplinerOri& ori,
                                 EESplinerArray& feet) const
{
  double t_phase = to.GetTime() - from.GetTime();
  z_poly.SetBoundary(t_phase, from.GetBase().lin.Get1d(Z), to.GetBase().lin.Get1d(Z));

  xpp::utils::StateLin3d rpy_from, rpy_to;
  rpy_from.p = TransformQuatToRpy(from.GetBase().ang.q);
  rpy_to.p   = TransformQuatToRpy(to.GetBase().ang.q);
  ori.SetBoundary(t_phase, rpy_from, rpy_to);


  for (EEID ee : from.GetEndeffectors()) {
    feet.At(ee).SetDuration(t_phase);
    feet.At(ee).SetXYParams(from.GetEEState().At(ee).Get2D(), to.GetEEState().At(ee).Get2D());
    feet.At(ee).SetZParams(from.GetPercentPhase(), leg_lift_height_);
  }
}

Eigen::Vector3d
WBTrajGenerator::TransformQuatToRpy(const Eigen::Quaterniond& q)
{
  // wrap orientation
  kindr::RotationQuaternionPD qIB(q);

  kindr::EulerAnglesXyzPD rpyIB(qIB);
  rpyIB.setUnique(); // wrap euler angles yaw from -pi..pi

  // if yaw jumped over range from -pi..pi
  static double yaw_prev = 0.0;
  static int counter360 = 0;
  if (rpyIB.yaw()-yaw_prev < -M_PI_2) {
    std::cout << "passed yaw=0.9pi->-0.9pi, increasing counter...\n";
    counter360 += 1;
  }
  if (rpyIB.yaw()-yaw_prev > M_PI_2) {
    std::cout << "passed yaw=-0.9pi->0.9pi, decreasing counter...\n";
    counter360 -= 1;
  }
  yaw_prev = rpyIB.yaw();

  // contains information that orientation went 360deg around
  kindr::EulerAnglesXyzPD yprIB_full = rpyIB;
  yprIB_full.setYaw(rpyIB.yaw() + counter360*2*M_PI);

  return yprIB_full.toImplementation();
}

void
WBTrajGenerator::FillZState(double t_global, State3d& pos) const
{
  double t_local = GetLocalPhaseTime(t_global);
  int  spline    = GetPhaseID(t_global);

  utils::StateLin1d z_splined;
  z_spliner_.at(spline).GetPoint(t_local, z_splined);
  pos.SetDimension(z_splined, Z);
}

WBTrajGenerator::BaseState
WBTrajGenerator::GetCurrentBase (double t_global) const
{
  BaseState base;
  base.lin = GetCurrPosition(t_global);
  base.ang = GetCurrOrientation(t_global);
  return base;
}

WBTrajGenerator::State3d
WBTrajGenerator::GetCurrPosition(double t_global) const
{
  State3d pos;

  xpp::utils::StateLin2d xy_optimized = com_motion_->GetCom(t_global);
  pos.p.topRows(kDim2d) = xy_optimized.p;
  pos.v.topRows(kDim2d) = xy_optimized.v;
  pos.a.topRows(kDim2d) = xy_optimized.a;

  FillZState(t_global, pos);
  return pos;
}

WBTrajGenerator::StateAng3d
WBTrajGenerator::GetCurrOrientation(double t_global) const
{
  double t_local = GetLocalPhaseTime(t_global);
  int  spline    = GetPhaseID(t_global);

  State3d ori_rpy;
  ori_spliner_.at(spline).GetPoint(t_local, ori_rpy);

  xpp::utils::StateAng3d ori;
  kindr::EulerAnglesXyzPD yprIB(ori_rpy.p);
  kindr::RotationQuaternionPD qIB(yprIB);
  ori.q = qIB.toImplementation();
  ori.v = ori_rpy.v;
  ori.a = ori_rpy.a;

  return ori;
}

WBTrajGenerator::FeetArray
WBTrajGenerator::GetCurrEndeffectors (double t_global) const
{
  double t_local = GetLocalPhaseTime(t_global);
  int  spline    = GetPhaseID(t_global);
  int  goal_node = spline+1;

  FeetArray feet = nodes_.at(goal_node).GetEEState();

  auto ee_in_contact = GetCurrContactState(t_global);
  for (EEID ee : feet.GetEEsOrdered())
    if (!ee_in_contact.At(ee)) // only lift swinglegs
      feet.At(ee) = ee_spliner_.at(spline).At(ee).GetState(t_local);

  return feet;
}

WBTrajGenerator::ContactArray
WBTrajGenerator::GetCurrContactState (double t_global) const
{
  int  spline    = GetPhaseID(t_global);
  int  goal_node = spline+1;

  return nodes_.at(goal_node).GetContactState(); // this must be pointing to first node!
}

double WBTrajGenerator::GetTotalTime() const
{
  return nodes_.back().GetTime();
}

double WBTrajGenerator::GetLocalPhaseTime(double t_global) const
{
  for (int i=0; i<nodes_.size(); ++i)
    if (nodes_.at(i).GetTime() > t_global)
      return t_global - nodes_.at(i-1).GetTime();
}

int WBTrajGenerator::GetPhaseID(double t_global) const
{
  assert(t_global <= GetTotalTime()); // time inside the time frame

  for (int i=0; i<nodes_.size(); ++i)
    if (nodes_.at(i).GetTime() > t_global)
      return i-1;
}

WBTrajGenerator::ArtiRobVec
WBTrajGenerator::BuildWholeBodyTrajectory (double dt) const
{
  ArtiRobVec trajectory;

  double t=0.0;
  double T = GetTotalTime();
  while (t<T) {

    SplineNode state(kNEE);
    state.SetBase(GetCurrentBase(t));
    state.SetEEState(GetCurrEndeffectors(t));
    state.SetContactState(GetCurrContactState(t));
    state.SetPercentPhase(GetPercentOfPhase(t));
    state.SetTime(t_start_ + t); // keep track of global time
    state.SetCurrentPhase(GetPhaseID(t));
    trajectory.push_back(state);

    t += dt;
  }

//  // add final node
//  SplineNode state(kNEE);
//  state.SetBase(GetCurrentBase(T));
//  state.SetEEState(GetCurrEndeffectors(T));
//  state.SetContactState(GetCurrContactState(T));
//  state.SetPercentPhase(GetPercentOfPhase(T));
//  state.SetTime(t_start_ + T); // keep track of global time
//  state.SetCurrentPhase(GetPhaseID(T));
//  trajectory.push_back(state);


  return trajectory;
}

double
WBTrajGenerator::GetPercentOfPhase (double t_global) const
{
  double t_local = GetLocalPhaseTime(t_global);
  int phase      = GetPhaseID(t_global);
  int goal_node  = phase+1;
  double t_phase = nodes_.at(goal_node).GetTime() - nodes_.at(goal_node-1).GetTime();

  return t_local/t_phase;
}

} // namespace opt
} // namespace xpp