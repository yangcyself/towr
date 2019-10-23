#include <towr/constraints/center_of_body_constraint.h>
#include <towr/variables/variable_names.h>

namespace towr {

int CenterOfBodyConstraint::p_is_in_tri(double px,double py, double ax,double ay, double bx, double by, double cx, double cy, double disc_rate)const{
  double x1,x2,x3,y1,y2,y3;
  int sign_of_tri,sign_of_ab,sign_of_ac,sign_of_bc,d1,d2,d3;
  x1 = ax-px;
  x2 = bx-px;
  x3 = cx-px;
  y1 = ay-py;
  y2 = by-py;
  y3 = cy-py;

  x1 *= disc_rate;
  x2 *= disc_rate;
  x3 *= disc_rate;
  y1 *= disc_rate;
  y2 *= disc_rate;
  y3 *= disc_rate;

  sign_of_tri = (x2-x1) * (y3-y1) - (y2-y1)*(x3-x1);
  sign_of_ab = (x2-x1)*(-y1) - (y2-y1)*(-x1);
  sign_of_ac = (x1-x3)*(-y3) - (y1-y3)*(-x3);
  sign_of_bc = (x3-x2)*(-y3) - (y3-y2)*(-x3);

  d1 = (sign_of_ab*sign_of_tri>0);
  d2 = (sign_of_ac*sign_of_tri>0);
  d3 = (sign_of_bc*sign_of_tri>0);

  return (d1 && d2 && d3);

}

void CenterOfBodyConstraint::add_to_arrange_res(int *used, int *arrange_list, int arrange_res[][20],int &arrange_len, int n)const
  {
    int i,j,arrange_tmp;
    arrange_tmp = 0;
    for(i=1;i<=n;i++)
    {
        for(j=0;j<n;j++)
        {
            if(used[j]==i)arrange_res[arrange_len][arrange_tmp++] = arrange_list[j];
        }
    }
    ++arrange_len;
  }

void CenterOfBodyConstraint::dfs(int p, int n, int *used, int *arrange_list, int arrange_res[][20], int &arrange_len)const{
  if(p>n)
      {
          CenterOfBodyConstraint::add_to_arrange_res(used,arrange_list,arrange_res,arrange_len,n);
          return ;
      }
      int i = 0;
      for(i=0;i<n;i++)
      {
          if(!used[i])
          {
              used[i]=p;
              dfs(p+1,n,used,arrange_list,arrange_res,arrange_len);
              used[i]=0;
          }
      }
}

int CenterOfBodyConstraint::check_fall(int rx,int ry,int feet_num, int* ground_feets) const{
  Eigen::VectorXd gx = CenterOfBodyConstraint::GetX();
  Eigen::VectorXd gy = CenterOfBodyConstraint::GetY();

  

  //try to implement the program to judge whether the robot is in steady condition, then use this to set zero bound or unreachable bound.

  int arrange_list[20];
  for(int i=0;i<feet_num;++i){
    arrange_list[i] = ground_feets[i];
  }

  int used[20];
  int arrange_res[20][20];
  int arrange_len = 0;
  int arrange_tmp = 0;
  

  
  CenterOfBodyConstraint::dfs(feet_num - 2,3,used,arrange_list,arrange_res,arrange_len);

  int no_fall = 0;
  for(int i=0;i<arrange_len;++i){
    double ax,ay,bx,by,cx,cy;
    ax = gx[arrange_res[i][0]];
    ay = gy[arrange_res[i][0]];
    bx = gx[arrange_res[i][1]];
    by = gy[arrange_res[i][1]];
    cx = gx[arrange_res[i][2]];
    cy = gy[arrange_res[i][2]];
    no_fall = no_fall || CenterOfBodyConstraint::p_is_in_tri(rx,ry,ax,ay,bx,by,cx,cy,0.8);
  }
  return (no_fall != 0);
}

CenterOfBodyConstraint::CenterOfBodyConstraint (const HeightMap::Ptr& terrain,
                                      std::string ee_motion, const SplineHolder& spline_holder)
    :ConstraintSet(kSpecifyLater, "terrain-" + ee_motion)
{
  ee_motion_id_ = ee_motion;
  terrain_ = terrain;
  base_linear_  = spline_holder.base_linear_;
}

void
CenterOfBodyConstraint::InitVariableDependedQuantities (const VariablesPtr& x)
{
  ee_motion_ = x->GetComponent<NodesVariablesPhaseBased>(ee_motion_id_);

  // skip first node, b/c already constrained by initial stance
  for (int id=1; id<ee_motion_->GetNodes().size(); ++id)
    node_ids_.push_back(id);

  int constraint_count = node_ids_.size();
  SetRows(constraint_count);
}

Eigen::VectorXd
CenterOfBodyConstraint::GetX () const
{
  VectorXd g(GetRows());

  auto nodes = ee_motion_->GetNodes();
  int row = 0;
  for (int id : node_ids_) {
    Vector3d p = nodes.at(id).p();
    g(row++) = p.x();
  }

  return g;
}

Eigen::VectorXd
CenterOfBodyConstraint::GetY () const
{
  VectorXd g(GetRows());

  auto nodes = ee_motion_->GetNodes();
  int row = 0;
  for (int id : node_ids_) {
    Vector3d p = nodes.at(id).p();
    g(row++) = p.y();
  }

  return g;
}



Eigen::VectorXd
CenterOfBodyConstraint::GetValues () const
{
  VectorXd g(GetRows());

  auto nodes = ee_motion_->GetNodes();
  int row = 0;
  for (int id : node_ids_) {
    Vector3d p = nodes.at(id).p();
    g(row++) = p.z() - terrain_->GetHeight(p.x(), p.y());
  }

  return g;
}

CenterOfBodyConstraint::VecBound
CenterOfBodyConstraint::GetBounds () const
{
  VecBound bounds(GetRows());
  double insure_rate = 0.8; // [m]

  Eigen::Vector3d base_W = base_linear_->GetPoint(0).p();

  double cx = base_W[0];
  double cy = base_W[1];
  
  VectorXd g(GetRows());

  g = CenterOfBodyConstraint::GetValues();
  int on_the_ground_feets[10]{0};
  int ground_foot_idx = 0;
  for(int id : node_ids_){
    if(abs(g(id))<0.02)on_the_ground_feets[ground_foot_idx++] = id;
  }

  if(CenterOfBodyConstraint::check_fall(cx,cy,ground_foot_idx,on_the_ground_feets)<=0){
    int row = 0;
    for (int id : node_ids_) {
      bounds.at(row) = ifopt::BoundZero;
      row++;
    }
  }
  else{
    int row = 0;
    for (int id : node_ids_) {
      bounds.at(row) = ifopt::NoBound;
      row++;
    }
  }

  return bounds;
}

void
CenterOfBodyConstraint::FillJacobianBlock (std::string var_set, Jacobian& jac) const
{
  if (var_set == ee_motion_->GetName()) {
    auto nodes = ee_motion_->GetNodes();
    int row = 0;
    for (int id : node_ids_) {
      int idx = ee_motion_->GetOptIndex(NodesVariables::NodeValueInfo(id, kPos, Z));
      jac.coeffRef(row, idx) = 1.0;

      Vector3d p = nodes.at(id).p();
      for (auto dim : {X,Y}) {
        int idx = ee_motion_->GetOptIndex(NodesVariables::NodeValueInfo(id, kPos, dim));
        jac.coeffRef(row, idx) = -terrain_->GetDerivativeOfHeightWrt(To2D(dim), p.x(), p.y());
      }
      row++;
    }
  }
}

} /* namespace xpp */

