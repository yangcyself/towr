/*
 * zero_moment_point.cpp
 *
 *  Created on: Apr 30, 2016
 *      Author: winklera
 */

#include <xpp/zmp/zero_moment_point.h>

namespace xpp {
namespace zmp {


ZeroMomentPoint::Vector2d
ZeroMomentPoint::CalcZmp(const State3d& cog, double height)
{
  double z_acc = cog.a.z();

  Vector2d zmp;
  zmp.x() = cog.p.x() - height/(gravity_+z_acc) * cog.a.x();
  zmp.y() = cog.p.y() - height/(gravity_+z_acc) * cog.a.y();

  return zmp;
}


ZeroMomentPoint::MatVec
ZeroMomentPoint::ExpressZmpThroughCoefficients(const ContinuousSplineContainer& spline_structure,
                                               double height, int dim)
{
  typedef xpp::utils::VecScalar VecScalar;

  int coeff = spline_structure.GetTotalFreeCoeff();
  int num_nodes = spline_structure.GetTotalNodes4ls() + spline_structure.GetTotalNodesNo4ls();

  MatVec zmp(num_nodes, coeff);

  const double z_acc = 0.0; // TODO: calculate z_acc based on foothold height
  double zmp_constant = height/(gravity_+z_acc);

  int n = 0; // node counter
  for (const ZmpSpline& s : spline_structure.GetSplines()) {

    // calculate e and f coefficients from previous values
    const int id = s.GetId();
    VecScalar Ek = spline_structure.GetCoefficient(id, dim, E);
    VecScalar Fk = spline_structure.GetCoefficient(id, dim, F);
    int a = ContinuousSplineContainer::Index(id,dim,A);
    int b = ContinuousSplineContainer::Index(id,dim,B);
    int c = ContinuousSplineContainer::Index(id,dim,C);
    int d = ContinuousSplineContainer::Index(id,dim,D);

    for (double i=0; i < s.GetNodeCount(spline_structure.dt_); ++i) {

      double time = i*spline_structure.dt_;
      std::array<double,6> t = utils::cache_exponents<6>(time);

      //  x_zmp = x_pos - height/(g+z_acc) * x_acc
      //      with  x_pos = at^5 +   bt^4 +  ct^3 + dt*2 + et + f
      //            x_acc = 20at^3 + 12bt^2 + 6ct   + 2d
      zmp.M(n, a)   = t[5]        - zmp_constant * 20.0 * t[3];
      zmp.M(n, b)   = t[4]        - zmp_constant * 12.0 * t[2];
      zmp.M(n, c)   = t[3]        - zmp_constant *  6.0 * t[1];
      zmp.M(n, d)   = t[2]        - zmp_constant *  2.0;
      zmp.M.row(n) += t[1]*Ek.v;
      zmp.M.row(n) += t[0]*Fk.v;

      zmp.v[n] = Ek.s*t[1] + Fk.s;

      ++n;
    }
  }

  assert(n<=num_nodes); // check that Eigen matrix didn't overflow
  return zmp;
}


} /* namespace zmp */
} /* namespace xpp */
