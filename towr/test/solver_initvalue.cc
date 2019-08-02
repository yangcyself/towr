/******************************************************************************
Copyright (c) 2018, Alexander W. Winkler. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

/usr/bin/c++  -I/home/yangcy/programs/towr/towr/include -isystem /usr/local/include -isystem /usr/local/include/eigen3 \
  -g   -std=gnu++11 -o test.out  /home/yangcy/programs/towr/towr/test/solver_initvalue.cc \
src/* -rdynamic build/libtowr.so /usr/local/lib/libifopt_ipopt.so /usr/local/lib/libifopt_core.so -Wl,-rpath,/home/yangcy/programs/towr/towr/build:/usr/local/lib
******************************************************************************/

#include <cmath>
#include <iostream>

#include <towr/terrain/examples/height_map_examples.h>
#include <towr/nlp_formulation.h>
#include <towr/initialization/gait_generator.h>
#include <ifopt/ipopt_solver.h>

// #include <towr/variables/variable_names.h> // is this necessary

using namespace towr;

// First search a result and use it as the init value of the next search

int main()
{
  NlpFormulation formulation;

  // terrain
  formulation.terrain_ = std::make_shared<FlatGround>(0.0);
  //formulation.terrain_ = std::make_shared<HalfBlock>();

  // Kinematic limits and dynamic parameters of the hopper
  formulation.model_ = RobotModel(RobotModel::Monoped);
  // formulation.model_ = RobotModel(RobotModel::Hexpod);
  // formulation.model_ = RobotModel(RobotModel::Biped);

  double z_ground = 0.0;
  auto nominal_stance_B = formulation.model_.kinematic_model_->GetNominalStanceInBase();
  double robot_z =  - nominal_stance_B.front().z() + z_ground;
  formulation.initial_ee_W_ =  nominal_stance_B;
  std::for_each(formulation.initial_ee_W_.begin(), formulation.initial_ee_W_.end(),
                [&](Eigen::Vector3d& p){ p(2) = z_ground; } // feet at 0 height
  );

  formulation.initial_base_.lin.at(kPos).z() = robot_z;
  // define the desired goal state of the hopper
  formulation.final_base_.lin.at(towr::kPos) << 1.3, 0, robot_z;

  // Parameters that define the motion. See c'tor for default values or
  // other values that can be modified.
  // First we define the initial phase durations, that can however be changed
  // by the optimizer. The number of swing and stance phases however is fixed.
  // alternating stance and swing:     ____-----_____-----_____-----_____
  // formulation.params_.ee_phase_durations_.push_back({0.4, 0.2, 0.4, 0.2, 0.4, 0.2, 0.2});
  int n_ee=1;
  auto gait_gen_ = GaitGenerator::MakeGaitGenerator(n_ee);

  gait_gen_->SetCombo(towr::GaitGenerator::C0);
  double total_duration = 2;
  for (int ee=0; ee<n_ee; ++ee) {
    formulation.params_.ee_phase_durations_.push_back(gait_gen_->GetPhaseDurations(total_duration, ee));
    formulation.params_.ee_in_contact_at_start_.push_back(gait_gen_->IsInContactAtStart(ee));
  }
  // Initialize the nonlinear-programming problem with the variables,
  // constraints and costs.
  ifopt::Problem nlp;
  SplineHolder solution;
  for (auto c : formulation.GetVariableSets(solution))
    nlp.AddVariableSet(c);
  for (auto c : formulation.GetConstraints(solution))
    nlp.AddConstraintSet(c);
  for (auto c : formulation.GetCosts())
    nlp.AddCostSet(c);

  // You can add your own elements to the nlp as well, simply by calling:
  // nlp.AddVariablesSet(your_custom_variables);
  // nlp.AddConstraintSet(your_custom_constraints);

  using namespace std;
  cout<<"Before solving\n######################"<<endl;
  
  ifopt::Composite::Ptr VariablePtr = nlp.GetOptVariables();
  cout <<"Total variable number: " << VariablePtr -> GetRows() <<endl;
  ifopt::Component::Ptr BaselinPtr = VariablePtr -> GetComponent("base-lin");
  cout <<"Base lin variable number: " << BaselinPtr -> GetRows() <<endl;
  ifopt::Component::Ptr BaseangPtr = VariablePtr -> GetComponent("base-ang");
  cout <<"Base ang variable number: " << BaseangPtr -> GetRows() <<endl;
  ifopt::Component::Ptr EEmotionPtr = VariablePtr -> GetComponent("ee-motion_0");
  cout <<"EE motion variable number: " << EEmotionPtr -> GetRows() <<endl;
  ifopt::Component::Ptr EEforcePtr = VariablePtr -> GetComponent("ee-force_0");
  cout <<"EE force variable number: " << EEforcePtr -> GetRows() <<endl;

  cout <<"EE lin variable first10: " << (BaselinPtr -> GetValues()).head(10) <<endl;

  nlp.PrintCurrent();
  // Choose ifopt solver (IPOPT or SNOPT), set some parameters and solve.
  // solver->SetOption("derivative_test", "first-order");
  auto solver = std::make_shared<ifopt::IpoptSolver>();
  solver->SetOption("jacobian_approximation", "exact"); // "finite difference-values"
  solver->SetOption("max_cpu_time", 60.0);
  solver->Solve(nlp);

  cout<<"finished solving first time"<<endl;
  cout<<"optimizing times:" << nlp.GetIterationCount()<<endl;
  cout <<"EE lin variable first10: " << (BaselinPtr -> GetValues()).head(10) <<endl;


  /**
   * #################################################################
   * Start the Second search
   * #################################################################
   */

  NlpFormulation formulation2;

  // terrain
  formulation2.terrain_ = std::make_shared<FlatGround>(0.0);
  formulation2.model_ = RobotModel(RobotModel::Monoped);
  formulation2.initial_base_.lin.at(kPos).z() = robot_z;
  formulation2.initial_ee_W_.push_back(Eigen::Vector3d::Zero());
  formulation2.final_base_.lin.at(towr::kPos) << 1.3, 0, robot_z;
  
  for (int ee=0; ee<n_ee; ++ee) {
    formulation2.params_.ee_phase_durations_.push_back(gait_gen_->GetPhaseDurations(total_duration, ee));
    formulation2.params_.ee_in_contact_at_start_.push_back(gait_gen_->IsInContactAtStart(ee));
  }
  ifopt::Problem nlp2;
  SplineHolder solution2;
  for (auto c : formulation2.GetVariableSets(solution))
    nlp2.AddVariableSet(c);
  for (auto c : formulation2.GetConstraints(solution))
    nlp2.AddConstraintSet(c);
  for (auto c : formulation2.GetCosts())
    nlp2.AddCostSet(c);


  cout<<"Before solving2\n###################"<<endl;
  
  ifopt::Composite::Ptr VariablePtr2 = nlp2.GetOptVariables();
  cout <<"Total variable number: " << VariablePtr2 -> GetRows() <<endl;
  ifopt::Component::Ptr BaselinPtr2 = VariablePtr2 -> GetComponent("base-lin");
  ifopt::Component::Ptr BaseangPtr2 = VariablePtr2 -> GetComponent("base-ang");
  cout <<"Base lin variable number: " << BaselinPtr2 -> GetRows() <<endl;
  ifopt::Component::Ptr EEmotionPtr2 = VariablePtr2 -> GetComponent("ee-motion_0");
  cout <<"EE motion variable number: " << EEmotionPtr2 -> GetRows() <<endl;
  ifopt::Component::Ptr EEforcePtr2 = VariablePtr2 -> GetComponent("ee-force_0");

  double NoiseRange = 1;
  BaselinPtr2 -> SetVariables(BaselinPtr -> GetValues() + Eigen::VectorXd()::Random(BaselinPtr -> GetRows() ));
  EEmotionPtr2 -> SetVariables(EEmotionPtr -> GetValues());
  EEforcePtr2 -> SetVariables(EEforcePtr -> GetValues());


  cout<<"BaselinPtr2 lenth: " << BaselinPtr2->GetValues().rows()<<endl;

  cout <<"EE lin variable first10: " << (BaselinPtr2 -> GetValues()).head(10) <<endl;
  nlp2.PrintCurrent();
  // Choose ifopt solver (IPOPT or SNOPT), set some parameters and solve.
  // solver->SetOption("derivative_test", "first-order");
  auto solver2 = std::make_shared<ifopt::IpoptSolver>();
  solver2->SetOption("jacobian_approximation", "exact"); // "finite difference-values"
  solver2->SetOption("max_cpu_time", 60.0);
  solver2->Solve(nlp2);

  cout<<"finished solving second time"<<endl;
  cout<<"optimizing times:" << nlp2.GetIterationCount()<<endl;
  cout <<"EE lin variable first10: " << (BaselinPtr2 -> GetValues()).head(10) <<endl;


  // Can directly view the optimization variables through:
  // Eigen::VectorXd x = nlp.GetVariableValues()
  // However, it's more convenient to access the splines constructed from these
  // variables and query their values at specific times:
  // ifopt::Component::Ptr EEschedulePtr = VariablePtr -> GetComponent("ee-schedule");
  // cout <<"EE schedule variable number: " << EEschedulePtr -> GetRows() <<endl;

  // cout <<"Base variable results: " << BasePtr -> GetValues() <<endl;
  // cout.precision(2);
  // nlp.PrintCurrent(); // view variable-set, constraint violations, indices,...
  // cout << fixed;
  // cout << "\n====================\nMonoped trajectory:\n====================\n";

  // double t = 0.0;
  // while (t<=solution.base_linear_->GetTotalTime() + 1e-5) {
  //   cout << "t=" << t << "\n";
  //   cout << "Base linear position x,y,z:   \t";
  //   cout << solution.base_linear_->GetPoint(t).p().transpose() << "\t[m]" << endl;

  //   cout << "Base Euler roll, pitch, yaw:  \t";
  //   Eigen::Vector3d rad = solution.base_angular_->GetPoint(t).p();
  //   cout << (rad/M_PI*180).transpose() << "\t[deg]" << endl;

  //   cout << "Foot position x,y,z:          \t";
  //   cout << solution.ee_motion_.at(0)->GetPoint(t).p().transpose() << "\t[m]" << endl;

  //   cout << "Contact force x,y,z:          \t";
  //   cout << solution.ee_force_.at(0)->GetPoint(t).p().transpose() << "\t[N]" << endl;

  //   bool contact = solution.phase_durations_.at(0)->IsContactPhase(t);
  //   std::string foot_in_contact = contact? "yes" : "no";
  //   cout << "Foot in contact:              \t" + foot_in_contact << endl;

  //   cout << endl;

  //   t += 0.2;
  // }
}
