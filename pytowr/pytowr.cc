#include "Python.h"
#include "pytowr.h"
#include "stdio.h"



/* int divide(int, int, int *) */
static PyObject *py_sample_run(PyObject *self, PyObject *args) {
  int a, b;
  if (!PyArg_ParseTuple(args, "ii", &a, &b)) {
    return NULL;
  }
  using namespace towr;
  NlpFormulation formulation;

  // terrain
  formulation.terrain_ = std::make_shared<FlatGround>(0.0);
  formulation.model_ = RobotModel(RobotModel::Monoped);
  double robot_z = 0.5; // hopper
  // set the initial position of the hopper
  formulation.initial_base_.lin.at(kPos).z() = robot_z;
  auto nominal_stance_B = formulation.model_.kinematic_model_->GetNominalStanceInBase();
  formulation.initial_ee_W_ = nominal_stance_B;

  // define the desired goal state of the hopper
  formulation.final_base_.lin.at(towr::kPos) << 1.3, 0, robot_z;

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

  auto solver = std::make_shared<ifopt::IpoptSolver>();
  solver->SetOption("jacobian_approximation", "exact"); // "finite difference-values"
  solver->SetOption("max_cpu_time", 60.0);
  solver->Solve(nlp);

  using namespace std;
  cout.precision(2);
  nlp.PrintCurrent(); // view variable-set, constraint violations, indices,...
  cout << fixed;
  cout << "\n====================\nMonoped trajectory:\n====================\n";

  double t = 0.0;
  while (t<=solution.base_linear_->GetTotalTime() + 1e-5) {
    cout << "t=" << t << "\n";
    cout << "Base linear position x,y,z:   \t";
    cout << solution.base_linear_->GetPoint(t).p().transpose() << "\t[m]" << endl;

    cout << "Base Euler roll, pitch, yaw:  \t";
    Eigen::Vector3d rad = solution.base_angular_->GetPoint(t).p();
    cout << (rad/M_PI*180).transpose() << "\t[deg]" << endl;

    cout << "Foot position x,y,z:          \t";
    cout << solution.ee_motion_.at(0)->GetPoint(t).p().transpose() << "\t[m]" << endl;

    cout << "Contact force x,y,z:          \t";
    cout << solution.ee_force_.at(0)->GetPoint(t).p().transpose() << "\t[N]" << endl;

    bool contact = solution.phase_durations_.at(0)->IsContactPhase(t);
    std::string foot_in_contact = contact? "yes" : "no";
    cout << "Foot in contact:              \t" + foot_in_contact << endl;

    cout << endl;

    t += 0.2;
  }

  return NULL;
}

/* Module method table */
static PyMethodDef PytowrMethods[] = {
  {"sample_run", py_sample_run, METH_VARARGS, "run the optimization of a monoped"}, // copid from tutorial
  { NULL, NULL, 0, NULL}
};

/* Module structure */
static struct PyModuleDef pytowrmodule = {
  PyModuleDef_HEAD_INIT,

  "pytowr",           /* name of module */
  "the python wrapper of towr",  /* Doc string (may be NULL) */
  -1,                 /* Size of per-interpreter state or -1 */
  PytowrMethods       /* Method table */
};

/* Module initialization function */
PyMODINIT_FUNC
PyInit_pytowr(void) {
  printf("hahaha I'm Initialized\n");
  return PyModule_Create(&pytowrmodule);
}