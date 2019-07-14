#include "Python.h"
#include "pytowr.h"
#include "stdio.h"


PyObject* eigenwrapper(Eigen::Vector3d a)
{
  return Py_BuildValue("(ddd)",a(0),a(1),a(2));
}

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

  PyObject* res = PyList_New(0);
  // for (int i = 0;i<a;i++)
    // PyList_SetItem(res,0,Py_BuildValue("i",a));
    // PyList_Append(res,Py_BuildValue("i",a));
  // return res;

  double t = 0.0;
  while (t<=solution.base_linear_->GetTotalTime() + 1e-5) {
    cout << "t=" << t << "\n";
    cout << "Base linear position x,y,z:   \t";
    cout << solution.base_linear_->GetPoint(t).p().transpose() << "\t[m]" << endl;
    // PyList_Append(res,eigenwrapper(solution.base_linear_->GetPoint(t).p()));
    PyObject* basePos = eigenwrapper(solution.base_linear_->GetPoint(t).p());
    cout << "Base Euler roll, pitch, yaw:  \t";
    Eigen::Vector3d rad = solution.base_angular_->GetPoint(t).p();
    cout << (rad/M_PI*180).transpose() << "\t[deg]" << endl;
    PyObject* baseEuler = eigenwrapper(solution.base_angular_->GetPoint(t).p());

    cout << "Foot position x,y,z:          \t";
    cout << solution.ee_motion_.at(0)->GetPoint(t).p().transpose() << "\t[m]" << endl;
    PyObject* footposition = eigenwrapper(solution.ee_motion_.at(0)->GetPoint(t).p());

    cout << "Contact force x,y,z:          \t";
    cout << solution.ee_force_.at(0)->GetPoint(t).p().transpose() << "\t[N]" << endl;

    bool contact = solution.phase_durations_.at(0)->IsContactPhase(t);
    PyObject* footstate = Py_BuildValue("(Oi)",footposition,contact);
    std::string foot_in_contact = contact? "yes" : "no";
    cout << "Foot in contact:              \t" + foot_in_contact << endl;

    PyList_Append(res,Py_BuildValue("(OOO)",basePos,baseEuler,footstate));
    cout << endl;

    t += 0.2;
  }

  return res;
}



void SetTowrInitialState(towr::NlpFormulation &formulation_)
{
    using namespace towr;
    auto nominal_stance_B = formulation_.model_.kinematic_model_->GetNominalStanceInBase();

    double z_ground = 0.0;
    formulation_.initial_ee_W_ =  nominal_stance_B;
    std::for_each(formulation_.initial_ee_W_.begin(), formulation_.initial_ee_W_.end(),
                  [&](Eigen::Vector3d& p){ p.z() = z_ground; } // feet at 0 height
    );

    formulation_.initial_base_.lin.at(kPos).z() = - nominal_stance_B.front().z() + z_ground;
}

towr::Parameters GetTowrParameters(int n_ee) 
// int GetTowrParameters(int n_ee, towr::NlpFormulation &formulation_) 

  {
    using namespace towr;
    Parameters towrparams(false);
    // Parameters a = Parameters();

    // Parameters a = Parameters();


    // Instead of manually defining the initial durations for each foot and
    // step, for convenience we use a GaitGenerator with some predefined gaits
    // for a variety of robots (walk, trot, pace, ...).
    auto gait_gen_ = GaitGenerator::MakeGaitGenerator(n_ee);
    auto id_gait   = static_cast<GaitGenerator::Combos>(0);
    gait_gen_->SetCombo(id_gait);
    double total_duration = 2.0;
    for (int ee=0; ee<n_ee; ++ee) {
      towrparams.ee_phase_durations_.push_back(gait_gen_->GetPhaseDurations(total_duration, ee));
      towrparams.ee_in_contact_at_start_.push_back(gait_gen_->IsInContactAtStart(ee));
    }
    // for (int ee=0; ee<n_ee; ++ee) {
    //   formulation_.params_.ee_phase_durations_.push_back(gait_gen_->GetPhaseDurations(total_duration, ee));
    //   formulation_.params_.ee_in_contact_at_start_.push_back(gait_gen_->IsInContactAtStart(ee));
    // }
    // Here you can also add other constraints or change parameters
    // towrparams.constraints_.push_back(Parameters::BaseRom);
    // increases optimization time, but sometimes helps find a solution for
    // more difficult terrain.
    
    return towrparams;
    // return 1;
  }


class pyterrain : public towr::HeightMap {
public:
  pyterrain(PyObject *func):terrainCallback(func){}
  double GetHeight(double x, double y) const override{
    PyObject *arglist;
    PyObject *result;
    double height;
    /* Time to call the callback */
    arglist = Py_BuildValue("(dd)", x,y);
    result = PyObject_CallObject(terrainCallback, arglist);
    if (!PyArg_Parse(result, "d", &height)) {
      std::cout<<"cannot parse the height returned from call back"<<std::endl;
      Py_DECREF(arglist);
      Py_DECREF(result);
      return NULL;
    }
    Py_DECREF(arglist);
    Py_DECREF(result);
    return height;
  }
private:
  PyObject *terrainCallback;
};

static PyObject *py_test_callback(PyObject *self, PyObject *args) {
  PyObject *func;
  if (!PyArg_ParseTuple(args, "O",&func)) {
    return NULL;
  }
  if (!PyCallable_Check(func)) {
      PyErr_SetString(PyExc_TypeError, "parameter must be callable");
      return NULL;
  }
  using namespace towr;
  auto terrain = std::make_shared<pyterrain>(func);
  terrain->GetHeight(0,0);
  return Py_None;
}

static PyObject *py_run(PyObject *self, PyObject *args) {
  double a, b, timescale;
  PyObject *func;
  if (!PyArg_ParseTuple(args, "dddO", &a, &b, &timescale, &func)) {
    return NULL;
  }
  using namespace towr;
  std::cout<<"### TARGET:" <<a<<" "<<b<<"###"<<std::endl;
  NlpFormulation formulation;

  // terrain
  // formulation.terrain_ = std::make_shared<FlatGround>(0.0);
  formulation.terrain_ = std::make_shared<pyterrain>(func);
  formulation.model_ = RobotModel(RobotModel::Hexpod);
  double robot_z = 0.45;
  // set the initial position of the hopper
  formulation.initial_base_.lin.at(kPos).z() = robot_z;
  auto nominal_stance_B = formulation.model_.kinematic_model_->GetNominalStanceInBase();
  formulation.initial_ee_W_ = nominal_stance_B;

  // // define the desired goal state of the hopper
  formulation.final_base_.lin.at(towr::kPos) << a, b, robot_z;

  int n_ee=6;
  // formulation.params_ = GetTowrParameters(n_ee);
  formulation.params_ = GetTowrParameters(n_ee);

  SetTowrInitialState(formulation);

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
  // solver_->SetOption("linear_solver", "mumps"); //  alot faster,

  solver->SetOption("max_cpu_time", 60.0);
  solver->SetOption("max_iter", 3000); // according to the towr_ros_app.cc
  solver->Solve(nlp);

  using namespace std;
  cout.precision(2);
  nlp.PrintCurrent(); // view variable-set, constraint violations, indices,...
  PyObject* res = PyList_New(0);

  double t = 0.0;
  while (t<=solution.base_linear_->GetTotalTime() + 1e-5) {
    // cout << "t=" << t << "\n";
    // cout << "Base linear position x,y,z:   \t";
    cout << solution.base_linear_->GetPoint(t).p().transpose() << "\t[m]" << endl;
    // PyList_Append(res,eigenwrapper(solution.base_linear_->GetPoint(t).p()));
    PyObject* basePos = eigenwrapper(solution.base_linear_->GetPoint(t).p());
    // cout << "Base Euler roll, pitch, yaw:  \t";
    Eigen::Vector3d rad = solution.base_angular_->GetPoint(t).p();
    // cout << (rad/M_PI*180).transpose() << "\t[deg]" << endl;
    PyObject* baseEuler = eigenwrapper(solution.base_angular_->GetPoint(t).p());

    PyObject* footstates[6];
    for(int i = 0;i<6;i++){ 
      // cout << "Foot position x,y,z:          \t";
      // cout << solution.ee_motion_.at(0)->GetPoint(t).p().transpose() << "\t[m]" << endl;
      PyObject* footposition = eigenwrapper(solution.ee_motion_.at(i)->GetPoint(t).p());

      // cout << "Contact force x,y,z:          \t";
      // cout << solution.ee_force_.at(0)->GetPoint(t).p().transpose() << "\t[N]" << endl;

      bool contact = solution.phase_durations_.at(i)->IsContactPhase(t);
      PyObject* footstate = Py_BuildValue("(Oi)",footposition,contact);
      // std::string foot_in_contact = contact? "yes" : "no";
      // cout << "Foot in contact:              \t" + foot_in_contact << endl;
      footstates[i] = footstate;
    }
    PyObject* foots = Py_BuildValue("(OOOOOO)",footstates[0],footstates[1],footstates[2],
                                      footstates[3],footstates[4],footstates[5]);
    
    
    PyList_Append(res,Py_BuildValue("(dOOO)",t,basePos,baseEuler,foots));
    // cout << endl;

    // t += 0.2;
    t += timescale;
  }

  int solve_cost = nlp.GetIterationCount();
  // return res;
  return Py_BuildValue("(Oi)",res,solve_cost);
}




/* Module method table */
static PyMethodDef PytowrMethods[] = {
  {"sample_run", py_sample_run, METH_VARARGS, "run the optimization of a monoped"}, // copid from tutorial
  {"run", py_run, METH_VARARGS, "the most import function for the hexpod"}, // copid from tutorial
  {"test_callback", py_test_callback, METH_VARARGS, "test the call back function implementation, for debug use"}, // copid from tutorial
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