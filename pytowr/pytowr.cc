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
    /**
     * Ture and False can set whether to use the elongation constraints
     */
    Parameters towrparams(false); 
    // Parameters towrparams(true);

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


    // towrparams.OptimizePhaseDurations();
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
      return 0.0;
    }
    Py_DECREF(arglist);
    Py_DECREF(result);
    return height;
  }
private:
  PyObject *terrainCallback;
};


/**
 * conver an numpy array to eigen vector or matrix
 */
Eigen::MatrixXd numpy2eigen(PyObject* np, bool tomatrix = false)
{
  PyObject* pybytes = PyObject_CallMethod(np,"tobytes",NULL); // this is a new reference thus should be cleaned
  char* bytes = PyBytes_AsString(pybytes); // the pointer to internal buffer, !!! should not be modified!!!
  int m,n;
  if(tomatrix){
    PyObject* shape = PyObject_GetAttrString(np,"shape");
    PyArg_ParseTuple(shape, "ii", &m, &n);
    Py_DECREF(shape);
  }else{
    PyObject* size = PyObject_GetAttrString(np,"size");
    m = PyLong_AsLong(size);   
    n = 1;
    Py_DECREF(size);
  }
  Eigen::Map<Eigen::Matrix <double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor> > mf( (double*)bytes,m,n);
  Py_DECREF(pybytes);
  return mf;
}


// Eigen::MatrixXd numpy2eigen(PyObject* np, bool tomatrix = false)
// {
//   PyObject* pybytes = PyObject_CallMethod(np,"tobytes",NULL); // this is a new reference thus should be cleaned
//   PyObject* size = PyObject_GetAttrString(np,"size");
//   int n = PyLong_AsLong(size);   
//   char* bytes = PyBytes_AsString(pybytes); // the pointer to internal buffer, !!! should not be modified!!!
  
//   Eigen::Map<Eigen::VectorXd> mf( (double*)bytes,n);
//   Py_DECREF(pybytes);
//   Py_DECREF(size);
//   return mf;
// }

PyObject* eigen2numpy(Eigen::VectorXd egn)
{
  
  double* v = egn.data();
  PyObject* bytes = PyBytes_FromStringAndSize((char*)v,egn.size()*sizeof(double)); // It seems that I do not know how to import numpy module
  return Py_BuildValue("(Oii)",bytes, egn.rows(),egn.cols());
}

static PyObject *py_test_callback(PyObject *self, PyObject *args) {
  // PyObject *func;
  // if (!PyArg_ParseTuple(args, "O",&func)) {
  //   return NULL;
  // }
  // if (!PyCallable_Check(func)) {
  //     PyErr_SetString(PyExc_TypeError, "parameter must be callable");
  //     return NULL;
  // }
  // using namespace towr;
  // auto terrain = std::make_shared<pyterrain>(func);
  // terrain->GetHeight(0,0);
  // return Py_None;
  PyObject *np;
  if (!PyArg_ParseTuple(args, "O",&np)) {
    return NULL;
  }
  std::cout<<numpy2eigen(np) <<std::endl;
  return Py_None;
}


const char* variableNames[] = {"base-lin", "base-ang",  // this should be const, otherwise it yield warning
                        "ee-motion_0", "ee-force_0", 
                        "ee-motion_1", "ee-force_1", 
                        "ee-motion_2", "ee-force_2", 
                        "ee-motion_3", "ee-force_3", 
                        "ee-motion_4", "ee-force_4", 
                        "ee-motion_5", "ee-force_5"};
/**
 * Same with the py_run except that this does not actually calculate a solution, it just returns the init value
 */
static PyObject *py_initValues(PyObject *self, PyObject *args) 
{
  double a, b, timescale;
  PyObject *func;
  if (!PyArg_ParseTuple(args, "dddO", &a, &b, &timescale, &func)) {
    return NULL;
  }
  using namespace towr;
  std::cout<<"### TARGET:" <<a<<" "<<b<<"###"<<std::endl;
  NlpFormulation formulation;

  // terrain
  formulation.terrain_ = std::make_shared<pyterrain>(func);
  formulation.model_ = RobotModel(RobotModel::Hexpod);
  double robot_z = 0.45;
  // set the initial position
  formulation.initial_base_.lin.at(kPos).z() = robot_z;
  auto nominal_stance_B = formulation.model_.kinematic_model_->GetNominalStanceInBase();
  formulation.initial_ee_W_ = nominal_stance_B;

  // define the desired goal state
  formulation.final_base_.lin.at(towr::kPos) << a, b, robot_z;

  int n_ee=6;
  formulation.params_ = GetTowrParameters(n_ee);

  SetTowrInitialState(formulation);

  ifopt::Problem nlp;
  SplineHolder solution;
  for (auto c : formulation.GetVariableSets(solution))
    nlp.AddVariableSet(c);
  for (auto c : formulation.GetConstraints(solution))
    nlp.AddConstraintSet(c);
  for (auto c : formulation.GetCosts())
    nlp.AddCostSet(c);

  PyObject *variableDict = PyDict_New();

  ifopt::Composite::Ptr VariablePtr = nlp.GetOptVariables();
  for (auto varstr : variableNames ){
    // std::cout<<"varstr: "<<varstr<<std::endl;
    ifopt::Component::Ptr componentPtr = VariablePtr -> GetComponent(varstr);
    PyDict_SetItemString(variableDict, varstr, eigen2numpy(componentPtr->GetValues()));
  }
  return variableDict;
}


static PyObject *py_run(PyObject *self, PyObject *args) {
  /**
   * input:
   *  target pos x
   *  target pos y
   *  output time scale
   *  terrian call back function
   *  init value dict {variable_name(pystring) : value(pylist)}
   *  Posture: a tuple: (body height, stance pos)
   *      stancePos: the init positure of the robot(init EE positions in global cordinate sys), should be a 6*3 numpy array. 
   *      None if the norminal_stance(defined in the robot model) is to be used
   */
  double a, b, timescale;
  PyObject *func;
  PyObject *initmap;
  PyObject *posture;
  if (!PyArg_ParseTuple(args, "dddOOO", &a, &b, &timescale, &func, &initmap, &posture)) {
    return NULL;
  }
  // if (!PyArg_ParseTuple(args, "dddOO", &a, &b, &timescale, &func, &initmap)) {
  //   return NULL;
  // }
  const int n_ee=6;
  using namespace towr;
  std::cout<<"### TARGET:" <<a<<" "<<b<<"###"<<std::endl;
  NlpFormulation formulation;

  // terrain
  // formulation.terrain_ = std::make_shared<FlatGround>(0.0);
  formulation.terrain_ = std::make_shared<pyterrain>(func);
  formulation.model_ = RobotModel(RobotModel::Hexpod);
  
  double robot_z = 0.45;

  //set the initial ee position
  if(posture == Py_None){ // None is passed, use the default value
    auto nominal_stance_B = formulation.model_.kinematic_model_->GetNominalStanceInBase();
    formulation.initial_ee_W_ = nominal_stance_B;
    double z_ground = 0.0;
    std::for_each(formulation.initial_ee_W_.begin(), formulation.initial_ee_W_.end(),
                  [&](Eigen::Vector3d& p){ p.z() = z_ground; } // feet at 0 height
    );
    formulation.initial_base_.lin.at(kPos).z() = - nominal_stance_B.front().z() + z_ground;
  }else{
    PyObject *stancePos;
    double bodyHeight;
    if (!PyArg_ParseTuple(args, "dO", &bodyHeight, &stancePos)) {
      return NULL;
    }
    towr::KinematicModel::EEPos init_ee_W;
    Eigen::Matrix<double, n_ee,3 > ee_W_mat = numpy2eigen(stancePos,true);
    for(int i = 0;i<n_ee;i++){
      init_ee_W.push_back(ee_W_mat.row(i) );
    }
    formulation.initial_ee_W_ = init_ee_W;
    formulation.initial_base_.lin.at(kPos).z() = bodyHeight;
  }
  // // define the desired goal state
  formulation.final_base_.lin.at(towr::kPos) << a, b, robot_z;
  // formulation.final_base_.ang.at(towr::kPos) << 0, 0, 1.57;

  
  // formulation.params_ = GetTowrParameters(n_ee);
  formulation.params_ = GetTowrParameters(n_ee);

  // SetTowrInitialState(formulation);

  // Initialize the nonlinear-programming problem with the variables,
  // constraints and costs.
  /**
   * 这里的一个variable 是一个 NodesVariablesAll
   * 它的父类一路上去是 towr::NodesVariables -> ifopt::VariableSet -> ifopt::Component (composite是component的另一个子类,不知道和variable set有什么区别)
   */
  ifopt::Problem nlp;
  SplineHolder solution;
  for (auto c : formulation.GetVariableSets(solution))
    nlp.AddVariableSet(c);
  for (auto c : formulation.GetConstraints(solution))
    nlp.AddConstraintSet(c);
  for (auto c : formulation.GetCosts())
    nlp.AddCostSet(c);

  /**
   * Update the variables
   */ 
  ifopt::Composite::Ptr VariablePtr = nlp.GetOptVariables();
  PyObject *key, *value;
  Py_ssize_t pos = 0;
  // std::cout<<"before init"<<std::endl;
  while (PyDict_Next(initmap, &pos, &key, &value)) {
    key = PyUnicode_AsUTF8String(key);
    std::string valName = std::string( PyBytes_AsString(key) );
    std::cout<<"key: "<<valName<<std::endl;
    ifopt::Component::Ptr compoPtr = VariablePtr -> GetComponent(valName);
    compoPtr -> SetVariables( numpy2eigen(value) );
  }
  
  /**
   * Bound x,y,z
   * Get the meaning of index via GetNodeValuesInfo
   * Each info is a struct containing the following attributes
   *  int id_;   ///< ID of the associated node (0 =< id < number of nodes in spline).
   *  Dx deriv_; ///< Derivative (pos,vel) of the node with that ID.
   *  int dim_;  ///< Dimension (x,y,z) of that derivative.
   */
#ifdef LOCKDIM
  std::shared_ptr<ifopt::Component> basecompon = VariablePtr -> GetComponent("base-lin");
  towr::NodesVariablesAll::Ptr BaselinPtr = std::dynamic_pointer_cast<towr::NodesVariables>(basecompon);
  Eigen::VectorXd baseVariables = BaselinPtr->GetValues(); //Need to cast from Component to NodesVariablesAll
  for(auto deriv : {towr::kPos} ) // can have `towr::kVel` if you also want to lock velocity
    for(auto dim : {0,1} ) // Bound the x,y dimension
      BaselinPtr->LockBound(deriv,dim,baseVariables);
#endif

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
    // cout << solution.base_linear_->GetPoint(t).p().transpose() << "\t[m]" << endl;
    // PyList_Append(res,eigenwrapper(solution.base_linear_->GetPoint(t).p()));
    PyObject* basePos = eigenwrapper(solution.base_linear_->GetPoint(t).p());
    // cout << "Base Euler roll, pitch, yaw:  \t";
    // Eigen::Vector3d rad = solution.base_angular_->GetPoint(t).p();
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

  /**
   * Build a dict to return the final variables
   */
  PyObject *variableDict = PyDict_New();
  
  for (auto varstr : variableNames ){
    // std::cout<<"varstr: "<<varstr<<std::endl;
    ifopt::Component::Ptr componentPtr = VariablePtr -> GetComponent(varstr);
    PyDict_SetItemString(variableDict, varstr, eigen2numpy(componentPtr->GetValues()));
  }
    
  // return res;
  return Py_BuildValue("(OiO)",res,solve_cost,variableDict);
}




/* Module method table */
static PyMethodDef PytowrMethods[] = {
  {"sample_run", py_sample_run, METH_VARARGS, "run the optimization of a monoped"}, // copid from tutorial
  {"run", py_run, METH_VARARGS, "the most import function for the hexpod"}, // copid from tutorial
  {"test_callback", py_test_callback, METH_VARARGS, "test the call back function implementation, for debug use"}, // copid from tutorial
  {"initValues", py_initValues, METH_VARARGS, "return the initvalues of the variables"}, // copid from tutorial
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