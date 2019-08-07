
#include <cmath>
#include <iostream>

#include <towr/terrain/examples/height_map_examples.h>
#include <towr/nlp_formulation.h>
#include <towr/initialization/gait_generator.h>
#include <ifopt/ipopt_solver.h>
#include <towr/variables/nodes_variables_all.h>

// #define LOCKDIM // whether to lock the x,y dimention of the hexpods body
// #define OPTMIZE_DURATION // whether to optimize the duration(change the phase and gait)
// #define DYNAMIC_CONSTRAINT // whether to use dynamic constraints
// #define BOX_CONSTRAINT // whether to use range of motion box constraint or to use elongation constraint
extern void sample_run(int a, int b);

