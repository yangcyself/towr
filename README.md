# pytowr
The python wrapper for the towr.

The aim for this fork is to expose the towr program to other trajectory optimization program and simulation environment, via python.

## setup pytowr

### compile the towr library
following the [cmake method](#towr-with-cmake) to compile towr
to compile `libtowr.so` that we will use later.

run the following command in the folder [pytowr](pytowr)

```bash
#bash pytowr/
python3 setup.py build_ext --inplace
```
Then the module `pytowr` can be used when `pytowr.cpython-35m-x86_64-linux-gnu.so` is in python path.

### on compiler error
I hard coded the general include path and the library path in the [setup script](pytowr/setup.py). If there is some compiler error, the paths are the first things to be checked.

#### to get the paths in your environment, look at the camkefile and its caches in towr
first make a build dir in towr, and use cmake to build in it following the [original procedure](#towr-with-cmake).

then check the [link.txt](towr/build/CMakeFiles/towr-example.dir/link.txt) and see where your `libifopt_core.so` is located, and put the location into the "library_dirs" field in [setup.py](pytowr/setup.py)

then check the [flags.make](towr/build/CMakeFiles/towr-example.dir/flags.make) to see the include path and other flags

## use pytowr
```python
# call python with LD_LIBRARY_PATH=$LD_LIBRARY_PATH:<path-to-libtowr.so> python3
import pytowr
pytowr.sample_run(1,2) # runs the trajectory search of monoped on flat plane.

terrain = lambda x,y: 0.0 # the terrain function
pos, cost = pytowr.run(5.,0.,0.1, terrain) # target x, target y, time scale of the return list
print("cost",cost)
def showtime(i):
    a = pos[i]
    print("time: ",a[0] ,"body: ",a[1],a[2])
    foots = a[3]
    for f,p in enumerate(foots):
        print( f, p ) # (position of foot point, is_contacting_ground)
```

# Blue_Towr

This fork of Towr aims to add some functionalities to work with little_blue_hexpod and to work in real envionment and real time

第二次修改添加的地方(加入并联电机长度约束):

1. range_of_elongation_constraint.h
   1. 这个是elongation constraint的头文件，这个类需要用到 parallel_kinematic_model
   2. 其中需要double的max和min length, Matrix3d 的 ped_root_pos
2. range_of_elongation_constraint.cc
   1. 这里主要是改了jac的计算方式，由于我们的约束变量变成了距离的平方，而之前的时候boundingbox 的变量是脚到身体的向量，所以我们的jac都要乘一个2*（脚到root）
   2. 得到数据利用model->GetMinimumLength 和 model->GetRootPosition(ee)
3. parallel_kinematic_model.h
   1. 是我创建的专门为了方便机器人这种并联结构的类，是KinematicModel的子类
   2. 除了继承了前面的东西之外，实现了GetRootPosition，GetMaximumLength的接口
4. hexpod_model.h
   1. 把hexpod model原先是KinematicModel的子类，现在变成ParallelKinematicModel 的子类
   2. 加入了root_positions， min_lengthmax_length 的参数定义
5. robot_model.h 
   1. 改了include
   2. （后面其实应该用类的那个啥来着，就是用基类指针访问高级功能，但是忘了是啥了，后面再改
6. towr_ros_app.cc
   1. 整个towr_ros_app的main函数在这里面，这个文件里面又初始化应用的过程，设置参数的过程，还有提交给IPOPT的过程， TowrRosApp  是 TowrRosInterface 的子类， 在towrinterface.cpp里面有调用 这里定义的SetIpoptParameters
   2. 在这里的设置参数的过程， 加了一个判断，如果n_ee = 6则params 的 useElongConstraint = true
7. parameters.h 和 parameters.cc
   1. parameters 用来选择求解的时候要用哪些约束
   2. 在里面的ConstraintName 里面加了 EEMotorRange
   3. parameter的初始化函数加了一个 useElongConstraint变量
8. nlp_formulation.cc 和 nlp_formulation.h
   1. nlp_formulation 是从model得到NLP的一个过程，调用它的逻辑在TowrRosInterface :: UserCommandCallback 里面
   2. 在GetConstraint里面加了case EEMotorRange的情况，并且NlpFormulation :: MakeRangeOfElongationConstraint (const SplineHolder& s) const 的定义

debug 的时候,程序的入口是:towr_ros/src/towr_ros_interface.cc UserCommandCallback
如果哪里运行中报错,从那里进入

添加的地方(加入小蓝机器人)：
1. robot_model.h
2. robot_model.cc
3. towrCommand.msg （没有啥用的啦）
4. 新增两个文件 hexpod_model.h 
5. 要增加对应的EEPos nominal_stance_
6. endeffector_mappings.h 增加六个脚的enum（按照vrep里面布局的convention
7. xpp_towr.rviz (未完成)
8. gait_generator.cc 
9. 增加hexaped_gait_generator.cc /h
10. CMakeList
11. towr_ros/include/towr_ros/towr_xpp_ee_map.h
    1.  增加了hexa_to_xpp_id (暂时使用的四足的xpp id)
    2.  hexa_to_name
# Original README
<img align="right" src="https://i.imgur.com/qI1Jfyl.gif" width="55%"/>

[<img src="https://i.imgur.com/qliQVx1.png" />](https://awinkler.github.io/publications/mypdfs/18-ral-winkler.pdf "Open RA-L paper")

*A light-weight and extensible C++ library for trajectory optimization for legged robots.*

[![Build Status](http://build.ros.org/buildStatus/icon?job=Kdev__towr__ubuntu_xenial_amd64)](http://build.ros.org/view/Kdev/job/Kdev__towr__ubuntu_xenial_amd64/)
[![Documentation](https://img.shields.io/badge/docs-generated-brightgreen.svg)](http://docs.ros.org/kinetic/api/towr/html/)
[![ROS hosting](https://img.shields.io/badge/ROS-integration-blue.svg)](http://wiki.ros.org/towr)
![](https://tokei.rs/b1/github/ethz-adrl/towr)
[![CodeFactor](https://www.codefactor.io/repository/github/ethz-adrl/towr/badge)](https://www.codefactor.io/repository/github/ethz-adrl/towr)
[![License BSD-3-Clause](https://img.shields.io/badge/license-BSD--3--Clause-blue.svg)](https://tldrlegal.com/license/bsd-3-clause-license-%28revised%29#fulltext)

A base-set of variables, costs and constraints that can be combined and extended to formulate trajectory optimization problems for legged systems. These implementations have been used to generate a variety of motions such as monoped hopping, biped walking, or a complete quadruped trotting cycle, while optimizing over the gait and step durations in less than 100ms ([paper](https://ieeexplore.ieee.org/document/8283570/)).  

Features:  
:heavy_check_mark: Inuitive and efficient formulation of variables, cost and constraints using [Eigen].   
:heavy_check_mark: [ifopt] enables using the high-performance solvers [Ipopt] and [Snopt].  
:heavy_check_mark: Elegant rviz visualization of motion plans using [xpp].  
:heavy_check_mark: [ROS]/[catkin] integration (optional).  
:heavy_check_mark: Light-weight ([~6k lines](https://i.imgur.com/gP3gv34.png) of code) makes it easy to use and extend.  

<br>

<p align="center">
  <a href="#install">Install</a> •
  <a href="#run">Run</a> •
  <a href="#develop">Develop</a> •
  <a href="#contribute">Contribute</a> •
  <a href="#publications">Publications</a> •
  <a href="#authors">Authors</a>
</p>

[<img src="https://i.imgur.com/8M4v4aP.gif" />](https://youtu.be/0jE46GqzxMM "Show more examples on Youtube")

## Install
The easiest way to install is through the [ROS binaries](http://wiki.ros.org/towr):
```bash
sudo apt-get install ros-<ros-distro>-towr-ros
```

In case these don't yet exist for your distro, there are two ways to build this code from source:
* [Option 1](#towr-with-cmake): core library and hopper-example with pure [CMake].
* [Option 2](#towr-ros-with-catkin) (recommended): core library & GUI & ROS-rviz-visualization built with [catkin] and [ROS]. 


#### <a name="towr-with-cmake"></a> Building with CMake
* Install dependencies [CMake], [Eigen], [Ipopt]:
  ```bash
  sudo apt-get install cmake libeigen3-dev coinor-libipopt-dev
  ```
  Install [ifopt], by cloning the repo and then: ``cmake .. && make install`` on your system. 

* Build towr:
  ```bash
  git clone https://github.com/ethz-adrl/towr.git && cd towr/towr
  mkdir build && cd build
  cmake .. -DCMAKE_BUILD_TYPE=Release
  make
  sudo make install # copies files in this folder to /usr/local/*
  # sudo xargs rm < install_manifest.txt # in case you want to uninstall the above
  ```

* Test ([hopper_example.cc](towr/test/hopper_example.cc)): Generates a motion for a one-legged hopper using Ipopt
  ```bash
  ./towr-example # or ./towr-test if gtest was found
  ```

* Use: You can easily customize and add your own constraints and variables to the optimization problem.
  Herefore, add the following to your *CMakeLists.txt*:
  ```cmake
  find_package(towr 1.2 REQUIRED)
  add_executable(main main.cpp) # Your custom variables, costs and constraints added to TOWR
  target_link_libraries(main PUBLIC towr::towr) # adds include directories and libraries
  ```

#### <a name="towr-ros-with-catkin"></a> Building with catkin
We provide a [ROS]-wrapper for the pure cmake towr library, which adds a keyboard interface to modify goal state and motion types as well as visualizes the produces motions plans in rviz using [xpp]. 

* Install dependencies [CMake], [catkin], [Eigen], [Ipopt], [ROS], [xpp], [ncurses], [xterm]:
  ```bash
  sudo apt-get install cmake libeigen3-dev coinor-libipopt-dev libncurses5-dev xterm
  sudo apt-get install ros-<ros-distro>-desktop-full ros-<ros-distro>-xpp
  ```

* Build workspace:
  ```bash
  cd catkin_workspace/src
  git clone https://github.com/ethz-adrl/ifopt.git
  git clone https://github.com/ethz-adrl/towr.git
  cd ..
  catkin_make_isolated -DCMAKE_BUILD_TYPE=Release # or `catkin build`
  source ./devel_isolated/setup.bash
  ```
  
* Use: Include in your catkin project by adding to your *CMakeLists.txt* 
  ```cmake
  add_compile_options(-std=c++11)
  find_package(catkin COMPONENTS towr) 
  include_directories(${catkin_INCLUDE_DIRS})
  target_link_libraries(foo ${catkin_LIBRARIES})
  ```
  Add the following to your *package.xml*:
  ```xml
  <package>
    <depend>towr</depend>
  </package>
  ```
  
## Run
  Launch the program using
  ```bash
  roslaunch towr_ros towr_ros.launch  # debug:=true  (to debug with gdb)
  ```
  Click in the xterm terminal and hit 'o'. 

  Information about how to tune the paramters can be found [here](http://docs.ros.org/api/towr/html/group__Parameters.html). 

## Develop
#### Library overview
 * The relevant classes and parameters to build on are collected [modules](http://docs.ros.org/api/towr/html/modules.html).
 * A nice graphical overview as UML can be seen [here](http://docs.ros.org/api/towr/html/inherits.html).
 * The [doxygen documentation](http://docs.ros.org/api/towr/html/) provides helpul information for developers.

#### Problem formulation
 * This code formulates the variables, costs and constraints using ifopt, so it makes sense to briefly familiarize with the syntax using [this example].
 * A minimal towr example without ROS, formulating a problem for a one-legged hopper, 
    can be seen [here](towr/test/hopper_example.cc) and is great starting point.
 * We recommend using the ROS infrastructure provided to dynamically visualize, plot and change the problem formulation. To define your own problem using this infrastructure, use this [example](towr_ros/src/towr_ros_app.cc) as a guide. 

#### Add your own variables, costs and constraints
 * This library provides a set of variables, costs and constraints to formulate the trajectory optimization problem. An [example formulation](towr/include/towr/nlp_formulation.h) of how to combine these is given, however, this formulation can probably be improved. To add your own e.g. constraint-set, define a class with it's values and derivatives, and then add it to the formulation ```nlp.AddConstraintSet(your_custom_constraints);``` as shown [here](towr/test/hopper_example.cc).

#### Add your own robot
 * Want to add your own robot to towr? Start [here](http://docs.ros.org/api/towr/html/group__Robots.html).
 * To visualize that robot in rviz, see [xpp].


## Contribute
We love pull request, whether its new constraint formulations, additional robot models, bug fixes, unit tests or updating the documentation. Please have a look at [CONTRIBUTING.md](CONTRIBUTING.md) for more information.  
See here the list of [contributors](https://github.com/ethz-adrl/towr/graphs/contributors) who participated in this project.


## Publications
All publications underlying this code can be found [here](https://www.alex-winkler.com). 
The core paper is:

    @article{winkler18,
      author    = {Winkler, Alexander W and Bellicoso, Dario C and 
                   Hutter, Marco and Buchli, Jonas},
      title     = {Gait and Trajectory Optimization for Legged Systems 
                   through Phase-based End-Effector Parameterization},
      journal   = {IEEE Robotics and Automation Letters (RA-L)},
      year      = {2018},
      month     = {July},
      pages     = {1560-1567},
      volume    = {3},
      doi       = {10.1109/LRA.2018.2798285},
    }

A broader overview of the topic of Trajectory optimization and derivation of 
the Single-Rigid-Body Dynamics model used in this work: 
[DOI 10.3929/ethz-b-000272432](https://doi.org/10.3929/ethz-b-000272432)  


## Authors 
[Alexander W. Winkler](https://www.alex-winkler.com) - Initial Work/Maintainer

The work was carried out at the following institutions:

[<img src="https://i.imgur.com/aGOnNTZ.png" height="45" />](https://www.ethz.ch/en.html "ETH Zurich") &nbsp; &nbsp; &nbsp; &nbsp; [<img src="https://i.imgur.com/uCvLs2j.png" height="45" />](http://www.adrl.ethz.ch/doku.php "Agile and Dexterous Robotics Lab")  &nbsp; &nbsp; &nbsp; &nbsp;[<img src="https://i.imgur.com/gYxWH9p.png" height="45" />](http://www.rsl.ethz.ch/ "Robotic Systems Lab")


[A. W. Winkler]: https://awinkler.github.io/publications.html
[CMake]: https://cmake.org/cmake/help/v3.0/
[std_msgs]: http://wiki.ros.org/std_msgs
[roscpp]: http://wiki.ros.org/roscpp
[message_generation]: http://wiki.ros.org/message_generation
[rosbag]: http://wiki.ros.org/rosbag
[HyQ]: https://www.iit.it/research/lines/dynamic-legged-systems
[ANYmal]: http://www.rsl.ethz.ch/robots-media/anymal.html
[ROS]: http://www.ros.org
[xpp]: http://wiki.ros.org/xpp
[ifopt_core]: https://github.com/ethz-adrl/ifopt
[ifopt]: https://github.com/ethz-adrl/ifopt
[Ipopt]: https://projects.coin-or.org/Ipopt
[ncurses]: http://invisible-island.net/ncurses/man/ncurses.3x.html
[xterm]: https://linux.die.net/man/1/xterm
[Snopt]: http://www.sbsi-sol-optimize.com/asp/sol_product_snopt.htm
[rviz]: http://wiki.ros.org/rviz
[catkin]: http://wiki.ros.org/catkin
[catkin tools]: http://catkin-tools.readthedocs.org/
[Eigen]: http://eigen.tuxfamily.org
[this example]: https://github.com/ethz-adrl/ifopt/blob/master/ifopt_core/test/ifopt/test_vars_constr_cost.h
