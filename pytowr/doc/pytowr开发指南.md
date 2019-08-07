# pytowr修改指南
这个文档介绍pytowr的结构, 变量的逻辑关系, 常用的常量设置, 以及我目前想到的问题
## pytowr系统结构
这里的结构按照编译来划分, 整个的结构实际分为如下三层, 他们之间都是独立编译的.
1. python
2. pytowr
3. towr

python 不必多说, 随改随用的脚本语言, 不用编译

### pytowr
pytowr作为python的module和python交互. pytowr的本体是 `pytowr.cpython-*.so`, 也就是如果你想更改属于pytowr的内容, 必须重新编译pytowr, 如果发现改了没用的情况首先检查一下 python 引用的 `pytowr.cpython-*.so` 是不是更新了. 

如果想看你的python调用的是哪一个文件以确保其加载的是最新的文件,可以:
```py
import pytowr
print(pytowr.__file__)
```

pytowr和下面的towr的交互也是通过动态链接的方式, 凡是使用动态链接, 都要确保动态链接库在你链接的路径里面. 如果`libtowr.so` 不在`/lib` 或 `/usr/lib`, 那么请设置 `LD_LIBRARY_PATH`

### towr
这个工程里面towr的主体仍然是人家的, 我仅仅是添加了六足机器人的定义, 一个新的并联constraint, 以及一个用来锁住一些变量的函数.

towr的本体是 `libtowr.so`, 如果改动了任何towr文件夹下面的, 请一定要重新编译towr, 否则没有效果.

## 变量逻辑关系
towr求解的是轨迹, 一个轨迹可以用不同的表示方法在空间中确定. 我觉得一个比较容易混淆的就是真正放在towr里面去求解的变量(咱们称之**轨迹变量**),以及把轨迹按照一段时间间隔取点,形成的一个类似spline一样的东西(咱们称之**轨迹折线**). 

这二者描述的是一样的东西, 但是请注意区分. 比如`pytowr.run`这个函数的返回值分别是: 轨迹折线, 迭代次数, 轨迹变量. 

想象轨迹是空间中的一个三次函数, 轨迹折线相当于是$f(t_0),f(t_1)....f(t_n)$, 我们引导机器人的动作就是使用的轨迹折线

轨迹变量相当于函数的参数$a,b,c,d$, 我们用来设初始值简化计算的就是设置这个东西


## towr内部机器人设置

### 我迄今为止在towr内部的修改
第三次修改添加的地方(方便pytowr)
1. `nodes_variables.h` & `nodes_variables.cc`
   1. 增加lockbound 函数

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

### 可能需要的参数修改位置
#### 机器人模型:
见[hexpod_model.h](../../towr/include/towr/models/examples/hexpod_model.h)
其中: 
- `nominal_stance_` 是机器人正常站立时的姿势, 用于作为搜索时的初始位姿, 也为了确定range of motion的box 位置
- `root_positions` 是机器人腿根的位置, 用于计算elongation
- `min_length`, `max_length` 是机器人的elongation 的范围 **这个需要更加精确的范围**
- `max_dev_from_nominal_` 是使用range of motion constraint的时候距离 `nominal_stance_` 的最大范围
- `SingleRigidBodyDynamics` 传入的参数分别是 (double mass,
                                  double Ixx, double Iyy, double Izz,
                                  double Ixy, double Ixz, double Iyz,
                                  int ee_count)

目前我并不知道这些Ixx等等东西什么意思, 但是反正我们已经把dynamic constraint去掉了,这个重要嘛?

#### 搜索参数
见[parameter.cc](../../towr/src/parameters.cc)

里面默认设置了很多参数,如下:
```c++
// constructs optimization variables
  duration_base_polynomial_ = 0.1;
  force_polynomials_per_stance_phase_ = 3;
  ee_polynomials_per_swing_phase_ = 2; // so step can at least lift leg

  // parameters related to specific constraints (only used when it is added as well)
  force_limit_in_normal_direction_ = 1000;
  dt_constraint_range_of_motion_ = 0.08;
  dt_constraint_dynamic_ = 0.1;
  dt_constraint_base_motion_ = duration_base_polynomial_/4.; // only for base RoM constraint
  bound_phase_duration_ = std::make_pair(0.2, 1.0);  // used only when optimizing phase durations, so gait

```
这些我没有全都改过, 但是后面可能有用.

**dt** 极其有用!
比如某一个试验中, 中间有一个柱子, 但是搜到的结果机器人直接就过去了, 是因为那个柱子太小了，而constraint的dt太大了，直接给忽略过去了

dt减小可能会导致搜索轮数增加, 同时每轮搜索时间也会增加, 如果减小了dt发现找不到解, 请关注一下此时CPU时间足够搜索多少轮, 防止是因为CPU时间不足搜索那么多轮导致的

#### 步态
步态除了决定几个脚一起走之外, 还决定了步数, 所以要改长距离的移动一定要改变步态 (或者如果为了方便的话可以换一种方式), 但是目前, 步态的步数是在[hexaped_gait_generator.cc](../../towr/src/hexaped_gait_generator.cc)里面设置好的

比如这里只设置了一种步态
```c++
void
HexapedGaitGenerator::SetCombo (Combos combo)
{
  switch (combo) {
    case C0: SetGaits({Stand, Walk1, Walk1, Walk1, Walk1, Stand}); break;
    default: assert(false); std::cout << "Gait not defined\n"; break;
  }
}

```
在`pytowr.cc`里面如下调用
```c++
auto gait_gen_ = GaitGenerator::MakeGaitGenerator(n_ee);
  gait_gen_->SetCombo(towr::GaitGenerator::C0);
```


# 目前仍然存在的问题

## 搜索结果往柱子里面搜索
见如下代码设置
```py
def terrain(x,y):
    if(0.8<x<1.2 and -0.2<y<0.2):
    # if(0.7<x<1.3 and -0.3<y<0.3):
        # print("#####CALLEDME#######")
        return 10
    else:
        return 0
pos,cost,varDict = pytowr.run(2.,0.,0.01, terrain,None,{}) # target x, target y, time scale of the return list
```
是一个中间有一个大柱子的地图, 按理来说机器人应该选择绕过去,但是知道目前的版本, 如果柱子比较大(下面那行`if(0.7<x<1.3 and -0.3<y<0.3):`)机器人会找不到解, 如果柱子比较小(`if(0.8<x<1.2 and -0.2<y<0.2):`),机器人会找到一个错误的解,(哪怕dt调得很小)

不知道设置一个初始轨迹让它绕过去会不会就能解了,这个不如就作为你的入手任务

# 最后, 加油!
进一步的实验上手, 可以参考[锁定XY实验过程与结果.md](锁定XY实验过程与结果.md), 里面详细的分析以及实验过程

如果有任何问题, 请在github上面提issue

如果有好的代码改动, 请在github上面提pull request