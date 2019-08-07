这是我们早就想做的实验, 由于接口藏得有点深, 之前一直没有做了. 现在把详细的实验过程放在这里, 即记录实验结果, 又算作是一个使用towr 系统的案例.

# 实验内容与假设
towr的搜索是一个优化变量的过程, 如果我们提前已知一些变量的解, 直接告诉并固定, 那么应该会增加搜索的速度吧

我们上下层可能的一个好处就是上层可以给出身体的位置, 这相当于一些变量的解. 所以我们要尝试提前设好以及固定身体x,y位置之后的求解会不会变快

所以,实验的步骤如下:
1. 正常传递求解参数进行求解,记录求解变量结果
2. 传入步骤1中的身体x,y 坐标并固定, 再次求解, 比较求解速度

# 根据pytowr功能的分析
pytowr的如下设计可以提供帮助

### py_run
py_run 是进行求解的主要函数, 参数分别是
   *  target pos x
   *  target pos y
   *  output time scale
   *  terrian call back function
   *  Posture: a tuple: (body height, stance pos) None if the norminal_stance(defined in the robot model) is to be used
      * stancePos: the init positure of the robot(init EE positions in global cordinate sys), should be a 6*3 numpy array. 
   *  init value dict {variable_name(pystring) : value(pylist)}

所以使用传入一个init value dict就可以设置初始值

同时, py_run 的返回值里面有一个是最后解到的

### initValues
由于为了方便接口设计,所以设定初始值的时候只能选择14种variable中的几种(比如base-lin base-ang, foot-lin-0),整体设置. 所以如果我们只想把x,y的值设置了,而不像改变一开始的初始z的话, 我们需要获取整个的初始的值, 然后在python层内替换其中的x,y,再将整个作为初始值赋过去. 

所以我写了这个获取初始值函数,这个函数输入和py_run 类似只是没有`init value dict`, 返回一个轨迹变量

### Complie Macros
同样为了接口的整洁, `py_run` 并没有设置控制锁定哪个变量的参数. 所以如果比如我们这次要锁定身体轨迹的x,y. 那么我们需要改变相应位置重现编译. 

为了方便编译的时候控制程序编译成什么样子, 所以使用编译的宏, 比如定义在[pytowr.h](../pytowr.h) 中有:

```c++
#define LOCKDIM // whether to lock the x,y dimention of the hexpods body

#define OPTMIZE_DURATION // whether to optimize the duration(change the phase and gait)
```

`LOCKDIM`就是控制pytowr是否需要固定x,y的宏, 如果我们注释掉那一行, 那么pytowr就不会固定 x,y, 其控制的代码区域在[pytowr.cc](../pytowr.cc)中如下:

```c++
#ifdef LOCKDIM
  std::shared_ptr<ifopt::Component> basecompon = VariablePtr -> GetComponent("base-lin");
  towr::NodesVariablesAll::Ptr BaselinPtr = std::dynamic_pointer_cast<towr::NodesVariables>(basecompon);
  Eigen::VectorXd baseVariables = BaselinPtr->GetValues(); //Need to cast from Component to NodesVariablesAll
  for(auto deriv : {towr::kPos} ) // can have `towr::kVel` if you also want to lock velocity
    for(auto dim : {0,1} ) // Bound the x,y dimension
      BaselinPtr->LockBound(deriv,dim,baseVariables);
#endif
```
这段代码用来把 base-lin 中所有的x,y position锁定.

这些宏只要搜索一下在哪出现了就能轻松明白是干啥的.

## 实验流程
所以根据前面所述pytowr的功能, 做这个实验的流程如下:
1. 编译不锁定xy变量的pytowr
   1. 注释掉 `#define LOCKDIM`这一行
   2. 重新编译pytowr, `python3 setup.py build_ext --inplace`
      1. 如果没有看到一堆编译输入,可能需要手动删除就的版本 `rm pytowr.cpython-3*`
2. 使用`pytowr.run`搜索, 把最后得到的*轨迹变量*保存到pkl中
3. 编译锁定xy变量的pytowr
   1. 取消`#define LOCKDIM`这一行的注释
   2. 和之前一样,重新编译pytowr
4. 使用`pytowr.initValues`获取默认的初始值, 读取pkl中上一次搜索的轨迹变量结果
5. 把我们需要固定的轨迹变量从轨迹变量结果赋值替换初始值
6. 使用`pytowr.run`再次搜索, 对比前后的迭代次数差异

实验相应的python代码在[initValueExperiment](initValueExperiment)中

# 实验结果
使用上0.3台阶的地图, 之前8次, 之后也8次, 没有差异!
