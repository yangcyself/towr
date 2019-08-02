# pytowr简介
pytowr是[towr](https://github.com/ethz-adrl/towr)的一个python wrapper. towr是一个使用优化方法计算机器人轨迹的求解器, 用三维空间中的曲线描述机器人身体以及足尖的轨迹, 求出满足约束条件的解. 

pytowr 使用python C extension 的方式让python可以调用towr的求解. pytowr是一个module, 里面提供了一些函数的定义, 这些函数会在后端调用towr.

pytowr 目前适用于六足机器人,在原来的towr基础上加入了六足机器人的模型, 以及基于并联结构的kinematic约束.
# 安装与使用
## 安装
安装的过程是编译pytowr的源文件,生成一个python的extension动态链接库. 由于我还不特别会用Cmake, 所以后面需要手动设置依赖的路径
### 编译
#### 编译towr, 生成 libtowr.so
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

#### 设置 setup.py
`setup.py` 里面的`include_dirs`, `library_dirs` 我都直接写的绝对路径, 但是这个应该是根据环境而改变的.

* `include_dirs` 路径设置
  
可以参考前面编译towr时的CMAKE临时文件, 比如打开 `flags.make` (通常是`towr/build/CMakeFiles/towr-example.dir/flags.make`,找不到就搜索这个文件) 可以看到一个 `CXX_INCLUDES` 变量, 这个里面所有 `-I`的路径都是 都是我们需要添加到 setup.py 里面的路径

比如我的是:
`CXX_INCLUDES = -I/home/yangcy/programs/towr/towr/include -isystem /usr/local/include -isystem /usr/local/include/eigen3 `

我添加到`setup.py`里面的路径就都是
```py
include_dirs = ['/home/yangcy/programs/towr/towr/include','/usr/local/include', '/usr/local/include/eigen3'],
```

* `library_dirs`路径设置

同样参考 CMAKE 临时文件, 打开 `link.txt`(通常是`towr/build/CMakeFiles/towr-example.dir/link.txt`).
 在那条编译语句中找到所有`.so`的路径, 把路径添加到 `library_dirs` 中. 比如我的是:
 `/usr/bin/c++   -g   CMakeFiles/towr-example.dir/test/hopper_example.cc.o  -o towr-example -rdynamic libtowr.so /usr/local/lib/libifopt_ipopt.so /usr/local/lib/libifopt_core.so -Wl,-rpath,/home/yangcy/programs/towr/towr/build:/usr/local/lib: `

 我就在 `setup.py` 里面写
 ```py
 library_dirs = ['/home/yangcy/programs/towr/towr/build'],,'/usr/local/lib'],
 ```

#### 编译pytowr
在pytowr文件夹里面运行
```bash
#bash pytowr/
python3 setup.py build_ext --inplace
```
生成一个 `pytowr.cpython-*-gnu.so`

其中inplace这个选项是会把生成的链接库放到当前目录, 所以只有它在python path中的时候才能`import pytowr`. 如果不怕给系统里面的python lib 增添一些垃圾的话可以试着去掉 `--inplace`


### 运行时设置路径
由于使用了动态链接库, 库的位置还可能不在系统默认的路径中,所以在每次运行前需要设置检查`LD_LIBRARY_PATH` 以及 pathon path 两个变量.

#### 设置 LD_LIBRARY_PATH
这一步是为了确保系统能够找的到`libtowr.so`, 如果你的 `libtowr.so`路径不在 `/lib` 或者 `/usr/lib` 的话, 你需要手动将路径添加到 `LD_LIBRARY_PATH` 里面

注意一个 terminal 有自己的一套环境变量, 所以在这个terminal 设置了环境变量不代表另一个或者下次再开的terminal 还有这个变量

如果图省事不想每次都添加的话可以加到`~/.bashrc`里面
```bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:<path-to-libtowr.so> 
```

#### 设置python path
由于前面编译pytowr的时候加了`--inplace` 所以需要把 `pytowr.cpython-*-gnu.so`所在的位置加到 python path里面

添加python path的方法是
```py
import sys
sys.path.append("<path-to-pytowr.cpython-*-gnu.so>")
import pytowr
```

### trouble shooting

#### 编译时出错
检查`include_dirs`

#### 运行时出错, 找不到module
检查python path

#### 运行时出错, 找不到动态链接库
检查`LD_LIBRARY_PATH`

## 使用
只用一个函数 `pytowr.run`

输入: 
1. double x
2. double y
3. double time scale(输出的时间间隔)
4. function(double,double) ->  double 地图的call back函数

输出:
一个pos 和 cost

pos 是一个list list的每一个元素是一个时间点的信息, 这个tuple包括 (time, body_lin, body_ang, (foot_pos0,...,foot_pos5) )

实例如下
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


# 算法介绍
这个算法机器人轨迹搜索的问题转换成一个非线性优化的问题, 所以从 **模型抽象**, **函数表示**, 和 **约束条件**讲解

## 模型抽象
算法中, 用一个身体和几个足尖表示机器人. 一个身体被视作是一个刚体, 有质量以及转动惯量, 有位置以及角度. 足尖点都是没有质量的点, 只有位置, 以及和地面接触的力

所以机器人的模型变成了一个刚体质心和几个足尖点的关系. 足尖点用与地面接触的力改变刚体质心的速度以及角速度, 驱动机器人.

## 函数表示
有了抽象的机器人模型, 还要考虑如何用参数描述我们要搜索的轨迹. 如前文所述, 在towr中需要描述的函数有两种,身体的位置/角度, 以及脚的位置,力.其中身体的位置/角度使用四次多项式来表示,脚的位置和力使用三次多项式来表示

如何表示一个三次多项式? 文中采用的 "hermite" 方法, 使用多项式的两个点的位置以及他们的导数表示一个三次多项式

## 约束条件
- 初始终止位置
- 动力学模型, 也就是前面代表身体的刚体的运动需要是角产生的合力造成的
- 运动学模型, 每一个脚的位置需要在相对身体的一定范围内(文中使用bounding box来简化计算)
- **phase**限制
  - 文中引入了phase的概念, 对于每一个脚, 有'触地phase' 和'腾空phase' 交替进行, 在'触地phase'中 这个脚需要固定在那个位置不动,但是可以提供力, 在'腾空phase'中,这个不能提供力, 但是位置可以移动
- **friction cone** 由于文中假设接触在地上的脚牢牢地踩在地上不移动, 这样要求脚和地之间的力的方向是在 friction cone 里面的
- **terrain height** 踩在地上的脚的高度应该是在地上的, 空中的脚的高度不应该小于地面高度
- 总共时长
  