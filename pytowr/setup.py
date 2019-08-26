from distutils.core import setup, Extension
import os
# import distutils
# distutils.dir_util.remove_tree()

os.environ["CC"] = "g++"
setup(name='sample',
      ext_modules=[
        Extension('pytowr',
                  ['pytowr.cc'],
                  include_dirs = ['/home/dada/catkin_ws/src/towr/towr/include','/usr/local/include', '/usr/include/eigen3'],
                #   define_macros = [('FOO','1')],
                #   undef_macros = ['BAR'],
                  library_dirs = ['/home/dada/catkin_ws/src/towr/towr/build','/usr/local/lib'],
                  libraries = ['ifopt_ipopt' ,'ifopt_core',"towr"],
                  extra_link_args = ['-rdynamic'],
                  extra_compile_args = ["-std=c++11"],
                  )
        ]
)  

#g++ -Wsign-compare -DNDEBUG -g -fwrapv -O3 -Wall -Wstrict-prototypes -fPIC -o build/temp.linux-x86_64-3.6/pytowr.o -std=gnu++11 -I../towr/include -I/usr/local/include -I/usr/local/include/eigen3 -I/home/yangcy/miniconda2/envs/cs231n/include/python3.6m -c pytowr.cc
#g++ -o /home/yangcy/programs/towr/pytowr/pytowr.cpython-36m-x86_64-linux-gnu.so -pthread -shared -B /home/yangcy/miniconda2/envs/cs231n/compiler_compat -L/home/yangcy/miniconda2/envs/cs231n/lib -Wl,-rpath=/home/yangcy/miniconda2/envs/cs231n/lib -Wl,--no-as-needed -Wl,--sysroot=/build/temp.linux-x86_64-3.6/pytowr.o -L/usr/local/lib -L../towr/build -lifopt_ipopt -lifopt_core -ltowr 

#g++ -o /home/yangcy/programs/towr/pytowr/pytowr.cpython-36m-x86_64-linux-gnu.so -pthread -shared -B /home/yangcy/miniconda2/envs/cs231n/compiler_compat -L/home/yangcy/miniconda2/envs/cs231n/lib -Wl,-rpath=/home/yangcy/miniconda2/envs/cs231n/lib -Wl,--no-as-needed  build/temp.linux-x86_64-3.6/pytowr.o -L/usr/local/lib -L/home/yangcy/programs/towr/towr/build/ -lifopt_ipopt -lifopt_core -ltowr  -rdynamic
