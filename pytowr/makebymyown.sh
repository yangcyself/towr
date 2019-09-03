
CXX_FLAGS = -g   -std=gnu++11

CXX_DEFINES = 

CXX_INCLUDES = -I/home/dada/catkin_ws/src/towr/towr/include -isystem /usr/local/include -isystem /usr/local/include/eigen3 

/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o test.out  /home/dada/catkin_ws/src/towr/towr/test/hopper_example.cc \
src/*

/usr/bin/c++  -I/home/dada/catkin_ws/src/towr/towr/include -isystem /usr/local/include -isystem /usr/local/include/eigen3 \
  -g   -std=gnu++11 -o test.out  /home/dada/catkin_ws/src/towr/towr/test/hopper_example.cc \
src/* -rdynamic libtowr.so /usr/local/lib/libifopt_ipopt.so /usr/local/lib/libifopt_core.so -Wl,-rpath,/home/dada/catkin_ws/src/towr/towr/build:/usr/local/lib