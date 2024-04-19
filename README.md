
# LayeredMAPF

Motivated by the exponential growth in the cost of solving MAPF instances (in terms of time and memory usage) as the number of agents increases, we proposed layered MAPF as a solution to reduce the computational burden. This approach decomposes a MAPF instance into multiple smaller subproblems without compromising solvability. Each subproblem is solved in isolation, with consideration given to other subproblems' solutions as dynamic obstacles. Our methodology involves a progressive decomposition of MAPF instances, ensuring that each step preserves solvability.

[toc]

# Download code
```
git clone git@github.com:JoeYao-bit/LayeredMAPF.git
cd LayeredMAPF
git submodule update --init  --recursive
mkdir build
cd build
cmake ..
make
```


# Install Dependencies
This branch is a full version of Layered MAPF, which have lots of dependencies, for visualization and develop convinience.
<font color=red>If you want a quick start, swtich to branch "minimum_denpendence"  would be a wise choice, follow its README.</font>

```
git checkout minimize_dependence
```
## Git
```
sudo apt-get install libgoogle-glog-dev
sudo apt-get install libgtest-dev
```

## Boost
```
sudo apt-get install libboost-graph-dev
sudo apt-get install libboost-thread-dev
sudo apt-get install libboost-filesystem-dev
sudo apt-get install libboost-system-dev
sudo apt-get install libboost-program-options-dev
```

## Visualization tools

### OpenCV
```
sudo apt-get install libopencv-dev
```

### OpenGL
```
sudo apt-get install libglfw3-dev
```

### Pangolin 
Down load source code from https://github.com/stevenlovegrove/Pangolin
And unzip it.
```
sudo apt-get install libglew-dev
sudo apt-get install libboost-dev libboost-thread-dev libboost-filesystem-dev
cd Pangolin
mkdir build
cd build
cmake ..
make -j2
sudo make install
```

### Qt5
```
sudo apt-get install build-essential
sudo apt-get install qtbase5-dev qtchooser qt5-qmake qtbase5-dev-tools
sudo apt-get install qtcreator
sudo apt-get install qt5*
```

## Calculation tools

### Eigen
```
sudo apt-get install libeigen3-dev

### SuiteSparse
sudo apt-get install libsuitesparse-dev
```

### G2O
```
sudo apt-get install qt5-qmake qt5-default libqglviewer-dev-qt5 libsuitesparse-dev libcxsparse3 libcholmod3
mkdir build
cd build
cmake ..
make
sudo make install
```

### CGAL
```
sudo apt-get install libcgal-dev
```

### Octomap
```
git clone https://github.com/OctoMap/octomap
cd octomap/octomap
mkdir build
cd build
cmake ..
make
sudo make install
```

##  Auxiliary tools

### Gtest
```
sudo apt-get install libgoogle-glog-dev
sudo apt-get install libgtest-dev
```

### Glog
```
git clone https://github.com/google/glog
cd glog
mkdir build && cd build
cmake -DGFLAGS_NAMESPACE=google -DCMAKE_CXX_FLAGS=-fPIC -DBUILD_SHARED_LIBS=ON ..
make
sudo make install
```

### Python
```
sudo apt install python3-pip
pip install matplotlib
```

### yaml-cpp
```
tar zxvf yaml-cpp-0.5.1.tar.gz
mkdir build
cd build
cmake -DBUILD_SHARED_LIBS=ON ..
make
sudo make install
sudo ldconfig
```

### libxml2
```
sudo apt-get install libxml2-dev
```

### argparse
```
https://github.com/jarro2783/cxxopts
```

# Usage

## LayeredMAPF with single MAPF instance
```
OpenCV viewer: ./test_2d_mapf_viewer
Qt: ./test_2d_mapf_viewer
```

## Massive Decomposition of MAPF instance test
```
./test_massive_decomposition
```

## Massive Layered MAPF test
```
./test_massive_layered_mapf
```


# License
LayeredMAPF is released under MIT License. See LICENSE for further details.
Third party codes remain their own licenses.
