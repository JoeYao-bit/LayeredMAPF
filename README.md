
# LayeredMAPF

Motivated by the exponential growth in the cost of solving MAPF instances (in terms of time and memory usage) as the number of agents increases, we proposed layered MAPF as a solution to reduce the computational burden. This approach decomposes a MAPF instance into multiple smaller subproblems without compromising solvability. Each subproblem is solved in isolation, with consideration given to other subproblems' solutions as dynamic obstacles. Our methodology involves a progressive decomposition of MAPF instances, ensuring that each step preserves solvability.

[toc]

# Download code
```
git clone git@github.com:JoeYao-bit/LayeredMAPF.git
cd LayeredMAPF
<<<<<<< HEAD
git checkout minimize_dependence
=======
>>>>>>> main
git submodule update --init  --recursive
mkdir build
cd build
cmake ..
make
```


# Install Dependencies
<<<<<<< HEAD
This branch is a simplified version of Layered MAPF, which have minimized dependencies.
A full version is branch "main", including visualization, but have lots of dependencies.

=======
This branch is a full version of Layered MAPF, which have lots of dependencies, for visualization and develop convinience.
<font color=red>If you want a quick start, swtich to branch "minimum_dependence"  would be a wise choice, follow its README.</font>

```
git checkout minimize_dependence
```
>>>>>>> main
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

<<<<<<< HEAD
# Eigen
sudo apt-get install libeigen3-dev

## Gtest
=======
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
>>>>>>> main
```
sudo apt-get install libgoogle-glog-dev
sudo apt-get install libgtest-dev
```

<<<<<<< HEAD
## Glog
=======
### Glog
>>>>>>> main
```
git clone https://github.com/google/glog
cd glog
mkdir build && cd build
cmake -DGFLAGS_NAMESPACE=google -DCMAKE_CXX_FLAGS=-fPIC -DBUILD_SHARED_LIBS=ON ..
make
sudo make install
```

<<<<<<< HEAD
## Python (Optional, if you want to run Python scripts)
=======
### Python
>>>>>>> main
```
sudo apt install python3-pip
pip install matplotlib
```

<<<<<<< HEAD
=======
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

>>>>>>> main
# Usage

## LayeredMAPF with single MAPF instance

<<<<<<< HEAD

```
./test_single_layered_MAPF
```
Expected output: \
map info: \
type octile \
height 194 \
width 194 \
map \
 map name ../test/test_data/lak303d.map \
get 330 instances \
-- instance_decomposition_time_cost_ (ms) = 50.831 \
-- cluster_decomposition_time_cost_  (ms) = 0.048 \
-- sort_level_time_cost_             (ms) = 0.019 \
-- Decomposition completeness ? 1 \
 max/total size 1 / 330 \
-- decomposition take 66.412 ms to get 330 clusters  \
 layered mapf success 1 \
330 agents  \
-- layered EECBS end in 2341.38ms \
 is solution valid ? 1 \
layered EECBS maximal usage = 15.1875 MB \
layered EECBS total cost          = 73383 \
layered EECBS maximum_single_cost = 469 \
330 paths  / agents 330 \
-- EECBS end in 5608.04ms \
 is solution valid ? 1 \
--variation of memory 142.352 MB \
--EECBS maximal usage = 146.344 MB \
EECBS total cost          = 64594 \
EECBS maximum_single_cost = 478 
```
=======
OpenCV viewer:
```
./test_2d_mapf_viewer
```
Qt:
```
 ./test_2d_mapf_viewer
```

>>>>>>> main
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
