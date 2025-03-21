
# LayeredMAPF

Motivated by the exponential growth in the cost of solving MAPF instances (in terms of time and memory usage) as the number of agents increases, we proposed layered MAPF as a solution to reduce the computational burden. This approach decomposes a MAPF instance into multiple smaller subproblems meanwhile minimizing loss of solvability. Each subproblem is solved in isolation, with consideration given to other subproblems' solutions as dynamic obstacles. Our methodology involves a progressive decomposition of MAPF instances, ensuring that each step minimizing loss of solvability.

[toc]

# Download code
```
git clone git@github.com:JoeYao-bit/LayeredMAPF.git
cd LayeredMAPF
git checkout minimize_dependence
git submodule update --init  --recursive
mkdir build
cd build
cmake ..
make
```


# Install Dependencies
This branch is a simplified version of Layered MAPF, which have minimized dependencies.
A full version is branch "main", including visualization, but have lots of dependencies.

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

# Eigen
sudo apt-get install libeigen3-dev

## Gtest

```
sudo apt-get install libgoogle-glog-dev
sudo apt-get install libgtest-dev
```

## Glog
```
git clone https://github.com/google/glog
cd glog
mkdir build && cd build
cmake -DGFLAGS_NAMESPACE=google -DCMAKE_CXX_FLAGS=-fPIC -DBUILD_SHARED_LIBS=ON ..
make
sudo make install
```

## argparse

```
# Clone the repository
git clone https://github.com/p-ranav/argparse
cd argparse

# Build the tests
mkdir build
cd build
cmake -DARGPARSE_BUILD_SAMPLES=on -DARGPARSE_BUILD_TESTS=on ..
make

# Run tests
./test/tests

# Install the library
sudo make install
```
## KaHyPar

Clone the repository including submodules:

git clone --depth=1 --recursive git@github.com:SebastianSchlag/kahypar.git

Create a build directory: mkdir build && cd build

Run cmake: cmake .. -DCMAKE_BUILD_TYPE=RELEASE

Run make: make


## Python (Optional, if you want to run Python scripts)
```
sudo apt install python3-pip
pip install matplotlib
```

# Usage

## LayeredMAPF with single MAPF instance


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
