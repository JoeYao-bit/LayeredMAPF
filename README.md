
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
This branch is a simplified version of Layered MAPF, which have minimized dependencies.

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

## Python
```
sudo apt install python3-pip
pip install matplotlib
```

# Usage

## LayeredMAPF with single MAPF instance


```
./test_single_layered_MAPF
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
