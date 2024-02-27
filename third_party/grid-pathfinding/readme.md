Grid Pathfinding
===
[![MIT License](http://img.shields.io/badge/license-MIT-blue.svg?style=flat)](LICENSE)
![test](https://github.com/Kei18/grid-pathfinding/workflows/test/badge.svg)

A simple graph library with single-agent pathfinding algorithms, used in my research projects, e.g., [time-independent-planning](https://github.com/Kei18/time-independent-planning), [mapf-IR](https://kei18.github.io/mapf-IR/).
It is written in C++(17) with [CMake](https://cmake.org/) (≥v3.16) build.
For usage, see [examples](https://github.com/Kei18/grid-pathfinding/blob/master/tests/test_graph.cpp).

map format: [Pathfinding Benchmarks](https://movingai.com/benchmarks/)

### Mac M1 CPU Support

To build this library for M1 CPUs, you need to add an argument to cmake as such:

``` bash
cmake .. -DCPU=M1
```

You should see an output similar to 

``` text
Configuring grid-pathfinding for M1 Mac.
-- Configuring done
-- Generating done
-- Build files have been written to:  XXX/XX/XXX
```

## Licence
This software is released under the MIT License, see [LICENSE.txt](LICENCE.txt).

## Author
[Keisuke Okumura](https://kei18.github.io) is a Ph.D. student at the Tokyo Institute of Technology, interested in controlling multiple moving agents.

