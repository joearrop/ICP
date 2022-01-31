# ICP

Implements Iterative closest point algorithm (ICP) point-to-plane variation

Sources:
 * -https://www.robots.ox.ac.uk/~avsegal/resources/papers/Generalized_ICP.pdf
 * -https://en.wikipedia.org/wiki/Iterative_closest_point

## Prerequisites

* C++17
* [Eigen3](https://eigen.tuxfamily.org/index.php?title=Main_Page)
```bash
sudo apt install libeigen3-dev
```
* [PCL](https://pointclouds.org/)
```bash
sudo apt install libpcl-dev
```
* [OpenMP](https://www.openmp.org/) (Optional)
```bash
sudo apt install libomp-dev
```

## Build

Standard CMake build using CMakeLists.txt

```bash
git clone https://github.com/joearrop/ICP
cd ICP
mkdir build
cd build
cmake ..
make
```