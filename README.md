# Density-based spatial clustering of applications with noise (DBSCAN)

This is a fast C++ implementation of dbscan clustering algorithm.

Compiling and running the example:

```bash
g++ example.cpp dbscan.cpp -I vendor/ -std=c++20 -o example
./example sample3d.csv 1 10 > output.csv
```

<p align="center">
  <img src="plot3.png" />
</p>