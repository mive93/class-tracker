# tracker_CLASS

  

This is an implementation of an Extended Kalman Filter based tracker.

There is both an implementation in C++ and in Matlab.

The idea was to develop a fast method to track objects from a pole-mounted camera. A visual tracker would have been too slow, therefore an EFK has been adopted. The idea is that after a Convolutional Neural Network (CNN) detects the bounding-boxes of the objects, the central bottom point of the bounding box is taken as a reference of that object and on that the tracker is instantiated.

However, the tracker implementation is general, but takes as input a point for an object for each frame and track those objects with an aging mechanism.

The idea is to use just the position of the object (x,y) to predict not only the new position (x’,y’), but also the velocity v the yaw ![equation](https://latex.codecogs.com/gif.latex?%5Cpsi), and the yaw-rate ![equation](https://latex.codecogs.com/gif.latex?%5Cdot%5Cpsi). Hence, the state of EKF is: 


![equation](https://latex.codecogs.com/gif.latex?%5Cbegin%7Bbmatrix%7D%20x%20%5C%5C%20y%20%5C%5C%20%5Cpsi%20%5C%5C%20v%20%5C%5C%20%5Cdot%5Cpsi%20%5C%5C%20%5Cend%7Bbmatrix%7D)

While the state transition adopted: 

![equation](https://latex.codecogs.com/gif.latex?%5Cbegin%7Bbmatrix%7D%20x%20&plus;%20%5Cfrac%7Bv%5Ccdot%28-sin%28%5Cpsi%29%20&plus;%20sin%28T%5Ccdot%20%5Cdot%5Cpsi%20&plus;%20%5Cpsi%29%29%7D%7B%5Cdot%5Cpsi%7D%20%5C%5C%20y&plus;%20%5Cfrac%7Bv%5Ccdot%28cos%28%5Cpsi%29%20-%20cos%28T%5Ccdot%20%5Cdot%5Cpsi%20&plus;%20%5Cpsi%29%29%7D%7B%5Cdot%5Cpsi%7D%20%5C%5C%20T%5Ccdot%20%5Cdot%5Cpsi%20&plus;%20%5Cpsi%20%5C%5C%20v%20%5C%5C%20%5Cdot%5Cpsi%20%5C%5C%20%5Cend%7Bbmatrix%7D)

## Dependencies 

```
sudo apt-get install libeigen3-dev libflann-dev python3-matplotlib python-dev python-dev libflann-dev

```

## Run the c++ code
```
git clone https://github.com/mive93/tracker_CLASS
cd tracker_CLASS/c++
mkdir build
cd build
```
build in Release or Debug with one the following commands

```
cmake -DCMAKE_BUILD_TYPE=Release .. 
cmake -DCMAKE_BUILD_TYPE=Debug .. 
```

and finally
```
make
./tracker
```
  

<!-- Acknowledgements -->

# Acknowledgements
 

This work has been supported by the EU H2020 project CLASS, contract #780622.
