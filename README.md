# tracker_CLASS

  

This is an implementation of an Extended Kalman Filter based tracker.

There is both an implementation in C++ and in Matlab.

The idea was to develop a fast method to track objects from a pole-mounted camera. A visual tracker would have been too slow, therefore an EFK has been adopted. The idea is that after a Convolutional Neural Network (CNN) detects the bounding-boxes of the objects, the central bottom point of the bounding box is taken as a reference of that object and on that the tracker is instantiated.

However, the tracker implementation is general, but takes as input a point for an object for each frame and track those objects with an aging mechanism.

The idea is to use just the position of the object $(x,y)$ to predict not only the new position $(x’,y’)$, but also the velocity $v$ the yaw $\psi$, and the yaw-rate $\dot\psi$. Hence, the state of EKF is: 

  $\left[\begin{array}{c}
x \\
y \\
\psi \\
v \\
\dot\psi \\
\end{array}\right]$ 
  
While the state transition adopted: 

$\left[\begin{array}{c}
x + \frac{v\cdot(-sin(\psi) + sin(T\cdot \dot\psi + \psi))}{\dot\psi} \\
y+ \frac{v\cdot(cos(\psi) - cos(T\cdot \dot\psi + \psi))}{\dot\psi} \\
T\cdot \dot\psi + \psi \\
v \\
\dot\psi \\
\end{array}\right]$ 

## Dependencies 

```
sudo apt-get install libeigen3-dev
sudo apt-get install libflann-dev
sudo apt-get install python3-matplotlib
```

## Run the c++ code
```
git clone https://github.com/mive93/tracker_CLASS
cd tracker_CLASS
cd c++
mkdir build
cd build
cmake ..
make
./tracker
```
  

<!-- Acknowledgements -->

# Acknowledgements
 

This work has been supported by the EU H2020 project CLASS, contract #780622.