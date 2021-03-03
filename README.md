# class-tracker

  

```class-tracker``` is a C++ library that implements both an Extended Kalman Filter (EFK) and an Unscented Kalman Filter (UKF), based tracker.

The tracker only works on the position of the object ```(x,y)``` to predict not only the new position ```(x’,y’)```, but also the velocity ```v``` the yaw ![equation](https://latex.codecogs.com/gif.latex?%5Cpsi), and the yaw-rate ![equation](https://latex.codecogs.com/gif.latex?%5Cdot%5Cpsi). Hence, the state of EKF is: 

![equation](https://latex.codecogs.com/gif.latex?%5Cbegin%7Bbmatrix%7D%20x%20%5C%5C%20y%20%5C%5C%20%5Cpsi%20%5C%5C%20v%20%5C%5C%20%5Cdot%5Cpsi%20%5C%5C%20%5Cend%7Bbmatrix%7D)

While the state transition adopted: 

![equation](https://latex.codecogs.com/gif.latex?%5Cbegin%7Bbmatrix%7D%20x%20&plus;%20%5Cfrac%7Bv%5Ccdot%28-sin%28%5Cpsi%29%20&plus;%20sin%28T%5Ccdot%20%5Cdot%5Cpsi%20&plus;%20%5Cpsi%29%29%7D%7B%5Cdot%5Cpsi%7D%20%5C%5C%20y&plus;%20%5Cfrac%7Bv%5Ccdot%28cos%28%5Cpsi%29%20-%20cos%28T%5Ccdot%20%5Cdot%5Cpsi%20&plus;%20%5Cpsi%29%29%7D%7B%5Cdot%5Cpsi%7D%20%5C%5C%20T%5Ccdot%20%5Cdot%5Cpsi%20&plus;%20%5Cpsi%20%5C%5C%20v%20%5C%5C%20%5Cdot%5Cpsi%20%5C%5C%20%5Cend%7Bbmatrix%7D)


It is important to know that the filter expects to receive data in meters and returns:
 - ```(x',y')``` in meters
 - ```v``` in m/s
 - ![equation](https://latex.codecogs.com/gif.latex?%5Cpsi) in radians
 - ![equation](https://latex.codecogs.com/gif.latex?%5Cdot%5Cpsi) in radians/second


Moreover, it is important to set the correct ```delta t``` and the wanted age factor when using the tracker as a library.

 ## Dependencies 

```
sudo apt-get install libeigen3-dev python3-matplotlib libpython3.6 

```

This library also depends upon: 
- [matplotlib-cpp](https://github.com/lava/matplotlib-cpp) for visualization
- [geotedic_utils](https://github.com/ethz-asl/geodetic_utils) for the convertion from GPS to meters

## Building this repo
```
git clone https://github.com/mive93/tracker_CLASS
cd class-tracker
git submodule update --init --recursive
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j4
```

Optionally
```
cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo .. 
cmake -DCMAKE_BUILD_TYPE=Debug ..
```

## Running the demo

This repository offers a library to exploit the implemented filter, however, there is also a dummy example of the usage of the trackers given by the program ```tracker```. Once the project has been built, it can just be run with:
```

./tracker
```

It exploits the file ```../data/test_ll.txt``` in which in each line there is 

- frame number
- timestamp
- latitude (GPS)
- longitude (GPS)

and it shows how to convert them into meters using the geodetic_converter. 

Once run, it will show the ground-truth (noisy positions) in red and the output of the filter (the prediction of EKF of UKF) in green as in this picture:

![example](img/example.png)



<!-- Acknowledgements -->

# Acknowledgements
 
This work has been supported by the EU H2020 project CLASS, contract #780622.
