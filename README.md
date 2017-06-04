# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets) == 0.14, but the master branch will probably work just fine
  * Follow the instructions in the [uWebSockets README](https://github.com/uWebSockets/uWebSockets/blob/master/README.md) to get setup for your platform. You can download the zip of the appropriate version from the [releases page](https://github.com/uWebSockets/uWebSockets/releases). Here's a link to the [v0.14 zip](https://github.com/uWebSockets/uWebSockets/archive/v0.14.0.zip).
  * If you have MacOS and have [Homebrew](https://brew.sh/) installed you can just run the ./install-mac.sh script to install this.
* [Ipopt](https://projects.coin-or.org/Ipopt)
  * Mac: `brew install ipopt`
  * Linux
    * You will need a version of Ipopt 3.12.1 or higher. The version available through `apt-get` is 3.11.x. If you can get that version to work great but if not there's a script `install_ipopt.sh` that will install Ipopt. You just need to download the source from the Ipopt [releases page](https://www.coin-or.org/download/source/Ipopt/) or the [Github releases](https://github.com/coin-or/Ipopt/releases) page.
    * Then call `install_ipopt.sh` with the source directory as the first argument, ex: `bash install_ipopt.sh Ipopt-3.12.1`.
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [CppAD](https://www.coin-or.org/CppAD/)
  * Mac: `brew install cppad`
  * Linux `sudo apt-get install cppad` or equivalent.
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/CarND-MPC-Project/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions


1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

## Tips

1. It's recommended to test the MPC on basic examples to see if your implementation behaves as desired. One possible example
is the vehicle starting offset of a straight line (reference). If the MPC implementation is correct, after some number of timesteps
(not too many) it should find and track the reference line.
2. The `lake_track_waypoints.csv` file has the waypoints of the lake track. You could use this to fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.

## Details

---
### The Model

**state**:

In this project, I used the Kinematic Model, so the state includes `x`, `y`, `psi`, `v`, `cte`, `epsi`, which are implemented at line 126-127 in main.cpp.

**actuators**:

the steering wheel `delta` and acceleration `a`

**update equations**:

``` Python
next_x = last_x + last_v * cos(last_psi) * dt;
next_y = last_x + last_v * sin(last_psi) * dt;
next_psi = last_psi + last_v / Lf * a * dt;
next_v = last_v + a * dt;
next_cte = poly(last_x) - last_y + last_v * sin(last_epsi) * dt;
next_epsi = last_psi - last_des_psi + last_v / Lf * a * dt;
```

### Timestep Length and Elapsed Duration (N & dt)

`N` is the number of timesteps in the horizon. `dt` is how much time elapses between actuations. When the speed of the car is high, the length of every timestep will increase. So we should reduce timestep length `dt` in this condition.

At first, I set the speed below 40, and I found that `N=10, dt=0.25` can keep the car driving on the road. Then I change the speed to 60, it turned out that `N=10, dt=25` can't word well. Then I modified `N` and `dt` until the car can keep driving on the road. Finally, I decided `N=16` and `dt=0.15`.

### Polynomial Fitting and MPC Preprocessing

Before fitting polynomials, the waypoints are transformed from map system into vehicle system. These codes are declared at line 108-119 in main.cpp. Then I used these waypoints to fit polynomials, the code is located in line 121.

In this map, I use quadratic polynomial to fit the route of car. It can fit waypoints well in this project.

### Model Predictive Control with Latency

To deal with latency, I have tried a method which is preprocessing the state before fitting polynomials: Using the current state to update itself with `timestep=0.1`, the code is at line 102-106 in main.cpp. But it will lead the car tumbling around and touching the road edges. So finally, I decided to remove the code.

In this project, it still work well without any problem when I just ignored the latency. Maybe the influence of 100 millisecond latency is not enough to change the state of the car too much with low speed. So finally, I didn't deal with latency in final model.
