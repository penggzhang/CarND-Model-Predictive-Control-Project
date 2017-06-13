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
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Fortran Compiler
  * Mac: `brew install gcc` (might not be required)
  * Linux: `sudo apt-get install gfortran`. Additionall you have also have to install gcc and g++, `sudo apt-get install gcc g++`. Look in [this Dockerfile](https://github.com/udacity/CarND-MPC-Quizzes/blob/master/Dockerfile) for more info.
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
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.


## Brief about Implementation

#### The Model

This Model Predictive Controller project was built to find a sequence of optimal actuations, which shall make the vehicle drive through the simulator track gracefully. As an MPC, the task is modeled as an optimization problem. Inputs are vehicle states and actuators along the time. Predictions as well as constraints are made based on vehicle dynamics. The objective is to minimize the cost related to the predictive states in reference to desired trajectory.

More in details, vehicle state include:

* Position x coordinate
* Position y coordinate
* Orientation
* Velocity
* Cross track error in reference to the desired trajectory
* Orientation error in reference to the desired trajectory

Actuators include:

* Steering
* Acceleration

Cost, the objective function, incorporate the following factors:

* Cross track error and orientation error in reference to desired trajectory, and the difference between the predicted velocity and the desired velocity
* Magnitude of actuators, i.e. steering and acceleration
* Gaps between sequential actuators

Constraint equations and value limits:

* Given current position, velocity and orientation, vehicle position at next time will be predicted.
* Given current orientation, velocity, vehicle length and the steering, vehicle orientation at next time will be predicted.
* Given current velocity and the acceleration, vehicle velocity at next time will be predicted.
* The reference trajectory can be fitted from a bunch of given way points. Along with current vehicle position and orientation, current cross track error and orientation error will be known. Then, cross track error and orientation error at next time will be derived.
* Each state or actuator variable has a pair of upper and lower value limits.

#### Timestep Length and Elapsed Duration (N & dt)

The product of N and dt is the horizon, T, over which prediction is made. While the guideline for choosing T value is as large as possible, it cannot be too large because the environment will change enough and it doesn't make sense to predict beyond the horizon. So, for this case of vehicle simulator, T of 1 second was chosen.

To more accurately approximate a continuous trajectory, dt shall be as small as possible. Here, 0.05, 0.1, 0.2 second for dt and corresponding values for N (= T / dt) are tried. Eventually, dt of 0.1 second and N of 10 were chosen.

#### Polynomial Fitting and MPC Preprocessing

For the ease of calculating cross track error and orientation error, waypoints from simulator which are in map coordinates were transformed to vehicle coordinates. Then a polynomial of reference trajectory was fitted to the transformed waypoints. Vehicle's initial state variables, including position px, py and orientation psi, were also transformed to zeros, since at the time the vehicle located on the origin and oriented along with x axis in vehicle coordinates. And then initial cross track error and orientation error were thus derived related to the polynomial within vehicle cooridnates. The magnitude of velocity stayed the same since it isn't affected by coordinate transformation. All the above variables compose the input to MPC. 

#### Model Predictive Control with Latency

Consider latency as a duration over which state prediction needs to be made. And then given such predicted state, MPC runs for the optimized control inputs over future horizon. So, state variables, including position, orientation and velocity after latency were estimated and then used to fit polynomial, transform initial state and run MPC. 

## Simulation

A [video](https://youtu.be/beDDjbLzbLM) was recorded for review.