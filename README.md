# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

Project Description

### Model predictive control reframes the task of following a trajectory as an optimization problem. The solution to the optimization problem is the optimal trajectory. This involves simulating the different actuator inputs, predicting the resulting rajectory and selecting that trajectory with minimum cost.

Model

### MPC uses an optimizer to find the control inputs and minimize the cost function. Only the very first set of control inputs are executed. This brings the vehicle to a new state and the process is repeated. 

### Following are the steps defined as part of setting up the model
* Define the length of the trajectory, N, and duration of each timestep, dt. Clearly, the number of variables optimized is directly proportional to N, so this must be considered in case there are computational constraints

* Define the vehicle model. Vehicle mode is a set of equations that describe the system behavior and updates across steps. In our case it's a simplified kinematic model described by a state of six parameters.
  * x car position (x-axis)
  * y car position (y-axis)
  * psi car's heading direction
  * v car's velocity
  * cte cross-track error
  * epsi orientation error
  
  Vehicle model update equations are defined as follows:
    * xt+1 = xt + vt ∗ cos( ψt ) ∗ dt
    * yt+1 = yt + vt ∗ sin( ψt ) ∗ dt
    * ψt+1 = ψt + vt / Lf ∗ δt ∗ dt
    * vt+1 = vt + at ∗ dt
    * ctet+1 = f(xt) − yt + ( vt ∗ sin( eψt ∗ dt )
    * eψt+1 = ψt − ψdest + ( vt / Lf ∗ δt ∗ dt )

* Contraints necessary to model contrants in actuators' respose. For instance, a vehicle will never be able to steer 90 deegrees in a single time step. In this project we set these constraints as follows:

steering: bounded in range [-25°, 25°]
acceleration: bounded in range [-1, 1] from full brake to full throttle

* Cost Function
  * Usually cost function is made of the sum of different terms. Besides the main terms that depends on reference values (e.g. cross-track or heading error), other regularization terms are present to enforce the smoothness in the controller response (e.g. avoid abrupt steering).
  The cost function is implemented at lines 54-79 at [MPC.cpp](https://github.com/asheeshv/CarND-MPC-Project/blob/master/src/MPC.cpp)

  ### Tuning Trajectory Parameters
  
  Both ***N*** and ***dt*** are fundamental parameters in the optimization process. In particular, ***T = N * dt*** constitutes the *prediction horizon* considered during optimization. These values have to be tuned keeping in mind a couple of things:
  - large *dt* result in less frequent actuations, which in turn could result in the difficulty in following a continuous reference trajectory (so called *discretization error*) 
  - despite the fact that having a large *T* could benefit the control process, consider that predicting too far in the future does not make sense in real-world scenarios.
  - large *T* and small *dt* lead to large *N*. As mentioned above, the number of variables optimized is directly proportional to *N*, so will lead to an higher computational cost.

In the current project I empirically set (by visually inspecting the vehicle's behavior in the simulator) these parameters to be ***N=10*** and ***dt=0.1***, for a total of ***T=1s*** in the future. 

### Changing Reference System

Simulator provides coordinates in global reference system. In order to ease later computation, these are converted into car's own reference system at lines 98-105 [main.cpp](https://github.com/asheeshv/CarND-MPC-Project/blob/master/src/main.cpp)


### Dealing with Latency

To mimic real driving conditions where the car does actuate the commands instantly, a *100ms* latency delay has been introduced before sending the data message to the simulator (line 185 in [main.cpp](https://github.com/asheeshv/CarND-MPC-Project/blob/master/src/main.cpp)). In order to deal with latency, state is predicted one time step ahead before feeding it to the solver (lines 126-131 in [main.cpp](https://github.com/asheeshv/CarND-MPC-Project/blob/master/src/main.cpp)).

### Simulation Video
The simulation video is available [here](https://github.com/asheeshv/CarND-MPC-Project/blob/master/MPC-Video.mp4)



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
       +  Some Mac users have experienced the following error:
       ```
       Listening to port 4567
       Connected!!!
       mpc(4561,0x7ffff1eed3c0) malloc: *** error for object 0x7f911e007600: incorrect checksum for freed object
       - object was probably modified after being freed.
       *** set a breakpoint in malloc_error_break to debug
       ```
       This error has been resolved by updrading ipopt with
       ```brew upgrade ipopt --with-openblas```
       per this [forum post](https://discussions.udacity.com/t/incorrect-checksum-for-freed-object/313433/19).
  * Linux
    * You will need a version of Ipopt 3.12.1 or higher. The version available through `apt-get` is 3.11.x. If you can get that version to work great but if not there's a script `install_ipopt.sh` that will install Ipopt. You just need to download the source from the Ipopt [releases page](https://www.coin-or.org/download/source/Ipopt/) or the [Github releases](https://github.com/coin-or/Ipopt/releases) page.
    * Then call `install_ipopt.sh` with the source directory as the first argument, ex: `sudo bash install_ipopt.sh Ipopt-3.12.1`. 
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

## Tips

1. It's recommended to test the MPC on basic examples to see if your implementation behaves as desired. One possible example
is the vehicle starting offset of a straight line (reference). If the MPC implementation is correct, after some number of timesteps
(not too many) it should find and track the reference line.
2. The `lake_track_waypoints.csv` file has the waypoints of the lake track. You could use this to fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/b1ff3be0-c904-438e-aad3-2b5379f0e0c3/concepts/1a2255a0-e23c-44cf-8d41-39b8a3c8264a)
for instructions and the project rubric.

## Hints!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.

## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to we ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./
