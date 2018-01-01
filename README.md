# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

There's an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`.

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Rubric Points

### Here I will consider the [rubric points](https://review.udacity.com/#!/rubrics/824/view) individually and describe how I addressed each point in my implementation.

---
### Your code should compile
I added the class Twiddle (`Twiddle.h` and `Twiddle.cpp`), therefore `CMakeLists.txt` has been updated accordingly. Code compiles with `cmake` and `make` without errors.

### The PID procedure and P, I, D components
The PID implementation follows what was taught in the lessons. For integral error, the recent 10 CTE errors are taken, but not all.

P: Proportional - the correction is applied to the control variable which is proportional to the difference between desired value and reference value.
I: Integral - magnifies the effect of long-term steady-state errors, applying ever-increasing effort until they reduce to zero
D: Derivative - concerned with the rate-of-change of the error with time. If the measured variable approaches the setpoint rapidly, then the actuator is backed off early to allow it to coast to the required level.

### Describe how the final hyperparameters were chosen
I combined manual tuning and twiddle to choose the final hyperparameters. The manual tuning helped me find the rough values which could keep vehicle on the track and twiddle helped to further tune the parameters to get the final result with minimum CTE errors.

To make it convenient, my app supports running in two modes: 'twiddle' and 'pid'. When run `./pid twiddle` it starts using twiddle to tune parameters. When run `./pid` or `./pid Kp Ki Kd` it runs in 'pid' mode with default or provided parameters to control how vehicle runs.

The twiddle algorithm is implemented in class Twiddle. To avoid unnecessary waiting, class Twiddle will check the speed and CTE error (in `Twiddle.cpp`, lines 60 ~ 64 ) to determine whether vehicle is stuck or runs too far away. If it is, this round of tuning will be terminated and starts next one. We will send the 'reset' message to simulator (in `main.cpp`, lines 81 ~ 85) before starts new round testing to make sure the vehicle is in good shape.

### The vehicle must successfully drive a lap around the track
Yes, the vehicle will successfully drive around the track with the chosen parameters. Please see the video.mp4 for details.
