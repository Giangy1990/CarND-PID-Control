# CarND-Controls-PID
This repository contains all the code needed to complete the final project for the Control course in Udacity's Self-Driving Car Nanodegree.

## Project Introduction
The car has a system able to provide the tracking error of the vehicle with respect to the center of the track. The purpose of this module is the implementation of a PID used to control the steer command and minimize the error.

## PID controller
The PID controller combines the effect of three control law to minimize the error between the reference and the feedback of the system. in particular, the contribution are:
* **P** (proportional): it establishes the linear relationship with the problem. It depends by the value of error at the current time step. Varying this gain, it is possible to reduce the tracking error for the current time step.
      up = -Kp * e
* **I** (integral): it establishes linear relationship between the average value of problem over time. It depends by all the story of the error from the beginning of time to the current time step. Varying this gain, it is possible to reduce the tracking error at steady state.
      ui = -Ki * ∑ e
* **D** (derivative): it establishes linear relationship with the rate of change of problem. It depends by the variation of the error between the previous and the current time step. Varying this gain, it is possible to reduce the oscillation around the steady state value.
      ud = -Kd * d(e)/dt

The final control signal is given by the sum of the three contributions.

## Project implementation
The controller is defined in the file [PID.h](./src/PID.h) and implemented in the file [PID.cpp](./src/PID.cpp).
To use the controller, there are two functions that must be called in sequence:
1. ***UpdateError(cte)***: this function updates the three errors of the PID and calls a private function ***twiddle()*** that performs the fine tuning of the gains.
2. ***TotalError()***: this function returns the control value according to the errors and the gains.

## Tuning
The PID controller has been tuned through two steps:
1. Manual tuning to have an acceptable driving behavior (the car drives remaining into the road margins)
  * First of all, the P gain has been tuned to allow the car to perform at least the first curve of the track;
  * Then, the D gain has been tuned to mitigate the amplitude of the oscillation;
  * In the end, the I gain has been tuned to reduce the error on the steady state (the track on the bridge has been used as reference for this step).


2. 30 minutes of Twiddle method to fine tune the gain. It performs three steps:
  * It waits for `steady_state_steps = 100` (approx 5s) to stabilize the control with the current gains;
  * It performs `observation_steps = 100` (approx 5s) to calculate the total error;
  * It applies the twiddle correction to the gain parameters according to the current total error.

The PID gains obtained by the two tuning phases are showed in the following table.

|    step     |  P  |  I   |  D  |
|:-----------:|:---:|:----:|:---:|
|manual tuning| 0.2 | 0.01 |  3  |
|   twiddle   |0.327|0.0294|4.909|

In this [video](./media/PID.mp4) it is possible to see the car that drives on the track using the final PID gain obtained from the twiddle step. During the record phase, the `observation_steps` has been set to an high value to avoid interference from the twiddle algorithm.

## Conclusions
The PID controller is sufficient to drive a vehicle through the track.
The set of gains used in this project can be improved using a different initial set for the twiddle and increasing the time period of the twiddle algorithm to have a better mean over the track and so perform a better optimization of the PID gains.

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

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`.

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/e8235395-22dd-4b87-88e0-d108c5e5bbf4/concepts/6a4d8d42-6a04-4aa6-b284-1697c0fd6562)
for instructions and the project rubric.
