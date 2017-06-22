# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

## Implementation

I have started by getting the current world positioning for the car along with the x and y points. 
Proceeded to first convert from global points to local coordinates. After that I used the global kinematic model
to be able to predict the next state, and edited the model to be simpler and apply latency.

I have then precceded with using other state equations to setup the actuators and update the states for 
x, y, psi and epsi

Link to working video: https://www.youtube.com/watch?v=2nctBKCR0_U&feature=youtu.be

I have ran into some issues where I wasn't getting the correct steering and throttle data. I also ran into issues with 
how to operate the vectors and how to implement the transformations and predictions

One of the main issues that was causing my predictions to go insance, was not properly initializing psi
relative to the car position, which it should be 0.0 since we are at the center of the car. The global 
kinematic model needed some tweaking. Thanks to Alex Cui, Miguel Morales, Denise James, and Oleg in the forums, and slack I was 
able to understand the model better and what state my car should start at and go to


## What does MPC do

MPC allows us to predict the next points and heading where the car should go. Its a pretty neat predictive c
controller for the car

In the begining we need to transform the global positons we are being handed over to local car coordinates:

```

transform_x.resize(ptsx.size());
	transform_y.resize(ptsy.size());
	for (int p = 0; p < ptsx.size(); p++)
	{
		auto x_diff = ptsx[p] - x;
		auto y_diff = ptsy[p] - y;
		auto psi_diff = 0 - psi;

		transform_x[p] = x_diff * CppAD::cos(psi_diff) - y_diff * CppAD::sin(psi_diff);
		transform_y[p] = x_diff * CppAD::sin(psi_diff) + y_diff * CppAD::cos(psi_diff);
	}

```

After getting the local x and y coordinates, the part that was mostly confusing was using the global kinematic model
Here is the global kinematic model which we using in the MPC quizzes:

```
	auto x = state(0);
	auto y = state(1);
	auto psi = state(2);
	auto v = state(3);

	auto delta = actuators(0);
	auto a = actuators(1);

	// Recall the equations for the model:
	// Global Kinematic
	// x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
	// y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
	// psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
	// v_[t+1] = v[t] + a[t] * dt
	next_state(0) = x + v * cos(psi) * dt;
	next_state(1) = y + v * sin(psi) * dt;
	next_state(2) = psi + v / Lf * delta * dt;
	next_state(3) = v + a * dt;

```

You would say it looks pretty straightforward and lets apply it right away.. Except , things are a little different here than the quiz
What we are trying to do is, take the local position of the car and predict where our car in its own understanding of the world should be next 
based on controling where its next way points are.

To be able to do so, we need to assume that the car is at initial positions 0.0 and heading to the new position the global kinematick
model will take the local car from its local space to where it should be next globally

To do so, I have adjusted the kinematic model:

```
// Recall the equations for the model:
		  // Global Kinematic
		  // x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
		  // y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
		  // psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
		  // v_[t+1] = v[t] + a[t] * dt
		  state << 0.0 + v * CppAD::cos(mpc.steering_) * delta_t * latency,
					0.0 + v * CppAD::sin(mpc.steering_) * delta_t * latency,
					0.0,
					0.0 + v * delta_t, 
					cte, 
					epsi;
```

As you can see the initial x and y are zeros, however we are applying the psi and delta changes on the x and y. Now comes to the most important part
that really could keep you awake and miserable, the psi. The psi needs to be at state zero, you should only have your x and y headings and your cte and epsi.

The way I understood it is, your car or your point is on the cars origin where your initial x and y are zero , however still taking into account 
the speed and orientation while your psi is zero. That made the model actually able to do proper predictions and move forward vs forward and backwards.


Another issue I had was with properly setting the following variables:

```
// N timesteps == N - 1 actuations
  size_t n_vars = state.size() * N + 2 * (N - 1);

  // TODO: Set the number of constraints
  size_t n_constraints = N * state.size();

```

The was mainly due to the fact that I have hardcoded the size, and it wasn't  either a good or efficient practice.

Another major bug that really took a while for me to catch is this portion here in the lecture notes:

```
 fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
  fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
  fg[1 + psi_start + t] = psi1 - (psi0 + v0 * delta0 / Lf * dt);
  fg[1 + v_start + t] = v1 - (v0 + a0 * dt);
  fg[1 + cte_start + t] =
      cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
  fg[1 + epsi_start + i] =
      epsi1 - ((psi0 - psides0) + v0 * delta0 / Lf * dt);

```

If you pay attention in the walkthrough, and trust me it took endless time to find this out via re-watching the 
walkthrough is its actually wrong, and in the walkthrough it is instead:

```
	fg[2 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
	fg[2 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
	fg[2 + psi_start + t] = psi1 - (psi0 + v0 * delta0 / Lf * dt);
	fg[2 + v_start + t] = v1 - (v0 + a0 * dt);
	fg[2 + cte_start + t] =
		cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
	fg[2 + epsi_start + t] =
		epsi1 - ((psi0 - psides0) + v0 * delta0 / Lf * dt);

```

Now that I was able to spot the bugs, and be able to adjust my speed. Thanks for Alex Cui , Oleg and other students I was advised
to play around with actuators, and also to play around with speed which thanks to Miguel Morales discovered.



Overall its been a rewarding project. 

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
