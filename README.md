# **Extended Kalman Filter Project-1 Term-2**
Self-Driving Car Engineer Nanodegree Program
![alt text][image0]

[//]: # (Image References)
[image0]: ./CarND-Extended-Kalman-Filter-Project/Docs/Result1.png "project"


### Overview ###
In this project utilize a kalman filter to estimate the state of a moving object of interest with noisy lidar and radar measurements.   
Passing the project requires obtaining RMSE values that are lower that the tolerance outlined in the project reburic. 

```sh

                               
                   y
                   ^     
                   |  [object] (px,py)
                   |   /
                   |  / range, range_dot
                   | /      (Radar)
                   |/ angle
                   ------------> x 
		lidar: px,py
		radar: range,range_dot,angle

```
In this project, we implement

|    Sensor     | Prediction    | Correction  |
|:-------------:|:-------------:|:-----------:|
|     Lidar     |      KF       |      KF     |
|     Radar     |      KF       |      EKF    |

### KF/EKF Steps ###


**STEP 1-Prediction:** Predicting the next state x', and covariance P'

_KF Formula_

```sh
		x' = F*x + u
		P' = F*P*Ft + Q
	where
		x  = state [px,py,vx,vy]
		P  = state covariance
		u  = control input or acceleration (ax,ay) in this case 
		     (assuem u = 0 for a constant velocity model)
		F  = State Jacobian matrix
		Q  = Process/motion noise (due to acceleration)
		
	
```


_Linear motion model(with a constant velocity)_:g(x)

```sh
	
		px' = px + vx*dt + x motion noise
		py' = py + vy*dt + y motion noise
		vx' = vx + x velocity noise
		vy' = vy + y velocity noise
	where
		px  = x position
		py  = y position
		vx  = x velocity 
		vy  = y velocity
		x motion noise = 0.5*ax*dt*dt
		y motion noise = 0.5*ay*dt*dt
		x velocity noise = ax*dt
		y velocity noise = ay*dt

```

_State Jacobian Matrix_:F

```sh
		F = dg/dx = 	[1 0 dt 0 ]
				[0 1 0  dt]
				[0 0 1  0 ]
				[0 0 0  1 ]

```

_Control Jacobian and Process Noise_:V,Q

```sh

		u = [ax,ay]
		Q = V*var_u*Vt

	where
		
		V = dg/du= 	[0.5*dt*dt   0        ]
				[0           0.5*dt*dt]
				[dt	     0        ]
				[0	     dt	      ]


		var_u = [noise_ax    0        ]
			[0           noise_ay ]	
		
		Q =     [dt_4/4*noise_ax, 0,               dt_3/2*noise_ax, 0,		     ]
			[0,               dt_4/4*noise_ay, 0,               dt_3/2*noise_ay, ]
			[dt_3/2*noise_ax, 0,               dt_2*noise_ax,   0,               ]
			[0,               dt_3/2*noise_ay, 0,               dt_2*noise_ay    ] 
	
		

```

**STEP 2-Correction:**  

_EKF Formula (Correction)_

```sh
	From the prediction set
		x = x' 
		P = P'

	then do the update

		K = 	    P*HT
              		--------------
               		(H*P*HT + R)

        	x  =     x + K*(Z - z)

        	P  =     (I - Kt*H)*x

	where 
		H  = Measurement Jacobian 
		Z  = Measurement
		z  = Predicted measurement
		R  = Measurement noises 
	

```

_Radar Measurement Update_
```sh

	h(x)  = [rho    ]   = [sqrt(px*px + py*py) ]
                [theta  ]     [atan2(py,px)        ]
		[rho_dot]     [(px*vx + py*vy)/rho ]

	where

		rho = range measurement of a target
		theta = bearing angle of a target
		rho_dot = rate of change in range of a target 

	
	H(x)  = dh/dx  = [ (px/c2),                (py/c2),               0,     0,     ]
	   		 [-(py/c1),                (px/c1),               0,     0,     ]
		 	 [ py*(vx*py - vy*px)/c3,  px*(px*vy - py*vx)/c3, px/c2, py/c2  ]

	where
		c1 = px*px + py*py
		c2 = sqrt(c1)
		c3 = c1*c2

	[Proof](./CarND-Extended-Kalman-Filter-Project/Docs/MeasurementJacobian.pdf)


```


_Lidar Measurement Update_
```sh

  	h(x) = [px]   =   H * [px]
               [py]           [py]
			      [vx]
	                      [vy]

	where

	px = a target/object x-position
	py = a target/object y-position


	H    = [1, 0, 0, 0,]
  	       [0, 1, 0, 0 ]
```


### Installation ###
(Note: see InstallationNote.md for my installation in Ubuntu) 

This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases/tag/v1.0)

This repository includes two files that can be used to set up and intall [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems.   
For windows you can use either Docker, VMware, or even [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) to install uWebSocketIO. 

Once the install for uWebSocketIO is complete, the main program can be built and ran by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./ExtendedKF



### Debugging with Eclipse IDE (Ubuntu 16.04) ###
see README in /ide_profiles for instruction and the example built project in EKF_WS folder


### File and data structure ###
Note that the programs that need for the project are in src/FusionEKF.cpp, FusionEKF.h, kalman_filter.cpp, kalman_fitler.h, tools.cpp, and tools.h

Here is the main protcol that main.cpp uses for uWebSocketIO in communicating with the simulator.

INPUT: values provided by the simulator to the c++ program

["sensor_measurement"] => the measurment that the simulator observed (either lidar or radar)


OUTPUT: values provided by the c++ program to the simulator

["estimate_x"] <= kalman filter estimated position x
["estimate_y"] <= kalman filter estimated position y
["rmse_x"]
["rmse_y"]
["rmse_vx"]
["rmse_vy"]

---

## Other Important Dependencies

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

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make` 
   * On windows, you may need to run: `cmake .. -G "Unix Makefiles" && make`
4. Run it: `./ExtendedKF path/to/input.txt path/to/output.txt`. You can find
   some sample inputs in 'data/'.
    - eg. `./ExtendedKF ../data/obj_pose-laser-radar-synthetic-input.txt`

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Generating Additional Data

This is optional!

If you'd like to generate your own radar and lidar data, see the
[utilities repo](https://github.com/udacity/CarND-Mercedes-SF-Utilities) for
Matlab scripts that can generate additional data.

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project resources page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/382ebfd6-1d55-4487-84a5-b6a5a4ba1e47)
for instructions and the project rubric.


Regardless of the IDE used, every submitted project must
still be compilable with cmake and make.
