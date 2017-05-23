step 1
Run Linux unity sim 
#----------------------------------------------------------------------------------#
To run .exe first time. Permission denied may happen, we need to unlock it first
$ chmod x+u term2_sim.x86_64
$ ./term2_sim.x86_64

step 2
(skip to step 3-using install.sh in ubuntu)
Install uWebsockets
#----------------------------------------------------------------------------------#
Dependency:
1-openssl
$ sudo apt-get update && sudo apt-get install libssl-dev

Install:
$ make
$ sudo make install



step 3
Build/Install CarND-Extended-Kalman-Filter-Project-project_v2
#----------------------------------------------------------------------------------#
Dependency:
1- cmake >=3.5
$ sudo apt install cmake

Install: automatic installation with other dependecy load 
$ chmod +x install-ubuntu.sh
$ ./install-ubuntu.sh

step 4
Run make: should all just work
#----------------------------------------------------------------------------------#
$ mkdir build
$ cd build
$ cmake ..
$ make
$ ./ExtendedKF




 




