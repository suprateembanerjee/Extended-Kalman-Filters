# Extended Kalman Filter
Self-Driving Car Engineer Nanodegree Program

This project involves the a Simulator developed by Udacity which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases).

## Dependencies

* **g++ 9.3** Installation Instructions: [Mac](https://developer.apple.com/xcode/features/) [Windows](http://www.mingw.org/) 
* **Ubuntu Terminal** for running UNIX Terminal on Windows. [download](https://aka.ms/wslubuntu2004)
* **uWebSocketIO** for smooth data flow between simulator and code. [download](https://github.com/uWebSockets/uWebSockets)  
    ```
    <Navigate to the project folder using Ubuntu terminal>
    chmod u+x install-ubuntu.sh
    ./install-ubuntu.sh
    ./install-linux.sh
    ```
* **cmake 3.5** [Installation Instructions](https://cmake.org/install/)  
* **make 4.1 (Linux and Mac), 3.81 (Windows)**  Installation Instructions : [Mac](https://developer.apple.com/xcode/features/) [Windows](http://gnuwin32.sourceforge.net/packages/make.html)

## Installation and Usage

This repository includes two files that can be used to set up and install [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems. For windows you can use either Docker, VMware, or even [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) to install uWebSocketIO.

Once the install for uWebSocketIO is complete, the main program can be built and run by doing the following from the project top directory.
   ```
   mkdir build && cd build
   cmake .. && make
   ./ExtendedKF
   ```
This enables the application to start listening to the port, which the simulator will connect to. Run the simulator to connect to the port directly.

Here is the main protocol that main.cpp uses for uWebSocketIO in communicating with the simulator.


**INPUT**: values provided by the *simulator to the c++ program*

`["sensor_measurement"]` The measurement that the simulator observed (either lidar or radar)


**OUTPUT**: values provided by the *c++ program to the simulator*

`["estimate_x"]` Kalman Filter estimated position x

`["estimate_y"]` Kalman Filter estimated position y

`["rmse_x"]` Root Mean Squared Error for x

`["rmse_y"]` Root Mean Squared Error for y

`["rmse_vx"]` Root Mean Squared Error for velocity along x

`["rmse_vy"]` Root Mean Squared Error for velocity along y

---

## Data

The data is located in a file titled *obj_pose-laser-radar-synthetic-input.txt* under the directory titled *data*.

### Format:

* Each row represents a sensor measurement where the first column tells you if the measurement comes from radar (R) or lidar (L).

* For a row containing **RaDAR** data, the columns are: 
  `sensor_type`, `rho_measured`, `phi_measured`, `rhodot_measured`, `timestamp`, `x_groundtruth`, `y_groundtruth`, `vx_groundtruth`, `vy_groundtruth`, `yaw_groundtruth`, `yawrate_groundtruth`.

* For a row containing **LiDAR** data, the columns are:
`sensor_type`, `x_measured`, `y_measured, timestamp`, `x_groundtruth`, `y_groundtruth`, `vx_groundtruth`, `vy_groundtruth`, `yaw_groundtruth`, `yawrate_groundtruth`.

If you'd like to generate your own radar and lidar data, see the [Udacity Utilities repo](https://github.com/udacity/CarND-Mercedes-SF-Utilities) for Matlab scripts that can generate additional data.

## Custom Libraries

```Eigen``` is already included in the repo. We majorly use two data structures, `VectorXd` and `MatrixXd` to build our algorithm. However, it must be noted that it does not initialize ```VectorXd``` or ```MatrixXd``` objects with zeros upon creation.
