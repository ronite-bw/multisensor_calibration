# multisensor_calibration

An actively maintained universal calibration toolbox for assisted, target-based multi-sensor calibration with ROS 1 and ROS 2 support. 
It provides a variety of methods and applications to calibrate complex multi-sensor systems, e.g.

- <b>Extrinsic Camera-LiDAR Calibration</b>,
- <b>Extrinsic Camera-Reference Calibration</b>,
- <b>Extrinsic LiDAR-LiDAR Calibration</b>,
- <b>Extrinsic LiDAR-Reference Calibration</b>, and
- <b>Extrinsic LiDAR-Vehicle Calibration</b> (prototype).

The software is licensed under the new [BSD 3-Clause license](license.md). If you use this project for your research, please cite:

```text
TODO: add bibtex
```

The `multisensor_calibration` is released as an official package for ROS 2 and can be installed with apt-get.
Since ROS 1 is soon end-of-life, there will be no official release for ROS 1.
However, there is a version of the source code available for ROS 1 under the branch [noetic](https://github.com/FraunhoferIOSB/multisensor_calibration/tree/noetic).


**Acknowledgement**: This software was developed as part of the projects [AKIT-PRO](https://a-kit.de) (grant no. 13N15673) and [ROBDEKON – Robotic Systems for Decontamination in Hazardous Environments](https://robdekon.de/) (grant nos. 13N14674 and 13N16538), funded by the Federal Ministry of Education and Research (BMBF) under the German Federal Government’s Research for Civil Security program.

------------------------

### Continuous Integration:

| Service    | devel   | main   |
| ---------- | ------- | ------ |
| GitHub     |         | [![deploy](https://github.com/FraunhoferIOSB/multisensor_calibration/actions/workflows/docs.yml/badge.svg)](https://github.com/FraunhoferIOSB/multisensor_calibration/actions/workflows/docs.yml) |

------------------------

### Contents:

- [Getting Started](#getting-started)
    - [Requirements](#requirements)
    - [Build](#build)
- [Contributing](CONTRIBUTING.md)
- [License](LICENSE)


------------------------

## Getting Started

### Requirements

Basic catkin requirements can be installed by calling following command from the top of the catkin workspace:

    rosdep install -y -r --from-paths src --ignore-src

Further requirements:

- [**PCL**](https://pointclouds.org/)
- [**OpenCV**](https://opencv.org/)
- [**Qt**](https://www.qt.io/)
- [**small_gicp**](https://github.com/koide3/small_gicp): This is included as git-submodule and will be cloned and built on the first build. It is licensed under the MIT-License.
- [**OpenMP**](https://www.openmp.org/) (optional): This is used to parallelize and speed up the processing of each point in the point cloud. If not found by CMake the processing will be done sequentially.
- [**Doxygen**](https://www.doxygen.nl/) (optional): If available, this Doxygen documentation will be build automatically.

### Build

1. Clone repository:

    ```bash
    git clone git@github.com:FraunhoferIOSB/multisensor_calibration.git
    ```

2. (OPTIONAL) Clone and build 'small_gicp'.<br>If this step is omitted, it will be executed as part of the first build.

    ```bash
    cd multisensor_calibration && ./thirdparty/clone_small_gicp.sh && ./thirdparty/build_and_install_small_gicp.sh
    ```

3. Initialize `rosdep` and install dependencies:

    ```bash
    sudo rosdep init
    rosdep update
    rosdep install --from-paths src -y --ignore-src
    ```

4. Run `catkin` to build from source:<br>
To build in 'Debug' mode add `-DCMAKE_BUILD_TYPE=Debug` to catkin command.
If 'CMAKE_BUILD_TYPE' omitted, multisensor_calibration will be build in 'Release' mode.

    ```bash
    catkin build -j8 -DCMAKE_BUILD_TYPE=Release multisensor_calibration
    ```

### Run

```bash
rosrun multisensor_calibration multisensor_calibration
```

See user documentation on how to use.

