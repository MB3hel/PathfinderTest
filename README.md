# PathfinderTest

Mathematical simulation of pathfinder for FRC to test Team 2655 Pathfinder tools library.

## Setup to build (Windows)

### Install required software
Install [CMake](https://cmake.org/)
Install [Conan](https://conan.io/)

Conan is easiest to install with pip

### Generate the project
Open a cmd window (or powershell) in the project folder
```
mkdir build
cd build
conan install ..
cmake .. -A x64
start PathfinderTest.sln
```

A project will open in Visual Studio

### Building
Click the run button in Visual Studio

## Use
The settings for paths, the window, pathfinder, and the robot are at the top. Most should stay the same. Changing the mode variable changes how the path is driven where Front means with the front of the robot (green line), back means with the back of the robot, forward means from the start of the path to the end, and reverse means from the end of the path to the start.