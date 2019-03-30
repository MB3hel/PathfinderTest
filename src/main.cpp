#include <iostream>
#include <thread>
#include <chrono>

#include <pathfinder.h>
#include <team2655/pftools.hpp>

using namespace team2655;

const int TICKS = 1024; // ticks per revolution
const double WHEEL_DIAMETER = 0.1;
const double WHEELBASE_WIDTH = .6;

const double MAX_VELOCITY = 5;
const double TIMESTEP = 0.02;

PathfinderMode mode;

// SimulatedEncoder position
int lenc = 0, renc = 0;
double gyro = 0;

int simulateDrive(double lPercent, double rPercent){
  // Calculate distances, velocities, and change in heading
  double dsLeft = TIMESTEP * (lPercent * MAX_VELOCITY);
  double dsRight = TIMESTEP * (rPercent * MAX_VELOCITY);
  double lvel = lPercent * MAX_VELOCITY;
  double rvel = rPercent * MAX_VELOCITY;
  double dTheta = ((rvel - lvel) / WHEELBASE_WIDTH) * TIMESTEP;
  // Adjust simulated sensors
  lenc += (dsLeft / WHEEL_DIAMETER / PI) * TICKS;
  renc += (dsRight / WHEEL_DIAMETER / PI) * TICKS;
  gyro += dTheta;

  // Print information
  std::cout << "Drive: " << lPercent << "," << rPercent << std::endl;
  std::cout << "Encoders: " << lenc << "," << renc << std::endl;
  std::cout << "Gyro: " << gyro * 180 / PI << std::endl << std::endl;
}

int main(){

  //////////////////////////////////////////
  // Settings
  //////////////////////////////////////////
  lenc = 0;
  renc = 0;
  gyro = PI;
  mode = PathfinderMode::BackForward;

  //////////////////////////////////////////
  // Generate trajectory
  //////////////////////////////////////////
  Waypoint points[2];
  points[0] = { 0, 0, 0 };
  points[1] = { 5, 2.5, 0 };
  TrajectoryCandidate candidate;
  pathfinder_prepare(points, 2, FIT_HERMITE_CUBIC, PATHFINDER_SAMPLES_HIGH, TIMESTEP, MAX_VELOCITY, 10.0, 60.0, &candidate);
  int length = candidate.length;

  // Array of Segments (the trajectory points) to store the trajectory in
  Segment *trajectory = new Segment[length];

  // Generate the trajectory
  pathfinder_generate(&candidate, trajectory);

  Segment *leftTrajectory = new Segment[length];
  Segment *rightTrajectory = new Segment[length];

  // Generate the Left and Right trajectories of the wheelbase using the 
  // originally generated trajectory
  pathfinder_modify_tank(trajectory, length, leftTrajectory, rightTrajectory, WHEELBASE_WIDTH);

  //////////////////////////////////////////
  // Follow trajectory
  //////////////////////////////////////////
  EncoderConfig leftcfg = { lenc, TICKS, WHEEL_DIAMETER * PI,
                         1.0, 0.0, 0.0, 1.0 / MAX_VELOCITY, 0.0};
  EncoderConfig rightcfg = { renc, TICKS, WHEEL_DIAMETER * PI,
                         1.0, 0.0, 0.0, 1.0 / MAX_VELOCITY, 0.0};

  EncoderFollower leftFollower = pathfindertools::createEncoderFollower(length, mode);
  EncoderFollower rightFollower = pathfindertools::createEncoderFollower(length, mode);

  std::cout << "Generated. Starting to follow." << std::endl;

  int stopCounter = 0;
  while(true){

    double l, r;
    if(mode == PathfinderMode::BackForward || mode == PathfinderMode::FrontReverse){
      // Swap left and right trajectories
      l = pathfindertools::followEncoder(leftcfg, &leftFollower, rightTrajectory, length, lenc, mode);
      r = pathfindertools::followEncoder(rightcfg, &rightFollower, leftTrajectory, length, renc, mode);
    }else{
      l = pathfindertools::followEncoder(leftcfg, &leftFollower, leftTrajectory, length, lenc, mode);
      r = pathfindertools::followEncoder(rightcfg, &rightFollower, rightTrajectory, length, renc, mode);
    }

    simulateDrive(l, r);

    if(std::abs(l) <= 0.05 && std::abs(r) <= 0.05)
      stopCounter++;
    else
      stopCounter = 0;

    if(stopCounter >= 20)
      break;

    std::this_thread::sleep_for(std::chrono::milliseconds((int)(TIMESTEP * 1000)));
  }

  std::cout << "Done" << std::endl;

}
