#include <iostream>
#include <thread>
#include <chrono>
#include <fstream>

#include <SDL.h>

#include <pathfinder.h>
#include <team2655/pftools.hpp>

using namespace team2655;

const int WIN_WIDTH = 600;
const int WIN_HEIGHT = 600;
const int SCALE = 60;  // px per meter

const int TICKS = 1024; // ticks per revolution
const double WHEEL_DIAMETER = 0.1;
const double WHEELBASE_WIDTH = .6;

const double MAX_VELOCITY = 5;
const double TIMESTEP = 0.02;             // dt

std::ofstream *datafile = nullptr;

// SDL Window and renderer
SDL_Window *window;
SDL_Renderer *renderer;

PathfinderMode mode;

// SimulatedEncoder position
int lenc = 0, renc = 0;
double gyro = 0;
double x = 0, y = 0;


void printHeader(std::ostream &ostr){
  ostr << "PathfinderMode: " << (int)mode << std::endl;
  ostr << "LeftPercent,RightPercent,LeftEncoder,RightEncoder,Gyro,X,Y,HeadingError" << std::endl;
}

void printStatus(std::ostream &ostr, double lPercent, double rPercent, double angle_difference){
  ostr << lPercent << "," << rPercent << "," << lenc << "," << renc << 
          "," << r2d(gyro) << "," << x << "," << y <<  "," << angle_difference << std::endl;
}

int simulateDrive(double lPercent, double rPercent, double angle_difference){

  static bool first = true;
  if(first){
    first = false;
    printHeader(std::cout);
    printHeader(*datafile);
  }

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

  // Adjust simulated position
  double dx = (((rvel + lvel) / 2) * cos(gyro)) * TIMESTEP;
  double dy = (((rvel + lvel) / 2) * sin(gyro)) * TIMESTEP;

  x += dx;
  y += dy;

  printStatus(std::cout, lPercent, rPercent, angle_difference);
  if(datafile != nullptr)
    printStatus(*datafile, lPercent, rPercent, angle_difference);
}

int main(){

  //////////////////////////////////////////
  // Settings
  //////////////////////////////////////////
  lenc = 0;
  renc = 0;
  gyro = 0;
  mode = PathfinderMode::FrontForward;
  x = 0;
  y = 0;

  //////////////////////////////////////////
  // Setup datafile for logging
  //////////////////////////////////////////

  datafile = new std::ofstream("output.csv");
  if(!datafile->is_open()){
    std::cerr << "Could not open 'output.csv'." << std::endl;
    delete datafile;
    datafile = nullptr;
  }

  //////////////////////////////////////////
  // Setup SDL and create a window
  //////////////////////////////////////////
  //SDL_Init(SDL_INIT_VIDEO);
  //window = SDL_CreateWindow( "PathfinderTest", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, WIN_WIDTH, WIN_HEIGHT, SDL_WINDOW_SHOWN );
  //renderer = SDL_CreateRenderer( window, -1, SDL_RENDERER_ACCELERATED );

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

  std::cout << "Generated. Starting to follow." << std::endl << std::endl;

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

    double angle_difference = r2d(leftFollower.heading) - r2d(gyro);    // Make sure to bound this from -180 to 180, otherwise you will get super large values
    angle_difference = std::fmod(angle_difference, 360.0);
    if (std::abs(angle_difference) > 180.0) {
      angle_difference = (angle_difference > 0) ? angle_difference - 360 : angle_difference + 360;
    }

    double turn = 0.8 * (-1.0/80.0) * angle_difference;

    simulateDrive(l + turn, r - turn, angle_difference);

    if(std::abs(l) <= 0.05 && std::abs(r) <= 0.05)
      stopCounter++;
    else
      stopCounter = 0;

    if(stopCounter >= 20)
      break;

    std::this_thread::sleep_for(std::chrono::milliseconds((int)(TIMESTEP * 1000)));
  }

  std::cout << std::endl << "Done" << std::endl;

  //SDL_Quit();


}
