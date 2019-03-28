#include <iostream>
#include <thread>
#include <chrono>
#include <fstream>

#include <SDL.h>

#include <pathfinder.h>
#include <team2655/pftools.hpp>

using namespace team2655;


// Change these settings to control the simulation
PathfinderMode mode = PathfinderMode::BackReverse;
Waypoint points[] = {
  {0, 0, 0},
  {5, 2.5, 0}
};
const int pointCount = sizeof(points) / sizeof(Waypoint);
const double WHEELBASE_WIDTH = .6;

const double WHEELBASE_DEPTH = 0.8; // Only used for drawing

const double MAX_VELOCITY = 5;
const double MAX_ACCEL = 10;
const double MAX_JERK = 50;
const double TIMESTEP = 0.02;             // dt

// Drawing settings

const int PADDING = 50; // px border
const int WIN_WIDTH = 600;
const int WIN_HEIGHT = 600;
const int SCALE = 60;  // px per meter

const int TICKS = 1024; // simulated encoder ticks per revolution
const double WHEEL_DIAMETER = 0.1;

bool closed = false;

std::ofstream *datafile = nullptr;

// SDL Window and renderer
SDL_Window *window;
SDL_Renderer *renderer;
SDL_Texture *robotTexture;
SDL_Event e;


// Simulated sensors and position
int lenc = 0, renc = 0;
double gyro = 0;
double x = 0, y = 0;

void handleSDLEvents(){
  while ( SDL_PollEvent( &e ) != 0 ) {
    switch ( e.type ) {
      case SDL_QUIT:
        closed = true; break;
      case SDL_WINDOWEVENT:
        if(e.window.event == SDL_WINDOWEVENT_CLOSE)
          closed = true;
        break;
    }
  }
}

void drawFrame(){
  double rectx = (x - WHEELBASE_DEPTH / 2.0) * SCALE;
  double recty = (y + WHEELBASE_WIDTH / 2.0) * SCALE;
  // Mirror y axis (pathfinder uses bottom left as 0,0 but window coords use top left)
  recty = std::abs(WIN_HEIGHT - recty);

  // Drawing on window
  SDL_SetRenderDrawColor( renderer, 255, 255, 255, 255 );
  SDL_RenderClear( renderer );

  SDL_Rect border;
  border.x = PADDING;
  border.y = PADDING;
  border.w = WIN_WIDTH - 2 * PADDING;
  border.h = WIN_HEIGHT - 2 * PADDING;
  SDL_SetRenderDrawColor( renderer, 0, 0, 0, 255 );
  SDL_RenderDrawLine(renderer, 0, WIN_HEIGHT - PADDING, WIN_WIDTH, WIN_HEIGHT - PADDING);
  SDL_RenderDrawLine(renderer, PADDING, 0, PADDING, WIN_HEIGHT);

  SDL_Rect bounds;
  bounds.x = rectx + PADDING;
  bounds.y = recty - PADDING;
  bounds.h = WHEELBASE_WIDTH * SCALE;
  bounds.w = WHEELBASE_DEPTH * SCALE;

  // Draw a point at each waypoint
  SDL_SetRenderDrawColor( renderer, 0, 0, 0, 255 );
  for(int i = 0; i < pointCount; ++i){
    SDL_Rect pt;
    pt.x = (points[i].x * SCALE - 2) + PADDING;
    pt.y = std::abs(WIN_HEIGHT - (points[i].y * SCALE + 2) - PADDING);
    pt.w = 5;
    pt.h = 5;
    SDL_RenderFillRect(renderer, &pt);
  }

  // Drawing on the robot texture
  SDL_SetRenderTarget(renderer, robotTexture);
  SDL_SetRenderDrawColor( renderer, 0, 0, 255, 255 );
  SDL_Rect r;
  r.x = 0; r.y = 0; r.w = WHEELBASE_DEPTH * SCALE; r.h = WHEELBASE_WIDTH * SCALE;
  SDL_RenderFillRect( renderer, &r);
  SDL_SetRenderDrawColor( renderer, 0, 255, 0, 255 );
  SDL_RenderDrawLine(renderer, bounds.w, bounds.h / 2.0, bounds.w - 10, bounds.h / 2.0);

  // Copy the texture onto the window
  SDL_SetRenderTarget(renderer, NULL);
  SDL_RenderCopyEx(renderer, robotTexture, NULL, &bounds, -r2d(gyro), NULL, SDL_FLIP_NONE);

  // Render the frame
  SDL_RenderPresent( renderer );
}

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
  // Mode based settings
  //////////////////////////////////////////
  if(mode == PathfinderMode::FrontReverse){
    gyro = points[0].angle + PI;
    x = points[pointCount - 1].x;
    y = points[pointCount - 1].y;
  }else if (mode == PathfinderMode::BackReverse){
    gyro = points[0].angle;
    x = points[pointCount - 1].x;
    y = points[pointCount - 1].y;
  }else if(mode == PathfinderMode::FrontForward){
    gyro = points[0].angle;
    x = points[0].x;
    y = points[0].y;
  }else{
    gyro = points[0].angle + PI;
    x = points[0].x;
    y = points[0].y;
  }

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
  // Generate trajectory
  //////////////////////////////////////////
  
  std::cout << "Generating path..." << std::endl;
  
  std::cout << "Point count: " << pointCount << std::endl;

  TrajectoryCandidate candidate;
  pathfinder_prepare(points, pointCount, FIT_HERMITE_CUBIC, PATHFINDER_SAMPLES_HIGH, TIMESTEP, MAX_VELOCITY, MAX_ACCEL, MAX_JERK, &candidate);
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

  // Swap the left and right trajectories if required by the chosen mode
  pathfindertools::trajetorySwapByMode(mode, &leftTrajectory, &rightTrajectory);

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

  //////////////////////////////////////////
  // Setup SDL and create a window
  //////////////////////////////////////////
  SDL_Init(SDL_INIT_VIDEO | SDL_INIT_EVENTS);
  window = SDL_CreateWindow( "PathfinderTest", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, WIN_WIDTH, WIN_HEIGHT, SDL_WINDOW_SHOWN );
  renderer = SDL_CreateRenderer( window, -1, SDL_RENDERER_ACCELERATED );
  robotTexture = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_RGBA8888, SDL_TEXTUREACCESS_TARGET, WHEELBASE_DEPTH * SCALE, WHEELBASE_WIDTH * SCALE);

  int stopCounter = 0;
  while(!closed){


    double l = pathfindertools::followEncoder(leftcfg, &leftFollower, leftTrajectory, length, lenc, mode);
    double r = pathfindertools::followEncoder(rightcfg, &rightFollower, rightTrajectory, length, renc, mode);

    double angle_difference = r2d(leftFollower.heading) - r2d(gyro);    // Make sure to bound this from -180 to 180, otherwise you will get super large values
    angle_difference = std::fmod(angle_difference, 360.0);
    if (std::abs(angle_difference) > 180.0) {
      angle_difference = (angle_difference > 0) ? angle_difference - 360 : angle_difference + 360;
    }

    double turn = 0.8 * (-1.0/80.0) * angle_difference;

    simulateDrive(l + turn, r - turn, angle_difference);

    drawFrame();

    if(std::abs(l) <= 0.05 && std::abs(r) <= 0.05)
      stopCounter++;
    else
      stopCounter = 0;

    if(stopCounter >= 20)
      break;

    
    handleSDLEvents();  
    SDL_Delay(TIMESTEP * 1000);
  }

  if(stopCounter >= 20)
    std::cout << std::endl << "Done" << std::endl;
  else
    std::cout << std::endl << "Canceled by user" << std::endl;

  while(!closed)
    handleSDLEvents();

  SDL_DestroyRenderer(renderer);
  SDL_Quit();

}
