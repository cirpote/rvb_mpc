#include "randomObjectSpawner.h"

using namespace std;

randomSpawner::randomSpawner() : previous_cmd(0.0f, 0.0f, 0.0f) {

  srand (time(NULL));
}

randomSpawner::~randomSpawner(){}

bool randomSpawner::computeDynamicObstacleSpawningPosition(Eigen::Vector3d& cmd, 
                                                           float* delay, 
                                                           Eigen::Vector3d& lastTracjPt, 
                                                           Eigen::Vector3d& spawningPt,
                                                           Eigen::Vector3d& spawningVel){

  curr_cmd = cmd;

  // TODO: calculations here
  if ( (curr_cmd - previous_cmd).norm() < 4.f )
    return false;

  spawningPt.x() = lastTracjPt.x() + ( ((float) rand()) / (float) RAND_MAX ) * 0.3 - 0.15 + ( curr_cmd.x() - previous_cmd.x() ) * 0.25f;
  spawningPt.y() = lastTracjPt.y() + ( ((float) rand()) / (float) RAND_MAX ) * 0.3 - 0.15 + ( curr_cmd.y() - previous_cmd.y() ) * 0.25f;
  spawningPt.z() = lastTracjPt.z() + ( ((float) rand()) / (float) RAND_MAX ) * 0.3 - 0.15 + ( curr_cmd.z() - previous_cmd.z() ) * 0.25f;

  spawningVel.x() = ( ((float) rand()) / (float) RAND_MAX ) * 0.8 - 0.4;
  spawningVel.y() = ( ((float) rand()) / (float) RAND_MAX ) * 0.8 - 0.4;
  spawningVel.z() = ( ((float) rand()) / (float) RAND_MAX ) * 0.8 - 0.4;

  *delay = ( ((float) rand()) / (float) RAND_MAX ) * 0.5;

  previous_cmd = curr_cmd;
}

bool randomSpawner::computeRandomXYCommand(Eigen::Vector2f& rand_xy_cmd){

  rand_xy_cmd.x() = ( ((float) rand()) / (float) RAND_MAX ) * 10 - 5;
  rand_xy_cmd.y() = ( ((float) rand()) / (float) RAND_MAX ) * 6 - 3;
}
