#pragma oncerandomObjestSpawner

#include <src/local_planner_solver/SHERPA_local_planner.h>

class randomSpawner
{
 public:
     randomSpawner();
     ~randomSpawner();

     bool computeDynamicObstacleSpawningPosition(Eigen::Vector3d& cmd, 
                                                 float* delay, 
                                                 Eigen::Vector3d& lastTracjPt, 
                                                 Eigen::Vector3d& spawningPt,
                                                 Eigen::Vector3d& spawningVel);

     bool computeRandomXYCommand(Eigen::Vector2f& rand_xy_cmd);

     EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:

     Eigen::Vector3d previous_cmd, curr_cmd;

};
