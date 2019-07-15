#pragma once

#include <src/base_ibvs_controller/base_IBVS_controller.h>

#include "core/acado_common.h"
#include "core/acado_auxiliary_functions.h"

ACADOvariables acadoVariables;
ACADOworkspace acadoWorkspace; 

class SherpaAckermannPlanner : public BaseibvsController
{  
public:
      SherpaAckermannPlanner(const std::string& yaml_file);
      ~SherpaAckermannPlanner();

      bool InitializeController();
      void calculateRollPitchYawRateThrustCommands(Eigen::Vector4d&);
      bool setCommandPose(const nav_msgs::Odometry);
      void restartSolver();

      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      //private:

      void printDifferentialVariables();
      void printControlVariables();

      const int NUM_STEPS = 5; /* Number of real-time iterations. */
      const float KKT_THRESHOLD = 1e-5; /* Threshold as termination criterion. */

      // solver matrices
      Eigen::Matrix<double, ACADO_NY, ACADO_NY> W_;
      Eigen::Matrix<double, ACADO_NYN, ACADO_NYN> WN_;
      Eigen::Matrix<double, ACADO_N + 1, ACADO_NX> state_;
      Eigen::Matrix<double, ACADO_N, ACADO_NU> input_;
      Eigen::Matrix<double, ACADO_N, ACADO_NY> reference_;
      Eigen::Matrix<double, 1, ACADO_NYN> referenceN_;
      Eigen::Matrix<double, ACADO_N + 1, ACADO_NOD> acado_online_data_;
      
      //debug info
      double solve_time_average_;
      int solve_time, iter;

};