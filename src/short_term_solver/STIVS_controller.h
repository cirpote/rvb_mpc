#pragma once

#include "../base_ibvs_controller/base_IBVS_controller.h"

#include "core/acado_common.h"
#include "core/acado_auxiliary_functions.h"

//namespace shortTerm{

      ACADOvariables acadoVariables;
      ACADOworkspace acadoWorkspace; 

      class stivsController : public BaseibvsController
      {  
      public:
            stivsController(const std::string& yaml_file);
            ~stivsController();

            bool InitializeController();
            void calculateRollPitchYawRateThrustCommands(Eigen::Vector4d&, Eigen::Vector2f&);
            bool setCommandPose(const nav_msgs::Odometry);
            void restartSolver();
            // void inline setGimbalAxes(const float& gimbal_pitch_axis, const float& gimbal_yaw_axis)
            // { gimbal_pitch_axis_ = gimbal_pitch_axis; gimbal_yaw_axis_ = gimbal_yaw_axis; };

            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            //private:

            Eigen::Vector4d computeControls();
            void computeDesiredState();
            void printDifferentialVariables();
            void printControlVariables();
            void fillOnlineData();

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
            // float gimbal_pitch_axis_, gimbal_yaw_axis_;

      };
//};
