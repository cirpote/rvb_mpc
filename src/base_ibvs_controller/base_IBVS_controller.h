#pragma once

#include <boost/bind.hpp>
#include <Eigen/Eigen>
#include <stdio.h>
#include "utils.hpp"
#include <chrono>
#include <yaml-cpp/yaml.h>

//ros
#include <ros/ros.h>
#include <ros/package.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>


class BaseibvsController
{
 public:
      BaseibvsController(const std::string& yaml_file);
      ~BaseibvsController();

      virtual bool InitializeController() = 0;
      virtual void calculateRollPitchYawRateThrustCommands(Eigen::Vector2d&) = 0;
      void setOdometry(const nav_msgs::OdometryConstPtr&);
      virtual bool setCommandPose(const nav_msgs::Odometry) = 0;
      float inline getMass(){return mass_;}
      void ComputeIntegralAction();
      
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      // path_constraints params
      Eigen::Vector2d q_command_, q_position_;
      double q_orientation_, q_steering_angle_;

      // terminal_constraints params
      Eigen::Vector2d qf_position_;
      double qf_orientation_, qf_steering_angle_;

      // boundary_constraints params
      Eigen::Vector2d vel_bnds_, phi_cmd_bnds_;

      // obstacle params
      Eigen::Vector2d pObst_vert1;
      Eigen::Vector2d pObst_vert1_WMat;
      float obst_penalty_;

      //constant by parameters
      float l_;
      float yaw_rate_damping;
      float mass_;
      float roll_time_constant_, roll_gain_, pitch_time_constant_, pitch_gain_;
      const double kGravity = 9.8066;
      int verbosity_;
      
      //state variables
      EigenOdometry odometry;
      EigenOdometry trajectory_point;
      float steer_angle_;

      //integral action variables
      Eigen::Vector4d integral_action_, integral_action_weights_;
      float attraction_ball_, anti_windup_ball_;
      float delta_time_, prev_time_;
      bool is_first_odometry_set_ = true;
      tf::TransformListener tfListner_;

 private:

      void initializeControllerfromYAML(const std::string& yaml_file);
};
