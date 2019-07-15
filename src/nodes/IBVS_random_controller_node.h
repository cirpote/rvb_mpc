#pragma once

#include <src/local_planner_solver/SHERPA_local_planner.h>
#include "randomObjectSpawner.h"
#include <ros/package.h>

//ros STUFF
#include <cv_bridge/cv_bridge.h>

//opencv library
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Twist.h>

//MavGuilocal_planner_solver_lib
#include <stdio.h>
#include "../gui/imgui/imgui.h"
#include "../gui/imgui/imgui_impl_glfw_gl3.h"
#include "../gui/imgui/gl3w.h"
#include <GLFW/glfw3.h>
#include "../gui/mav_imgui.h"

bool randomWaypointGeneration = false;
bool randomSpawnDynObj = false;

class IBVSRandomNode: public MavGUI
{
 public:
      IBVSRandomNode(ros::NodeHandle& nh, const std::string& yaml_short_file, const std::string& gui_file);
      ~IBVSRandomNode();


      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 private:
   
      // Boolean controls
      bool first_trajectory_cmd_;
      randomSpawner dynObjSpawner;

      ofstream logFileStream;
      char packagePath[200];
      std::string fileName;

      //Controllers
      stivsController stnl_controller;

      // ros node handles
      ros::NodeHandle nh_;
 
      // ros publisher and subscribers
      ros::Publisher command_roll_pitch_yawrate_thrust_pub_;
      ros::Subscriber odom_sub_;
      ros::Subscriber cmd_pose_sub_;
      
      //state variables
      mav_msgs::EigenOdometry trajectory_point;
      
      //Solver Functions
      void initializeAcadoSolver();
      
      //Callbacks
      void OdometryCallback(const nav_msgs::OdometryConstPtr&);
      void CommandPoseCallback(const nav_msgs::OdometryConstPtr&);
      void ImageCallback(const sensor_msgs::ImageConstPtr&);
      void changeDynObstaclePosition();
      void changeFixedObstaclePosition();
      void writeLogData();

      //commands
      Eigen::Vector4d command_roll_pitch_yaw_thrust_st_, command_roll_pitch_yaw_thrust_lt_;  
      mav_msgs::RollPitchYawrateThrust command_roll_pitch_yawrate_thrust_msg;

      //Random things
      bool new_comand = false;

      float RandomGenerationTiming = 10.f;
      float startTime;
      float GenerationNum = 1;

};
