#pragma once

#include "../short_term_solver/STIVS_controller.h"

//ros STUFF
#include <cv_bridge/cv_bridge.h>

// TXT output STUFF
#include <iostream>
#include <fstream>

//mav msgs
#include <mav_msgs/default_topics.h>

//opencv library
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/PointStamped.h>

//MavGui
#include <stdio.h>
#include "../gui/imgui/imgui.h"
#include "../gui/imgui/imgui_impl_glfw_gl3.h"
#include "../gui/imgui/gl3w.h"
#include <GLFW/glfw3.h>
#include "../gui/mav_imgui.h"

class IBVSNode: public MavGUI
{
 public:
      IBVSNode(ros::NodeHandle& nh, const std::string& yaml_short_file, const std::string& yaml_long_file, const std::string& gui_file);
      ~IBVSNode();

      //Controllers
      stivsController stnl_controller;

      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 private:
   
      // Boolean controls
      bool first_trajectory_cmd_;
      
      // ros node handles
      ros::NodeHandle nh_;
 
      // ros publisher and subscribers
      ros::Publisher command_roll_pitch_yawrate_thrust_pub_;
      ros::Subscriber odom_sub_;
      ros::Subscriber cmd_pose_sub_;
      
      //state variables
      mav_msgs::EigenOdometry odometry;
      mav_msgs::EigenOdometry trajectory_point;
      
      //Solver Functions
      void initializeAcadoSolver();
      
      //Callbacks
      void OdometryCallback(const nav_msgs::OdometryConstPtr&);
      void CommandPoseCallback(const nav_msgs::OdometryConstPtr&);
      void ImageCallback(const sensor_msgs::ImageConstPtr&);
      void changeDynObstaclePosition();
      void changeFixedObstaclePosition();

      //commands
      Eigen::Vector4d command_roll_pitch_yaw_thrust_st_, command_roll_pitch_yaw_thrust_lt_;  

};
