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
#include <std_msgs/Float32.h>
#include <tf/transform_broadcaster.h>

//MavGuilocal_planner_solver_lib
#include <stdio.h>
#include "../gui/imgui/imgui.h"
#include "../gui/imgui/imgui_impl_glfw_gl3.h"
#include "../gui/imgui/gl3w.h"
#include <GLFW/glfw3.h>
#include "../gui/mav_imgui.h"
#include <std_msgs/Int32MultiArray.h>

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
     //  randomSpawner dynObjSpawner;

      ofstream logFileStream;
      char packagePath[200];
      std::string fileName;

      //Controllers
      SherpaAckermannPlanner SHERPA_planner_;

      // ros node handles
      ros::NodeHandle nh_;
 
      // ros publisher and subscribers
      ros::Publisher trajectory_pts_pub_;
      ros::Subscriber odom_sub_;
      ros::Subscriber cmd_pose_sub_;
      ros::Subscriber ackrmann_cms_sub_;
      ros::Subscriber lyapunov_sub_;
      
      //Solver Functions
      void initializeAcadoSolver();
      
      //Callbacks
      void OdometryCallback(const nav_msgs::OdometryConstPtr&);
      void CommandPoseCallback(const nav_msgs::OdometryConstPtr&);
      void AkrmCommandsCallback(const geometry_msgs::TwistConstPtr&);
      void ImageCallback(const sensor_msgs::ImageConstPtr&);
      void LyapunovCallback(const std_msgs::Float32ConstPtr&);
      void resetSolver();
      void setDynamicObstacle();
      void getStaticObstacle(); 
      void writeLogData();

      //commands
      Eigen::Matrix<double,3,1>& ang_vel_ref;

     std::vector<Eigen::Vector2d> trees_array;
     bool trees_received = false;
     tf::TransformBroadcaster br;

     void computeClosestTrees();

};
