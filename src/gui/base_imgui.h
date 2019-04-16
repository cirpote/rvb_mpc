#pragma once

#include <iostream>
#include <fstream>
#include <time.h>
#include "imgui/imgui.h"
#include "imgui/imgui_addons.h"

#include <ros/ros.h>
#include <gazebo_msgs/SetModelState.h>
#include <std_srvs/Empty.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Image.h>
#include <Eigen/Geometry>
#include "../base_ibvs_controller/utils.hpp"

#define PLOT_LINE_ARRAY_SIZE 90
#define IM_ARRAYSIZE(_ARR) ((int)(sizeof(_ARR)/sizeof(*_ARR)))

class BaseGUI {
 public:

  BaseGUI(ros::NodeHandle nh){
    _base_nh = nh;
    _show_gazebo_gui = false;
    _y2k.tm_hour = 0;   _y2k.tm_min = 0; _y2k.tm_sec = 0;
    _y2k.tm_year = 100; _y2k.tm_mon = 0; _y2k.tm_mday = 1;       
  }

  ~BaseGUI(){
	std::cerr << "[BaseGUI]: deleting\n";
	std::cerr << "[BaseGUI]: deleted\n";
  }
  
 protected:

  void playGazeboScene(){
    ros::ServiceClient play_client =  _base_nh.serviceClient<std_srvs::Empty>("/gazebo/unpause_physics");
    std_srvs::Empty srv_play;
    play_client.call(srv_play);
  }

  void pauseGazeboScene(){
    ros::ServiceClient pause_client =  _base_nh.serviceClient<std_srvs::Empty>("/gazebo/pause_physics");
    std_srvs::Empty srv_pause;
    pause_client.call(srv_pause);
  }
  
  void resetGazeboScene(const std::string& model_name) {
    ros::ServiceClient resetModel_client =  _base_nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
    gazebo_msgs::SetModelState srv_model_state;
    srv_model_state.request.model_state.model_name = model_name;
    srv_model_state.request.model_state.pose.position.z = 0.08;

    resetModel_client.call(srv_model_state);
  }

  
  
  void showGazeboGUI(bool* p_open) {
    ImGuiWindowFlags window_flags = 0;
    window_flags |= ImGuiWindowFlags_MenuBar;
 
    ImGui::SetNextWindowSize(ImVec2(200, 200), ImGuiCond_FirstUseEver);
    if (!ImGui::Begin("Gazebo utils", p_open, window_flags)) {
      // Early out if the window is collapsed, as an optimization.
      ImGui::End();
      return;
    }
    ImGui::Text("Gazebo Utilities");
    ImGui::Separator();
    if(ImGui::Button("Pause")) pauseGazeboScene(); ImGui::SameLine();
    if(ImGui::Button("Play")) playGazeboScene();
    ImGui::Separator();
    ImGui::Spacing();

    static char urdf_model_name[64] = "firefly";
    ImGui::InputText("model name", urdf_model_name, 64);
    if(ImGui::Button("Reset")) resetGazeboScene(urdf_model_name); ImGui::NextColumn();
  }

  void addDataPlot(float* values, float& min, float& max, const float& last_value) {
    for(size_t i=1; i < PLOT_LINE_ARRAY_SIZE; ++i){
      values[i-1] = values[i];    
    }
    values[PLOT_LINE_ARRAY_SIZE-1] = last_value;

    if(last_value < min)
      min = last_value;
    if(last_value > max)
      max = last_value;
  }
  
  ros::NodeHandle _base_nh;
  time_t _timer;
  struct tm _y2k = {0};

  bool _show_gazebo_gui;

  // These vars sucks, but are needed since the
  // gui works with float*, not with gnomic::real*
  float _antiwindup_ball_float;
  float _tf_float;    
  
};
