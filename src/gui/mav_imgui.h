#pragma once

#include <iostream>
#include <fstream>
#include <time.h>
#include <gazebo_msgs/SetModelState.h>
#include <std_srvs/Empty.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/core/version.hpp>
#include <opencv2/opencv.hpp>
#include <opencv/cv.h>

#include "base_imgui.h"
#include "dynObstacle.h"
#include <gazebo_msgs/SetModelState.h>
#include <yaml-cpp/yaml.h>

#include "../gui/imgui/imgui.h"
#include "../gui/imgui/imgui_impl_glfw_gl3.h"
#include "../gui/imgui/gl3w.h"
#include <GLFW/glfw3.h>


#include "../assimp_loader/shader.h"
#include "../assimp_loader/camera.h"
#include "../assimp_loader/model.h"

#include <rm3_ackermann_controller/SetKvalues.h>

class MavGUI: public BaseGUI{
public:
    MavGUI(ros::NodeHandle nh, const std::string& yaml_file);
    
    ~MavGUI() {
	std::cerr << "[MavGUI]: deleting\n";
	std::cerr << "[MavGUI]: deleted\n";
    }

    void showGUI(bool* p_open);
    void processAvatar();
    void init3DObjRendering(std::string&& package_path_str);

protected:

    // ROS service call
    ros::ServiceClient _set_control_gains;
    ros::Time _gui_ros_time;

    // ROS subscriber
    ros::Subscriber _img_sub;
    
    // ROS publisher
    ros::Publisher _cmd_pub, _waypoint_pub;

    void updateDesiredState();
    void sendWaypoint();
    void imageCb(const sensor_msgs::ImageConstPtr&);
    void activatePublisher(const std::string&, const std::string&);
    virtual void resetSolver() = 0;
    void changeControlLawGains();

    // Gui utils
    float _des_pos_vec3f_t[2], _des_pos_vec3f_w[2];
    float _des_orientationf_t, _des_orientationf_w;
    bool _sendingWaypoint = false;
    float _K_values[3];


    float _x_values[PLOT_LINE_ARRAY_SIZE] = {0};
    float _x_min = 0;
    float _x_max = 0;
    float _y_values[PLOT_LINE_ARRAY_SIZE] = {0};
    float _y_min = 0;
    float _y_max = 0;
    float _z_values[PLOT_LINE_ARRAY_SIZE] = {0};
    float _z_min = 0;
    float _z_max = 0;
    float _yaw_values[PLOT_LINE_ARRAY_SIZE] = {0};
    float _yaw_min = 0;
    float _yaw_max = 0;
    

    // Target Variables
    Eigen::Vector3d curr_pos_, curr_att_, curr_lin_vel_, curr_ang_vel_; 
    Eigen::Quaterniond curr_q_;

    cv::Mat currImg_;
    cv::Mat draw_image_, draw_image_res_;
    GLuint my_opengl_texture;
    GLuint my_avatar_texture;

protected:

    Eigen::Vector3d _current_odom_position;
    float _current_yaw_orientation;
    Eigen::Quaterniond _current_orientation;

    std::shared_ptr<Camera> camera;
    std::shared_ptr<Shader> shader;
    std::shared_ptr<Model> model;
    cv::Mat avatarImg, avatarImg_res;

};
