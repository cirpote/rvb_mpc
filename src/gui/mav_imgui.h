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

#include "../AprilTags/TagFamily.h"
#include "../AprilTags/TagDetector.h"
#include "../AprilTags/Tag16h5.h"
#include "../AprilTags/Tag25h7.h"
#include "../AprilTags/Tag25h9.h"
#include "../AprilTags/Tag36h9.h"
#include "../AprilTags/Tag36h11.h"

#include "../gui/imgui/imgui.h"
#include "../gui/imgui/imgui_impl_glfw_gl3.h"
#include "../gui/imgui/gl3w.h"
#include <GLFW/glfw3.h>


#include "../assimp_loader/shader.h"
#include "../assimp_loader/camera.h"
#include "../assimp_loader/model.h"

class MavGUI: public BaseGUI{
public:
    MavGUI(ros::NodeHandle nh, const std::string& yaml_file);
    
    ~MavGUI() {
	std::cerr << "[MavGUI]: deleting\n";
	std::cerr << "[MavGUI]: deleted\n";
    }

    void showGUI(bool* p_open);
    void processAvatar();

protected:

    ros::ServiceClient _set_mode_state;

    ros::Time _gui_ros_time;

    //ROS subscriber
    ros::Subscriber _img_sub;

    //ROS publisher
    ros::Publisher _cmd_pub;

    void updateFinalState();
    void imageCb(const sensor_msgs::ImageConstPtr&);
    void activatePublisher(const std::string &cmd_publisher_name);
    void setDynObstacleState();
    virtual void changeDynObstaclePosition() = 0;
    void changeGazeboFixedObstacleposition();
    virtual void changeFixedObstaclePosition() = 0;
    void computeRelativeTargetPose( vector<AprilTags::TagDetection>& detections, Eigen::Matrix4d& T);
    void initializeFromYaml(const std::string& yaml_file);
    void checkAndChangeDynObstacle();

    // Gui utils
    float _des_pos_vec3f[3];
    float _des_orientationf;
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
    float _target_att3f[3];
    float *_t_delay = new float(0.f);
    float start_time;
    bool command_sent = false;
    bool trigger_dyn_obst_1_request = false;
    bool start_time_taken = false;
    Eigen::Vector3d curr_pos_, curr_att_, curr_lin_vel_, curr_ang_vel_; 
    Eigen::Quaterniond curr_q_;
    float prev_time;
    bool isTargetSet = false;
    cv::Matx33f cameraMatrix;
    cv::Vec4f distParam;
    std::vector< cv::Point3f > objPts;
    Eigen::Matrix3d R_body__cam;
    Eigen::Vector3d pCam_B_;
    Eigen::Matrix3d RCam_B__Cam_;

    // Tag Detection Variables
    AprilTags::TagDetector* m_tagDetector;
    AprilTags::TagCodes* m_tagCodes;

    cv::Mat currImg_;
    cv::Mat draw_image_, draw_image_res_;
    GLuint my_opengl_texture;
    GLuint my_avatar_texture;

protected:

    Eigen::Vector3d _current_odom_position;
    float _current_yaw_orientation;
    Eigen::Quaterniond _current_orientation;
    float _u_bounds[2], _v_bounds[2]; 
    bool _UV_changed = false;
    float _target_pos3f[3], _target_vel3f[3];
    float _vert_obst1_[2], _vert_obst2_[2], _horiz_obst_[2], _horiz_obst2_[2];

    std::shared_ptr<Camera> camera;
    std::shared_ptr<Shader> shader;
    std::shared_ptr<Model> model;
    cv::Mat avatarImg, avatarImg_res;
    std::shared_ptr<DynObstacle> dynObst_;
    bool integrateDynObjecet = false;
    float previous_time, dt;

};
