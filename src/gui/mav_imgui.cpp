#include "mav_imgui.h"

MavGUI::MavGUI(ros::NodeHandle nh, const std::string& yaml_file) : BaseGUI(nh) {
  
  dynObst_ = std::shared_ptr<DynObstacle>( new DynObstacle(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()) );

  previous_time = ros::Time::now().toSec();

  _des_pos_vec3f[0] = 0.f;
  _des_pos_vec3f[1] = 0.f;
  _des_pos_vec3f[2] = 0.f;
  _des_orientationf = 0.f;

  _u_bounds[0] = -300.f;
  _u_bounds[1] = 300.f;

  _v_bounds[0] = -300.f;
  _v_bounds[1] = 300.f;

  _target_pos3f[0] = 3.5;
  _target_pos3f[1] = 0.f;
  _target_pos3f[2] = 3.f;

  _target_vel3f[0] = 0.f;
  _target_vel3f[1] = 0.f;
  _target_vel3f[2] = 0.f;
  
  _target_att3f[0] = 0.f;
  _target_att3f[1] = 0.f;
  _target_att3f[2] = 0.f;

  _vert_obst1_[0] = 1.f;
  _vert_obst1_[1] = 0.45;

  _vert_obst2_[0] = 2;
  _vert_obst2_[1] = -0.45;

  _horiz_obst_[0] = 1.5;
  _horiz_obst_[1] = 2.6;

  _horiz_obst2_[0] = 1.5;
  _horiz_obst2_[1] = 3.7;

  _gui_ros_time = ros::Time::now();

  yellow_.a = 0.8; yellow_.r = 1.0; yellow_.g = 1.0; yellow_.b = 0.0;
  green_.a = 0.8; green_.r = 0.0; green_.g = 1.0; green_.b = 0.0;
  red_.a = 0.8; red_.r = 1.0; red_.g = 0.0; red_.b = 0.0;
  blue_.a = 0.8; blue_.r = 0.0; blue_.g = 0.0; blue_.b = 1.0;
  cyan_.a = 0.8; cyan_.r = 0.0; cyan_.g = 1.0; cyan_.b = 1.0;
  orange_.a = 0.8; orange_.r = 1.0; orange_.g = 0.55; orange_.b = 0.f;

  _img_sub = _base_nh.subscribe("/firefly/downward_cam/camera/image", 1, &MavGUI::imageCb, this, ros::TransportHints().tcpNoDelay());
  _set_mode_state = _base_nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
  _vis_pub = _base_nh.advertise<visualization_msgs::Marker>( "/visualization_marker", 1 );
  _path_pub = _base_nh.advertise<nav_msgs::Path>( "/path", 1);
  command_gimbal_pitch_axis_ = _base_nh.advertise<std_msgs::Float64>("/firefly/gimbal_pitch_angle_controller/command/",1);
  command_gimbal_yaw_axis_ = _base_nh.advertise<std_msgs::Float64>("/firefly/gimbal_yaw_angle_controller/command/",1);

  m_tagDetector = NULL;
  m_tagCodes = new AprilTags::TagCodes(AprilTags::tagCodes36h11);
  m_tagDetector = new AprilTags::TagDetector(*m_tagCodes);

  initializeFromYaml(yaml_file);

  camera = std::make_shared<Camera>( glm::vec3(15.f, 20.f, -65.0f), glm::vec3(0.0f, 1.0f, 0.0f), 100.f );
  std::cout << FGRN("Camera Correctly Initialized\n\n");

  avatarImg = cv::Mat(cv::Size(640,480), CV_8UC3);
  
  
}

void MavGUI::init3DObjRendering(std::string&& package_path_str){

  char vs_path[100], fs_path[100], model_path[100];

  strcpy(vs_path, package_path_str.c_str());
  strcat(vs_path, "/src/assimp_loader/assets/shaders/modelTextured.vs");

  strcpy(fs_path, package_path_str.c_str());
  strcat(fs_path, "/src/assimp_loader/assets/shaders/modelTextured.fs");

  strcpy(model_path, package_path_str.c_str());
  strcat(model_path, "/src/assimp_loader/assets/zeppelin/ZEPLIN_OBJ.obj");

  shader =  std::make_shared<Shader>( vs_path, fs_path );
  std::cout << FGRN("Shader Correctly Initialized\n\n");

  model = std::make_shared<Model>(model_path);
  std::cout << FGRN("Model Correctly Initialized\n\n");

}

void MavGUI::processAvatar(){

    ImVec4 clear_color = ImColor(34, 43, 46);
    glClearColor(clear_color.x, clear_color.y, clear_color.z, clear_color.w);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glViewport(0, 0, 640, 480);

    // don't forget to enable shader before setting uniforms
    shader->use();

    // view/projection transformations
    glm::mat4 projection = glm::perspective(glm::radians(camera->Zoom), (float)640 / (float)480, 0.1f, 250.0f);
    glm::mat4 view = camera->GetViewMatrix();
    shader->setMat4("projection", projection);
    shader->setMat4("view", view);

    // render the loaded model
    glm::mat4 currmodel = glm::mat4(1.0f);


    float angle = 2 * std::acos(_current_orientation.w());
    float norm_fact = std::sqrt(1 - _current_orientation.w()*_current_orientation.w());
    glm::vec3 rot_axis(_current_orientation.y() / norm_fact,
                       _current_orientation.z() / norm_fact,
                       _current_orientation.x() / norm_fact);


    currmodel = glm::rotate(currmodel, angle, rot_axis);
    currmodel = glm::translate(currmodel, glm::vec3(0.f, 0.f, 0.f)); // translate it down so it's at the center of the scene
    currmodel = glm::scale(currmodel, glm::vec3(0.08f, 0.08f, 0.08f));	// it's a bit too big for our scene, so scale it down
    shader->setMat4("model", currmodel);
    model->Draw(*shader);

    glReadPixels ( 0, 0, 640, 480, GL_BGR,
                   GL_UNSIGNED_BYTE, ( GLubyte * ) avatarImg.data );

    cv::flip(avatarImg, avatarImg, 0);
    cv::cvtColor(avatarImg, avatarImg, CV_RGB2BGR);
    cv::resize(avatarImg, avatarImg_res, cv::Size(avatarImg.cols/2, avatarImg.rows/2) );

}

void MavGUI::initializeFromYaml(const std::string& yaml_file){

  std::cerr << "\n" << FBLU("Initializing GUI from:") << " " << yaml_file << "\n" << "\n";
  YAML::Node configuration = YAML::LoadFile(yaml_file);

  std::vector<double> camera_intrinsics, distortion_params, pCam_B, qCam_B__Cam;

  double _target_size = configuration["camera_parameters"]["target_size"].as<double>();
  camera_intrinsics = configuration["camera_parameters"]["camera_intrinsics"].as<std::vector<double>>();
  distortion_params = configuration["camera_parameters"]["distortion_params"].as<std::vector<double>>();
  pCam_B = configuration["camera_parameters"]["pCam_B"].as<std::vector<double>>();
  qCam_B__Cam = configuration["camera_parameters"]["qCam_B__Cam"].as<std::vector<double>>();


  cameraMatrix = cv::Matx33f( camera_intrinsics[0], 0, camera_intrinsics[2],
                              0, camera_intrinsics[1], camera_intrinsics[3],
                              0, 0, 1 ); 

  distParam = cv::Vec4f(distortion_params[0], distortion_params[1], distortion_params[2], distortion_params[3]);

  double s = _target_size/2;
  objPts.push_back(cv::Point3f(-s,-s, 0));
  objPts.push_back(cv::Point3f( s,-s, 0));
  objPts.push_back(cv::Point3f( s, s, 0));
  objPts.push_back(cv::Point3f(-s, s, 0));

  R_body__cam << 0, 0, 1,
                 1, 0, 0,
                 0, -1, 0;

  pCam_B_ << pCam_B[0], pCam_B[1], pCam_B[2];


  Eigen::AngleAxisd rollAngle(qCam_B__Cam[0], Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd pitchAngle(qCam_B__Cam[1], Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd yawAngle(qCam_B__Cam[2], Eigen::Vector3d::UnitZ());

  RCam_B__Cam_ = yawAngle * pitchAngle * rollAngle;

}

void MavGUI::computeRelativeTargetPose( vector<AprilTags::TagDetection>& detections, Eigen::Matrix4d& T){

    cv::Mat rvec, tvec;
    Eigen::Matrix3d wRo;

    std::pair<float, float> p1 = detections[0].p[0];
    std::pair<float, float> p2 = detections[0].p[1];
    std::pair<float, float> p3 = detections[0].p[2];
    std::pair<float, float> p4 = detections[0].p[3];

    std::vector<cv::Point2f> imgPts;
    imgPts.push_back(cv::Point2f(p1.first, p1.second));
    imgPts.push_back(cv::Point2f(p2.first, p2.second));
    imgPts.push_back(cv::Point2f(p3.first, p3.second));
    imgPts.push_back(cv::Point2f(p4.first, p4.second));


    cv::solvePnP(objPts, imgPts, cameraMatrix, distParam, rvec, tvec);
    cv::Mat r;
    cv::Rodrigues(rvec, r);

    r.convertTo(r, CV_64FC1);
    tvec.convertTo(tvec, CV_64FC1);
    
    wRo << r.at<double>(0,0), r.at<double>(0,1), r.at<double>(0,2), r.at<double>(1,0), r.at<double>(1,1), r.at<double>(1,2), r.at<double>(2,0), r.at<double>(2,1), r.at<double>(2,2);

    T.topLeftCorner(3,3) = wRo;
    T.col(3).head(3) << tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2);
    T.col(3).head(3) = T.col(3).head(3);
    T.row(3) << 0,0,0,1; 

    return;
}


void MavGUI::imageCb(const sensor_msgs::ImageConstPtr& img_msg){


  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::RGB8);

  draw_image_ = cv_ptr->image.clone();

  //currImg_ = cv_ptr->image.clone();
  //cv::cvtColor(currImg_, draw_image_, cv::COLOR_GRAY2BGR);

  vector<AprilTags::TagDetection> detections = m_tagDetector->extractTags(currImg_);
  
  cv::circle(draw_image_, cv::Point(draw_image_.cols/2, draw_image_.rows/2), 3, cv::Scalar(255,0,0));
  Eigen::Matrix4d T;

  if (detections.size() > 0) {
    
    computeRelativeTargetPose(detections, T);
    cv::circle(draw_image_, cv::Point2f(detections[0].cxy.first, detections[0].cxy.second), 3, cv::Scalar(0,255,0), 4);
    cv::circle(draw_image_, cv::Point2f(detections[0].p[0].first,detections[0].p[0].second),2,cv::Scalar(255,0,0),2); //punto in basso a destra
    cv::circle(draw_image_, cv::Point2f(detections[0].p[1].first,detections[0].p[1].second),2,cv::Scalar(255,0,0),2); //punto in alto a destra
    cv::circle(draw_image_, cv::Point2f(detections[0].p[2].first,detections[0].p[2].second),2,cv::Scalar(255,0,0),2); //punto in alto a sinistra
    cv::circle(draw_image_, cv::Point2f(detections[0].p[3].first,detections[0].p[3].second),2,cv::Scalar(255,0,0),2); //punto in basso a sinistra

    // COMPUTING TARGET WORLD COORDINATES
    /*Eigen::Vector3d curr_pT_W = _current_orientation.toRotationMatrix() * Eigen::AngleAxisd(_current_yaw_orientation, Eigen::Vector3d::UnitZ()).inverse() 
          * Eigen::AngleAxisd(_current_yaw_orientation, Eigen::Vector3d::UnitZ()).inverse() * ( RCam_B__Cam_*R_body__cam*T.col(3).head(3) + pCam_B_) + _current_odom_position;
          
    if( ( curr_pT_W - pT_W_ ).norm() > 1e-1 ){
      std::cout << FRED("ERROR: ") << ( curr_pT_W - pT_W_ ).norm() << std::endl;
      pT_W_ = curr_pT_W;
    }*/

  }

  cv::resize(draw_image_, draw_image_res_, cv::Size( draw_image_.cols/2, draw_image_.rows/2) );

}


void MavGUI::updateFinalState() {

  nav_msgs::Odometry cmd_msg;
  cmd_msg.header.stamp = ros::Time::now();
  cmd_msg.pose.pose.position.x = _des_pos_vec3f[0];
  cmd_msg.pose.pose.position.y = _des_pos_vec3f[1];
  cmd_msg.pose.pose.position.z = _des_pos_vec3f[2];

  Eigen::Quaterniond q( Eigen::AngleAxisd(_des_orientationf, Eigen::Vector3d::UnitZ() ) );
  cmd_msg.pose.pose.orientation = mav_utils::fromEigenQuaternionrToQuaternion(q);
  _cmd_pub.publish( cmd_msg );
}

void MavGUI::activatePublisher(const std::string &cmd_publisher_name) {
  _cmd_pub = _base_nh.advertise<nav_msgs::Odometry>(cmd_publisher_name, 1, this);
}

void MavGUI::showGUI(bool *p_open) {

  processAvatar();
  float curr_time = ros::Time::now().toSec();
  dt = curr_time - previous_time;
  previous_time = curr_time;

  ImGuiWindowFlags window_flags = 0;
  window_flags |= ImGuiWindowFlags_MenuBar;
 
  ImGui::SetNextWindowSize(ImVec2(550, 680), ImGuiCond_FirstUseEver);
  if (!ImGui::Begin("GnomicMavGUI", p_open, window_flags)) {
    // Early out if the window is collapsed, as an optimization.
    ImGui::End();
    return;
  }
  
  /// MAIN WINDOW CONTENT
  ImGui::TextColored(ImVec4(0.4f, 0.8f, 0.0f, 1.0f), "GnomicMavGUI");
  ImGui::Text("Activate callback and publisher with Gazebo simulator open before sending the desired goal.");
  ImGui::Spacing();
  if(ImGui::Button("Gazebo Utils")) _show_gazebo_gui ^= 1;
  ImGui::Spacing(); 
  ImGui::Separator();
  ImGui::Spacing(); 
  ImGui::Separator();
  ImGui::Spacing();
  static char cmd_pub[64] = "/command/pose";
  ImGui::InputText("cmd_pub", cmd_pub, 64);
  bool activate_publisher = ImGui::Button("Activate Publisher");
  if (activate_publisher) {
    std::cerr << "activating publisher: " << cmd_pub << std::endl;
    activatePublisher(cmd_pub);
  }

  ImGui::Spacing();
  ImGui::Separator();
  
  
  ImGui::Columns(2, "state_time");
  ImGui::Text("Desired Final State.");
  ImGui::DragFloat3("x y z [meters]", _des_pos_vec3f, 0.01f, -10.0f, 10.0f);
  ImGui::DragFloat("yaw [radians]", &_des_orientationf, 0.01f, -M_PI, M_PI);
  bool update_final_state2 = ImGui::Button("Update Final State");
  if (update_final_state2) {
    command_sent = true;
    updateFinalState();
  }


  ImGui::NextColumn();
  ImGui::Text("Activate Perception Boundaries");
  ImGui::DragFloat2("u_Lb u_Ub [pixels]", _u_bounds, 0.01f, -10000.0f, 10000.0f);
  ImGui::DragFloat2("v_Lb v_Ub [pixels]", _v_bounds, 0.01f, -10000.0f, 10000.0f);
  if(ImGui::Button("Activate Perception Boundaries")) std::cout << FRED("set UV bounds not available anymore!!\n");
  
  ImGui::Spacing();
  ImGui::Columns(1);
  
  //add new data for plot and update min_max
  addDataPlot(_x_values, _x_min, _x_max, _current_odom_position(0));
  addDataPlot(_y_values, _y_min, _y_max, _current_odom_position(1));
  addDataPlot(_z_values, _z_min, _z_max, _current_odom_position(2));
  addDataPlot(_yaw_values, _yaw_min, _yaw_max, _current_yaw_orientation);

  ImGui::Spacing();
  ImGui::Separator();

  ImGui::NextColumn();
  std::string text = "Current State:";
  ImGui::SetCursorPosX(ImGui::GetCursorPosX() + ImGui::GetColumnWidth() - ImGui::CalcTextSize(text.c_str()).x );
  ImGui::Text(text.c_str());
  ImGui::Columns(4, "mycolumns"); // 4-ways, with border
  ImGui::Separator();
  ImGui::Text("x[m]"); ImGui::NextColumn();
  ImGui::Text("y[m]"); ImGui::NextColumn();
  ImGui::Text("z[m]"); ImGui::NextColumn();
  ImGui::Text("yaw[rad]"); ImGui::NextColumn();
  ImGui::Separator();
  ImGui::Text("%f", _current_odom_position(0)); ImGui::NextColumn();
  ImGui::Text("%f", _current_odom_position(1)); ImGui::NextColumn();
  ImGui::Text("%f", _current_odom_position(2)); ImGui::NextColumn();
  ImGui::Text("%f", _current_yaw_orientation);  ImGui::NextColumn();

  ImGui::PlotLines("",_x_values, IM_ARRAYSIZE(_x_values), 0,
                    "x", _x_min, _x_max, ImVec2(0,40)); ImGui::NextColumn();
  ImGui::PlotLines("",_y_values, IM_ARRAYSIZE(_y_values), 0,
                    "y", _y_min, _y_max, ImVec2(0,40)); ImGui::NextColumn();
  ImGui::PlotLines("",_z_values, IM_ARRAYSIZE(_z_values), 0,
                    "z", _z_min, _z_max, ImVec2(0,40)); ImGui::NextColumn();
  ImGui::PlotLines("",_yaw_values, IM_ARRAYSIZE(_yaw_values), 0,
                    "yaw", _yaw_min, _yaw_max, ImVec2(0,40));
  
  ImGui::Columns(1);
  ImGui::Spacing();
  ImGui::Separator();
  ImGui::Spacing();


  addDataPlot(_yaw_gimbal_axis_values, _yaw_gimbal_axis_min, _yaw_gimbal_axis_max, yaw_joint_value_);
  addDataPlot(_pitch_gimbal_axis_values, _pitch_gimbal_axis_min, _pitch_gimbal_axis_max, pitch_joint_value_);

  ImGui::NextColumn();
  text = "Current State:";
  ImGui::SetCursorPosX(ImGui::GetCursorPosX() + 2*ImGui::GetColumnWidth() - ImGui::CalcTextSize(text.c_str()).x );
  ImGui::Text(text.c_str());
  ImGui::Columns(3, "mycolumns"); // 4-ways, with border
  ImGui::Separator();
  ImGui::Text("yaw_axis [rad] %f", yaw_joint_value_); ImGui::NextColumn();
  ImGui::Text("pitch_axis [rad] %f ", pitch_joint_value_); ImGui::NextColumn();
  ImGui::Text(" gimbal axes commands "); ImGui::NextColumn();
  ImGui::Separator();
  ImGui::PlotLines("",_yaw_gimbal_axis_values, IM_ARRAYSIZE(_yaw_gimbal_axis_values), 0,
                    "x", _yaw_gimbal_axis_min, _yaw_gimbal_axis_max, ImVec2(0,40)); ImGui::NextColumn();
  ImGui::PlotLines("",_pitch_gimbal_axis_values, IM_ARRAYSIZE(_pitch_gimbal_axis_values), 0,
                    "y", _pitch_gimbal_axis_min, _pitch_gimbal_axis_max, ImVec2(0,40)); ImGui::NextColumn();
  ImGui::DragFloat("yaw command [rad]", &_des_gimbal_yaw_axis_, 0.01f, -M_PI/2, M_PI/2);
  ImGui::DragFloat("pitch command [rad]", &_des_gimbal_pitch_axis_, 0.01f, -M_PI/2, M_PI/2);
  if (ImGui::Button("send commands")) 
    pubGimbalCommands(_des_gimbal_pitch_axis_, _des_gimbal_yaw_axis_);
  
  ImGui::NextColumn();
  ImGui::Columns(1);
  

  ImGui::Spacing();
  ImGui::Separator();
  ImGui::Spacing();
  
  ImGui::Columns(2, "target_state");
  ImGui::Text("Desired Terget State");
  ImGui::DragFloat3("xt yt zt [meters]", _target_pos3f, 0.01f, -10.0f, 10.0f);
  ImGui::DragFloat3("vxt vyt vzt [meters/sec]", _target_vel3f, 0.01f, -10.0f, 10.0f);
  ImGui::DragFloat("time delay [sec]", _t_delay, 0.01f, 0.f, 0.5f);

  if( ImGui::Button("Set Target State") ) {
    command_sent = false;
    trigger_dyn_obst_1_request = true;
    integrateDynObjecet = false;
    std::cout << FGRN("Dyn Obstacle change position request received\n\n");
  }

  ImGui::NextColumn();
  ImGui::Text("Fixed Obstacles Position");
  ImGui::DragFloat2("x1 y1 [meters]", _vert_obst1_, 0.01f, -10.0f, 10.0f);
  ImGui::DragFloat2("x2 y2 [meters]", _vert_obst2_, 0.01f, -10.0f, 10.0f);
  ImGui::DragFloat2("x3 z3 [meters]", _horiz_obst_, 0.01f, -10.0f, 10.0f);
  ImGui::DragFloat2("x4 z4 [meters]", _horiz_obst2_, 0.01f, -10.0f, 10.0f);
  if(ImGui::Button("Change Fixed Obstacle Position")) changeGazeboFixedObstacleposition();

  ImGui::Columns(1); 
  checkAndChangeDynObstacle();

  if(integrateDynObjecet){
    dynObst_->Integrate(dt);
    setDynObstacleState();
  }

  ImGui::Spacing();
  ImGui::Separator();

  // Show Here Auxiliar GUIs
  if(_show_gazebo_gui) {
    showGazeboGUI(&_show_gazebo_gui);
  }
   
  // Turn the RGB pixel data into an OpenGL texture:
  glGenTextures(1, &my_opengl_texture);
  glBindTexture(GL_TEXTURE_2D, my_opengl_texture);
  glTexParameterf(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR);
  glTexParameterf(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);
  glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_WRAP_S , GL_REPEAT );
  glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT );
  glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);

  ImGui::Columns(2, "Current Image and UAV Avatar");
  ImGui::Text("Augmented Current Image");
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, draw_image_res_.cols, draw_image_res_.rows, 0, GL_RGB, GL_UNSIGNED_BYTE, draw_image_res_.data);
  ImGui::Image((void*)(intptr_t)my_opengl_texture, ImVec2(draw_image_res_.cols, draw_image_res_.rows));

  // Turn the RGB pixel data into an OpenGL texture:
  glGenTextures(1, &my_avatar_texture);
  glBindTexture(GL_TEXTURE_2D, my_avatar_texture);
  glTexParameterf(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR);
  glTexParameterf(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);
  glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_WRAP_S , GL_REPEAT );
  glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT );
  glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);

  ImGui::NextColumn();
  ImGui::Text("UAV Avatar");
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, avatarImg_res.cols, avatarImg_res.rows, 0, GL_RGB, GL_UNSIGNED_BYTE, avatarImg_res.data);
  ImGui::Image((void*)(intptr_t)my_avatar_texture, ImVec2(avatarImg_res.cols, avatarImg_res.rows));

  publishVizMarkers();
}

void MavGUI::pubGimbalCommands(const float& p_cmd, const float& y_cmd){

    std_msgs::Float64 pitch_cmd, yaw_cmd;
    yaw_cmd.data = y_cmd;
    pitch_cmd.data = p_cmd;

    sent_pitch_cmd = p_cmd;
    sent_yaw_cmd = y_cmd;

    command_gimbal_yaw_axis_.publish(yaw_cmd);
    command_gimbal_pitch_axis_.publish(pitch_cmd);

}

void MavGUI::publishVizMarkers(){
  
  
  visualization_msgs::Marker marker;
  marker.header.frame_id = "world";
  marker.header.stamp = ros::Time::now();
  marker.ns = "horizontal_obst_1";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::MODIFY;
  //marker.lifetime = ros::Duration();
  marker.pose.position.x = 1.5;
  marker.pose.position.y = 0.f;
  marker.pose.position.z = 2.6;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.15;
  marker.scale.y = 3;
  marker.scale.z = 0.15;
  marker.color.a = 1.0; 
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  _vis_pub.publish( marker );

  marker.header.stamp = ros::Time();
  marker.ns = "horizontal_obst_2";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::MODIFY;
  //marker.lifetime = ros::Duration();
  marker.pose.position.x = 1.5;
  marker.pose.position.y = 0.f;
  marker.pose.position.z = 3.7;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.15;
  marker.scale.y = 3;
  marker.scale.z = 0.15;
  marker.color.a = 1.0; 
  marker.color.r = 1.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  _vis_pub.publish( marker );

  marker.header.stamp = ros::Time();
  marker.ns = "vertical_obst_1";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::MODIFY;
  //marker.lifetime = ros::Duration();
  marker.pose.position.x = 1.f;
  marker.pose.position.y = 0.45;
  marker.pose.position.z = 2.f;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.15;
  marker.scale.y = 0.15;
  marker.scale.z = 4.f;
  marker.color.a = 1.0; 
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  _vis_pub.publish( marker );

  marker.header.stamp = ros::Time();
  marker.ns = "vertical_obst_2";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::MODIFY;
  //marker.lifetime = ros::Duration();
  marker.pose.position.x = 2.f;
  marker.pose.position.y = -.45;
  marker.pose.position.z = 2.f;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.15;
  marker.scale.y = 0.15;
  marker.scale.z = 4.f;
  marker.color.a = 1.0; 
  marker.color.r = 0.0;
  marker.color.g = 0.0;
  marker.color.b = 1.0;
  _vis_pub.publish( marker );

  marker.header.stamp = ros::Time();
  marker.ns = "dynamic_obstacle";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::MODIFY;
  //marker.lifetime = ros::Duration();
  marker.pose.position.x = dynObst_->getPose()(0);
  marker.pose.position.y = dynObst_->getPose()(1);
  marker.pose.position.z = dynObst_->getPose()(2);
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.2;
  marker.scale.y = 0.2;
  marker.scale.z = 0.2;
  marker.color.a = 1.0; 
  marker.color.r = 0.5;
  marker.color.g = 0.0;
  marker.color.b = 1.0;
  _vis_pub.publish( marker );
}

void MavGUI::changeGazeboFixedObstacleposition(){

  curr_pos_ = Eigen::Vector3d(_horiz_obst_[0] - 1.5, 0, _horiz_obst_[1] - 2.6);
  curr_att_ = Eigen::Vector3d(0, 0, 0);
  curr_q_ = Eigen::Quaterniond( mav_utils::fromEulerAngToRotMat(curr_att_) );

  gazebo_msgs::SetModelState setmodelstate;
  setmodelstate.request.model_state.model_name = "horizontal_obst_1";
  setmodelstate.request.model_state.pose.position = mav_utils::fromEigenVectorToPoint(curr_pos_);
  setmodelstate.request.model_state.pose.orientation = mav_utils::fromEigenQuaternionrToQuaternion(curr_q_);
  _set_mode_state.call(setmodelstate);

  curr_pos_ = Eigen::Vector3d(_horiz_obst2_[0] - 1.5, 0, _horiz_obst2_[1] - 3.7);
  setmodelstate.request.model_state.model_name = "horizontal_obst_2";
  setmodelstate.request.model_state.pose.position = mav_utils::fromEigenVectorToPoint(curr_pos_);
  _set_mode_state.call(setmodelstate);


  curr_pos_ = Eigen::Vector3d(_vert_obst1_[0] - 1, 0, _vert_obst1_[1] - 0.45);
  setmodelstate.request.model_state.model_name = "vertical_obst_1";
  setmodelstate.request.model_state.pose.position = mav_utils::fromEigenVectorToPoint(curr_pos_);
  _set_mode_state.call(setmodelstate);


  curr_pos_ = Eigen::Vector3d(_vert_obst2_[0] - 2, 0, _vert_obst2_[1] + 0.45);
  setmodelstate.request.model_state.model_name = "vertical_obst_2";
  setmodelstate.request.model_state.pose.position = mav_utils::fromEigenVectorToPoint(curr_pos_);
  _set_mode_state.call(setmodelstate);


  changeFixedObstaclePosition();
}

void MavGUI::checkAndChangeDynObstacle(){

    if(trigger_dyn_obst_1_request && command_sent){

      if(!start_time_taken){
            std::cout << FGRN("Target ") << "dynamic_obst" << FGRN(" Set to: ") << "\n"
                      << FGRN("position: ") << _target_pos3f[0] << " " << _target_pos3f[1] << " " << _target_pos3f[2] << "\n"
                      << FGRN("attitude: ") << _target_att3f[0] << " " << _target_att3f[1] << " " << _target_att3f[2] << "\n\n";
            start_time = ros::Time::now().toSec();
            start_time_taken = true;
      }

      float diff = ros::Time::now().toSec() - start_time;
      if(diff >= *_t_delay){
          dynObst_->setPoseandVelocity( Eigen::Vector3d( _target_pos3f[0], _target_pos3f[1], _target_pos3f[2] ),
                                        Eigen::Vector3d( _target_vel3f[0], _target_vel3f[1], _target_vel3f[2] ));
          setDynObstacleState();
          changeGazeboFixedObstacleposition();
          command_sent = false;
          trigger_dyn_obst_1_request = false;
          start_time_taken = false;
          integrateDynObjecet = true;
          std::cout << FGRN("Dynamic Obstacle moved!\n");
      }
    }
}

void MavGUI::setDynObstacleState(){

  curr_pos_ = Eigen::Vector3d( dynObst_->getPose()(0) + 4, dynObst_->getPose()(1), dynObst_->getPose()(2) );
  curr_att_ = Eigen::Vector3d(0.f, 0.f, 0.f);
  curr_q_ = Eigen::Quaterniond( mav_utils::fromEulerAngToRotMat(curr_att_) );

  gazebo_msgs::SetModelState setmodelstate;
  setmodelstate.request.model_state.model_name = "dynamic_obst_1";
  setmodelstate.request.model_state.pose.position = mav_utils::fromEigenVectorToPoint(curr_pos_);
  setmodelstate.request.model_state.pose.orientation = mav_utils::fromEigenQuaternionrToQuaternion(curr_q_);
  
  _set_mode_state.call(setmodelstate);
  changeDynObstaclePosition();
}
