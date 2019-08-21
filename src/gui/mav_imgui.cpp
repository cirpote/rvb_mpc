#include "mav_imgui.h"

MavGUI::MavGUI(ros::NodeHandle nh, const std::string& yaml_file) : BaseGUI(nh) {

  _des_pos_vec3f_t[0] = 0.f;
  _des_pos_vec3f_t[1] = 0.f;
  _des_orientationf_t = 0.f;

  _des_pos_vec3f_w[0] = 0.f;
  _des_pos_vec3f_w[1] = 0.f;
  _des_orientationf_w = 0.f;

  _des_pos[0] = 0.f;
  _des_pos[1] = 0.f;
  _des_orientation_w = 0.f;

  _gui_ros_time = ros::Time::now();

  _img_sub = _base_nh.subscribe("/firefly/vi_sensor/right/image_raw", 1, &MavGUI::imageCb, this, ros::TransportHints().tcpNoDelay());
  _set_mode_state = _base_nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");

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
  //strcat(model_path, "/src/assimp_loader/assets/siege_engine/siegeEngine.obj");
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

void MavGUI::imageCb(const sensor_msgs::ImageConstPtr& img_msg){


  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);

  currImg_ = cv_ptr->image.clone();
  cv::cvtColor(currImg_, draw_image_, cv::COLOR_GRAY2BGR);

  cv::resize(draw_image_, draw_image_res_, cv::Size( draw_image_.cols/2, draw_image_.rows/2) );

}


void MavGUI::updateDesiredState() {

  nav_msgs::Odometry cmd_msg;
  cmd_msg.header.stamp = ros::Time::now();
  cmd_msg.pose.pose.position.x = _des_pos_vec3f_t[0];
  cmd_msg.pose.pose.position.y = _des_pos_vec3f_t[1];
  cmd_msg.pose.pose.position.z = 0;

  Eigen::Quaterniond q( Eigen::AngleAxisd(_des_orientationf_t, Eigen::Vector3d::UnitZ() ) );
  cmd_msg.pose.pose.orientation = utils::fromEigenQuaternionrToQuaternion(q);
  _cmd_pub.publish( cmd_msg );
}

void MavGUI::sendWaypoint() {

  geometry_msgs::Point pt_msg;
  pt_msg.x = _des_pos[0];
  pt_msg.y = _des_pos[1];
  pt_msg.z = _des_orientation_w;
  _waypoint_pub.publish(pt_msg);
}

void MavGUI::activatePublisher(const std::string &cmd_publisher_name, const std::string &waypoint_publisher_name) {
  _cmd_pub = _base_nh.advertise<nav_msgs::Odometry>(cmd_publisher_name, 1, this);
  _waypoint_pub = _base_nh.advertise<geometry_msgs::Point>(waypoint_publisher_name, 1, this);
}

void MavGUI::showGUI(bool *p_open) {

  //processAvatar();
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
  static char cmd_pub[64] = "/command/pose";
  static char waypoint_pub[64] = "/waypoint";
  ImGui::InputText("cmd_pub", cmd_pub, 64);
  ImGui::InputText("waypoint_pub", waypoint_pub, 64);
  if (ImGui::Button("Activate Publisher")) {
    std::cerr << "activating publishers: " << cmd_pub << " " << waypoint_pub << "\n";
    activatePublisher(cmd_pub, waypoint_pub);
  }

  ImGui::Spacing();
  ImGui::Separator();
  ImGui::Columns(2, "state_time");
  ImGui::Text("Desired State (Trajectory)");
  ImGui::DragFloat2("x y [meters]", _des_pos_vec3f_t, 0.01f, -20.0f, 20.0f);
  ImGui::DragFloat("yaw [radians]", &_des_orientationf_t, 0.01f, -M_PI, M_PI);
  if (ImGui::Button("Update Desired State")) 
    updateDesiredState();
  


  ImGui::NextColumn();
  ImGui::Text("Desired State (Waypoint)");
  ImGui::DragFloat2("x y [meters] ", _des_pos_vec3f_w, 0.01f, -20.0f, 200.0f);
  ImGui::DragFloat("yaw [radians] ", &_des_orientationf_w, 0.01f, -M_PI, M_PI);
  if (ImGui::Button("Send Waypoint")){
    _des_pos[0] = _des_pos_vec3f_w[0];
    _des_pos[1] = _des_pos_vec3f_w[1];
    _des_orientation_w = _des_orientationf_w;
  }
    
  sendWaypoint();
  

  ImGui::Spacing();
  ImGui::Separator();
  ImGui::Columns(1); // 4-ways, with border
  ImGui::Text("Current State:");
  ImGui::Columns(4, "mycolumns"); // 4-ways, with border
  //add new data for plot and update min_max
  addDataPlot(_x_values, _x_min, _x_max, _current_odom_position(0));
  addDataPlot(_y_values, _y_min, _y_max, _current_odom_position(1));
  addDataPlot(_z_values, _z_min, _z_max, _current_odom_position(2));
  addDataPlot(_yaw_values, _yaw_min, _yaw_max, _current_yaw_orientation);
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

}