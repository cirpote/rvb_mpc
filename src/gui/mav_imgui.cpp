#include "mav_imgui.h"

MavGUI::MavGUI(ros::NodeHandle nh, const std::string& yaml_file) : BaseGUI(nh), Obst1_(0,0,2) ,
                                                                   Obst2_(0,0,2), Obst3_(0,0,2) ,
                                                                   Obst4_(0,0,2) , Obst5_(0,0,2) , Obst6_(0,0,2) {

  _des_pos_vec3f_t[0] = 0.f;
  _des_pos_vec3f_t[1] = 0.f;
  _des_orientationf_t = 0.f;

  _des_pos_vec3f_w[0] = 0.f;
  _des_pos_vec3f_w[1] = 0.f;
  _des_orientationf_w = 0.f;

  _K_values[0] = 1.f;
  _K_values[1] = 6.f;
  _K_values[2] = 3.f;

  _dyn_obst_vec2f[0] = 0.f;
  _dyn_obst_vec2f[1] = 0.f;

  _gui_ros_time = ros::Time::now();

  _img_sub = _base_nh.subscribe("/camera_gui/camera/image_raw", 1, &MavGUI::imageCb, this, ros::TransportHints().tcpNoDelay());
  _set_control_gains = _base_nh.serviceClient<rm3_ackermann_controller::SetKvalues>("/set_k");
  _activate_controller = _base_nh.serviceClient<rm3_ackermann_controller::ActivateController>("/activate_controller");
  _set_model_state = _base_nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
  _vis_pub = _base_nh.advertise<visualization_msgs::Marker>( "/visualization_marker", 1 );
  _path_pub = _base_nh.advertise<nav_msgs::Path>( "/path", 1);

  camera = std::make_shared<Camera>( glm::vec3(15.f, 20.f, -65.0f), glm::vec3(0.0f, 1.0f, 0.0f), 100.f );
  std::cout << FGRN("Camera Correctly Initialized\n\n");
}

void MavGUI::drawMarkerRViz(const Eigen::Vector3f& p, const std::string& ns){

  visualization_msgs::Marker marker;
  marker.header.stamp = ros::Time();
  marker.header.frame_id = "/odom";
  marker.ns = ns;
  marker.id = 0;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.lifetime = ros::Duration();
  marker.pose.position.x = p(0);
  marker.pose.position.y = p(1);
  marker.pose.position.z = p(2);
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

}

void MavGUI::imageCb(const sensor_msgs::ImageConstPtr& img_msg){


  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);

  draw_image_ = cv_ptr->image.clone();

  float k = 476.70308;
  float c = 400.5;
  int size = 800;

  int N = trajectory_pts_.points[0].positions.size()/3;
  std::vector<Eigen::Vector2f> pt( N, Eigen::Vector2f(0,0) );

  for(unsigned int iter = 0; iter < N; ++iter){
    pt[iter] = Eigen::Vector2f( - trajectory_pts_.points[0].positions[iter * 3 + 1], - trajectory_pts_.points[0].positions[iter * 3] ) - 
               Eigen::Vector2f( - _current_odom_position(1), - _current_odom_position(0));

    pt[iter] = k*pt[iter]/15 + Eigen::Vector2f(c,c); 

    cv::circle(draw_image_, cv::Point2i(pt[iter](0), pt[iter](1)), 5, cv::Scalar(255,0,0), 3);
  }

  for(unsigned int iter = 0; iter < N - 1; ++iter)
    cv::line(draw_image_, cv::Point2i(pt[iter](0), pt[iter](1)), cv::Point2i(pt[iter+1](0), pt[iter+1](1)), cv::Scalar(0,0,255),3);

  //cv::imshow("ciao", draw_image_);
  //cv::waitKey(10);

  cv::resize(draw_image_, draw_image_res_, cv::Size( draw_image_.cols/2, draw_image_.rows/2) );
  cv::cvtColor(draw_image_res_, draw_image_res_, cv::COLOR_BGR2RGB);

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
  pt_msg.x = _des_pos_vec3f_w[0];
  pt_msg.y = _des_pos_vec3f_w[1];
  pt_msg.z = _des_orientationf_w;
  if(_sendingWaypoint)
    _waypoint_pub.publish(pt_msg);
}

void MavGUI::activatePublisher(const std::string &cmd_publisher_name, const std::string &waypoint_publisher_name) {
  _cmd_pub = _base_nh.advertise<nav_msgs::Odometry>(cmd_publisher_name, 1, this);
  _waypoint_pub = _base_nh.advertise<geometry_msgs::Point>(waypoint_publisher_name, 1, this);
}

void MavGUI::activateController(){

  rm3_ackermann_controller::ActivateController srvCall;
  srvCall.request.is_active = true;
  _activate_controller.call(srvCall);

  std::cout << FBLU("MavGUI: ") << srvCall.response.result << "\n";

}

void MavGUI::disactivateController(){
  
  rm3_ackermann_controller::ActivateController srvCall;
  srvCall.request.is_active = false;
  _activate_controller.call(srvCall);

  std::cout << FBLU("MavGUI: ") << srvCall.response.result << "\n";
}

void MavGUI::showGUI(bool *p_open) {

  ImGuiWindowFlags window_flags = 0;
  window_flags |= ImGuiWindowFlags_MenuBar;
 
  ImGui::SetNextWindowSize(ImVec2(700, 800), ImGuiCond_FirstUseEver);
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
  if (ImGui::Button("Start Sending"))
    _sendingWaypoint = true;
  ImGui::SameLine();
  if (ImGui::Button("Stop Sending"))
    _sendingWaypoint = false;
    
  sendWaypoint();
  
  // Plotting Telemetry
  ImGui::Spacing();
  ImGui::Separator();
  ImGui::Columns(1); // 4-ways, with border
  ImGui::Text("Current State:");
  ImGui::Columns(4, "mycolumns"); // 4-ways, with border
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

  ImGui::PlotLinesWithTarget("",_x_values, IM_ARRAYSIZE(_x_values), _des_pos_vec3f_w[0],
                             0,"x", FLT_MAX, FLT_MAX, ImVec2(0,40)); ImGui::NextColumn();
  ImGui::PlotLinesWithTarget("",_y_values, IM_ARRAYSIZE(_y_values), _des_pos_vec3f_w[1],
                             0, "y", FLT_MAX, FLT_MAX, ImVec2(0,40)); ImGui::NextColumn();
  ImGui::PlotLinesWithTarget("",_z_values, IM_ARRAYSIZE(_z_values), 0,
                             0, "z", _z_min, _z_max, ImVec2(0,40)); ImGui::NextColumn();
  ImGui::PlotLinesWithTarget("",_yaw_values, IM_ARRAYSIZE(_yaw_values), _des_orientationf_w,
                             0, "yaw", FLT_MAX, FLT_MAX, ImVec2(0,40));

  // Plotting Ackermann Commands and Lyapunov Cost Function
  ImGui::Spacing();
  ImGui::Separator();
  ImGui::Columns(1); // 4-ways, with border
  ImGui::Text("Commands and Lyapunov Cost Function:");
  ImGui::Columns(3, "mycolumns"); 
  addDataPlot(_v_values, _v_min, _v_max, _v);
  addDataPlot(_phi_values, _phi_min, _phi_max, _phi);
  addDataPlot(_lyapunov_values, _lyapunov_min, _lyapunov_max, _lyapunov_cost);
  ImGui::Separator();
  ImGui::Text("v[m/s]"); ImGui::NextColumn();
  ImGui::Text("phi[rad]"); ImGui::NextColumn();
  ImGui::Text("Lyapunov"); ImGui::NextColumn();
  ImGui::Separator();
  ImGui::Text("%f", _v); ImGui::NextColumn();
  ImGui::Text("%f", _phi); ImGui::NextColumn();
  ImGui::Text("%f", _lyapunov_cost); ImGui::NextColumn();

  // ImGui::PlotHistogram("Histogram", _v_values, IM_ARRAYSIZE(_v_values), 0, NULL, _v_min, _v_max, ImVec2(0,80)); ImGui::NextColumn();

  ImGui::PlotLinesSaturation("",_v_values, IM_ARRAYSIZE(_v_values), _v_sup, _v_sdown,
                    0, "v", FLT_MAX, FLT_MAX, ImVec2(0,40)); ImGui::NextColumn();
  ImGui::PlotLinesSaturation("",_phi_values, IM_ARRAYSIZE(_phi_values), _phi_sup, _phi_sdown, 
                    0, "phi", FLT_MAX, FLT_MAX, ImVec2(0,40)); ImGui::NextColumn();
  ImGui::PlotLines("",_lyapunov_values, IM_ARRAYSIZE(_lyapunov_values), 0,
                    "lyapunov", _lyapunov_min, _lyapunov_max, ImVec2(0,40)); ImGui::NextColumn();





  ImGui::Columns(1);
  ImGui::Spacing();
  ImGui::Separator();

  // Show Here Auxiliar GUIs
  if(_show_gazebo_gui) {
    showGazeboGUI(&_show_gazebo_gui);
  }
   
  // Turn the RGB pixel data into an OpenGL texture:
  glDeleteTextures(1, &my_opengl_texture);
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

  ImGui::NextColumn();
  ImGui::Text("Control law gains");
  ImGui::Text("0.2 0.4 3.5 Pose Regulation");
  ImGui::Text("0.5 2 3 Traj. Tracking");
  ImGui::DragFloat3(" K1 K2 K3 ", _K_values, 0.01f, -20.0f, 200.0f);
  if (ImGui::Button("Send gains"))
    changeControlLawGains();

  
  ImGui::Spacing();
  ImGui::Text("Ackermann Controller");
  if (ImGui::Button("Activate"))
    activateController();
  ImGui::SameLine();
  if (ImGui::Button("Disactivate"))
    disactivateController();



  ImGui::Spacing();
  ImGui::Text("Dynamic Obstacle");
  ImGui::DragFloat2(" x y ", _dyn_obst_vec2f, 0.01f, -20.0f, 20.0f);
  if (ImGui::Button("set Dyn. Obstacle"))
    setDynamicObstacle();  

  ImGui::Spacing();
  ImGui::Text("Read Static Obstacles");
  if (ImGui::Button("get Static Obstacle"))
    getStaticObstacle();  

  drawMarkerRViz(Obst1_,"hazelnut_tree_1"); 
  drawMarkerRViz(Obst2_,"hazelnut_tree_2");
  drawMarkerRViz(Obst3_,"hazelnut_tree_3");
  drawMarkerRViz(Obst4_,"hazelnut_tree_4");
  drawMarkerRViz(Obst5_,"hazelnut_tree_5");
  drawMarkerRViz(Obst6_,"hazelnut_tree_6");

}



void MavGUI::changeControlLawGains(){

  rm3_ackermann_controller::SetKvalues srvCall;
  srvCall.request.k1 = _K_values[0];
  srvCall.request.k2 = _K_values[1];
  srvCall.request.k3 = _K_values[2];
  _set_control_gains.call(srvCall);

  std::cout << FBLU("MavGUI: ") << srvCall.response.result << "\n";

}