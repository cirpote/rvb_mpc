#include "IBVS_random_controller_node.h"

using namespace std;

IBVSRandomNode::IBVSRandomNode(ros::NodeHandle& nh, const std::string& yaml_short_file, const std::string& gui_file)
  : MavGUI(nh, gui_file), nh_(nh), first_trajectory_cmd_(false), command_roll_pitch_yaw_thrust_st_(0,0,0,0), 
    command_roll_pitch_yaw_thrust_lt_(0,0,0,0), stnl_controller(yaml_short_file)
{

  joint_states_sub_ = nh_.subscribe("/firefly/joint_states", 1, &IBVSRandomNode::JointStateCallback, this, ros::TransportHints().tcpNoDelay() );
  odom_sub_ = nh_.subscribe( "/firefly/ground_truth/odometry", 1, &IBVSRandomNode::OdometryCallback, this, ros::TransportHints().tcpNoDelay() );
  cmd_pose_sub_ = nh_.subscribe("/command/pose", 1, &IBVSRandomNode::CommandPoseCallback, this, ros::TransportHints().tcpNoDelay() );
  command_roll_pitch_yawrate_thrust_pub_ = nh_.advertise<mav_msgs::RollPitchYawrateThrust>("/command/roll_pitch_yawrate_thrust", 1);

  std::cerr << "\n" << FBLU("Initializing short term Controller from:") << " " << yaml_short_file << "\n";
  stnl_controller.InitializeController();
  this->init3DObjRendering( ros::package::getPath("rvb_mpc") );

  int iter = 0;
  while(true){
    if (utils::exists( ros::package::getPath("rvb_mpc") + "/log_output_folder/" + "log_output_" + to_string(iter) + ".txt" ) ){
        iter++;
    } else {
      logFileStream.open( ros::package::getPath("rvb_mpc") + "/log_output_folder/" + "log_output_" + to_string(iter) + ".txt" );
      break;
    }
  }

}

IBVSRandomNode::~IBVSRandomNode(){
  logFileStream.close();
}

void IBVSRandomNode::JointStateCallback(const sensor_msgs::JointState & joint_state_msg){

  for(unsigned int iter = 0; iter < joint_state_msg.name.size(); ++iter){
    
    if( !strcmp( joint_state_msg.name[iter].c_str(), "gimbal_p_joint_" ) )
      pitch_joint_value_ = joint_state_msg.position[iter];

    if( !strcmp( joint_state_msg.name[iter].c_str(), "gimbal_y_joint_" ) )
      yaw_joint_value_ = joint_state_msg.position[iter];
  }

}

void IBVSRandomNode::changeFixedObstaclePosition(){

    for (size_t i = 0; i < ACADO_N; ++i) {
      acadoVariables.od[ACADO_NOD * i + 15] = _vert_obst1_[0];
      acadoVariables.od[ACADO_NOD * i + 16] = _vert_obst1_[1];
      acadoVariables.od[ACADO_NOD * i + 19] = _vert_obst2_[0];
      acadoVariables.od[ACADO_NOD * i + 20] = _vert_obst2_[1];
      acadoVariables.od[ACADO_NOD * i + 23] = _horiz_obst_[0];
      acadoVariables.od[ACADO_NOD * i + 24] = _horiz_obst_[1];
      acadoVariables.od[ACADO_NOD * i + 33] = _horiz_obst2_[0];
      acadoVariables.od[ACADO_NOD * i + 34] = _horiz_obst2_[1];
    }

    std::cout << FBLU("First vertical obstacle (x,y) Position: ") << acadoVariables.od[15] << " " << acadoVariables.od[16] << "\n";
    std::cout << FBLU("Second vertical obstacle (x,y) Position: ") << acadoVariables.od[19] << " " << acadoVariables.od[20] << "\n";
    std::cout << FBLU("First Horizontal obstacle (x,z) position: ") << acadoVariables.od[23] << " " << acadoVariables.od[24] << "\n";
    std::cout << FBLU("Second Horizontal obstacle (x,z) position: ") << acadoVariables.od[33] << " " << acadoVariables.od[34] << "\n\n";
}

void IBVSRandomNode::changeDynObstaclePosition(){

    for (int i = 0; i < ACADO_N + 1; i++) {
      acadoVariables.od[ACADO_NOD * i + 27] = dynObst_->getPose()(0);
      acadoVariables.od[ACADO_NOD * i + 28] = dynObst_->getPose()(1);
      acadoVariables.od[ACADO_NOD * i + 29] = dynObst_->getPose()(2);
    }

    if( dynObst_->getVel().norm() > 1e-2 ){
      for (int i = 0; i < ACADO_N + 1; i++) {
        acadoVariables.od[ACADO_NOD * i + 30] =  (_target_vel3f[0] > 1e-1) ? 1 / (dynObst_->getVel()(0) * 20) : 1;
        acadoVariables.od[ACADO_NOD * i + 31] =  (_target_vel3f[1] > 1e-1) ? 1 / (dynObst_->getVel()(1) * 20) : 1;
        acadoVariables.od[ACADO_NOD * i + 32] =  (_target_vel3f[2] > 1e-1) ? 1 / (dynObst_->getVel()(2) * 20) : 1;
      }
    }

}

void IBVSRandomNode::writeLogData(){

  logFileStream << ros::Time::now().toSec() << " " << stnl_controller.odometry.position_W.x() << " " << stnl_controller.odometry.position_W.y() << " " << stnl_controller.odometry.position_W.z() << " " <<
                   stnl_controller.odometry.orientation_W_B.w() << " " << stnl_controller.odometry.orientation_W_B.x() << " " << stnl_controller.odometry.orientation_W_B.y() << " " << stnl_controller.odometry.orientation_W_B.z() << " " <<
                   command_roll_pitch_yawrate_thrust_msg.roll << " " << command_roll_pitch_yawrate_thrust_msg.pitch << " " << command_roll_pitch_yawrate_thrust_msg.yaw_rate << " " <<  command_roll_pitch_yawrate_thrust_msg.thrust.z << " " <<
                   trajectory_point.position_W.x() << " " << trajectory_point.position_W.y() << " " << trajectory_point.position_W.z() << " " << trajectory_point.getYaw() << " " <<
                   _target_pos3f[0] << " " << _target_pos3f[1] << " " << _target_pos3f[2] << " " << _target_vel3f[0] << " " << _target_vel3f[1] << " " << _target_vel3f[2] << " " << *_t_delay << " " <<
                   _vert_obst1_[0] << " " << _vert_obst1_[1] << " " << _vert_obst2_[0] << " " << _vert_obst2_[1] << " " << _horiz_obst_[0] << " " << _horiz_obst_[1] << " " <<
                   stnl_controller.pT_W_.x() << " " << stnl_controller.pT_W_.y() << " " << stnl_controller.pT_W_.z() << " " << stnl_controller.camera_instrinsics_.x() << " " << stnl_controller.camera_instrinsics_.y() <<  " " <<
                   stnl_controller.iter << " " << stnl_controller.solve_time << "\n";
}

void IBVSRandomNode::CommandPoseCallback(const nav_msgs::OdometryConstPtr& cmd_pose_msg)
{
  ROS_INFO_ONCE("Optimal IBVS controller got first command message.");
  eigenOdometryFromMsg(*cmd_pose_msg, &trajectory_point);
  stnl_controller.setCommandPose(*cmd_pose_msg);
  
  if(first_trajectory_cmd_){
    new_comand = true;
    return;
  }

  startTime = ros::Time::now().toSec();
  first_trajectory_cmd_ = true;  

  return;
}

void IBVSRandomNode::OdometryCallback(const nav_msgs::OdometryConstPtr& odom_msg)
{
  
  ROS_INFO_ONCE("Optimal IBVS controller got first odometry message.");

  // Check Is the first trajectory msg has been received
  if(!first_trajectory_cmd_)
    return;

  stnl_controller.setOdometry(odom_msg);
  _current_orientation = mav_utils::quaternionFromMsg(odom_msg->pose.pose.orientation); 
  _current_yaw_orientation = mav_utils::yawFromQuaternion(_current_orientation);
  _current_odom_position = mav_utils::vector3FromPointMsg(odom_msg->pose.pose.position);   

  stnl_controller.calculateRollPitchYawRateThrustCommands(command_roll_pitch_yaw_thrust_st_);

  nav_msgs::Path path;
  path.header.frame_id = "/world";
  for(unsigned int iter = 0; iter < ACADO_N; ++iter){
    geometry_msgs::PoseStamped curr_pt;
    curr_pt.pose.position.x = acadoVariables.x[iter * ACADO_NX];
    curr_pt.pose.position.y = acadoVariables.x[iter * ACADO_NX + 1];
    curr_pt.pose.position.z = acadoVariables.x[iter * ACADO_NX + 2];
    curr_pt.pose.orientation.w = 1;
    curr_pt.pose.orientation.x = 0;
    curr_pt.pose.orientation.y = 0;
    curr_pt.pose.orientation.z = 0;
    curr_pt.header.frame_id = "/world";
    path.poses.push_back(curr_pt);
  }
  _path_pub.publish(path);

  if(new_comand && randomSpawnDynObj){
    new_comand = false;
    
    Eigen::Vector3d tracjPt( acadoVariables.x[ ACADO_N * ACADO_NX ], 
                             acadoVariables.x[ ACADO_N * ACADO_NX + 1 ], 
                             acadoVariables.x[ ACADO_N * ACADO_NX + 2 ]);

    Eigen::Vector3d spawningPt, spawningVel;

    if( dynObjSpawner.computeDynamicObstacleSpawningPosition( stnl_controller.trajectory_point.position_W,
                                                              _t_delay,
                                                              tracjPt,
                                                              spawningPt,
                                                              spawningVel ) ){

      std::cout << FRED("Dynamic Object spawn request sent!\n");

      _target_pos3f[0] = spawningPt.x();
      _target_pos3f[1] = spawningPt.y();
      _target_pos3f[2] = spawningPt.z();

      _target_vel3f[0] = spawningVel.x();
      _target_vel3f[1] = spawningVel.y();
      _target_vel3f[2] = spawningVel.z();

      trigger_dyn_obst_1_request = true;
    }
  }

  command_roll_pitch_yawrate_thrust_msg.header = odom_msg->header;
  command_roll_pitch_yawrate_thrust_msg.roll = command_roll_pitch_yaw_thrust_st_(0);
  command_roll_pitch_yawrate_thrust_msg.pitch = command_roll_pitch_yaw_thrust_st_(1);
  command_roll_pitch_yawrate_thrust_msg.yaw_rate = command_roll_pitch_yaw_thrust_st_(2);
  command_roll_pitch_yawrate_thrust_msg.thrust.z = command_roll_pitch_yaw_thrust_st_(3)*stnl_controller.getMass();
    
  command_roll_pitch_yawrate_thrust_pub_.publish(command_roll_pitch_yawrate_thrust_msg); 

  if( ( ros::Time::now().toSec() - startTime ) > GenerationNum*10.f && randomWaypointGeneration ){

    GenerationNum++;
    Eigen::Vector2f rand_xy_cmd(Eigen::Vector2f::Zero());
    bool generationSucceded = false;
    
    while(!generationSucceded){
      dynObjSpawner.computeRandomXYCommand(rand_xy_cmd);
      nav_msgs::Odometry random_cmd;
      random_cmd.pose.pose.position.x = rand_xy_cmd.x();
      random_cmd.pose.pose.position.y = rand_xy_cmd.y();
      generationSucceded = stnl_controller.setCommandPose(random_cmd);
    }
    command_sent = true;
    new_comand = true;
    std::cout << FGRN("Generating new trajectory comand: ") << GenerationNum << " " << ros::Time::now().toSec() << "\n";
  }

  writeLogData();
  return;
}


static void error_callback(int error, const char* description)
{
  fprintf(stderr, "Error %d: %s\n", error, description);
}

int main(int argc, char** argv)
{

  if(argc < 5) {
        std::cerr << FRED("Other Params Expected!") << " node_name <params_short_term_file> <gui_params> <waypoint_generation> <dyn_obst_spawn>" << "\n";
        std::exit(1);
  }

  std::string yaml_short_filename = argv[1];
  std::string gui_filename = argv[2];
  std::string waypoint_generation = argv[3];
  std::string dynamic_obstacle_spawning = argv[4];

  if(!strcmp(waypoint_generation.c_str(), "true") ){
    randomWaypointGeneration = true;
    std::cout << FGRN(" Random Waypoint Generation ON ") << "\n";
  } else {
    randomWaypointGeneration = false;
    std::cout << FGRN(" Random Waypoint Generation OFF ") << "\n";
  }

    if(!strcmp(dynamic_obstacle_spawning.c_str(), "true") ){
    randomSpawnDynObj = true;
    std::cout << FGRN(" Random Dynamic Object Spawning ON ") << "\n";
  } else {
    randomSpawnDynObj = false;
    std::cout << FGRN(" Random Dynamic Object Spawning OFF ") << "\n";
  }

  // Setup window
  glfwSetErrorCallback(error_callback);
  if (!glfwInit())
    return 1;
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
  #if __APPLE__
  glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
  #endif
  GLFWwindow* window = glfwCreateWindow(1000, 1000, "mav_gui_app", NULL, NULL);

  glfwMakeContextCurrent(window);
  glfwSwapInterval(1); // Enable vsync
  gl3wInit();

  // Setup ImGui binding
  ImGui_ImplGlfwGL3_Init(window, true);

  bool show_test_window = false;
  ImVec4 clear_color = ImColor(114, 144, 154);

  glEnable(GL_DEPTH_TEST);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  if(argc < 2) {
        std::cerr << FRED("Other Params Expected!") << " node_name <params_file.txt>" << "\n";
        std::exit(1);
  }

  ros::init(argc, argv, "mav_gnomic_gui_node");
  ros::NodeHandle nh("~");

  IBVSRandomNode IBVS_node(nh, yaml_short_filename, gui_filename);
  bool show_gnomic_GUI = true;

  // Main loop
  while (ros::ok()){
    glfwPollEvents();
    ImGui_ImplGlfwGL3_NewFrame();

    if (show_test_window) {
      ImGui::SetNextWindowPos(ImVec2(650, 20), ImGuiCond_FirstUseEver);
      ImGui::ShowTestWindow(&show_test_window);
    }

    if(show_gnomic_GUI) {
      ImGui::SetNextWindowPos(ImVec2(650,650), ImGuiCond_FirstUseEver);
      IBVS_node.showGUI(&show_gnomic_GUI);
    }

    // Rendering
    int display_w, display_h;
    glfwGetFramebufferSize(window, &display_w, &display_h);
    glViewport(0, 0, display_w, display_h);
    glClearColor(clear_color.x, clear_color.y, clear_color.z, clear_color.w);
    glClear(GL_COLOR_BUFFER_BIT);
    ImGui::Render();
    glfwSwapBuffers(window);

    ros::spinOnce();
  }

  // Cleanup
  ImGui_ImplGlfwGL3_Shutdown();
  glfwTerminate();

  return 0;
}
