#include "IBVS_controller_node.h"

using namespace std;

IBVSNode::IBVSNode(ros::NodeHandle& nh, const std::string& yaml_short_file, const std::string& yaml_long_file, const std::string& gui_file) 
  : MavGUI(nh, gui_file), nh_(nh), first_trajectory_cmd_(false), command_roll_pitch_yaw_thrust_st_(0,0,0,0), 
    command_roll_pitch_yaw_thrust_lt_(0,0,0,0), stnl_controller(yaml_short_file)//, ltnl_controller(yaml_long_file) 
{

  odom_sub_ = nh_.subscribe( "/firefly/ground_truth/odometry", 1, &IBVSNode::OdometryCallback, this, ros::TransportHints().tcpNoDelay() );
  cmd_pose_sub_ = nh_.subscribe("/command/pose", 1, &IBVSNode::CommandPoseCallback, this, ros::TransportHints().tcpNoDelay() );
  command_roll_pitch_yawrate_thrust_pub_ = nh_.advertise<mav_msgs::RollPitchYawrateThrust>("/command/roll_pitch_yawrate_thrust", 1);

  std::cerr << "\n" << FBLU("Initializing short term Controller from:") << " " << yaml_short_file << "\n";
  stnl_controller.InitializeController();

}

IBVSNode::~IBVSNode(){}

void IBVSNode::changeFixedObstaclePosition(){

    for (size_t i = 0; i < ACADO_N; ++i) {
      shortTermAcadoVariables.od[ACADO_NOD * i + 15] = _vert_obst1_[0];
      shortTermAcadoVariables.od[ACADO_NOD * i + 16] = _vert_obst1_[1];
      shortTermAcadoVariables.od[ACADO_NOD * i + 19] = _vert_obst2_[0];
      shortTermAcadoVariables.od[ACADO_NOD * i + 20] = _vert_obst2_[1];
      shortTermAcadoVariables.od[ACADO_NOD * i + 23] = _horiz_obst_[0];
      shortTermAcadoVariables.od[ACADO_NOD * i + 24] = _horiz_obst_[1];
      shortTermAcadoVariables.od[ACADO_NOD * i + 33] = _horiz_obst2_[0];
      shortTermAcadoVariables.od[ACADO_NOD * i + 34] = _horiz_obst2_[1];
    }

    std::cout << FBLU("First vertical obstacle (x,y) Position: ") << shortTermAcadoVariables.od[15] << " " << shortTermAcadoVariables.od[16] << "\n";
    std::cout << FBLU("Second vertical obstacle (x,y) Position: ") << shortTermAcadoVariables.od[19] << " " << shortTermAcadoVariables.od[20] << "\n";
    std::cout << FBLU("First Horizontal obstacle (x,z) position: ") << shortTermAcadoVariables.od[23] << " " << shortTermAcadoVariables.od[24] << "\n";
    std::cout << FBLU("Second Horizontal obstacle (x,z) position: ") << shortTermAcadoVariables.od[33] << " " << shortTermAcadoVariables.od[34] << "\n\n";
}

void IBVSNode::changeDynObstaclePosition(){

    for (int i = 0; i < ACADO_N + 1; i++) {
      shortTermAcadoVariables.od[ACADO_NOD * i + 27] = dynObst_->getPose()(0);
      shortTermAcadoVariables.od[ACADO_NOD * i + 28] = dynObst_->getPose()(1);
      shortTermAcadoVariables.od[ACADO_NOD * i + 29] = dynObst_->getPose()(2);
    }

    if( dynObst_->getVel().norm() > 1e-2 ){
      for (int i = 0; i < ACADO_N + 1; i++) {
        shortTermAcadoVariables.od[ACADO_NOD * i + 30] =  (_target_vel3f[0] > 1e-1) ? 1 / (dynObst_->getVel()(0) * 20) : 1;
        shortTermAcadoVariables.od[ACADO_NOD * i + 31] =  (_target_vel3f[1] > 1e-1) ? 1 / (dynObst_->getVel()(1) * 20) : 1;
        shortTermAcadoVariables.od[ACADO_NOD * i + 32] =  (_target_vel3f[2] > 1e-1) ? 1 / (dynObst_->getVel()(2) * 20) : 1;
      }
    }

}

void IBVSNode::CommandPoseCallback(const nav_msgs::OdometryConstPtr& cmd_pose_msg)
{
  ROS_INFO_ONCE("Optimal IBVS controller got first command message.");
  eigenOdometryFromMsg(*cmd_pose_msg, &trajectory_point);
  stnl_controller.setCommandPose(*cmd_pose_msg);
  first_trajectory_cmd_ = true;  
  return;
}

void IBVSNode::OdometryCallback(const nav_msgs::OdometryConstPtr& odom_msg)
{
  
  ROS_INFO_ONCE("Optimal IBVS controller got first odometry message.");

  stnl_controller.setOdometry(odom_msg);
  _current_orientation = mav_utils::quaternionFromMsg(odom_msg->pose.pose.orientation); 
  _current_yaw_orientation = mav_utils::yawFromQuaternion(_current_orientation);
  _current_odom_position = mav_utils::vector3FromPointMsg(odom_msg->pose.pose.position);   

  // Check Is the first trajectory msg has been received
  if(!first_trajectory_cmd_)
    return;

  stnl_controller.calculateRollPitchYawRateThrustCommands(command_roll_pitch_yaw_thrust_st_);

  mav_msgs::RollPitchYawrateThrust command_roll_pitch_yawrate_thrust_msg;
  command_roll_pitch_yawrate_thrust_msg.header = odom_msg->header;
  command_roll_pitch_yawrate_thrust_msg.roll = command_roll_pitch_yaw_thrust_st_(0);
  command_roll_pitch_yawrate_thrust_msg.pitch = command_roll_pitch_yaw_thrust_st_(1);
  command_roll_pitch_yawrate_thrust_msg.yaw_rate = command_roll_pitch_yaw_thrust_st_(2);
  command_roll_pitch_yawrate_thrust_msg.thrust.z = command_roll_pitch_yaw_thrust_st_(3)*stnl_controller.getMass();
    
  command_roll_pitch_yawrate_thrust_pub_.publish(command_roll_pitch_yawrate_thrust_msg); 

  //std::cout << ltnl_controller.state_.rows() << " " << ltnl_controller.state_.cols() << " " << 
  //             stnl_controller.state_.rows() << " " << stnl_controller.state_.cols() << "\n";

  //std::cout << command_roll_pitch_yaw_thrust_st_.transpose() << "\n";
  
  return;
}



static void error_callback(int error, const char* description)
{
  fprintf(stderr, "Error %d: %s\n", error, description);
}

int main(int argc, char** argv)
{

  if(argc < 4) {
        std::cerr << FRED("Other Params Expected!") << " node_name <params_short_term_file> <params_long_term_file> <gui_params>" << "\n";
        std::exit(1);
  }

  std::string yaml_short_filename = argv[1];
  std::string yaml_long_filename = argv[2];
  std::string gui_filename = argv[3];

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
  //ros::NodeHandle nh("~");

  IBVSNode IBVS_node(nh, yaml_short_filename, gui_filename, gui_filename);
  //MavGUI gnomic_gui(nh, gui_filename);
  bool show_gnomic_GUI = true;

  // Main loop
  while (!glfwWindowShouldClose(window)) {
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

  ros::spin();

  return 0;
}
